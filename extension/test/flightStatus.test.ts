import assert from "node:assert/strict";
import { describe, it } from "node:test";

import { Latest, applyMessages, summarize, trackOutage } from "../src/flightStatus";

const NOW = 1_000_000;

function latestFrom(events: { topic: string; message: unknown }[], receivedAt = NOW): Latest {
  return applyMessages({}, events, receivedAt);
}

const status = (fields: Record<string, unknown> = {}) => ({
  topic: "/fmu/out/vehicle_status_v1",
  message: {
    arming_state: 2,
    nav_state: 3,
    failsafe: false,
    gcs_connection_lost: false,
    pre_flight_checks_pass: true,
    ...fields,
  },
});

describe("summarize", () => {
  it("shows dashes and waits for data before any message", () => {
    const view = summarize({}, NOW);
    assert.equal(view.mode.value, "—");
    assert.equal(view.battery.value, "—");
    assert.deepEqual(view.link, { value: "waiting for data", level: "warn" });
  });

  it("names modes, arming and VTOL state like QGC", () => {
    const view = summarize(
      latestFrom([
        status(),
        { topic: "/fmu/out/vtol_vehicle_status", message: { vehicle_vtol_state: 4 } },
      ]),
      NOW,
    );
    assert.equal(view.armed.value, "ARMED");
    assert.equal(view.mode.value, "Mission");
    assert.equal(view.vtol.value, "Fixed-wing");
    assert.equal(view.control.value, "PX4");
    assert.equal(view.failsafe.value, "none");
  });

  it("accepts the unversioned topic names of other PX4 versions", () => {
    const view = summarize(
      latestFrom([{ ...status({ nav_state: 14 }), topic: "/fmu/out/vehicle_status" }]),
      NOW,
    );
    assert.equal(view.mode.value, "Offboard");
    assert.equal(view.control.value, "Jetson (offboard)");
  });

  it("flags Return mode, VTOL transitions and failsafes", () => {
    const view = summarize(
      latestFrom([
        status({ nav_state: 5, failsafe: true }),
        { topic: "/fmu/out/vtol_vehicle_status", message: { vehicle_vtol_state: 1 } },
      ]),
      NOW,
    );
    assert.deepEqual(view.mode, { value: "Return", level: "alert" });
    assert.deepEqual(view.vtol, { value: "Transition → FW", level: "warn" });
    assert.deepEqual(view.failsafe, { value: "FAILSAFE ACTIVE", level: "alert" });
  });

  it("falls back to the PX4 name for modes without a QGC name", () => {
    assert.equal(summarize(latestFrom([status({ nav_state: 23 })]), NOW).mode.value, "EXTERNAL1");
    assert.equal(
      summarize(latestFrom([status({ nav_state: 99 })]), NOW).mode.value,
      "unknown (99)",
    );
  });

  it("computes altitude above home, speeds and climb from NED", () => {
    const view = summarize(
      latestFrom([
        {
          topic: "/fmu/out/vehicle_local_position_v1",
          message: {
            z: -51.5,
            vx: 3,
            vy: 4,
            vz: -2,
            z_valid: true,
            v_xy_valid: true,
            v_z_valid: true,
          },
        },
        { topic: "/fmu/out/home_position_v1", message: { z: -1.5, valid_lpos: true } },
        { topic: "/fmu/out/airspeed_validated_v1", message: { true_airspeed_m_s: 21.34 } },
      ]),
      NOW,
    );
    assert.deepEqual(view.altitude, { label: "Alt (home)", value: "50.0 m", level: "ok" });
    assert.equal(view.groundSpeed.value, "5.0 m/s");
    assert.equal(view.climb.value, "2.0 m/s");
    assert.equal(view.airspeed.value, "21.3 m/s");
  });

  it("hides values PX4 marks invalid", () => {
    const view = summarize(
      latestFrom([
        {
          topic: "/fmu/out/vehicle_local_position_v1",
          message: {
            z: -10,
            vx: 1,
            vy: 1,
            vz: 0,
            z_valid: true,
            v_xy_valid: false,
            v_z_valid: false,
          },
        },
        { topic: "/fmu/out/airspeed_validated_v1", message: { true_airspeed_m_s: NaN } },
      ]),
      NOW,
    );
    assert.deepEqual(view.altitude, { label: "Alt (origin)", value: "10.0 m", level: "warn" });
    assert.equal(view.groundSpeed.value, "—");
    assert.equal(view.airspeed.value, "—");
  });

  it("shows battery level and warnings", () => {
    const battery = (remaining: number, warning: number) =>
      summarize(
        latestFrom([
          {
            topic: "/fmu/out/battery_status_v1",
            message: { connected: true, remaining, voltage_v: 22.84, warning },
          },
        ]),
        NOW,
      ).battery;
    assert.deepEqual(battery(0.71, 0), { value: "71 %  (22.8 V)", level: "ok" });
    assert.equal(battery(0.15, 0).level, "warn");
    assert.deepEqual(battery(0.1, 2), { value: "10 %  (22.8 V)  CRITICAL", level: "alert" });
  });

  it("prefers the Jetson's mission progress and counts like QGC", () => {
    const progress = (current_index: number) => ({
      topic: "/vtol/mission_progress",
      message: { current_index, total_items: 18, px4_total_items: 21 },
    });
    const result = {
      topic: "/fmu/out/mission_result",
      message: { seq_current: 0, seq_total: 21, valid: true, finished: false },
    };

    assert.equal(
      summarize(latestFrom([result, progress(-1)]), NOW).mission.value,
      "not started (18 items)",
    );
    const view = summarize(latestFrom([result, progress(3)]), NOW);
    assert.equal(view.mission.value, "WP 4 / 21");
    assert.equal(view.mission.fraction, 4 / 21);
    assert.equal(
      summarize(latestFrom([progress(19)]), NOW).mission.value,
      "WP 20 / 21  (PX4 landing items)",
    );
  });

  it("uses PX4's mission result without the Jetson", () => {
    const result = (state: "flying" | "finished") => ({
      topic: "/fmu/out/mission_result",
      message: { seq_current: 2, seq_total: 9, valid: true, finished: state === "finished" },
    });
    assert.equal(summarize(latestFrom([result("flying")]), NOW).mission.value, "WP 3 / 9");
    assert.equal(
      summarize(latestFrom([result("finished")]), NOW).mission.value,
      "finished (9 items)",
    );
  });

  it("turns the link red when data stops", () => {
    const latest = latestFrom([status()], NOW);
    assert.deepEqual(summarize(latest, NOW + 1500).link, { value: "live", level: "ok" });
    assert.deepEqual(summarize(latest, NOW + 5000).link, {
      value: "no data for 5 s, check QGC",
      level: "alert",
    });
  });

  it("flags the view as lost while data is stopped", () => {
    const latest = latestFrom([status()], NOW);
    assert.equal(summarize({}, NOW).lost, false);
    assert.equal(summarize(latest, NOW + 1500).lost, false);
    assert.equal(summarize(latest, NOW + 5000).lost, true);
  });

  it("remembers the last outage after data comes back", () => {
    let latest = latestFrom([status()], NOW);
    let outage = trackOutage(undefined, latest, NOW + 1000);
    assert.equal(outage, undefined);
    outage = trackOutage(outage, latest, NOW + 3000);
    assert.deepEqual(outage, { start: NOW });
    outage = trackOutage(outage, latest, NOW + 9000);
    assert.deepEqual(outage, { start: NOW });

    latest = applyMessages(latest, [status()], NOW + 12000);
    outage = trackOutage(outage, latest, NOW + 12000);
    assert.deepEqual(outage, { start: NOW, end: NOW + 12000 });
    const link = summarize(latest, NOW + 13000, outage).link;
    assert.equal(link.level, "ok");
    assert.match(link.value, /^live · last lost \S+ for 12 s$/);

    // a later outage replaces it
    outage = trackOutage(outage, latest, NOW + 15000);
    assert.deepEqual(outage, { start: NOW + 12000 });
  });

  it("ignores topics it does not know", () => {
    assert.deepEqual(latestFrom([{ topic: "/something/else", message: {} }]), {});
  });
});
