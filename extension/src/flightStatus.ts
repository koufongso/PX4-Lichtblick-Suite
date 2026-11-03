// Turns the latest PX4 / vtol_node messages into what the Flight Status panel shows.
// Kept free of React and Lichtblick so it can be unit tested.

import {
  ARMING_STATE,
  BATTERY_WARNING,
  NAVIGATION_STATE,
  VEHICLE_VTOL_STATE,
} from "./generated/px4Enums";

// PX4 1.17 versions some topic names (e.g. vehicle_status_v1), subscribe to both so other PX4 versions work too
export const TOPICS = {
  vehicleStatus: ["/fmu/out/vehicle_status_v1", "/fmu/out/vehicle_status"],
  vtolStatus: ["/fmu/out/vtol_vehicle_status", "/fmu/out/vtol_vehicle_status_v1"],
  // /viz/*: throttled relay on the Jetson (viz:=true), the raw topic in recorded bags
  localPosition: [
    "/viz/vehicle_local_position_v1",
    "/fmu/out/vehicle_local_position_v1",
    "/fmu/out/vehicle_local_position",
  ],
  homePosition: ["/fmu/out/home_position_v1", "/fmu/out/home_position"],
  airspeed: ["/fmu/out/airspeed_validated_v1", "/fmu/out/airspeed_validated"],
  battery: ["/fmu/out/battery_status_v1", "/fmu/out/battery_status"],
  landDetected: ["/fmu/out/vehicle_land_detected"],
  missionResult: ["/fmu/out/mission_result"],
  missionProgress: ["/vtol/mission_progress"],
} as const;

export type Source = keyof typeof TOPICS;
export const ALL_TOPICS: string[] = Object.values(TOPICS).flat();

const SOURCE_OF_TOPIC = new Map<string, Source>(
  (Object.entries(TOPICS) as [Source, readonly string[]][]).flatMap(([source, topics]) =>
    topics.map((topic) => [topic, source] as [string, Source]),
  ),
);

// only the fields the panel uses
type VehicleStatus = {
  arming_state: number;
  nav_state: number;
  failsafe: boolean;
  gcs_connection_lost: boolean;
  pre_flight_checks_pass: boolean;
};
type VtolStatus = { vehicle_vtol_state: number };
type LocalPosition = {
  z: number;
  vx: number;
  vy: number;
  vz: number;
  z_valid: boolean;
  v_xy_valid: boolean;
  v_z_valid: boolean;
};
type HomePosition = { z: number; valid_lpos: boolean };
type Airspeed = { true_airspeed_m_s: number };
type Battery = { connected: boolean; remaining: number; voltage_v: number; warning: number };
type LandDetected = { landed: boolean };
type MissionResult = { seq_current: number; seq_total: number; valid: boolean; finished: boolean };
type MissionProgress = { current_index: number; total_items: number; px4_total_items: number };

type Messages = {
  vehicleStatus: VehicleStatus;
  vtolStatus: VtolStatus;
  localPosition: LocalPosition;
  homePosition: HomePosition;
  airspeed: Airspeed;
  battery: Battery;
  landDetected: LandDetected;
  missionResult: MissionResult;
  missionProgress: MissionProgress;
};

// latest message per source, with the wall-clock time it arrived [ms]
export type Latest = { [S in Source]?: { message: Messages[S]; receivedAt: number } };

export function applyMessages(
  latest: Latest,
  events: readonly { topic: string; message: unknown }[],
  receivedAt: number,
): Latest {
  const next: Record<string, unknown> = { ...latest };
  for (const event of events) {
    const source = SOURCE_OF_TOPIC.get(event.topic);
    if (source) {
      next[source] = { message: event.message, receivedAt };
    }
  }
  return next;
}

export type Level = "ok" | "warn" | "alert" | "none";
export type Field = { value: string; level: Level };

export type FlightStatusView = {
  armed: Field;
  mode: Field;
  vtol: Field;
  control: Field;
  altitude: Field & { label: string };
  airspeed: Field;
  groundSpeed: Field;
  climb: Field;
  battery: Field;
  mission: Field & { fraction?: number };
  failsafe: Field;
  link: Field;
};

// QGC style flight mode names, anything else falls back to the PX4 constant name
const MODE_NAMES: Record<string, string> = {
  MANUAL: "Manual",
  ALTCTL: "Altitude",
  POSCTL: "Position",
  AUTO_MISSION: "Mission",
  AUTO_LOITER: "Hold",
  AUTO_RTL: "Return",
  ACRO: "Acro",
  DESCEND: "Descend",
  TERMINATION: "Termination",
  OFFBOARD: "Offboard",
  STAB: "Stabilized",
  AUTO_TAKEOFF: "Takeoff",
  AUTO_LAND: "Land",
  AUTO_FOLLOW_TARGET: "Follow Me",
  AUTO_PRECLAND: "Precision Land",
  ORBIT: "Orbit",
  AUTO_VTOL_TAKEOFF: "VTOL Takeoff",
};
const ALERT_MODES = new Set(["AUTO_RTL", "AUTO_LAND", "DESCEND", "TERMINATION"]);

const VTOL_NAMES: Record<string, string> = {
  MC: "Multicopter",
  FW: "Fixed-wing",
  TRANSITION_TO_FW: "Transition → FW",
  TRANSITION_TO_MC: "Transition → MC",
};

const STALE_AFTER_MS = 2000;
const NONE: Field = { value: "—", level: "none" };

function enumName(table: Readonly<Record<number, string>>, value: number): string {
  return table[value] ?? `unknown (${String(value)})`;
}

function meters(value: number, digits = 1): string {
  return `${value.toFixed(digits)} m`;
}

function speed(value: number): string {
  return `${value.toFixed(1)} m/s`;
}

export function summarize(latest: Latest, now: number): FlightStatusView {
  const status = latest.vehicleStatus?.message;
  const vtol = latest.vtolStatus?.message;
  const lpos = latest.localPosition?.message;
  const home = latest.homePosition?.message;
  const airspeed = latest.airspeed?.message;
  const battery = latest.battery?.message;
  const landed = latest.landDetected?.message;

  const armedName = status ? enumName(ARMING_STATE, status.arming_state) : undefined;
  const modeName = status ? enumName(NAVIGATION_STATE, status.nav_state) : undefined;
  const vtolName = vtol ? enumName(VEHICLE_VTOL_STATE, vtol.vehicle_vtol_state) : undefined;

  // altitude above home when PX4 has one, otherwise above the EKF origin
  let altitude: FlightStatusView["altitude"] = { label: "Altitude", ...NONE };
  if (lpos?.z_valid === true) {
    altitude =
      home?.valid_lpos === true
        ? { label: "Alt (home)", value: meters(home.z - lpos.z), level: "ok" }
        : { label: "Alt (origin)", value: meters(-lpos.z), level: "warn" };
  }

  return {
    armed: armedName
      ? {
          value: armedName === "ARMED" ? "ARMED" : "DISARMED",
          level: armedName === "ARMED" ? "warn" : "ok",
        }
      : NONE,
    mode: modeName
      ? {
          value: MODE_NAMES[modeName] ?? modeName,
          level: ALERT_MODES.has(modeName) ? "alert" : "ok",
        }
      : NONE,
    vtol: vtolName
      ? {
          value: VTOL_NAMES[vtolName] ?? vtolName,
          level: vtolName.startsWith("TRANSITION") ? "warn" : "ok",
        }
      : NONE,
    control: modeName
      ? modeName === "OFFBOARD"
        ? { value: "Jetson (offboard)", level: "ok" }
        : { value: "PX4", level: "ok" }
      : NONE,
    altitude,
    airspeed:
      airspeed && Number.isFinite(airspeed.true_airspeed_m_s)
        ? { value: speed(airspeed.true_airspeed_m_s), level: "ok" }
        : NONE,
    groundSpeed:
      lpos?.v_xy_valid === true
        ? { value: speed(Math.hypot(lpos.vx, lpos.vy)), level: "ok" }
        : NONE,
    climb: lpos?.v_z_valid === true ? { value: speed(-lpos.vz), level: "ok" } : NONE,
    battery: summarizeBattery(battery),
    mission: summarizeMission(latest),
    failsafe: status
      ? status.failsafe
        ? { value: "FAILSAFE ACTIVE", level: "alert" }
        : status.gcs_connection_lost
          ? { value: "GCS link lost", level: "alert" }
          : { value: landed?.landed === true ? "none (landed)" : "none", level: "ok" }
      : NONE,
    link: summarizeLink(latest, now),
  };
}

function summarizeBattery(battery: Battery | undefined): Field {
  if (battery?.connected !== true) {
    return NONE;
  }
  const warning = enumName(BATTERY_WARNING, battery.warning);
  const percent =
    Number.isFinite(battery.remaining) && battery.remaining >= 0
      ? `${String(Math.round(battery.remaining * 100))} %`
      : "?";
  const value =
    `${percent}  (${battery.voltage_v.toFixed(1)} V)` + (warning === "NONE" ? "" : `  ${warning}`);
  const level: Level =
    battery.warning >= 2
      ? "alert"
      : battery.warning === 1 || battery.remaining < 0.2
        ? "warn"
        : "ok";
  return { value, level };
}

// prefer the Jetson's progress (vtol_node), it also covers offboard, otherwise PX4's mission result.
// shown 1-based, like the item numbers in QGC
function summarizeMission(latest: Latest): FlightStatusView["mission"] {
  const progress = latest.missionProgress?.message;
  if (progress && progress.total_items > 0) {
    if (progress.current_index < 0) {
      return {
        value: `not started (${String(progress.total_items)} items)`,
        level: "ok",
        fraction: 0,
      };
    }
    const current = progress.current_index + 1;
    // past the Jetson's items PX4 flies the rest (e.g. the VTOL landing pattern)
    const total = Math.max(progress.total_items, progress.px4_total_items);
    const byPx4 = progress.current_index >= progress.total_items ? "  (PX4 landing items)" : "";
    return {
      value: `WP ${String(current)} / ${String(total)}${byPx4}`,
      level: "ok",
      fraction: Math.min(current / total, 1),
    };
  }

  const result = latest.missionResult?.message;
  if (result && result.valid && result.seq_total > 0) {
    if (result.finished) {
      return { value: `finished (${String(result.seq_total)} items)`, level: "ok", fraction: 1 };
    }
    const current = result.seq_current + 1;
    return {
      value: `WP ${String(current)} / ${String(result.seq_total)}`,
      level: "ok",
      fraction: Math.min(current / result.seq_total, 1),
    };
  }
  return NONE;
}

function summarizeLink(latest: Latest, now: number): Field {
  const times = Object.values(latest).map((entry) => entry.receivedAt);
  if (times.length === 0) {
    return { value: "waiting for data", level: "warn" };
  }
  const ageMs = now - Math.max(...times);
  return ageMs > STALE_AFTER_MS
    ? { value: `no data for ${(ageMs / 1000).toFixed(0)} s, check QGC`, level: "alert" }
    : { value: "live", level: "ok" };
}
