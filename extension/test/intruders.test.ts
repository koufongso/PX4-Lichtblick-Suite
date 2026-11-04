import assert from "node:assert/strict";
import { describe, it } from "node:test";

import {
  COOPERATIVE_COLOR,
  IntruderState,
  NON_COOPERATIVE_COLOR,
  climbAngle,
  details,
  heading,
  intruderColor,
  isValid,
  shortLabel,
  typeName,
} from "../src/intruders";
import { northEastOf, offsetLatLon } from "../src/mapShapes";

const intruder = (fields: Partial<IntruderState> = {}): IntruderState => ({
  id: 2,
  type: 2,
  cooperative: true,
  latitude: 47.397971,
  longitude: 8.551469,
  altitude: 80,
  velocity_ned: new Float32Array([0, -18, 0]),
  ...fields,
});

const near = (actual: number, expected: number, tolerance = 1e-6) => {
  assert.ok(Math.abs(actual - expected) < tolerance, `${String(actual)} != ${String(expected)}`);
};

describe("intruders", () => {
  it("heading follows the ground track, clockwise from north", () => {
    near(heading(intruder({ velocity_ned: [10, 0, 0] })), 0);
    near(heading(intruder({ velocity_ned: [0, 10, 0] })), Math.PI / 2);
    near(heading(intruder({ velocity_ned: [0, -18, 0] })), -Math.PI / 2);
    near(heading(intruder({ velocity_ned: [-5, 0, 0] })), Math.PI);
  });

  it("keeps the previous heading when hovering", () => {
    near(heading(intruder({ velocity_ned: [0.1, 0.1, 0] }), 1.2), 1.2);
    near(heading(intruder({ velocity_ned: [Number.NaN, Number.NaN, 0] }), 1.2), 1.2);
  });

  it("climb angle is positive climbing (NED down negative)", () => {
    near(climbAngle(intruder({ velocity_ned: [10, 0, -10] })), Math.PI / 4);
    near(climbAngle(intruder({ velocity_ned: [10, 0, 10] })), -Math.PI / 4);
    near(climbAngle(intruder({ velocity_ned: [0, 0, 0.1] })), 0);
  });

  it("colours cooperative and non-cooperative intruders differently", () => {
    assert.equal(intruderColor(intruder({ cooperative: true })), COOPERATIVE_COLOR);
    assert.equal(intruderColor(intruder({ cooperative: false })), NON_COOPERATIVE_COLOR);
  });

  it("names the types", () => {
    assert.equal(typeName(1), "multirotor");
    assert.equal(typeName(2), "fixed-wing");
    assert.equal(typeName(0), "unknown");
  });

  it("labels the height relative to the vehicle when known, else AMSL", () => {
    assert.equal(shortLabel(intruder(), undefined), "#2 80 m AMSL");
    assert.equal(shortLabel(intruder(), 50), "#2 +30 m");
    assert.equal(shortLabel(intruder(), 100), "#2 −20 m");
    assert.deepEqual(details(intruder(), 50), [
      "Intruder #2: fixed-wing, cooperative",
      "80 m AMSL (+30 m to vehicle)",
      "18.0 m/s, climb 0.0 m/s",
    ]);
  });

  it("rejects positions that are not numbers", () => {
    assert.ok(isValid(intruder()));
    assert.ok(!isValid(intruder({ latitude: Number.NaN })));
  });

  it("northEastOf inverts offsetLatLon", () => {
    const [lon, lat] = offsetLatLon(47.397971, 8.546164, 200, -400);
    const [north, east] = northEastOf(47.397971, 8.546164, lat, lon);
    near(north, 200, 1e-6);
    near(east, -400, 1e-3);
  });
});
