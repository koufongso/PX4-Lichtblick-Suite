// Intruders from /intruder/states (vtol_interfaces/IntruderArray): the display logic shared by the Flight Map
// panel and the 3D converter. No Leaflet or Lichtblick here, unit tested in test/intruders.test.ts

export const INTRUDER_TOPIC = "/intruder/states";
export const INTRUDER_SCHEMA = "vtol_interfaces/msg/IntruderArray";

// IntruderState.type
export const TYPE_MULTIROTOR = 1;
export const TYPE_FIXED_WING = 2;

export type IntruderState = {
  id: number;
  type: number;
  cooperative: boolean; // broadcasts its own state (ADS-B, Remote ID)
  latitude: number; // [deg]
  longitude: number; // [deg]
  altitude: number; // [m] AMSL
  velocity_ned: ArrayLike<number>; // [m/s]
};
export type IntruderArray = { intruders: IntruderState[] };

// red: only known from detection, magenta: cooperative
export const NON_COOPERATIVE_COLOR = "#ff3b30";
export const COOPERATIVE_COLOR = "#e040fb";

export const PREDICTION_S = 10; // the velocity is drawn as where the intruder will be in this many seconds
const MIN_HEADING_SPEED = 0.5; // [m/s] below this the heading is kept, it is noise when hovering

export function intruderColor(intruder: IntruderState): string {
  return intruder.cooperative ? COOPERATIVE_COLOR : NON_COOPERATIVE_COLOR;
}

export function typeName(type: number): string {
  return type === TYPE_MULTIROTOR
    ? "multirotor"
    : type === TYPE_FIXED_WING
      ? "fixed-wing"
      : "unknown";
}

export function isValid(intruder: IntruderState): boolean {
  return (
    Number.isFinite(intruder.latitude) &&
    Number.isFinite(intruder.longitude) &&
    Number.isFinite(intruder.altitude)
  );
}

// [north, east, down], zeros for missing or invalid values
export function velocityNed(intruder: IntruderState): [number, number, number] {
  const v = intruder.velocity_ned;
  const value = (i: number) => {
    const x = v[i];
    return x != undefined && Number.isFinite(x) ? x : 0;
  };
  return [value(0), value(1), value(2)];
}

export function groundSpeed(intruder: IntruderState): number {
  const [north, east] = velocityNed(intruder);
  return Math.hypot(north, east);
}

// [rad] from north, clockwise, along the ground track; `previous` when (almost) hovering
export function heading(intruder: IntruderState, previous = 0): number {
  const [north, east] = velocityNed(intruder);
  return Math.hypot(north, east) < MIN_HEADING_SPEED ? previous : Math.atan2(east, north);
}

// [rad] flight path angle, positive climbing
export function climbAngle(intruder: IntruderState): number {
  const [north, east, down] = velocityNed(intruder);
  const horizontal = Math.hypot(north, east);
  return horizontal < MIN_HEADING_SPEED && Math.abs(down) < MIN_HEADING_SPEED
    ? 0
    : Math.atan2(-down, horizontal);
}

const signed = (x: number) => `${x >= 0 ? "+" : "−"}${Math.abs(x).toFixed(0)}`;

// short label next to the marker: the height relative to the vehicle when known, else AMSL
export function shortLabel(intruder: IntruderState, vehicleAltitude: number | undefined): string {
  return vehicleAltitude == undefined
    ? `#${String(intruder.id)} ${intruder.altitude.toFixed(0)} m AMSL`
    : `#${String(intruder.id)} ${signed(intruder.altitude - vehicleAltitude)} m`;
}

// tooltip lines
export function details(intruder: IntruderState, vehicleAltitude: number | undefined): string[] {
  const [, , down] = velocityNed(intruder);
  const altitude =
    `${intruder.altitude.toFixed(0)} m AMSL` +
    (vehicleAltitude == undefined
      ? ""
      : ` (${signed(intruder.altitude - vehicleAltitude)} m to vehicle)`);
  return [
    `Intruder #${String(intruder.id)}: ${typeName(intruder.type)}, ${intruder.cooperative ? "cooperative" : "non-cooperative"}`,
    altitude,
    `${groundSpeed(intruder).toFixed(1)} m/s, climb ${(-down).toFixed(1)} m/s`,
  ];
}
