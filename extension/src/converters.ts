// Message converters: PX4 / vtol_node messages -> Foxglove schemas the built-in Map and 3D panels understand.
// They run in Lichtblick on the PC (live and for recorded bags), so the Jetson only publishes the raw topics.

import { ExtensionContext, Immutable, MessageEvent, Time } from "@lichtblick/suite";

import {
  BODY_FRAME,
  Quaternion,
  Vector3,
  WORLD_FRAME,
  nedToEnu,
  px4QuaternionToEnu,
  rotate,
} from "./frames";
import {
  INTRUDER_SCHEMA,
  IntruderArray,
  IntruderState,
  PREDICTION_S,
  TYPE_MULTIROTOR,
  climbAngle,
  heading,
  intruderColor,
  isValid,
  typeName,
  velocityNed,
} from "./intruders";
import {
  Feature,
  distanceM,
  line,
  northEastOf,
  numberedDot,
  offsetLatLon,
  vehicleArrow,
} from "./mapShapes";

// only the fields the converters use
type VehicleOdometry = { pose_frame: number; position: ArrayLike<number>; q: ArrayLike<number> };
type VehicleGlobalPosition = {
  lat: number;
  lon: number;
  alt: number;
  eph: number;
  epv: number;
  lat_lon_valid: boolean;
};
type VehicleLocalPosition = {
  x: number;
  y: number;
  z: number;
  xy_valid: boolean;
  z_valid: boolean;
  xy_global: boolean; // ref_lat / ref_lon valid
  ref_lat: number;
  ref_lon: number;
  heading: number; // [rad] from north, clockwise
};
type MissionItem = {
  seq: number;
  command: number;
  frame: number;
  x: number;
  y: number;
  position_ned_valid: boolean;
  position_ned: ArrayLike<number>;
};
type MissionPlan = {
  ref_valid: boolean; // EKF origin
  ref_lat: number;
  ref_lon: number;
  ref_alt: number;
  home_valid: boolean;
  home_z: number;
  items: MissionItem[];
};

const POSE_FRAME_NED = 1;
const FRAME_LOCAL_NED = 1; // vtol_interfaces MissionItem frames: 0 global, 1 local NED, 3 global relative alt
const CMD_TAKEOFF = new Set([22, 84]); // NAV_TAKEOFF, NAV_VTOL_TAKEOFF
const CMD_LAND = new Set([21, 85]); // NAV_LAND, NAV_VTOL_LAND

// 3D scene, sizes in metres, chosen to stay visible when the whole mission (~1 km) is in view
const VEHICLE_SPAN_M = 30; // wingspan of the airplane outline of the vehicle
const HISTORY_SPAN_M = 12; // wingspan of the outlines left along the trajectory
const HISTORY_SPACING_M = 30; // one outline every this far along the trajectory
const TRAIL_SPACING_M = 1.5; // add a trajectory point after moving this far
const TRAIL_CHUNK_POINTS = 500; // the trajectory is sent in chunks, only the newest one changes
const WAYPOINT_SIZE_M = 8;
const GRID_CELL_M = 50;
const GRID_MAJOR_M = 100; // thicker, darker line every this far, so the scale reads when zoomed out
const GRID_HALF_SIZE_M = 1500; // the grid spans 3 km x 3 km around its centre
const GRID_RECENTRE_M = 1000; // move the grid when the vehicle is this far from its centre

// Map panel, sizes in metres
const MAP_VEHICLE_LENGTH_M = 40;
const MAP_WAYPOINT_RADIUS_M = 12;
const MAP_TRAIL_SPACING_M = 3;
const MAP_TRAIL_MAX_POINTS = 5000;

// mid tones, readable on the 3D panel's light and dark background (white or light grey vanish on the light one)
type Color = { r: number; g: number; b: number; a: number };
const TRAIL_COLOR: Color = { r: 1, g: 0.6, b: 0, a: 1 };
const PATH_COLOR: Color = { r: 0, g: 0.8, b: 1, a: 1 };
const WAYPOINT_COLOR: Color = { r: 0, g: 0.8, b: 1, a: 0.8 };
const TAKEOFF_COLOR: Color = { r: 0.3, g: 0.9, b: 0.3, a: 0.9 };
const LAND_COLOR: Color = { r: 1, g: 0.4, b: 0.2, a: 0.9 };
const TEXT_COLOR: Color = { r: 1, g: 1, b: 1, a: 1 };
const VEHICLE_COLOR: Color = { r: 1, g: 0.9, b: 0.1, a: 1 };
const DROP_LINE_COLOR: Color = { r: 1, g: 1, b: 1, a: 0.5 };
const SHADOW_COLOR: Color = { r: 0.6, g: 0.6, b: 0.6, a: 0.5 };
const HISTORY_COLOR: Color = { r: 1, g: 1, b: 1, a: 0.8 };
const GRID_COLOR: Color = { r: 0.5, g: 0.5, b: 0.5, a: 0.4 };
const GRID_MAJOR_COLOR: Color = { r: 0, g: 0, b: 0, a: 0.35 };
const GRID_WIDTH = 1; // [px]
const GRID_MAJOR_WIDTH = 1.5; // [px]
const NORTH_COLOR: Color = { r: 1, g: 0.3, b: 0.3, a: 1 };

const INTRUDER_FIXED_WING_SPAN_M = 25;
const INTRUDER_ARROW_LENGTH_M = 40; // solid arrow along the intruder's pose, centred on it
const PREDICTION_DASHES = 8; // the prediction line is dashed like on the Flight Map, so it is not taken for the trail
const INTRUDER_MULTIROTOR_SPAN_M = 16;
const INTRUDER_TRAIL_SPACING_M = 3;
const INTRUDER_TRAIL_MAX_POINTS = 1000;

// ENU height of the ground (home), so heights can be drawn as drop lines down to it.
// Taken from the mission plan, which carries home; the EKF origin (0) until then
let groundZ = 0;

const IDENTITY_POSE = { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } };

function sceneEntity(timestamp: Time, id: string, primitives: Record<string, unknown[]>) {
  return {
    timestamp,
    frame_id: WORLD_FRAME,
    id,
    lifetime: { sec: 0, nsec: 0 }, // keep until replaced
    frame_locked: false,
    metadata: [],
    arrows: [],
    cubes: [],
    spheres: [],
    cylinders: [],
    lines: [],
    triangles: [],
    texts: [],
    models: [],
    ...primitives,
  };
}

function lineStrip(
  points: Vector3[],
  color: Color,
  thickness: number,
  type = 0,
  pose: { position: Vector3; orientation: Quaternion } = IDENTITY_POSE,
) {
  return {
    type, // 0 strip, 2 list (pairs of points)
    pose, // the points are relative to it
    thickness,
    scale_invariant: true,
    points,
    color,
    colors: [],
    indices: [],
  };
}

// vehicle_odometry -> foxglove.FrameTransform map -> base_link, so the 3D panel can follow the vehicle
function odometryToTransform(
  msg: VehicleOdometry,
  event: Immutable<MessageEvent<VehicleOdometry>>,
) {
  const pose = odometryToEnuPose(msg);
  if (!pose) {
    return undefined;
  }
  return {
    timestamp: event.receiveTime,
    parent_frame_id: WORLD_FRAME,
    child_frame_id: BODY_FRAME,
    translation: pose.position,
    rotation: pose.orientation,
  };
}

// airplane outline in FLU body coordinates for a 1 m wingspan, as a line list: fuselage, wings, tail plane, fin
const AIRPLANE: Vector3[] = [
  [0.55, 0, 0, -0.5, 0, 0],
  [0.05, -0.5, 0, 0.05, 0.5, 0],
  [-0.42, -0.18, 0, -0.42, 0.18, 0],
  [-0.48, 0, 0, -0.48, 0, 0.18],
].flatMap(([x1, y1, z1, x2, y2, z2]) => [
  { x: x1!, y: y1!, z: z1! },
  { x: x2!, y: y2!, z: z2! },
]);

function airplane(
  pose: { position: Vector3; orientation: Quaternion },
  span: number,
  color: Color,
  thickness: number,
) {
  const points = AIRPLANE.map((p) => ({ x: p.x * span, y: p.y * span, z: p.z * span }));
  return lineStrip(points, color, thickness, 2, pose);
}

// grid lines at ground height around a centre, as a line list, with a red line and "N" pointing north
// the centre is snapped to GRID_CELL_M, so the major lines fall on multiples of GRID_MAJOR_M from the origin
function groundGrid(timestamp: Time, centre: { x: number; y: number }) {
  const minor: Vector3[] = [];
  const major: Vector3[] = [];
  const isMajor = (v: number) => Math.abs(Math.round(v) % GRID_MAJOR_M) === 0;
  const [minX, maxX] = [centre.x - GRID_HALF_SIZE_M, centre.x + GRID_HALF_SIZE_M];
  const [minY, maxY] = [centre.y - GRID_HALF_SIZE_M, centre.y + GRID_HALF_SIZE_M];
  for (let x = minX; x <= maxX; x += GRID_CELL_M) {
    (isMajor(x) ? major : minor).push({ x, y: minY, z: groundZ }, { x, y: maxY, z: groundZ });
  }
  for (let y = minY; y <= maxY; y += GRID_CELL_M) {
    (isMajor(y) ? major : minor).push({ x: minX, y, z: groundZ }, { x: maxX, y, z: groundZ });
  }
  const north = { x: centre.x, y: maxY + 2 * GRID_CELL_M, z: groundZ };
  return sceneEntity(timestamp, "ground_grid", {
    lines: [
      lineStrip(minor, GRID_COLOR, GRID_WIDTH, 2),
      lineStrip(major, GRID_MAJOR_COLOR, GRID_MAJOR_WIDTH, 2),
      lineStrip([{ x: centre.x, y: maxY, z: groundZ }, north], NORTH_COLOR, 3),
    ],
    texts: [
      {
        pose: { ...IDENTITY_POSE, position: north },
        billboard: true,
        font_size: 24,
        scale_invariant: true,
        color: NORTH_COLOR,
        text: "N",
      },
    ],
  });
}

// vehicle_odometry -> foxglove.SceneUpdate: the whole 3D flight scene, from the one topic that keeps arriving
// - the vehicle as an airplane outline (shows roll and pitch), a drop line to the ground and its height
// - the flown trajectory, its shadow on the ground, and a smaller outline with a drop line every 30 m
// - a ground grid around the vehicle, so the space has a scale
// Restarts when time jumps back (seeking in a bag)
let trajectory: Vector3[] = [];
let lastHistory: Vector3 | undefined;
let historyCount = 0;
let gridCentre: { x: number; y: number } | undefined;
let gridZ: number | undefined;
let sceneLastTime: Time | undefined;

const distance = (a: Vector3, b: Vector3) => Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
const onGround = (p: Vector3) => ({ ...p, z: groundZ });

function odometryToScene(msg: VehicleOdometry, event: Immutable<MessageEvent<VehicleOdometry>>) {
  const time = event.receiveTime;
  const deletions: unknown[] = [];
  const entities: unknown[] = [];
  if (isBefore(time, sceneLastTime)) {
    trajectory = [];
    lastHistory = undefined;
    historyCount = 0;
    gridCentre = undefined;
    deletions.push({ timestamp: time, type: 1, id: "" }); // 1: delete all entities of this topic
  }
  sceneLastTime = time;

  const pose = odometryToEnuPose(msg);
  if (!pose) {
    return deletions.length > 0 ? { deletions, entities } : undefined;
  }
  const { position } = pose;

  entities.push(
    sceneEntity(time, "vehicle", {
      lines: [
        airplane(pose, VEHICLE_SPAN_M, VEHICLE_COLOR, 3),
        lineStrip([position, onGround(position)], DROP_LINE_COLOR, 1),
      ],
      texts: [
        {
          pose: { ...IDENTITY_POSE, position: { ...position, z: position.z + VEHICLE_SPAN_M / 2 } },
          billboard: true,
          font_size: 14,
          scale_invariant: true,
          color: VEHICLE_COLOR,
          text: `${(position.z - groundZ).toFixed(0)} m`,
        },
      ],
    }),
  );

  const last = trajectory[trajectory.length - 1];
  if (!last || distance(last, position) >= TRAIL_SPACING_M) {
    trajectory.push(position);
    // the chunk shares its first point with the previous one, so the line has no gaps
    const chunk = Math.floor((trajectory.length - 1) / TRAIL_CHUNK_POINTS);
    const points = trajectory.slice(Math.max(0, chunk * TRAIL_CHUNK_POINTS - 1));
    entities.push(
      sceneEntity(time, `trajectory_${String(chunk)}`, {
        lines: [
          lineStrip(points, TRAIL_COLOR, 2),
          lineStrip(points.map(onGround), SHADOW_COLOR, 1),
        ],
      }),
    );
  }

  if (!lastHistory || distance(lastHistory, position) >= HISTORY_SPACING_M) {
    lastHistory = position;
    historyCount += 1;
    entities.push(
      sceneEntity(time, `pose_${String(historyCount)}`, {
        lines: [
          airplane(pose, HISTORY_SPAN_M, HISTORY_COLOR, 1.5),
          lineStrip([position, onGround(position)], DROP_LINE_COLOR, 1),
        ],
      }),
    );
  }

  const offGrid =
    !gridCentre ||
    Math.max(Math.abs(position.x - gridCentre.x), Math.abs(position.y - gridCentre.y)) >
      GRID_RECENTRE_M;
  if (offGrid || gridZ !== groundZ) {
    if (offGrid) {
      const snap = (v: number) => Math.round(v / GRID_CELL_M) * GRID_CELL_M;
      gridCentre = { x: snap(position.x), y: snap(position.y) };
    }
    gridZ = groundZ;
    entities.push(groundGrid(time, gridCentre!));
  }

  return { deletions, entities };
}

function odometryToEnuPose(msg: VehicleOdometry) {
  const [north, east, down] = [msg.position[0], msg.position[1], msg.position[2]];
  if (
    msg.pose_frame !== POSE_FRAME_NED ||
    north == undefined ||
    east == undefined ||
    down == undefined ||
    !Number.isFinite(north)
  ) {
    return undefined;
  }
  const orientation = px4QuaternionToEnu(msg.q);
  return orientation ? { position: nedToEnu(north, east, down), orientation } : undefined;
}

// vehicle_global_position -> foxglove.LocationFix for the built-in Map panel, which needs a position topic
// to centre and follow the map
function globalPositionToLocationFix(
  msg: VehicleGlobalPosition,
  event: Immutable<MessageEvent<VehicleGlobalPosition>>,
) {
  if (!msg.lat_lon_valid) {
    return undefined;
  }
  const horizontal = msg.eph * msg.eph;
  return {
    timestamp: event.receiveTime,
    frame_id: "",
    latitude: msg.lat,
    longitude: msg.lon,
    altitude: msg.alt,
    position_covariance: [horizontal, 0, 0, 0, horizontal, 0, 0, 0, msg.epv * msg.epv],
    position_covariance_type: 2, // DIAGONAL_KNOWN
  };
}

function waypointColor(item: MissionItem): Color {
  return CMD_TAKEOFF.has(item.command)
    ? TAKEOFF_COLOR
    : CMD_LAND.has(item.command)
      ? LAND_COLOR
      : WAYPOINT_COLOR;
}

// QGC numbers the items from 1
const waypointName = (item: MissionItem) => `WP ${String(item.seq + 1)}`;

// /vtol/mission -> foxglove.SceneUpdate: waypoints, labels, the path between them and drop lines
// from the waypoints in the 3D panel. Also sets the ground height (home) of the whole scene
function missionToScene(msg: MissionPlan, event: Immutable<MessageEvent<MissionPlan>>) {
  if (msg.home_valid) {
    groundZ = -msg.home_z;
  }
  if (msg.ref_valid) {
    sceneOrigin = { lat: msg.ref_lat, lon: msg.ref_lon, alt: msg.ref_alt, fromMission: true };
  }
  const items = msg.items.filter((item) => item.position_ned_valid);
  const positions = items.map((item) =>
    nedToEnu(item.position_ned[0] ?? 0, item.position_ned[1] ?? 0, item.position_ned[2] ?? 0),
  );
  const spheres = items.map((item, i) => ({
    pose: { ...IDENTITY_POSE, position: positions[i] },
    size: { x: WAYPOINT_SIZE_M, y: WAYPOINT_SIZE_M, z: WAYPOINT_SIZE_M },
    color: waypointColor(item),
  }));
  const texts = items.map((item, i) => ({
    pose: { ...IDENTITY_POSE, position: { ...positions[i]!, z: positions[i]!.z + 10 } },
    billboard: true,
    font_size: 14,
    scale_invariant: true,
    color: TEXT_COLOR,
    text: waypointName(item),
  }));
  return {
    deletions: [],
    entities: [
      sceneEntity(event.receiveTime, "mission", {
        spheres,
        texts,
        lines: [
          lineStrip(positions, PATH_COLOR, 2),
          lineStrip(
            positions.flatMap((p) => [p, { ...p, z: groundZ }]),
            DROP_LINE_COLOR,
            1,
            2,
          ),
        ],
      }),
    ],
  };
}

// lat/lon/alt of the 3D scene's origin (the EKF origin), for the intruders, which arrive in WGS84.
// From /vtol/mission, the only topic that carries it. Without a mission (e.g. testing the intruders alone)
// the first intruder is used, labelled in the scene since the vehicle would then not line up with them
let sceneOrigin: { lat: number; lon: number; alt: number; fromMission: boolean } | undefined;

function toEnu(lat: number, lon: number, alt: number): Vector3 {
  const origin = sceneOrigin!;
  const [north, east] = northEastOf(origin.lat, origin.lon, lat, lon);
  return { x: east, y: north, z: alt - origin.alt };
}

// a dashed line from a to b as a line list (pairs of points): `count` dashes with equal gaps between them
function dashes(a: Vector3, b: Vector3, count: number): Vector3[] {
  const at = (t: number) => ({
    x: a.x + (b.x - a.x) * t,
    y: a.y + (b.y - a.y) * t,
    z: a.z + (b.z - a.z) * t,
  });
  const step = 1 / (2 * count - 1); // dash, gap, dash, ..., dash
  const points: Vector3[] = [];
  for (let i = 0; i < count; i++) {
    points.push(at(2 * i * step), at((2 * i + 1) * step));
  }
  return points;
}

// quadcopter outline in FLU for a 1 m span (rotor centre to rotor centre), as a line list: the X frame, four rotors
const QUADCOPTER: Vector3[] = (() => {
  const points: Vector3[] = [];
  const arm = 0.5 * Math.SQRT1_2;
  const rotors = [
    [arm, arm],
    [arm, -arm],
    [-arm, -arm],
    [-arm, arm],
  ] as const;
  points.push({ x: arm, y: arm, z: 0 }, { x: -arm, y: -arm, z: 0 });
  points.push({ x: arm, y: -arm, z: 0 }, { x: -arm, y: arm, z: 0 });
  for (const [cx, cy] of rotors) {
    for (let i = 0; i < 12; i++) {
      for (const k of [i, i + 1]) {
        const angle = (k / 12) * 2 * Math.PI;
        points.push({ x: cx + 0.2 * Math.cos(angle), y: cy + 0.2 * Math.sin(angle), z: 0 });
      }
    }
  }
  return points;
})();

// yaw about z, then pitch nose up (a negative rotation about FLU y)
function yawPitch(yaw: number, pitch: number): Quaternion {
  const [cy, sy, cp, sp] = [
    Math.cos(yaw / 2),
    Math.sin(yaw / 2),
    Math.cos(-pitch / 2),
    Math.sin(-pitch / 2),
  ];
  return { w: cy * cp, x: -sy * sp, y: cy * sp, z: sy * cp };
}

function hexColor(hexString: string, a = 1): Color {
  const value = parseInt(hexString.slice(1), 16);
  return {
    r: ((value >> 16) & 255) / 255,
    g: ((value >> 8) & 255) / 255,
    b: (value & 255) / 255,
    a,
  };
}

function ensureOrigin(intruders: IntruderState[]): void {
  const first = intruders[0];
  if (!sceneOrigin && first) {
    sceneOrigin = { lat: first.latitude, lon: first.longitude, alt: 0, fromMission: false };
  }
}

// FLU body -> ENU: nose along the ground track (heading from north clockwise, ENU yaw from east
// counter-clockwise), pitched by the flight path angle. Intruders report no attitude, so no roll
function intruderOrientation(intruder: IntruderState, headingRad: number): Quaternion {
  return yawPitch(Math.PI / 2 - headingRad, climbAngle(intruder));
}

export const intruderFrame = (id: number): string => `intruder_${String(id)}`;

// /intruder/states -> foxglove.FrameTransforms: a frame map -> intruder_<id> per intruder, like the vehicle's
// map -> base_link, so the 3D panel shows its axes and can follow it (display frame intruder_<id>).
// A frame stays at its last pose after the intruder is gone, transforms can not be deleted
let transformHeadings = new Map<number, number>();
let transformsLastTime: Time | undefined;

function intrudersToTransforms(msg: IntruderArray, event: Immutable<MessageEvent<IntruderArray>>) {
  const time = event.receiveTime;
  if (isBefore(time, transformsLastTime)) {
    transformHeadings = new Map();
  }
  transformsLastTime = time;

  const intruders = msg.intruders.filter(isValid);
  ensureOrigin(intruders);
  if (intruders.length === 0) {
    return undefined;
  }
  return {
    transforms: intruders.map((intruder) => {
      const headingRad = heading(intruder, transformHeadings.get(intruder.id) ?? 0);
      transformHeadings.set(intruder.id, headingRad);
      return {
        timestamp: time,
        parent_frame_id: WORLD_FRAME,
        child_frame_id: intruderFrame(intruder.id),
        translation: toEnu(intruder.latitude, intruder.longitude, intruder.altitude),
        rotation: intruderOrientation(intruder, headingRad),
      };
    }),
  };
}

type IntruderTrack = { heading: number; trail: { lat: number; lon: number; alt: number }[] };
let intruderTracks = new Map<number, IntruderTrack>();
let intrudersLastTime: Time | undefined;

// /intruder/states -> foxglove.SceneUpdate: per intruder its outline along the flight path (airplane or
// quadcopter), a drop line, its id and height, a line to where it will be in PREDICTION_S, and its trail.
// The whole topic is replaced by every message, so intruders that are gone disappear
// The vehicle's scene draws the ground grid. Without the vehicle (e.g. testing the intruders alone) the
// intruders draw it around themselves, so the 3D space has a scale. Once the vehicle's grid exists this one
// is dropped (each message replaces the whole topic)
let intruderGridCentre: { x: number; y: number } | undefined;

function intruderGrid(time: Time, intruders: IntruderState[]) {
  if (gridCentre != undefined || intruders.length === 0) {
    return undefined;
  }
  const positions = intruders.map((i) => toEnu(i.latitude, i.longitude, i.altitude));
  const mean = (values: number[]) => values.reduce((sum, v) => sum + v, 0) / values.length;
  const centre = { x: mean(positions.map((p) => p.x)), y: mean(positions.map((p) => p.y)) };
  if (
    !intruderGridCentre ||
    Math.max(Math.abs(centre.x - intruderGridCentre.x), Math.abs(centre.y - intruderGridCentre.y)) >
      GRID_RECENTRE_M
  ) {
    const snap = (v: number) => Math.round(v / GRID_CELL_M) * GRID_CELL_M;
    intruderGridCentre = { x: snap(centre.x), y: snap(centre.y) };
  }
  return groundGrid(time, intruderGridCentre);
}

function intrudersToScene(msg: IntruderArray, event: Immutable<MessageEvent<IntruderArray>>) {
  const time = event.receiveTime;
  if (isBefore(time, intrudersLastTime)) {
    intruderTracks = new Map();
    intruderGridCentre = undefined;
  }
  intrudersLastTime = time;

  const intruders = msg.intruders.filter(isValid);
  ensureOrigin(intruders);

  const entities: unknown[] = [];
  const seen = new Set<number>();
  for (const intruder of intruders) {
    if (seen.has(intruder.id)) {
      continue;
    }
    seen.add(intruder.id);
    entities.push(intruderEntity(time, intruder));
  }
  for (const id of intruderTracks.keys()) {
    if (!seen.has(id)) {
      intruderTracks.delete(id);
    }
  }
  const grid = intruderGrid(time, intruders);
  if (grid) {
    entities.push(grid);
  }
  if (sceneOrigin && !sceneOrigin.fromMission) {
    entities.push(
      sceneEntity(time, "intruder_origin_note", {
        texts: [
          {
            pose: { ...IDENTITY_POSE, position: { x: 0, y: 0, z: 0 } },
            billboard: true,
            font_size: 13,
            scale_invariant: true,
            color: DROP_LINE_COLOR,
            text: "origin: first intruder, heights AMSL (no /vtol/mission, the vehicle is not aligned)",
          },
        ],
      }),
    );
  }
  // 1: delete all entities of this topic, applied before the new ones
  return { deletions: [{ timestamp: time, type: 1, id: "" }], entities };
}

function intruderEntity(time: Time, intruder: IntruderState) {
  let track = intruderTracks.get(intruder.id);
  if (!track) {
    track = { heading: 0, trail: [] };
    intruderTracks.set(intruder.id, track);
  }
  track.heading = heading(intruder, track.heading);

  const point = { lat: intruder.latitude, lon: intruder.longitude, alt: intruder.altitude };
  const last = track.trail[track.trail.length - 1];
  if (
    !last ||
    distanceM([last.lon, last.lat], [point.lon, point.lat]) >= INTRUDER_TRAIL_SPACING_M ||
    Math.abs(last.alt - point.alt) >= INTRUDER_TRAIL_SPACING_M
  ) {
    track.trail.push(point);
    if (track.trail.length > INTRUDER_TRAIL_MAX_POINTS) {
      track.trail = track.trail.slice(-INTRUDER_TRAIL_MAX_POINTS);
    }
  }

  const position = toEnu(point.lat, point.lon, point.alt);
  const [north, east, down] = velocityNed(intruder);
  const predicted = {
    x: position.x + east * PREDICTION_S,
    y: position.y + north * PREDICTION_S,
    z: position.z - down * PREDICTION_S,
  };
  const color = hexColor(intruderColor(intruder));
  const pose = {
    position,
    orientation: intruderOrientation(intruder, track.heading),
  };
  const forward = rotate(pose.orientation, { x: 1, y: 0, z: 0 });
  const arrowTail = {
    x: position.x - (forward.x * INTRUDER_ARROW_LENGTH_M) / 2,
    y: position.y - (forward.y * INTRUDER_ARROW_LENGTH_M) / 2,
    z: position.z - (forward.z * INTRUDER_ARROW_LENGTH_M) / 2,
  };
  const multirotor = intruder.type === TYPE_MULTIROTOR;
  const span = multirotor ? INTRUDER_MULTIROTOR_SPAN_M : INTRUDER_FIXED_WING_SPAN_M;
  const outline = (multirotor ? QUADCOPTER : AIRPLANE).map((p) => ({
    x: p.x * span,
    y: p.y * span,
    z: p.z * span,
  }));
  const height =
    sceneOrigin?.fromMission === true
      ? `${(position.z - groundZ).toFixed(0)} m`
      : `${intruder.altitude.toFixed(0)} m AMSL`;

  return sceneEntity(time, `intruder_${String(intruder.id)}`, {
    // solid, so it stays visible from afar; points along the flight path (+x of the pose)
    arrows: [
      {
        pose: { position: arrowTail, orientation: pose.orientation },
        shaft_length: INTRUDER_ARROW_LENGTH_M * 0.65,
        shaft_diameter: 3,
        head_length: INTRUDER_ARROW_LENGTH_M * 0.35,
        head_diameter: 9,
        color,
      },
    ],
    lines: [
      lineStrip(outline, color, 3, 2, pose),
      lineStrip([position, onGround(position)], DROP_LINE_COLOR, 1),
      lineStrip(dashes(position, predicted, PREDICTION_DASHES), { ...color, a: 0.8 }, 2, 2),
      lineStrip(
        track.trail.map((p) => toEnu(p.lat, p.lon, p.alt)),
        { ...color, a: 0.6 },
        1.5,
      ),
    ],
    texts: [
      {
        pose: { ...IDENTITY_POSE, position: { ...position, z: position.z + span / 2 + 4 } },
        billboard: true,
        font_size: 14,
        scale_invariant: true,
        color,
        text: `#${String(intruder.id)} ${typeName(intruder.type)}\n${height}`,
      },
    ],
  });
}

const geoJson = (features: Feature[]) => ({
  geojson: JSON.stringify({ type: "FeatureCollection", features }),
});

// /vtol/mission -> foxglove.GeoJSON: the path and QGC style numbered dots on the Map panel
function missionToGeoJson(msg: MissionPlan) {
  const items = msg.items.filter((item) => item.frame !== FRAME_LOCAL_NED); // global frames: x = lat, y = lon
  const path = line(
    items.map((item) => [item.y, item.x]),
    `#${hex(PATH_COLOR)}`,
    2,
    "Mission (Jetson)",
  );
  const dots = items.flatMap((item) =>
    numberedDot(
      item.x,
      item.y,
      MAP_WAYPOINT_RADIUS_M,
      `#${hex(waypointColor(item))}`,
      String(item.seq + 1),
    ),
  );
  return geoJson([path, ...dots]);
}

// vehicle_local_position -> foxglove.GeoJSON: the vehicle as an arrow along its heading on the Map panel.
// The Map panel only draws the latest message of a topic when live, so the arrow moves instead of smearing
function localPositionToMapVehicle(msg: VehicleLocalPosition) {
  if (!msg.xy_valid || !msg.xy_global || !Number.isFinite(msg.heading)) {
    return undefined;
  }
  const [lon, lat] = offsetLatLon(msg.ref_lat, msg.ref_lon, msg.x, msg.y);
  return geoJson([
    vehicleArrow(lat, lon, msg.heading, MAP_VEHICLE_LENGTH_M, `#${hex(VEHICLE_COLOR)}`),
  ]);
}

// vehicle_global_position -> foxglove.GeoJSON: the flown trail on the Map panel, which has no history
// of its own when live. Restarts when time jumps back (seeking in a bag)
let mapTrail: [number, number][] = [];
let mapTrailLastTime: Time | undefined;

function globalPositionToMapTrail(
  msg: VehicleGlobalPosition,
  event: Immutable<MessageEvent<VehicleGlobalPosition>>,
) {
  const time = event.receiveTime;
  if (isBefore(time, mapTrailLastTime)) {
    mapTrail = [];
  }
  mapTrailLastTime = time;
  if (!msg.lat_lon_valid) {
    return undefined;
  }
  const point: [number, number] = [msg.lon, msg.lat];
  const last = mapTrail[mapTrail.length - 1];
  if (last && distanceM(last, point) < MAP_TRAIL_SPACING_M) {
    return undefined; // nothing new to draw
  }
  mapTrail.push(point);
  if (mapTrail.length > MAP_TRAIL_MAX_POINTS) {
    mapTrail = mapTrail.slice(-MAP_TRAIL_MAX_POINTS);
  }
  return geoJson([line([...mapTrail], `#${hex(TRAIL_COLOR)}`, 3, "Flight trail")]);
}

function isBefore(time: Time, reference: Time | undefined): boolean {
  return (
    reference != undefined &&
    (time.sec < reference.sec || (time.sec === reference.sec && time.nsec < reference.nsec))
  );
}

function hex(color: Color): string {
  return [color.r, color.g, color.b]
    .map((c) =>
      Math.round(c * 255)
        .toString(16)
        .padStart(2, "0"),
    )
    .join("");
}

export function registerConverters(context: ExtensionContext): void {
  context.registerMessageConverter<VehicleOdometry>({
    fromSchemaName: "px4_msgs/msg/VehicleOdometry",
    toSchemaName: "foxglove.FrameTransform",
    supportsLatestPerRenderTick: true,
    converter: odometryToTransform,
  });
  context.registerMessageConverter<VehicleOdometry>({
    fromSchemaName: "px4_msgs/msg/VehicleOdometry",
    toSchemaName: "foxglove.SceneUpdate",
    // needs every message (not only the latest per render tick) to build the trajectory
    converter: odometryToScene,
  });
  context.registerMessageConverter<VehicleGlobalPosition>({
    fromSchemaName: "px4_msgs/msg/VehicleGlobalPosition",
    toSchemaName: "foxglove.LocationFix",
    converter: globalPositionToLocationFix,
  });
  // needs every message to build the trail
  context.registerMessageConverter<VehicleGlobalPosition>({
    fromSchemaName: "px4_msgs/msg/VehicleGlobalPosition",
    toSchemaName: "foxglove.GeoJSON",
    converter: globalPositionToMapTrail,
  });
  context.registerMessageConverter<VehicleLocalPosition>({
    fromSchemaName: "px4_msgs/msg/VehicleLocalPosition",
    toSchemaName: "foxglove.GeoJSON",
    supportsLatestPerRenderTick: true,
    converter: localPositionToMapVehicle,
  });
  context.registerMessageConverter<MissionPlan>({
    fromSchemaName: "vtol_interfaces/msg/MissionPlan",
    toSchemaName: "foxglove.SceneUpdate",
    supportsLatestPerRenderTick: true,
    converter: missionToScene,
  });
  context.registerMessageConverter<MissionPlan>({
    fromSchemaName: "vtol_interfaces/msg/MissionPlan",
    toSchemaName: "foxglove.GeoJSON",
    supportsLatestPerRenderTick: true,
    converter: missionToGeoJson,
  });
  context.registerMessageConverter<IntruderArray>({
    fromSchemaName: INTRUDER_SCHEMA,
    toSchemaName: "foxglove.FrameTransforms",
    supportsLatestPerRenderTick: true,
    converter: intrudersToTransforms,
  });
  // needs every message to build the trails
  context.registerMessageConverter<IntruderArray>({
    fromSchemaName: INTRUDER_SCHEMA,
    toSchemaName: "foxglove.SceneUpdate",
    converter: intrudersToScene,
  });
}
