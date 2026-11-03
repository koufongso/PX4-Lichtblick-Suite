// PX4 uses NED (world) and FRD (body), Lichtblick's 3D panel uses the ROS convention ENU (world) and FLU (body).

export type Vector3 = { x: number; y: number; z: number };
export type Quaternion = { x: number; y: number; z: number; w: number };

// frame ids of the 3D scene: "map" is ENU with its origin at the EKF origin (vehicle_local_position ref_lat/ref_lon)
export const WORLD_FRAME = "map";
export const BODY_FRAME = "base_link";

export function nedToEnu(north: number, east: number, down: number): Vector3 {
  return { x: east, y: north, z: -down };
}

function multiply(a: Quaternion, b: Quaternion): Quaternion {
  return {
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

// rotate a vector by a unit quaternion
export function rotate(q: Quaternion, v: Vector3): Vector3 {
  const r = multiply(multiply(q, { w: 0, ...v }), { w: q.w, x: -q.x, y: -q.y, z: -q.z });
  return { x: r.x, y: r.y, z: r.z };
}

// NED -> ENU: 180 deg about (1, 1, 0) / sqrt(2); FLU -> FRD: 180 deg about x
const ENU_FROM_NED: Quaternion = { w: 0, x: Math.SQRT1_2, y: Math.SQRT1_2, z: 0 };
const FRD_FROM_FLU: Quaternion = { w: 0, x: 1, y: 0, z: 0 };

// PX4 attitude [w, x, y, z] (FRD body -> NED) to the ROS one (FLU body -> ENU)
export function px4QuaternionToEnu(q: ArrayLike<number>): Quaternion | undefined {
  const [w, x, y, z] = [q[0], q[1], q[2], q[3]];
  if (w == undefined || x == undefined || y == undefined || z == undefined || !Number.isFinite(w)) {
    return undefined; // PX4 sets the first value to NaN when the attitude is invalid
  }
  return multiply(multiply(ENU_FROM_NED, { w, x, y, z }), FRD_FROM_FLU);
}
