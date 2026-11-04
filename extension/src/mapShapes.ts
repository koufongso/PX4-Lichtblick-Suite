// GeoJSON shapes for the Map panel. It draws GeoJSON points as large pin markers and has no text,
// so the vehicle, the waypoint dots and their numbers are drawn as small polygons and lines instead.

type LonLat = [number, number]; // GeoJSON order
export type Feature = {
  type: "Feature";
  geometry:
    | { type: "LineString"; coordinates: LonLat[] }
    | { type: "MultiLineString"; coordinates: LonLat[][] }
    | { type: "Polygon"; coordinates: LonLat[][] };
  properties: { name?: string; style: Record<string, unknown> };
};

const EARTH_RADIUS_M = 6371000;

// lat/lon of a point north / east of a reference [m], flat earth (fine over a few km)
export function offsetLatLon(lat: number, lon: number, north: number, east: number): LonLat {
  const dLat = (north / EARTH_RADIUS_M) * (180 / Math.PI);
  const dLon = (east / (EARTH_RADIUS_M * Math.cos((lat * Math.PI) / 180))) * (180 / Math.PI);
  return [lon + dLon, lat + dLat];
}

// [north, east] of a point from a reference [m], the inverse of offsetLatLon
export function northEastOf(
  refLat: number,
  refLon: number,
  lat: number,
  lon: number,
): [number, number] {
  const north = ((lat - refLat) * Math.PI * EARTH_RADIUS_M) / 180;
  const east =
    ((lon - refLon) * Math.PI * EARTH_RADIUS_M * Math.cos((refLat * Math.PI) / 180)) / 180;
  return [north, east];
}

export function distanceM(a: LonLat, b: LonLat): number {
  const north = ((b[1] - a[1]) * Math.PI * EARTH_RADIUS_M) / 180;
  const east = ((b[0] - a[0]) * Math.PI * EARTH_RADIUS_M * Math.cos((a[1] * Math.PI) / 180)) / 180;
  return Math.hypot(north, east);
}

// QGC style vehicle arrow (pointing along the heading) centred on lat/lon; heading [rad] from north, clockwise
export function vehicleArrow(
  lat: number,
  lon: number,
  heading: number,
  length: number,
  color: string,
): Feature {
  // outline in (forward, right) [m]: tip, right wing, notch, left wing
  const shape: [number, number][] = [
    [length / 2, 0],
    [-length / 2, length * 0.35],
    [-length / 4, 0],
    [-length / 2, -length * 0.35],
    [length / 2, 0],
  ];
  const [cos, sin] = [Math.cos(heading), Math.sin(heading)];
  const ring = shape.map(([forward, right]) =>
    offsetLatLon(lat, lon, forward * cos - right * sin, forward * sin + right * cos),
  );
  return {
    type: "Feature",
    geometry: { type: "Polygon", coordinates: [ring] },
    properties: {
      name: "Vehicle",
      style: { color: "#000000", weight: 1, fillColor: color, fillOpacity: 1 },
    },
  };
}

export function line(coordinates: LonLat[], color: string, weight: number, name?: string): Feature {
  return {
    type: "Feature",
    geometry: { type: "LineString", coordinates },
    properties: { name, style: { color, weight, opacity: 0.9 } },
  };
}

// filled dot with the number drawn inside, like QGC's mission items
export function numberedDot(
  lat: number,
  lon: number,
  radius: number,
  color: string,
  label: string,
): Feature[] {
  const ring: LonLat[] = [];
  for (let i = 0; i <= 24; i++) {
    const angle = (i / 24) * 2 * Math.PI;
    ring.push(offsetLatLon(lat, lon, radius * Math.cos(angle), radius * Math.sin(angle)));
  }
  const dot: Feature = {
    type: "Feature",
    geometry: { type: "Polygon", coordinates: [ring] },
    properties: {
      name: label,
      style: { color: "#ffffff", weight: 1, fillColor: color, fillOpacity: 1 },
    },
  };
  const text: Feature = {
    type: "Feature",
    geometry: { type: "MultiLineString", coordinates: digitStrokes(lat, lon, label, radius * 1.1) },
    properties: { name: label, style: { color: "#ffffff", weight: 2, opacity: 1 } },
  };
  return [dot, text];
}

// seven-segment digits: segment end points in a 0.5 wide, 1 high cell, (x right, y up)
const SEGMENTS: Record<string, [[number, number], [number, number]]> = {
  a: [
    [0, 1],
    [0.5, 1],
  ],
  b: [
    [0.5, 1],
    [0.5, 0.5],
  ],
  c: [
    [0.5, 0.5],
    [0.5, 0],
  ],
  d: [
    [0, 0],
    [0.5, 0],
  ],
  e: [
    [0, 0.5],
    [0, 0],
  ],
  f: [
    [0, 1],
    [0, 0.5],
  ],
  g: [
    [0, 0.5],
    [0.5, 0.5],
  ],
};
const DIGITS: Record<string, string> = {
  "0": "abcdef",
  "1": "bc",
  "2": "abged",
  "3": "abgcd",
  "4": "fgbc",
  "5": "afgcd",
  "6": "afgedc",
  "7": "abc",
  "8": "abcdefg",
  "9": "abcdfg",
};

// strokes of a number centred on lat/lon, digits `height` metres high
function digitStrokes(lat: number, lon: number, text: string, height: number): LonLat[][] {
  const digits = text.match(/\d/g) ?? [];
  const advance = 0.8; // cell width incl. spacing, in digit heights
  const width = (digits.length - 1) * advance + 0.5;
  const strokes: LonLat[][] = [];
  digits.forEach((digit, i) => {
    for (const segment of DIGITS[digit] ?? "") {
      const ends = SEGMENTS[segment];
      if (!ends) {
        continue;
      }
      strokes.push(
        ends.map(([x, y]) =>
          offsetLatLon(lat, lon, (y - 0.5) * height, (i * advance + x - width / 2) * height),
        ),
      );
    }
  });
  return strokes;
}
