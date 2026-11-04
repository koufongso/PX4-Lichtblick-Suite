// Intruders on the Flight Map: an icon per intruder (airplane or quadcopter) along its ground track, a label with
// its id and height relative to the vehicle, its trail, and a dashed line to where it will be in PREDICTION_S.
// Markers are updated in place; an intruder missing from the latest message is removed.

import L from "leaflet";

import {
  IntruderArray,
  IntruderState,
  PREDICTION_S,
  TYPE_MULTIROTOR,
  details,
  heading,
  intruderColor,
  isValid,
  shortLabel,
  velocityNed,
} from "./intruders";
import { distanceM, offsetLatLon } from "./mapShapes";

const TRAIL_SPACING_M = 3;
const TRAIL_MAX_POINTS = 2000;

export const INTRUDER_STYLE = `
.fm-intruder { position: relative; width: 100%; height: 100%; }
.fm-intruder svg { overflow: visible; filter: drop-shadow(0 0 2px rgba(0, 0, 0, 0.8)); }
.fm-intruder span {
  position: absolute; left: 50%; top: 100%; transform: translateX(-50%); white-space: nowrap;
  padding: 0 3px; border-radius: 3px; background: rgba(0, 0, 0, 0.6); color: #fff; font: bold 11px sans-serif;
}
`;

function iconSvg(intruder: IntruderState): string {
  const color = intruderColor(intruder);
  const shape =
    intruder.type === TYPE_MULTIROTOR
      ? // quadcopter from above: X frame and four rotors
        `<path d="M-8,-8 L8,8 M8,-8 L-8,8" stroke="#000" stroke-width="4"/>
         <path d="M-8,-8 L8,8 M8,-8 L-8,8" stroke="${color}" stroke-width="2"/>
         ${[
           [-8, -8],
           [8, -8],
           [8, 8],
           [-8, 8],
         ]
           .map(
             ([x, y]) =>
               `<circle cx="${String(x)}" cy="${String(y)}" r="4.5" fill="${color}" stroke="#000" stroke-width="1.2"/>`,
           )
           .join("")}
         <path d="M0,-6 L3,-1 L-3,-1 Z" fill="#fff"/>`
      : // fixed-wing (also unknown) from above, nose up
        `<path d="M0,-13 L2,-6 L13,1 L13,4 L2,1 L1.5,8 L5,11 L5,13 L0,12 L-5,13 L-5,11 L-1.5,8 L-2,1 L-13,4 L-13,1 L-2,-6 Z"
          fill="${color}" stroke="#000" stroke-width="1.2" stroke-linejoin="round"/>`;
  return `<svg width="30" height="30" viewBox="-15 -15 30 30">${shape}</svg>`;
}

type Entry = {
  type: number;
  cooperative: boolean;
  heading: number;
  marker: L.Marker;
  trail: L.Polyline;
  trailPoints: L.LatLng[];
  prediction: L.Polyline;
};

export class IntruderMapLayer {
  readonly #layer = L.layerGroup();
  readonly #entries = new Map<number, Entry>();

  constructor(map: L.Map) {
    this.#layer.addTo(map);
  }

  // positions of the intruders in the latest message (to centre the map on them)
  update(msg: IntruderArray, vehicleAltitude: number | undefined): L.LatLng[] {
    const seen = new Set<number>();
    const positions: L.LatLng[] = [];
    for (const intruder of msg.intruders) {
      if (!isValid(intruder) || seen.has(intruder.id)) {
        continue;
      }
      seen.add(intruder.id);
      const position = L.latLng(intruder.latitude, intruder.longitude);
      positions.push(position);
      this.#updateIntruder(intruder, position, vehicleAltitude);
    }
    for (const [id, entry] of this.#entries) {
      if (!seen.has(id)) {
        this.#remove(id, entry);
      }
    }
    return positions;
  }

  // time jumped back (seeking in a bag)
  clear(): void {
    for (const [id, entry] of this.#entries) {
      this.#remove(id, entry);
    }
  }

  #updateIntruder(
    intruder: IntruderState,
    position: L.LatLng,
    vehicleAltitude: number | undefined,
  ) {
    let entry = this.#entries.get(intruder.id);
    if (entry && (entry.type !== intruder.type || entry.cooperative !== intruder.cooperative)) {
      this.#remove(intruder.id, entry); // icon or colour changed, rebuild
      entry = undefined;
    }
    entry ??= this.#create(intruder, position);

    entry.heading = heading(intruder, entry.heading);
    entry.marker.setLatLng(position);
    const element = entry.marker.getElement();
    const svg = element?.querySelector("svg");
    if (svg) {
      svg.style.transform = `rotate(${String((entry.heading * 180) / Math.PI)}deg)`;
    }
    const label = element?.querySelector("span");
    if (label) {
      label.textContent = shortLabel(intruder, vehicleAltitude);
    }
    entry.marker.setTooltipContent(details(intruder, vehicleAltitude).join("<br>"));

    const [north, east] = velocityNed(intruder);
    const [lon, lat] = offsetLatLon(
      intruder.latitude,
      intruder.longitude,
      north * PREDICTION_S,
      east * PREDICTION_S,
    );
    entry.prediction.setLatLngs([position, [lat, lon]]);

    const last = entry.trailPoints[entry.trailPoints.length - 1];
    if (!last || distanceM([last.lng, last.lat], [position.lng, position.lat]) >= TRAIL_SPACING_M) {
      entry.trailPoints.push(position);
      if (entry.trailPoints.length > TRAIL_MAX_POINTS) {
        entry.trailPoints = entry.trailPoints.slice(-TRAIL_MAX_POINTS);
        entry.trail.setLatLngs(entry.trailPoints);
      } else {
        entry.trail.addLatLng(position);
      }
    }
  }

  #create(intruder: IntruderState, position: L.LatLng): Entry {
    const color = intruderColor(intruder);
    const trail = L.polyline([], { color, weight: 2, opacity: 0.7, interactive: false }).addTo(
      this.#layer,
    );
    const prediction = L.polyline([], {
      color,
      weight: 2,
      opacity: 0.9,
      dashArray: "4 4",
      interactive: false,
    }).addTo(this.#layer);
    const marker = L.marker(position, {
      icon: L.divIcon({
        className: "",
        html: `<div class="fm-intruder">${iconSvg(intruder)}<span></span></div>`,
        iconSize: [30, 30],
      }),
      keyboard: false,
      zIndexOffset: 500, // below the vehicle (1000), above the waypoints
    })
      .bindTooltip("", { direction: "top", offset: [0, -14] })
      .addTo(this.#layer);
    const entry: Entry = {
      type: intruder.type,
      cooperative: intruder.cooperative,
      heading: 0,
      marker,
      trail,
      trailPoints: [],
      prediction,
    };
    this.#entries.set(intruder.id, entry);
    return entry;
  }

  #remove(id: number, entry: Entry): void {
    this.#layer.removeLayer(entry.marker);
    this.#layer.removeLayer(entry.trail);
    this.#layer.removeLayer(entry.prediction);
    this.#entries.delete(id);
  }
}
