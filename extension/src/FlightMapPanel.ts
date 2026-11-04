// Flight Map panel: a QGC style map. Unlike the built-in Map panel, which deletes and redraws a topic's shapes
// on every message (the vehicle flickers at 15 Hz), the vehicle marker and the trail are updated in place.
// It can also combine topics, to highlight the current waypoint.

import { MessageEvent, PanelExtensionContext, Time } from "@lichtblick/suite";
import L from "leaflet";

import { INTRUDER_STYLE, IntruderMapLayer } from "./IntruderMapLayer";
import { LEAFLET_CSS } from "./generated/leafletCss";
import { INTRUDER_TOPIC, IntruderArray } from "./intruders";
import { distanceM, offsetLatLon } from "./mapShapes";

// /viz/*: throttled relay on the Jetson (viz:=true), the raw topics in recorded bags
const LOCAL_POSITION_TOPICS = [
  "/viz/vehicle_local_position_v1",
  "/fmu/out/vehicle_local_position_v1",
  "/fmu/out/vehicle_local_position",
];
const MISSION_TOPIC = "/vtol/mission";
const MISSION_PROGRESS_TOPIC = "/vtol/mission_progress";
const MISSION_RESULT_TOPIC = "/fmu/out/mission_result";

const TRAIL_SPACING_M = 3;
const TRAIL_MAX_POINTS = 10000;
const FRAME_LOCAL_NED = 1; // vtol_interfaces MissionItem frames: 0 global, 1 local NED, 3 global relative alt
const CMD_TAKEOFF = new Set([22, 84]);
const CMD_LAND = new Set([21, 85]);

// same tiles as the built-in Map panel
const STREET_TILES = "https://tile.openstreetmap.org/{z}/{x}/{y}.png";
const SATELLITE_TILES =
  "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}";

type LocalPosition = {
  x: number;
  y: number;
  z: number;
  xy_valid: boolean;
  z_valid: boolean;
  xy_global: boolean;
  z_global: boolean; // ref_alt valid
  ref_lat: number;
  ref_lon: number;
  ref_alt: number;
  heading: number;
};
type MissionItem = { seq: number; command: number; frame: number; x: number; y: number };
type MissionPlan = { items: MissionItem[] };
type MissionProgress = { current_index: number };
type MissionResult = { seq_current: number; valid: boolean };

type SavedState = { follow: boolean; satellite: boolean };

const STYLE = `
.fm-vehicle svg { overflow: visible; filter: drop-shadow(0 0 2px rgba(0, 0, 0, 0.8)); }
.fm-wp {
  display: flex; align-items: center; justify-content: center; box-sizing: border-box;
  width: 100%; height: 100%; border-radius: 50%; border: 2px solid #fff;
  color: #fff; font: bold 11px sans-serif; box-shadow: 0 0 2px rgba(0, 0, 0, 0.8);
}
.fm-wp-current { border-color: #000; color: #000; box-shadow: 0 0 6px 2px #ffb300; }
.fm-button {
  background: #fff; color: #333; border: none; padding: 4px 8px; font: 12px sans-serif; cursor: pointer;
}
.fm-button-on { background: #ffb300; }
`;

const VEHICLE_SVG = `<svg width="34" height="34" viewBox="-17 -17 34 34">
  <path d="M0,-15 L11,13 L0,6 L-11,13 Z" fill="#ffe01a" stroke="#000" stroke-width="1.5" stroke-linejoin="round"/>
</svg>`;

function waypointColor(item: MissionItem): string {
  return CMD_TAKEOFF.has(item.command)
    ? "#2e7d32"
    : CMD_LAND.has(item.command)
      ? "#c62828"
      : "#0090c0";
}

function isBefore(time: Time, reference: Time | undefined): boolean {
  return (
    reference != undefined &&
    (time.sec < reference.sec || (time.sec === reference.sec && time.nsec < reference.nsec))
  );
}

class FlightMap {
  readonly #map: L.Map;
  readonly #streets: L.TileLayer;
  readonly #satellite: L.TileLayer;
  readonly #vehicle: L.Marker;
  #vehicleArrow: HTMLElement | SVGElement;
  readonly #trail: L.Polyline;
  readonly #missionPath: L.Polyline;
  readonly #missionLayer = L.layerGroup();
  readonly #followButton: HTMLButtonElement;
  readonly #intruders: IntruderMapLayer;

  #waypoints = new Map<number, { marker: L.Marker; item: MissionItem }>();
  #currentIndex: number | undefined;
  #hasProgress = false; // /vtol/mission_progress seen, then PX4's mission_result is ignored
  #trailPoints: L.LatLng[] = [];
  #vehicleAltitude: number | undefined; // [m] AMSL, for the intruders' relative height
  #lastTime: Time | undefined;
  #centred = false;
  #state: SavedState;

  constructor(
    element: HTMLElement,
    initialState: SavedState,
    private readonly saveState: (state: SavedState) => void,
  ) {
    this.#state = initialState;
    this.#map = L.map(element, { zoomControl: true, attributionControl: true }).setView([0, 0], 2);
    this.#streets = L.tileLayer(STREET_TILES, {
      maxZoom: 22,
      maxNativeZoom: 19,
      attribution: "&copy; OpenStreetMap contributors",
    });
    this.#satellite = L.tileLayer(SATELLITE_TILES, {
      maxZoom: 22,
      maxNativeZoom: 18,
      attribution: "&copy; Esri",
    });
    (initialState.satellite ? this.#satellite : this.#streets).addTo(this.#map);
    L.control
      .layers({ Map: this.#streets, Satellite: this.#satellite }, undefined, {
        position: "topright",
        collapsed: false, // the layer icon image is not bundled, show the list instead
      })
      .addTo(this.#map);
    this.#map.on("baselayerchange", (event: L.LayersControlEvent) => {
      this.#setState({ satellite: event.name === "Satellite" });
    });

    this.#missionPath = L.polyline([], { color: "#00ccff", weight: 2, opacity: 0.9 }).addTo(
      this.#map,
    );
    this.#missionLayer.addTo(this.#map);
    this.#trail = L.polyline([], { color: "#ff9900", weight: 3, opacity: 0.9 }).addTo(this.#map);
    this.#intruders = new IntruderMapLayer(this.#map);

    const icon = L.divIcon({ className: "fm-vehicle", html: VEHICLE_SVG, iconSize: [34, 34] });
    this.#vehicle = L.marker([0, 0], {
      icon,
      interactive: false,
      keyboard: false,
      zIndexOffset: 1000,
    });
    this.#vehicleArrow = document.createElement("div"); // replaced once the marker is on the map

    this.#followButton = document.createElement("button");
    const followControl = new L.Control({ position: "topleft" });
    followControl.onAdd = () => {
      const container = L.DomUtil.create("div", "leaflet-bar");
      this.#followButton.className = "fm-button";
      this.#followButton.textContent = "Follow";
      this.#followButton.title = "Keep the vehicle in the centre";
      L.DomEvent.disableClickPropagation(container);
      this.#followButton.addEventListener("click", () => {
        this.#setState({ follow: !this.#state.follow });
      });
      container.appendChild(this.#followButton);
      return container;
    };
    followControl.addTo(this.#map);
    this.#updateFollowButton();
  }

  resize(): void {
    this.#map.invalidateSize();
  }

  destroy(): void {
    this.#map.remove();
  }

  handle(event: MessageEvent): void {
    if (isBefore(event.receiveTime, this.#lastTime)) {
      this.#clearTrail(); // time jumped back: seeking in a bag
      this.#intruders.clear();
    }
    this.#lastTime = event.receiveTime;

    if (LOCAL_POSITION_TOPICS.includes(event.topic)) {
      this.#updateVehicle(event.message as LocalPosition);
    } else if (event.topic === MISSION_TOPIC) {
      this.#updateMission(event.message as MissionPlan);
    } else if (event.topic === MISSION_PROGRESS_TOPIC) {
      this.#hasProgress = true;
      this.#setCurrentIndex((event.message as MissionProgress).current_index);
    } else if (event.topic === MISSION_RESULT_TOPIC && !this.#hasProgress) {
      const result = event.message as MissionResult;
      this.#setCurrentIndex(result.valid ? result.seq_current : undefined);
    } else if (event.topic === INTRUDER_TOPIC) {
      const positions = this.#intruders.update(
        event.message as IntruderArray,
        this.#vehicleAltitude,
      );
      // nothing else to centre on yet (e.g. testing the intruders without the vehicle)
      if (!this.#centred && positions.length > 0) {
        this.#centred = true;
        this.#map.fitBounds(L.latLngBounds(positions), { padding: [60, 60], maxZoom: 16 });
      }
    }
  }

  #updateVehicle(msg: LocalPosition): void {
    this.#vehicleAltitude =
      msg.z_valid && msg.z_global && Number.isFinite(msg.z) ? msg.ref_alt - msg.z : undefined;
    if (!msg.xy_valid || !msg.xy_global) {
      return;
    }
    const [lon, lat] = offsetLatLon(msg.ref_lat, msg.ref_lon, msg.x, msg.y);
    const position = L.latLng(lat, lon);

    if (!this.#map.hasLayer(this.#vehicle)) {
      this.#vehicle.addTo(this.#map);
      this.#vehicleArrow = this.#vehicle.getElement()?.querySelector("svg") ?? this.#vehicleArrow;
    }
    this.#vehicle.setLatLng(position);
    if (Number.isFinite(msg.heading)) {
      this.#vehicleArrow.style.transform = `rotate(${String((msg.heading * 180) / Math.PI)}deg)`;
    }

    const last = this.#trailPoints[this.#trailPoints.length - 1];
    if (!last || distanceM([last.lng, last.lat], [lon, lat]) >= TRAIL_SPACING_M) {
      this.#trailPoints.push(position);
      if (this.#trailPoints.length > TRAIL_MAX_POINTS) {
        this.#trailPoints = this.#trailPoints.slice(-TRAIL_MAX_POINTS);
        this.#trail.setLatLngs(this.#trailPoints);
      } else {
        this.#trail.addLatLng(position);
      }
    }

    if (this.#state.follow) {
      this.#map.panTo(position, { animate: false });
    } else if (!this.#centred) {
      this.#centred = true;
      this.#map.setView(position, 16);
    }
  }

  #updateMission(msg: MissionPlan): void {
    this.#missionLayer.clearLayers();
    this.#waypoints.clear();
    const items = msg.items.filter((item) => item.frame !== FRAME_LOCAL_NED); // global frames: x = lat, y = lon
    for (const item of items) {
      const marker = L.marker([item.x, item.y], {
        icon: this.#waypointIcon(item),
        keyboard: false,
        title: `WP ${String(item.seq + 1)}`,
      });
      marker.addTo(this.#missionLayer);
      this.#waypoints.set(item.seq, { marker, item });
    }
    const path = items.map((item) => L.latLng(item.x, item.y));
    this.#missionPath.setLatLngs(path);

    // show the whole mission right away, until the vehicle position takes over
    if (!this.#centred && path.length > 0) {
      this.#centred = true;
      this.#map.fitBounds(L.latLngBounds(path), { padding: [30, 30] });
    }
  }

  #setCurrentIndex(index: number | undefined): void {
    if (index === this.#currentIndex) {
      return;
    }
    const previous = this.#currentIndex;
    this.#currentIndex = index;
    for (const seq of [previous, index]) {
      const waypoint = seq == undefined ? undefined : this.#waypoints.get(seq);
      waypoint?.marker.setIcon(this.#waypointIcon(waypoint.item));
    }
  }

  // QGC style dot with the item number (QGC counts from 1); the current one is larger, amber
  #waypointIcon(item: MissionItem): L.DivIcon {
    const current = item.seq === this.#currentIndex;
    const size = current ? 26 : 20;
    const background = current ? "#ffb300" : waypointColor(item);
    return L.divIcon({
      className: "",
      html: `<div class="fm-wp${current ? " fm-wp-current" : ""}" style="background:${background}">${String(item.seq + 1)}</div>`,
      iconSize: [size, size],
    });
  }

  #clearTrail(): void {
    this.#trailPoints = [];
    this.#trail.setLatLngs([]);
  }

  #setState(change: Partial<SavedState>): void {
    this.#state = { ...this.#state, ...change };
    this.saveState(this.#state);
    this.#updateFollowButton();
    const position = this.#vehicle.getLatLng();
    if (change.follow === true && this.#map.hasLayer(this.#vehicle)) {
      this.#map.panTo(position);
    }
  }

  #updateFollowButton(): void {
    this.#followButton.classList.toggle("fm-button-on", this.#state.follow);
  }
}

export function initFlightMapPanel(context: PanelExtensionContext): () => void {
  const style = document.createElement("style");
  style.textContent = LEAFLET_CSS + STYLE + INTRUDER_STYLE;
  context.panelElement.appendChild(style);

  const container = document.createElement("div");
  container.style.width = "100%";
  container.style.height = "100%";
  context.panelElement.appendChild(container);

  const saved = (context.initialState ?? {}) as Partial<SavedState>;
  const map = new FlightMap(
    container,
    { follow: saved.follow ?? false, satellite: saved.satellite ?? true },
    (state) => {
      context.saveState(state);
    },
  );

  const resizeObserver = new ResizeObserver(() => {
    map.resize();
  });
  resizeObserver.observe(container);

  context.onRender = (renderState, done) => {
    for (const event of renderState.currentFrame ?? []) {
      map.handle(event);
    }
    done();
  };
  context.watch("currentFrame");
  context.subscribe(
    [
      ...LOCAL_POSITION_TOPICS,
      MISSION_TOPIC,
      MISSION_PROGRESS_TOPIC,
      MISSION_RESULT_TOPIC,
      INTRUDER_TOPIC,
    ].map((topic) => ({ topic })),
  );

  return () => {
    resizeObserver.disconnect();
    map.destroy();
  };
}
