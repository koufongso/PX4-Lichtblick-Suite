# PX4 Tools (Lichtblick extension)

Lichtblick panels for the Supernal eVTOL. The data comes from `foxglove_bridge` on the Jetson (or in SITL), launched with `ros2 launch px4_vtol px4_vtol.launch.py viz:=true`. The PC needs no ROS and no message definitions.

## Panels

**Flight Status** shows the vehicle state with readable names instead of raw PX4 numbers:

| Row | Source topic | Notes |
|---|---|---|
| Armed, Mode, VTOL state, Control | `vehicle_status_v1`, `vtol_vehicle_status` | QGC-style mode names; Control is "Jetson (offboard)" in Offboard mode, otherwise "PX4" |
| Altitude, Ground speed, Climb | `vehicle_local_position_v1`, `home_position_v1` | altitude above home (above the EKF origin, in amber, until home is known); "—" when PX4 marks a value invalid |
| Airspeed | `airspeed_validated_v1` | true airspeed |
| Battery | `battery_status_v1` | amber below 20 % or on a LOW warning, red from CRITICAL |
| Mission | `/vtol/mission_progress`, else `mission_result` | 1-based like QGC; prefers the Jetson's progress, which also covers offboard |
| Failsafe | `vehicle_status_v1`, `vehicle_land_detected` | failsafe active, GCS link lost, landed |
| Link | all of the above | red after 2 s without data, with a "ROS link lost, check QGC" banner at the top and the other values greyed out; once data is back it shows when the last loss started and how long it lasted |

Both the versioned (`_v1`) and unversioned topic names are subscribed, so other PX4 versions work too.

**Flight Map** is a QGC style map (recommended over the built-in Map panel, which redraws the vehicle on every message so it flickers):

- the vehicle as an arrow along its heading, and the flown trail, from `/viz/vehicle_local_position_v1` (or `/fmu/out/vehicle_local_position_v1` in bags); updated in place, so it moves smoothly
- the Jetson's mission from `/vtol/mission` as numbered dots (numbered from 1 like QGC) and the path; drawn as soon as it arrives
- the current waypoint (amber, larger) from `/vtol/mission_progress`, or PX4's `mission_result` without the Jetson
- intruders from `/intruder/states` (see below)
- Map / Satellite switch (top right) and a **Follow** button (top left) to keep the vehicle centred; both are saved with the layout

It needs internet for the map tiles (OpenStreetMap, Esri satellite).

## Layout

[`layouts/px4-vtol.json`](layouts/px4-vtol.json) is the default layout: Flight Status, Flight Map and a 3D panel with the scene topics already visible (`/viz/vehicle_odometry` and `/fmu/out/vehicle_odometry` for bags, `/vtol/mission`, `/intruder/states`), the camera zoomed out to ~1.5 km and the frame connector lines hidden. Import it once in Lichtblick (Layouts → Import from file). The 3D panel hides every topic by default, so without this layout each one has to be switched on in the panel's Topics settings.

After changing the layout in Lichtblick, export it (Layouts → Export) over this file to keep it as the default.

## Intruders

`/intruder/states` (`vtol_interfaces/IntruderArray`, from the `virtual_intruder` node, later from real ADS-B / Remote ID) is drawn on the Flight Map and in the 3D panel. Red: non-cooperative (only known from detection), magenta: cooperative (broadcasts its own state).

- **Flight Map**: an airplane or quadcopter icon along the ground track, a label with the id and the height relative to the vehicle (AMSL until the vehicle's altitude is known), the trail, and a dashed line to where it will be in 10 s. Hover for type, altitude and speed. An intruder missing from the latest message is removed. With nothing else to show, the map zooms to the intruders.
- **3D**: enable `/intruder/states` (converted to `foxglove.SceneUpdate`): a solid arrow along the intruder's pose (heading and climb) with its outline, a drop line, id and height above home, the dashed 10 s prediction line (constant velocity, not the flight intent) and the solid trail. Without the vehicle, the intruders draw the 50 m ground grid around themselves; once the vehicle's grid is there, theirs is dropped.
- **3D frames**: `/intruder/states` converted to `foxglove.FrameTransforms` adds a frame `intruder_<id>` per intruder (`map` → `intruder_<id>`, like the vehicle's `map` → `base_link`). The panel shows its axes (x forward along the flight path, z up; no roll, intruders report no attitude), and setting the display frame to `intruder_<id>` follows it. A frame stays at its last pose after the intruder is gone.

The 3D scene is centred on the EKF origin, which only `/vtol/mission` carries. Without it (e.g. testing the intruders alone) the first intruder is used as the origin and heights are AMSL, as noted in the scene; the vehicle would not line up with the intruders then.

To test the intruders without the vehicle, run the node and a bridge on the laptop (in the dev container, `--network=host`) and connect Lichtblick to `ws://localhost:8765`:

```bash
ros2 launch virtual_intruder virtual_intruder.launch.py
ros2 run foxglove_bridge foxglove_bridge   # second terminal
```

## Map and 3D views (message converters)

The extension converts PX4 / `vtol_node` topics into types the built-in **Map** and **3D** panels understand. The conversion runs in Lichtblick, live and for recorded bags, so the Jetson only publishes the raw topics.

Live from the Jetson, the high rate PX4 topics arrive throttled (15 Hz, `viz_rate:=`) as `/viz/<topic>`; in recorded bags use the `/fmu/out/<topic>` originals, the converters work on both.

| Source topic | Converted to | Panel: what to enable |
|---|---|---|
| `/viz/vehicle_global_position` | `foxglove.LocationFix` | Map: a dot at the position, which the built-in Map panel needs to centre and follow the map |
| `/viz/vehicle_global_position` | `foxglove.GeoJSON` | Map: the flown trail, built up live (one point per 3 m) |
| `/viz/vehicle_local_position_v1` | `foxglove.GeoJSON` | Map: the vehicle as a QGC style arrow along its heading |
| `/vtol/mission` | `foxglove.GeoJSON` | Map: QGC style dots with the item number, and the path |
| `/viz/vehicle_odometry` | `foxglove.FrameTransform` (`map` → `base_link`) | 3D: lets the camera follow the vehicle (display frame `base_link`, follow mode Position) |
| `/viz/vehicle_odometry` | `foxglove.SceneUpdate` | 3D: the whole flight scene. The vehicle as an airplane outline (shows roll and pitch) with a drop line and its height above home; the flown trajectory with its ground shadow and a small outline every 30 m (attitude history); a ground grid around the vehicle (50 m cells, a darker line every 250 m) with a north marker. Restarts when seeking back in a bag |
| `/intruder/states` | `foxglove.SceneUpdate` | 3D: the intruders, see [Intruders](#intruders) |
| `/intruder/states` | `foxglove.FrameTransforms` (`map` → `intruder_<id>`) | 3D: a frame per intruder, to see its pose or follow it |
| `/vtol/mission` | `foxglove.SceneUpdate` | 3D: waypoint spheres (green takeoff, orange land), labels, the path and drop lines; also sets the ground height to home |

The 3D scene is ENU with its origin at the EKF origin; PX4's NED / FRD is converted in `src/frames.ts`. The mission only contains the Jetson's simple items, PX4's landing pattern is not drawn.

## Development

Requires Node 20 (`nvm install 20`).

```bash
npm install
npm run build          # compile
npm run lint           # ESLint + Prettier
npm test               # unit tests of the display logic (src/flightStatus.ts, src/intruders.ts)
npm run local-install  # build and install into ~/.lichtblick-suite/extensions, then restart Lichtblick
npm run package        # build a .foxe file to install on other machines (Lichtblick: Settings -> Extensions)
```

The enum names (flight modes, VTOL states, battery warnings) are generated from `px4_msgs`. Regenerate them after a PX4 update:

```bash
npm run gen:enums      # reads ../../Supernal-eVTOL/ros2/src/px4_msgs/msg
```

### Layout

- `src/index.ts`: registers the panels
- `src/flightStatus.ts`: turns messages into displayed values, no React, unit tested in `test/`
- `src/FlightStatusPanel.tsx`: the panel UI
- `src/FlightMapPanel.ts`: the Flight Map panel (Leaflet)
- `src/intruders.ts`: intruder display logic (heading, labels, colours), unit tested; `src/IntruderMapLayer.ts`: the intruders on the Flight Map
- `src/converters.ts`: the message converters, `src/frames.ts`: NED/FRD -> ENU/FLU
- `src/generated/px4Enums.ts`: generated by `scripts/gen_px4_enums.py`, do not edit by hand
- `src/generated/leafletCss.ts`: Leaflet's stylesheet, generated by `npm run gen:leaflet-css` (re-run after updating leaflet)
