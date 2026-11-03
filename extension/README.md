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
- Map / Satellite switch (top right) and a **Follow** button (top left) to keep the vehicle centred; both are saved with the layout

It needs internet for the map tiles (OpenStreetMap, Esri satellite).

**Control Panel** sends commands to the Jetson. It only works on a live Foxglove WebSocket connection (the bridge must have the `clientPublish` capability and `/intruder/*` in `client_topic_whitelist`).

| Control | Topic | Notes |
|---|---|---|
| Intruder simulation Start / Stop | `/intruder/sim_enable` (`std_msgs/msg/Bool`, published) | nothing is sent until a button is pressed; then the last command is resent every 1 s, so a node started later still gets it |
| Jetson status | `/intruder/sim_active` (`std_msgs/msg/Bool`, subscribed, optional) | shows running / stopped, "stale" after 3 s without a message, "no feedback" if never received |

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
| `/viz/vehicle_odometry` | `foxglove.SceneUpdate` | 3D: the whole flight scene. The vehicle as an airplane outline (shows roll and pitch) with a drop line and its height above home; the flown trajectory with its ground shadow and a small outline every 30 m (attitude history); a 50 m ground grid around the vehicle with a north marker. Restarts when seeking back in a bag |
| `/vtol/mission` | `foxglove.SceneUpdate` | 3D: waypoint spheres (green takeoff, orange land), labels, the path and drop lines; also sets the ground height to home |

The 3D scene is ENU with its origin at the EKF origin; PX4's NED / FRD is converted in `src/frames.ts`. The mission only contains the Jetson's simple items, PX4's landing pattern is not drawn.

## Development

Requires Node 20 (`nvm install 20`).

```bash
npm install
npm run build          # compile
npm run lint           # ESLint + Prettier
npm test               # unit tests of the display logic (src/flightStatus.ts)
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
- `src/ControlPanel.tsx`: the Control Panel (publishes to the Jetson)
- `src/converters.ts`: the message converters, `src/frames.ts`: NED/FRD -> ENU/FLU
- `src/generated/px4Enums.ts`: generated by `scripts/gen_px4_enums.py`, do not edit by hand
- `src/generated/leafletCss.ts`: Leaflet's stylesheet, generated by `npm run gen:leaflet-css` (re-run after updating leaflet)
