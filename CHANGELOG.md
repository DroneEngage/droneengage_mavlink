# Changelog

Major changes to `de_mavlink` — the DroneEngage MAVLink bridge to ArduPilot/PX4. Newest first.

## [7.4.5] - 2026-09

- Regenerated the bundled `c_library_v2` MAVLink library from the latest message definitions.
- `LANDING_TARGET` now uses `MAV_FRAME_BODY_FRD` and computes slant range correctly.
- Fixed remaining elapsed-time checks to use the monotonic clock.
- Hardened the traffic optimizer: mutex-protected shared state, malformed `message_timeouts` entries are skipped instead of crashing, and negative config values are clamped.

## [7.4.4] - 2026-09-11

- Added module-specific memory health thresholds.
- Added the precision-landing safety gate: `PRECLAND_TARGET`/`STATUS` messages are converted to MAVLink `LANDING_TARGET` only behind an armed/mode/freshness gate.

## [7.4.1–7.4.3] - 2026-09

- `de_common` submodule updates (PRECLAND defines, socket-directory fallback, parse-error reporting).

## [7.3.0] - 2026-08-29

- Migrated the `follow_me` config to the `tracking.quad`/`tracking.plane` structure.

## [7.2.x] - 2026-08

- `de_common` submodule updates (module health monitoring, local overrides).

## [7.1.x] - 2026-08-22

- Wired in the localConfig override layer (`*.local` files override the module config).

## [7.0.0] - 2026-04

- CMake minimum raised to 3.10.
- Arrow formation: dynamic side assignment and bearing stabilization with smoothed target positioning.
- Added geofence zone-transition events with action, type, and distance.
- Added automatic `s2s_udp_packet_size` detection.

## [6.x / 5.x] - 2025–2026

- Added geofence (fence) validation and inter-module remote-execute API.
- Added swarm support and VTOL modes.
- Added smart RC channel mapping (RCMAP), including PX4 support.
- Added autopilot parameter get-by-name and the autopilot field in the ID message.
- Auto-incrementing build version; builds moved to CMake only.

## [Earlier] - initial releases

- Mission upload/download compatible with QGroundControl and Mission Planner files.
- Guided-mode points, joystick/RC remote control, and acro mode.
- Telemetry and binary messages over the Andruav protocol; MAVLink v2 support.
- First working FCB-to-WebClient interface: arm/disarm, mode changes, takeoff, GPS and battery reporting.
