# AGENTS.md — drone_engage_mavlink

DroneEngage MAVLink bridge (`de_mavlink`, binary `de_ardupilot`). The most
important plugin — bridges ArduPilot/PX4 flight controllers to the
DroneEngage bus via MAVLink. C++17. See parent `../AGENTS.md` for
workspace architecture, `de_common` vendoring, and config conventions.

## Build

    ./build.sh                 # DEBUG
    ./build_release.sh         # RELEASE
    ./build_ddebug.sh          # DEBUG + DDEBUG=ON

Out-of-source in `build/`. Binary: `bin/de_ardupilot`.

### CMake options

- `DDEBUG` — detailed debug.
- Auto-increment build number from `.version` on RELEASE
  (MAJOR.MINOR.BUGFIX.BUILD; current 7.1.0). Bump via `MAJOR_VERSION`/
  `MINOR_VERSION`/`BUGFIX_VERSION` in `CMakeLists.txt`.
- CPack Debian packaging.

### Dependencies

OpenSSL, Threads, plog (`3rdparty`). MAVLink headers via
`c_library_v2/` and `mavlink_sdk/`. `3rdparty/` holds vendored deps.

## Run / sim

    ./bin/de_ardupilot -c ./de_mavlink.config.module.json
    ./sim_drone.sh             # SITL drone
    ./sim_plane.sh             # SITL plane
    ./sim_mix.sh               # mixed SITL
    ./runSITLPlane.sh          # SITL plane launcher
    ./docker.run.sh            # containerized run
    ./redirect.traffic.sh      # traffic redirect helper

`deployment/` has parent/slave multi-unit configs and start scripts.

## Config

- `de_mavlink.config.module.json` — module config (WebClient UI);
  `de_mavlink.config.module.org.json` is the original/template copy.
- `template.json` — UI schema groups.
- `de_mavlink.local` — instance identity.

## Source layout

`src/` — `main.cpp`, `fcb_main.{cpp,hpp}` (flight controller bridge),
`fcb_facade.{cpp,hpp}`, `fcb_andruav_message_parser.{cpp,hpp}`,
`fcb_modes.{cpp,hpp}`, `fcb_traffic_optimizer.{cpp,hpp}`,
`de_pilot/`, `de_general_mission_planner/`, `geofence/`, `mission/`,
`helpers/`, `genericClientPort.h`, `de_common/` (vendored).
`ParameterList.param` / `RealCobatlDroneParameter.param` — MAVLink param
files. `eeprom.bin` — SITL EEPROM. `gitcommit.sh` — git info stamping.
