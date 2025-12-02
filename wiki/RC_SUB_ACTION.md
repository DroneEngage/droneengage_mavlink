`RC_SUB_ACTION` is an enumeration that defines the operational state of remote control (RC) channel input handling in a drone control system.  
It determines how RC commands from a transmitter (TX) are processed and whether they are passed through, modified, or overridden by software.

This enum is used to manage the autonomy level of RC input—ranging from fully released (no input sent) to active joystick control (direct channel override)—and plays a key role in safety and automation logic, especially during mode transitions like switching to guided flight.

---

### Definition

```cpp
92:105:drone_engage/drone_engage_mavlink/src/defines.hpp
typedef enum RC_SUB_ACTION 
{
    // No RC channel data is sent to the drone.
    RC_SUB_ACTION_RELEASED                      =   0,
    
    // Center values (1500 µs) are sent; physical TX has no effect.
    RC_SUB_ACTION_CENTER_CHANNELS               =   1,
    
    // Last known RC values are frozen and continuously sent.
    RC_SUB_ACTION_FREEZE_CHANNELS               =   2,
    
    // Live RC channel values from software (e.g. app) are sent.
    RC_SUB_ACTION_JOYSTICK_CHANNELS             =   4,
    
    // Velocity commands sent instead of raw RC; used in guided mode.
    RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED      =   8
} RC_SUB_ACTION;
```

- **Type**: `enum` (C++ scoped enumeration)
- **Purpose**: Controls how RC input is applied to the drone’s flight controller
- **Used in**: `ANDRUAV_VEHICLE_INFO::rc_sub_action` to reflect current RC override state
- **Values are powers of two or sequential integers**, suggesting possible bitmask use or simple state tracking
- **Initial value**: Set to `RC_SUB_ACTION_RELEASED` during initialization

The enum is tightly coupled with RC override logic, where physical transmitter input can be disabled in favor of software-generated commands—critical for autonomous or app-controlled flight.

---

### Example Usages

One primary use is initializing the RC state when the system starts:

```cpp
249:251:drone_engage/drone_engage_mavlink/src/fcb_main.cpp
de::fcb::mission::CMissionManager::getInstance().getAndruavMission().clear();

// Initialize RC override state to released (no input sent)
m_andruav_vehicle_info.rc_sub_action = RC_SUB_ACTION::RC_SUB_ACTION_RELEASED;
```

Another key usage is in parsing incoming commands from a remote source (e.g. mobile app):

```cpp
420:422:drone_engage/drone_engage_mavlink/src/fcb_andruav_message_parser.cpp
int rc_sub_action = cmd["b"].get<int>();

// Interpret integer command as RC_SUB_ACTION enum
m_fcbMain.adjustRemoteJoystickByMode((RC_SUB_ACTION)rc_sub_action);
```

Here, the JSON field `"b"` carries the desired RC mode, which is cast directly into the `RC_SUB_ACTION` enum and dispatched to control logic.

**Overall usage pattern**:  
`RC_SUB_ACTION` is used across 12 files, primarily in:
- `defines.hpp`: Definition and vehicle state struct
- `fcb_main.cpp`: State machine logic for RC handling
- `fcb_andruav_message_parser.cpp`: Incoming command parsing
- `fcb_main.hpp`: Method signature for `adjustRemoteJoystickByMode()`

It is central to **remote control override systems**, especially in scenarios involving app-based joystick control, geofencing, or autonomous mission execution.

---

### Notes

- `RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED` is specifically intended for **guided mode flight**, where velocity commands (not raw PWM) are sent—this enables smoother autonomous maneuvers.
- The enum values skip `3` and `5–7`, suggesting possible future extensions or alignment with bitmask-style flags (though not currently used as bitflags).
- Despite being called "joystick", this system does **not require a physical joystick**—it refers to software-emulated RC inputs from apps or tracking systems.

---

### See Also

- `ANDRUAV_VEHICLE_INFO`: Struct that holds `rc_sub_action` as part of the drone's state; used for telemetry and control decisions.
- `CFCBMain::adjustRemoteJoystickByMode()`: Method that acts on `RC_SUB_ACTION` values to change RC behavior.
- `RCCHANNEL_OVERRIDES_TIMEOUT`: A related define (`3000000` µs) that may govern how long overrides remain active before timing out.
- `mavlink::sendRCChannels()`: Function likely called when sending RC data in `CENTER`, `FREEZE`, or `JOYSTICK` modes.