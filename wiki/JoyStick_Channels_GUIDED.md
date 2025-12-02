`RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED` is an enum constant representing a remote control mode for guided flight using joystick input.  
It enables velocity-based control of the drone in guided mode, where RC channel values are interpreted as movement commands rather than direct actuator outputs.

---

### Definition

```c++
102:105:drone_engage/drone_engage_mavlink/src/defines.hpp
// Velocity is sent for Thr, Pitch, Roll , YAWRate ... applicable in Arducopter and Rover
// Drone may switch {@link _7adath_FCB_RemoteControlSettings#RC_SUB_ACTION_JOYSTICK_CHANNELS} to this automatically if drone mode is guided.
RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED = 8
} RC_SUB_ACTION;
```

- **Type**: `enum` value within `RC_SUB_ACTION`
- **Value**: `8`
- **Purpose**: Indicates that joystick inputs should be interpreted as velocity commands in guided mode
- **Scope**: Used in remote control logic to determine how RC channel data is processed and sent to the flight controller

This value is part of a bitmask-style enumeration used to manage different remote control sub-modes in the drone’s RC override system. Unlike standard RC override (`RC_SUB_ACTION_JOYSTICK_CHANNELS = 4`), this mode triggers a different control path—specifically, sending velocity setpoints via `ctrlGuidedVelocityInLocalFrame` instead of raw RC channel overrides.

---

### Example Usages

The `RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED` mode is activated when the drone enters guided flight and joystick control is desired. It is used in conditional logic to switch between direct RC channel override and guided velocity control.

```c++
1371:1373:drone_engage/drone_engage_mavlink/src/fcb_main.cpp
m_andruav_vehicle_info.rc_sub_action =
    RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED;

m_fcb_facade.sendErrorMessage(..., std::string("RX Joystick Guided Mode"));
```

This snippet shows how the system explicitly sets the RC sub-action to guided joystick mode and notifies GCS (Ground Control Station) users via a warning message.

Another key usage occurs in the control dispatch logic:

```c++
1398:1404:drone_engage/drone_engage_mavlink/src/fcb_main.cpp
case RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS:
case RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED: {
  if (m_andruav_vehicle_info.flying_mode == VEHICLE_MODE_GUIDED) {
    enableRemoteControlGuided(); // Uses ctrlGuidedVelocityInLocalFrame
  } else {
    enableRemoteControl(); // Uses sendRCChannels()
  }
}
```

Here, both joystick modes are grouped, but behavior diverges based on current flight mode. If in `VEHICLE_MODE_GUIDED`, it calls `enableRemoteControlGuided()`, which internally uses velocity setpoints.

In the main control loop, timeout handling is also applied:

```c++
489:492:drone_engage/drone_engage_mavlink/src/fcb_main.cpp
case RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED: {
  if (now - m_andruav_vehicle_info.rc_command_last_update_time > RCCHANNEL_OVERRIDES_TIMEOUT) {
    releaseRemoteControl();
  }
}
```

This ensures safety: if no new joystick input arrives within `RCCHANNEL_OVERRIDES_TIMEOUT` (3 seconds), control is released and the drone may enter a safe mode (e.g., brake).

**Overall Usage Summary**:  
This symbol is used in 4 core functions across `fcb_main.cpp` and referenced in `fcb_tracker_logic.cpp`. It is central to the remote control override system, particularly during guided flight operations involving manual joystick input. Its usage spans initialization, runtime dispatch, and timeout handling.

---

### Notes

- Despite being named "joystick", this mode does **not** send joystick data directly. Instead, it maps joystick deflections to **velocity setpoints** in the local NED frame.
- The system may **automatically switch** to this mode from `RC_SUB_ACTION_JOYSTICK_CHANNELS` when entering guided flight, as noted in the comment: *"Drone may switch [...] to this automatically if drone mode is guided."*
- This mode sets `rc_command_active = true` and updates `rc_command_last_update_time` on each cycle, enabling timeout detection for safety.

---

### See Also

- `RC_SUB_ACTION_JOYSTICK_CHANNELS`: Base joystick control mode; uses direct RC channel override unless upgraded to guided.
- `VEHICLE_MODE_GUIDED`: Flight mode that allows external control via setpoints; required for this RC sub-action to take full effect.
- `ctrlGuidedVelocityInLocalFrame`: The actual control function used when this mode is active; sends velocity commands to the drone.
- `ANDRUAV_VEHICLE_INFO::rc_sub_action`: The struct field that holds the current RC sub-action state, including this value.
- `RCCHANNEL_OVERRIDES_TIMEOUT`: Timeout threshold (3 seconds) used to detect loss of joystick input and trigger safe disengagement.