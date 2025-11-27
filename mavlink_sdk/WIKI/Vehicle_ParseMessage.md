`mavlinksdk::CVehicle::parseMessage` is a method that processes incoming MAVLink messages in a vehicle communication system.  
It decodes and routes MAVLink protocol messages to appropriate internal handlers based on message type.

---

### Definition

The `parseMessage` function is a core part of the `mavlinksdk::CVehicle` class, responsible for interpreting raw MAVLink messages received from a drone or ground station. It uses a switch statement on the message ID to dispatch handling logic to specific methods, updating internal state and triggering callbacks.

```cpp
167:167:/mnt/8a619ce7-cd3f-4520-af65-7991f16410f7/public_versions/drone_engage/drone_engage_mavlink/mavlink_sdk/src/vehicle.h
void parseMessage (const mavlink_message_t& mavlink_message);
```

```cpp
419:813:/mnt/8a619ce7-cd3f-4520-af65-7991f16410f7/public_versions/drone_engage/drone_engage_mavlink/mavlink_sdk/src/vehicle.cpp
void mavlinksdk::CVehicle::parseMessage(const mavlink_message_t& mavlink_message)
{
    const u_int32_t msgid = mavlink_message.msgid;
    mavlink_message_temp = mavlink_message;

    switch (mavlink_message.msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&mavlink_message, &heartbeat);
            if (handle_heart_beat(heartbeat)) { /* update state */ }
        }
        break;

        case MAVLINK_MSG_ID_DISTANCE_SENSOR:
        {
            mavlink_distance_sensor_t distance_sensor;
            mavlink_msg_distance_sensor_decode(&mavlink_message, &distance_sensor);
            time_stamps.setTimestamp(msgid, get_time_usec());
            handle_distance_sensor(distance_sensor);
            return;
        }
        break;

        case MAVLINK_MSG_ID_VIBRATION:
        {
            mavlink_vibration_t vibration;
            mavlink_msg_vibration_decode(&mavlink_message, &vibration);
            time_stamps.setTimestamp(msgid, get_time_usec());
            handle_vibration_report(vibration);
            return;
        }
        break;

        // ... dozens of other message types handled similarly ...

        case MAVLINK_MSG_ID_GPS_RAW_INT:
        {
            mavlink_msg_gps_raw_int_decode(&mavlink_message, &(m_gps_raw_int));
            exit_high_latency();
        }
        // Note: no break — falls through to next case

        case MAVLINK_MSG_ID_GPS2_RAW:
        {
            mavlink_msg_gps2_raw_decode(&mavlink_message, &(m_gps2_raw));
            exit_high_latency();
        }
        break;

        default:
        {
            // Unhandled message IDs
        }
        break;
    }

    // Update timestamp for all messages (even unhandled ones)
    time_stamps.setTimestamp(msgid, get_time_usec());
}
```

- **Params**: `const mavlink_message_t& mavlink_message` — Reference to a decoded MAVLink message structure containing message ID, payload, and metadata.
- **Side effects**: Updates internal vehicle state (e.g. GPS, sensors, system status), invokes registered callbacks via `m_callback_vehicle`, and records message timestamps using `time_stamps.setTimestamp()`.
- **Returns**: `void` — No return value; all processing is done through state mutation and event callbacks.

Note: The function stores a copy of the message in `mavlink_message_temp` for debugging or inspection purposes.

---

### Example Usages

While no direct caller is shown in the provided context, `parseMessage` is designed to be invoked by a communication loop that receives serialized MAVLink data, decodes it with `mavlink_parse_char`, and forwards completed messages.

A typical usage pattern (inferred from standard MAVLink SDK design) would be:

```cpp
// Pseudocode: How parseMessage is likely used in a receiver thread
mavlink_status_t status;
mavlink_message_t msg;

for (uint8_t c : received_bytes) {
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        vehicle.parseMessage(msg);  // <-- This is where it's called
    }
}
```

Within the codebase:
- `parseMessage` is defined in `vehicle.cpp` and declared in `vehicle.h`.
- It is referenced in header and source files but no explicit callers are listed in the usage context.
- It handles over 50 different MAVLink message types, making it one of the central dispatchers in the system.

Its widespread switch-case structure indicates it's a critical integration point between low-level communication and high-level vehicle state management.

---

### Notes

- **Fall-through logic**: The cases for `MAVLINK_MSG_ID_GPS_RAW_INT` and `MAVLINK_MSG_ID_GPS2_RAW` intentionally lack a `break`, allowing both to trigger `exit_high_latency()` — this suggests GPS messages collectively signal a return from low-bandwidth operation mode.
- **High-latency handling**: Messages like `HIGH_LATENCY` and `HIGH_LATENCY2` are stored in member fields (`m_high_latency`, `m_high_latency2`) and trigger synthetic heartbeat updates via `handle_high_latency`, which simulates normal operation during poor connectivity.
- **Timestamp ordering**: The final `time_stamps.setTimestamp()` call occurs after the switch, ensuring even unhandled messages are timestamped — critical for detecting stale data.

---

### See Also

- `mavlink_message_t`: The core MAVLink message struct from the MAVLink C library; contains `msgid`, payload, and CRC fields.
- `handle_heart_beat`: Processes heartbeat messages to update vehicle mode, autopilot type, and connection health.
- `m_callback_vehicle`: Pointer to a `mavlinksdk::CCallBack_Vehicle` instance; used to notify external code of state changes (e.g. `OnVibrationChanged`, `OnDistanceSensorChanged`).
- `time_stamps`: Tracks when each message type was last received, used to detect delays or loss.
- `exit_high_latency`: Called when real-time messages (like GPS or position) arrive, indicating the system can leave degraded communication mode.