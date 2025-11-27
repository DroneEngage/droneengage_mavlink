# CVehicle Class

`mavlinksdk::CVehicle` is the central class for managing vehicle state and telemetry data in the MAVLink SDK. It automatically parses incoming MAVLink messages, maintains the current vehicle state, and provides access to all telemetry data.

---

## Singleton Pattern

`CVehicle` is implemented as a **singleton** - only one instance exists throughout the application lifetime. This ensures all components share the same vehicle state.

### Accessing the Instance

```cpp
mavlinksdk::CVehicle& vehicle = mavlinksdk::CVehicle::getInstance();
```

### Singleton Enforcement

The constructor is **private**, and copy/assignment are **deleted**:

```cpp
// vehicle.h
static CVehicle& getInstance()
{
    static CVehicle instance;
    return instance;
}

CVehicle(CVehicle const&)               = delete;
void operator=(CVehicle const&)         = delete;

private:
    CVehicle();  // Private constructor
```

This guarantees:
- Only one `CVehicle` instance exists
- All classes share the same vehicle state
- Thread-safe initialization (C++11 static local)

---

## How CVehicle is Accessed by Other Classes

### 1. CMavlinkSDK - Routes Messages to CVehicle

The main SDK class forwards all received MAVLink messages to CVehicle for parsing:

```cpp
// mavlink_sdk.cpp
void CMavlinkSDK::OnMessageReceived(const mavlink_message_t &mavlink_message)
{
    mavlinksdk::CVehicle::getInstance().parseMessage(mavlink_message);
    this->m_mavlink_events->OnMessageReceived(mavlink_message);
}
```

### 2. CMavlinkCommand - Uses Vehicle State for Commands

The command class holds a reference to CVehicle to access system IDs and current state when sending commands:

```cpp
// mavlink_command.h
class CMavlinkCommand
{
private:
    mavlinksdk::CVehicle &m_vehicle = mavlinksdk::CVehicle::getInstance();
    
    // Uses m_vehicle.getSysId(), m_vehicle.getCompId() for targeting
    // Uses m_vehicle.getMsgGlobalPositionInt() for position-based commands
};
```

### 3. External User Code - Access Telemetry

Any code in the application can access the singleton to read vehicle state:

```cpp
// User application code
void checkVehicleStatus()
{
    mavlinksdk::CVehicle& vehicle = mavlinksdk::CVehicle::getInstance();
    
    if (vehicle.isFCBConnected()) {
        auto pos = vehicle.getMsgGlobalPositionInt();
        bool armed = vehicle.isArmed();
        bool flying = vehicle.isFlying();
        
        std::cout << "Position: " << pos.lat / 1e7 << ", " << pos.lon / 1e7 << std::endl;
        std::cout << "Armed: " << armed << ", Flying: " << flying << std::endl;
    }
}
```

---

## Class Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      CMavlinkSDK                            │
│  - Receives raw MAVLink messages from communication layer   │
│  - Calls CVehicle::getInstance().parseMessage()             │
└─────────────────────┬───────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────┐
│                       CVehicle (Singleton)                  │
│  - parseMessage() decodes and stores all telemetry          │
│  - Maintains cached copies of all MAVLink messages          │
│  - Detects state changes (arm, mode, flying)                │
│  - Triggers callbacks via CCallBack_Vehicle                 │
└─────────────────────┬───────────────────────────────────────┘
                      │
          ┌───────────┴───────────┐
          ▼                       ▼
┌─────────────────────┐  ┌─────────────────────┐
│   CMavlinkCommand   │  │   User Application  │
│  - Reads sysid/     │  │  - Reads telemetry  │
│    compid           │  │  - Checks state     │
│  - Gets position    │  │  - Monitors health  │
│    for commands     │  │                     │
└─────────────────────┘  └─────────────────────┘
```

---

## Key Methods

### State Detection

| Method | Description |
|--------|-------------|
| `isFCBConnected()` | Returns `true` if heartbeat received within last 3 seconds |
| `isArmed()` | Returns `true` if vehicle is armed |
| `isFlying()` | Returns `true` if vehicle is in flight |
| `isReadyToArm()` | Returns `true` if pre-arm checks passed |
| `isMotorEnabled()` | Returns `true` if motors are enabled and healthy |

### Identity

| Method | Description |
|--------|-------------|
| `getSysId()` | Get MAVLink system ID of connected vehicle |
| `getCompId()` | Get MAVLink component ID of connected vehicle |
| `restrictMessageToSysID(id)` | Filter messages to specific system ID |
| `restrictMessageToCompID(id)` | Filter messages to specific component ID |

### Telemetry Accessors

| Method | Returns |
|--------|---------|
| `getMsgHeartBeat()` | `mavlink_heartbeat_t` |
| `getMsgGlobalPositionInt()` | `mavlink_global_position_int_t` |
| `getMsgAttitude()` | `mavlink_attitude_t` |
| `getMsgSysStatus()` | `mavlink_sys_status_t` |
| `getMsgVFRHud()` | `mavlink_vfr_hud_t` |
| `getMsgHomePosition()` | `mavlink_home_position_t` |
| `getMSGGPSRaw()` | `mavlink_gps_raw_int_t` |
| `getRCChannels()` | `mavlink_rc_channels_t` |
| `getServoOutputRaw()` | `mavlink_servo_output_raw_t` |

---

## See Also

- [Vehicle_ParseMessage.md](Vehicle_ParseMessage.md) - Details on message parsing
- `CCallBack_Vehicle` - Callback interface for state change notifications
- `CMavlinkCommand` - Command API that uses CVehicle state
