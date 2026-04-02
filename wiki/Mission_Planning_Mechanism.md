# DroneEngage Mission Planning Mechanism

## Overview

The DroneEngage mission planning system implements a sophisticated two-layer architecture that enables complex autonomous drone operations with coordinated multi-module actions and event-driven mission flows. This document provides a comprehensive analysis of the mission planning mechanism across both the communication and MAVLink modules.

## Architecture Overview

### Two-Layer Design

1. **High-Level Mission Planning** (`de_general_mission_planner`)
   - Cross-module mission coordination
   - Event-driven command execution
   - Mission file parsing and management

2. **Low-Level Mission Execution** (`mission`)
   - Ardupilot board mission management
   - Real-time mission synchronization
   - Event communication via servo channels

### Core Components

```
┌─────────────────────────────────┐
│   Communication Module          │
│  CMissionManagerBase            │
│  ├─ extractPlanModule()         │
│  ├─ fireWaitingCommands()       │
│  └─ getCommandsAttached...()    │
└─────────────┬───────────────────┘
              │ JSON Mission Files
              │ Event Coordination
┌─────────────▼───────────────────┐
│      MAVLink Module             │
│     CMissionManager             │
│  ├─ uploadMissionIntoSystem()   │
│  ├─ extractPlanMavlinkMission() │
│  ├─ deEventFiredExternally()    │
│  └─ processMyWaitingEvent()     │
└─────────────┬───────────────────┘
              │ MAVLink Protocol
              │ Servo Channel Events
┌─────────────▼───────────────────┐
│      Ardupilot Board            │
│    Mission Execution            │
│    Event Signaling              │
└─────────────────────────────────┘
```

## Mission File Structure

### DE Plan Format

The system uses a comprehensive JSON-based mission file format:

```json
{
  "fileType": "de_plan",
  "unit": {
    "home": {
      "lat": 30.123456,
      "lng": 31.654321,
      "alt": 100
    }
  },
  "de_mission": {
    "modules": [
      {
        "c": [
          {
            "mt": 1001,
            "ty": "uv",
            "GU": "CAMERA_MODULE",
            "params": {...}
          }
        ],
        "ls": "1",
        "ew": "event_001"
      }
    ],
    "mav_waypoints": [
      {
        "mv": [15, 16, 0, 0, 30.123456, 31.654321, 100],
        "ft": 3,
        "c": 16
      }
    ]
  }
}
```

### Field Definitions

| Field | Purpose | Example |
|-------|---------|---------|
| `fileType` | Mission file type identifier | `"de_plan"` |
| `unit.home` | Home position coordinates | `{lat, lng, alt}` |
| `modules` | Array of module mission items | See below |
| `mav_waypoints` | Ardupilot waypoint definitions | See below |
| `c` | Command array for module | `[{"mt": 1001, ...}]` |
| `ls` | Linked to step (mission ID) | `"1"` |
| `ew` | Waiting event ID | `"event_001"` |
| `mv` | MAVLink parameters | `[p1, p2, p3, p4, x, y, z]` |
| `ft` | Frame type | `3` (GLOBAL_RELATIVE_ALT) |
| `c` | MAVLink command | `16` (WAYPOINT) |

## High-Level Mission Management

### CMissionManagerBase Class

The base mission manager provides the foundation for cross-module coordination:

#### Key Data Structures

```cpp
std::map<std::string, std::vector<Json_de>> m_module_missions;
std::map<std::string, std::vector<Json_de>> m_module_missions_by_de_events;
std::string m_last_executed_mission_id;
```

#### Core Functions

**extractPlanModule()**
- Parses JSON mission files
- Extracts module-specific commands
- Links commands to MAVLink waypoints
- Sets up event waiting relationships

**fireWaitingCommands()**
- Executes commands waiting for specific events
- Routes commands to appropriate modules
- Handles inter-module communication

**getCommandsAttachedToMavlinkMission()**
- Triggers module commands when MAVLink missions reach specific waypoints
- Prevents duplicate command execution
- Maintains mission state synchronization

### Event-Driven Command Execution

The system supports two types of event-driven execution:

1. **Waiting Events** (`ew` field)
   - Commands wait for specific events before executing
   - Events can be fired by other modules or external systems

2. **Linked Steps** (`ls` field)
   - Commands execute when specific MAVLink waypoints are reached
   - Enables precise timing of module actions

## Low-Level Mission Execution

### CMissionManager Class

The MAVLink mission manager extends the base class to handle Ardupilot-specific operations:

#### Event Synchronization System

The most innovative aspect of the system is its use of servo channels for event communication:

```cpp
int m_event_fire_channel;    // Channel for firing events (e.g., 16)
int m_event_wait_channel;    // Channel for waiting events (e.g., 15)
```

**Event Flow:**
1. Ardupilot sets servo PWM value on fire channel
2. Companion computer detects change via `readFiredEventFromFCB()`
3. Event is broadcast to other modules/agents
4. Other modules respond with their own events
5. Ardupilot monitors wait channel via `readWaitingEventFromFCB()`
6. Mission continues when expected event is received

#### Mission Upload Process

```cpp
uploadMissionIntoSystem2(plan_text)
    ↓
extractPlanMavlinkMission() + extractPlanModule()
    ↓
saveWayPointsToFCB()
    ↓
Ardupilot executes mission
```

### Mission Item Classes

The system implements a comprehensive hierarchy of mission item types:

#### Navigation Commands
- `CWayPoint_Step` - Standard waypoint navigation
- `CTakeOff_Step` - Takeoff procedures
- `CLand_Step` - Landing operations
- `CRTL_Step` - Return to launch
- `CLoiter_Turns_Step` - Circular loiter patterns

#### Action Commands
- `CChange_Speed_Step` - Speed adjustments
- `CChange_Altitude_Step` - Altitude changes
- `CChange_Heading_Step` - Heading control
- `CGuided_Enabled_Step` - Guided mode control

#### Timing Commands
- `CDelay_Step` - Time-based delays
- `CDelay_State_Machine_Step` - State machine delays

#### Camera Commands
- `CCameraTrigger_Step` - Camera triggering
- `CCameraControl_Step` - Camera parameter control

## Mission Translation System

### CMissionTranslator

Supports multiple mission file formats:

1. **QGC Format** - JSON-based files from QGroundControl
2. **Mission Planner Format** - Tab-separated text files
3. **DE Plan Format** - Custom DroneEngage JSON format

### Mission Item Builder

Factory pattern implementation:

```cpp
CMissionItem *getClassByMavlinkCMD(const mavlink_mission_item_int_t& mission_item_int)
{
    switch (mission_item_int.command) {
        case MAV_CMD_NAV_WAYPOINT: return new CWayPoint_Step();
        case MAV_CMD_NAV_TAKEOFF: return new CTakeOff_Step();
        case MAV_CMD_DO_CHANGE_SPEED: return new CChange_Speed_Step();
        // ... more cases
        default: return new CDummy_Step();
    }
}
```

## Event System Architecture

### Event Types

1. **Fire Events** (`ef`)
   - Initiated by Ardupilot via servo channels
   - Broadcast to all system components
   - Used to signal mission milestones

2. **Wait Events** (`ew`)
   - Monitored by Ardupilot via servo channels
   - Can be fired by any module or external system
   - Used to synchronize mission execution

### Event Communication Protocol

```
Ardupilot → Servo Channel (fire) → Companion Computer → 
Event Broadcast → Other Modules → Response Events → 
Servo Channel (wait) → Ardupilot → Mission Continue
```

### Event Handling Functions

**readFiredEventFromFCB()**
- Monitors servo output for fire events
- Extracts event values from PWM signals
- Broadcasts events to system

**readWaitingEventFromFCB()**
- Monitors servo output for wait events
- Updates current waiting event state
- Triggers event processing

**processMyWaitingEvent()**
- Checks if waited-for event has been received
- Advances mission to next waypoint
- Handles event cleanup

## Mission Execution Flow

### Complete Mission Lifecycle

1. **Mission Upload**
   ```
   JSON Plan → extractPlanModule() → extractPlanMavlinkMission() → 
   saveWayPointsToFCB() → Ardupilot Mission Upload
   ```

2. **Mission Execution**
   ```
   Ardupilot executes waypoints → Servo events → 
   Event detection → Module commands → Response events → 
   Mission synchronization
   ```

3. **Event-Driven Coordination**
   ```
   Waypoint reached → getCommandsAttachedToMavlinkMission() → 
   Module command execution → Event firing → 
   Other modules respond → Mission continue
   ```

### Synchronization Mechanisms

#### MAVLink Mission Progress
- Real-time tracking of current waypoint
- Automatic triggering of linked module commands
- Prevention of duplicate command execution

#### Event-Based Coordination
- Reliable event communication via servo channels
- Support for complex multi-agent scenarios
- Configurable event channels and timeouts

#### Module Integration
- Seamless routing of commands to appropriate modules
- Inter-module communication support
- Event-driven command execution

## Configuration and Customization

### Servo Channel Configuration

```json
{
  "event_fire_channel": 16,
  "event_wait_channel": 15
}
```

### Mission File Management

- Automatic backup of mission files
- Real-time file monitoring for updates
- Support for multiple mission formats

### Module Registration

Modules register themselves to receive mission commands:
```cpp
de::comm::CUavosModulesManager::getInstance().processIncommingServerMessage(
    sender, command_type, cmd_text.c_str(), cmd_text.length(), std::string()
);
```

## Advanced Features

### Multi-Drone Coordination

The event system enables sophisticated multi-drone operations:
- Event broadcasting to all agents
- Coordinated mission execution
- Inter-drone synchronization

### State Machine Integration

Special delay commands support state machine patterns:
- `CDelay_State_Machine_Step` for state-based delays
- Event-driven state transitions
- Complex mission logic implementation

### Error Recovery

The system includes basic error handling:
- Mission retry mechanisms
- Event timeout handling
- Graceful degradation scenarios

## Best Practices

### Mission Design

1. **Plan Event Flow** - Design event sequences carefully
2. **Use Unique Event IDs** - Avoid event ID conflicts
3. **Set Appropriate Timeouts** - Prevent mission deadlocks
4. **Test Module Integration** - Verify command routing

### Configuration

1. **Choose Servo Channels** - Use non-critical servo channels
2. **Monitor Event System** - Log event flows for debugging
3. **Validate Mission Files** - Check JSON syntax and structure

### Development

1. **Follow Factory Pattern** - Use mission item builder for new types
2. **Implement Event Handlers** - Add proper event processing
3. **Test Synchronization** - Verify event timing and coordination

## Troubleshooting

### Common Issues

1. **Event Not Received**
   - Check servo channel configuration
   - Verify event ID matching
   - Monitor servo output signals

2. **Mission Stuck**
   - Check for missing events
   - Verify event waiting logic
   - Review timeout configurations

3. **Module Commands Not Executed**
   - Verify module registration
   - Check command routing
   - Review message parsing

### Debugging Tools

1. **Event Logging** - Monitor event flow in system logs
2. **Mission State** - Track current mission progress
3. **Servo Monitoring** - Observe servo channel values

## Future Enhancements

### Recommended Improvements

1. **Enhanced Error Recovery**
   - Mission rollback capabilities
   - Automatic retry mechanisms
   - Graceful failure handling

2. **Advanced Event System**
   - Event acknowledgment protocol
   - Event sequence numbering
   - CRC validation for events

3. **Mission Validation**
   - Pre-flight validation checks
   - Runtime consistency verification
   - Parameter boundary checking

4. **Performance Optimization**
   - Event queue management
   - Memory usage optimization
   - Real-time performance improvements

## Conclusion

The DroneEngage mission planning mechanism represents a sophisticated approach to autonomous drone operations. Its two-layer architecture, event-driven coordination, and comprehensive mission item support enable complex multi-module and multi-drone scenarios with reliable synchronization and flexible mission design.

The innovative use of servo channels for event communication provides a robust physical layer for coordination between the flight controller and companion computer, while the JSON-based mission file format offers maximum flexibility for mission designers.

This architecture successfully bridges the gap between high-level mission planning and low-level flight control execution, providing a solid foundation for advanced autonomous drone operations.

---

*This document provides a comprehensive overview of the DroneEngage mission planning mechanism. For specific implementation details, refer to the source code in the respective modules.*
