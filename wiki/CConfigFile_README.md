# CConfigFile Configuration System Documentation

## Overview

`CConfigFile` is a singleton pattern implementation that manages JSON-based configuration for the DroneEngage MAVLink module. It provides centralized configuration management with file monitoring, dynamic updates, and backup capabilities.

## Class Architecture

### Singleton Pattern
- **Location**: `src/de_common/de_databus/configFile.hpp` and `configFile.cpp`
- **Pattern**: Scott Meyers singleton using static local instance
- **Thread Safety**: C++11 guarantees thread-safe initialization
- **Access Method**: `CConfigFile::getInstance()`

### Core Methods

#### Configuration Loading
- `initConfigFile(const char* fileURL)` - Initial configuration loading
- `reloadFile()` - Reload configuration from disk
- `GetConfigJSON()` - Get reference to parsed JSON object

#### File Monitoring
- `fileUpdated()` - Check if configuration file has been modified
- Monitors both file timestamps and programmatic updates

#### Dynamic Updates
- `updateJSON(const std::string& jsonString)` - Update configuration at runtime
- Supports nested key notation (e.g., "follow_me.quad.PID_P_X")
- `saveConfigFile()` - Persist changes with automatic backup

## Configuration Values Read by System

### 1. Main System Configuration (`main.cpp`)

#### Logger Settings
```json
{
    "logger_enabled": true,      // Enable/disable logging system
    "logger_debug": false        // Enable debug logging
}
```

#### Event Channels
```json
{
    "event_fire_channel": 16,     // Channel for firing events
    "event_wait_channel": 15      // Channel for waiting events
}
```

#### MAVLink Optimization
```json
{
    "default_optimization_level": 2,  // Default message optimization level
    "udp_proxy_enabled": true          // Enable UDP telemetry proxy
}
```

#### MAVLink IDs
```json
{
    "mavlink_ids": {
        "de_mavlink_gcs_id": 255,           // GCS system ID
        "only_allow_ardupilot_compid": 0,   // Component ID filter
        "only_allow_ardupilot_sysid": 0     // System ID filter
    }
}
```

#### Message Timeouts
Complex nested structure defining timeout intervals for different MAVLink message IDs:
```json
{
    "message_timeouts": {
        "0": [0, 1000, 2000, 3000],      // HEARTBEAT timeouts
        "1": [0, 250, 800, 1000],        // SYS_STATUS timeouts
        // ... many more message ID configurations
    }
}
```

### 2. FCB Main Configuration (`fcb_main.cpp`)

#### RC Channel Configuration
```json
{
    "rc_block_channel": -1,              // Channel to block (-1 = disabled)
    "rc_channels": {
        "rc_channel_enabled": [1, 1, 1, ...],     // 18 channel enable flags
        "rc_channel_reverse": [1, 1, 1, ...],     // 18 channel reverse flags  
        "rc_channel_limits_max": [2000, 2000, ...], // 18 channel max values
        "rc_channel_limits_min": [1000, 1000, ...], // 18 channel min values
        "rc_smart_channels": {
            "active": true,                           // Enable smart channels
            "rc_channel_enabled": [1, 1, 1, 1],      // Smart channel enables
            "rc_channel_limits_max": [2000, 2000, 2000, 2000],
            "rc_channel_limits_min": [1000, 1000, 1000, 1000]
        }
    }
}
```

#### Safety Mode
```json
{
    "read_only_mode": false            // Enable read-only safety mode
}
```

### 3. Tracking Configuration

#### Quadcopter Follow-Me (`fcb_tracker_logic_quad.cpp`)
```json
{
    "follow_me": {
        "quad": {
            "PID_P_X": 0.2,              // Proportional gain X-axis
            "PID_P_Y": 3.6,              // Proportional gain Y-axis  
            "PID_I_X": 0.0,              // Integral gain X-axis
            "PID_I_Y": 0.0,              // Integral gain Y-axis
            "PID_D_X": 0.01,             // Derivative gain X-axis
            "PID_D_Y": 0.05,             // Derivative gain Y-axis
            "expo_x": 0.0,               // X-axis exponential curve
            "expo_y": 0.0,               // Y-axis exponential curve
            "deadband_x": 0.001,         // X-axis deadband
            "deadband_y": 0.025,         // Y-axis deadband
            "center_hold_enabled": false,    // Enable center hold
            "center_hold_y_band": 0.05,     // Center hold Y-band
            "center_hold_decay": 0.1,       // Center hold decay rate
            "kalman_enabled": true,         // Enable Kalman filter
            "kalman_measurement_noise_r": 0.1, // Measurement noise
            "kalman_process_noise_q": 0.05,   // Process noise
            "rate_limit": 0.05,               // Rate limiting
            "loose_altitude": false            // Loose altitude mode
        }
    }
}
```

#### Plane Follow-Me (`fcb_tracker_logic_plan.cpp`)
```json
{
    "follow_me": {
        "plane": {
            "PID_P_X": 0.8,              // Proportional gain X-axis
            "PID_P_Y": 0.6,              // Proportional gain Y-axis
            "PID_I_X": 0.0,              // Integral gain X-axis  
            "PID_I_Y": 0.1,              // Integral gain Y-axis
            "PID_D_X": 0.1,              // Derivative gain X-axis
            "PID_D_Y": 0.5,              // Derivative gain Y-axis
            // ... similar control parameters as quad
        }
    }
}
```

### 4. Network Configuration
```json
{
    "fcb_connection_uri": {
        "ip": "0.0.0.0",              // FCB connection IP
        "port": 7660,                 // FCB connection port
        "type": "tcp"                 // Connection type
    },
    "s2s_udp_listening_ip": "127.0.0.1",     // UDP listening IP
    "s2s_udp_listening_port": "61003",       // UDP listening port
    "s2s_udp_packet_size": "8192",           // UDP packet size
    "s2s_udp_target_ip": "127.0.0.1",        // UDP target IP
    "s2s_udp_target_port": "60000"           // UDP target port
}
```

### 5. Module Identification
```json
{
    "module_id": "FCB_CTRL"          // Module identifier
}
```

## Usage Patterns in Code

### Standard Access Pattern
```cpp
de::CConfigFile &cConfigFile = CConfigFile::getInstance();
const Json_de &jsonConfig = cConfigFile.GetConfigJSON();

if (jsonConfig.contains("parameter_name")) {
    auto value = jsonConfig["parameter_name"].get<type>();
}
```

### File Update Monitoring
```cpp
const bool updated = cConfigFile.fileUpdated();
if (updated) {
    cConfigFile.reloadFile();
    // Re-read configuration values
}
```

### Dynamic Configuration Updates
```cpp
// Runtime configuration update via message parser
cConfigFile.updateJSON(config.dump());
```

## File Management Features

### Automatic Backup
- Creates timestamped backups before saving: `filename.bak_YYYYMMDD_HHMMSS`
- Preserves configuration history

### File Monitoring
- Checks file modification timestamps
- Supports both file-based and programmatic updates
- Thread-safe update detection

### Error Handling
- Graceful handling of missing configuration files
- Default value fallbacks for critical parameters
- Comprehensive error logging with colored console output

## Configuration File Locations

### Primary Configuration
- **Main**: `de_mavlink.config.module.json`

## Integration Points

### Message Parser Integration
The configuration system integrates with the message parser (`de_message_parser_base.cpp`) to allow remote configuration updates via MAVLink messages.

### Cross-Module Usage
- **FCB Main**: Primary consumer of RC and system configuration
- **Tracking Modules**: Consume follow-me PID parameters
- **Logger System**: Uses logger configuration
- **Mission Manager**: Uses event channel configuration

## Best Practices

1. **Always check existence** of configuration keys before access
2. **Use appropriate type casting** with `.get<type>()`
3. **Handle missing configurations** with sensible defaults
4. **Monitor file updates** in long-running processes
5. **Use nested key notation** for dynamic updates to complex structures

## Thread Safety

- Singleton initialization is thread-safe (C++11 guarantee)
- File operations use filesystem-level synchronization
- Update operations set pending flags for atomic detection

This configuration system provides a robust, flexible foundation for managing the complex parameter space required by the DroneEngage MAVLink module.
