# MAVLink SDK for ArduPilot (C++)

A lightweight, standalone C++17 MAVLink SDK for communicating with ArduPilot-based autopilots. This SDK provides a clean, event-driven API for connecting to drones via UDP, TCP, or Serial interfaces.

## Features

- **Multiple Connection Types**: UDP, TCP, and Serial port support
- **Event-Driven Architecture**: Callback-based notifications for vehicle state changes
- **Vehicle State Management**: Automatic parsing and storage of MAVLink messages
- **Mission Management**: Upload, download, and monitor waypoint missions
- **Parameter Management**: Read and write autopilot parameters
- **Command Interface**: High-level API for common drone operations
- **C++17**: Modern C++ with smart pointers and RAII patterns
- **Singleton Pattern**: Easy access to SDK components
- **ArduPilot & PX4 Support**: Compatible with both autopilot firmwares

## Project Structure

```
mavlink_sdk/
├── src/
│   ├── mavlink_sdk.h/cpp        # Main SDK entry point
│   ├── vehicle.h/cpp            # Vehicle state and telemetry
│   ├── mavlink_command.h/cpp    # Command API for vehicle control
│   ├── mavlink_events.h         # Event callback interface
│   ├── mavlink_waypoint_manager.h/cpp    # Mission management
│   ├── mavlink_parameter_manager.h/cpp   # Parameter management
│   ├── mavlink_communicator.h/cpp        # Message routing
│   ├── mavlink_helper.h/cpp     # Utility functions
│   ├── generic_port.h           # Abstract port interface
│   ├── udp_port.h/cpp           # UDP communication
│   ├── tcp_client_port.h/cpp    # TCP communication
│   ├── serial_port.h/cpp        # Serial communication
│   └── helpers/
│       ├── colors.h             # Console color definitions
│       └── utils.h              # Utility helpers
├── CMakeLists.txt               # CMake build configuration
└── build.sh                     # Build script
```

## Requirements

- **C++17** compatible compiler (GCC 7+ or Clang 5+)
- **CMake** 3.1.0 or higher
- **MAVLink C library** (c_library_v2) - should be placed at `../c_library_v2` relative to the SDK

## Building

### Using the Build Script

```bash
cd mavlink_sdk
./build.sh
```

### Manual Build

```bash
cd mavlink_sdk
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE ..
make
```

### Build Outputs

The build produces both static and shared libraries in the `bin/` directory:
- `libmavlink_sdk.a` - Static library
- `libmavlink_sdk.so` - Shared library

## Quick Start

### 1. Include Headers

```cpp
#include "mavlink_sdk.h"
#include "mavlink_command.h"
#include "vehicle.h"
```

### 2. Create Event Handler

Inherit from `mavlinksdk::CMavlinkEvents` and override the callbacks you need:

```cpp
class MyDroneHandler : public mavlinksdk::CMavlinkEvents
{
public:
    void OnConnected(const bool& connected) override
    {
        std::cout << "Connection status: " << connected << std::endl;
    }

    void OnHeartBeat_First(const mavlink_heartbeat_t& heartbeat) override
    {
        std::cout << "First heartbeat received!" << std::endl;
    }

    void OnArmed(const bool& armed, const bool& ready_to_arm) override
    {
        std::cout << "Armed: " << armed << ", Ready: " << ready_to_arm << std::endl;
    }

    void OnModeChanges(const uint32_t& custom_mode, const int& firmware_type, 
                       const MAV_AUTOPILOT& autopilot) override
    {
        std::cout << "Mode changed to: " << custom_mode << std::endl;
    }

    void OnMessageReceived(const mavlink_message_t& mavlink_message) override
    {
        // Handle raw MAVLink messages if needed
    }
};
```

### 3. Initialize and Connect

```cpp
int main()
{
    MyDroneHandler handler;
    
    // Get SDK instance
    mavlinksdk::CMavlinkSDK& sdk = mavlinksdk::CMavlinkSDK::getInstance();
    
    // Connect via UDP (e.g., to SITL or MAVProxy)
    sdk.connectUDP("0.0.0.0", 14550);
    
    // Or connect via Serial
    // sdk.connectSerial("/dev/ttyUSB0", 57600, false);
    
    // Or connect via TCP
    // sdk.connectTCP("127.0.0.1", 5760);
    
    // Start the SDK with your event handler
    sdk.start(&handler);
    
    // Your application loop here...
    
    // Cleanup
    sdk.stop();
    return 0;
}
```

## API Reference

### CMavlinkSDK (Main Entry Point)

| Method | Description |
|--------|-------------|
| `getInstance()` | Get singleton instance |
| `connectUDP(ip, port)` | Configure UDP connection |
| `connectSerial(device, baudrate, dynamic)` | Configure serial connection |
| `connectTCP(ip, port)` | Configure TCP connection |
| `start(event_handler)` | Start SDK with event handler |
| `stop()` | Stop SDK and close connections |
| `sendMavlinkMessage(msg)` | Send raw MAVLink message |

### CMavlinkCommand (Vehicle Control)

Access via `mavlinksdk::CMavlinkCommand::getInstance()`:

| Method | Description |
|--------|-------------|
| `doArmDisarm(arm, force)` | Arm or disarm the vehicle |
| `doSetMode(mode, custom_mode, custom_sub_mode)` | Change flight mode |
| `takeOff(altitude)` | Initiate takeoff to specified altitude |
| `gotoGuidedPoint(lat, lon, alt)` | Fly to GPS coordinate |
| `changeAltitude(altitude)` | Change target altitude |
| `setHome(yaw, lat, lon, alt)` | Set home position |
| `setROI(lat, lon, alt)` | Set region of interest |
| `resetROI()` | Clear region of interest |
| `setYawCondition(angle, rate, clockwise, relative)` | Control yaw |
| `setNavigationSpeed(type, speed, throttle, relative)` | Set navigation speed |
| `setServo(channel, pwm)` | Control servo output |
| `sendRCChannels(channels, length)` | Send RC override |
| `releaseRCChannels()` | Release RC override |
| `requestMissionList()` | Request mission from vehicle |
| `writeMission(mission_map)` | Upload mission to vehicle |
| `clearWayPoints()` | Clear mission on vehicle |
| `setCurrentMission(mission_number)` | Set active waypoint |
| `readParameter(param_name)` | Read single parameter |
| `writeParameter(param_name, value)` | Write parameter |
| `requestParametersList()` | Request all parameters |
| `cmdTerminateFlight()` | Emergency flight termination |
| `requestHomeLocation()` | Request home position |
| `sendNative(mavlink_message)` | Send raw MAVLink message |

### CVehicle (Vehicle State & Telemetry)

The `CVehicle` class is the central component for managing vehicle state and telemetry data. It automatically parses incoming MAVLink messages, maintains the current vehicle state, and triggers callbacks when significant changes occur.

Access via `mavlinksdk::CVehicle::getInstance()`:

#### Architecture

`CVehicle` is a singleton that:
1. **Parses all incoming MAVLink messages** via `parseMessage()`
2. **Maintains cached copies** of all telemetry data
3. **Detects state changes** (arm/disarm, mode changes, flying status)
4. **Triggers callbacks** through `CCallBack_Vehicle` interface
5. **Handles message timestamps** for timeout detection

#### Supported MAVLink Messages

The SDK automatically parses and stores these MAVLink messages:

| Message ID | Description | Accessor Method |
|------------|-------------|-----------------|
| `HEARTBEAT` | Vehicle type, mode, arm state | `getMsgHeartBeat()` |
| `SYS_STATUS` | System health, battery voltage | `getMsgSysStatus()` |
| `BATTERY_STATUS` | Detailed battery info (voltage array, current, temp) | `getMsgBatteryStatus()` |
| `BATTERY2` | Secondary battery voltage | `getMsgBattery2Status()` |
| `GLOBAL_POSITION_INT` | GPS position (lat/lon/alt) | `getMsgGlobalPositionInt()` |
| `LOCAL_POSITION_NED` | Local NED position | `getMsgLocalPositionNED()` |
| `GPS_RAW_INT` | Raw GPS data with fix type | `getMSGGPSRaw()` |
| `GPS2_RAW` | Secondary GPS data | `getMSGGPS2Raw()` |
| `ATTITUDE` | Roll, pitch, yaw angles | `getMsgAttitude()` |
| `VFR_HUD` | Airspeed, groundspeed, heading, throttle | `getMsgVFRHud()` |
| `HOME_POSITION` | Home location | `getMsgHomePosition()` |
| `NAV_CONTROLLER_OUTPUT` | Navigation targets, wp_dist, alt_error | `getMsgNavController()` |
| `RC_CHANNELS` | RC input channels (18 channels) | `getRCChannels()` |
| `SERVO_OUTPUT_RAW` | Servo/motor outputs | `getServoOutputRaw()` |
| `RADIO_STATUS` | Radio link quality (RSSI, noise) | `getRadioStatus()` |
| `SYSTEM_TIME` | Autopilot time (detects reboots) | `getSystemTime()` |
| `TERRAIN_REPORT` | Terrain altitude data | `getTerrainReport()` |
| `EKF_STATUS_REPORT` | EKF health flags | `getEkf_status_report()` |
| `VIBRATION` | Vibration levels and clipping | `getVibration()` |
| `DISTANCE_SENSOR` | Rangefinder/lidar data (all orientations) | `getDistanceSensor(direction)` |
| `WIND` | Wind speed and direction | `getMsgWind()` |
| `ADSB_VEHICLE` | ADS-B traffic | `getADSBVechile()` |
| `HIGH_LATENCY` / `HIGH_LATENCY2` | Satellite link telemetry | `getHighLatency()` / `getHighLatency2()` |
| `FLIGHT_INFORMATION` | Flight time info | `getFlightInformation()` |
| `EXTENDED_SYS_STATE` | PX4 landed state | (internal use) |

#### State Detection Methods

| Method | Description |
|--------|-------------|
| `isFCBConnected()` | Returns `true` if heartbeat received within last 3 seconds |
| `isArmed()` | Returns `true` if vehicle is armed |
| `isFlying()` | Returns `true` if vehicle is in flight (armed + active state) |
| `isReadyToArm()` | Returns `true` if pre-arm checks passed |
| `isMotorEnabled()` | Returns `true` if motors are enabled and healthy |
| `hasLidarAltitude()` | Returns `true` if downward-facing lidar is available |

#### System ID Filtering

By default, the SDK accepts messages from any vehicle. You can restrict to specific system/component IDs:

```cpp
mavlinksdk::CVehicle& vehicle = mavlinksdk::CVehicle::getInstance();

// Only accept messages from system ID 1
vehicle.restrictMessageToSysID(1);

// Only accept messages from component ID 1 (autopilot)
vehicle.restrictMessageToCompID(1);

// Get current system/component ID of connected vehicle
int sysid = vehicle.getSysId();
int compid = vehicle.getCompId();
```

#### Distance Sensors

The SDK supports all MAVLink sensor orientations:

```cpp
// Get downward-facing lidar (altitude)
auto lidar = vehicle.getLidarAltitude();
std::cout << "Altitude: " << lidar.current_distance << " cm" << std::endl;

// Get specific orientation
auto front = vehicle.getDistanceSensor(MAV_SENSOR_ROTATION_NONE);        // Forward
auto back = vehicle.getDistanceSensor(MAV_SENSOR_ROTATION_YAW_180);      // Backward
auto down = vehicle.getDistanceSensor(MAV_SENSOR_ROTATION_PITCH_270);    // Down (lidar)
```

#### High Latency Mode

The SDK automatically detects and handles high-latency satellite links:

```cpp
int mode = vehicle.getHighLatencyMode();
// 0 = Normal mode
// MAVLINK_MSG_ID_HIGH_LATENCY = High latency v1
// MAVLINK_MSG_ID_HIGH_LATENCY2 = High latency v2

if (mode == MAVLINK_MSG_ID_HIGH_LATENCY2) {
    auto hl2 = vehicle.getHighLatency2();
    // Use high latency data
}
```

#### Guided Mode Position Tracking

For altitude changes in GUIDED mode:

```cpp
// Set current guided target (called internally when gotoGuidedPoint is used)
vehicle.setGuidedPoint(latitude, longitude, relative_altitude);

// Get position for altitude change commands
LOCATION_3D pos = vehicle.getPositionforChangeAltitude();
```

#### Message Timestamps

Track when messages were last received:

```cpp
// Get timestamp of last GPS message (microseconds)
uint64_t gps_time = vehicle.getMessageTime(MAVLINK_MSG_ID_GLOBAL_POSITION_INT);

// Check if message has been processed
uint16_t flags = vehicle.getProcessedFlag(MAVLINK_MSG_ID_ATTITUDE);
if (flags == MESSAGE_UNPROCESSED) {
    // New data available
    vehicle.setProcessedFlag(MAVLINK_MSG_ID_ATTITUDE, MESSAGE_PROCESSED);
}
```

#### Autopilot Compatibility

The SDK handles differences between ArduPilot and PX4:

- **ArduPilot**: Flying state detected from `system_status` in heartbeat
- **PX4**: Flying state detected from `EXTENDED_SYS_STATE` message
- **Rovers**: Relative altitude normalized to 0

#### Complete CVehicle Usage Example

```cpp
void printTelemetry()
{
    mavlinksdk::CVehicle& vehicle = mavlinksdk::CVehicle::getInstance();
    
    if (!vehicle.isFCBConnected()) {
        std::cout << "Vehicle disconnected!" << std::endl;
        return;
    }
    
    // Position
    auto gps = vehicle.getMsgGlobalPositionInt();
    std::cout << "Position: " 
              << gps.lat / 1e7 << ", " 
              << gps.lon / 1e7 << std::endl;
    std::cout << "Altitude (rel): " << gps.relative_alt / 1000.0 << " m" << std::endl;
    
    // Attitude
    auto att = vehicle.getMsgAttitude();
    std::cout << "Roll: " << att.roll * 57.3 << "°" << std::endl;
    std::cout << "Pitch: " << att.pitch * 57.3 << "°" << std::endl;
    std::cout << "Yaw: " << att.yaw * 57.3 << "°" << std::endl;
    
    // Speed
    auto hud = vehicle.getMsgVFRHud();
    std::cout << "Groundspeed: " << hud.groundspeed << " m/s" << std::endl;
    std::cout << "Airspeed: " << hud.airspeed << " m/s" << std::endl;
    std::cout << "Throttle: " << hud.throttle << "%" << std::endl;
    
    // Battery
    auto sys = vehicle.getMsgSysStatus();
    std::cout << "Battery: " << sys.voltage_battery / 1000.0 << " V" << std::endl;
    std::cout << "Current: " << sys.current_battery / 100.0 << " A" << std::endl;
    std::cout << "Remaining: " << (int)sys.battery_remaining << "%" << std::endl;
    
    // GPS Quality
    auto gps_raw = vehicle.getMSGGPSRaw();
    std::cout << "GPS Fix: " << (int)gps_raw.fix_type << std::endl;
    std::cout << "Satellites: " << (int)gps_raw.satellites_visible << std::endl;
    
    // Status
    std::cout << "Armed: " << vehicle.isArmed() << std::endl;
    std::cout << "Flying: " << vehicle.isFlying() << std::endl;
    std::cout << "Ready to Arm: " << vehicle.isReadyToArm() << std::endl;
    
    // Home
    auto home = vehicle.getMsgHomePosition();
    std::cout << "Home: " << home.latitude / 1e7 << ", " 
              << home.longitude / 1e7 << std::endl;
    
    // Navigation
    auto nav = vehicle.getMsgNavController();
    std::cout << "WP Distance: " << nav.wp_dist << " m" << std::endl;
    std::cout << "Alt Error: " << nav.alt_error << " m" << std::endl;
    
    // Lidar altitude (if available)
    if (vehicle.hasLidarAltitude()) {
        auto lidar = vehicle.getLidarAltitude();
        std::cout << "Lidar Alt: " << lidar.current_distance / 100.0 << " m" << std::endl;
    }
}
```

### CMavlinkEvents (Callbacks)

Override these methods in your event handler class:

#### Connection Events
- `OnConnected(connected)` - Connection state changed
- `OnHeartBeat_First(heartbeat)` - First heartbeat received
- `OnHeartBeat_Resumed(heartbeat)` - Heartbeat resumed after timeout
- `OnBoardRestarted()` - Flight controller restarted

#### Vehicle State Events
- `OnArmed(armed, ready_to_arm)` - Arm state changed
- `OnFlying(isFlying)` - Flying state changed
- `OnModeChanges(custom_mode, firmware_type, autopilot)` - Flight mode changed
- `OnStatusText(severity, status)` - Status message received
- `OnACK(cmd, result, result_msg)` - Command acknowledgment received

#### Telemetry Events
- `OnHomePositionUpdated(home_position)` - Home position updated
- `OnServoOutputRaw(servo_output_raw)` - Servo output changed
- `OnEKFStatusReportChanged(ekf_status_report)` - EKF status changed
- `OnVibrationChanged(vibration)` - Vibration data updated
- `OnDistanceSensorChanged(distance_sensor)` - Distance sensor updated
- `OnADSBVechileReceived(adsb_vehicle)` - ADS-B traffic received
- `OnHighLatencyModeChanged(latency_mode)` - High latency mode changed
- `OnHighLatencyMessageReceived(latency_mode)` - High latency message received

#### Mission Events
- `OnWaypointReached(seq)` - Waypoint reached
- `OnWayPointReceived(mission_item_int)` - Mission item downloaded
- `OnWayPointsLoadingCompleted()` - Mission download complete
- `OnMissionACK(result, mission_type, result_msg)` - Mission acknowledgment
- `OnMissionSaveFinished(result, mission_type, result_msg)` - Mission upload complete
- `OnMissionCurrentChanged(mission_current)` - Active waypoint changed

#### Parameter Events
- `OnParamReceived(param_name, param_message, changed, first_iteration)` - Parameter received
- `OnParamReceivedCompleted()` - All parameters received

#### Raw Message Event
- `OnMessageReceived(mavlink_message)` - Any MAVLink message received

## Example: Complete Application

```cpp
#include <iostream>
#include <thread>
#include <chrono>
#include "mavlink_sdk.h"
#include "mavlink_command.h"
#include "vehicle.h"

class DroneController : public mavlinksdk::CMavlinkEvents
{
private:
    bool m_connected = false;
    bool m_armed = false;

public:
    void OnConnected(const bool& connected) override
    {
        m_connected = connected;
        std::cout << "Connected: " << connected << std::endl;
    }

    void OnHeartBeat_First(const mavlink_heartbeat_t& heartbeat) override
    {
        std::cout << "Vehicle detected! Type: " << (int)heartbeat.type << std::endl;
        
        // Request parameters after connection
        mavlinksdk::CMavlinkCommand::getInstance().requestParametersList();
    }

    void OnArmed(const bool& armed, const bool& ready_to_arm) override
    {
        m_armed = armed;
        std::cout << "Armed: " << armed << std::endl;
    }

    void OnModeChanges(const uint32_t& custom_mode, const int& firmware_type,
                       const MAV_AUTOPILOT& autopilot) override
    {
        std::cout << "Mode: " << custom_mode << std::endl;
    }

    void OnStatusText(const std::uint8_t& severity, const std::string& status) override
    {
        std::cout << "Status [" << (int)severity << "]: " << status << std::endl;
    }

    void OnACK(const int& cmd, const int& result, const std::string& result_msg) override
    {
        std::cout << "ACK for CMD " << cmd << ": " << result_msg << std::endl;
    }

    bool isConnected() const { return m_connected; }
    bool isArmed() const { return m_armed; }
};

int main()
{
    DroneController controller;
    mavlinksdk::CMavlinkSDK& sdk = mavlinksdk::CMavlinkSDK::getInstance();
    mavlinksdk::CMavlinkCommand& cmd = mavlinksdk::CMavlinkCommand::getInstance();
    mavlinksdk::CVehicle& vehicle = mavlinksdk::CVehicle::getInstance();

    // Connect to SITL
    sdk.connectUDP("0.0.0.0", 14550);
    sdk.start(&controller);

    // Wait for connection
    while (!controller.isConnected())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Wait for vehicle to be ready
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Example: Set mode to GUIDED (mode 4 for ArduCopter)
    cmd.doSetMode(4);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Example: Arm the vehicle
    cmd.doArmDisarm(true, false);

    // Main loop - print telemetry
    for (int i = 0; i < 100; ++i)
    {
        if (vehicle.isFCBConnected())
        {
            auto pos = vehicle.getMsgGlobalPositionInt();
            auto att = vehicle.getMsgAttitude();
            
            std::cout << "Position: " 
                      << pos.lat / 1e7 << ", " 
                      << pos.lon / 1e7 << ", "
                      << pos.relative_alt / 1000.0 << "m" << std::endl;
            
            std::cout << "Attitude: Roll=" << att.roll * 57.3 
                      << " Pitch=" << att.pitch * 57.3 
                      << " Yaw=" << att.yaw * 57.3 << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Cleanup
    sdk.stop();
    return 0;
}
```

## Integration with Your Project

### CMake Integration

Add to your `CMakeLists.txt`:

```cmake
# Add MAVLink SDK
add_subdirectory(path/to/mavlink_sdk)

# Link against the SDK
target_link_libraries(your_target PRIVATE mavlink_sdk)

# Include directories
target_include_directories(your_target PRIVATE 
    path/to/mavlink_sdk/src
    path/to/c_library_v2
)
```

### Manual Linking

```bash
g++ -std=c++17 your_app.cpp \
    -I/path/to/mavlink_sdk/src \
    -I/path/to/c_library_v2 \
    -L/path/to/mavlink_sdk/bin \
    -lmavlink_sdk \
    -lpthread \
    -o your_app
```

## Connection Examples

### SITL (Software In The Loop)

```cpp
// Connect to ArduPilot SITL default port
sdk.connectUDP("0.0.0.0", 14550);
```

### MAVProxy

```cpp
// Connect to MAVProxy output
sdk.connectUDP("127.0.0.1", 14550);
```

### Pixhawk via USB

```cpp
// Linux
sdk.connectSerial("/dev/ttyACM0", 115200, false);

// Or with dynamic port detection
sdk.connectSerial("/dev/ttyACM0", 115200, true);
```

### Telemetry Radio

```cpp
// 3DR Radio or similar at 57600 baud
sdk.connectSerial("/dev/ttyUSB0", 57600, false);
```

### TCP Connection

```cpp
// Connect to MAVLink router or proxy
sdk.connectTCP("192.168.1.100", 5760);
```

## Thread Safety

- The SDK uses internal threading for message reception
- Callbacks are invoked from the communication thread
- Use appropriate synchronization when accessing shared data from callbacks
- The singleton instances are thread-safe for access

## License

This SDK is provided under the BSD-3-Clause license, consistent with the MAVLink project licensing.

## Contributing

Contributions are welcome! Please ensure your code follows the existing style and includes appropriate documentation.

## Related Projects

- [MAVLink](https://mavlink.io/) - MAVLink protocol
- [ArduPilot](https://ardupilot.org/) - ArduPilot autopilot
- [PX4](https://px4.io/) - PX4 autopilot
