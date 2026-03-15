# Tracking System Logic

## Overview

The DroneEngage tracking system provides autonomous target tracking capabilities for different vehicle types, with specialized logic for quadcopters and fixed-wing aircraft. The system processes visual tracking data and converts it into RC control commands to maintain visual lock on targets.

## Architecture

### Core Components

- **CTrackingManager**: Main coordinator that handles tracking status and delegates to appropriate tracker logic
- **CTrackerQuadLogic**: Specialized tracking logic for quadcopter/multirotor vehicles
- **CTrackerPlanLogic**: Specialized tracking logic for fixed-wing aircraft
- **CTrackerLogic**: Base class with common tracking functionality

### Key Classes

```cpp
namespace de::fcb::tracking {
    class CTrackingManager;     // Main tracking coordinator
    class CTrackerLogic;        // Base tracking functionality
    class CTrackerQuadLogic;    // Quadcopter-specific logic
    class CTrackerPlanLogic;    // Fixed-wing-specific logic
}
```

## Tracking Modes

### 1. TRACKING_TARGET (0)
Standard target tracking mode for following moving objects.

### 2. TRACKING_FOLLOW_ME (1)
Follow-me mode where the vehicle tracks a specific target (typically the operator).

### 3. TRACKING_STANDING (2)
Vertical camera tracking mode for ground-based target tracking.

## Status Management

### Tracking Status States

- **TargetTracking_STATUS_TRACKING_STOPPED**: Tracking inactive
- **TargetTracking_STATUS_TRACKING_ENABLED**: Tracking activated, waiting for target
- **TargetTracking_STATUS_TRACKING_DETECTED**: Target acquired and tracking active
- **TargetTracking_STATUS_TRACKING_LOST**: Target lost, tracking paused

### Status Change Flow

```cpp
void CTrackingManager::onStatusChanged(const int status) {
    switch (status) {
        case TargetTracking_STATUS_TRACKING_ENABLED:
            m_tracking_running = true;
            getTracker().onStatusChanged(status);
            break;
            
        case TargetTracking_STATUS_TRACKING_DETECTED:
            m_tracking_running = true;
            m_object_detected = true;
            getTracker().onStatusChanged(status);
            break;
            
        case TargetTracking_STATUS_TRACKING_LOST:
            m_object_detected = false;
            getTracker().onStatusChanged(status);
            break;
            
        case TargetTracking_STATUS_TRACKING_STOPPED:
            m_object_detected = false;
            m_tracking_running = false;
            getTracker().onStatusChanged(status);
            break;
    }
}
```

## Quadcopter Tracking Logic

### Tracker Selection

The system automatically selects the quadcopter tracker for multirotor vehicles:

```cpp
CTrackerLogic &CTrackingManager::getTracker() {
    switch (vehicle_type) {
        case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_TRI:
        case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_QUAD:
            return m_tracker_quad_logic;
        case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_PLANE:
            return m_tracker_plan_logic;
        default:
            return m_tracker_plan_logic;
    }
}
```

### Forward Camera Tracking

For forward-facing cameras, the quadcopter tracker controls all four flight channels:

```cpp
void CTrackerQuadLogic::trackingDroneForward(const double x, const double yz, 
                                             const double tracking_x, 
                                             const double tracking_yz) {
    int16_t rc_channels[RC_CHANNEL_TRACKING_COUNT] = {SKIP_RC_CHANNEL};
    
    // Roll control (horizontal tracking)
    rc_channels[RC_CHANNEL_TRACKING_ROLL] = 500;
    
    // Yaw control (horizontal tracking)
    rc_channels[RC_CHANNEL_TRACKING_YAW] = 1000 - tracking_x;
    
    // Pitch and Throttle control based on mode
    if (m_loose_altitude) {
        // Coordinated altitude control with state machine
        // Complex logic for balanced descent
    } else {
        // Simple pitch control
        rc_channels[RC_CHANNEL_TRACKING_PITCH] = tracking_yz;
        rc_channels[RC_CHANNEL_TRACKING_THROTTLE] = (yz > 0.3) ? 0 : 500;
    }
    
    m_fcbMain2.updateTrackingControlChannels(rc_channels);
}
```

### Vertical Camera Tracking

For downward-facing cameras (standing mode):

```cpp
void CTrackerQuadLogic::trackingStanding(const double x, const double yz,
                                        const double tracking_x, 
                                        const double tracking_yz) {
    int16_t rc_channels[RC_CHANNEL_TRACKING_COUNT] = {SKIP_RC_CHANNEL};
    
    // Horizontal positioning only
    rc_channels[RC_CHANNEL_TRACKING_ROLL] = 1000 - tracking_x;
    rc_channels[RC_CHANNEL_TRACKING_PITCH] = 1000 - tracking_yz;
    rc_channels[RC_CHANNEL_TRACKING_YAW] = 500;      // Neutral
    rc_channels[RC_CHANNEL_TRACKING_THROTTLE] = 500; // Neutral
    
    m_fcbMain2.updateTrackingControlChannels(rc_channels);
}
```

### Quadcopter State Machine

The quadcopter tracker uses a state machine for advanced altitude control:

```cpp
#define TRACK_COPTER_STATUS_NONE 0b00000000
#define TRACK_COPTER_STATUS_VERTICAL_CENTER_REACHED 0b00000001
#define TRACK_COPTER_STATUS_HORIZONTAL_CENTER_REACHED 0b00000010

// State management methods
inline void resetTrackerCopterStatus() { m_tracker_copter_status = 0; }
inline void addTrackerCopterStatus(uint8_t status) { m_tracker_copter_status |= status; }
inline bool isTrackerCopterStatus(uint8_t status) { return m_tracker_copter_status & status; }
```

## Configuration Parameters

### Quadcopter Tracking Parameters

The tracking system reads configuration from the `follow_me.quad` section:

```json
{
  "follow_me": {
    "quad": {
      "loose_altitude": false,
      "PID_P_X": 1.0,
      "PID_P_Y": 1.0,
      "PID_I_X": 0.1,
      "PID_I_Y": 0.1,
      "PID_D_X": 0.0,
      "PID_D_Y": 0.0,
      "expo_x": 0.3,
      "expo_y": 0.3,
      "deadband_x": 0.01,
      "deadband_y": 0.01,
      "center_hold_enabled": false,
      "center_hold_y_band": 0.02,
      "center_hold_decay": 0.02,
      "rate_limit": 0.05,
      "kalman_enabled": true,
      "kalman_process_noise_q": 0.01,
      "kalman_measurement_noise_r": 0.1
    }
  }
}
```

### Parameter Descriptions

- **PID_P_X/Y**: Proportional gains for horizontal and vertical tracking
- **PID_I_X/Y**: Integral gains for eliminating steady-state error
- **PID_D_X/Y**: Derivative gains for damping oscillations
- **expo_x/y**: Exponential curve for response shaping
- **deadband_x/y**: Deadband regions to prevent small oscillations
- **rate_limit**: Maximum rate of change for smooth control
- **kalman_enabled**: Enable/disable Kalman filtering
- **kalman_process_noise_q**: Process noise for Kalman filter
- **kalman_measurement_noise_r**: Measurement noise for Kalman filter

## Signal Processing

### Input Processing Pipeline

1. **Raw Input**: Receive tracking coordinates (x, yz) from vision system
2. **Deadband**: Apply deadband to filter small movements
3. **Exponential**: Apply exponential curve for response shaping
4. **Rate Limiting**: Limit maximum rate of change
5. **Kalman Filtering**: Optional smoothing using Kalman filter
6. **PID Control**: Apply PID control for stable tracking
7. **RC Output**: Convert to RC channel values

### PID Control Implementation

```cpp
class CPIDController {
    // Implements standard PID control with:
    // - Proportional term for immediate response
    // - Integral term for steady-state error elimination
    // - Derivative term for oscillation damping
};
```

### Kalman Filtering

The system uses simple Kalman filters for smooth tracking:

```cpp
class SimpleKalmanFilter {
    // Provides noise reduction and signal smoothing
    // Tunable via process_noise_q and measurement_noise_r parameters
};
```

## RC Channel Mapping

### Channel Definitions

```cpp
#define RC_CHANNEL_TRACKING_ROLL 0
#define RC_CHANNEL_TRACKING_PITCH 1  
#define RC_CHANNEL_TRACKING_THROTTLE 2
#define RC_CHANNEL_TRACKING_YAW 3
#define RC_CHANNEL_TRACKING_COUNT 4
#define SKIP_RC_CHANNEL -999
```

### Output Range

- **Range**: 0-1000 (standard PWM range)
- **Neutral**: 500 (center position)
- **Skip**: -999 (don't modify channel)

## Integration Points

### Main System Integration

The tracking system integrates with:

1. **FCB Main**: Receives tracking data and sends RC commands
2. **Vision System**: Provides target coordinates
3. **Configuration System**: Loads tracking parameters
4. **MAVLink Interface**: Communicates with flight controller

### Message Flow

```
Vision System → CTrackingManager → CTrackerQuadLogic → FCB Main → Flight Controller
```

## Initialization Sequence

```cpp
void CTrackingManager::init() {
    readConfigParameters();           // Load configuration
    m_tracker_plan_logic.init();      // Initialize plane tracker
    m_tracker_quad_logic.init();      // Initialize quad tracker
}
```

## Usage Example

### Basic Tracking Setup

```cpp
// Initialize tracking system
CTrackingManager &tracker = CTrackingManager::getInstance();
tracker.init();

// Enable tracking
tracker.onStatusChanged(TargetTracking_STATUS_TRACKING_ENABLED);

// Process tracking data
tracker.onTrack(x, yz, is_forward_camera);

// Handle target detection
tracker.onStatusChanged(TargetTracking_STATUS_TRACKING_DETECTED);
```

### Configuration Reload

```cpp
// Reload parameters if config file changes
tracker.reloadParametersIfConfigChanged();
```

## Debugging and Monitoring

### Debug Output

The system provides debug output for:

- Tracking status changes
- RC channel values
- Parameter loading
- Tracking mode selection

### Status Monitoring

```cpp
int status = tracker.getTrackingStatus();
bool is_running = tracker.m_tracking_running;
bool has_target = tracker.m_object_detected;
```

## Performance Considerations

### Timing

- High-resolution clock used for delta-time calculations
- Rate limiting prevents excessive control changes
- Efficient state management for real-time performance

### Memory

- Singleton pattern minimizes memory usage
- Static allocation for tracking instances
- Efficient data structures for real-time processing

## Error Handling

### Lost Target Recovery

When target is lost:
1. Set `m_object_detected = false`
2. Reset quadcopter state machine
3. Continue tracking with last known good parameters
4. Wait for target reacquisition

### Configuration Errors

- Missing parameters use default values
- Invalid ranges are clamped to safe limits
- Configuration errors logged but don't crash system

## Future Enhancements

### Planned Features

- Adaptive PID tuning
- Multiple target tracking
- Predictive tracking algorithms
- Enhanced Kalman filtering
- Machine learning integration

### Extension Points

- New vehicle types (hexacopter, octocopter)
- Custom tracking behaviors
- Advanced sensor fusion
- Autonomous mission integration
