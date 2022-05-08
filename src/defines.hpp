#ifndef DEFINES_H_
#define DEFINES_H_


#include <iostream>

#define CONNECTION_TYPE_SERIAL  1
#define CONNECTION_TYPE_UDP     2
#define CONNECTION_TYPE_TCP     3
#define CONNECTION_TYPE_UNKNOWN 4

#define E_UNDEFINED_MODE          9999

#define TelemetryProtocol_No_Telemetry       0
#define TelemetryProtocol_DroneKit_Telemetry 4

#define PI  3.1415926535897932384626433832795
#define DEGREES_TO_RADIANS PI / 180.0f
#define RADIANS_TO_DEGREES 180.0f / PI

// 3 seconds timeout
#define RCCHANNEL_OVERRIDES_TIMEOUT 3000000 

typedef enum ANDRUAV_UNIT_TYPE
{
        VEHICLE_TYPE_UNKNOWN    = 0,
        VEHICLE_TYPE_TRI        = 1,
        VEHICLE_TYPE_QUAD       = 2,
        VEHICLE_TYPE_PLANE      = 3,
        VEHICLE_TYPE_ROVER      = 4,
        VEHICLE_TYPE_HELI       = 5,        
        VEHICLE_TYPE_SUBMARINE  =12,
        // no used here ... only for refence
        VEHICLE_TYPE_GIMBAL     = 15,
        VEHICLE_TYPE_GCS = 999
        // end of reference
} ANDRUAV_UNIT_TYPE;

typedef enum ANDRUAV_UNIT_MODE 
{
    VEHICLE_MODE_RTL                = 2,
    VEHICLE_MODE_FOLLOW_ME          = 3,
    VEHICLE_MODE_AUTO               = 5,
    VEHICLE_MODE_STABILIZE          = 6,
    VEHICLE_MODE_ALT_HOLD           = 7,
    VEHICLE_MODE_MANUAL             = 8,
    VEHICLE_MODE_GUIDED             = 9,
    VEHICLE_MODE_LOITER             = 10,
    VEHICLE_MODE_POS_HOLD           = 11,
    VEHICLE_MODE_LAND               = 12,
    VEHICLE_MODE_CIRCLE             = 13,
    VEHICLE_MODE_FBWA               = 14,
    VEHICLE_MODE_CRUISE             = 15,
    VEHICLE_MODE_FBWB               = 16,
    VEHICLE_MODE_BRAKE              = 17,
    VEHICLE_MODE_SMART_RTL          = 21,
    VEHICLE_MODE_TAKEOFF            = 22,
    VEHICLE_MODE_QHOVER        		= 23,
    VEHICLE_MODE_QLOITER       		= 24,
    VEHICLE_MODE_QSTABILIZE    		= 25,
    VEHICLE_MODE_QLAND         		= 26,
    VEHICLE_MODE_QRTL          		= 27,
    VEHICLE_MODE_INITALIZING        = 99,
    VEHICLE_MODE_SURFACE            = 101,
    VEHICLE_MODE_MOTOR_DETECT       = 102,
    VEHICLE_MODE_UNKNOWN            = 999,
    
    
} ANDRUAV_UNIT_MODE ;


typedef enum RC_SUB_ACTION 
{
    // there is no RCChannel info sent to Drone.
    RC_SUB_ACTION_RELEASED                      =   0,
    // 1500 channels values are sent. TX is no longer effective.
    RC_SUB_ACTION_CENTER_CHANNELS               =   1,
    // last TX readings are freezed and sent as fixed values. TX is no longer effective.
    RC_SUB_ACTION_FREEZE_CHANNELS               =   2,
    // RCChannels is being sent to Drone. TX  is no longer effective for some channels.
    RC_SUB_ACTION_JOYSTICK_CHANNELS             =   4,
    // Velocity is sent for Thr, Pitch, Roll , YAWRate ... applicable in Arducopter and Rover
    // Drone may switch {@link _7adath_FCB_RemoteControlSettings#RC_SUB_ACTION_JOYSTICK_CHANNELS} to this automatically if drone mode is guided.
    RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED      =   8
} RC_SUB_ACTION;

typedef struct ANDRUAV_VEHICLE_INFO 
{
    std::string         party_id                           = std::string("");
    std::string         group_id                           = std::string("");
    bool                use_fcb                             = false;
    bool                is_armed                            = false;
    bool                is_flying                           = false;
    bool                is_tracking_mode                    = false;
    bool                is_gcs_blocked                      = false;
    int16_t             flying_mode;
    int16_t             gps_mode                            ;
    u_int64_t           flying_total_duration               = 0; // total flight duration
    u_int64_t           flying_last_start_time              = 0; // duration of the current or latest flight
    int16_t             vehicle_type                        = 0;

    int16_t             current_waypoint                    = 0;         

    bool                is_rcChannelBlock                   = false; //TODO not implemented
    RC_SUB_ACTION       rc_sub_action                       = RC_SUB_ACTION::RC_SUB_ACTION_RELEASED;
    /**
     * @brief if true then channl override is active
     * 
     */
    bool                rc_command_active                   = false;      
    u_int64_t           rc_command_last_update_time         = 0l;    
    int16_t             rc_channels[18]                     = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    int16_t             rc_channels_min[18]                 = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    int16_t             rc_channels_max[18]                 = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    bool                rc_channels_enabled[18];
    bool                rc_channels_reverse[18];

}   ANDRUAV_VEHICLE_INFO;


typedef struct
{
    uint16_t rcmap_pitch;
    uint16_t rcmap_roll;
    uint16_t rcmap_throttle;
    uint16_t rcmap_yaw;
    /**
     * @brief if true then rc_map is valid and can be used for mapping.
     * 
     */
    bool is_valid = false;
    /**
     * @brief if true the user requires using rc_map
     * 
     */
    bool use_smart_rc = false;
} RCMAP_CHANNELS_MAP_INFO_STRUCT;

#endif