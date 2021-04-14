#ifndef DEFINES_H_
#define DEFINES_H_

#define CONNECTION_TYPE_SERIAL  1
#define CONNECTION_TYPE_UDP     2
#define CONNECTION_TYPE_TCP     3
#define CONNECTION_TYPE_UNKNOWN 4

#define E_UNDEFINED_MODE          9999

#define TelemetryProtocol_DroneKit_Telemetry 4

#define PI  3.1415926535897932384626433832795
#define DEGREES_TO_RADIANS PI / 180.0f
#define RADIANS_TO_DEGREES 180.0f / PI


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
    VEHICLE_MODE_SURFACE            = 101,
    VEHICLE_MODE_MOTOR_DETECT       = 102,
    VEHICLE_MODE_INITALIZING        = 99,
    VEHICLE_MODE_UNKNOWN            = 999,
    
    
} ANDRUAV_UNIT_MODE ;

typedef struct ANDRUAV_VEHICLE_INFO 
{
    bool        use_fcb                             = false;
    bool        is_armed                            = false;
    bool        is_flying                           = false;
    bool        is_tracking_mode                    = false;
    int16_t     manual_TX_blocked_mode              ;
    bool        is_gcs_blocked                      = false;
    int16_t     flying_mode;
    int16_t     gps_mode                            ;
    u_int64_t   flying_total_duration               = 0;
    u_int64_t   flying_last_start_time              = 0;
    int16_t     vehicle_type                        = 0;

    int16_t     current_waypoint                    = 0;         
}   ANDRUAV_VEHICLE_INFO;


#endif