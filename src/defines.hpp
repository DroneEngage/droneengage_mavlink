#ifndef DEFINES_H_
#define DEFINES_H_

#define CONNECTION_TYPE_SERIAL  1
#define CONNECTION_TYPE_UDP     2
#define CONNECTION_TYPE_TCP     3
#define CONNECTION_TYPE_UNKNOWN 4

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


typedef struct ANDRUAV_VEHICLE_INFO 
{
    bool        usd_fcb                             = false;
    bool        use_FCB_IMU                         = false;
    bool        is_armed                            = false;
    int16_t     flying_mode;
    int16_t     gps_mode;
    u_int64_t   flying_total_duration               = 0;
    u_int64_t   flying_last_start_time              = 0;
    int16_t     vehicle_type                        = 0;
}   ANDRUAV_VEHICLE_INFO;


#endif