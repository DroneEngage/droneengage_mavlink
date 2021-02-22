#ifndef MISSIONS_H_
#define MISSIONS_H_

#include <map>
#include <memory>
#include <iostream>
#include <mavlink_command.h>

#include "./helpers/json.hpp"
using Json = nlohmann::json;

namespace uavos
{
namespace fcb
{

namespace mission
{

typedef enum ANDRUAV_MISSION_TYPE
{
        ANDRUAV_MISSION_UNKNOWN             = 0,
        ANDRUAV_MISSION_ARDUPILOT_WAYPOINTS = 1
} ANDRUAV_MISSION_TYPE;


class CMissionItem 
{


    public:

        virtual Json getAndruavMission   (){};
        virtual mavlink_mission_item_int_t getArdupilotMission (){};

    public:
        int m_sequence;
        int m_mission_command;
        bool m_auto_continue;
        /**
         * @brief local: x position in meters * 1e4, global: latitude in degrees * 10^7
         * 
         */
        
        

};


class CRTL_Step : public CMissionItem
{
    public:
        CRTL_Step (){};
        explicit CRTL_Step (const int& sequence)
        {
            m_sequence = sequence;
            m_mission_command = MAV_CMD_NAV_RETURN_TO_LAUNCH;
        }

    public:
        Json getAndruavMission   () override;
        mavlink_mission_item_int_t getArdupilotMission () override;

};


class CTakeOff_Step : public CMissionItem
{
    public:
        CTakeOff_Step (){};
        explicit CTakeOff_Step (const int& sequence)
        {
            m_sequence = sequence;
            m_mission_command = MAV_CMD_NAV_TAKEOFF;
        }

    public:
        Json getAndruavMission   () override;
        mavlink_mission_item_int_t getArdupilotMission () override;

    public:

        /**
         * @brief Minimum pitch (if airspeed sensor present), desired pitch without sensor.
         * in degrees
         */
        int m_minimum_pitch = 0;

        /**
         * @brief Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
         * in degrees
         * 
         */
        int m_desired_yaw;

        /**
         * @brief  in e7
         * 
         */
        int m_latitude = 0;

        /**
         * @brief in e7
         * 
         */
        int m_longitude = 0;

        /**
         * @brief in meters
         * 
         */
        int m_altitude = 0;
};

class CLand_Step : public CMissionItem
{
    public:
        CLand_Step (){};
        explicit CLand_Step (const int& sequence)
        {
            m_sequence = sequence;
            m_mission_command = TYPE_CMissionAction_Landing;
        }

    public:
        Json getAndruavMission   () override;
        mavlink_mission_item_int_t getArdupilotMission () override;

    public:

        /**
         * @brief Minimum target altitude if landing is aborted (0 = undefined/use system default).
         * in meters
         */
        int m_abort_altitude = 0;

        /**
         * @brief Precision land mode.
         * 
         */
        int m_land_mode = PRECISION_LAND_MODE_OPPORTUNISTIC;

        /**
         * @brief Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
         * in degrees
         */
        int m_desired_yaw;

        /**
         * @brief  in e7
         * 
         */
        int m_latitude = 0;

        /**
         * @brief in e7
         * 
         */
        int m_longitude = 0;

        /**
         * @brief Landing altitude (ground level in current frame).
         * in meters
         */
        int m_altitude = 0;

};

class CWayPoint_Step : public CMissionItem
{
    public:
        CWayPoint_Step (){};
        explicit CWayPoint_Step (const int& sequence)
        {
            m_sequence = sequence;
            m_mission_command = MAV_CMD_NAV_WAYPOINT;
        }

    public:
        Json getAndruavMission   () override;
        mavlink_mission_item_int_t getArdupilotMission () override;

    public:

        /**
         * @brief Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)
         * in seconds
         */
        int m_time_to_stay = 0;

        /**
         * @brief Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
         * in degrees
         */
        int m_desired_yaw = 0;

        /**
         * @brief Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)
         * in meters
         */
        int m_accepted_radius = 0; 

        /**
         * @brief 0 to pass through the WP, if > 0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
         * in meters
         */
        int m_pass_radius = 0;  

        /**
         * @brief  in e7
         * 
         */
        int m_latitude = 0;

        /**
         * @brief in e7
         * 
         */
        int m_longitude = 0;
        
        /**
         * @brief in meters
         * 
         */
        int m_altitude = 0;
    
};


/**
 * @brief holds andruav missions
 * Andruav missions can be ardupilot mission or andruav mission.
 * 
 * * Andruav Mission can contain swarm and waiting event actions.
 * * Ardupilot mission can contains embedded commands but no need 
 * * to save them here as they are not handled by Andruav right now.
 */
typedef struct ANDRUAV_UNIT_MISSION
{
    ANDRUAV_MISSION_TYPE mission_type;
    std::map <int, std::unique_ptr<CMissionItem>> mission_items;

    void clear ()
    {
        mission_type = ANDRUAV_MISSION_UNKNOWN;
        mission_items.clear();
    }

} ANDRUAV_UNIT_MISSION;

}
}
}

#endif