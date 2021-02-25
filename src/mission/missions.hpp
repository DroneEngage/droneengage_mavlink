#ifndef MISSIONS_H_
#define MISSIONS_H_

#include <map>
#include <memory>
#include <iostream>
#include <mavlink_command.h>

#include "../helpers/json.hpp"
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
        virtual void decodeMavlink (const mavlink_mission_item_int_t& mavlink_mission_item) =0;
        virtual Json getAndruavMission   ()=0;
        virtual mavlink_mission_item_int_t getArdupilotMission ()=0;

    public:
        int m_sequence;
        int m_mission_command;
        int m_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        bool m_auto_continue;
        bool m_current = false;
        

};


/**
 * @brief zero delay can be used later to store mission related to andruav.
 * Assuming zero delays is an andruav related mission.
 * 
 */
class CDummy_Step : public CMissionItem
{
    public:
        CDummy_Step ()
        {
            m_mission_command = TYPE_CMissionAction_DummyMission;
        };
        

        void decodeMavlink (const mavlink_mission_item_int_t& mission_item_int) override;


    public:
        Json getAndruavMission   () override;
        mavlink_mission_item_int_t getArdupilotMission () override;

};


/**
 * @brief Delay the next navigation command a number of seconds or until a specified time
 * 
 */
class CDelay_Step : public CMissionItem
{
    public:
        CDelay_Step ()
        {
            m_mission_command = TYPE_CMissionAction_Delay;
        };
        

        void decodeMavlink (const mavlink_mission_item_int_t& mission_item_int) override;


    public:
        Json getAndruavMission   () override;
        mavlink_mission_item_int_t getArdupilotMission () override;

    public:
        /**
         * @brief Delay (-1 to enable time-of-day fields)
         * 
         */
        int m_delay = -1;
        /**
         * @brief hour (24h format, UTC, -1 to ignore)
         * 
         */
        int m_delay_hours   = -1;
        /**
         * @brief minute (24h format, UTC, -1 to ignore)
         * 
         */
        int m_delay_minutes = -1;
        /**
         * @brief second (24h format, UTC, -1 to ignore)
         * 
         */
        int m_delay_seconds = -1;


};

/**
 * @brief Delay mission state machine.
 * 
 */
class CDelay_State_Machine_Step : public CMissionItem
{
    public:
        CDelay_State_Machine_Step ()
        {
            m_mission_command = TYPE_CMissionAction_Delay_STATE_MACHINE;
        };
        

        void decodeMavlink (const mavlink_mission_item_int_t& mission_item_int) override;


    public:
        Json getAndruavMission   () override;
        mavlink_mission_item_int_t getArdupilotMission () override;

    public:
        /**
         * @brief minimum zero in seconds
         * 
         */
        int m_delay_seconds = 0;

};



class CChange_Heading_Step : public CMissionItem
{
    public:
        CChange_Heading_Step ()
        {
            m_mission_command = TYPE_CMissionAction_ChangeHeading;
        };
        

        void decodeMavlink (const mavlink_mission_item_int_t& mission_item_int) override;


    public:
        Json getAndruavMission   () override;
        mavlink_mission_item_int_t getArdupilotMission () override;

    public:
       
       /**
        * @brief target angle, 0 is north in deg
        * 
        */
       double m_angle;

       /**
        * @brief angular speed in deg/sec
        * 
        */
       double m_angular_speed;

       /**
        * @brief direction: -1: counter clockwise, 1: clockwise 
        * 
        */
       double m_direction;

       /**
        * @brief 0: absolute angle, 1: relative offset
        * 
        */
       double m_relative;

};


class CLoiter_Turns_Step : public CMissionItem
{
    public:
        CLoiter_Turns_Step ()
        {
            m_mission_command = TYPE_CMissionAction_Circle;
        };
        

        void decodeMavlink (const mavlink_mission_item_int_t& mission_item_int) override;


    public:
        Json getAndruavMission   () override;
        mavlink_mission_item_int_t getArdupilotMission () override;

        /**
         * @brief  Number of turns. min 0
         * 
         */
        int m_num_of_turn;

        /**
         * @brief  Leave loiter circle only once heading towards the next waypoint (0 = False)
         * 
         */
        bool m_heading_required;

        /**
         * @brief Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). 
         * NaN to use the current system default xtrack behaviour.
         * 
         */
        double m_xtrack_location;

        /**
         * @brief  Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise	
         * 
         */
        double m_radius;

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


class CRTL_Step : public CMissionItem
{
    public:
        CRTL_Step ()
        {
            m_mission_command = TYPE_CMissionAction_RTL;
        };
        

        void decodeMavlink (const mavlink_mission_item_int_t& mission_item_int) override;


    public:
        Json getAndruavMission   () override;
        mavlink_mission_item_int_t getArdupilotMission () override;

};


class CTakeOff_Step : public CMissionItem
{
    public:
        CTakeOff_Step ()
        {
            m_mission_command = TYPE_CMissionAction_TakeOff;
        };
        

    void decodeMavlink (const mavlink_mission_item_int_t& mission_item_int) override;

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
        CLand_Step ()
        {
            m_mission_command = TYPE_CMissionAction_Landing;
        }

        void decodeMavlink (const mavlink_mission_item_int_t& mission_item_int) override;

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
        int m_desired_yaw = 0;

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
        CWayPoint_Step ()
        {
            m_mission_command = TYPE_CMissionItem_WayPointStep;
        }

        void decodeMavlink (const mavlink_mission_item_int_t& mission_item_int) override;

    public:
        Json getAndruavMission   () override;
        mavlink_mission_item_int_t getArdupilotMission () override;

    public:

        /**
         * @brief Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)
         * in seconds
         */
        double m_time_to_stay = 0;

        /**
         * @brief Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
         * in degrees
         */
        double m_desired_yaw = 0;

        /**
         * @brief Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)
         * in meters
         */
        double m_accepted_radius = 0; 

        /**
         * @brief 0 to pass through the WP, if > 0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
         * in meters
         */
        double m_pass_radius = 0;  

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


class CMissionItemBuilder
{
    public:
    static CMissionItem * getClassByMavlinkCMD (const mavlink_mission_item_int_t& mission_item_int)
    {
        switch (mission_item_int.command)
        {

            case MAV_CMD_NAV_DELAY:
                return new CDelay_Step();

            case MAV_CMD_NAV_RETURN_TO_LAUNCH:
                return new CRTL_Step();

            case MAV_CMD_NAV_TAKEOFF:
                return new CTakeOff_Step();

            case MAV_CMD_NAV_LAND:
                return new CLand_Step();

            case MAV_CMD_NAV_WAYPOINT:
                return new CWayPoint_Step();


            case MAV_CMD_CONDITION_DELAY:
                if (mission_item_int.param1 == 0)
                {
                    return new CDummy_Step();
                }
            default:
               return new CDelay_State_Machine_Step();

        }

        
    }
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