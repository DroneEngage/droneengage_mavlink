#include <mavlink_sdk.h>
#include "../messages.hpp"


#include "../helpers/colors.hpp"
#include "../mission/missions.hpp"


using namespace uavos::fcb::mission;
//***********************************CRTL_Step

void CRTL_Step::decodeMavlink (const mavlink_mission_item_int_t& mission_item_int)
{
    
    m_sequence = mission_item_int.seq;
    m_auto_continue = mission_item_int.autocontinue;
    m_frame = mission_item_int.frame;
   
}

Json CRTL_Step::getAndruavMission()
{
    /*
        t : TYPE_CMissionAction_RTL
        s : sequence
    */
    Json message =
    {
        {"t", TYPE_CMissionAction_RTL},
        {"s", this->m_sequence}
    };

    return message;
}

mavlink_mission_item_int_t CRTL_Step::getArdupilotMission()
{
    mavlink_mission_item_int_t mavlink_mission;

    mavlink_mission.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_mission.seq = m_sequence;
    mavlink_mission.command = MAV_CMD_NAV_RETURN_TO_LAUNCH;
    mavlink_mission.autocontinue = m_auto_continue?1:0;
    mavlink_mission.frame = m_frame;

    
    return mavlink_mission;
}


//***********************************CTakeOff_Step


void CTakeOff_Step::decodeMavlink (const mavlink_mission_item_int_t& mission_item_int)
{
    
    m_sequence = mission_item_int.seq;
    m_auto_continue = mission_item_int.autocontinue;
    m_frame = mission_item_int.frame;
    
    m_minimum_pitch = mission_item_int.param1;
    m_desired_yaw = mission_item_int.param4;
    m_latitude = mission_item_int.x;
    m_longitude = mission_item_int.y;
    m_altitude = mission_item_int.z;
}


Json CTakeOff_Step::getAndruavMission()
{
    /*
        t : TYPE_CMissionAction_RTL
        s : sequence
        l : altitude
        p : minimum_pitch
    */
    Json message =
    {
        {"t", TYPE_CMissionAction_TakeOff},
        {"s", this->m_sequence},
        {"l", this->m_altitude},
        {"p", this->m_minimum_pitch}
    };

    return message;
}


mavlink_mission_item_int_t CTakeOff_Step::getArdupilotMission()
{
    mavlink_mission_item_int_t mavlink_mission;

    mavlink_mission.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_mission.command = MAV_CMD_NAV_TAKEOFF;
    mavlink_mission.autocontinue = m_auto_continue?1:0;
    mavlink_mission.frame = m_frame;
    
    mavlink_mission.seq = m_sequence;
    mavlink_mission.param1 = m_minimum_pitch;
    mavlink_mission.param2 = 0;
    mavlink_mission.param3 = 0;
    mavlink_mission.param4 = m_desired_yaw;
    mavlink_mission.x = m_latitude;
    mavlink_mission.y = m_longitude;
    mavlink_mission.z = m_altitude;

    return mavlink_mission;
    
}


//***********************************CLand_Step


void CLand_Step::decodeMavlink (const mavlink_mission_item_int_t& mission_item_int)
{
    
    m_sequence = mission_item_int.seq;
    m_auto_continue = mission_item_int.autocontinue;
    m_frame = mission_item_int.frame;
    
    m_abort_altitude = mission_item_int.param1;
    m_land_mode = mission_item_int.param2;
    m_desired_yaw = mission_item_int.param4;
    m_latitude = mission_item_int.x;
    m_longitude = mission_item_int.y;
    m_altitude = mission_item_int.z;
}


Json CLand_Step::getAndruavMission()
{
    /*
        t : TYPE_CMissionAction_RTL
        s : sequence
    */
    Json message =
    {
        {"t", TYPE_CMissionAction_Landing},
        {"s", this->m_sequence}
    };

    return message;
}


mavlink_mission_item_int_t CLand_Step::getArdupilotMission()
{
    mavlink_mission_item_int_t mavlink_mission;

    mavlink_mission.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_mission.command = MAV_CMD_NAV_LAND;
    mavlink_mission.autocontinue = m_auto_continue?1:0;
    mavlink_mission.frame = m_frame;
    
    mavlink_mission.seq = m_sequence;
    mavlink_mission.param1 = m_abort_altitude;
    mavlink_mission.param2 = m_land_mode;
    mavlink_mission.param3 = 0;
    mavlink_mission.param4 = m_desired_yaw;
    mavlink_mission.x = m_latitude;
    mavlink_mission.y = m_longitude;
    mavlink_mission.z = m_altitude;
    
    return mavlink_mission;
    
}


//***********************************CWayPoint_Step

void CWayPoint_Step::decodeMavlink (const mavlink_mission_item_int_t& mission_item_int)
{
    
    m_sequence = mission_item_int.seq;
    m_auto_continue = mission_item_int.autocontinue;
    m_frame = mission_item_int.frame;
    
    m_time_to_stay     = mission_item_int.param1;
    m_accepted_radius  = mission_item_int.param2;
    m_pass_radius      = mission_item_int.param3;
    m_desired_yaw      = mission_item_int.param4;
    m_latitude         = mission_item_int.x;
    m_longitude        = mission_item_int.y;
    m_altitude         = mission_item_int.z;
} 


Json CWayPoint_Step::getAndruavMission()
{
    /*
        t : TYPE_CMissionAction_RTL
        s : sequence
        a : latitude in format xx.xxxxx
        g : longitude in format xx.xxxxx
        l : altitude in meter
        h : desired yaw in degree
        y : time to stay
    */
    Json message =
    {
        {"t", TYPE_CMissionItem_WayPointStep},
        {"s", this->m_sequence},
        {"a", this->m_latitude / 10000000.0},
        {"g", this->m_longitude / 10000000.0},
        {"l", this->m_altitude},
        {"h", this->m_desired_yaw},
        {"y", this->m_time_to_stay}
    };

    return message;
}


mavlink_mission_item_int_t CWayPoint_Step::getArdupilotMission()
{
    mavlink_mission_item_int_t mavlink_mission;

    mavlink_mission.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_mission.command = MAV_CMD_NAV_WAYPOINT;
    mavlink_mission.autocontinue = m_auto_continue?1:0;
    mavlink_mission.frame = m_frame;
    
    mavlink_mission.seq     = m_sequence;
    mavlink_mission.param1  = m_time_to_stay;
    mavlink_mission.param2  = m_accepted_radius;
    mavlink_mission.param3  = m_pass_radius;
    mavlink_mission.param4  = m_desired_yaw;
    mavlink_mission.x       = m_latitude;
    mavlink_mission.y       = m_longitude;
    mavlink_mission.z       = m_altitude;

    return mavlink_mission;
}