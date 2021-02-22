#include <mavlink_sdk.h>
#include "messages.hpp"


#include "./helpers/colors.hpp"
#include "missions.hpp"


Json uavos::fcb::mission::CRTL_Step::getAndruavMission()
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

mavlink_mission_item_int_t uavos::fcb::mission::CRTL_Step::getArdupilotMission()
{
    mavlink_mission_item_int_t mavlink_mission;

    mavlink_mission.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_mission.command = MAV_CMD_NAV_RETURN_TO_LAUNCH;
    
    mavlink_mission.seq = m_sequence;

    return mavlink_mission;
}


Json uavos::fcb::mission::CTakeOff_Step::getAndruavMission()
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

mavlink_mission_item_int_t uavos::fcb::mission::CTakeOff_Step::getArdupilotMission()
{
    mavlink_mission_item_int_t mavlink_mission;

    mavlink_mission.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_mission.command = MAV_CMD_NAV_TAKEOFF;
    
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


Json uavos::fcb::mission::CLand_Step::getAndruavMission()
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

mavlink_mission_item_int_t uavos::fcb::mission::CLand_Step::getArdupilotMission()
{
    mavlink_mission_item_int_t mavlink_mission;

    mavlink_mission.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_mission.command = MAV_CMD_NAV_LAND;
    
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


Json uavos::fcb::mission::CWayPoint_Step::getAndruavMission()
{
    /*
        t : TYPE_CMissionAction_RTL
        s : sequence
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



mavlink_mission_item_int_t uavos::fcb::mission::CWayPoint_Step::getArdupilotMission()
{
    mavlink_mission_item_int_t mavlink_mission;

    mavlink_mission.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_mission.command = MAV_CMD_NAV_WAYPOINT;
    
    mavlink_mission.seq = m_sequence;
    mavlink_mission.param1 = m_time_to_stay;
    mavlink_mission.param2 = m_accepted_radius;
    mavlink_mission.param3 = m_pass_radius;
    mavlink_mission.param4 = m_desired_yaw;
    mavlink_mission.x = m_latitude;
    mavlink_mission.y = m_longitude;
    mavlink_mission.z = m_altitude;

    return mavlink_mission;
    
}