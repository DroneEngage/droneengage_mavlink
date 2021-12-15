#include <mavlink_sdk.h>
#include "../messages.hpp"


#include "../helpers/colors.hpp"
#include "../mission/missions.hpp"


using namespace uavos::fcb::mission;


//***********************************Delay_Step

void CDummy_Step::decodeMavlink (const mavlink_mission_item_int_t& mission_item_int)
{
    
    m_original_mission = mission_item_int;
    m_valid = true;
   
}

Json CDummy_Step::getAndruavMission()
{
    /*
        t : TYPE_CMissionAction_RTL
        s : sequence
    */
    Json message =
    {
        {"t", TYPE_CMissionAction_DummyMission},
        {"s", this->m_sequence}
        //TODO: COMPLETE THIS *** SWARM MESSAGES
    };

    return message;
}

mavlink_mission_item_int_t CDummy_Step::getArdupilotMission()
{
    if (m_valid == true) return m_original_mission;
    
    mavlink_mission_item_int_t mavlink_mission;

    mavlink_mission.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_mission.seq = m_sequence;
    mavlink_mission.command = MAV_CMD_CONDITION_DELAY;
    mavlink_mission.current = m_current?1:0;
    mavlink_mission.autocontinue = m_auto_continue?1:0;
    mavlink_mission.frame = m_frame;
    mavlink_mission.param1 = 0;  // Mandatory Zero as Ardupilot reads it as delay in seconds
    
    return mavlink_mission;
}


//***********************************Delay_Step
void CDelay_Step::decodeMavlink (const mavlink_mission_item_int_t& mission_item_int)
{
    
    m_sequence = mission_item_int.seq;
    m_auto_continue = mission_item_int.autocontinue;
    m_frame = mission_item_int.frame;

    m_delay = mission_item_int.param1; // Delay (-1 to enable time-of-day fields) in sec
    m_delay_hours = mission_item_int.param2;
    m_delay_minutes = mission_item_int.param3;
    m_delay_seconds = mission_item_int.param4;

   
}

Json CDelay_Step::getAndruavMission()
{
    /*
        t : TYPE_CMissionAction_RTL
        s : sequence
        d : if (-1) then Hh:mm:ss is time of day.
        [h] : condition delay in m_delay_hours
        [m] : condition delay in m_delay_minutes
          c : condition delay in m_delay_seconds
    */
    Json message =
    {
        {"t", TYPE_CMissionAction_Delay},
        {"s", this->m_sequence},
        {"d", (this->m_delay==-1)}, // time of day (hh:mm::ss)
        {"h", (this->m_delay==-1)?0:this->m_delay_hours},
        {"m", (this->m_delay==-1)?0:this->m_delay_minutes},
        {"c", (this->m_delay==-1)?this->m_delay:this->m_delay_seconds}
    };

    return message;
}


mavlink_mission_item_int_t CDelay_Step::getArdupilotMission()
{
    mavlink_mission_item_int_t mavlink_mission = {0};

    mavlink_mission.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_mission.seq = m_sequence;
    mavlink_mission.command = MAV_CMD_NAV_DELAY;
    mavlink_mission.current = m_current?1:0;
    mavlink_mission.autocontinue = m_auto_continue?1:0;
    mavlink_mission.frame = m_frame;

    mavlink_mission.param1 = m_delay; 
    mavlink_mission.param2 = m_delay_hours; 
    mavlink_mission.param3 = m_delay_minutes; 
    mavlink_mission.param4 = m_delay_seconds; 
    
    return mavlink_mission;
}


//***********************************Delay State Machine Step
void CDelay_State_Machine_Step::decodeMavlink (const mavlink_mission_item_int_t& mission_item_int)
{
    
    m_sequence = mission_item_int.seq;
    m_auto_continue = mission_item_int.autocontinue;
    m_frame = mission_item_int.frame;
   
}

Json CDelay_State_Machine_Step::getAndruavMission()
{
    /*
        t : TYPE_CMissionAction_RTL
        s : sequence
        d : condition delay in seconds
    */
    Json message =
    {
        {"t", TYPE_CMissionAction_Delay_STATE_MACHINE},
        {"s", this->m_sequence},
        {"d", this->m_delay_seconds}
    };

    return message;
}

mavlink_mission_item_int_t CDelay_State_Machine_Step::getArdupilotMission()
{
    mavlink_mission_item_int_t mavlink_mission = {0};

    mavlink_mission.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_mission.seq = m_sequence;
    mavlink_mission.command = MAV_CMD_CONDITION_DELAY;
    mavlink_mission.current = m_current?1:0;
    mavlink_mission.autocontinue = m_auto_continue?1:0;
    mavlink_mission.frame = m_frame;

    mavlink_mission.param1 = m_delay_seconds; 
    
    return mavlink_mission;
}

//*********************************** Guided Enabled Step
void CGuided_Enabled_Step::decodeMavlink (const mavlink_mission_item_int_t& mission_item_int)
{
    
    m_sequence = mission_item_int.seq;
    m_auto_continue = mission_item_int.autocontinue;
    m_frame = mission_item_int.frame;
   
    m_enabled = mission_item_int.param1==1?true:false;

}

Json CGuided_Enabled_Step::getAndruavMission()
{
    /*
        t : TYPE_CMissionAction_RTL
        s : sequence
        e : enable guided mode
    */
    Json message =
    {
        {"t", TYPE_CMissionAction_Guided_Enabled},
        {"s", this->m_sequence},
        {"e", this->m_enabled}
    };

    return message;
}

mavlink_mission_item_int_t CGuided_Enabled_Step::getArdupilotMission()
{
    mavlink_mission_item_int_t mavlink_mission = {0};

    mavlink_mission.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_mission.seq = m_sequence;
    mavlink_mission.command = MAV_CMD_NAV_GUIDED_ENABLE;
    mavlink_mission.current = m_current?1:0;
    mavlink_mission.autocontinue = m_auto_continue?1:0;
    mavlink_mission.frame = m_frame;

    mavlink_mission.param1 = this->m_enabled==true?1:0; 
    
    return mavlink_mission;
}

//*********************************** Change Altitude Step
void CChange_Altitude_Step::decodeMavlink (const mavlink_mission_item_int_t& mission_item_int)
{
    
    m_sequence = mission_item_int.seq;
    m_auto_continue = mission_item_int.autocontinue;
    m_frame = mission_item_int.frame;
   
    m_ascend_descent_rate = mission_item_int.param1;
    
}

Json CChange_Altitude_Step::getAndruavMission()
{
    /*
        t : TYPE_CMissionAction_RTL
        s : sequence
        e : enable guided mode
    */
    Json message =
    {
        {"t", TYPE_CMissionAction_ChangeAlt},
        {"s", this->m_sequence},
        {"r", this->m_ascend_descent_rate}
    };

    return message;
}

mavlink_mission_item_int_t CChange_Altitude_Step::getArdupilotMission()
{
    mavlink_mission_item_int_t mavlink_mission = {0};

    mavlink_mission.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_mission.seq = m_sequence;
    mavlink_mission.command = MAV_CMD_DO_CHANGE_ALTITUDE;
    mavlink_mission.current = m_current?1:0;
    mavlink_mission.autocontinue = m_auto_continue?1:0;
    mavlink_mission.frame = m_frame;

    mavlink_mission.param1 = m_ascend_descent_rate; 
    
    return mavlink_mission;
}

//*********************************** Continue And Change Altitude Step
void CContinue_And_Change_Altitude_Step::decodeMavlink (const mavlink_mission_item_int_t& mission_item_int)
{
    
    m_sequence = mission_item_int.seq;
    m_auto_continue = mission_item_int.autocontinue;
    m_frame = mission_item_int.frame;
   
    m_ascend_descent_command = mission_item_int.param1;
    m_desired_altitude = mission_item_int.z;
    
}

Json CContinue_And_Change_Altitude_Step::getAndruavMission()
{
    /*
        t : TYPE_CMissionAction_RTL
        s : sequence
        e : enable guided mode
    */
    Json message =
    {
        {"t", TYPE_CMissionAction_CONTINUE_AND_CHANGE_ALT},
        {"s", this->m_sequence},
        {"c", this->m_ascend_descent_command},
        {"a", this->m_desired_altitude}
    };

    return message;
}

mavlink_mission_item_int_t CContinue_And_Change_Altitude_Step::getArdupilotMission()
{
    mavlink_mission_item_int_t mavlink_mission = {0};

    mavlink_mission.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_mission.seq = m_sequence;
    mavlink_mission.command = MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
    mavlink_mission.current = m_current?1:0;
    mavlink_mission.autocontinue = m_auto_continue?1:0;
    mavlink_mission.frame = m_frame;

    mavlink_mission.param1 = m_ascend_descent_command; 
    mavlink_mission.z = m_desired_altitude; 
    
    return mavlink_mission;
}

//***********************************CChange_Speed_Step

void CChange_Speed_Step::decodeMavlink (const mavlink_mission_item_int_t& mission_item_int)
{
    
    m_sequence = mission_item_int.seq;
    m_auto_continue = mission_item_int.autocontinue;
    m_frame = mission_item_int.frame;

    m_speed_type = mission_item_int.param1;
    m_speed = mission_item_int.param2;
    m_throttle = mission_item_int.param3;
    m_relative = mission_item_int.param4 == 0? false:true;
   
}

Json CChange_Speed_Step::getAndruavMission()
{
    /*
        t : TYPE_CMissionAction_ChangeSpeed
        s : sequence
        p : speed
       [t]: if null then speedtype is ground speed.
        r : is relative
        l : throttle in % if -1 then ignored.

    */
    Json message =
    {
        {"t", TYPE_CMissionAction_ChangeSpeed},
        {"s", this->m_sequence},
        {"p", this->m_speed},
        {"t", this->m_speed_type},
        {"r", (this->m_relative==0)?true:false}, // inverted here .. leave it as is
        {"l", this->m_throttle},
        
    };

    return message;
}

mavlink_mission_item_int_t CChange_Speed_Step::getArdupilotMission()
{
    mavlink_mission_item_int_t mavlink_mission = {0};

    mavlink_mission.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_mission.seq = m_sequence;
    mavlink_mission.command = MAV_CMD_DO_CHANGE_SPEED;
    mavlink_mission.current = m_current?1:0;
    mavlink_mission.autocontinue = m_auto_continue?1:0;
    mavlink_mission.frame = m_frame;

    mavlink_mission.param1 = this->m_speed_type;
    mavlink_mission.param2 = this->m_speed;
    mavlink_mission.param3 = this->m_throttle;
    mavlink_mission.param4 = (this->m_relative==true)?1:0; 

    
    return mavlink_mission;
}


//***********************************CChange_Heading_Step

void CChange_Heading_Step::decodeMavlink (const mavlink_mission_item_int_t& mission_item_int)
{
    
    m_sequence = mission_item_int.seq;
    m_auto_continue = mission_item_int.autocontinue;
    m_frame = mission_item_int.frame;

    m_angle = mission_item_int.param1;
    m_angular_speed = mission_item_int.param2;
    m_direction = mission_item_int.param3;
    m_relative = mission_item_int.param4;
   
}

Json CChange_Heading_Step::getAndruavMission()
{
    /*
        t : TYPE_CMissionAction_ChangeHeading
        s : sequence
    */
    Json message =
    {
        {"t", TYPE_CMissionAction_ChangeHeading},
        {"s", this->m_sequence},
        {"b", this->m_angle},
        {"c", this->m_angular_speed},
        {"d", (this->m_relative==0)?true:false}, // inverted here .. leave it as is
        {"e", (this->m_direction==1)?true:false},
        
    };

    return message;
}

mavlink_mission_item_int_t CChange_Heading_Step::getArdupilotMission()
{
    mavlink_mission_item_int_t mavlink_mission = {0};

    mavlink_mission.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_mission.seq = m_sequence;
    mavlink_mission.command = MAV_CMD_CONDITION_YAW;
    mavlink_mission.current = m_current?1:0;
    mavlink_mission.autocontinue = m_auto_continue?1:0;
    mavlink_mission.frame = m_frame;

    mavlink_mission.param1 = this->m_angle;
    mavlink_mission.param2 = this->m_angular_speed;
    mavlink_mission.param3 = (this->m_direction==true)?1:-1;
    mavlink_mission.param4 = (this->m_relative==true)?1:0; 

    
    return mavlink_mission;
}


//***********************************CLoiter_Turns_Step

void CLoiter_Turns_Step::decodeMavlink (const mavlink_mission_item_int_t& mission_item_int)
{
    m_sequence = mission_item_int.seq;
    m_auto_continue = mission_item_int.autocontinue;
    m_frame = mission_item_int.frame;
   
    m_num_of_turn = mission_item_int.param1;
    m_heading_required = (mission_item_int.param2 ==1)?true:false;
    m_radius = mission_item_int.param3;
    m_xtrack_location = mission_item_int.param4;
    m_latitude = mission_item_int.x;
    m_longitude = mission_item_int.y;
    m_altitude = mission_item_int.z;
}

Json CLoiter_Turns_Step::getAndruavMission()
{
    /*
        t : TYPE_CMissionAction_RTL
        s : sequence
    */
    Json message =
    {
        {"t", TYPE_CMissionAction_Circle},
        {"s", this->m_sequence},
        {"n", this->m_num_of_turn},
        {"q", this->m_heading_required},
        {"r", this->m_radius},
        {"s", this->m_sequence},
        {"x", this->m_xtrack_location},
        {"a", this->m_latitude  / 10000000.0},
        {"g", this->m_longitude / 10000000.0},
        {"l", this->m_altitude}
        
    };

    return message;
}

mavlink_mission_item_int_t CLoiter_Turns_Step::getArdupilotMission()
{
    mavlink_mission_item_int_t mavlink_mission = {0};

    mavlink_mission.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_mission.seq = m_sequence;
    mavlink_mission.command = MAV_CMD_NAV_LOITER_TURNS;
    mavlink_mission.current = m_current?1:0;
    mavlink_mission.autocontinue = m_auto_continue?1:0;
    mavlink_mission.frame = m_frame;

    mavlink_mission.param1 = m_num_of_turn;
    mavlink_mission.param2 = (m_heading_required==false)?0:1;
    mavlink_mission.param3 = m_radius;
    mavlink_mission.param4 = m_xtrack_location;
    mavlink_mission.x = m_latitude;
    mavlink_mission.y = m_longitude;
    mavlink_mission.z = m_altitude;
    
    
    
    return mavlink_mission;
}


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
    mavlink_mission_item_int_t mavlink_mission = {0};

    mavlink_mission.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_mission.seq = m_sequence;
    mavlink_mission.command = MAV_CMD_NAV_RETURN_TO_LAUNCH;
    mavlink_mission.current = m_current?1:0;
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
        t : TYPE_CMissionAction_TakeOff
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
    mavlink_mission_item_int_t mavlink_mission = {0};

    mavlink_mission.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_mission.command = MAV_CMD_NAV_TAKEOFF;
    mavlink_mission.current = m_current?1:0;
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
        t : TYPE_CMissionAction_Landing
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
    mavlink_mission_item_int_t mavlink_mission = {0};

    mavlink_mission.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_mission.command = MAV_CMD_NAV_LAND;
    mavlink_mission.current = m_current?1:0;
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
    mavlink_mission_item_int_t mavlink_mission = {0};

    mavlink_mission.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_mission.command = MAV_CMD_NAV_WAYPOINT;
    mavlink_mission.current = m_current?1:0;
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