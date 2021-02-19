#include <iostream>


#include "mavlink_command.h"







void mavlinksdk::CMavlinkCommand::sendLongCommand (const uint16_t& command,
				const bool& confirmation,
                const float& param1,
                const float& param2,
                const float& param3,
                const float& param4,
                const float& param5,
                const float& param6,
                const float& param7)
{
	mavlink_command_long_t msg = { 0 };
	msg.target_system    = m_mavlink_sdk.getSysId();
	msg.target_component = m_mavlink_sdk.getCompId();
	msg.command          = command;
	msg.confirmation     = confirmation;
	
    msg.param1           = param1;
    msg.param2           = param2;
    msg.param3           = param3;
	msg.param4           = param4;
	msg.param5           = param5;
	msg.param6           = param6;
	msg.param7           = param7;
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_command_long_encode(m_mavlink_sdk.getSysId(), m_mavlink_sdk.getCompId(), &mavlink_message, &msg);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}



/**
 * https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
 */
void mavlinksdk::CMavlinkCommand::doArmDisarm (const bool& arm, const bool& force)
{

    float forceArm = 0;
    if (force == true)
    {
        forceArm = 21196;
    }

    float flagArm = 0.0f;
    if (arm == true)
    {
        flagArm = 1.0f;
    }

	sendLongCommand (MAV_CMD_COMPONENT_ARM_DISARM, true,
		(float) flagArm, 
		forceArm);

	return ;	
}

/**
 * @brief change vehicle mode.
 * 
 * @param mode depends on vehicle and firmware_type.
 */
void mavlinksdk::CMavlinkCommand::doSetMode (const int& mode)
{
    
	sendLongCommand (MAV_CMD_DO_SET_MODE, true,
		(float) 1.0f, 
		(float) mode,
		(float) 0.0f);

	return ;
}


/**
 * @brief define home location for RTL
 * @param yaw		in deg
 * @param latitude  in xx.xxxx format
 * @param longitude in xx.xxxx format
 * @param altitude  in meters
 */
void mavlinksdk::CMavlinkCommand::setHome (const float& yaw, const float& latitude, const float& longitude, const float& altitude)
{
    sendLongCommand (MAV_CMD_DO_SET_HOME, false,
		0,  // use specified location
		0,  // unused
		0,  // unused
		yaw,
		latitude,
		longitude,
		altitude);

	return ;
}

void mavlinksdk::CMavlinkCommand::setROI (const float& latitude, const float& longitude, const float& altitude)
{
    sendLongCommand (MAV_CMD_DO_SET_ROI, false,
		0,  // use specified location
		0,  // unused
		0,  // unused
		0,  // unused
		latitude,
		longitude,
		altitude);

	return ;
}

void mavlinksdk::CMavlinkCommand::resetROI ()
{
    setROI(0,0,0);
}


void mavlinksdk::CMavlinkCommand::cmdTerminalFlight ()
{
    sendLongCommand (MAV_CMD_DO_FLIGHTTERMINATION, false,
		1);

	return ;
}

void mavlinksdk::CMavlinkCommand::changeAltitude (const float& altitude)
{
	const mavlink_global_position_int_t& mavlink_global_position_int = m_mavlink_sdk.getVehicle().get()->getMsgGlobalPositionInt();
	
	gotoGuidedPoint(mavlink_global_position_int.lat / 10000000.0f, mavlink_global_position_int.lon / 10000000.0f, altitude );
}

/**
 * @brief take of when vehicle is not flying
 * 
 * @param altitude in meters [relative altitude]
 */
void mavlinksdk::CMavlinkCommand::takeOff (const float& altitude)
{
    sendLongCommand (MAV_CMD_NAV_TAKEOFF, false,
		0,  // unused
		0,  // unused
		0,  // unused
		0,  // unused
		0,  // unused
		0,  // unused
		altitude);

	return ;
}

/**
 * @brief 
 * 
 * @param latitude  in xx.xxxxx format
 * @param longitude in xx.xxxxx format
 * @param altitude  in meters [relative altitude]
 */
void mavlinksdk::CMavlinkCommand::gotoGuidedPoint (const double& latitude, const double& longitude, const double& altitude)
{
	
	
// 	mavlink_position_target_global_int_t msg = { 0 };
// 	msg.type_mask = POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_AX_IGNORE |
// 					POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_AY_IGNORE |
// 					POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_AZ_IGNORE |
// 					POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_VX_IGNORE |
// 					POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_VY_IGNORE |
// 					POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_VZ_IGNORE ;
					
//     msg.coordinate_frame = MAV_FRAME::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
// 	msg.lat_int 		 = (float) latitude;
// 	msg.lon_int 		 = (float) longitude;
// 	msg.alt 			 = (float) altitude;
	
//     // Encode
// 	mavlink_message_t mavlink_message;
	
// 	mavlink_msg_position_target_global_int_encode(m_mavlink_sdk.getSysId(), m_mavlink_sdk.getCompId(), &mavlink_message, &msg);

//    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);


	mavlink_mission_item_t msg = {0};
	msg.target_system    = m_mavlink_sdk.getSysId();
	msg.target_component = m_mavlink_sdk.getCompId();
	msg.seq = 0;
    msg.current = 2; // TODO use guided mode enum
    msg.frame = MAV_FRAME::MAV_FRAME_GLOBAL_RELATIVE_ALT;
    msg.command = MAV_CMD::MAV_CMD_NAV_WAYPOINT; //
    msg.param1 = 0; // TODO use correct parameter
    msg.param2 = 0; // TODO use correct parameter
    msg.param3 = 0; // TODO use correct parameter
    msg.param4 = 0; // TODO use correct parameter
    msg.x = (float) latitude;
    msg.y = (float) longitude;
    msg.z = (float) altitude;
    msg.autocontinue = 1; // TODO use correct parameter

	mavlink_message_t mavlink_message;
	
 	mavlink_msg_mission_item_encode (m_mavlink_sdk.getSysId(), m_mavlink_sdk.getCompId(), &mavlink_message, &msg);

	m_mavlink_sdk.sendMavlinkMessage(mavlink_message);
    
}


/**
 * @brief set yaw direction
 * 
 * @param target_angle in deg
 * @param turn_rate deg/sec
 * @param is_clock_wise 
 * @param is_relative relative or absolute angle
 */
void mavlinksdk::CMavlinkCommand::setYawCondition( const double& target_angle, const double& turn_rate, const bool& is_clock_wise, const bool& is_relative)
{
	float direction = is_clock_wise?1.0f:-1.0f;
	float relative = is_relative?1.0f:0.0f;
	
	sendLongCommand (MAV_CMD_CONDITION_YAW, false,
		target_angle,  
		turn_rate,  
		direction,  
		relative);

	return ;
}


/**
 * @brief 
 * @param speed_type Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
 * @param speed Speed (-1 indicates no change) m/s
 * @param is_ground_speed 
 * @param throttle Throttle (-1 indicates no change) %
 * @param is_relative 0: absolute, 1: relative
 */
void mavlinksdk::CMavlinkCommand::setNavigationSpeed ( const int& speed_type, const double& speed, const double& throttle, const bool& is_relative)
{
	const int relative = is_relative == true?1:0;

	sendLongCommand (MAV_CMD_DO_CHANGE_SPEED, false,
		speed_type,  
		speed,  
		throttle,  
		relative);

	return ;
}



void mavlinksdk::CMavlinkCommand::loadWayPoints ()
{
	m_mavlink_sdk.getWayPointManager().get()->loadWayPoints();

	mavlink_mission_request_list_t mission_request;

	mission_request.target_system = m_mavlink_sdk.getSysId();
	mission_request.target_component = m_mavlink_sdk.getCompId();
	mission_request.mission_type = MAV_MISSION_TYPE_MISSION;
	
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_mission_request_list_encode(m_mavlink_sdk.getSysId(), m_mavlink_sdk.getCompId(), &mavlink_message, &mission_request);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}


void mavlinksdk::CMavlinkCommand::clearWayPoints ()
{
	mavlink_mission_clear_all_t mission_clear;

	mission_clear.target_system = m_mavlink_sdk.getSysId();
	mission_clear.target_component = m_mavlink_sdk.getCompId();
	mission_clear.mission_type = MAV_MISSION_TYPE_MISSION; //MAV_MISSION_TYPE_ALL;
	
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_mission_clear_all_encode(m_mavlink_sdk.getSysId(), m_mavlink_sdk.getCompId(), &mavlink_message, &mission_clear);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}

/**
 * @brief Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
 * 
 * @param mission_number Sequence
 */
void mavlinksdk::CMavlinkCommand::setCurrentMission (const int& mission_number)
{
	mavlink_mission_set_current_t mission_current;

	mission_current.target_system = m_mavlink_sdk.getSysId();
	mission_current.target_component = m_mavlink_sdk.getCompId();
	mission_current.seq = mission_number;

	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_mission_set_current_encode(m_mavlink_sdk.getSysId(), m_mavlink_sdk.getCompId(), &mavlink_message, &mission_current);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}
