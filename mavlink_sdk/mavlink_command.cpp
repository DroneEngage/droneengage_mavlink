#include <iostream>


#include "mavlink_command.h"
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

    // Prepare command for off-board mode
	mavlink_command_long_t msg = { 0 };
	msg.target_system    = m_mavlink_sdk.getSysId();
	msg.target_component = m_mavlink_sdk.getCompId();
	msg.command          = MAV_CMD_COMPONENT_ARM_DISARM;
	msg.confirmation     = true;
	msg.param1           = (float) flagArm;
	msg.param2           = forceArm;

	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_command_long_encode(0,0, &mavlink_message, &msg);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);
}

void mavlinksdk::CMavlinkCommand::doSetMode (const int& mode)
{
    // mavlink_message_t mavlink_message;
    // mavlink_set_mode_t set_mode;
    // set_mode.base_mode = 1;
    // set_mode.custom_mode = mode;
	// mavlink_msg_set_mode_encode(m_mavlink_sdk.getSysId(), m_mavlink_sdk.getCompId(), &mavlink_message, &set_mode) ;

    // Prepare command for off-board mode
	mavlink_command_long_t msg = { 0 };
	msg.target_system    = m_mavlink_sdk.getSysId();
	msg.target_component = m_mavlink_sdk.getCompId();
	msg.command          = MAV_CMD_DO_SET_MODE;
	msg.confirmation     = true;
	
    msg.param1           = (float) 1.0f;
    msg.param2           = (float) mode;
    msg.param3           = (float) 0.0f;
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_command_long_encode(m_mavlink_sdk.getSysId(), m_mavlink_sdk.getCompId(), &mavlink_message, &msg);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);
}



void mavlinksdk::CMavlinkCommand::setHome (const float& latitude, const float& longitude, const float& altitude)
{
    mavlink_command_long_t msg = { 0 };
	msg.target_system    = m_mavlink_sdk.getSysId();
	msg.target_component = m_mavlink_sdk.getCompId();
	msg.command          = MAV_CMD_DO_SET_HOME;
	
    msg.param5           = latitude;
	msg.param6           = longitude;
	msg.param7           = altitude;
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_command_long_encode(m_mavlink_sdk.getSysId(), m_mavlink_sdk.getCompId(), &mavlink_message, &msg);

   m_mavlink_sdk.sendMavlinkMessage(mavlink_message);
}

void mavlinksdk::CMavlinkCommand::setROI (const float& latitude, const float& longitude, const float& altitude)
{
    mavlink_command_long_t msg = { 0 };
	msg.target_system    = m_mavlink_sdk.getSysId();
	msg.target_component = m_mavlink_sdk.getCompId();
	msg.command          = MAV_CMD_DO_SET_ROI;
	
    msg.param5           = latitude;
	msg.param6           = longitude;
	msg.param7           = altitude;
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_command_long_encode(m_mavlink_sdk.getSysId(), m_mavlink_sdk.getCompId(), &mavlink_message, &msg);

   m_mavlink_sdk.sendMavlinkMessage(mavlink_message);
}

void mavlinksdk::CMavlinkCommand::resetROI ()
{
    setROI(0,0,0);
}


void mavlinksdk::CMavlinkCommand::cmdTerminalFlight ()
{
    mavlink_command_long_t msg = { 0 };
	msg.target_system    = m_mavlink_sdk.getSysId();
	msg.target_component = m_mavlink_sdk.getCompId();
	msg.command          = MAV_CMD_DO_FLIGHTTERMINATION;
	
    msg.param1           = 1;

    // Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_command_long_encode(m_mavlink_sdk.getSysId(), m_mavlink_sdk.getCompId(), &mavlink_message, &msg);

   m_mavlink_sdk.sendMavlinkMessage(mavlink_message);
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
    mavlink_command_long_t msg = { 0 };
	msg.target_system    = m_mavlink_sdk.getSysId();
	msg.target_component = m_mavlink_sdk.getCompId();
	msg.command          = MAV_CMD_NAV_TAKEOFF;
	
    msg.param7           = altitude;

    // Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_command_long_encode(m_mavlink_sdk.getSysId(), m_mavlink_sdk.getCompId(), &mavlink_message, &msg);

   m_mavlink_sdk.sendMavlinkMessage(mavlink_message);
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