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
	mavlink_command_long_t com = { 0 };
	com.target_system    = m_mavlink_sdk.getSysId();
	com.target_component = m_mavlink_sdk.getCompId();
	com.command          = MAV_CMD_COMPONENT_ARM_DISARM;
	com.confirmation     = true;
	com.param1           = (float) flagArm;
	com.param2           = forceArm;

	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_command_long_encode(0,0, &mavlink_message, &com);

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
	mavlink_command_long_t com = { 0 };
	com.target_system    = m_mavlink_sdk.getSysId();
	com.target_component = m_mavlink_sdk.getCompId();
	com.command          = MAV_CMD_DO_SET_MODE;
	com.confirmation     = true;
	
    com.param1           = (float) 1.0f;
    com.param2           = (float) mode;
    com.param3           = (float) 0.0f;
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_command_long_encode(m_mavlink_sdk.getSysId(), m_mavlink_sdk.getCompId(), &mavlink_message, &com);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);
}



void mavlinksdk::CMavlinkCommand::setHome (const float& latitude, const float& longitude, const float& altitude)
{
    mavlink_command_long_t com = { 0 };
	com.target_system    = m_mavlink_sdk.getSysId();
	com.target_component = m_mavlink_sdk.getCompId();
	com.command          = MAV_CMD_DO_SET_HOME;
	
    com.param5           = latitude;
	com.param6           = longitude;
	com.param7           = altitude;
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_command_long_encode(m_mavlink_sdk.getSysId(), m_mavlink_sdk.getCompId(), &mavlink_message, &com);

   m_mavlink_sdk.sendMavlinkMessage(mavlink_message);
}

void mavlinksdk::CMavlinkCommand::setROI (const float& latitude, const float& longitude, const float& altitude)
{
    mavlink_command_long_t com = { 0 };
	com.target_system    = m_mavlink_sdk.getSysId();
	com.target_component = m_mavlink_sdk.getCompId();
	com.command          = MAV_CMD_DO_SET_ROI;
	
    com.param5           = latitude;
	com.param6           = longitude;
	com.param7           = altitude;
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_command_long_encode(m_mavlink_sdk.getSysId(), m_mavlink_sdk.getCompId(), &mavlink_message, &com);

   m_mavlink_sdk.sendMavlinkMessage(mavlink_message);
}

void mavlinksdk::CMavlinkCommand::resetROI ()
{
    setROI(0,0,0);
}


void mavlinksdk::CMavlinkCommand::cmdTerminalFlight ()
{
    mavlink_command_long_t com = { 0 };
	com.target_system    = m_mavlink_sdk.getSysId();
	com.target_component = m_mavlink_sdk.getCompId();
	com.command          = MAV_CMD_DO_FLIGHTTERMINATION;
	
    com.param1           = 1;

    // Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_command_long_encode(m_mavlink_sdk.getSysId(), m_mavlink_sdk.getCompId(), &mavlink_message, &com);

   m_mavlink_sdk.sendMavlinkMessage(mavlink_message);
}


void mavlinksdk::CMavlinkCommand::changeAltitude (const float& altitude)
{
    mavlink_command_long_t com = { 0 };
	com.target_system    = m_mavlink_sdk.getSysId();
	com.target_component = m_mavlink_sdk.getCompId();
	com.command          = MAV_CMD_NAV_TAKEOFF;
	
    com.param7           = altitude;

    // Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_command_long_encode(m_mavlink_sdk.getSysId(), m_mavlink_sdk.getCompId(), &mavlink_message, &com);

   m_mavlink_sdk.sendMavlinkMessage(mavlink_message);
}
