#include <iostream>

#include "mavlink_command.h"
#include "mavlink_waypoint_manager.h"


using namespace mavlinksdk;

#define GCS_SYSID 255

/**
 * @brief sens long command up to seven params.
 * @see https://mavlink.io/en/services/command.html
 * 
 * @param command 
 * @param confirmation 
 * @param param1 
 * @param param2 
 * @param param3 
 * @param param4 
 * @param param5 
 * @param param6 
 * @param param7 
 */
void CMavlinkCommand::sendLongCommand (const uint16_t& command,
				const bool& confirmation,
                const float& param1,
                const float& param2,
                const float& param3,
                const float& param4,
                const float& param5,
                const float& param6,
                const float& param7) const
{
	
	
 
	mavlink_command_long_t msg = { 0 };
	msg.target_system    = m_vehicle.getSysId();
	msg.target_component = m_vehicle.getCompId();
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
	mavlink_msg_command_long_encode(GCS_SYSID,190, &mavlink_message, &msg);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}


void CMavlinkCommand::sendIntCommand (const uint16_t& command,
				const uint8_t& frame,
                const float& param1,
                const float& param2,
                const float& param3,
                const float& param4,
                const int32_t& x,
                const int32_t& y,
                const float& z) const
{
	
	
	
	mavlink_command_int_t msg = { 0 };
	msg.target_system    = m_vehicle.getSysId();
	msg.target_component = m_vehicle.getCompId();
	msg.command          = command;
	msg.frame     		 = frame;
	
    msg.param1           = param1;
    msg.param2           = param2;
    msg.param3           = param3;
	msg.param4           = param4;
	msg.x           	 = x;
	msg.y           	 = y;
	msg.z           	 = z;

	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_command_int_encode(GCS_SYSID,190, &mavlink_message, &msg);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}



/**
 * @brief forward any mavlink message to FCB. No previous processing
 * 
 * @param mavlink_message 
 */
void CMavlinkCommand::sendNative(const mavlink_message_t mavlink_message) const
{
	m_mavlink_sdk.sendMavlinkMessage(mavlink_message);
	return ;
}

void CMavlinkCommand::requestMessageEmit(const uint32_t message_id) const 
{

	// std::cout << "requestMessageInterval" << std::endl;
	// 
	// mavlink_message_interval_t mavlink_message_interval;
	// mavlink_message_interval.message_id = message_id;
	// mavlink_message_interval.interval_us = interval_us;
	// // mavlink_message_interval.target_system    = m_vehicle.getSysId();
	// // mavlink_message_interval.target_component = m_vehicle.getCompId();
	
	// mavlink_message_t mavlink_message;
	// mavlink_msg_message_interval_encode(m_vehicle.getSysId(),mavlink_sdk.getCompId(), &mavlink_message, &mavlink_message_interval);
	
	// mavlink_sdk.sendMavlinkMessage(mavlink_message);


	
	sendLongCommand (MAV_CMD_REQUEST_MESSAGE, false,
		MAVLINK_MSG_ID_WIND,  // use specified location . if 1 then use current location.
		0,  // unused
		0,  // unused
		0,
		0,
		0,
		0);

	return ;

}

void CMavlinkCommand::sendHeartBeatOfGCS() const
{
	mavlink_heartbeat_t mavlink_heartbeat;
	mavlink_heartbeat.autopilot = 8;
	mavlink_heartbeat.base_mode = 0;
	mavlink_heartbeat.custom_mode = 0;
	mavlink_heartbeat.mavlink_version = 3;
	mavlink_heartbeat.system_status = 0;
	mavlink_heartbeat.type = 6;
	
	mavlink_message_t mavlink_message;
	mavlink_msg_heartbeat_encode(GCS_SYSID,0, &mavlink_message, &mavlink_heartbeat);
	
	m_mavlink_sdk.sendMavlinkMessage(mavlink_message);
	return ;
}
        

void CMavlinkCommand::sendHeartBeatOfComponent(const uint8_t component_id) const
{
	

	const mavlink_heartbeat_t mavlink_heartbeat= m_vehicle.getMsgHeartBeat();
	
	mavlink_message_t mavlink_message;
	mavlink_msg_heartbeat_encode(m_vehicle.getSysId(),component_id, &mavlink_message, &mavlink_heartbeat);

	m_mavlink_sdk.sendMavlinkMessage(mavlink_message);
	return ;
}

/**
 * @brief Arm & Disarm with/out force.
 * @see https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
 * @param arm 
 * @param force 
 */
void CMavlinkCommand::doArmDisarm (const bool& arm, const bool& force)  const
{

    float forceArm = 0;
    

    float flagArm = 0.0f;
    if (arm == true)
    {
        flagArm = 1.0f;
		if (force == true)
		{
			forceArm = 2989;
		}
    }
	else
	{
		if (force == true)
		{
			forceArm = 21196;
		}
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
void CMavlinkCommand::doSetMode (const int& mode, const int& custom_mode, const int& custom_sub_mode)  const
{
    
	sendLongCommand (MAV_CMD_DO_SET_MODE, true,
		(float) mode,
		(float) custom_mode,
		(float) custom_sub_mode);

	return ;
}


/**
 * @brief define home location for RTL
 * @param yaw		in deg
 * @param latitude  in xx.xxxx format
 * @param longitude in xx.xxxx format
 * @param altitude  in meters
 */
void CMavlinkCommand::setHome (const float& yaw, const float& latitude, const float& longitude, const float& altitude)  const
{
    sendLongCommand (MAV_CMD_DO_SET_HOME, false,
		0,  // use specified location . if 1 then use current location.
		0,  // unused
		0,  // unused
		yaw,
		latitude,
		longitude,
		altitude);

	return ;
}

void CMavlinkCommand::setROI (const float& latitude, const float& longitude, const float& altitude) const
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

void CMavlinkCommand::resetROI () const
{
    setROI(0,0,0);
}

/**
 * @brief Flight termination immediately and irreversably terminates the current flight.
 * @see https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FLIGHTTERMINATION
 * 
 */
void CMavlinkCommand::cmdTerminateFlight () const
{
    sendLongCommand (MAV_CMD_DO_FLIGHTTERMINATION, false,
		1);

	return ;
}


/**
 * @brief change altitude by calling Guided mode with target point and new altitude.
 * @details altitude will increase or decrease along the way to the target. 
 * If you want instant change then you need to change the altitude in the current point then move to target.
 * @param altitude 
 */
void CMavlinkCommand::changeAltitude (const float& altitude) const
{

	const mavlink_position_target_global_int_t& mavlink_global_position_int = mavlinksdk::CVehicle::getInstance().getMsgTargetPositionGlobalInt();
	if ((mavlink_global_position_int.lon_int!=0) || (mavlink_global_position_int.lat_int!=0))
	{
		gotoGuidedPoint(mavlink_global_position_int.lat_int / 10000000.0f, mavlink_global_position_int.lon_int / 10000000.0f, altitude );
	}
	else
	{
		const mavlink_global_position_int_t&  mavlink_global_position_int = mavlinksdk::CVehicle::getInstance().getMsgGlobalPositionInt();
		gotoGuidedPoint(mavlink_global_position_int.lat / 10000000.0f, mavlink_global_position_int.lon / 10000000.0f, altitude );
	}
	

	// sendLongCommand (MAV_CMD_NAV_TAKEOFF, true,
	// 	-1,  // unused
	// 	0,  // unused
	// 	0,  // unused
	// 	NAN,  // unused
	// 	NAN,  // unused
	// 	NAN,  // unused
	// 	altitude);

}

/**
 * @brief take of when vehicle is not flying
 * 
 * @param altitude in meters [relative altitude]
 */
void CMavlinkCommand::takeOff (const float& altitude) const
{
    
	switch(mavlinksdk::CVehicle::getInstance().getMsgHeartBeat().autopilot)
	{
		case MAV_AUTOPILOT::MAV_AUTOPILOT_PX4: 
			return takeOff_px4(altitude);
	}

	
	return takeOff_default(altitude);

}


/**
 * @brief 
 * 
 * @param latitude  in xx.xxxxx format
 * @param longitude in xx.xxxxx format
 * @param relative_altitude  in meters [relative altitude]
 */
void CMavlinkCommand::gotoGuidedPoint (const double& latitude, const double& longitude, const double& relative_altitude) const
{
	
	switch(mavlinksdk::CVehicle::getInstance().getMsgHeartBeat().autopilot)
	{
		case MAV_AUTOPILOT::MAV_AUTOPILOT_PX4: 
			return gotoGuidedPoint_px4(latitude,longitude,relative_altitude);
	}

	
	return gotoGuidedPoint_default(latitude,longitude,relative_altitude);

}


/**
 * @brief set yaw direction
 * 
 * @param target_angle in deg
 * @param turn_rate deg/sec
 * @param is_clock_wise 
 * @param is_relative relative or absolute angle
 */
void CMavlinkCommand::setYawCondition( const double& target_angle, const double& turn_rate, const bool& is_clock_wise, const bool& is_relative) const
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
 * @param is_relative false: absolute, true: relative
 */
void CMavlinkCommand::setNavigationSpeed ( const int& speed_type, const double& speed, const double& throttle, const bool& is_relative) const
{
	const int relative = is_relative == true?1:0;

	sendLongCommand (MAV_CMD_DO_CHANGE_SPEED, false,
		speed_type,  
		speed,  
		throttle,  
		relative);

	return ;
}

/**
 * @brief Accept message received from FCB.
 * 
 */
void CMavlinkCommand::sendMissionAck (const  uint8_t target_system, uint8_t target_component, const  uint8_t result) const
{
	#ifdef DEBUG
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: sendMissionAck " << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    
	mavlink_mission_ack_t mission_ack;
	mission_ack.type = result;
	mission_ack.target_system = target_system;
	mission_ack.target_component = target_component;
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_mission_ack_encode(255,190, &mavlink_message, &mission_ack);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}


void CMavlinkCommand::reloadWayPoints () const
{
	mavlinksdk::CMavlinkWayPointManager::getInstance().reloadWayPoints();

	return ;
}


void CMavlinkCommand::getWayPointByNumber (const int& mission_number) const
{
	#ifdef DEBUG
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: getWayPointByNumber " << std::to_string(mission_number) << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    
	
	
	
	mavlink_mission_request_int_t mission_request = {0};

	mission_request.target_system = m_vehicle.getSysId();
	mission_request.target_component = m_vehicle.getCompId();
	mission_request.mission_type = MAV_MISSION_TYPE_MISSION;
	mission_request.seq = mission_number;
	
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_mission_request_int_encode(255,190, &mavlink_message, &mission_request);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}

void CMavlinkCommand::clearWayPoints () const
{
	
	
	mavlinksdk::CMavlinkWayPointManager::getInstance().clearWayPoints();

	mavlink_mission_clear_all_t mission_clear;

	mission_clear.target_system = m_vehicle.getSysId();
	mission_clear.target_component = m_vehicle.getCompId();
	mission_clear.mission_type = MAV_MISSION_TYPE_MISSION; //MAV_MISSION_TYPE_ALL;
	
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_mission_clear_all_encode(GCS_SYSID,190, &mavlink_message, &mission_clear);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}


/**
 * @brief Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
 * 
 * @param mission_number Sequence
 */
void CMavlinkCommand::setCurrentMission (const int& mission_number) const
{
	#ifdef DEBUG
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: setCurrentMission " << std::to_string(mission_number) << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    

	mavlink_mission_set_current_t mission_current ={0};

	
	mission_current.target_system = m_vehicle.getSysId();
	mission_current.target_component = m_vehicle.getCompId();
	mission_current.seq = mission_number;

	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_mission_set_current_encode(GCS_SYSID,190, &mavlink_message, &mission_current);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}


void CMavlinkCommand::requestMissionList () const
{
	#ifdef DEBUG
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: requestMissionList"  << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    
	mavlink_mission_request_list_t mission_request_list ={0};

	
	
	mission_request_list.target_system = m_vehicle.getSysId();
	mission_request_list.target_component = m_vehicle.getCompId();
	mission_request_list.mission_type = MAV_MISSION_TYPE_MISSION;

	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_mission_request_list_encode(255,190, &mavlink_message, &mission_request_list);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}


/**
 * @brief Set mission count required by ardupilot to start receiving messages from companion computer.
 * * This method is called internally by CMavlinkWayPointManager.
 *
 * @param mission_count 
 * @param mission_type 
 */
void CMavlinkCommand::setMissionCount (const int& mission_count, MAV_MISSION_TYPE mission_type) const
{
	#ifdef DEBUG
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: requestMissionList"  << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    
	mavlink_mission_count_t mission_count_msg = {0};

	

	mission_count_msg.target_system = m_vehicle.getSysId();
	mission_count_msg.target_component = m_vehicle.getCompId();
	mission_count_msg.mission_type = mission_type;
	mission_count_msg.count = mission_count;

	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_mission_count_encode(255,190, &mavlink_message, &mission_count_msg);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}


void CMavlinkCommand::writeMission (std::map <int, mavlink_mission_item_int_t> mavlink_mission) const
{
	mavlinksdk::CMavlinkWayPointManager::getInstance().saveWayPoints(mavlink_mission, MAV_MISSION_TYPE::MAV_MISSION_TYPE_MISSION);	

	return ;
}


void CMavlinkCommand::writeMissionItem (mavlink_mission_item_int_t mavlink_mission) const
{
	
	
	mavlink_mission.target_system = m_vehicle.getSysId();
	mavlink_mission.target_component = m_vehicle.getCompId();
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_mission_item_int_encode(255, 190, &mavlink_message, &mavlink_mission);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}

void CMavlinkCommand::writeMissionItem (mavlink_mission_item_t mavlink_mission) const
{
	
	
	mavlink_mission.target_system = m_vehicle.getSysId();
	mavlink_mission.target_component = m_vehicle.getCompId();
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_mission_item_encode(255, 190, &mavlink_message, &mavlink_mission);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}
        


/**
 * @brief 
 * Request all parameters of this component. After this request, all parameters are emitted.
 * The parameter microservice is documented at https://mavlink.io/en/services/parameter.html
 * 
 * @callgraph
 */
void CMavlinkCommand::requestParametersList () const
{
	
	mavlink_param_request_list_t mavlink_param;

	
	
	mavlink_param.target_system = m_vehicle.getSysId();
	mavlink_param.target_component = m_vehicle.getCompId();
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_param_request_list_encode(GCS_SYSID,190, &mavlink_message, &mavlink_param);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;	
}


/**
 * @brief 
 * (MAVLink 2) Request all parameters of this component. All parameters should be emitted in response as PARAM_EXT_VALUE.
 * ! messages are not handled correctly no by mavsdk module.
 * TODO: please fix
 */
void CMavlinkCommand::requestExtParametersList () const
{
	
	// mavlink_param_ext_request_list_t mavlink_param;

	// 
	
	// mavlink_param.target_system = m_vehicle.getSysId();
	// mavlink_param.target_component = m_vehicle.getCompId();
	
	// // Encode
	// mavlink_message_t mavlink_message;
	// mavlink_msg_param_ext_request_list_encode(GCS_SYSID,190, &mavlink_message, &mavlink_param);

    // mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;	
}


/**
 * @brief 
 *  Write parameter by name.
 * !NOT TESTED
 * @callgraph
 * @param param_name 
 * @param value 
 *  
 */
void CMavlinkCommand::writeParameter (const std::string& param_name, const double &value)  const
{
	//TODO to be implemented	
	
	const std::map<std::string, mavlink_param_value_t>& parameters_list = mavlinksdk::CMavlinkParameterManager::getInstance().getParametersList();
	

	mavlink_param_set_t mavlink_param;
	
	auto it = parameters_list.find(param_name);

	if (it != parameters_list.end())
	{
		return ; // not found
	} 
	
	mavlink_param.target_system = m_vehicle.getSysId();
	mavlink_param.target_component = m_vehicle.getCompId();
	
	mavlink_param.param_value = value;
	mavlink_param.param_type = it->second.param_type;
	memcpy (mavlink_param.param_id, it->first.c_str(), 16);
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_param_set_encode(GCS_SYSID,190, &mavlink_message, &mavlink_param);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}


/**
 * @brief 
 * ! Not Tested
 * @param param_name name is saved in parameters_list = mavlinksdk::CVehicle::getInstance().getParametersList()
 */
void CMavlinkCommand::readParameter (const std::string& param_name) const
{
	//TODO to be implemented	
	
	const std::map<std::string, mavlink_param_value_t>& parameters_list = mavlinksdk::CMavlinkParameterManager::getInstance().getParametersList();
	

	mavlink_param_request_read_t mavlink_param;
	
	auto it = parameters_list.find(param_name);

	if (it != parameters_list.end())
	{
		return ; // not found
	} 
	
	mavlink_param.target_system = m_vehicle.getSysId();
	mavlink_param.target_component = m_vehicle.getCompId();
	memcpy (mavlink_param.param_id, it->first.c_str(), 16);
	mavlink_param.param_index = -1; //Send -1 to use the param ID field as identifier (else the param id will be ignored)
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_param_request_read_encode(GCS_SYSID,190, &mavlink_message, &mavlink_param);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;	
}



/**
 * @brief 
 * ! Not Tested
 * @param param_name name is saved in parameters_list = mavlinksdk::CVehicle::getInstance().getParametersList()
 */
void CMavlinkCommand::readParameterByIndex (const uint16_t& param_index) const
{
	
	#ifdef DEBUG
			std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  
			<< _LOG_CONSOLE_TEXT << "DEBUG: parameter index:" << std::to_string(param_index) 
			<< std::endl;
    #endif


	
	//const std::map<std::string, mavlink_param_value_t>& parameters_list = mavlinksdk::CMavlinkParameterManager::getInstance().getParametersList();
	

	mavlink_param_request_read_t mavlink_param;
	
	mavlink_param.target_system = m_vehicle.getSysId();
	mavlink_param.target_component = m_vehicle.getCompId();
	memset (mavlink_param.param_id, 0, 16);
	mavlink_param.param_index = param_index; 
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_param_request_read_encode(GCS_SYSID,190, &mavlink_message, &mavlink_param);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;	
}

void CMavlinkCommand::requestDataStream(MAV_DATA_STREAM stream_id) const
{
	
	

	mavlink_request_data_stream_t mavlink_request_data_stream;
	
	mavlink_request_data_stream.target_system = m_vehicle.getSysId();
	mavlink_request_data_stream.target_component = m_vehicle.getCompId();
	mavlink_request_data_stream.req_stream_id = stream_id;
	mavlink_request_data_stream.req_message_rate = 4;
	mavlink_request_data_stream.start_stop = 1;
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_request_data_stream_encode(GCS_SYSID,190, &mavlink_message, &mavlink_request_data_stream);

    m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

}

/**
 * @brief 
 * Set servo value using channel.
 * @param channel Servo instance number.
 * @param pwm Pulse Width Modulation.  us
 */
void CMavlinkCommand::setServo (const int& channel, const int& pwm) const
{
	sendLongCommand (MAV_CMD_DO_SET_SERVO, false,
		(float) channel, 
		(float) pwm);

	return ;
}


/**
 * @brief 
 * request vehicle to send its hole location.
 * 
 */
void CMavlinkCommand::requestHomeLocation () const
{
	sendLongCommand (MAV_CMD_GET_HOME_POSITION, false);

	return ;
}

/**
 * @brief Release all 18 channels.
 * * from channel 1 to 8:
 * 			value UINT16_MAX means ignore & value 0 means release.
 * from channel 9 to 18:
 * 		    value 0 or UINT16_MAX meanse ignore & value (UINT16_MAX-1) means release.
 */
void CMavlinkCommand::releaseRCChannels() const
{
	
	
	mavlink_rc_channels_override_t mavlink_rc_channels = {0};
	mavlink_rc_channels.target_system    = m_vehicle.getSysId();
	mavlink_rc_channels.target_component = m_vehicle.getCompId();
	
	mavlink_rc_channels.chan1_raw = 0;				// release
    mavlink_rc_channels.chan2_raw = 0;				// release
    mavlink_rc_channels.chan3_raw = 0;				// release
	mavlink_rc_channels.chan4_raw = 0;				// release
    mavlink_rc_channels.chan5_raw = 0;				// release
    mavlink_rc_channels.chan6_raw = 0;				// release
    mavlink_rc_channels.chan7_raw = 0;				// release
    mavlink_rc_channels.chan8_raw = 0;				// release
    mavlink_rc_channels.chan9_raw  = UINT16_MAX-1;  // release
    mavlink_rc_channels.chan10_raw = UINT16_MAX-1;  // release
    mavlink_rc_channels.chan11_raw = UINT16_MAX-1;  // release
    mavlink_rc_channels.chan12_raw = UINT16_MAX-1;  // release
    mavlink_rc_channels.chan13_raw = UINT16_MAX-1;  // release
    mavlink_rc_channels.chan14_raw = UINT16_MAX-1;  // release
    mavlink_rc_channels.chan15_raw = UINT16_MAX-1;  // release
    mavlink_rc_channels.chan16_raw = UINT16_MAX-1;  // release
	mavlink_rc_channels.chan17_raw = UINT16_MAX-1;  // release
    mavlink_rc_channels.chan18_raw = UINT16_MAX-1;  // release
	
    mavlink_message_t mavlink_message;
	mavlink_msg_rc_channels_override_encode (GCS_SYSID, 190, &mavlink_message, &mavlink_rc_channels);

	m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}

/**
 * @brief Send RCChannels from 1 to channel_length others will set to zero.
 * *S to zero will release these channels.
 * from channel 1 to 8:
 * 			value UINT16_MAX means ignore & value 0 means release.
 * from channel 9 to 18:
 * 		    value 0 or UINT16_MAX meanse ignore & value (UINT16_MAX-1) means release.
 * @param channels array of MAX_RC_CHANNELS contains PWM values.
 * @param channel_length a number <= MAX_RC_CHANNELS (16) Minimum value is 1
 */
void CMavlinkCommand::sendRCChannels(const int16_t channels[MAX_RC_CHANNELS], int channel_length) const
{
	if ((channel_length >MAX_RC_CHANNELS) || (channel_length >MAX_RC_CHANNELS))
	{
		channel_length = MAX_RC_CHANNELS;
	}

	
	
	mavlink_rc_channels_override_t mavlink_rc_channels = {0};
	mavlink_rc_channels.target_system    = m_vehicle.getSysId();
	mavlink_rc_channels.target_component = m_vehicle.getCompId();
	
	mavlink_rc_channels.chan1_raw = channel_length>=1?channels[0]:UINT16_MAX;  			// apply or ignore
    mavlink_rc_channels.chan2_raw = channel_length>=2?channels[1]:UINT16_MAX;  			// apply or ignore
    mavlink_rc_channels.chan3_raw = channel_length>=3?channels[2]:UINT16_MAX;  			// apply or ignore
	mavlink_rc_channels.chan4_raw = channel_length>=4?channels[3]:UINT16_MAX;  			// apply or ignore
    mavlink_rc_channels.chan5_raw = channel_length>=5?channels[4]:UINT16_MAX;  			// apply or ignore
    mavlink_rc_channels.chan6_raw = channel_length>=6?channels[5]:UINT16_MAX;  			// apply or ignore
    mavlink_rc_channels.chan7_raw = channel_length>=7?channels[6]:UINT16_MAX;  			// apply or ignore
    mavlink_rc_channels.chan8_raw = channel_length>=8?channels[7]:UINT16_MAX;  			// apply or ignore
    mavlink_rc_channels.chan9_raw = channel_length>=9?channels[8]:UINT16_MAX;		   	// apply or ignore	
    mavlink_rc_channels.chan10_raw = channel_length>=10?channels[9]:UINT16_MAX;  	   	// apply or ignore
    mavlink_rc_channels.chan11_raw = channel_length>=11?channels[10]:UINT16_MAX;  	   	// apply or ignore
    mavlink_rc_channels.chan12_raw = channel_length>=12?channels[11]:UINT16_MAX;  	   	// apply or ignore
    mavlink_rc_channels.chan13_raw = channel_length>=13?channels[12]:UINT16_MAX;  	   	// apply or ignore
    mavlink_rc_channels.chan14_raw = channel_length>=14?channels[13]:UINT16_MAX;  	   	// apply or ignore
    mavlink_rc_channels.chan15_raw = channel_length>=15?channels[14]:UINT16_MAX;  	   	// apply or ignore
    mavlink_rc_channels.chan16_raw = channel_length>=16?channels[15]:UINT16_MAX;  	   	// apply or ignore
	mavlink_rc_channels.chan17_raw = channel_length>=17?channels[16]:UINT16_MAX;  	   	// apply or ignore
    mavlink_rc_channels.chan18_raw = channel_length>=18?channels[17]:UINT16_MAX;  	   	// apply or ignore
	
    mavlink_message_t mavlink_message;
	mavlink_msg_rc_channels_override_encode (GCS_SYSID, 190, &mavlink_message, &mavlink_rc_channels);

	m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

	// #ifdef DEBUG
    //     std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT 
	// 			<< std::to_string(mavlink_rc_channels.chan1_raw) << " - "
	// 			<< std::to_string(mavlink_rc_channels.chan2_raw) << " - "
	// 			<< std::to_string(mavlink_rc_channels.chan3_raw) << " - "
	// 			<< std::to_string(mavlink_rc_channels.chan4_raw) << " - "
	// 			<< std::to_string(mavlink_rc_channels.chan5_raw) << " - "
	// 			<< std::to_string(mavlink_rc_channels.chan6_raw) << " - "
	// 			<< std::to_string(mavlink_rc_channels.chan7_raw) << " - "
	// 			<< std::to_string(mavlink_rc_channels.chan8_raw) << " - "
	// 			<< std::to_string(mavlink_rc_channels.chan9_raw) << " - "
	// 			<< std::to_string(mavlink_rc_channels.chan10_raw) << " - "
	// 			<< std::to_string(mavlink_rc_channels.chan11_raw) << " - "
	// 			<< std::to_string(mavlink_rc_channels.chan12_raw) 
	// 			<< _NORMAL_CONSOLE_TEXT_ << std::endl;
    // #endif

	return ;
}

/**
 * @brief Control velocity and turning rate of vehicle in guided mode.
 * 
 * @param vx 
 * @param vy 
 * @param vz 
 * @param yaw_rate 
 */
void CMavlinkCommand::ctrlGuidedVelocityInLocalFrame (const float vx, const float vy, const float vz, const float yaw_rate, MAV_FRAME mav_frame) const
{
	
	mavlink_set_position_target_local_ned_t mavlink_set_position = {0};


	mavlink_set_position.target_system    = m_vehicle.getSysId();
	mavlink_set_position.target_component = m_vehicle.getCompId();
	
	mavlink_set_position.type_mask =  POSITION_TARGET_TYPEMASK_X_IGNORE |
									  POSITION_TARGET_TYPEMASK_Y_IGNORE |
									  POSITION_TARGET_TYPEMASK_Z_IGNORE |
									  POSITION_TARGET_TYPEMASK_AX_IGNORE |
									  POSITION_TARGET_TYPEMASK_AY_IGNORE |
									  POSITION_TARGET_TYPEMASK_AZ_IGNORE |
									  POSITION_TARGET_TYPEMASK_YAW_IGNORE;

    mavlink_set_position.coordinate_frame = mav_frame;

	mavlink_set_position.vx = vx;
	mavlink_set_position.vy = vy;
	mavlink_set_position.vz = vz;

	mavlink_set_position.yaw_rate = yaw_rate;

	mavlink_message_t mavlink_message;
	mavlink_msg_set_position_target_local_ned_encode (GCS_SYSID, m_vehicle.getCompId(), &mavlink_message, &mavlink_set_position);

	m_mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}


void CMavlinkCommand::gotoGuidedPoint_default (const double& latitude, const double& longitude, const double& relative_altitude) const
{
	// 	mavlink_position_target_global_int_t msg = { 0 };
	// 	msg.type_mask = POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_AX_IGNORE |
	// 					POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_AY_IGNORE |
	// 					POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_AZ_IGNORE |
	// 					POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_VX_IGNORE |
	// 					POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_VY_IGNORE |
	// 					POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_VZ_IGNORE ;
						
	//  msg.coordinate_frame = MAV_FRAME::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
	// 	msg.lat_int 		 = (float) latitude;
	// 	msg.lon_int 		 = (float) longitude;
	// 	msg.alt 			 = (float) altitude;
		
	//     // Encode
	// 	mavlink_message_t mavlink_message;
		
	// 	mavlink_msg_position_target_global_int_encode(GCS_SYSID,190, &mavlink_message, &msg);

	//    mavlink_sdk.sendMavlinkMessage(mavlink_message);

	
	
	mavlink_mission_item_t msg = {0};
	msg.target_system    = m_vehicle.getSysId();
	msg.target_component = m_vehicle.getCompId();
    msg.current = 2; // TODO use guided mode enum
    msg.frame = MAV_FRAME::MAV_FRAME_GLOBAL_RELATIVE_ALT;
    msg.command = MAV_CMD::MAV_CMD_NAV_WAYPOINT; //
    msg.param1 = 0; // TODO use correct parameter
    msg.param2 = 0; // TODO use correct parameter
    msg.param3 = 0; // TODO use correct parameter
    msg.param4 = 0; // TODO use correct parameter
    msg.x = (float) latitude;
    msg.y = (float) longitude;
    msg.z = (float) relative_altitude;
    msg.autocontinue = 1; // TODO use correct parameter

	mavlink_message_t mavlink_message;
	
 	mavlink_msg_mission_item_encode (GCS_SYSID,190, &mavlink_message, &msg);

	m_mavlink_sdk.sendMavlinkMessage(mavlink_message);
}


/**
 * @brief 
 * 
 * @param latitude 
 * @param longitude 
 * @param relative_altitude relative altitude
 */
void CMavlinkCommand::gotoGuidedPoint_px4 (const double& latitude, const double& longitude, const double& relative_altitude) const
{
	const mavlink_global_position_int_t& mavlink_global_position_int = mavlinksdk::CVehicle::getInstance().getMsgGlobalPositionInt();
	std::cout << "mavlink_global_position_int.alt:" << std::to_string((mavlink_global_position_int.alt - mavlink_global_position_int.relative_alt) / 1000) << "  alt:" << std::to_string(relative_altitude);
	sendIntCommand (MAV_CMD_DO_REPOSITION, MAV_FRAME_GLOBAL,
		-1.0f,  	// Ground speed, less than 0 (-1) for default
 		MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,  		// Loiter radius for planes. Positive values only, direction is controlled by Yaw value. A value of zero or NaN is ignored.
		0.0f,		// Loiter radius for planes. Positive values only, direction is controlled by Yaw value. A value of zero or NaN is ignored.
        NAN,  		// Yaw heading. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). For planes indicates loiter direction (0: clockwise, 1: counter clockwise)
		(int)(latitude*1e7),  	// Latitude
		(int)(longitude*1e7),  // Longitude
		relative_altitude + (mavlink_global_position_int.alt - mavlink_global_position_int.relative_alt) / 1000	// absolure altitude
		);

	return ;
}


void CMavlinkCommand::takeOff_default (const float& altitude) const
{
	sendLongCommand (MAV_CMD_NAV_TAKEOFF, true,
		0,  // unused
		0,  // unused
		0,  // unused
		0,  // unused
		0,  // unused
		0,  // unused
		altitude);

	return ;
}

void CMavlinkCommand::takeOff_px4 (const float& altitude) const
{
	sendLongCommand (MAV_CMD_NAV_TAKEOFF, true,
	   -1,  // unused
		0,  // unused
		0,  // unused
		NAN,  // unused
		NAN,  // unused
		NAN,  // unused
		altitude);

	return ;
}

