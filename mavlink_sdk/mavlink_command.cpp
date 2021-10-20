#include <iostream>

#include "mavlink_command.h"
#include "mavlink_sdk.h"
#include "mavlink_waypoint_manager.h"



using namespace mavlinksdk;


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
	
	mavlinksdk::CMavlinkSDK& mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
	
	mavlink_command_long_t msg = { 0 };
	msg.target_system    = mavlink_sdk.getSysId();
	msg.target_component = mavlink_sdk.getCompId();
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
	mavlink_msg_command_long_encode(mavlink_sdk.getSysId(), mavlink_sdk.getCompId(), &mavlink_message, &msg);

    mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}


/**
 * @brief forward any mavlink message to FCB. No previous processing
 * 
 * @param mavlink_message 
 */
void CMavlinkCommand::sendNative(const mavlink_message_t mavlink_message) const
{
	mavlinksdk::CMavlinkSDK& mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
	
	mavlink_sdk.sendMavlinkMessage(mavlink_message);
	return ;
}

/**
 * https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
 */
void CMavlinkCommand::doArmDisarm (const bool& arm, const bool& force)  const
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
void CMavlinkCommand::doSetMode (const int& mode)  const
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
void CMavlinkCommand::setHome (const float& yaw, const float& latitude, const float& longitude, const float& altitude)  const
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


void CMavlinkCommand::cmdTerminateFlight () const
{
    sendLongCommand (MAV_CMD_DO_FLIGHTTERMINATION, false,
		1);

	return ;
}

void CMavlinkCommand::changeAltitude (const float& altitude) const
{
	mavlinksdk::CMavlinkSDK& mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
	
	const mavlink_global_position_int_t& mavlink_global_position_int = mavlinksdk::CVehicle::getInstance().getMsgGlobalPositionInt();
	
	gotoGuidedPoint(mavlink_global_position_int.lat / 10000000.0f, mavlink_global_position_int.lon / 10000000.0f, altitude );
}

/**
 * @brief take of when vehicle is not flying
 * 
 * @param altitude in meters [relative altitude]
 */
void CMavlinkCommand::takeOff (const float& altitude) const
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
void CMavlinkCommand::gotoGuidedPoint (const double& latitude, const double& longitude, const double& altitude) const
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
	
// 	mavlink_msg_position_target_global_int_encode(mavlink_sdk.getSysId(), mavlink_sdk.getCompId(), &mavlink_message, &msg);

//    mavlink_sdk.sendMavlinkMessage(mavlink_message);

	mavlinksdk::CMavlinkSDK& mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
	
	mavlink_mission_item_t msg = {0};
	msg.target_system    = mavlink_sdk.getSysId();
	msg.target_component = mavlink_sdk.getCompId();
	msg.seq = 110;
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
	
 	mavlink_msg_mission_item_encode (mavlink_sdk.getSysId(), mavlink_sdk.getCompId(), &mavlink_message, &msg);

	mavlink_sdk.sendMavlinkMessage(mavlink_message);
    
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
void CMavlinkCommand::sendMissionAck () const
{
	#ifdef DEBUG
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: sendMissionAck " << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    
	mavlinksdk::CMavlinkSDK& mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
	
	mavlink_mission_ack_t mission_ack;
	mission_ack.type = MAV_MISSION_ACCEPTED;

	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_mission_ack_encode(mavlink_sdk.getSysId(), mavlink_sdk.getCompId(), &mavlink_message, &mission_ack);

    mavlink_sdk.sendMavlinkMessage(mavlink_message);

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
    
	
	mavlinksdk::CMavlinkSDK& mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
	
	mavlink_mission_request_int_t mission_request = {0};

	mission_request.target_system = mavlink_sdk.getSysId();
	mission_request.target_component = mavlink_sdk.getCompId();
	mission_request.mission_type = MAV_MISSION_TYPE_MISSION;
	mission_request.seq = mission_number;
	
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_mission_request_int_encode(mavlink_sdk.getSysId(), mavlink_sdk.getCompId(), &mavlink_message, &mission_request);

    mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}

void CMavlinkCommand::clearWayPoints () const
{
	mavlinksdk::CMavlinkSDK& mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
	
	mavlinksdk::CMavlinkWayPointManager::getInstance().clearWayPoints();

	mavlink_mission_clear_all_t mission_clear;

	mission_clear.target_system = mavlink_sdk.getSysId();
	mission_clear.target_component = mavlink_sdk.getCompId();
	mission_clear.mission_type = MAV_MISSION_TYPE_MISSION; //MAV_MISSION_TYPE_ALL;
	
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_mission_clear_all_encode(mavlink_sdk.getSysId(), mavlink_sdk.getCompId(), &mavlink_message, &mission_clear);

    mavlink_sdk.sendMavlinkMessage(mavlink_message);

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

	mavlinksdk::CMavlinkSDK& mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
	
	mission_current.target_system = mavlink_sdk.getSysId();
	mission_current.target_component = mavlink_sdk.getCompId();
	mission_current.seq = mission_number;

	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_mission_set_current_encode(mavlink_sdk.getSysId(), mavlink_sdk.getCompId(), &mavlink_message, &mission_current);

    mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}


void CMavlinkCommand::requestMissionList () const
{
	#ifdef DEBUG
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: requestMissionList"  << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    
	mavlink_mission_request_list_t mission_request_list ={0};

	mavlinksdk::CMavlinkSDK& mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
	
	mission_request_list.target_system = mavlink_sdk.getSysId();
	mission_request_list.target_component = mavlink_sdk.getCompId();
	mission_request_list.mission_type = MAV_MISSION_TYPE_MISSION;

	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_mission_request_list_encode(mavlink_sdk.getSysId(), mavlink_sdk.getCompId(), &mavlink_message, &mission_request_list);

    mavlink_sdk.sendMavlinkMessage(mavlink_message);

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

	mavlinksdk::CMavlinkSDK& mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
	
	mission_count_msg.target_system = mavlink_sdk.getSysId();
	mission_count_msg.target_component = mavlink_sdk.getCompId();
	mission_count_msg.mission_type = mission_type;
	mission_count_msg.count = mission_count;

	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_mission_count_encode(mavlink_sdk.getSysId(), mavlink_sdk.getCompId(), &mavlink_message, &mission_count_msg);

    mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}


void CMavlinkCommand::writeMission (std::map <int, mavlink_mission_item_int_t> mavlink_mission) const
{
	mavlinksdk::CMavlinkWayPointManager::getInstance().saveWayPoints(mavlink_mission, MAV_MISSION_TYPE::MAV_MISSION_TYPE_MISSION);	

	return ;
}


void CMavlinkCommand::writeMissionItem (mavlink_mission_item_int_t mavlink_mission) const
{
	mavlinksdk::CMavlinkSDK& mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
	
	mavlink_mission.target_system = mavlink_sdk.getSysId();
	mavlink_mission.target_component = mavlink_sdk.getCompId();
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_mission_item_int_encode(mavlink_sdk.getSysId(), mavlink_sdk.getCompId(), &mavlink_message, &mavlink_mission);

    mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}


/**
 * @brief 
 * Request all parameters of this component. After this request, all parameters are emitted. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html
 * 
 */
void CMavlinkCommand::requestParametersList () const
{
	
	mavlink_param_request_list_t mavlink_param;

	mavlinksdk::CMavlinkSDK& mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
	
	mavlink_param.target_system = mavlink_sdk.getSysId();
	mavlink_param.target_component = mavlink_sdk.getCompId();
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_param_request_list_encode(mavlink_sdk.getSysId(), mavlink_sdk.getCompId(), &mavlink_message, &mavlink_param);

    mavlink_sdk.sendMavlinkMessage(mavlink_message);

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

	// mavlinksdk::CMavlinkSDK& mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
	
	// mavlink_param.target_system = mavlink_sdk.getSysId();
	// mavlink_param.target_component = mavlink_sdk.getCompId();
	
	// // Encode
	// mavlink_message_t mavlink_message;
	// mavlink_msg_param_ext_request_list_encode(mavlink_sdk.getSysId(), mavlink_sdk.getCompId(), &mavlink_message, &mavlink_param);

    // mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;	
}


/**
 * @brief 
 *  Write parameter by name.
 * !NOT TESTED
 * @param param_name 
 * @param value 
 */
void CMavlinkCommand::writeParameter (const std::string& param_name, const double &value)  const
{
	//TODO to be implemented	
	mavlinksdk::CMavlinkSDK& mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
	const std::map<std::string, Parameter_Value>& parameters_list = mavlinksdk::CVehicle::getInstance().getParametersList();
	

	mavlink_param_set_t mavlink_param;
	
	auto it = parameters_list.find(param_name);

	if (it != parameters_list.end())
	{
		return ; // not found
	} 
	
	mavlink_param.target_system = mavlink_sdk.getSysId();
	mavlink_param.target_component = mavlink_sdk.getCompId();
	
	mavlink_param.param_value = value;
	mavlink_param.param_type = it->second.param_type;
	memcpy (mavlink_param.param_id, it->first.c_str(), 16);
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_param_set_encode(mavlink_sdk.getSysId(), mavlink_sdk.getCompId(), &mavlink_message, &mavlink_param);

    mavlink_sdk.sendMavlinkMessage(mavlink_message);

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
	mavlinksdk::CMavlinkSDK& mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
	const std::map<std::string, Parameter_Value>& parameters_list = mavlinksdk::CVehicle::getInstance().getParametersList();
	

	mavlink_param_request_read_t mavlink_param;
	
	auto it = parameters_list.find(param_name);

	if (it != parameters_list.end())
	{
		return ; // not found
	} 
	
	mavlink_param.target_system = mavlink_sdk.getSysId();
	mavlink_param.target_component = mavlink_sdk.getCompId();
	memcpy (mavlink_param.param_id, it->first.c_str(), 16);
	mavlink_param.param_index = -1;
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_param_request_read_encode(mavlink_sdk.getSysId(), mavlink_sdk.getCompId(), &mavlink_message, &mavlink_param);

    mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;	
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

void CMavlinkCommand::releaseRCChannels() const
{
	mavlinksdk::CMavlinkSDK& mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
	
	mavlink_rc_channels_override_t mavlink_rc_channels = {0};
	mavlink_rc_channels.target_system    = mavlink_sdk.getSysId();
	mavlink_rc_channels.target_component = mavlink_sdk.getCompId();
	
	// mavlink_rc_channels.chan1_raw = channel_length>=1?channels[0]:0;
    // mavlink_rc_channels.chan2_raw = channel_length>=2?channels[1]:0;
    // mavlink_rc_channels.chan3_raw = channel_length>=3?channels[2]:0;
	// mavlink_rc_channels.chan4_raw = channel_length>=4?channels[3]:0;
    // mavlink_rc_channels.chan5_raw = channel_length>=5?channels[4]:0;
    // mavlink_rc_channels.chan6_raw = channel_length>=6?channels[5]:0;
    // mavlink_rc_channels.chan7_raw = channel_length>=7?channels[6]:0;
    // mavlink_rc_channels.chan8_raw = channel_length>=8?channels[7]:0;
    mavlink_rc_channels.chan9_raw  = UINT16_MAX-1;
    mavlink_rc_channels.chan10_raw = UINT16_MAX-1;
    mavlink_rc_channels.chan11_raw = UINT16_MAX-1;
    mavlink_rc_channels.chan12_raw = UINT16_MAX-1;
    mavlink_rc_channels.chan13_raw = UINT16_MAX-1;
    mavlink_rc_channels.chan14_raw = UINT16_MAX-1;
    mavlink_rc_channels.chan15_raw = UINT16_MAX-1;
    mavlink_rc_channels.chan16_raw = UINT16_MAX-1;
	mavlink_rc_channels.chan15_raw = UINT16_MAX-1;
    mavlink_rc_channels.chan16_raw = UINT16_MAX-1;
	
    mavlink_message_t mavlink_message;
	mavlink_msg_rc_channels_override_encode (255, mavlink_sdk.getCompId(), &mavlink_message, &mavlink_rc_channels);

	mavlink_sdk.sendMavlinkMessage(mavlink_message);

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

	mavlinksdk::CMavlinkSDK& mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
	
	mavlink_rc_channels_override_t mavlink_rc_channels = {0};
	mavlink_rc_channels.target_system    = mavlink_sdk.getSysId();
	mavlink_rc_channels.target_component = mavlink_sdk.getCompId();
	
	mavlink_rc_channels.chan1_raw = channel_length>=1?channels[0]:0;
    mavlink_rc_channels.chan2_raw = channel_length>=2?channels[1]:0;
    mavlink_rc_channels.chan3_raw = channel_length>=3?channels[2]:0;
	mavlink_rc_channels.chan4_raw = channel_length>=4?channels[3]:0;
    mavlink_rc_channels.chan5_raw = channel_length>=5?channels[4]:0;
    mavlink_rc_channels.chan6_raw = channel_length>=6?channels[5]:0;
    mavlink_rc_channels.chan7_raw = channel_length>=7?channels[6]:0;
    mavlink_rc_channels.chan8_raw = channel_length>=8?channels[7]:0;
    mavlink_rc_channels.chan9_raw = channel_length>=9?channels[8]:0;
    mavlink_rc_channels.chan10_raw = channel_length>=10?channels[9]:0;
    mavlink_rc_channels.chan11_raw = channel_length>=11?channels[10]:0;
    mavlink_rc_channels.chan12_raw = channel_length>=12?channels[11]:0;
    mavlink_rc_channels.chan13_raw = channel_length>=13?channels[12]:0;
    mavlink_rc_channels.chan14_raw = channel_length>=14?channels[13]:0;
    mavlink_rc_channels.chan15_raw = channel_length>=15?channels[14]:0;
    mavlink_rc_channels.chan16_raw = channel_length>=16?channels[15]:0;
	mavlink_rc_channels.chan15_raw = channel_length>=17?channels[16]:0;
    mavlink_rc_channels.chan16_raw = channel_length>=18?channels[17]:0;
	
    mavlink_message_t mavlink_message;
	mavlink_msg_rc_channels_override_encode (255, mavlink_sdk.getCompId(), &mavlink_message, &mavlink_rc_channels);

	mavlink_sdk.sendMavlinkMessage(mavlink_message);

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
void CMavlinkCommand::ctrlGuidedVelocityInLocalFrame (const float vx, const float vy, const float vz, const float yaw_rate) const
{
	mavlinksdk::CMavlinkSDK& mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
	
	mavlink_set_position_target_local_ned_t mavlink_set_position = {0};


	mavlink_set_position.target_system    = mavlink_sdk.getSysId();
	mavlink_set_position.target_component = mavlink_sdk.getCompId();
	
	mavlink_set_position.type_mask =  POSITION_TARGET_TYPEMASK_X_IGNORE |
									  POSITION_TARGET_TYPEMASK_Y_IGNORE |
									  POSITION_TARGET_TYPEMASK_Z_IGNORE |
									  POSITION_TARGET_TYPEMASK_AX_IGNORE |
									  POSITION_TARGET_TYPEMASK_AY_IGNORE |
									  POSITION_TARGET_TYPEMASK_AZ_IGNORE |
									  POSITION_TARGET_TYPEMASK_YAW_IGNORE;

    mavlink_set_position.coordinate_frame = MAV_FRAME_BODY_OFFSET_NED;

	mavlink_set_position.vx = vx;
	mavlink_set_position.vy = vy;
	mavlink_set_position.vz = vz;

	mavlink_set_position.yaw_rate = yaw_rate;

	mavlink_message_t mavlink_message;
	mavlink_msg_set_position_target_local_ned_encode (255, mavlink_sdk.getCompId(), &mavlink_message, &mavlink_set_position);

	mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}
        