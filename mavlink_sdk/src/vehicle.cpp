
#include <iostream>
#include <string>
#include <limits.h>

#include "./helpers/colors.h"
#include "./helpers/utils.h"
#include "vehicle.h"
#include "mavlink_command.h"
#include "mavlink_waypoint_manager.h"
#include "mavlink_parameter_manager.h"


mavlinksdk::CVehicle::CVehicle()
{
	time_stamps.reset_timestamps();
	m_system_time.time_boot_ms = 0;
	m_system_time.time_unix_usec = 0;
}

void mavlinksdk::CVehicle::set_callback_vehicle (mavlinksdk::CCallBack_Vehicle* callback_vehicle)
{
	m_callback_vehicle = callback_vehicle;
}

void mavlinksdk::CVehicle::handle_heart_beat (const mavlink_heartbeat_t& heartbeat)
{
	
	const bool is_armed  = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;

	
	const bool is_flying = is_armed && ((heartbeat.system_status == MAV_STATE_ACTIVE) || 
           	             ((heartbeat.system_status == MAV_STATE_CRITICAL) || (m_heartbeat.system_status == MAV_STATE_EMERGENCY)));

	//Detect mode change
	const bool is_mode_changed = (m_heartbeat.custom_mode != heartbeat.custom_mode) || !m_heart_beat_first_recieved;
	

	// copy 
	m_heartbeat = heartbeat;
	
	const uint64_t now = get_time_usec();

	if (m_heart_beat_first_recieved == false) 
	{  // Notify that we have something alive here.
		m_heart_beat_first_recieved = true;
		m_firmware_type = mavlinksdk::CMavlinkHelper::getFirmewareType (heartbeat.type, heartbeat.autopilot);
		m_callback_vehicle->OnHeartBeat_First (heartbeat);
	}
	else 
	if ((now - time_stamps.message_id[MAVLINK_MSG_ID_HEARTBEAT]) > HEART_BEAT_TIMEOUT)
	{  // Notify when heart beat get live again.
		m_firmware_type = mavlinksdk::CMavlinkHelper::getFirmewareType (heartbeat.type, heartbeat.autopilot);
		m_callback_vehicle->OnHeartBeat_Resumed (heartbeat);
	}

	// Detect change in arm status
	if (m_armed != is_armed)
	{
		m_armed = is_armed;
		m_callback_vehicle->OnArmed(m_armed);
	}

	// Detect change in flying status
	if (m_is_flying != is_flying)
	{
		m_is_flying = is_flying;
		m_callback_vehicle->OnFlying(m_is_flying);
	}
	
	if (is_mode_changed)
	{
		m_callback_vehicle->OnModeChanges (m_heartbeat.custom_mode, m_firmware_type);
	}
	
	return ;
}


/**
 * @brief Drone is disconnected if no heartbeat has been recieved for @link HEART_BEAT_TIMEOUT @endlink
 * 
 * @return true 
 * @return false 
 */
const bool mavlinksdk::CVehicle::isFCBConnected() const
{
	return !((get_time_usec() - time_stamps.message_id[MAVLINK_MSG_ID_HEARTBEAT]) > HEART_BEAT_TIMEOUT);
}


void mavlinksdk::CVehicle::handle_cmd_ack (const mavlink_command_ack_t& command_ack)
{
	m_callback_vehicle->OnACK (command_ack.command, command_ack.result, mavlinksdk::CMavlinkHelper::getACKError (command_ack.result));
}



void mavlinksdk::CVehicle::handle_status_text (const mavlink_statustext_t& status_text)
{
	// message is 50 bytes max and is NOT null terminated string.
	char msg [52];
	memcpy (msg, status_text.text,50);
	msg[51] = 0;
	m_status_text = std::string(msg);
	m_status_severity = status_text.severity;
	this->m_callback_vehicle->OnStatusText(m_status_severity, m_status_text);
}


void mavlinksdk::CVehicle::handle_home_position (const mavlink_home_position_t& home_position)
{
	const bool home_changed = ((m_home_position.latitude != home_position.latitude)
							|| (m_home_position.latitude != home_position.latitude)
							|| (m_home_position.latitude != home_position.latitude));

	memcpy((void *) &m_home_position, (void *) &home_position, sizeof(mavlink_home_position_t));

	if (home_changed == true)
	{
		this->m_callback_vehicle->OnHomePositionUpdated(m_home_position);
	}
}

/**
 * @brief 
 * ! Value & Names are corrupted when returned
 * ! messages are not handled correctly no by madsdk module.
 * TODO: please fix
 * 
 * @param param_message 
 */
void mavlinksdk::CVehicle::handle_param_ext_value  (const mavlink_param_ext_value_t& param_message)
{
	/**
		@param param_id
		Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	*/
	char param_id[17], param_value[129];
	param_id[16] =0;
	memcpy((void *)&param_id[0], param_message.param_id,16);
	memcpy((void *)&param_value[0], param_message.param_value,128);
	
	
	#ifdef DEBUG
	std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: " 
		<< std::string(param_id) << " : " << "count: " << std::to_string(param_message.param_index) 
		<< " : " << std::to_string(param_message.param_type) << " : " << param_value
		<< _NORMAL_CONSOLE_TEXT_ << std::endl;
	#endif
}




void mavlinksdk::CVehicle::handle_rc_channels_raw  (const mavlink_rc_channels_t& rc_channels)
{
	m_rc_channels = rc_channels;
}


void mavlinksdk::CVehicle::handle_servo_output_raw  (const mavlink_servo_output_raw_t& servo_output_raw)
{
	m_servo_output_raw = servo_output_raw;
	m_callback_vehicle->OnServoOutputRaw(servo_output_raw);
}


void mavlinksdk::CVehicle::handle_system_time (const mavlink_system_time_t& system_time)
{
	if ((system_time.time_boot_ms < m_system_time.time_boot_ms) && (m_system_time.time_boot_ms > 0))
	{
		m_callback_vehicle->OnBoardRestarted();
	}

	m_system_time = system_time;
}

void mavlinksdk::CVehicle::parseMessage (const mavlink_message_t& mavlink_message)
{

	const u_int32_t msgid = mavlink_message.msgid;
	// #ifdef DEBUG
    // std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "parseMessage:" << std::to_string(msgid) << _NORMAL_CONSOLE_TEXT_ << std::endl;
    // #endif
	
	if (mavlink_message.sysid == 255) return ;

	switch (mavlink_message.msgid)
	{
        case MAVLINK_MSG_ID_HEARTBEAT:
		{
			mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&mavlink_message, &(heartbeat));
			// std::cout << _LOG_CONSOLE_TEXT << "mavlink_msg_heartbeat_decode:" 
			//         << " autopilot " << std::to_string(heartbeat.autopilot)
            //         << " custom_mode " << std::to_string(heartbeat.custom_mode)
            //         << " type " << std::to_string(heartbeat.type) << std::endl; 

			handle_heart_beat (heartbeat);
			mavlinksdk::CMavlinkParameterManager::getInstance().handle_heart_beat (heartbeat);
		}
        break;

		case MAVLINK_MSG_ID_SYSTEM_TIME:
		{
			mavlink_system_time_t system_time;
			mavlink_msg_system_time_decode(&mavlink_message, &(system_time));

			handle_system_time (system_time);
		}
		break;

        case MAVLINK_MSG_ID_SYS_STATUS:
		{
            mavlink_msg_sys_status_decode(&mavlink_message, &(m_sys_status));
					
        }
        break;

        case MAVLINK_MSG_ID_BATTERY_STATUS:
		{
            mavlink_msg_battery_status_decode(&mavlink_message, &(m_battery_status));

        }
        break;

        case MAVLINK_MSG_ID_RADIO_STATUS:
		{
            mavlink_msg_radio_status_decode(&mavlink_message, &(m_radio_status));
			
        }
        break;

		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
		{
            mavlink_msg_local_position_ned_decode(&mavlink_message, &(m_local_position_ned));
			
        }
        break;

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
		{
            mavlink_msg_global_position_int_decode(&mavlink_message, &(m_global_position_int));
			
        }
        break;

        case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
		{
		    mavlink_msg_position_target_local_ned_decode(&mavlink_message, &(m_position_target_local_ned));
			
        }
		break;

		case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
		{
			mavlink_msg_position_target_global_int_decode(&mavlink_message, &(m_position_target_global_int));
				
        }
		break;

		case MAVLINK_MSG_ID_GPS_RAW_INT:
		{
			mavlink_msg_gps_raw_int_decode(&mavlink_message, &(m_gps_raw_int));
			
		}

		case MAVLINK_MSG_ID_HIGHRES_IMU:
		{
			mavlink_msg_highres_imu_decode(&mavlink_message, &(m_highres_imu));
				
        }
		break;

		case MAVLINK_MSG_ID_ATTITUDE:
		{
			mavlink_msg_attitude_decode(&mavlink_message, &(m_attitude));
				
        }
		break;

		case MAVLINK_MSG_ID_HOME_POSITION:
		{
			mavlink_home_position_t home_position;

			mavlink_msg_home_position_decode(&mavlink_message, &(home_position));
			handle_home_position (home_position);
		
        }
		break;

		case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
		{
			mavlink_msg_nav_controller_output_decode(&mavlink_message, &m_nav_controller);
		}
		break;

		case MAVLINK_MSG_ID_COMMAND_ACK:
		{
			mavlink_command_ack_t command_ack;
			
			mavlink_msg_command_ack_decode(&mavlink_message, &command_ack);
			handle_cmd_ack(command_ack);

		}
		break;

		case MAVLINK_MSG_ID_STATUSTEXT:
		{
			mavlink_statustext_t status_text;

			mavlink_msg_statustext_decode(&mavlink_message, &status_text);
			handle_status_text (status_text);
			
		}
		break;

		case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST:
		{
			// never called
		}
		break;
		

		case MAVLINK_MSG_ID_PARAM_VALUE:
		{
			// mavlink2
			// TODO: to be implemented

			mavlink_param_value_t param_message;
			mavlink_msg_param_value_decode(&mavlink_message, &param_message);
			
			mavlinksdk::CMavlinkParameterManager::getInstance().handle_param_value (param_message);
		}
		break;
		
		
		case MAVLINK_MSG_ID_PARAM_EXT_VALUE:
		{
			// mavlink2
			// TODO: to be implemented
			
			// mavlink_param_ext_value_t param_message;
			// mavlink_msg_param_ext_value_decode(&mavlink_message, &param_message);
			
			// handle_param_ext_value (param_message);
		}
		break;
		
		case MAVLINK_MSG_ID_PARAM_EXT_ACK:
		{
			// mavlink2
			// TODO: to be implemented
		}
		break;
		
		case MAVLINK_MSG_ID_RC_CHANNELS:
		{
			mavlink_rc_channels_t rc_message;
			mavlink_msg_rc_channels_decode(&mavlink_message, &rc_message);
			
			handle_rc_channels_raw (rc_message);
		}
		break;

		case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
		{
			mavlink_servo_output_raw_t servo_message;
			mavlink_msg_servo_output_raw_decode(&mavlink_message, &servo_message);
			
			handle_servo_output_raw (servo_message);
		}
		break;


		

		// MISSION PART START ======================================================================================
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        {
            // mavlink_mission_item_int_t mission_item_int;
            // mavlink_msg_mission_item_int_decode (&mavlink_message, &mission_item_int);

            mavlinksdk::CMavlinkWayPointManager::getInstance().handle_mission_item (mavlink_message);
        }
        break;


		case MAVLINK_MSG_ID_MISSION_COUNT:
        {
            mavlink_mission_count_t mission_count;
            mavlink_msg_mission_count_decode (&mavlink_message, &mission_count);

            mavlinksdk::CMavlinkWayPointManager::getInstance().handle_mission_count (mission_count);
        }
        break;
        
        case MAVLINK_MSG_ID_MISSION_CURRENT:
        {
            mavlink_mission_current_t mission_current;
            mavlink_msg_mission_current_decode (&mavlink_message, &mission_current);

            mavlinksdk::CMavlinkWayPointManager::getInstance().handle_mission_current (mission_current);
        }
        break;

		case MAVLINK_MSG_ID_MISSION_ACK:
		{
			mavlink_mission_ack_t mission_ack;
			mavlink_msg_mission_ack_decode(&mavlink_message, &mission_ack);

			mavlinksdk::CMavlinkWayPointManager::getInstance().handle_mission_ack(mission_ack);
		}
        break;

		case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
		{
			mavlink_mission_item_reached_t mission_item_reached;
			mavlink_msg_mission_item_reached_decode(&mavlink_message, &mission_item_reached);

			mavlinksdk::CMavlinkWayPointManager::getInstance().handle_mission_item_reached(mission_item_reached);
		}
        break;
		
		case MAVLINK_MSG_ID_MISSION_REQUEST:
        {
            mavlink_mission_request_t mission_request;
            mavlink_msg_mission_request_decode (&mavlink_message, &mission_request);

            mavlinksdk::CMavlinkWayPointManager::getInstance().handle_mission_item_request (mission_request);
        }
        break;
		
		case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        {
            mavlink_mission_request_int_t mission_request_int;
            mavlink_msg_mission_request_int_decode (&mavlink_message, &mission_request_int);

            mavlinksdk::CMavlinkWayPointManager::getInstance().handle_mission_item_request (mission_request_int);
        }
        break;


		// MISSION PART END ======================================================================================

		default:
		{
			// printf("Warning, did not handle message id %i\n",message.msgid);
			break;
		}

    }

	// update last so that messages can test delay such as on heartbeat resume
	time_stamps.message_id[msgid] = get_time_usec();

}