
#include <iostream>
#include <string>

#include "./helpers/colors.h"
#include "./helpers/utils.h"
#include <common/mavlink.h>
#include "vehicle.h"



mavlinksdk::CVehicle::CVehicle(mavlinksdk::CCallBack_Vehicle& callback_vehicle): m_callback_vehicle(callback_vehicle)
{
	
	time_stamps.reset_timestamps();
}



mavlinksdk::CVehicle::~CVehicle()
{
    
}

void mavlinksdk::CVehicle::handle_heart_beat (const mavlink_heartbeat_t& heartbeat)
{
	
	const bool is_armed  = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;

	
	const bool is_flying = ((heartbeat.system_status == MAV_STATE_ACTIVE) || 
           	             ((heartbeat.system_status == MAV_STATE_CRITICAL) || (m_heartbeat.system_status == MAV_STATE_EMERGENCY)));

	//Detect mode change
	const bool is_mode_changed = (m_heartbeat.custom_mode != heartbeat.custom_mode) || !m_heart_beat_first;
	

	// copy 
	m_heartbeat = heartbeat;
	
	// if ((m_heartbeat.type != heartbeat.type) ||(m_heartbeat.autopilot != m_heartbeat.autopilot))
	// {
	// 	m_firmware_type = mavlinksdk::CMavlinkHelper::getFirmewareType (heartbeat.type, heartbeat.autopilot);
	// }

	// Notify that we have something alive here.
	if (m_heart_beat_first == false)
	{
		m_heart_beat_first = true;
		m_firmware_type = mavlinksdk::CMavlinkHelper::getFirmewareType (heartbeat.type, heartbeat.autopilot);
		m_callback_vehicle.OnHeartBeat_First (heartbeat);
	}

	// Detect change in arm status
	if (m_armed != is_armed)
	{
		m_armed = is_armed;
		m_callback_vehicle.OnArmed(m_armed);
	}

	// Detect change in flying status
	if (m_is_flying != is_flying)
	{
		m_is_flying = is_flying;
		m_callback_vehicle.OnFlying(m_is_flying);
	}
	
	if (is_mode_changed)
	{
		m_callback_vehicle.OnModeChanges (m_heartbeat.custom_mode, m_firmware_type);
	}
	
	
 	
	return ;
}


void mavlinksdk::CVehicle::handle_cmd_ack (const mavlink_command_ack_t& command_ack)
{
	m_callback_vehicle.OnACK (command_ack.result, mavlinksdk::CMavlinkHelper::getACKError (command_ack.result));
}


void mavlinksdk::CVehicle::handle_status_text (const mavlink_statustext_t& status_text)
{
	// message is 50 bytes max and is NOT null terminated string.
	char msg [52];
	memcpy (msg, status_text.text,50);
	msg[51] = 0;
	m_status_text = std::string(msg);
	this->m_callback_vehicle.OnStatusText(m_status_text);
}

void mavlinksdk::CVehicle::parseMessage (mavlink_message_t& mavlink_message)
{

	const int msgid = mavlink_message.msgid;
	time_stamps.message_id[msgid] = get_time_usec();
	
	

    switch (msgid)
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

		case MAV_CMD_GET_HOME_POSITION:
		{
			mavlink_msg_home_position_decode(&mavlink_message, &(m_home_position));
		
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

		
		default:
		{
			// printf("Warning, did not handle message id %i\n",message.msgid);
			break;
		}

    }

}