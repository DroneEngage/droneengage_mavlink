
#include <iostream>
#include <string>
#include <limits.h>

#include "./global.hpp"
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
	m_sysid  = 0;
    m_compid = 0;

	m_last_guided_mode_point.latitude = 0;
	m_last_guided_mode_point.longitude = 0;
	m_last_guided_mode_point.altitude = 0;
	
}

void mavlinksdk::CVehicle::set_callback_vehicle (mavlinksdk::CCallBack_Vehicle* callback_vehicle)
{
	m_callback_vehicle = callback_vehicle;
}

bool mavlinksdk::CVehicle::handle_heart_beat (const mavlink_heartbeat_t& heartbeat)
{
	
	if (m_sys_id == NO_SYSID_RESTRICTION)
	{
		const uint8_t c_type = heartbeat.type;
		// IGNORE UNIT TYPES 
		if ((c_type >=  MAV_TYPE::MAV_TYPE_GIMBAL)
		|| (c_type ==  MAV_TYPE::MAV_TYPE_GCS)
		|| (c_type ==  MAV_TYPE::MAV_TYPE_ONBOARD_CONTROLLER)
		|| (heartbeat.autopilot == MAV_AUTOPILOT_INVALID))
		return false;


		if (m_comp_id == NO_SYSID_RESTRICTION)
		{
			if (mavlink_message_temp.compid  != 1) return false; // assuming comp_id is 1 
		}
		else
		{
			if (m_comp_id != mavlink_message_temp.compid) return false;
		}
		
	}
	else
	{
		// Only search for SYSID
		if (m_sys_id != mavlink_message_temp.sysid) return false;
	}
	
	m_sysid = mavlink_message_temp.sysid;
	m_compid = mavlink_message_temp.compid;

	const bool is_armed  = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;

	
	
	//Detect mode change
	const bool is_mode_changed = ((m_heartbeat.custom_mode != heartbeat.custom_mode) || (m_heartbeat.type != heartbeat.type) || (m_heartbeat.autopilot != heartbeat.autopilot)) || !m_heart_beat_first_recieved;
	

	// copy 
	m_heartbeat = heartbeat;
	
	const uint64_t now = get_time_usec();
	
		
	if (m_heart_beat_first_recieved == false) 
	{  // Notify that we have something alive here.
		m_heart_beat_first_recieved = true;
		m_callback_vehicle->OnHeartBeat_First (heartbeat);
		
		requestDataStream();
	}
	else 
	if ((now - time_stamps.getMessageTime(MAVLINK_MSG_ID_HEARTBEAT)) > HEART_BEAT_TIMEOUT)
	{  // Notify when heart beat get live again.
		m_callback_vehicle->OnHeartBeat_Resumed (heartbeat);
		m_ready_to_arm_trigger_first_tick = false;

		requestDataStream();
	}

	// Detect change in arm status
	if (m_armed != is_armed)
	{
		m_armed = is_armed;
		m_callback_vehicle->OnArmed(m_armed, isReadyToArm());
	}

	if (heartbeat.autopilot != MAV_AUTOPILOT::MAV_AUTOPILOT_PX4)
	{
		const bool is_flying = is_armed && ((heartbeat.system_status == MAV_STATE_ACTIVE) || 
            	             ((heartbeat.system_status == MAV_STATE_CRITICAL) || (m_heartbeat.system_status == MAV_STATE_EMERGENCY)));
			
		// Detect change in flying status
		if (m_is_flying != is_flying)
		{
			m_is_flying = is_flying;
			m_callback_vehicle->OnFlying(m_is_flying);
		}
	}
	
	if (is_mode_changed)
	{
		m_callback_vehicle->OnModeChanges (m_heartbeat.custom_mode, m_heartbeat.type, (MAV_AUTOPILOT)m_heartbeat.autopilot);
	}


	// Handle GCS ID configuration: 0 means default 255, -1 means don't send heartbeat
	if (m_gcs_id != -1) {
		uint8_t gcs_id_to_use = (m_gcs_id == 0) ? 255 : m_gcs_id;
		mavlinksdk::CMavlinkCommand::getInstance().sendHeartBeatOfGCS(gcs_id_to_use);
	}

	return true;
}

void mavlinksdk::CVehicle::handle_sys_status(const mavlink_sys_status_t& sys_status)
{
	// Check if the pre-arm check bits are set correctly
    const bool prearm_ready = (sys_status.onboard_control_sensors_health & MAV_SYS_STATUS_PREARM_CHECK) != 0;

	const bool is_ready_to_arm = isReadyToArm();
	const bool state_changed = (prearm_ready != is_ready_to_arm);
	
	m_sys_status = sys_status;
	if (state_changed)
	{
#ifdef DEBUG		
		std::cout << "state_changed changed to " << is_ready_to_arm << " Sending Notification" << std::endl;
#endif		
		m_callback_vehicle->OnArmed(m_armed, prearm_ready);
	}
	
}

void mavlinksdk::CVehicle::handle_extended_system_state (const mavlink_extended_sys_state_t& extended_system_state)
{
	if (m_heartbeat.autopilot != MAV_AUTOPILOT::MAV_AUTOPILOT_PX4) return ;

	switch (extended_system_state.landed_state) {
		case MAV_LANDED_STATE_ON_GROUND:
			if (m_is_flying)
			{
				m_is_flying = false;
				m_callback_vehicle->OnFlying(m_is_flying);
			}
        	m_is_landing = false;
        break;
		case MAV_LANDED_STATE_TAKEOFF:
		case MAV_LANDED_STATE_IN_AIR:
			if (!m_is_flying)
			{
				m_is_flying = true;
				m_callback_vehicle->OnFlying(m_is_flying);
			}
        	m_is_landing = false;
			break;
		case MAV_LANDED_STATE_LANDING:
			if (!m_is_flying)
			{
				m_is_flying = true;
				m_callback_vehicle->OnFlying(m_is_flying);
			}
        	m_is_landing = true;
			break;
		default:
        break;
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
	return !((get_time_usec() - time_stamps.getMessageTime(MAVLINK_MSG_ID_HEARTBEAT)) > HEART_BEAT_TIMEOUT);
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
							|| (m_home_position.longitude != home_position.longitude)
							|| (m_home_position.altitude != home_position.altitude));

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


void mavlinksdk::CVehicle::handle_adsb_vehicle (const mavlink_adsb_vehicle_t& adsb_vehicle)
{
	m_adsb_vehicle = adsb_vehicle;
	m_callback_vehicle->OnADSBVechileReceived (adsb_vehicle);

	return ;
}
            

void mavlinksdk::CVehicle::handle_rc_channels_raw  (const mavlink_rc_channels_t& rc_channels)
{
	m_rc_channels = rc_channels;
}


void mavlinksdk::CVehicle::handle_servo_output_raw  (const mavlink_servo_output_raw_t& servo_output_raw)
{
	bool changed = false;

	if ((m_servo_output_raw.time_usec == 0) && (m_servo_output_raw.port == 0))
	{
		changed = true;
	}
	else
	{
		auto is_servo_changed = [](uint16_t old_value, uint16_t new_value) -> bool
		{
			if (old_value == new_value) return false;
			const int diff = (new_value > old_value) ? (new_value - old_value) : (old_value - new_value);
			const int range = 1000; // typical PWM range 1000-2000
			return (diff * 100) >= (5 * range);
		};

		if (is_servo_changed(m_servo_output_raw.servo5_raw, servo_output_raw.servo5_raw)
			|| is_servo_changed(m_servo_output_raw.servo6_raw, servo_output_raw.servo6_raw)
			|| is_servo_changed(m_servo_output_raw.servo7_raw, servo_output_raw.servo7_raw)
			|| is_servo_changed(m_servo_output_raw.servo8_raw, servo_output_raw.servo8_raw)
			|| is_servo_changed(m_servo_output_raw.servo9_raw, servo_output_raw.servo9_raw)
			|| is_servo_changed(m_servo_output_raw.servo10_raw, servo_output_raw.servo10_raw)
			|| is_servo_changed(m_servo_output_raw.servo11_raw, servo_output_raw.servo11_raw)
			|| is_servo_changed(m_servo_output_raw.servo12_raw, servo_output_raw.servo12_raw)
			|| is_servo_changed(m_servo_output_raw.servo13_raw, servo_output_raw.servo13_raw)
			|| is_servo_changed(m_servo_output_raw.servo14_raw, servo_output_raw.servo14_raw)
			|| is_servo_changed(m_servo_output_raw.servo15_raw, servo_output_raw.servo15_raw)
			|| is_servo_changed(m_servo_output_raw.servo16_raw, servo_output_raw.servo16_raw)
		)
		{
			changed = true;
		}
	}

	m_servo_output_raw = servo_output_raw;
	m_callback_vehicle->OnServoOutputRaw(servo_output_raw, changed);
}


void mavlinksdk::CVehicle::handle_system_time (const mavlink_system_time_t& system_time)
{
	if ((system_time.time_boot_ms < m_system_time.time_boot_ms) && (m_system_time.time_boot_ms > 0))
	{
		m_callback_vehicle->OnBoardRestarted();
	}

	m_system_time = system_time;
}


void mavlinksdk::CVehicle::handle_terrain_data_report (const mavlink_terrain_report_t& terrain_report)
{
	// Handle if changed to avoid replicated messages.
	m_terrain_report = terrain_report;
}


void mavlinksdk::CVehicle::handle_radio_status(const mavlink_radio_status_t& radio_status)
{
	m_radio_status = radio_status;
}

void mavlinksdk::CVehicle::handle_ekf_status_report(const mavlink_ekf_status_report_t& ekf_status_report)
{
	//TODO: Handle EKF info here.
	if (m_ekf_status_report.flags != ekf_status_report.flags)
	{
		m_ekf_status_report = ekf_status_report;
		m_callback_vehicle->OnEKFStatusReportChanged(ekf_status_report);
	}
	
	return ;
}


void mavlinksdk::CVehicle::handle_vibration_report(const mavlink_vibration_t& vibration)
{
	if ((vibration.clipping_0 != m_vibration.clipping_0)
	|| (vibration.clipping_1 != m_vibration.clipping_1)
	|| (vibration.clipping_2 != m_vibration.clipping_2))
	{
		m_vibration = vibration;
		m_callback_vehicle->OnVibrationChanged(vibration);
	}

	m_vibration = vibration;

	return;
}

void mavlinksdk::CVehicle::handle_distance_sensor (const mavlink_distance_sensor_t& distance_sensor)
{
	
	if (distance_sensor.orientation<=MAV_SENSOR_ROTATION_ROLL_90_PITCH_315)
	{
		mavlink_distance_sensor_t old_distance_sensor = m_distance_sensors[distance_sensor.orientation];
		
		m_has_lidar_altitude = (distance_sensor.orientation ==  MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_270 );
		if ((old_distance_sensor.current_distance != distance_sensor.current_distance)
		|| ((distance_sensor.time_boot_ms - old_distance_sensor.time_boot_ms) > DISTANCE_SENSOR_TIMEOUT))
		{
			// distance in range
			m_distance_sensors[distance_sensor.orientation] = distance_sensor;
			this->m_callback_vehicle->OnDistanceSensorChanged(distance_sensor);
		}
	}
	else
	if (distance_sensor.orientation == MAV_SENSOR_ROTATION_CUSTOM)
	{
		//TODO: Handle Custom Orientation		
	}
	
}


void mavlinksdk::CVehicle::requestDataStream()
{
	mavlinksdk::CMavlinkCommand::getInstance().requestDataStream(MAV_DATA_STREAM::MAV_DATA_STREAM_ALL);
	mavlinksdk::CMavlinkCommand::getInstance().requestDataStream(MAV_DATA_STREAM::MAV_DATA_STREAM_RAW_CONTROLLER);
	mavlinksdk::CMavlinkCommand::getInstance().requestDataStream(MAV_DATA_STREAM::MAV_DATA_STREAM_RAW_SENSORS);
	mavlinksdk::CMavlinkCommand::getInstance().requestDataStream(MAV_DATA_STREAM::MAV_DATA_STREAM_RC_CHANNELS);
	mavlinksdk::CMavlinkCommand::getInstance().requestDataStream(MAV_DATA_STREAM::MAV_DATA_STREAM_EXTENDED_STATUS);
	mavlinksdk::CMavlinkCommand::getInstance().requestDataStream(MAV_DATA_STREAM::MAV_DATA_STREAM_EXTRA1);
	mavlinksdk::CMavlinkCommand::getInstance().requestDataStream(MAV_DATA_STREAM::MAV_DATA_STREAM_EXTRA2);
	mavlinksdk::CMavlinkCommand::getInstance().requestDataStream(MAV_DATA_STREAM::MAV_DATA_STREAM_EXTRA3);
}

void mavlinksdk::CVehicle::exit_high_latency ()
{
	if (m_high_latency_mode==0) return ;
	m_high_latency_mode = 0; 
	this->m_callback_vehicle->OnHighLatencyModeChanged (0);
}

void mavlinksdk::CVehicle::handle_high_latency (const int message_id)
{
	const int temp = m_high_latency_mode;
	m_high_latency_mode = message_id;
	if (temp != message_id)
	{
		// high latency started
		this->m_callback_vehicle->OnHighLatencyModeChanged (m_high_latency_mode);
	}
	
	
	switch (message_id)
	{
		case 0:
		return ; // no high latency message

		case MAVLINK_MSG_ID_HIGH_LATENCY:
		{
			mavlink_heartbeat_t fake_heartbeat;
			memcpy(&fake_heartbeat, &m_heartbeat, sizeof (mavlink_heartbeat_t));
			fake_heartbeat.base_mode = m_high_latency.base_mode;
			fake_heartbeat.custom_mode = m_high_latency.custom_mode;
			uint64_t now = get_time_usec();
			time_stamps.setTimestamp(MAVLINK_MSG_ID_HIGH_LATENCY, now);
			time_stamps.setTimestamp(MAVLINK_MSG_ID_HEARTBEAT, now);
			handle_heart_beat(fake_heartbeat);
		}
		break;
		
		case MAVLINK_MSG_ID_HIGH_LATENCY2:
		{
			mavlink_heartbeat_t fake_heartbeat;
			memcpy(&fake_heartbeat, &m_heartbeat, sizeof (mavlink_heartbeat_t));
			fake_heartbeat.type = m_high_latency2.type;
			fake_heartbeat.autopilot = m_high_latency2.autopilot;
			fake_heartbeat.custom_mode = m_high_latency2.custom_mode;
			uint64_t now = get_time_usec();
			time_stamps.setTimestamp(MAVLINK_MSG_ID_HIGH_LATENCY2, now);
			time_stamps.setTimestamp(MAVLINK_MSG_ID_HEARTBEAT, now);
			handle_heart_beat(fake_heartbeat);
		}
		break;
	} 

	this->m_callback_vehicle->OnHighLatencyMessageReceived (message_id);

	return ;
}

bool mavlinksdk::CVehicle::parseMessage (const mavlink_message_t& mavlink_message)
{

	const u_int32_t msgid = mavlink_message.msgid;
	// #ifdef DEBUG
    // std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "parseMessage:" << std::to_string(msgid) << _NORMAL_CONSOLE_TEXT_ << std::endl;
    // #endif

	mavlink_message_temp = mavlink_message;
	bool message_processed = true;

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

			if (handle_heart_beat (heartbeat))
			{
				mavlinksdk::CMavlinkParameterManager::getInstance().handle_heart_beat (heartbeat);
			}
			else
			{
				message_processed = false;
			}
		}
        break;

		case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
		{
			mavlink_extended_sys_state_t extended_system_state;
			mavlink_msg_extended_sys_state_decode(&mavlink_message, &(extended_system_state));

			handle_extended_system_state(extended_system_state);
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
            // handle ready to arm
			mavlink_sys_status_t sys_status;
			mavlink_msg_sys_status_decode(&mavlink_message, &sys_status);
			
			handle_sys_status(sys_status);		
        }
        break;

        case MAVLINK_MSG_ID_BATTERY_STATUS:
		{
            mavlink_msg_battery_status_decode(&mavlink_message, &(m_battery_status));

        }
        break;

		case MAVLINK_MSG_ID_BATTERY2:
		{
			mavlink_msg_battery2_decode(&mavlink_message, &(m_battery2));
		}
		break;

		case MAVLINK_MSG_ID_FLIGHT_INFORMATION:
		{
			mavlink_msg_flight_information_decode(&mavlink_message, &(m_flight_information));
		}
		break;

        case MAVLINK_MSG_ID_WIND:
		{
			mavlink_msg_wind_decode(&mavlink_message, &(m_wind));
		}
		break;

		case MAVLINK_MSG_ID_DISTANCE_SENSOR:
		{
			mavlink_distance_sensor_t distance_sensor;
			mavlink_msg_distance_sensor_decode(&mavlink_message, &(distance_sensor));
			
			time_stamps.setTimestamp(msgid, get_time_usec());
			handle_distance_sensor(distance_sensor);
			return ;
		}
		break;


		case MAVLINK_MSG_ID_RADIO_STATUS:
		{
			mavlink_radio_status_t radio_status;
            mavlink_msg_radio_status_decode(&mavlink_message, &(radio_status));
			handle_radio_status (radio_status);
        }
        break;

		case MAVLINK_MSG_ID_HIGH_LATENCY:
		{
			mavlink_msg_high_latency_decode (&mavlink_message, &m_high_latency);
		    handle_high_latency (MAVLINK_MSG_ID_HIGH_LATENCY);
		}
        break;

		case MAVLINK_MSG_ID_HIGH_LATENCY2:
		{
			mavlink_msg_high_latency2_decode (&mavlink_message, &m_high_latency2);
		    handle_high_latency (MAVLINK_MSG_ID_HIGH_LATENCY2);
		}
        break;
		
		case MAVLINK_MSG_ID_EKF_STATUS_REPORT:
		{
			mavlink_ekf_status_report_t ekf_status_report;
			mavlink_msg_ekf_status_report_decode (&mavlink_message, &ekf_status_report);

			time_stamps.setTimestamp(msgid, get_time_usec());
			handle_ekf_status_report(ekf_status_report);
			return ;
		}
		break;

		case MAVLINK_MSG_ID_VIBRATION:
		{
			mavlink_vibration_t vibration;
			mavlink_msg_vibration_decode (&mavlink_message, &vibration);
			
			time_stamps.setTimestamp(msgid, get_time_usec());
			handle_vibration_report(vibration);
			return ;
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
			if (m_heartbeat.type == MAV_TYPE::MAV_TYPE_GROUND_ROVER)
			{ // ROVER: relative altitude is not meaningful, normalize to 0
				m_global_position_int.relative_alt = 0;
			}
			exit_high_latency ();
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
			exit_high_latency();	
        }
		break;

		case MAVLINK_MSG_ID_GPS_RAW_INT:
		{
			mavlink_msg_gps_raw_int_decode(&mavlink_message, &(m_gps_raw_int));
			exit_high_latency ();
		}

		case MAVLINK_MSG_ID_GPS2_RAW:
		{
			mavlink_msg_gps2_raw_decode(&mavlink_message, &(m_gps2_raw));
			exit_high_latency ();
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

		case MAVLINK_MSG_ID_VFR_HUD:
		{
			mavlink_msg_vfr_hud_decode(&mavlink_message, &(m_vfr_hud));
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
		
		case MAVLINK_MSG_ID_ADSB_VEHICLE:
		{
			mavlink_adsb_vehicle_t adsb_vehicle;
			mavlink_msg_adsb_vehicle_decode(&mavlink_message, &adsb_vehicle);
			
			time_stamps.setTimestamp(msgid, get_time_usec());
			handle_adsb_vehicle(adsb_vehicle);

			return ;
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

		case MAVLINK_MSG_ID_TERRAIN_REPORT:
		{
			mavlink_terrain_report_t terrain_report;
			mavlink_msg_terrain_report_decode (&mavlink_message, &terrain_report);
		
		    handle_terrain_data_report (terrain_report);
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
	time_stamps.setTimestamp(msgid, get_time_usec());

	return message_processed;
}