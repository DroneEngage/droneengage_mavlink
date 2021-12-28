#include <iostream>
#include <cstdlib>
#include <csignal>

#include <vehicle.h>
#include <mavlink_waypoint_manager.h>
#include <mavlink_parameter_manager.h>
#include <mavlink_ftp_manager.h>
#include "./helpers/colors.hpp"
#include "./helpers/helpers.hpp"

#include "configFile.hpp"
#include "messages.hpp"
#include "fcb_modes.hpp"
#include "fcb_main.hpp"

#include "./geofence/fcb_geo_fence_base.hpp"
#include "./geofence/fcb_geo_fence_manager.hpp"

using namespace uavos::fcb;

void SchedulerThread(void * This) {
	((CFCBMain *)This)->loopScheduler(); 

    #ifdef DEBUG
        std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: Exit SchedulerThread" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

    return ;
}

/**
 * @brief get connection type UDP or serial based on config file.
 * 
 * @return int 
 */
int CFCBMain::getConnectionType () const 
{
    std::string connection = str_tolower(m_jsonConfig["fcbConnectionURI"]["type"].get<std::string>());

    std::size_t found = connection.find("serial");
    if (found!=std::string::npos) return CONNECTION_TYPE_SERIAL;

    found = connection.find("udp");
    if (found!=std::string::npos) return CONNECTION_TYPE_UDP;

    found = connection.find("tcp");
    if (found!=std::string::npos) return CONNECTION_TYPE_TCP;

    return CONNECTION_TYPE_UNKNOWN;
}

/**
 * @brief actual connect to the board in UDP or serial.
 * 
 * @return true 
 * @return false 
 */
bool CFCBMain::connectToFCB ()
{
    m_connection_type = getConnectionType();
    
    switch (m_connection_type)
    {

        case CONNECTION_TYPE_SERIAL:
            std::cout << _INFO_CONSOLE_TEXT << "Serial Connection Initializing" << _NORMAL_CONSOLE_TEXT_ << std::endl; 
            m_mavlink_sdk.connectSerial((m_jsonConfig["fcbConnectionURI"])["port"].get<std::string>().c_str(),
                                     (m_jsonConfig["fcbConnectionURI"])["baudrate"].get<int>());
            return true;
        
        
        case CONNECTION_TYPE_UDP:
            std::cout << _INFO_CONSOLE_TEXT << "UDP Connection Initializing" << _NORMAL_CONSOLE_TEXT_ << std::endl; 
            m_mavlink_sdk.connectUDP((m_jsonConfig["fcbConnectionURI"])["ip"].get<std::string>().c_str(),
                                     (m_jsonConfig["fcbConnectionURI"])["port"].get<int>());
            return true;
        
        
        case CONNECTION_TYPE_TCP:
            std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "TCP Connection is not supported" << _NORMAL_CONSOLE_TEXT_ << std::endl; 
            exit(0);
        break;

        default:
            throw "Connection to FCB is not in (serial, udp, tcp) ";
        break;
    }

    return false;
}


void CFCBMain::init ()
{
    
    m_exit_thread = false; 

    uavos::CConfigFile& cConfigFile = CConfigFile::getInstance();
    m_jsonConfig = cConfigFile.GetConfigJSON();
    
    initVehicleChannelLimits();
    m_mavlink_optimizer.init (m_jsonConfig["Message_Timeouts"]);
    m_mavlink_optimizer.setOptimizationLevel(m_jsonConfig["Default_Optimization_Level"].get<int>());
    
    if (connectToFCB() == true)
    {
        m_mavlink_sdk.start(this);
    }

    m_andruav_missions.clear();
    m_andruav_vehicle_info.rc_sub_action = RC_SUB_ACTION::RC_SUB_ACTION_RELEASED;
    
    m_scheduler_thread = std::thread(SchedulerThread, (void *) this);

    
    
    return ;
}


void CFCBMain::uninit ()
{
    // exit thread.
    if (m_exit_thread == true) 
    {
        std::cout << "m_exit_thread == true" << std::endl;
        return ;
    }

    m_exit_thread = true;

    m_scheduler_thread.join();

    #ifdef DEBUG
        std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: ~CFCBMain  Scheduler Thread Off" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

    
    return ;
}

void CFCBMain::initVehicleChannelLimits()
{
    uavos::CConfigFile& cConfigFile = CConfigFile::getInstance();
    cConfigFile.reloadFile();
    m_jsonConfig = cConfigFile.GetConfigJSON();
    
    if (!m_jsonConfig.contains("RC_Channels")
        || !m_jsonConfig["RC_Channels"].contains("RC_channelEnabled")
        || !m_jsonConfig["RC_Channels"].contains("RC_channelReverse")
        || !m_jsonConfig["RC_Channels"].contains("RC_channelLimitsMax")
        || !m_jsonConfig["RC_Channels"].contains("RC_channelLimitsMin"))
    {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "RC Channels RC_channelEnabled, RC_channelReverse, RC_channelLimitsMax or RC_channelLimitsMin not found" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        raise(SIGABRT);
    }
    
    int index =0;
    Json values;
    
    values = m_jsonConfig["RC_Channels"]["RC_channelEnabled"];
    for (auto it = values.begin(); it != values.end(); ++it){
            m_andruav_vehicle_info.rc_channels_enabled[index] = *it==1?true:false;
            index++;
    }

    index =0;
    values = m_jsonConfig["RC_Channels"]["RC_channelReverse"];
    for (auto it = values.begin(); it != values.end(); ++it){
            m_andruav_vehicle_info.rc_channels_reverse[index] = *it==1?true:false;
            index++;
    }

    index =0;
    values = m_jsonConfig["RC_Channels"]["RC_channelLimitsMax"];
    for (auto it = values.begin(); it != values.end(); ++it){
            m_andruav_vehicle_info.rc_channels_max[index] = *it;
            index++;
    }

    index =0;
    values = m_jsonConfig["RC_Channels"]["RC_channelLimitsMin"];
    for (auto it = values.begin(); it != values.end(); ++it){
            m_andruav_vehicle_info.rc_channels_min[index] = *it;
            index++;
    }
}


/**
 * @brief Send periodic RCControl values to FCB board to avoid timeout.
 * 
 * 
 */
void CFCBMain::remoteControlSignal ()
{
    if (!m_andruav_vehicle_info.rc_command_active) return ;
    const u_int64_t now = get_time_usec();

    switch (m_andruav_vehicle_info.rc_sub_action)
    {
        case RC_SUB_ACTION::RC_SUB_ACTION_RELEASED:
        {
            // should ignore these values.
        }
        break;

        case RC_SUB_ACTION::RC_SUB_ACTION_CENTER_CHANNELS:
        {
            // should ignore these values.
            mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels(m_andruav_vehicle_info.rc_channels,18);

        }
        break;
        
        case RC_SUB_ACTION::RC_SUB_ACTION_FREEZE_CHANNELS:
        {
            // should ignore these values.
            mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels(m_andruav_vehicle_info.rc_channels,18);

        }
        break;
        
        case RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS:
        {
            if (now - m_andruav_vehicle_info.rc_command_last_update_time > RCCHANNEL_OVERRIDES_TIMEOUT)
            {
                m_andruav_vehicle_info.rc_command_active = false;
                
                releaseRemoteControl();
                
                // if RC Timeout then switch to brake or equivelant mode.
                const int ardupilot_mode = CFCBModes::getArduPilotMode(VEHICLE_MODE_BRAKE, m_andruav_vehicle_info.vehicle_type);
                mavlinksdk::CMavlinkCommand::getInstance().doSetMode(ardupilot_mode);

                return ;
            }
            mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels(m_andruav_vehicle_info.rc_channels,18);

        }
        break;

        case RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED:
        {
            if (now - m_andruav_vehicle_info.rc_command_last_update_time > RCCHANNEL_OVERRIDES_TIMEOUT)
            {
                releaseRemoteControl();
                
                // if RC Timeout then switch to brake or equivelant mode.
                const int ardupilot_mode = CFCBModes::getArduPilotMode(VEHICLE_MODE_BRAKE, m_andruav_vehicle_info.vehicle_type);
                mavlinksdk::CMavlinkCommand::getInstance().doSetMode(ardupilot_mode);
                
                return ;
            }

            //mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels(m_andruav_vehicle_info.rc_channels,18);
        }
        break;
    }
}

/**
 * @brief A separate thread schedular for calling recurring tasks.
 * 
 */
void CFCBMain::loopScheduler ()
{
    while (!m_exit_thread)
    {
        // timer each 10m sec.
        wait_time_nsec (0,10000000);

        m_counter++;

        if (m_counter%10 ==0)
        {   // each 100 msec
            remoteControlSignal();
        }

        if (m_counter%30 ==0)
        {   // each 300 msec
            
        }

        if (m_counter%50 == 0)
        {
            m_fcb_facade.sendGPSInfo(std::string());

            m_fcb_facade.sendNavInfo(std::string());
 
            updateGeoFenceHitStatus();

        }

        if (m_counter %100 ==0)
        {
            // each second
         
            // .................

            if (this->m_counter_sec % 2 ==0)
            {// 2 sec

                // update ranges dynamically.
                initVehicleChannelLimits();
            }

            if (m_counter_sec % 5 ==0)
            {// 5 sec
                m_fcb_facade.sendPowerInfo(std::string());
                bool fcb_connected = mavlinksdk::CVehicle::getInstance().isFCBConnected();

                if (fcb_connected != m_fcb_connected)
                {
                   m_fcb_connected = fcb_connected;
                   m_fcb_facade.sendID(std::string());
                }
    
            }

            if (m_counter_sec % 10 ==0)
            {// 10 sec
                m_fcb_facade.sendID(std::string());
            }

            if (m_counter_sec % 15 ==0)
            {// 15 sec
            
            }

            m_counter_sec++;
        }
    }

    return ;
}


void CFCBMain::reloadWayPoints()
{
    m_andruav_missions.clear();
    mavlinksdk::CMavlinkCommand::getInstance().reloadWayPoints();

    return ;
    
}

/**
 * @brief clear way points from andruav and fcb.
 * * In FCB it clears all types of maps.
 * * But in andruav geo fence is not affected by this command.
 * 
 */
void CFCBMain::clearWayPoints()
{
    m_andruav_missions.clear();
    //m_fcb_facade.sendWayPoints(std::string());
    mavlinksdk::CMavlinkCommand::getInstance().clearWayPoints();
    return ;
    
}

/**
 * @brief uploads mavlink mission items to FCB.
 * 
 */
void CFCBMain::saveWayPointsToFCB()
{
    //m_andruav_missions.clear();
    //m_fcb_facade.sendWayPoints(std::string());
    
    const std::size_t length = m_andruav_missions.mission_items.size();
    if (length == 0)
    {
        // ignore
        return ;
    }

    
    std::map <int, mavlink_mission_item_int_t> mavlink_mission;
    for (int i=0; i<length; ++i)
    {
        mavlink_mission.insert(std::make_pair(i,m_andruav_missions.mission_items.at(i).get()->getArdupilotMission()));
    }

    mavlinksdk::CMavlinkWayPointManager::getInstance().saveWayPoints(mavlink_mission, MAV_MISSION_TYPE_MISSION);

    return ;
    
}

void CFCBMain::OnMessageReceived (const mavlink_message_t& mavlink_message)
{
    //std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ << "OnMessageReceived" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    //m_traffic_optimizer.shouldForwardThisMessage (mavlink_message);
    
    if (mavlink_message.msgid == MAVLINK_MSG_ID_HEARTBEAT)
    {
        OnHeartBeat();
    }

    // if streaming active check each message to forward.
    if (m_mavlink_optimizer.shouldForwardThisMessage (mavlink_message))
    {
        std::vector<std::unique_ptr<ANDRUAV_UNIT_STRUCT>> ::iterator it;
        
        for(it=m_TelemetryUnits.begin(); it!=m_TelemetryUnits.end(); it++)
        {
            ANDRUAV_UNIT_STRUCT *unit_ptr = it->get();
        
            if (unit_ptr->is_online == true)
            {
                #ifdef DEBUG_2
	                std::cout << "send to " << unit_ptr->party_id << std::endl;
                #endif
                m_fcb_facade.sendTelemetryData (unit_ptr->party_id, mavlink_message);
            }
        }
    }

    return ;
}


void CFCBMain::OnConnected (const bool& connected)
{
    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "OnConnected" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    if (m_andruav_vehicle_info.use_fcb != connected)
    {
        m_andruav_vehicle_info.use_fcb = connected;
    }

    return ;

}


/**
 * @brief called locally used for ticking.
 * 
 */
void CFCBMain::OnHeartBeat ()
{
    if (m_andruav_vehicle_info.is_flying)
    {
        m_andruav_vehicle_info.flying_last_start_time = ( get_time_usec() - m_last_start_flying ) / 1000000l;
    }

    return ;
}

/**
 * @brief Called when HearBeat is received for the first time.
 * 
 * @param heartbeat 
 */
void CFCBMain::OnHeartBeat_First (const mavlink_heartbeat_t& heartbeat)
{
    
    m_andruav_vehicle_info.vehicle_type = CFCBModes::getAndruavVehicleType (heartbeat.type, heartbeat.autopilot);

    m_andruav_vehicle_info.gps_mode =  GPS_MODE_FCB;

    m_fcb_facade.sendID(std::string());

    // request home location
    mavlinksdk::CMavlinkCommand::getInstance().requestHomeLocation();
    
    
    return ;
}


void CFCBMain::OnHeartBeat_Resumed (const mavlink_heartbeat_t& heartbeat)
{
    m_andruav_vehicle_info.vehicle_type = CFCBModes::getAndruavVehicleType (heartbeat.type, heartbeat.autopilot);

    m_fcb_facade.sendID(std::string());
    
    
    return ;
}

void CFCBMain::OnBoardRestarted ()
{
    std::cout << std::endl << _ERROR_CONSOLE_BOLD_TEXT_ << "Flight Controller Restarted" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    
    m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_ERROR, "FCB boad has been restarted");

    return ;
}

            
void CFCBMain::OnArmed (const bool& armed)
{
    std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ << "OnArmed" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    
    m_andruav_vehicle_info.is_armed = armed;

    m_fcb_facade.sendID(std::string());

    return ;
}


void CFCBMain::OnFlying (const bool& is_flying)
{
    std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ << "OnFlying" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    
    if (m_andruav_vehicle_info.is_flying != is_flying)
    {
        if (is_flying == true)
        {
            m_last_start_flying = get_time_usec();
            // start capture a flying
            m_andruav_vehicle_info.flying_last_start_time = 0;
        }
        else
        {
            // add duration to total
            m_andruav_vehicle_info.flying_total_duration +=  m_andruav_vehicle_info.flying_last_start_time;
            m_andruav_vehicle_info.flying_last_start_time = 0;
            m_last_start_flying = 0;
        }

        m_andruav_vehicle_info.is_flying = is_flying;
        m_fcb_facade.sendID(std::string());
    }

    return ;
 
}


void CFCBMain::OnStatusText (const std::uint8_t& severity, const std::string& status)
{
    std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ << "OnStatusText " << _NORMAL_CONSOLE_TEXT_ << status << std::endl;
    
    m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_3DR, severity, status);

    return ;
    
}

void CFCBMain::OnMissionACK (const int& result, const int& mission_type, const std::string& result_msg)
{
    int sevirity;

    switch (result)
    {
        case MAV_MISSION_ACCEPTED:
        case MAV_MISSION_OPERATION_CANCELLED:
            sevirity = NOTIFICATION_TYPE_INFO;
        break;

        case MAV_MISSION_UNSUPPORTED:
        case MAV_MISSION_UNSUPPORTED_FRAME:
            sevirity = NOTIFICATION_TYPE_WARNING;
        break;

        case MAV_MISSION_ERROR:
        case MAV_MISSION_NO_SPACE:
        case MAV_MISSION_INVALID:
        case MAV_MISSION_INVALID_PARAM1:
        case MAV_MISSION_INVALID_PARAM2:
        case MAV_MISSION_INVALID_PARAM3:
        case MAV_MISSION_INVALID_PARAM4:
        case MAV_MISSION_INVALID_PARAM5_X:
        case MAV_MISSION_INVALID_PARAM6_Y:
        case MAV_MISSION_INVALID_PARAM7:
        case MAV_MISSION_INVALID_SEQUENCE:
        case MAV_MISSION_DENIED:
            sevirity = NOTIFICATION_TYPE_ERROR;
        break;

        default:
            sevirity = NOTIFICATION_TYPE_WARNING;
        break;
    }

    m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_3DR, sevirity, result_msg);
    return ;
}


void CFCBMain::OnWaypointReached(const int& seq) 
{
    m_andruav_vehicle_info.current_waypoint = seq;
    m_fcb_facade.sendWayPointReached (std::string(), seq);

    return ;
}

void CFCBMain::OnWayPointReceived(const mavlink_mission_item_int_t& mission_item_int)
{
   if (mission_item_int.mission_type == MAV_MISSION_TYPE_MISSION)
   {
       mission::CMissionItem *mission_item  = mission::CMissionItemBuilder::getClassByMavlinkCMD(mission_item_int);
       mission_item->decodeMavlink(mission_item_int);
       m_andruav_missions.mission_items.insert(std::make_pair( mission_item_int.seq, std::unique_ptr<mission::CMissionItem>(mission_item)));
   }

    return ;
}

void CFCBMain::OnWayPointsLoadingCompleted ()
{
    // ?Please check if we need to notify GCS.
    // notify that mission has been updated
    m_fcb_facade.sendWayPoints(std::string());

    
}

/**
 * @brief called when ACK receivied while writing messages.
 * if result is MAV_MISSION_RESULT::MAV_MISSION_ACCEPTED then mission has been successfully uploaded 
 * 
 * @param result 
 * @param mission_type 
 * @param result_msg 
 */
void CFCBMain::OnMissionSaveFinished (const int& result, const int& mission_type, const std::string& result_msg)
{
    if (result == MAV_MISSION_RESULT::MAV_MISSION_ACCEPTED)
    {
        m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_INFO, "mission saved successfully");
        //reloadWayPoints();
    }
    else
    {
        // broadcast error.
        OnMissionACK(result, mission_type, result_msg);
    }
}
            
void CFCBMain::OnACK (const int& acknowledged_cmd, const int& result, const std::string& result_msg)
{
    int sevirity;

    switch (result)
    {
        case MAV_RESULT_ACCEPTED:
        case MAV_RESULT_IN_PROGRESS:
            sevirity = NOTIFICATION_TYPE_INFO;
        break;

        case MAV_RESULT_TEMPORARILY_REJECTED:
        case MAV_RESULT_UNSUPPORTED:
            sevirity = NOTIFICATION_TYPE_WARNING;
        break;

        case MAV_RESULT_DENIED:
        case MAV_RESULT_FAILED:
            sevirity = NOTIFICATION_TYPE_ERROR;
        break;

        default:
            sevirity = NOTIFICATION_TYPE_WARNING;
        break;
    }

    if (result !=MAV_CMD_ACK_OK)
    {
        // dont annoy the user if everything is OK.
        m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_3DR, sevirity, result_msg);
    }

    return ;
}

 
void CFCBMain::OnModeChanges(const int& custom_mode, const int& firmware_type)
{
    
    m_andruav_vehicle_info.flying_mode = CFCBModes::getAndruavMode (custom_mode, m_andruav_vehicle_info.vehicle_type);
    adjustRemoteJoystickByMode(m_andruav_vehicle_info.rc_sub_action);
    m_fcb_facade.sendID(std::string());

    return ;
}   


void CFCBMain::OnHomePositionUpdated(const mavlink_home_position_t& home_position)
{
    m_fcb_facade.sendHomeLocation(std::string());

    return ;
}


void CFCBMain::OnServoOutputRaw(const mavlink_servo_output_raw_t& servo_output_raw)
{
    
    m_event_time_divider = m_event_time_divider + 1;
    m_event_time_divider = m_event_time_divider % EVENT_TIME_DIVIDER;
    int event_value;
    if (m_event_time_divider == 0)
    { // no need to get here with every event.
        switch (m_event_fire_channel)
        {
            case 11:
                event_value = servo_output_raw.servo11_raw;
                break;
            case 12:
                event_value = servo_output_raw.servo12_raw;
                break;
            case 13:
                event_value = servo_output_raw.servo13_raw;
                break;
            case 14:
                event_value = servo_output_raw.servo14_raw;
                break;
            case 15:
                event_value = servo_output_raw.servo15_raw;
                break;
            case 16:
                event_value = servo_output_raw.servo16_raw;
                break;
            default:
                event_value = 0;
                break;
        }

        if(std::find(m_event_fired_by_me.begin(), m_event_fired_by_me.end(), event_value) == m_event_fired_by_me.end()) 
        {
            m_event_fired_by_me.push_back(event_value);
            m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_WARNING, std::string("Trigger Event:") + std::to_string(event_value));
            m_fcb_facade.sendSyncEvent(std::string(""), event_value);
            
            std::cout << _INFO_CONSOLE_TEXT << "Event Triggered: " << std::to_string(event_value) << _NORMAL_CONSOLE_TEXT_ << std::endl;
            
        }
    }
    


    switch (m_event_wait_channel)
    {
        case 11:
            event_value = servo_output_raw.servo11_raw;
            break;
        case 12:
            event_value = servo_output_raw.servo12_raw;
            break;
        case 13:
            event_value = servo_output_raw.servo13_raw;
            break;
        case 14:
            event_value = servo_output_raw.servo14_raw;
            break;
        case 15:
            event_value = servo_output_raw.servo15_raw;
            break;
        case 16:
            event_value = servo_output_raw.servo16_raw;
            break;
        default:
            event_value = 0;
            break;
    }
    if (m_event_waiting_for  != event_value) 
    {
        m_event_waiting_for = event_value;
        m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_WARNING, std::string("Wait Event:") + std::to_string(m_event_waiting_for));

        processIncommingEvent ();
    }

    return ;
}


/**
 * @brief Called when parameter is recieved for first time or its value has changed.
 * @details Forward parameters only when changed = true. As parameters are sent in chunks to save bandwidth.
 * @param param_name parameter name
 * @param param_message parameter mavlink message
 * @param changed true if changed & false if new
 */
void CFCBMain::OnParamReceived(const std::string& param_name, const mavlink_param_value_t& param_message, const bool& changed)
{
    #ifdef DEBUG
	std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: " 
		<< std::string(param_name) << " : " << "count: " << std::to_string(param_message.param_index) << " of " << std::to_string(param_message.param_count)
		<< " type: " << std::to_string(param_message.param_type) << " value: " << std::to_string(param_message.param_value)
		<< " changed: " << std::to_string(changed) <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
	#endif

    if (changed) m_fcb_facade.sendParameterValue(std::string(), param_message);
}


void CFCBMain::OnParamReceivedCompleted()
{
    m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_INFO, std::string("Parameters Loaded Successfully"));
}


/**
 * @brief called when websocket connection changed state
 * @details called when connection between communication_pro and andruav server changes. 
 * 
 * @param status 
 */
void CFCBMain::OnConnectionStatusChangedWithAndruavServer (const int status)
{
    if (status == SOCKET_STATUS_REGISTERED)
    {
        m_fcb_facade.callModule_reloadSavedTasks(TYPE_AndruavSystem_LoadTasks);
        // Json json_msg = sendMREMSG(TYPE_AndruavSystem_LoadTasks);
        // const std::string msg = json_msg.dump();
        // cUDPClient.sendMSG(msg.c_str(), msg.length());
    }
}


void CFCBMain::alertUavosOffline()
{


    return ;
}



/**
 * @brief This function get PWM signal from Andruav [-500,500] value and afetr applying reverse, deadband & limites.
 * 
 * @param scaled_channels 
 * @param ignode_dead_band 
 * @return int16_t 
 */
void CFCBMain::calculateChannels(const int16_t scaled_channels[18], const bool ignode_dead_band, int16_t *output)
{
    
    for (int i=0; i<18;++i)
    {
        int scaled_channel = scaled_channels[i];

        if (scaled_channel == -999) 
        {
            output[i] = 0;
            continue;
        }

        if (!m_andruav_vehicle_info.rc_channels_enabled[i])
        {
            output[i] = 0;
            continue;
        }

        scaled_channel = scaled_channel - 500; // range from [-500,500]
        if ((!ignode_dead_band) && (abs(scaled_channel) < 20)) 
        {
            scaled_channel = 0;
        }

        // Apply Reverse
        if (m_andruav_vehicle_info.rc_channels_reverse[i])
        {
            scaled_channel = -scaled_channel;
        }

        // Limit Min Max
        const int min_value = 1500 - m_andruav_vehicle_info.rc_channels_min[i];
        const int max_value = m_andruav_vehicle_info.rc_channels_max[i] - 1500;

        if (scaled_channel <=0)
        {
            output[i] = int (min_value / 500.0f * scaled_channel);
        }
        else
        {
            output[i] = int (max_value / 500.0f * scaled_channel);
        }

        output[i]+=1500;
    }
}


void CFCBMain::releaseRemoteControl()
{
    if (m_andruav_vehicle_info.rc_sub_action == RC_SUB_ACTION::RC_SUB_ACTION_RELEASED) return ;

    memset(m_andruav_vehicle_info.rc_channels, 0, 18 * sizeof(int16_t));
    m_andruav_vehicle_info.rc_sub_action = RC_SUB_ACTION::RC_SUB_ACTION_RELEASED;
    m_andruav_vehicle_info.rc_command_active = false;

    mavlinksdk::CMavlinkCommand::getInstance().releaseRCChannels();

    m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_RCCONTROL, NOTIFICATION_TYPE_WARNING, std::string("RX Released."));

}



/**
 * @brief Set first 4 FOUR channels to 1500 and release others.
 * 
 */
void CFCBMain::centerRemoteControl()
{
    if (m_andruav_vehicle_info.rc_sub_action == RC_SUB_ACTION::RC_SUB_ACTION_CENTER_CHANNELS) return ;
    
    memset(m_andruav_vehicle_info.rc_channels, 0, 18 * sizeof(int16_t));
    memset(m_andruav_vehicle_info.rc_channels, 1500, 4 * sizeof(int16_t));
    
    m_andruav_vehicle_info.rc_sub_action = RC_SUB_ACTION::RC_SUB_ACTION_CENTER_CHANNELS;
    m_andruav_vehicle_info.rc_command_active = true;
    m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_RCCONTROL, NOTIFICATION_TYPE_WARNING, std::string("RX Centered and Locked"));
}


/**
 * @brief Read the current 8 EIGHT channels of RC and save them.
 * Other channels are released.
 * 
 */
void CFCBMain::freezeRemoteControl()
{
    
    if (m_andruav_vehicle_info.rc_sub_action == RC_SUB_ACTION::RC_SUB_ACTION_FREEZE_CHANNELS) return ;
    
    
    const mavlink_rc_channels_t& mavlink_rc_channels = mavlinksdk::CVehicle::getInstance().getRCChannels();

    m_andruav_vehicle_info.rc_channels[0] = mavlink_rc_channels.chan1_raw;    
    m_andruav_vehicle_info.rc_channels[1] = mavlink_rc_channels.chan2_raw;
    m_andruav_vehicle_info.rc_channels[2] = mavlink_rc_channels.chan3_raw;
    m_andruav_vehicle_info.rc_channels[3] = mavlink_rc_channels.chan4_raw;
    m_andruav_vehicle_info.rc_channels[4] = mavlink_rc_channels.chan5_raw;
    m_andruav_vehicle_info.rc_channels[5] = mavlink_rc_channels.chan6_raw;
    m_andruav_vehicle_info.rc_channels[6] = mavlink_rc_channels.chan7_raw;
    m_andruav_vehicle_info.rc_channels[7] = mavlink_rc_channels.chan8_raw;
    m_andruav_vehicle_info.rc_channels[8] = mavlink_rc_channels.chan9_raw;    
    m_andruav_vehicle_info.rc_channels[9] = mavlink_rc_channels.chan10_raw;
    m_andruav_vehicle_info.rc_channels[10] = mavlink_rc_channels.chan11_raw;
    m_andruav_vehicle_info.rc_channels[11] = mavlink_rc_channels.chan12_raw;
    m_andruav_vehicle_info.rc_channels[12] = mavlink_rc_channels.chan13_raw;
    m_andruav_vehicle_info.rc_channels[13] = mavlink_rc_channels.chan14_raw;
    m_andruav_vehicle_info.rc_channels[14] = mavlink_rc_channels.chan15_raw;
    m_andruav_vehicle_info.rc_channels[15] = mavlink_rc_channels.chan16_raw;
    m_andruav_vehicle_info.rc_channels[16] = mavlink_rc_channels.chan17_raw;
    m_andruav_vehicle_info.rc_channels[17] = mavlink_rc_channels.chan18_raw;

    m_andruav_vehicle_info.rc_sub_action = RC_SUB_ACTION::RC_SUB_ACTION_FREEZE_CHANNELS;
    m_andruav_vehicle_info.rc_command_active = true;
    
    m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_RCCONTROL, NOTIFICATION_TYPE_WARNING, std::string("RX Freeze Mode"));

}

/**
 * @brief Override channels. Overridden channels are determined dynamically.
 * 
 */
void CFCBMain::enableRemoteControl()
{
    m_andruav_vehicle_info.rc_command_last_update_time = get_time_usec();
    m_andruav_vehicle_info.rc_command_active = false;  // remote control data will enable it                   
    m_andruav_vehicle_info.rc_sub_action = RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS;
    
    m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_RCCONTROL, NOTIFICATION_TYPE_WARNING, std::string("RX Joystick Mode"));
}


void CFCBMain::enableRemoteControlGuided()
{

    m_andruav_vehicle_info.rc_command_last_update_time = get_time_usec();
    m_andruav_vehicle_info.rc_command_active = false;  // remote control data will enable it       
    
    // release channels
    memset(m_andruav_vehicle_info.rc_channels, 0, 18 * sizeof(int16_t));
    mavlinksdk::CMavlinkCommand::getInstance().releaseRCChannels();

    m_andruav_vehicle_info.rc_sub_action = RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED;
    
    m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_RCCONTROL, NOTIFICATION_TYPE_WARNING, std::string("RX Joystick Guided Mode"));

    return ;
}


void CFCBMain::adjustRemoteJoystickByMode (RC_SUB_ACTION rc_sub_action)
{
    switch (rc_sub_action)
    {
        case RC_SUB_ACTION::RC_SUB_ACTION_RELEASED:
        { 
            releaseRemoteControl();
        }
        break;

        case RC_SUB_ACTION::RC_SUB_ACTION_CENTER_CHANNELS:
        {
            centerRemoteControl();
        }
        break;

        case RC_SUB_ACTION::RC_SUB_ACTION_FREEZE_CHANNELS:
        {
            freezeRemoteControl();
        }
        break;

        case RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS:
        case RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED:
        {
            if (m_andruav_vehicle_info.flying_mode == VEHICLE_MODE_GUIDED)
            {
                //if (m_andruav_vehicle_info.rc_sub_action == RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS)
                    enableRemoteControlGuided();
            }
            else
            {
                //if (m_andruav_vehicle_info.rc_sub_action == RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED)
                    enableRemoteControl();
            }

        }
        break;

                                
    }
    
    
}

/**
 * @brief 
 * 
 * @param rc_channels values from [-500,500] ... -1 meanse release channel.
 */
void CFCBMain::updateRemoteControlChannels(const int16_t rc_channels[18])
{
    
    switch (m_andruav_vehicle_info.rc_sub_action)
    {
        case RC_SUB_ACTION::RC_SUB_ACTION_RELEASED:
        {
            // should ignore these values.
        }
        break;

        case RC_SUB_ACTION::RC_SUB_ACTION_CENTER_CHANNELS:
        {
            // should ignore these values.
        }
        break;
        
        case RC_SUB_ACTION::RC_SUB_ACTION_FREEZE_CHANNELS:
        {
            // should ignore these values.
        }
        break;
        
        case RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS:
        {
            m_andruav_vehicle_info.rc_command_last_update_time = get_time_usec();
            m_andruav_vehicle_info.rc_command_active = true;
            
            int16_t rc_chammels_pwm[18] = {0};

            calculateChannels(rc_channels, true, rc_chammels_pwm);
            
            for (int i=0; i<18; ++i)
            {
                m_andruav_vehicle_info.rc_channels[i] = rc_chammels_pwm[i];
            }

            mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels(m_andruav_vehicle_info.rc_channels,18);

        }
        break;
        
        case RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED:
        {
            m_andruav_vehicle_info.rc_command_last_update_time = get_time_usec();

            int16_t rc_chammels_pwm[18] = {0};

            calculateChannels(rc_channels, true, rc_chammels_pwm);
            
            mavlinksdk::CMavlinkCommand::getInstance().ctrlGuidedVelocityInLocalFrame (
                (1500 - rc_chammels_pwm[1]) / 100.0f,
                (rc_chammels_pwm[0] - 1500) / 100.0f,
                (1500 - rc_chammels_pwm[2]) / 100.0f,
                (rc_chammels_pwm[3]-1500)   / 100.0f
            );
    
        }
        break;
    }

}


void CFCBMain::insertIncommingEvent(const int16_t event_id)
{
    if(std::find(m_event_received_from_others.begin(), m_event_received_from_others.end(), event_id) == m_event_received_from_others.end()) 
    {
        m_event_received_from_others.push_back(event_id);

        std::cout << _INFO_CONSOLE_TEXT << "Event Received (new): " << std::to_string(event_id) << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }

    processIncommingEvent();
}


void CFCBMain::processIncommingEvent()
{

    
    std::vector<int>::iterator it = std::find(m_event_received_from_others.begin(), m_event_received_from_others.end(), m_event_waiting_for);

    if( it != m_event_received_from_others.end()) 
    { // event is found 
        if (getAndruavVehicleInfo().flying_mode == VEHICLE_MODE_AUTO) 
        {
            // +3 because setServo & Delay does not appear in Mission Item Reached
            //TODO: replace with break mode unless delay is used for timeout.
            const int next_seq = m_andruav_vehicle_info.current_waypoint+3; 
            if (m_andruav_missions.mission_items.size()<next_seq+1)
                mavlinksdk::CMavlinkCommand::getInstance().setCurrentMission(next_seq);
        }

        m_event_received_from_others.erase(it);
    }
}


/**
 * @details Starts, Stops & Update Mavlink streaming to other units & GCS.
 * 
 * @param target_party_id 
 * @param request_type @link CONST_TELEMETRY_REQUEST_START @endlink @link CONST_TELEMETRY_REQUEST_END @endlink @link CONST_TELEMETRY_REQUEST_RESUME @endlink 
 * @param streaming_level from 0 means no optimization to 3 max optimization.
 */
void CFCBMain::toggleMavlinkStreaming (const std::string& target_party_id, const int& request_type, const int& streaming_level)
{
        printf("toggleMavlinkStreaming %s %d %d\r\n", target_party_id.c_str(), request_type, streaming_level);

        if (streaming_level != -1)
        {
            // update streaming level globally.
            m_mavlink_optimizer.setOptimizationLevel(streaming_level);
            m_mavlink_optimizer.reset_timestamps();
        }
        
        if (!target_party_id.empty())
        {
            std::vector<std::unique_ptr<ANDRUAV_UNIT_STRUCT>> ::iterator it;
            
            bool found = false;
            for(it=m_TelemetryUnits.begin(); it!=m_TelemetryUnits.end(); it++)
            {
                ANDRUAV_UNIT_STRUCT *unit = it->get();
                
                std::cout << "compare to " << unit->party_id << " to " << target_party_id << std::endl;
                
                if (unit->party_id.compare(target_party_id)==0)
                {
                    found = true;
                    if (request_type == CONST_TELEMETRY_REQUEST_END)
                    {
                        std::cout << "found unit->is_online = false" << std::endl;
                        unit->is_online = false;
                    }
                    else
                    {
                        std::cout << "compare to " << unit->party_id << " to " << target_party_id << std::endl;
                        if (unit->party_id.compare(target_party_id)==0)
                        {
                            std::cout << "found unit->is_online = true" << std::endl;
                            unit->is_online = true;
                        }
                    }
                }
            }

            if (!found && (request_type != CONST_TELEMETRY_REQUEST_END))
            {
                std::cout << "Adding to m_TelemetryUnits " << target_party_id << std::endl;
                ANDRUAV_UNIT_STRUCT * unit_ptr= new ANDRUAV_UNIT_STRUCT;
                unit_ptr->party_id = target_party_id;
                unit_ptr->is_online = true;
                
                m_TelemetryUnits.push_back(std::unique_ptr<ANDRUAV_UNIT_STRUCT> (unit_ptr));
            }
        }

       return ;
}


void CFCBMain::takeActionOnFenceViolation(uavos::fcb::geofence::CGeoFenceBase * geo_fence)
{
    const int fence_action = geo_fence->hardFenceAction();
    switch (fence_action)
    {
        case CONST_FENCE_ACTION_SOFT:
        {
            return ;
        }
        break;

        case CONST_FENCE_ACTION_RTL:
        case CONST_FENCE_ACTION_LAND:
        case CONST_FENCE_ACTION_LOITER:
        case CONST_FENCE_ACTION_BRAKE:
        case CONST_FENCE_ACTION_SMART_RTL:
        {
            const int ardupilot_mode = CFCBModes::getArduPilotMode(fence_action, getAndruavVehicleInfo().vehicle_type);
            if (ardupilot_mode == E_UNDEFINED_MODE)
            {   
                //TODO: Send Error Message
                return ;
            }

            mavlinksdk::CMavlinkCommand::getInstance().doSetMode(ardupilot_mode);
            
            return ;
        }
        break;

    }
}
            
/**
 * @brief review status of each attached geo fence
 * 
 * @details each attached geo fence is called isInside() and based on result global fence status is calculated.
 * also status is sent to other parties using update hit status. Actions is taken when hard fences are violated.
 * 
 */
void CFCBMain::updateGeoFenceHitStatus()
{
    /* 
	    bit 0: out of green zone
		bit 1: in bad zone
		bit 2: in good zone
	*/
	int total_violation = 0b000;

    std::vector<geofence::GEO_FENCE_STRUCT*> geo_fence_struct_list = geofence::CGeoFenceManager::getInstance().getFencesOfParty(getAndruavVehicleInfo().party_id);
        
    const std::size_t size = geo_fence_struct_list.size();

    mavlinksdk::CVehicle&  vehicle =  mavlinksdk::CVehicle::getInstance();

    const mavlink_global_position_int_t&  gpos = vehicle.getMsgGlobalPositionInt();

    // test each fence and check if inside or not.
    for(int i = 0; i < size; i++)
    {
        uavos::fcb::geofence::GEO_FENCE_STRUCT * g = geo_fence_struct_list[i];
        uavos::fcb::geofence::CGeoFenceBase * geo_fence = g->geoFence.get();
        const int local_index = geo_fence_struct_list[i]->local_index;
        double in_zone_new = geo_fence->isInside(gpos.lat * 0.0000001, gpos.lon * 0.0000001, gpos.alt);
        double in_zone = g->parties[local_index].get()->in_zone;
        
        if ((in_zone == -INFINITY) || (signum(in_zone_new) != signum(in_zone)))
        {
            // change status
            //TODO: Alert & Act
            std::cout << "in_zone_new" << std::to_string(in_zone_new) << std::endl;
            g->parties[local_index].get()->in_zone = in_zone_new;
            if ((in_zone_new<=0) && geo_fence->shouldKeepOutside()) 
            {
                // violate should be OUTSIDE
                total_violation = total_violation | 0b010; //bad 

                std::string error_str = "violate fence " + std::string(geo_fence->getName());
                m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_GEO_FENCE_ERROR, NOTIFICATION_TYPE_ERROR, error_str);

                // Note that action is taken once when state changes.
                // This is important to allow user to reverse action or take other actions.
                // For example if you break you need a mechanize to land or take drone out ...etc.
                takeActionOnFenceViolation(geo_fence);
            }
            else if ((in_zone_new>0) && !geo_fence->shouldKeepOutside()) 
            {
                // multiple allowed fences may exist so a viuolation for one is not a violation.
                // a safe green fence but I am not inside it.
                // unless this is the only one.
                total_violation = total_violation | 0b001; // not in greed zone  

            }
            else if  ((in_zone_new<=0) && !geo_fence->shouldKeepOutside()) 
            {
                // green fence and I am in.
                total_violation = total_violation | 0b100; // good
            
                std::string error_str = "safe fence " + std::string(geo_fence->getName());
                m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_GEO_FENCE_ERROR, NOTIFICATION_TYPE_NOTICE, error_str);
            }
            
            
            m_fcb_facade.sendGeoFenceHit(std::string(""), 
                                        geo_fence->getName(),
                                        in_zone_new, 
                                        in_zone_new<=0,
                                        geo_fence->shouldKeepOutside());
        }

    }
}

