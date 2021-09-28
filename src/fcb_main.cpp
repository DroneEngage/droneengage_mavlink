#include <iostream>
#include <cstdlib>
#include <csignal>

#include <vehicle.h>


#include "./helpers/colors.hpp"
#include "./helpers/helpers.hpp"

#include "configFile.hpp"
#include "messages.hpp"
#include "fcb_modes.hpp"
#include "fcb_main.hpp"


void SchedulerThread(void * This) {
	((uavos::fcb::CFCBMain *)This)->loopScheduler(); 

    #ifdef DEBUG
        std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: Exit SchedulerThread" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

    return ;
}

int uavos::fcb::CFCBMain::getConnectionType ()
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


bool uavos::fcb::CFCBMain::connectToFCB ()
{
    m_connection_type = getConnectionType();
    
    switch (m_connection_type)
    {

        case CONNECTION_TYPE_SERIAL:
            std::cout << _INFO_CONSOLE_TEXT << "Serial Connection detected" << _NORMAL_CONSOLE_TEXT_ << std::endl; 
            m_mavlink_sdk.connectSerial((m_jsonConfig["fcbConnectionURI"])["port"].get<std::string>().c_str(),
                                     (m_jsonConfig["fcbConnectionURI"])["baudrate"].get<int>());
            return true;
        
        
        case CONNECTION_TYPE_UDP:
            std::cout << _INFO_CONSOLE_TEXT << "UDP Connection detected" << _NORMAL_CONSOLE_TEXT_ << std::endl; 
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


void uavos::fcb::CFCBMain::init ()
{
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


void uavos::fcb::CFCBMain::uninit ()
{
    // exit thread.
    m_exit_thread = true;
    m_scheduler_thread.join();

    #ifdef DEBUG
        std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: ~CFCBMain  Scheduler Thread Off" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

    return ;
}

void uavos::fcb::CFCBMain::initVehicleChannelLimits()
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
void uavos::fcb::CFCBMain::remoteControlSignal ()
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
            mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels(m_andruav_vehicle_info.rc_channels,16);

        }
        break;
        
        case RC_SUB_ACTION::RC_SUB_ACTION_FREEZE_CHANNELS:
        {
            // should ignore these values.
            mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels(m_andruav_vehicle_info.rc_channels,16);

        }
        break;
        
        case RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS:
        {
            if (now - m_andruav_vehicle_info.rc_command_last_update_time > RCCHANNEL_OVERRIDES_TIMEOUT)
            {
                m_andruav_vehicle_info.rc_command_active = false;
                
                releaseRemoteControl();
                
                // if RC Timeout then switch to brake or equivelant mode.
                const int ardupilot_mode = uavos::fcb::CFCBModes::getArduPilotMode(VEHICLE_MODE_BRAKE, m_andruav_vehicle_info.vehicle_type);
                mavlinksdk::CMavlinkCommand::getInstance().doSetMode(ardupilot_mode);

                return ;
            }
            mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels(m_andruav_vehicle_info.rc_channels,16);

        }
        break;

        case RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED:
        {
            if (now - m_andruav_vehicle_info.rc_command_last_update_time > RCCHANNEL_OVERRIDES_TIMEOUT)
            {
                releaseRemoteControl();
                
                // if RC Timeout then switch to brake or equivelant mode.
                const int ardupilot_mode = uavos::fcb::CFCBModes::getArduPilotMode(VEHICLE_MODE_BRAKE, m_andruav_vehicle_info.vehicle_type);
                mavlinksdk::CMavlinkCommand::getInstance().doSetMode(ardupilot_mode);
                
                return ;
            }

            //mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels(m_andruav_vehicle_info.rc_channels,16);
        }
        break;
    }
}

void uavos::fcb::CFCBMain::loopScheduler ()
{
    while (!m_exit_thread)
    {
        
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


void uavos::fcb::CFCBMain::reloadWayPoints()
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
void uavos::fcb::CFCBMain::clearWayPoints()
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
void uavos::fcb::CFCBMain::saveWayPointsToFCB()
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

    m_mavlink_sdk.getWayPointManager().get()->saveWayPoints(mavlink_mission, MAV_MISSION_TYPE_MISSION);

    return ;
    
}

void uavos::fcb::CFCBMain::OnMessageReceived (const mavlink_message_t& mavlink_message)
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
        std::vector<std::unique_ptr<uavos::fcb::ANDRUAV_UNIT_STRUCT>> ::iterator it;
        
        for(it=m_TelemetryUnits.begin(); it!=m_TelemetryUnits.end(); it++)
        {
            uavos::fcb::ANDRUAV_UNIT_STRUCT *unit_ptr = it->get();
        
            if (unit_ptr->is_online == true)
            {
                #ifdef DEBUG
	                std::cout << "send to " << unit_ptr->party_id << std::endl;
                #endif
                m_fcb_facade.sendTelemetryData (unit_ptr->party_id, mavlink_message);
            }
        }
    }

    return ;
}


void uavos::fcb::CFCBMain::OnConnected (const bool& connected)
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
void uavos::fcb::CFCBMain::OnHeartBeat ()
{
    if (m_andruav_vehicle_info.is_flying)
    {
        m_andruav_vehicle_info.flying_last_start_time = ( get_time_usec() - m_last_start_flying ) / 1000000l;
    }

    return ;
}

void uavos::fcb::CFCBMain::OnHeartBeat_First (const mavlink_heartbeat_t& heartbeat)
{
    
    m_andruav_vehicle_info.vehicle_type = uavos::fcb::CFCBModes::getAndruavVehicleType (heartbeat.type, heartbeat.autopilot);

    m_andruav_vehicle_info.gps_mode =  GPS_MODE_FCB;

    m_fcb_facade.sendID(std::string());

    mavlinksdk::CMavlinkCommand::getInstance().requestHomeLocation();
    
    mavlinksdk::CMavlinkCommand::getInstance().requestParametersList();
   
    return ;
}


void uavos::fcb::CFCBMain::OnHeartBeat_Resumed (const mavlink_heartbeat_t& heartbeat)
{
    m_andruav_vehicle_info.vehicle_type = uavos::fcb::CFCBModes::getAndruavVehicleType (heartbeat.type, heartbeat.autopilot);

    m_fcb_facade.sendID(std::string());
    
    
    return ;
}
            
void uavos::fcb::CFCBMain::OnArmed (const bool& armed)
{
    std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ << "OnArmed" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    
    m_andruav_vehicle_info.is_armed = armed;

    m_fcb_facade.sendID(std::string());

    return ;
}


void uavos::fcb::CFCBMain::OnFlying (const bool& is_flying)
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


void uavos::fcb::CFCBMain::OnStatusText (const std::uint8_t& severity, const std::string& status)
{
    std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ << "OnStatusText " << _NORMAL_CONSOLE_TEXT_ << status << std::endl;
    
    m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_3DR, severity, status);

    return ;
    
}

void uavos::fcb::CFCBMain::onMissionACK (const int& result, const int& mission_type, const std::string& result_msg)
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


void uavos::fcb::CFCBMain::OnWaypointReached(const int& seq) 
{
    m_andruav_vehicle_info.current_waypoint = seq;
    m_fcb_facade.sendWayPointReached (std::string(), seq);

    return ;
}

void uavos::fcb::CFCBMain::OnWayPointReceived(const mavlink_mission_item_int_t& mission_item_int)
{
   if (mission_item_int.mission_type == MAV_MISSION_TYPE_MISSION)
   {
       uavos::fcb::mission::CMissionItem *mission_item  = uavos::fcb::mission::CMissionItemBuilder::getClassByMavlinkCMD(mission_item_int);
       mission_item->decodeMavlink(mission_item_int);
       m_andruav_missions.mission_items.insert(std::make_pair( mission_item_int.seq, std::unique_ptr<uavos::fcb::mission::CMissionItem>(mission_item)));
   }

    return ;
}

void uavos::fcb::CFCBMain::OnWayPointsLoadingCompleted ()
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
void uavos::fcb::CFCBMain::OnMissionSaveFinished (const int& result, const int& mission_type, const std::string& result_msg)
{
    if (result == MAV_MISSION_RESULT::MAV_MISSION_ACCEPTED)
    {
        m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_3DR, NOTIFICATION_TYPE_INFO, "mission saved successfully");
        //reloadWayPoints();
    }
    else
    {
        // broadcast error.
        onMissionACK(result, mission_type, result_msg);
    }
}
            
void uavos::fcb::CFCBMain::OnACK (const int& result, const std::string& result_msg)
{
    int sevirity;

    switch (result)
    {
        case MAV_RESULT_ACCEPTED:
        case MAV_RESULT_IN_PROGRESS:
        case MAV_RESULT_CANCELLED:
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

    m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_3DR, sevirity, result_msg);

    return ;
}

 
void uavos::fcb::CFCBMain::OnModeChanges(const int& custom_mode, const int& firmware_type)
{
    
    m_andruav_vehicle_info.flying_mode = uavos::fcb::CFCBModes::getAndruavMode (custom_mode, m_andruav_vehicle_info.vehicle_type);
    adjustRemoteJoystickByMode(m_andruav_vehicle_info.rc_sub_action);
    m_fcb_facade.sendID(std::string());

    return ;
}   


void uavos::fcb::CFCBMain::OnHomePositionUpdated(const mavlink_home_position_t& home_position)
{
    m_fcb_facade.sendHomeLocation(std::string());

    return ;
}


void uavos::fcb::CFCBMain::OnParamChanged(const std::string& param_name, const mavlink_param_value_t& param_message, const bool& changed)
{
    #ifdef DEBUG
	std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: " 
		<< std::string(param_name) << " : " << "count: " << std::to_string(param_message.param_index) 
		<< " type: " << std::to_string(param_message.param_type) << " value: " << std::to_string(param_message.param_value)
		<< _NORMAL_CONSOLE_TEXT_ << std::endl;
	#endif
}

void uavos::fcb::CFCBMain::alertUavosOffline()
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
void uavos::fcb::CFCBMain::calculateChannels(const int16_t scaled_channels[16], const bool ignode_dead_band, int16_t *output)
{
    
    for (int i=0; i<16;++i)
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


void uavos::fcb::CFCBMain::releaseRemoteControl()
{
    memset(m_andruav_vehicle_info.rc_channels, 0, 16 * sizeof(int16_t));
    m_andruav_vehicle_info.rc_sub_action = RC_SUB_ACTION::RC_SUB_ACTION_RELEASED;
    m_andruav_vehicle_info.rc_command_active = false;

    mavlinksdk::CMavlinkCommand::getInstance().releaseRCChannels();

    m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_RCCONTROL, NOTIFICATION_TYPE_WARNING, std::string("RX Released."));

}



/**
 * @brief Set first 4 FOUR channels to 1500 and release others.
 * 
 */
void uavos::fcb::CFCBMain::centerRemoteControl()
{
    memset(m_andruav_vehicle_info.rc_channels, 0, 16 * sizeof(int16_t));
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
void uavos::fcb::CFCBMain::freezeRemoteControl()
{
    
    memset(m_andruav_vehicle_info.rc_channels, 0, 16 * sizeof(int16_t));
    
    const mavlink_rc_channels_t& mavlink_rc_channels = m_mavlink_sdk.getVehicle().get()->getRCChannels();

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
void uavos::fcb::CFCBMain::enableRemoteControl()
{
    m_andruav_vehicle_info.rc_command_last_update_time = get_time_usec();
    m_andruav_vehicle_info.rc_command_active = false;  // remote control data will enable it                   
    m_andruav_vehicle_info.rc_sub_action = RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS;
    
    m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_RCCONTROL, NOTIFICATION_TYPE_WARNING, std::string("RX Joystick Mode"));
}


void uavos::fcb::CFCBMain::enableRemoteControlGuided()
{

    m_andruav_vehicle_info.rc_command_last_update_time = get_time_usec();
    m_andruav_vehicle_info.rc_command_active = false;  // remote control data will enable it       
    
    // release channels
    memset(m_andruav_vehicle_info.rc_channels, 0, 16 * sizeof(int16_t));
    mavlinksdk::CMavlinkCommand::getInstance().releaseRCChannels();

    m_andruav_vehicle_info.rc_sub_action = RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED;
    
    m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_RCCONTROL, NOTIFICATION_TYPE_WARNING, std::string("RX Joystick Guided Mode"));

    return ;
}


void uavos::fcb::CFCBMain::adjustRemoteJoystickByMode (RC_SUB_ACTION rc_sub_action)
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
void uavos::fcb::CFCBMain::updateRemoteControlChannels(const int16_t rc_channels[18])
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
            
            int16_t rc_chammels_pwm[16] = {0};

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

            int16_t rc_chammels_pwm[16] = {0};

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


/**
 * @details Starts, Stops & Update Mavlink streaming to other units & GCS.
 * 
 * @param target_party_id 
 * @param request_type @link CONST_TELEMETRY_REQUEST_START @endlink @link CONST_TELEMETRY_REQUEST_END @endlink @link CONST_TELEMETRY_REQUEST_RESUME @endlink 
 * @param streaming_level from 0 means no optimization to 3 max optimization.
 */
void uavos::fcb::CFCBMain::toggleMavlinkStreaming (const std::string& target_party_id, const int& request_type, const int& streaming_level)
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
            std::vector<std::unique_ptr<uavos::fcb::ANDRUAV_UNIT_STRUCT>> ::iterator it;
            
            bool found = false;
            for(it=m_TelemetryUnits.begin(); it!=m_TelemetryUnits.end(); it++)
            {
                uavos::fcb::ANDRUAV_UNIT_STRUCT *unit = it->get();
                
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
                uavos::fcb::ANDRUAV_UNIT_STRUCT * unit_ptr= new uavos::fcb::ANDRUAV_UNIT_STRUCT;
                unit_ptr->party_id = target_party_id;
                unit_ptr->is_online = true;
                
                m_TelemetryUnits.push_back(std::unique_ptr<uavos::fcb::ANDRUAV_UNIT_STRUCT> (unit_ptr));
            }
        }

        

        return ;

        return ;
}

