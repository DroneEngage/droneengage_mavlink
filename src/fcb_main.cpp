#include <iostream>

#include <vehicle.h>


#include "./helpers/colors.hpp"
#include "./helpers/helpers.hpp"

#include "messages.hpp"
#include "fcb_modes.hpp"
#include "fcb_main.hpp"
#include "fcb_traffic_optimizer.hpp"


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


void uavos::fcb::CFCBMain::init (const Json &jsonConfig)
{
    m_jsonConfig = jsonConfig;

    if (connectToFCB() == true)
    {
        m_mavlink_sdk.start(this);
    }

    m_scheduler_thread = std::thread(SchedulerThread, (void *) this);
}


void uavos::fcb::CFCBMain::uninit ()
{
    // exit thread.
    m_exit_thread = true;
    m_scheduler_thread.join();

    #ifdef DEBUG
        std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: ~CFCBMain  Scheduler Thread Off" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
}

void uavos::fcb::CFCBMain::loopScheduler ()
{
    while (!m_exit_thread)
    {
        
        wait_time_nsec (0,10000000);

        m_counter++;

        if (m_counter%10 ==0)
        {   // each 100 msec
            m_fcb_facade.sendGPSInfo(std::string());
            
        }

        if (m_counter%50 == 0)
        {
            m_fcb_facade.sendNavInfo(std::string());
        }

        if (m_counter %100 ==0)
        {// each second
            if (this->m_counter_sec % 2 ==0)
            {// 2 sec

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
}


void uavos::fcb::CFCBMain::OnMessageReceived (mavlink_message_t& mavlink_message)
{
    //std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ << "OnMessageReceived" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    uavos::fcb::CMavlinkTrafficOptimizer::ShouldForwardThisMessage (mavlink_message);

    if (mavlink_message.msgid == MAVLINK_MSG_ID_HEARTBEAT)
    {
        OnHeartBeat();
    }
}


void uavos::fcb::CFCBMain::OnConnected (const bool& connected)
{
    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "OnConnected" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    if (m_andruav_vehicle_info.use_fcb != connected)
    {
        m_andruav_vehicle_info.use_fcb = connected;
    }

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
}

void uavos::fcb::CFCBMain::OnHeartBeat_First (const mavlink_heartbeat_t& heartbeat)
{
    
    m_andruav_vehicle_info.vehicle_type = uavos::fcb::CFCBModes::getAndruavVehicleType (heartbeat.type, heartbeat.autopilot);

    m_andruav_vehicle_info.gps_mode =  GPS_MODE_FCB;

    m_fcb_facade.sendID(std::string());
}


void uavos::fcb::CFCBMain::OnHeartBeat_Resumed (const mavlink_heartbeat_t& heartbeat)
{
    std::unique_ptr<mavlinksdk::CVehicle>& vehicle = m_mavlink_sdk.getVehicle();

    m_andruav_vehicle_info.vehicle_type = uavos::fcb::CFCBModes::getAndruavVehicleType (heartbeat.type, heartbeat.autopilot);

    m_fcb_facade.sendID(std::string());
}
            
void uavos::fcb::CFCBMain::OnArmed (const bool& armed)
{
    std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ << "OnArmed" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    
    m_andruav_vehicle_info.is_armed = armed;

    m_fcb_facade.sendID(std::string());
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

    
}


void uavos::fcb::CFCBMain::OnStatusText (const std::uint8_t& severity, const std::string& status)
{
    std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ << "OnStatusText" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    
    m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_3DR, severity, status);
    
}

void uavos::fcb::CFCBMain::OnMissionACK (const int& result, const int& mission_type, const std::string& result_msg)
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
}

 
void uavos::fcb::CFCBMain::OnModeChanges(const int& custom_mode, const int& firmware_type)
{
    
    m_andruav_vehicle_info.flying_mode = uavos::fcb::CFCBModes::getAndruavMode (custom_mode, m_andruav_vehicle_info.vehicle_type);

    m_fcb_facade.sendID(std::string());
}   


void uavos::fcb::CFCBMain::OnHomePositionUpdated(const mavlink_home_position_t& home_position)
{
    m_fcb_facade.sendHomeLocation(std::string());
}
            

void uavos::fcb::CFCBMain::alertUavosOffline()
{

}
