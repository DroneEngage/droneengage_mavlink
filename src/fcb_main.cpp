#include <iostream>

#include <vehicle.h>

#include "./helpers/colors.hpp"
#include "./helpers/helpers.hpp"

#include "fcb_modes.hpp"
#include "fcb_main.hpp"
#include "fcb_traffic_optimizer.hpp"




uavos::fcb::CFCBMain::~CFCBMain()
{

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
}


void uavos::fcb::CFCBMain::OnMessageReceived (mavlink_message_t& mavlink_message)
{
    std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ << "OnMessageReceived" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    uavos::fcb::CMavlinkTrafficOptimizer::ShouldForwardThisMessage (mavlink_message);
}


void uavos::fcb::CFCBMain::OnConnected (const bool connected)
{
    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "OnConnected" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    if (m_andruav_vehicle_info.usd_fcb != connected)
    {
        m_andruav_vehicle_info.usd_fcb = connected;
    }
}


void uavos::fcb::CFCBMain::OnHeartBeat_First (const mavlink_heartbeat_t& heartbeat)
{
    
    m_andruav_vehicle_info.vehicle_type = uavos::fcb::CFCBModes::getAndruavVehicleType (heartbeat.type, heartbeat.autopilot);
}


void uavos::fcb::CFCBMain::OnHeartBeat_Resumed (const mavlink_heartbeat_t& heartbeat)
{
    std::unique_ptr<mavlinksdk::CVehicle>& vehicle = m_mavlink_sdk.getVehicle();

    m_andruav_vehicle_info.vehicle_type = uavos::fcb::CFCBModes::getAndruavVehicleType (heartbeat.type, heartbeat.autopilot);
}
            
void uavos::fcb::CFCBMain::OnArmed (const bool armed)
{
    std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ << "OnArmed" << _NORMAL_CONSOLE_TEXT_ << std::endl;
}


void uavos::fcb::CFCBMain::OnFlying (const bool isFlying)
{
    std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ << "OnFlying" << _NORMAL_CONSOLE_TEXT_ << std::endl;

    
}


void uavos::fcb::CFCBMain::OnStatusText (const std::uint8_t severity, const std::string& status)
{
    std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ << "OnStatusText" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    
    m_fcb_facade.sendErrorMessage(NULL, severity, status);
    
}

 
void uavos::fcb::CFCBMain::OnModeChanges(const int custom_mode, const int firmware_type)
{
    
    //m_fcb_facade.sendID(NULL, m_andruav_vehicle_info);
}   


void uavos::fcb::CFCBMain::alertUavosOffline()
{

}
