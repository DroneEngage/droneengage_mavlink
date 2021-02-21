
#include <iostream>


#include "serial_port.h"
#include "udp_port.h"

#include "mavlink_sdk.h"


void mavlinksdk::CMavlinkSDK::start(mavlinksdk::CMavlinkEvents * mavlink_events)
{
    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "MavlinkSDK Started" << _NORMAL_CONSOLE_TEXT_ << std::endl;    

    this->m_mavlink_events = mavlink_events;

    this->m_port.get()->start();
    
    this->m_vehicle      = std::unique_ptr<mavlinksdk::CVehicle> ( new mavlinksdk::CVehicle(*this->m_callback_vehicle));
    this->m_mavlink_waypoint_manager= std::unique_ptr<mavlinksdk::CMavlinkWayPointManager> ( new mavlinksdk::CMavlinkWayPointManager(*this->m_callback_waypoint));
    this->m_communicator = std::unique_ptr<mavlinksdk::comm::CMavlinkCommunicator> ( new mavlinksdk::comm::CMavlinkCommunicator(this->m_port, this));
    this->m_communicator.get()->start();


}

void mavlinksdk::CMavlinkSDK::connectUDP (const char *target_ip, int udp_port)
{
    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "connectUDP on " << target_ip << " port " << udp_port << _NORMAL_CONSOLE_TEXT_ << std::endl;    

    this->m_port = std::shared_ptr<mavlinksdk::comm::GenericPort>( new mavlinksdk::comm::UDPPort(target_ip, udp_port));
}

void mavlinksdk::CMavlinkSDK::connectSerial (const char *uart_name, int baudrate)
{
    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "connectSerial on " << uart_name << " baudrate " << baudrate << _NORMAL_CONSOLE_TEXT_ << std::endl;    

    this->m_port   = std::shared_ptr<mavlinksdk::comm::GenericPort>( new mavlinksdk::comm::SerialPort(uart_name, baudrate));
}

void mavlinksdk::CMavlinkSDK::stop()
{

    if (this->m_port.get()!= nullptr)
        this->m_port.get()->stop();    
    this->m_stopped_called = true;
}

mavlinksdk::CMavlinkSDK::~CMavlinkSDK()
{
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: ~CMavlinkSDK" << _NORMAL_CONSOLE_TEXT_ << std::endl;

    if (this->m_stopped_called == false)
    {
        this->stop();
    }
}


void mavlinksdk::CMavlinkSDK::OnMessageReceived (mavlink_message_t& mavlink_message)
{
    //std::cout << _SUCCESS_CONSOLE_TEXT_ << "Message Received" << _NORMAL_CONSOLE_TEXT_ << std::endl;    
    try
    {
        m_sysid  = mavlink_message.sysid;
	    m_compid = mavlink_message.compid;

        this->m_vehicle.get()->parseMessage(mavlink_message);
        this->m_mavlink_waypoint_manager.get()->parseMessage(mavlink_message);

    
    this->m_mavlink_events->OnMessageReceived(mavlink_message);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    
}

void mavlinksdk::CMavlinkSDK::OnConnected (const bool& connected) 
{
    std::cout << _SUCCESS_CONSOLE_TEXT_ << "Connected Live" << _NORMAL_CONSOLE_TEXT_ << std::endl;    

    this->m_mavlink_events->OnConnected (connected);
}


void mavlinksdk::CMavlinkSDK::sendMavlinkMessage (const mavlink_message_t& mavlink_message)
{
    
    this->m_communicator.get()->send_message(mavlink_message);

    return ;
}
