#include <iostream>
#include "serial_port.h"
#include "udp_port.h"
#include "tcp_client_port.h"
#include "mavlink_sdk.h"

using namespace mavlinksdk;

void CMavlinkSDK::start(mavlinksdk::CMavlinkEvents *mavlink_events)
{
    this->m_mavlink_events = mavlink_events;
    this->m_stopped_called = false;

    this->m_port.get()->start();

    mavlinksdk::CVehicle::getInstance().set_callback_vehicle(this);
    mavlinksdk::CMavlinkWayPointManager::getInstance().setCallbackWaypoint(this);
    mavlinksdk::CMavlinkParameterManager::getInstance().set_callback_parameter(this);
    this->m_communicator = std::unique_ptr<mavlinksdk::comm::CMavlinkCommunicator>(new mavlinksdk::comm::CMavlinkCommunicator(this->m_port, this));
    this->m_communicator.get()->start();
}

void CMavlinkSDK::connectUDP(const char *target_ip, const int udp_port)
{
    std::cout << _LOG_CONSOLE_BOLD_TEXT << "UDP is listening on: " << _INFO_CONSOLE_TEXT << target_ip << _LOG_CONSOLE_BOLD_TEXT << " port " << _INFO_CONSOLE_TEXT << udp_port << _NORMAL_CONSOLE_TEXT_ << std::endl;

    this->m_port = std::shared_ptr<mavlinksdk::comm::GenericPort>(new mavlinksdk::comm::UDPPort(target_ip, udp_port));
}

void CMavlinkSDK::connectSerial(const char *uart_name, const int baudrate, const bool dynamic)
{
    std::string dynamic_str = " serial search dynamic option is disabled.";
    if (dynamic == true)
    {
        dynamic_str = " serial search dynamic option is enabled.";
    }
    std::cout << _LOG_CONSOLE_BOLD_TEXT << "connectSerial on " << _INFO_CONSOLE_TEXT << uart_name << " baudrate " << baudrate << _INFO_CONSOLE_TEXT << dynamic_str << _NORMAL_CONSOLE_TEXT_ << std::endl;

    this->m_port = std::shared_ptr<mavlinksdk::comm::GenericPort>(new mavlinksdk::comm::SerialPort(uart_name, baudrate, dynamic));
}

void CMavlinkSDK::connectTCP(const char *target_ip, const int tcp_port)
{
    std::cout << _LOG_CONSOLE_BOLD_TEXT << "TCP is listening on: " << _INFO_CONSOLE_TEXT << target_ip << _LOG_CONSOLE_BOLD_TEXT << " port " << _INFO_CONSOLE_TEXT << tcp_port << _NORMAL_CONSOLE_TEXT_ << std::endl;

    this->m_port = std::shared_ptr<mavlinksdk::comm::GenericPort>(new mavlinksdk::comm::TCPClientPort(target_ip, tcp_port));
}

void CMavlinkSDK::stop()
{
    // Stop the communicator first (this will join the read thread)
    if (this->m_communicator.get() != nullptr)
    {
        this->m_communicator.get()->stop();
        this->m_communicator.reset();
    }
    
    // Then stop the port
    if (this->m_port.get() != nullptr)
        this->m_port.get()->stop();
    
    this->m_stopped_called = true;
}

CMavlinkSDK::~CMavlinkSDK()
{
#ifdef DEBUG
    std::cout << __FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  " << _LOG_CONSOLE_TEXT << "DEBUG: ~CMavlinkSDK" << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif

    if (this->m_stopped_called == false)
    {
        this->stop();
    }
}

void CMavlinkSDK::OnMessageReceived(const mavlink_message_t &mavlink_message)
{
    try
    {
        // WARNING::: DO NOT  uncomment because in this point we do not know if this is a drone message or not.
        // if (mavlink_message.sysid != 255) {
        //     // most probably this is not your vehicle.
        //     m_sysid  = mavlink_message.sysid;
        //     m_compid = mavlink_message.compid;
        // }

        mavlinksdk::CVehicle::getInstance().parseMessage(mavlink_message);

        this->m_mavlink_events->OnMessageReceived(mavlink_message);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

void CMavlinkSDK::OnConnected(const bool &connected)
{
    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << " ** Mavlink Connected " << _NORMAL_CONSOLE_TEXT_ << std::endl;

    this->m_mavlink_events->OnConnected(connected);
}

void CMavlinkSDK::sendMavlinkMessage(const mavlink_message_t &mavlink_message)
{
    this->m_communicator.get()->send_message(mavlink_message);
}