#include <iostream>
#include "./helpers/colors.hpp"
#include "./helpers/helpers.hpp"
#include "fcb_main.hpp"


uavos::FCB::CFCBMain::~CFCBMain()
{

}


int uavos::FCB::CFCBMain::getConnectionType ()
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


bool uavos::FCB::CFCBMain::connectToFCB ()
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


void uavos::FCB::CFCBMain::init (const Json &jsonConfig)
{
    m_jsonConfig = jsonConfig;

    if (connectToFCB() == true)
    {
        m_mavlink_sdk.start();
    }
    

}




void uavos::FCB::CFCBMain::alertUavosOffline()
{

}
