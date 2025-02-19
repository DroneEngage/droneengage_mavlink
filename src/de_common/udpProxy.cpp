#include <iostream>
#include <cstring> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <sys/types.h>
#include <unistd.h>
       #include <netdb.h>

#include "../helpers/colors.hpp"
#include "../helpers/json_nlohmann.hpp"
using Json_de = nlohmann::json;

#include "udpProxy.hpp"





    
de::comm::CUDPProxy::~CUDPProxy ()
{
    
    #ifdef DEBUG
	std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: ~CUDPProxy" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

    if (m_stopped_called == false)
    {
        #ifdef DEBUG
	    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: ~CUDPProxy" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        #endif

        stop();
    }

    #ifdef DEBUG
	std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: ~CUDPProxy" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

    // destroy mutex
	//pthread_mutex_destroy(&m_lock);

    #ifdef DEBUG
	std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: ~CUDPProxy" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

}

/**
 * @brief 
 * 
 * @param targetIP communication server ip
 * @param targetPort communication server port
 * @param host de-module listening ips default is 0.0.0.0
 * @param listenningPort de-module listerning port.
 */
bool de::comm::CUDPProxy::init (const char * target_address, int targetPort, const char * host, int listenningPort)
{

    // pthread initialization
	m_thread = pthread_self(); // get pthread ID
	pthread_setschedprio(m_thread, SCHED_FIFO); // setting priority


    // Creating socket file descriptor 
    if ( (m_SocketFD = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        std::cout << _ERROR_CONSOLE_TEXT_ << "UDPProxy: Socket creation failed: " << _INFO_CONSOLE_TEXT << target_address  << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return false;
    }

    m_ModuleAddress = new (struct sockaddr_in)();
    m_udpProxyServer = new (struct sockaddr_in)();
    memset(m_ModuleAddress, 0, sizeof(struct sockaddr_in)); 
    memset(m_udpProxyServer, 0, sizeof(struct sockaddr_in)); 
     
    // THIS MODULE (IP - PORT) 
    m_ModuleAddress->sin_family = AF_INET; 
    m_ModuleAddress->sin_port = htons(listenningPort); 
    m_ModuleAddress->sin_addr.s_addr = inet_addr(host);//INADDR_ANY; 

    // UDP Proxy could have a hostname
    struct hostent * hostinfo = gethostbyname(target_address);
    if(!hostinfo) {
        std::cout << _ERROR_CONSOLE_TEXT_ << "UDPProxy: Cannot get info for udpProxy" << _INFO_CONSOLE_TEXT << target_address  << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return false;
    }
    char * target_ip = inet_ntoa(*(struct in_addr *)*hostinfo->h_addr_list);
    if (!target_ip)
    {
        std::cout << _ERROR_CONSOLE_TEXT_ << "UDPProxy: Cannot connect udp proxy " << _INFO_CONSOLE_TEXT << target_address  << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return false;
    }
    std::cout << _LOG_CONSOLE_BOLD_TEXT << "UDPProxy: Trasnlate " <<  _INFO_CONSOLE_TEXT << target_address << " into " <<  target_ip << _NORMAL_CONSOLE_TEXT_ << std::endl;  
    // Communication Server (IP - PORT) 
    m_udpProxyServer->sin_family = AF_INET; 
    m_udpProxyServer->sin_port = htons(targetPort); 
    m_udpProxyServer->sin_addr.s_addr = inet_addr(target_ip); 

    // Bind the socket with the server address 
    if (bind(m_SocketFD, (const struct sockaddr *)m_ModuleAddress, sizeof(struct sockaddr_in)) > 0) 
    { 
        std::cout << _LOG_CONSOLE_BOLD_TEXT<< "UDPProxy: Listener  " << _ERROR_CONSOLE_TEXT_ << " BAD BIND: " << host << ":" << listenningPort << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return false ;
    } 

    std::cout << _LOG_CONSOLE_BOLD_TEXT<< "UDPProxy: Drone Created UDP Socket at " << _INFO_CONSOLE_TEXT << host << ":" << listenningPort << _NORMAL_CONSOLE_TEXT_ << std::endl;

    std::cout << _LOG_CONSOLE_BOLD_TEXT<< "UDPProxy: Expected UdpProxy at " <<  _INFO_CONSOLE_TEXT << target_ip << ":" <<  targetPort << _NORMAL_CONSOLE_TEXT_ << std::endl;  
    
    m_stopped_called = false;
    
    return true;
}

void de::comm::CUDPProxy::start()
{
    // call directly as we are already in a thread.
    if (m_starrted == true)
        throw "Starrted called twice";

    startReceiver ();
    m_starrted = true;
}


void de::comm::CUDPProxy::startReceiver ()
{
    m_threadCreateUDPSocket = std::thread {[&](){ InternalReceiverEntry(); }};
}


void de::comm::CUDPProxy::stop()
{

    #ifdef DEBUG
	std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: Stop" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

    m_stopped_called = true;

    if (m_SocketFD != -1)
    {
        std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Close UDP Socket" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        //https://stackoverflow.com/questions/6389970/unblock-recvfrom-when-socket-is-closed
        shutdown(m_SocketFD, SHUT_RDWR);
    }
    
    #ifdef DEBUG
	std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: Stop" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

    try
    {
        if (m_starrted) 
        {
            m_threadCreateUDPSocket.join();
            //m_threadSenderID.join();
            m_starrted = false;
        }
        delete m_ModuleAddress;
        delete m_udpProxyServer;

        #ifdef DEBUG
	    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: Stop" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        #endif
    }
    catch(const std::exception& e)
    {
        //std::cerr << e.what() << '\n';
    }

    #ifdef DEBUG
	std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: Stop" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    
    

    
}

void de::comm::CUDPProxy::InternalReceiverEntry()
{
    #ifdef DEBUG
	std::cout << "CUDPProxy::InternalReceiverEntry called" << std::endl; 
    #endif
    
    struct sockaddr_in  cliaddr;
    int n;
     __socklen_t sender_address_size = sizeof (cliaddr);
    
    while (!m_stopped_called)
    {
        // TODO: you should send header ot message length and handle if total message size is larger than MAXLINE.
        n = recvfrom(m_SocketFD, (char *)buffer, MAXLINE,  
                MSG_WAITALL, ( struct sockaddr *) &cliaddr, &sender_address_size);
        
        if (n > 0) 
        {
            buffer[n]=0;
            if (m_callback_udp_proxy != nullptr)
            {
                m_callback_udp_proxy->OnMessageReceived(this, (const char *) buffer,n);
            } 
        }
    }

    #ifdef DEBUG
	std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: InternalReceiverEntry EXIT" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

}



/**
 * Sends binary to Communicator
 **/
void de::comm::CUDPProxy::sendMSG (const char * msg, const int length)
{
    
    try
    {
        sendto(m_SocketFD, msg, length,  
            MSG_CONFIRM, (const struct sockaddr *) m_udpProxyServer, 
                sizeof(struct sockaddr_in));         
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    

}