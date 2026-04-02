#include <iostream>
#include <cstring> 
#include <cerrno>
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <sys/types.h>
#include <unistd.h>
       #include <netdb.h>

#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/json_nlohmann.hpp"
#include "../de_common/de_databus/messages.hpp"
#include "../de_common/de_databus/de_facade_base.hpp"
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
    sched_param sp{};
    sp.sched_priority = 1;
    pthread_setschedparam(m_thread, SCHED_FIFO, &sp); // ignore failure


    // Creating socket file descriptor 
    if ( (m_SocketFD = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        const int err = errno;
        std::cout << _ERROR_CONSOLE_TEXT_ << "UDPProxy: Socket creation failed: " << _INFO_CONSOLE_TEXT << target_address  << _NORMAL_CONSOLE_TEXT_ << std::endl;
        de::comm::CFacade_Base::getInstance().sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), err, ERROR_3DR, NOTIFICATION_TYPE_ERROR,
            std::string("UDPProxy: Socket creation failed ") + target_address + " err:" + std::to_string(err) + " " + strerror(err));
        return false;
    }

    if (!setupSocketOptions()) {
        closeSockets();
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

    // UDP Proxy could have a hostname or IP
    in_addr ipv4{};
    if (inet_pton(AF_INET, target_address, &ipv4) == 1) {
        m_udpProxyServer->sin_addr = ipv4;
        m_dns_resolved = true;
        std::cout << _LOG_CONSOLE_BOLD_TEXT << "UDPProxy: Using IP address " <<  _INFO_CONSOLE_TEXT << target_address << _NORMAL_CONSOLE_TEXT_ << std::endl;
    } else {
        // Try DNS resolution
        addrinfo hints{}, *res = nullptr;
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;
        const int gai = getaddrinfo(target_address, nullptr, &hints, &res);
        if (gai != 0 || res == nullptr) {
            const int err = errno;
            std::cout << _ERROR_CONSOLE_TEXT_ << "UDPProxy: Cannot resolve " << _INFO_CONSOLE_TEXT
                      << target_address << _NORMAL_CONSOLE_TEXT_ << std::endl;
            de::comm::CFacade_Base::getInstance().sendErrorMessage(
                std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), err, ERROR_3DR, NOTIFICATION_TYPE_ERROR,
                std::string("UDPProxy: Cannot resolve ") + target_address + " gai:" + std::to_string(gai));
            closeSockets();
            return false;
        }
        auto* a = reinterpret_cast<sockaddr_in*>(res->ai_addr);
        m_udpProxyServer->sin_addr = a->sin_addr;
        freeaddrinfo(res);
        m_dns_resolved = true;
        std::cout << _LOG_CONSOLE_BOLD_TEXT << "UDPProxy: Resolved " <<  _INFO_CONSOLE_TEXT << target_address 
                  << " to " << inet_ntoa(m_udpProxyServer->sin_addr) << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }
    m_udpProxyServer->sin_family = AF_INET;
    m_udpProxyServer->sin_port = htons(targetPort); 

    // Bind the socket with the server address 
    if (bind(m_SocketFD, (const struct sockaddr *)m_ModuleAddress, sizeof(struct sockaddr_in)) < 0) 
    {  
        const int err = errno;
        std::cout << _LOG_CONSOLE_BOLD_TEXT<< "UDPProxy: Listener  " << _ERROR_CONSOLE_TEXT_ << " BAD BIND: " << host << ":" << listenningPort << " err:" << err << " " << strerror(err) << _NORMAL_CONSOLE_TEXT_ << std::endl;
        de::comm::CFacade_Base::getInstance().sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), err, ERROR_3DR, NOTIFICATION_TYPE_ERROR,
            std::string("UDPProxy: BAD BIND ") + host + ":" + std::to_string(listenningPort) + " err:" + std::to_string(err) + " " + strerror(err));
        closeSockets();
        return false ;
    } 

    std::cout << _LOG_CONSOLE_BOLD_TEXT<< "UDPProxy: Drone Created UDP Socket at " << _INFO_CONSOLE_TEXT << host << ":" << listenningPort << _NORMAL_CONSOLE_TEXT_ << std::endl;

    std::cout << _LOG_CONSOLE_BOLD_TEXT<< "UDPProxy: Connected to UDP Proxy at " <<  _INFO_CONSOLE_TEXT << inet_ntoa(m_udpProxyServer->sin_addr) << ":" <<  targetPort << _NORMAL_CONSOLE_TEXT_ << std::endl;  
    
    m_stopped_called = false;
    
    return true;
}

bool de::comm::CUDPProxy::setupSocketOptions()
{
    int one = 1;
    setsockopt(m_SocketFD, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    timeval rcv_to{UDP_RCV_TIMEOUT_MS / 1000, (UDP_RCV_TIMEOUT_MS % 1000) * 1000};
    timeval snd_to{UDP_SND_TIMEOUT_MS / 1000, (UDP_SND_TIMEOUT_MS % 1000) * 1000};
    setsockopt(m_SocketFD, SOL_SOCKET, SO_RCVTIMEO, &rcv_to, sizeof(rcv_to));
    setsockopt(m_SocketFD, SOL_SOCKET, SO_SNDTIMEO, &snd_to, sizeof(snd_to));

    int rcv = UDP_BUF_SIZE;
    int snd = UDP_BUF_SIZE;
    setsockopt(m_SocketFD, SOL_SOCKET, SO_RCVBUF, &rcv, sizeof(rcv));
    setsockopt(m_SocketFD, SOL_SOCKET, SO_SNDBUF, &snd, sizeof(snd));

    return true;
}

void de::comm::CUDPProxy::sendKeepAlive()
{
    static const char k = 0;
    sendto(m_SocketFD, &k, 1, MSG_CONFIRM, (const struct sockaddr *)m_udpProxyServer, sizeof(sockaddr_in));
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


void de::comm::CUDPProxy::closeSockets()
{
    if (m_SocketFD != -1)
    {
        std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Close UDP Socket" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        shutdown(m_SocketFD, SHUT_RDWR);
        close(m_SocketFD);
        m_SocketFD = -1;
    }
    delete m_ModuleAddress;  m_ModuleAddress = nullptr;
    delete m_udpProxyServer; m_udpProxyServer = nullptr;
}


void de::comm::CUDPProxy::stop()
{

    #ifdef DEBUG
	std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: Stop" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

    m_stopped_called = true;

    closeSockets();
    
    #ifdef DEBUG
	std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: Stop" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

    try
    {
        if (m_starrted) 
        {
            if (m_threadCreateUDPSocket.joinable()) m_threadCreateUDPSocket.join();
            m_starrted = false;
        }

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
        n = recvfrom(m_SocketFD, (char *)buffer, MAXLINE,  
                0, ( struct sockaddr *) &cliaddr, &sender_address_size);
        
        if (n > 0) 
        {
            const int safeN = (n < MAXLINE ? n : MAXLINE - 1);
            buffer[safeN] = 0; // safe clamp to avoid overflow
            if (m_callback_udp_proxy != nullptr)
            {
                m_callback_udp_proxy->OnMessageReceived(this, (const char *) buffer,n);
            } 
            m_last_keepalive = std::chrono::steady_clock::now();
        } else {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - m_last_keepalive).count() >= UDP_KEEPALIVE_MS) {
                    sendKeepAlive();
                    m_last_keepalive = now;
                }
                continue;
            } else if (errno == EINTR) {
                continue;
            } else if (m_stopped_called) {
                break;
            } else {
                // other errors: ignore transient, loop
                continue;
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