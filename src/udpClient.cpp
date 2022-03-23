#include <iostream>
#include <cstring> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <unistd.h>

#include "./helpers/colors.h"
#include "./helpers/json.hpp"
using Json = nlohmann::json;

#include "udpClient.hpp"


#define MAXLINE 8192 
char buffer[MAXLINE]; 
    
uavos::comm::CUDPClient::~CUDPClient ()
{
    
    #ifdef DEBUG
	std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: ~CUDPClient" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

    if (m_stopped_called == false)
    {
        #ifdef DEBUG
	    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: ~CUDPClient" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        #endif

        stop();
    }

    #ifdef DEBUG
	std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: ~CUDPClient" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

    // destroy mutex
	//pthread_mutex_destroy(&m_lock);

    #ifdef DEBUG
	std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: ~CUDPClient" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

}

/**
 * @brief 
 * 
 * @param targetIP communication server ip
 * @param broadcatsPort communication server port
 * @param host uavos-module listening ips default is 0.0.0.0
 * @param listenningPort uavos-module listerning port.
 */
void uavos::comm::CUDPClient::init (const char * targetIP, int broadcatsPort, const char * host, int listenningPort)
{

    // pthread initialization
	m_thread = pthread_self(); // get pthread ID
	pthread_setschedprio(m_thread, SCHED_FIFO); // setting priority


    // Creating socket file descriptor 
    if ( (m_SocketFD = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    }

    m_ModuleAddress = new (struct sockaddr_in)();
    m_CommunicatorModuleAddress = new (struct sockaddr_in)();
    memset(m_ModuleAddress, 0, sizeof(struct sockaddr_in)); 
    memset(m_CommunicatorModuleAddress, 0, sizeof(struct sockaddr_in)); 
     
    // THIS MODULE (IP - PORT) 
    m_ModuleAddress->sin_family = AF_INET; 
    m_ModuleAddress->sin_port = htons(listenningPort); 
    m_ModuleAddress->sin_addr.s_addr = inet_addr(host);//INADDR_ANY; 
    
    // Communication Server (IP - PORT) 
    m_CommunicatorModuleAddress->sin_family = AF_INET; 
    m_CommunicatorModuleAddress->sin_port = htons(broadcatsPort); 
    m_CommunicatorModuleAddress->sin_addr.s_addr = inet_addr(targetIP); 

    // Bind the socket with the server address 
    if (bind(m_SocketFD, (const struct sockaddr *)m_ModuleAddress, sizeof(struct sockaddr_in)) < 0 ) 
    { 
        std::cout << _LOG_CONSOLE_TEXT_BOLD_ << "UDP Listener  " << _ERROR_CONSOLE_TEXT_ << " BAD BIND: " << host << ":" << listenningPort << _NORMAL_CONSOLE_TEXT_ << std::endl;
        exit(-1) ;
    } 

    std::cout << _LOG_CONSOLE_TEXT_BOLD_ << "UDP Listener at " << _INFO_CONSOLE_TEXT << host << ":" << listenningPort << _NORMAL_CONSOLE_TEXT_ << std::endl;

    std::cout << "Expected Comm Server at " <<  _LOG_CONSOLE_TEXT_BOLD_ << targetIP << ":" <<  broadcatsPort << _NORMAL_CONSOLE_TEXT_ << std::endl;  
}

void uavos::comm::CUDPClient::start()
{
    // call directly as we are already in a thread.
    if (m_starrted == true)
        throw "Starrted called twice";

    startReceiver ();
    startSenderID();

    m_starrted = true;
}


void uavos::comm::CUDPClient::startReceiver ()
{
    m_threadCreateUDPSocket = std::thread (this->InternalReceiverThreadEntryFunc, (void *) this);
}


void uavos::comm::CUDPClient::startSenderID ()
{
    m_threadSenderID = std::thread (this->InternalSenderIDThreadEntryFunc, (void *) this);
}


void uavos::comm::CUDPClient::stop()
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
        //pthread_join(m_threadSenderID, NULL); 	// close the thread
        //pthread_join(m_threadCreateUDPSocket, NULL); 	// close the thread
        if (m_starrted) 
        {
            m_threadCreateUDPSocket.join();
            m_threadSenderID.join();
        }
        //pthread_join(m_thread, NULL); 	// close the thread
        //close(m_SocketFD); 					// close UDP socket
        delete m_ModuleAddress;
        delete m_CommunicatorModuleAddress;

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

void uavos::comm::CUDPClient::InternalReceiverEntry()
{
    std::cout << "InternalReceiverEntry called" << std::endl; 
    
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
            if (m_OnReceive != NULL)
            {
                m_OnReceive((const char *) buffer,n);
            } 
        }
    }

    #ifdef DEBUG
	std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: InternalReceiverEntry EXIT" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

}


/**
 * Store ID Card in JSON
 */
void uavos::comm::CUDPClient::SetJSONID (std::string jsonID)
{
    m_JsonID = jsonID;
}

void uavos::comm::CUDPClient::SetMessageOnReceive (void (*onReceive)(const char *, int len))
{
    m_OnReceive = onReceive;
}

/**
 * Sending ID Periodically
 **/
void uavos::comm::CUDPClient::InternelSenderIDEntry()
{

    #ifdef DEBUG
	std::cout << "InternelSenderIDEntry called" << std::endl; 
    #endif

    while (!m_stopped_called)
    {   
        if (m_JsonID.empty() == false)
        {
            //std::cout << m_JsonID.is_null() << " - " << m_JsonID.empty() << "-" << m_JsonID.is_string() << std::endl;
            const std::string msg = m_JsonID;
            sendMSG(msg.c_str(), msg.length());
        }
        sleep (1);
    }

    #ifdef DEBUG
	std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: InternelSenderIDEntry EXIT" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

}


/**
 * Starts ID Sender function 
 **/
void * uavos::comm::CUDPClient::InternalSenderIDThreadEntryFunc(void * This) {
	((CUDPClient *)This)->InternelSenderIDEntry(); 
    return NULL;
}


void * uavos::comm::CUDPClient::InternalReceiverThreadEntryFunc(void * This) {
	((CUDPClient *)This)->InternalReceiverEntry(); 
    return NULL;
}


/**
 * Sends binary to Communicator
 **/
void uavos::comm::CUDPClient::sendMSG (const char * msg, const int length)
{
    
    try
    {
        sendto(m_SocketFD, msg, length,  
            MSG_CONFIRM, (const struct sockaddr *) m_CommunicatorModuleAddress, 
                sizeof(struct sockaddr_in));         
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    

}