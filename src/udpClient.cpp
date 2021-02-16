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

using namespace uavos;

#define MAXLINE 8192 
char buffer[MAXLINE]; 
    
// uavos::comm::CUDPClient::CUDPClient (const char * targetIP, int broadcatsPort, const char * host, int listenningPort)
// {

//     // pthread initialization
// 	m_thread = pthread_self(); // get pthread ID
// 	pthread_setschedprio(m_thread, SCHED_FIFO); // setting priority


//     // Creating socket file descriptor 
//     if ( (m_SocketFD = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
//         perror("socket creation failed"); 
//         exit(EXIT_FAILURE); 
//     }

//     m_ModuleAddress = new (struct sockaddr_in)();
//     m_CommunicatorModuleAddress = new (struct sockaddr_in)();
//     memset(m_ModuleAddress, 0, sizeof(struct sockaddr_in)); 
//     memset(m_CommunicatorModuleAddress, 0, sizeof(struct sockaddr_in)); 
     
//     // THIS MODULE (IP - PORT) 
//     m_ModuleAddress->sin_family = AF_INET; 
//     m_ModuleAddress->sin_port = htons(listenningPort); 
//     m_ModuleAddress->sin_addr.s_addr = inet_addr(host);//INADDR_ANY; 
    
//     // Communication Server (IP - PORT) 
//     m_CommunicatorModuleAddress->sin_family = AF_INET; 
//     m_CommunicatorModuleAddress->sin_port = htons(broadcatsPort); 
//     m_CommunicatorModuleAddress->sin_addr.s_addr = inet_addr(targetIP); 

//     // Bind the socket with the server address 
//     if (bind(m_SocketFD, (const struct sockaddr *)m_ModuleAddress, sizeof(struct sockaddr_in)) < 0 ) 
//     { 
//         perror("bind failed"); 
//         exit(EXIT_FAILURE); 
//     } 

//     std::cout << "UDP Listener at " << _LOG_CONSOLE_TEXT_BOLD_ << host << ":" << listenningPort << _NORMAL_CONSOLE_TEXT_ << std::endl;

//     std::cout << "Expected Comm Server at " <<  _LOG_CONSOLE_TEXT_BOLD_ << targetIP << ":" <<  broadcatsPort << _NORMAL_CONSOLE_TEXT_ << std::endl;  


// }

uavos::comm::CUDPClient::~CUDPClient ()
{
    
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: ~CUDPClient" << _NORMAL_CONSOLE_TEXT_ << std::endl;

    if (m_stopped_called == false)
    {
        std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: ~CUDPClient" << _NORMAL_CONSOLE_TEXT_ << std::endl;

        stop();
    }

    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: ~CUDPClient" << _NORMAL_CONSOLE_TEXT_ << std::endl;

    // destroy mutex
	//pthread_mutex_destroy(&m_lock);

    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: ~CUDPClient" << _NORMAL_CONSOLE_TEXT_ << std::endl;

}

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
        throw "bind failed";
    } 

    std::cout << "UDP Listener at " << _LOG_CONSOLE_TEXT_BOLD_ << host << ":" << listenningPort << _NORMAL_CONSOLE_TEXT_ << std::endl;

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
    // if(pthread_create(&m_threadCreateUDPSocket, NULL, this->InternalReceiverThreadEntryFunc, (void *) this)) {
    //     fprintf(stderr, "Error creating thread\n");
    //     return ;
    // }

    m_threadCreateUDPSocket = std::thread (this->InternalReceiverThreadEntryFunc, (void *) this);
}


void uavos::comm::CUDPClient::startSenderID ()
{
    // const bool bsend = (pthread_create(&m_threadSenderID, NULL, this->InternalSenderIDThreadEntryFunc, (void *) this) == 0);

    // if (!bsend)
    // {
    //     std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "Error creating UDP SenderID thread" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    //     exit(EXIT_FAILURE); 
    // }

    m_threadSenderID = std::thread (this->InternalSenderIDThreadEntryFunc, (void *) this);

}


void uavos::comm::CUDPClient::stop()
{

    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: Stop" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    
    m_stopped_called = true;

    if (m_SocketFD != -1)
        close (m_SocketFD);
    
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: Stop" << _NORMAL_CONSOLE_TEXT_ << std::endl;

    try
    {
        usleep (500);
        //pthread_join(m_threadSenderID, NULL); 	// close the thread
        //pthread_join(m_threadCreateUDPSocket, NULL); 	// close the thread
        m_threadCreateUDPSocket.join();
        m_threadSenderID.join();
        //pthread_join(m_thread, NULL); 	// close the thread
        //close(m_SocketFD); 					// close UDP socket
        usleep (500);
        delete m_ModuleAddress;
        delete m_CommunicatorModuleAddress;

        std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: Stop" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }
    catch(const std::exception& e)
    {
        //std::cerr << e.what() << '\n';
    }

    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: Stop" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    
    

    
}

void uavos::comm::CUDPClient::InternalReceiverEntry()
{
    std::cout << "InternalReceiverEntry called" << std::endl; 
    
    struct sockaddr_in  cliaddr;
    int n;
    __socklen_t len;
    
    while (!m_stopped_called)
    {
        
        n = recvfrom(m_SocketFD, (char *)buffer, MAXLINE,  
                MSG_WAITALL, ( struct sockaddr *) &cliaddr, &len);
        
        if (n > 0) 
        {
            buffer[n]=0;
            if (m_OnReceive != NULL)
            {
                m_OnReceive((const char *) buffer,n);
            } 
        }
    }

    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: InternalReceiverEntry EXIT" << _NORMAL_CONSOLE_TEXT_ << std::endl;

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

    std::cout << "InternelSenderIDEntry called" << std::endl; 
    while (!m_stopped_called)
    {   
        if (m_JsonID.empty() == false)
        {
            //std::cout << m_JsonID.is_null() << " - " << m_JsonID.empty() << "-" << m_JsonID.is_string() << std::endl;
            SendJMSG(m_JsonID);
        }
        sleep (1);
    }

    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: InternelSenderIDEntry EXIT" << _NORMAL_CONSOLE_TEXT_ << std::endl;

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
 * Sends JMSG to Communicator
 **/
void uavos::comm::CUDPClient::SendJMSG(const std::string& jmsg)
{
    
    #ifdef DEBUG        
        std::cout << _LOG_CONSOLE_TEXT << "SendJMSG: " << jmsg << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    
    try
    {
    sendto(m_SocketFD, jmsg.c_str(), jmsg.size(),  
        MSG_CONFIRM, (const struct sockaddr *) m_CommunicatorModuleAddress, 
            sizeof(struct sockaddr_in));         
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    

}