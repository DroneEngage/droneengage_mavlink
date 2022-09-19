#ifndef CUDPROXY_H

#define CUDPROXY_H

#include <thread>         // std::thread
#include <mutex>          // std::mutex, std::unique_lock

#ifndef MAXLINE
#define MAXLINE 8192 
#endif

namespace uavos
{
namespace comm
{
class CUDPProxy;
class CCallBack_UdpProxy
{
    public:

        virtual void OnMessageReceived (const uavos::comm::CUDPProxy * udp_proxy, const char * message, int len) {};
        virtual void OnConnected (const bool& connected) {};
};

class CUDPProxy
{

   public:

        CUDPProxy()
        {
            
        }
        
        ~CUDPProxy ();
        
    public:
        
        void init(const char * targetIP, int broadcatsPort, const char * host, int listenningPort);
        void setCallback (CCallBack_UdpProxy * callback_udp_proxy)
        {
            m_callback_udp_proxy = callback_udp_proxy;
        }
        void start();
        void stop();
        void sendMSG(const char * msg, const int length);

        bool isStarted() const { return m_starrted;}


    protected:
                
        void startReceiver();

        void InternalReceiverEntry();

        struct sockaddr_in  *m_ModuleAddress = nullptr, *m_CommunicatorModuleAddress = nullptr; 
        int m_SocketFD = -1; 
        std::thread m_threadSenderID, m_threadCreateUDPSocket;
        pthread_t m_thread;

        std::string m_JsonID;
        void (*m_OnReceive)(const char *, int len) = nullptr;

    protected:
        bool m_starrted = false;
        bool m_stopped_called = false;
        std::mutex m_lock;  

        char buffer[MAXLINE]; 

        CCallBack_UdpProxy * m_callback_udp_proxy = nullptr;
        
};
}
}

#endif