#ifndef CUDPCLIENT_H

#define CUDPCLIENT_H

#include <thread>         // std::thread
#include <mutex>          // std::mutex, std::unique_lock

#ifndef MAXLINE
#define MAXLINE 65507 
#endif

namespace uavos
{
namespace comm
{
class CUDPClient
{

    public:
        //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
        // static CUDPClient& getInstance()
        // {
        //     static CUDPClient instance;

        //     return instance;
        // }

        // CUDPClient(CUDPClient const&)            = delete;
        // void operator=(CUDPClient const&)        = delete;

    public:

        CUDPClient()
        {

        }

    public:
        
        ~CUDPClient ();
        
        void init(const char * targetIP, int broadcatsPort, const char * host, int listenningPort);
        void start();
        void stop();
        void setJsonId (std::string jsonID);
        void setMessageOnReceive (void (*onReceive)(const char *, int len));
        void sendMSG(const char * msg, const int length);

        bool isStarted() const { return m_starrted;}


    protected:
                
        void startReceiver();
        void startSenderID();

        void InternalReceiverEntry();
        void InternelSenderIDEntry();

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

        
};
}
}

#endif