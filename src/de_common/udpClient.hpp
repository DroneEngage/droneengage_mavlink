#ifndef CUDPCLIENT_H

#define CUDPCLIENT_H

#include <thread>         // std::thread
#include <mutex>          // std::mutex, std::unique_lock

#ifndef MAXLINE
#define MAXLINE 65507 
#endif


#define MAX_UDP_DATABUS_PACKET_SIZE 0xffff
#define DEFAULT_UDP_DATABUS_PACKET_SIZE 8192

namespace de
{
namespace comm
{

class CCallBack_UDPClient
{
    public:
        virtual void onReceive (const char *, int len) {};
};

class CUDPClient
{

    public:

        CUDPClient(CCallBack_UDPClient * callback)
        {
            m_callback = callback;
        }

    public:
        
        ~CUDPClient ();
        
        void init(const char * targetIP, int broadcatsPort, const char * host, int listenningPort, int chunkSize);
        void start();
        void stop();
        void setJsonId (std::string jsonID);
        void sendMSG(const char * msg, const int length);

        inline bool isStarted() const { return m_starrted;}


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
        CCallBack_UDPClient* m_callback  = nullptr;

    protected:
        bool m_starrted = false;
        bool m_stopped_called = false;
        std::mutex m_lock;  
        std::mutex m_lock2;  
 
        char buffer[MAXLINE]; 
        int m_chunkSize;
        
};
}
}

#endif