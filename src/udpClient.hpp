#ifndef CUDPCLIENT_H

#define CUDPCLIENT_H

#include <thread>         // std::thread
#include <mutex>          // std::mutex, std::unique_lock


namespace uavos::comm
{
class CUDPClient
{

    public:
        //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
        static CUDPClient& getInstance()
        {
            static CUDPClient instance;

            return instance;
        }

        CUDPClient(CUDPClient const&)            = delete;
        void operator=(CUDPClient const&)        = delete;

    private:

        CUDPClient()
        {

        }

    public:
        
        ~CUDPClient ();
        void init(const char * targetIP, int broadcatsPort, const char * host, int listenningPort);
        void start();
        void stop();
        void SetJSONID (std::string jsonID);
        void SetMessageOnReceive (void (*onReceive)(const char *, int len));
        void SendJMSG(const std::string& jmsg);

    protected:
        // This static function only needed once
        // it sends ID to communicator. 
        // you need to create UDP with communicator first.
        static void * InternalSenderIDThreadEntryFunc(void * func);
        static void * InternalReceiverThreadEntryFunc(void * func);

        
        void startReceiver();
        void startSenderID();

        void InternalReceiverEntry();
        void InternelSenderIDEntry();

        struct sockaddr_in  *m_ModuleAddress, *m_CommunicatorModuleAddress; 
        int m_SocketFD = -1; 
        std::thread m_threadSenderID, m_threadCreateUDPSocket;
        pthread_t m_thread;

        std::string m_JsonID;
        void (*m_OnReceive)(const char *, int len);

    protected:
        bool m_starrted = false;
        bool m_stopped_called = false;
        std::mutex m_lock;  
        
};
}

#endif