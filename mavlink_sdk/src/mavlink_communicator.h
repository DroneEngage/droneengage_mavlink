#ifndef MAVLINK_COMMUNICATOR_H_
#define MAVLINK_COMMUNICATOR_H_

#include <memory>
#include <thread>         // std::thread
#include <mutex>          // std::mutex, std::unique_lock
#include "generic_port.h"

namespace mavlinksdk
{
namespace comm
{
 


    class CCallBack_Communicator
    {
        public:

        virtual void OnMessageReceived (const mavlink_message_t& mavlink_message) {};
        virtual void OnConnected (const bool& connected) {};
    };


    
    /**
     * @brief 
     * @details Important: This class is singleton so that module can handle only one vehicle. It is designed to be simple.
     * 
     * 
     */
    class CMavlinkCommunicator
    {

        public:
        
            CMavlinkCommunicator (std::shared_ptr<mavlinksdk::comm::GenericPort> port,
                        CCallBack_Communicator* callback_communicator): m_port(port), m_callback_communicator(callback_communicator)
            {
                  
            }

            ~CMavlinkCommunicator();

        protected:
            std::shared_ptr<mavlinksdk::comm::GenericPort> m_port;
            pthread_t m_read_tid;
	        pthread_t m_write_tid;
            bool m_time_to_exit = false;
            bool m_reading_status = false;
            bool m_writing_status = false;
            bool m_connected = false;
            CCallBack_Communicator* m_callback_communicator;

        public:
            void start ();
            void stop ();
            const int send_message (const mavlink_message_t& mavlink_message);
            void readThread ();
            //void writeThread ();

        protected:
            void read_messages ();
            std::thread m_threadRead;
            std::thread m_threadWrite;
        
    };

}
    
}

#endif // MAVLINK_COMMUNICATOR_H_
