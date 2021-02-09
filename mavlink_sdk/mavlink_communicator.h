#ifndef MAVLINK_COMMUNICATOR_H_
#define MAVLINK_COMMUNICATOR_H_

#include <memory>
#include <pthread.h>
#include "generic_port.h"

namespace mavlinksdk::comm
{
    void *startReadThread(void *args);
    void *startWriteThread(void *args);

    class CMavlinkCommunicator
    {

        public:
        
            CMavlinkCommunicator (std::shared_ptr<mavlinksdk::comm::GenericPort> port)
            {
                m_port = port;
            }

            ~CMavlinkCommunicator();


        protected:
            std::shared_ptr<mavlinksdk::comm::GenericPort> m_port;
            pthread_t m_read_tid;
	        pthread_t m_write_tid;
            bool m_time_to_exit = false;
            bool m_reading_status = false;


        public:
            void start ();
            void stop ();
            void _readThread ();
            void _writeThread ();

        protected:
            void read_messages ();
            

            

    };
}

#endif
