#ifndef MAVLINK_SDK_H_
#define MAVLINK_SDK_H_

#include <memory>
#include "generic_port.h"
#include "mavlink_communicator.h"

namespace mavlinksdk
{
    class CMavlinkSDK
    {
        public:
        //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
        static CMavlinkSDK& getInstance()
        {
            static CMavlinkSDK instance;

            return instance;
        }

        CMavlinkSDK(CMavlinkSDK const&)           = delete;
        void operator=(CMavlinkSDK const&)        = delete;

        
            // Note: Scott Meyers mentions in his Effective Modern
            //       C++ book, that deleted functions should generally
            //       be public as it results in better error messages
            //       due to the compilers behavior to check accessibility
            //       before deleted status

        private:

            CMavlinkSDK() {}                    // Constructor? (the {} brackets) are needed here.

            // C++ 11
            // =======
            // We can use the better technique of deleting the methods we don't want.

        public:
            void start ();
            void connectUDP (const char *target_ip, int udp_port);
            void connectSerial (const char *uart_name, int baudrate);
            void stop();
            ~CMavlinkSDK ();
        

        protected:
              std::shared_ptr<mavlinksdk::comm::GenericPort> m_port;
              std::unique_ptr<mavlinksdk::comm::CMavlinkCommunicator> m_communicator;
              bool m_stopped_called = false;
    };
}



#endif