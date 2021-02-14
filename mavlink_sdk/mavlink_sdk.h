#ifndef MAVLINK_SDK_H_
#define MAVLINK_SDK_H_

#include <memory>
#include "./helpers/colors.h"
#include "generic_port.h"
#include "mavlink_communicator.h"
#include "vehicle.h"
#include "mavlink_events.h"


namespace mavlinksdk
{
    class CMavlinkSDK : protected mavlinksdk::comm::CCallBack_Communicator, protected mavlinksdk::CCallBack_Vehicle
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

            CMavlinkSDK() 
            {
                m_sysid  = 0;
                m_compid = 0;
                m_callback_vehicle = (mavlinksdk::CCallBack_Vehicle*) this;
            }                    // Constructor? (the {} brackets) are needed here.

            // C++ 11
            // =======
            // We can use the better technique of deleting the methods we don't want.
        
        public:
            
            ~CMavlinkSDK ();

        public:
            void start (mavlinksdk::CMavlinkEvents * mavlink_events);
            void connectUDP (const char *target_ip, int udp_port);
            void connectSerial (const char *uart_name, int baudrate);
            void stop();


        // Reading Status
        public:
            int getSysId () 
            {
                return m_sysid;
            }
            int getCompId ()
            {
                return m_compid;
            }

        // Writing Status
        public:    
            void doSetMode (const uint8_t mode);
            void doArmDisarm (const bool arm, const bool force);
        

        protected:
              mavlinksdk::CMavlinkEvents * m_mavlink_events; 
              mavlinksdk::CCallBack_Vehicle* m_callback_vehicle;
              std::shared_ptr<mavlinksdk::comm::GenericPort> m_port;
              std::unique_ptr<mavlinksdk::comm::CMavlinkCommunicator> m_communicator;
              std::unique_ptr<mavlinksdk::CVehicle> m_vehicle;
              bool m_stopped_called = false;
              int m_sysid;
              int m_compid;

        // CCallback_Communicator inheritance
        protected:
            void OnMessageReceived (mavlink_message_t& mavlink_message) override;
            void OnConnected (const bool connected) override;

        // CCallback_Communicator inheritance
        protected:
            void OnHeartBeat_First (const mavlink_heartbeat_t& heartbeat) override 
            {
                //doArmDisarm(true,false);
            };
            void OnArmed (const bool armed) override 
            {
                std::cout << _INFO_CONSOLE_TEXT << "OnArmed " << std::to_string(armed) << _NORMAL_CONSOLE_TEXT_ << std::endl;    

            };
            void OnFlying (const bool isFlying) override 
            {
                std::cout << _INFO_CONSOLE_TEXT << "OnFlying " << std::to_string(isFlying) << _NORMAL_CONSOLE_TEXT_ << std::endl;    

            };

            void OnACK (const int result, const std::string& result_msg) override 
            {
                std::cout << _INFO_CONSOLE_TEXT << "OnACK " << std::to_string(result) << " - " << result_msg << _NORMAL_CONSOLE_TEXT_ << std::endl;    
            };

            void OnStatusText (const std::string& status) override 
            {
                std::cout << _INFO_CONSOLE_TEXT << "Status " << status << _NORMAL_CONSOLE_TEXT_ << std::endl;
            };
            
            void OnModeChanges(const int mode_number, const int firmware_type)  override
            {
                std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Vehicle Mode " << mavlinksdk::CMavlinkHelper::getMode (mode_number, firmware_type) << _NORMAL_CONSOLE_TEXT_ << std::endl;    
            };


    };
}



#endif // MAVLINK_SDK_H_