#ifndef FCB_MAIN_H_
#define FCB_MAIN_H_
#include <thread>         // std::thread
#include <memory>
#include <common/mavlink.h>
#include <mavlink_sdk.h>
#include <mavlink_events.h>

#include "./helpers/json.hpp"
using Json = nlohmann::json;


#include "defines.hpp"
#include "fcb_facade.hpp"

namespace uavos
{
namespace fcb
{


    
    class CFCBMain: public mavlinksdk::CMavlinkEvents
    {
        public:
            //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
            static CFCBMain& getInstance()
            {
                static CFCBMain instance;

                return instance;
            }

            CFCBMain(CFCBMain const&)               = delete;
            void operator=(CFCBMain const&)         = delete;

        
            // Note: Scott Meyers mentions in his Effective Modern
            //       C++ book, that deleted functions should generally
            //       be public as it results in better error messages
            //       due to the compilers behavior to check accessibility
            //       before deleted status

        private:

            CFCBMain()
            {

            }

        public:
            
            ~CFCBMain ()
            {
                if (m_exit_thread == false)
                {
                    uninit();
                }
            };
                

        public:

            void init (const Json &jsonConfig);
            void uninit ();

            void loopScheduler();

            void registerSendJMSG (SENDJMSG_CALLBACK sendJMSG)
            {
                m_fcb_facade.setSendJMSG(sendJMSG);
            };            

            /* cannot connect to uavos comm*/
            void alertUavosOffline ();
            
            const ANDRUAV_VEHICLE_INFO& getAndruavVehicleInfo ()
            {
                return m_andruav_vehicle_info;
            }


            

        // Events implementation of mavlinksdk::CMavlinkEvents
        public:
            void OnMessageReceived (mavlink_message_t& mavlink_message) override;           
            void OnConnected (const bool& connected) override;
            void OnHeartBeat_First (const mavlink_heartbeat_t& heartbeat) override;
            void OnHeartBeat_Resumed (const mavlink_heartbeat_t& heartbeat) override ;
            void OnArmed (const bool& armed) override;
            void OnFlying (const bool& isFlying) override;
            void OnStatusText (const std::uint8_t& severity, const std::string& status) override;
            void OnModeChanges(const int& custom_mode, const int& firmware_type) override;
            void OnHomePositionUpdated(const mavlink_home_position_t& home_position)  override;
            void OnMissionACK (const int& result, const int& mission_type, const std::string& result_msg)override;
            void OnACK (const int& result, const std::string& result_msg) override;
        

        protected: 
            void OnHeartBeat ();
        
        protected:
            mavlinksdk::CMavlinkSDK& m_mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
            uavos::fcb::CFCBFacade& m_fcb_facade = uavos::fcb::CFCBFacade::getInstance();
        
        protected:
            int getConnectionType ();
            bool connectToFCB ();
            
            

        protected:
            Json m_jsonConfig;
            int m_connection_type;
            ANDRUAV_VEHICLE_INFO m_andruav_vehicle_info;
            
            
            
            bool m_exit_thread = false;
            std::thread m_scheduler_thread;

        private:
            u_int64_t m_last_start_flying =0 ;
            u_int64_t m_counter =0;
    };
}
}
#endif