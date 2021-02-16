#ifndef FCB_MAIN_H_
#define FCB_MAIN_H_
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
            
            ~CFCBMain ();

        public:

            void init (const Json &jsonConfig);
            void RegisterSendJMSG (SENDJMSG_CALLBACK sendJMSG)
            {
                m_fcb_facade.setSendJMSG(sendJMSG);
            };            
            /* cannot connect to uavos comm*/
            void alertUavosOffline ();

        // Events implementation of mavlinksdk::CMavlinkEvents
        public:
            void OnMessageReceived (mavlink_message_t& mavlink_message) override;           
            void OnConnected (const bool connected) override;
            void OnHeartBeat_First (const mavlink_heartbeat_t& heartbeat) override;
            void OnHeartBeat_Resumed (const mavlink_heartbeat_t& heartbeat) override ;
            void OnArmed (const bool armed) override;
            void OnFlying (const bool isFlying) override;
            void OnStatusText (const std::uint8_t severity, const std::string& status) override;
            void OnModeChanges(const int custom_mode, const int firmware_type);

            

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
            
            
    };
}
}
#endif