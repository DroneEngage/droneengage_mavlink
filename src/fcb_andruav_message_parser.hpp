#ifndef FCB_ANDRUAV_MESSAGE_PARSER_H_
#define FCB_ANDRUAV_MESSAGE_PARSER_H_

#include <mavlink_command.h>
#include <mavlink_sdk.h>


#include "./de_common/helpers/json_nlohmann.hpp"
using Json_de = nlohmann::json;


#include "./mission/missions.hpp"
#include "./swarm/fcb_swarm_manager.hpp"
#include "fcb_facade.hpp"
#include "fcb_traffic_optimizer.hpp"
#include "fcb_main.hpp"
#include "./mission/mission_manager.hpp"

namespace de
{
namespace fcb
{

    /**
     * @brief This class parses messages received via communicator and executes it.
     * 
     */
    class CFCBAndruavMessageParser
    {
        public:

            static CFCBAndruavMessageParser& getInstance()
            {
                static CFCBAndruavMessageParser instance;

                return instance;
            }

            CFCBAndruavMessageParser(CFCBAndruavMessageParser const&)           = delete;
            void operator=(CFCBAndruavMessageParser const&)                     = delete;

        
        private:

            CFCBAndruavMessageParser() 
            {

            }

            
        public:
            
            ~CFCBAndruavMessageParser ()
            {

            }
        
        public:

            void parseMessage (Json_de &andruav_message, const char * message, const int & message_length);
            
        protected:
            void parseRemoteExecute (Json_de &andruav_message);
   

        private:
            de::fcb::CFCBMain&  m_fcbMain = de::fcb::CFCBMain::getInstance();
            mission::CMissionManager& m_mission_manager = mission::CMissionManager::getInstance();
            mavlinksdk::CMavlinkSDK& m_mavlinksdk = mavlinksdk::CMavlinkSDK::getInstance();
            de::fcb::CFCBFacade& m_fcb_facade = de::fcb::CFCBFacade::getInstance();
            de::fcb::swarm::CSwarmManager& m_fcb_swarm_manager = de::fcb::swarm::CSwarmManager::getInstance();
    };

}
}
#endif
