#ifndef FCB_ANDRUAV_MESSAGE_PARSER_H_
#define FCB_ANDRUAV_MESSAGE_PARSER_H_

#include <mavlink_command.h>
#include <mavlink_sdk.h>

#include "./helpers/json.hpp"
using Json_de = nlohmann::json;

#include "./mission/missions.hpp"
#include "./swarm/fcb_swarm_manager.hpp"
#include "fcb_facade.hpp"
#include "fcb_traffic_optimizer.hpp"
#include "fcb_main.hpp"


namespace uavos
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

        CFCBAndruavMessageParser()
        {

        };


        public:

            void parseMessage (Json_de &andruav_message, const char * message, const int & message_length);
            
        protected:
            void parseRemoteExecute (Json_de &andruav_message);
   

        private:
            uavos::fcb::CFCBMain&  m_fcbMain = uavos::fcb::CFCBMain::getInstance();
            mavlinksdk::CMavlinkSDK& m_mavlinksdk = mavlinksdk::CMavlinkSDK::getInstance();
            uavos::fcb::CFCBFacade& m_fcb_facade = uavos::fcb::CFCBFacade::getInstance();
            uavos::fcb::swarm::CSwarmManager& m_fcb_swarm_manager = uavos::fcb::swarm::CSwarmManager::getInstance();
    };

}
}
#endif
