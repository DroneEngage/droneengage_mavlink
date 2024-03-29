#ifndef FCB_ANDRUAV_MESSAGE_PARSER_H_
#define FCB_ANDRUAV_MESSAGE_PARSER_H_

#include <mavlink_command.h>
#include <mavlink_sdk.h>

#include "./helpers/json.hpp"
using Json = nlohmann::json;

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

            void parseMessage (Json &andruav_message, const char * message, const int & message_length);
            
        protected:
            void parseRemoteExecute (Json &andruav_message);

            // inline bool validateField (const Json& message, const char *field_name, const Json::value_t field_type)
            // {
            //     if (
            //         (message.contains(field_name) == false) 
            //         || (message[field_name].type() != field_type)
            //         ) 
            //         return false;

            //     return true;
            // }

        private:
            uavos::fcb::CFCBMain&  m_fcbMain = uavos::fcb::CFCBMain::getInstance();
            mavlinksdk::CMavlinkSDK& m_mavlinksdk = mavlinksdk::CMavlinkSDK::getInstance();
            uavos::fcb::CFCBFacade& m_fcb_facade = uavos::fcb::CFCBFacade::getInstance();
            uavos::fcb::swarm::CSwarmManager& m_fcb_swarm_manager = uavos::fcb::swarm::CSwarmManager::getInstance();
    };

}
}
#endif
