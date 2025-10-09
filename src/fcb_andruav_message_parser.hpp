#ifndef FCB_ANDRUAV_MESSAGE_PARSER_H_
#define FCB_ANDRUAV_MESSAGE_PARSER_H_

#include <mavlink_command.h>
#include <mavlink_sdk.h>


#include "./de_common/helpers/json_nlohmann.hpp"
using Json_de = nlohmann::json;
#include "./de_common/de_databus/de_message_parser_base.hpp"
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
    class CFCBAndruavMessageParser: public de::comm::CAndruavMessageParserBase
    {
    public:
        static CFCBAndruavMessageParser& getInstance()
        {
            static CFCBAndruavMessageParser instance;
            return instance;
        }

        CFCBAndruavMessageParser(CFCBAndruavMessageParser const&) = delete;
        void operator=(CFCBAndruavMessageParser const&) = delete;

    private:
        CFCBAndruavMessageParser() {}

    public:
        ~CFCBAndruavMessageParser() {}

    protected:
        void parseRemoteExecute(Json_de &andruav_message) override;
        void parseCommand(Json_de &andruav_message, const char *full_message, const int &full_message_length, int messageType, uint32_t permission) override;

    private:
        de::fcb::CFCBMain& m_fcbMain = de::fcb::CFCBMain::getInstance();
        mission::CMissionManager& m_mission_manager = mission::CMissionManager::getInstance();
        mavlinksdk::CMavlinkSDK& m_mavlinksdk = mavlinksdk::CMavlinkSDK::getInstance();
        de::fcb::CFCBFacade& m_fcb_facade = de::fcb::CFCBFacade::getInstance();
        de::fcb::swarm::CSwarmManager& m_fcb_swarm_manager = de::fcb::swarm::CSwarmManager::getInstance();
    };
}
}

#endif