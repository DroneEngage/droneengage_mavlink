#ifndef FCB_ANDRUAV_MESSAGE_PARSER_H_
#define FCB_ANDRUAV_MESSAGE_PARSER_H_

#include <mavlink_sdk.h>

#include "./helpers/json.hpp"
using Json = nlohmann::json;

#include "fcb_main.hpp"

namespace uavos
{
namespace fcb
{

    class CFCBAndruavMessageParser
    {
        public:

        CFCBAndruavMessageParser()
        {

        };


        public:

            void parseMessage (Json &andruav_message);

        protected:
            void parseRemoteExecute (Json &andruav_message);

            uavos::fcb::CFCBMain&  m_fcbMain = uavos::fcb::CFCBMain::getInstance();
            mavlinksdk::CMavlinkSDK& m_mavlinksdk = mavlinksdk::CMavlinkSDK::getInstance();
    };

}
}
#endif
