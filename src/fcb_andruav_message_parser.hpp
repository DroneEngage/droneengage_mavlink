#ifndef FCB_ANDRUAV_MESSAGE_PARSER_H_
#define FCB_ANDRUAV_MESSAGE_PARSER_H_

#include <mavlink_command.h>
#include <mavlink_sdk.h>

#include "./helpers/json.hpp"
using Json = nlohmann::json;

#include "fcb_main.hpp"

namespace uavos
{
namespace fcb
{

    /**
     * @brief This class parses messages received via communicator and executes it.
     * 
     */
    class CFCBAndruavResalaParser
    {
        public:

        CFCBAndruavResalaParser()
        {

        };


        public:

            void parseMessage (Json &andruav_message, const char * message, const int & message_length);
            void Scheduler_1Hz ();
            
        protected:
            void parseRemoteExecute (Json &andruav_message);

            inline bool validateField (const Json& message, const char *field_name, const Json::value_t field_type)
            {
                if (
                    (message.contains(field_name) == false) 
                    || (message[field_name].type() != field_type)
                    ) 
                    return false;

                return true;
            }

        private:
            uavos::fcb::CFCBMain&  m_fcbMain = uavos::fcb::CFCBMain::getInstance();
            mavlinksdk::CMavlinkSDK& m_mavlinksdk = mavlinksdk::CMavlinkSDK::getInstance();
            uavos::fcb::CFCBFacade& m_fcb_facade = uavos::fcb::CFCBFacade::getInstance();
    };

}
}
#endif
