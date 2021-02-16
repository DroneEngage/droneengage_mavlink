#include <iostream>
#include "messages.hpp"
#include "fcb_andruav_message_parser.hpp"


void uavos::fcb::CFCBAndruavMessageParser::parseMessage (Json &andruav_message)
{
    const int messageType = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_TYPE].get<int>();
    
    if (messageType == TYPE_AndruavResala_RemoteExecute)
    {
        parseRemoteExecute(andruav_message);

        return ;
    }

    else
    {
        Json message = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_CMD];
        std::cout << "messageType: " << messageType << std::endl;
        
        switch (messageType)
        {

            case TYPE_AndruavMessage_Arm:
            {
                bool arm = message["A"].get<bool>();
                bool force = false;
                if (message.contains("D") == true)
                {
                    force = message["D"].get<bool>();
                }
                m_mavlinksdk.doArmDisarm(arm,force);
            }
            break;

            case TYPE_AndruavResala_RemoteExecute:
            {
                parseRemoteExecute(andruav_message);
            }
            break;

        }
    }
}


void uavos::fcb::CFCBAndruavMessageParser::parseRemoteExecute (Json &andruav_message)
{
    const Json cmd = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_CMD];
    const int remoteCommand = cmd["C"].get<int>();
    std::cout << "cmd: " << remoteCommand << std::endl;
    // switch (remoteCommand)
    // {
        
    // } 
}