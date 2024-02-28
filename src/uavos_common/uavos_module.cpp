#include "../helpers/colors.hpp"
#include "uavos_module.hpp"







void uavos::comm::CModule::defineModule (
                 std::string module_class,
                 std::string module_id,
                 std::string module_key,
                 std::string module_version,
                 Json message_filter
            ) 
{
    m_module_class = module_class;
    m_module_id = module_id;
    m_module_key = module_key;
    m_module_version = module_version;
    m_message_filter = message_filter;
    return ;
}


bool uavos::comm::CModule::init (const std::string targetIP, int broadcatsPort, const std::string host, int listenningPort)
{
    // UDP Server
    cUDPClient.init(targetIP.c_str(), broadcatsPort, host.c_str() ,listenningPort);
    
    Json jsonID = createJSONID(true);
    cUDPClient.setJsonId (jsonID.dump());
    cUDPClient.start();

    return true;
}


bool uavos::comm::CModule::uninit ()
{
    cUDPClient.stop();
}

           
void uavos::comm::CModule::onReceive (const char * message, int len)
{
    static bool bFirstReceived = false;
        
    #ifdef DEBUG        
        std::cout << _INFO_CONSOLE_TEXT << "RX MSG: :len " << std::to_string(len) << ":" << message <<   _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    
    try
    {
        /* code */
        Json jMsg = Json::parse(message);
        const int messageType = jMsg[ANDRUAV_PROTOCOL_MESSAGE_TYPE].get<int>();

        if (std::strcmp(jMsg[INTERMODULE_ROUTING_TYPE].get<std::string>().c_str(),CMD_TYPE_INTERMODULE)==0)
        {
            const Json cmd = jMsg[ANDRUAV_PROTOCOL_MESSAGE_CMD];
            
        
            if (messageType== TYPE_AndruavModule_ID)
            {
                const Json moduleID = cmd ["f"];
                m_party_id = std::string(moduleID[ANDRUAV_PROTOCOL_SENDER].get<std::string>());
                m_group_id = std::string(moduleID[ANDRUAV_PROTOCOL_GROUP_ID].get<std::string>());
                
                if (!bFirstReceived)
                { 
                    // tell server you dont need to send ID again.
                    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << " ** Communicator Server Found" << _SUCCESS_CONSOLE_TEXT_ << ": m_party_id(" << _INFO_CONSOLE_TEXT << m_party_id << _SUCCESS_CONSOLE_TEXT_ << ") m_group_id(" << _INFO_CONSOLE_TEXT << m_group_id << _SUCCESS_CONSOLE_TEXT_ << ")" <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
                    Json jsonID = createJSONID(false);
                    cUDPClient.setJsonId (jsonID.dump());
                    bFirstReceived = true;
                }
                
                if (m_OnReceive!= nullptr) m_OnReceive(message, len);

                return ;
            }

            
        }

        if (m_OnReceive!= nullptr) m_OnReceive(message, len);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

/**
 * @brief creates JSON message that identifies Module
 * @details generates JSON message that identifies module
 * 'a': module_id
 * 'b': module_class. fixed "fcb"
 * 'c': module_messages. can be updated from config file.
 * 'd': module_features. fixed per module. [T,R]
 * 'e': module_key. uniqueley identifies this instance and can be set in config file.
 * 's': hardware_serial. 
 * 't': hardware_type. 
 * 'z': resend request flag
 * @param reSend if true then server should reply with server json_msg
 * @return const Json 
 */
Json uavos::comm::CModule::createJSONID (bool reSend) const
{
        Json json_msg;        
        
        json_msg[INTERMODULE_ROUTING_TYPE] =  CMD_TYPE_INTERMODULE;
        json_msg[ANDRUAV_PROTOCOL_MESSAGE_TYPE] =  TYPE_AndruavModule_ID;
        Json ms;
              
        ms[JSON_INTERMODULE_MODULE_ID]              = m_module_id;
        ms[JSON_INTERMODULE_MODULE_CLASS]           = m_module_class;
        ms[JSON_INTERMODULE_MODULE_MESSAGES_LIST]   = m_message_filter;
        ms[JSON_INTERMODULE_MODULE_FEATURES]        = m_module_features;
        ms[JSON_INTERMODULE_MODULE_KEY]             = m_module_key; 
        ms[JSON_INTERMODULE_HARDWARE_ID]            = m_hardware_serial; 
        ms[JSON_INTERMODULE_HARDWARE_TYPE]          = m_hardware_serial_type; 
        ms[JSON_INTERMODULE_VERSION]                = m_module_version;
        ms[JSON_INTERMODULE_RESEND]                 = reSend;
        ms[JSON_INTERMODULE_TIMESTAMP_INSTANCE]     = m_instance_time_stamp;

        json_msg[ANDRUAV_PROTOCOL_MESSAGE_CMD] = ms;
        #ifdef DEBUG
            //std::cout << json_msg.dump(4) << std::endl;              
        #endif
        return json_msg;
}