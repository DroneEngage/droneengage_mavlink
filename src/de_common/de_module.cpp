#include "../helpers/colors.hpp"
#include "de_module.hpp"







void de::comm::CModule::defineModule (
                 std::string module_class,
                 std::string module_id,
                 std::string module_key,
                 std::string module_version,
                 Json_de message_filter
            ) 
{
    m_module_class = module_class;
    m_module_id = module_id;
    m_module_key = module_key;
    m_module_version = module_version;
    m_message_filter = message_filter;
    return ;
}


bool de::comm::CModule::init (const std::string targetIP, int broadcatsPort, const std::string host, int listenningPort,  int chunkSize)
{
    // UDP Server
    cUDPClient.init(targetIP.c_str(), broadcatsPort, host.c_str() ,listenningPort, chunkSize);
    
    createJSONID(true);
    cUDPClient.start();

    return true;
}


bool de::comm::CModule::uninit ()
{
    cUDPClient.stop();

    return true;
}


void de::comm::CModule::sendSYSMSG (const Json_de& jmsg, const int& andruav_message_id)
{
    Json_de fullMessage;

    fullMessage[ANDRUAV_PROTOCOL_TARGET_ID]         = SPECIAL_NAME_SYS_NAME; 
    fullMessage[INTERMODULE_ROUTING_TYPE]           = CMD_COMM_SYSTEM;
    fullMessage[ANDRUAV_PROTOCOL_MESSAGE_TYPE]      = andruav_message_id;
    fullMessage[ANDRUAV_PROTOCOL_MESSAGE_CMD]       = jmsg;
    
    const std::string& msg = fullMessage.dump();
    #ifdef DEBUG
        //std::cout << "sendJMSG:" << msg.c_str() << std::endl;
    #endif
    sendMSG(msg.c_str(), msg.length());         
}


/**
 * @brief sends JSON packet
 * @details sends JSON packet.
 * 
 * 
 * @param targetPartyID 
 * @param jmsg 
 * @param andruav_message_id 
 * @param internal_message if true @link INTERMODULE_MODULE_KEY @endlink equaqls to Module key
 */
void de::comm::CModule::sendJMSG (const std::string targetPartyID, const Json_de jmsg, const int andruav_message_id, const bool internal_message)
{
    std::lock_guard<std::mutex> lock(m_lock);
                
    Json_de fullMessage;

    /**
    // Route messages:
    //  Internally: i.e. DroneEngage Communication module will handle it and will resend it to other modules
    //                  or modulated then forwarded to Cmmunication Server.
    //  Group: i.e. to all members of groups.
    //  Individual: i.e. to a given member or a certain type of members i.e. all vehicles or all GCS.
    */
    std::string msg_routing_type = CMD_COMM_GROUP;
    if (internal_message == true)
    {
        msg_routing_type = CMD_TYPE_INTERMODULE;
    }
    else
    {
        if (targetPartyID.length() != 0 )
        {
            msg_routing_type = CMD_COMM_INDIVIDUAL;
        }
    }
        
    fullMessage[INTERMODULE_MODULE_KEY]             = m_module_key;
    fullMessage[ANDRUAV_PROTOCOL_TARGET_ID]         = targetPartyID; // targetID can exist even if routing is intermodule
    fullMessage[INTERMODULE_ROUTING_TYPE]           = std::string(msg_routing_type);
    fullMessage[ANDRUAV_PROTOCOL_MESSAGE_TYPE]      = andruav_message_id;
    fullMessage[ANDRUAV_PROTOCOL_MESSAGE_CMD]       = jmsg;
    const std::string& msg = fullMessage.dump();
    #ifdef DDEBUG
        std::cout << "sendJMSG:" << msg.c_str() << std::endl;
    #endif
    sendMSG(msg.c_str(), msg.length());
}


/**
 * @brief sends binary packet
 * @details sends binary packet.
 * Binary packet always has JSON header then 0 then binary data.
 * 
 * @param targetPartyID 
 * @param bmsg 
 * @param andruav_message_id 
 * @param internal_message if true @link INTERMODULE_MODULE_KEY @endlink equaqls to Module key
 * @param message_cmd JSON message in ms section of JSON header. if null then pass Json_de()
 */
void de::comm::CModule::sendBMSG (const std::string& targetPartyID, const char * bmsg, const int bmsg_length, const int& andruav_message_id, const bool& internal_message, const Json_de& message_cmd)
{
    std::lock_guard<std::mutex> lock(m_lock);
                
    Json_de fullMessage;

    std::string msg_routing_type = CMD_COMM_GROUP;
    if (internal_message == true)
    {
        msg_routing_type = CMD_TYPE_INTERMODULE;
        
    }
    else
    {
        if (targetPartyID.length() != 0 )
        {
                    
            msg_routing_type = CMD_COMM_INDIVIDUAL;
        }
        
    }
        
    fullMessage[INTERMODULE_MODULE_KEY]             = m_module_key;    
    fullMessage[ANDRUAV_PROTOCOL_TARGET_ID]         = targetPartyID; // targetID can exist even if routing is intermodule
    fullMessage[INTERMODULE_ROUTING_TYPE]           = std::string(msg_routing_type);
    fullMessage[ANDRUAV_PROTOCOL_MESSAGE_TYPE]      = andruav_message_id;
    fullMessage[ANDRUAV_PROTOCOL_MESSAGE_CMD]       = message_cmd;

    std::string json_msg = fullMessage.dump();
    
    /**** Attach Binary part to String after inserting NULL ***/

    // Prepare a vector for the whole message
    std::vector<char> msg(json_msg.begin(), json_msg.end());
    msg.push_back('\0'); // Add null terminator

    // Append binary message
    if (bmsg_length != 0)
    {
        msg.insert(msg.end(), bmsg, bmsg + bmsg_length);
    }

    // Access the complete message as a char array
    char* msg_ptr = msg.data();

    /**** Attachment End ****/


    sendMSG(msg_ptr, json_msg.length()+1+bmsg_length);

    return ;
}


/**
* @brief similar to Remote execute command but between modules.
* 
* @param command_type 
* @return const Json_de 
*/
void de::comm::CModule::sendMREMSG(const int& command_type)
{
    std::lock_guard<std::mutex> lock(m_lock);
                
    Json_de json_msg;        
        
    json_msg[INTERMODULE_MODULE_KEY]        = m_module_key;
    json_msg[INTERMODULE_ROUTING_TYPE]      =  CMD_TYPE_INTERMODULE;
    json_msg[ANDRUAV_PROTOCOL_MESSAGE_TYPE] =  TYPE_AndruavModule_RemoteExecute;
    

    Json_de ms;
    ms["C"] = command_type;
    json_msg[ANDRUAV_PROTOCOL_MESSAGE_CMD]          = ms;
    
    
    const std::string msg = json_msg.dump();
    sendMSG(msg.c_str(), msg.length());         
}


/**
 * @brief forward a message received from another channel.
 * example: P@P module receives a messages from telemetry and wants to forward it on DE databus.
 * 
 * @param message 
 * @param datalength 
 */
void de::comm::CModule::forwardMSG (const char * message, const std::size_t datalength)
{
    sendMSG(message, datalength);
}


void de::comm::CModule::onReceive (const char * message, int len)
{
    static bool bFirstReceived = false;
        
    #ifdef DDEBUG        
        std::cout << _INFO_CONSOLE_TEXT << "RX MSG: :len " << std::to_string(len) << ":" << message <<   _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    
    try
    {
        /* code */
        Json_de jMsg = Json_de::parse(message);
        
        #ifdef DDEBUG
        std::cout << _INFO_CONSOLE_TEXT << "RX MSG: jMsg" << jMsg.dump() <<   _NORMAL_CONSOLE_TEXT_ << std::endl;
        #endif

        if (!jMsg.contains(ANDRUAV_PROTOCOL_MESSAGE_TYPE)) return ;
        
        if (!jMsg.contains(INTERMODULE_ROUTING_TYPE)) return ;
        
        
        if (std::strcmp(jMsg[INTERMODULE_ROUTING_TYPE].get<std::string>().c_str(),CMD_TYPE_INTERMODULE)==0)
        {
            if (!jMsg.contains(ANDRUAV_PROTOCOL_MESSAGE_CMD)) return ;
            
            const Json_de cmd = jMsg[ANDRUAV_PROTOCOL_MESSAGE_CMD];
            

            const int messageType = jMsg[ANDRUAV_PROTOCOL_MESSAGE_TYPE].get<int>();
            switch (messageType)
            {
            case TYPE_AndruavModule_ID:
                {
                    if (!cmd.contains(JSON_INTERMODULE_PARTY_RECORD)) return ;
                    
                    const Json_de unit_ids = cmd [JSON_INTERMODULE_PARTY_RECORD];
                    if (!unit_ids.contains(ANDRUAV_PROTOCOL_SENDER)) return ;
                    if (!unit_ids.contains(ANDRUAV_PROTOCOL_GROUP_ID)) return ;
            
                    m_party_id = std::string(unit_ids[ANDRUAV_PROTOCOL_SENDER].get<std::string>());
                    m_group_id = std::string(unit_ids[ANDRUAV_PROTOCOL_GROUP_ID].get<std::string>());
                    
                    if (!bFirstReceived)
                    { 
                        // tell server you dont need to send ID again.
                        std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << " ** Communicator Server Found" << _SUCCESS_CONSOLE_TEXT_ << ": m_party_id(" << _INFO_CONSOLE_TEXT << m_party_id << _SUCCESS_CONSOLE_TEXT_ << ") m_group_id(" << _INFO_CONSOLE_TEXT << m_group_id << _SUCCESS_CONSOLE_TEXT_ << ")" <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
                        createJSONID(false);
                        bFirstReceived = true;
                    }
                    
                    if (m_OnReceive!= nullptr) m_OnReceive(message, len, jMsg);

                    return ;
                }
                break;
            
            case TYPE_AndruavMessage_DUMMY:
                {
                    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << " TYPE_AndruavMessage_DUMMY" << _SUCCESS_CONSOLE_TEXT_ << message <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
                        
                }
                break;

                default:
                    break;
            }
            
            

            
        }

        if (m_OnReceive!= nullptr) m_OnReceive(message, len, jMsg);
    }
    catch(const std::exception& e)
    {
        std::cout << "ERROR:" << e.what() << std::endl ;
    }
}


void de::comm::CModule::appendExtraField(const std::string name, const Json_de& ms)
{
    // Add the provided ms object as an entry to m_stdinValues
    m_stdinValues[name] = ms;
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
 * @return 
 */
void de::comm::CModule::createJSONID (bool reSend)
{
        Json_de json_msg;        
        
        json_msg[INTERMODULE_ROUTING_TYPE] =  CMD_TYPE_INTERMODULE;
        json_msg[ANDRUAV_PROTOCOL_MESSAGE_TYPE] =  TYPE_AndruavModule_ID;
        Json_de ms;
              
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

        // Add fields from m_stdinValues to ms
        for (const std::pair<std::string, Json_de>&  entry : m_stdinValues) {
            const std::string& key = entry.first;
            const Json_de& value = entry.second;
            ms[key] = value;
        }

        json_msg[ANDRUAV_PROTOCOL_MESSAGE_CMD] = ms;

        #ifdef DEBUG
            //std::cout << json_msg.dump(4) << std::endl;              
        #endif

        cUDPClient.setJsonId (json_msg.dump());

        return ;
}