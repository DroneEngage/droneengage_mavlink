#include <iostream>
#include <signal.h>


#include "version.h"
#include "./helpers/colors.hpp"
#include "./helpers/helpers.hpp"
#include "./helpers/util_rpi.hpp"
#include "./helpers/getopt_cpp.hpp"
#include "messages.hpp"
#include "configFile.hpp"
#include "udpClient.hpp"
#include "fcb_main.hpp"
#include "fcb_andruav_message_parser.hpp"

using namespace uavos;



bool exit_me = false;

// UAVOS Current PartyID read from communicator
std::string  PartyID;
// UAVOS Current GroupID read from communicator
std::string  GroupID;
std::string  ModuleID;
std::string  ModuleKey;

uavos::fcb::CFCBMain& cFCBMain = uavos::fcb::CFCBMain::getInstance();
uavos::fcb::CFCBAndruavResalaParser cAndruavResalaParser = uavos::fcb::CFCBAndruavResalaParser();

uavos::CConfigFile& cConfigFile = CConfigFile::getInstance();

uavos::comm::CUDPClient& cUDPClient = uavos::comm::CUDPClient::getInstance();  

/**
 * @brief hardware serial number
 * 
 */
static std::string hardware_serial;

/**
 * @brief configuraytion file path & name
 * 
 */
static std::string configName = "config.module.json";

/**
 * @brief module features i.e. can transmit, can recieve, needs stream ...etc.
 * in this case we use T & R only.
 */
Json module_features = Json::array();
        
void quit_handler( int sig );
void onReceive (const char * message, int len);
void uninit ();


/**
 * @brief display version info
 * 
 */
void _version (void)
{
    std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ "Drone-Engage Communicator version " << _INFO_CONSOLE_TEXT << version_string << _NORMAL_CONSOLE_TEXT_ << std::endl;
}


/**
 * @brief display help for -h command argument.
 * 
 */
void _usage(void)
{
    _version ();
    std::cout << std::endl << _INFO_CONSOLE_TEXT "Options" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    std::cout << std::endl << _INFO_CONSOLE_TEXT "\t--serial:          display serial number needed for registration" << _NORMAL_CONSOLE_TEXT_ << std::ends;
    std::cout << std::endl << _INFO_CONSOLE_TEXT "\t                   -s " << _NORMAL_CONSOLE_TEXT_ << std::ends;
    std::cout << std::endl << _INFO_CONSOLE_TEXT "\t--config:          name and path of configuration file. default [./config.module.json]" << _NORMAL_CONSOLE_TEXT_ << std::ends;
    std::cout << std::endl << _INFO_CONSOLE_TEXT "\t                   -c ./config.json" << _NORMAL_CONSOLE_TEXT_ << std::ends;
    std::cout << std::endl << _INFO_CONSOLE_TEXT "\t--version:         -v" << _NORMAL_CONSOLE_TEXT_ << std::endl;
}

/**
 * @brief display hardware serial number.
 * 
 */
void _displaySerial (void)
{
    _version ();
    std::cout << std::endl << _INFO_CONSOLE_TEXT "Serial Number: " << _TEXT_BOLD_HIGHTLITED_ << hardware_serial << _NORMAL_CONSOLE_TEXT_ << std::endl;
    
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
 */
void sendBMSG (const std::string& targetPartyID, const char * bmsg, const int bmsg_length, const int& andruav_message_id, const bool& internal_message)
{
    Json fullMessage;

    std::string msgRoutingType = CMD_COMM_GROUP;
    if (internal_message == true)
    {
        msgRoutingType = CMD_TYPE_INTERMODULE;
        fullMessage[INTERMODULE_MODULE_KEY]             = ModuleKey;
    }
    else
    {
        if (targetPartyID.length() != 0 )
        {
                    
            msgRoutingType = CMD_COMM_INDIVIDUAL;
            fullMessage[ANDRUAV_PROTOCOL_TARGET_ID]     = targetPartyID;
        }
        
    }
        
        fullMessage[INTERMODULE_COMMAND_TYPE]           = std::string(msgRoutingType);
        fullMessage[ANDRUAV_PROTOCOL_MESSAGE_TYPE]      = andruav_message_id;
        std::string json_msg = fullMessage.dump();
        
        // prepare an array for the whole message
        char * msg_ptr = new char[json_msg.length() + 1 + bmsg_length];
        std::unique_ptr<char []> msg = std::unique_ptr<char []> (msg_ptr);
        // copy json part
        strcpy(msg_ptr,json_msg.c_str());
        // add zero '0' delimeter
        msg_ptr[json_msg.length()] = 0;
        // copy binary message
        memcpy(&msg[json_msg.length()+1], bmsg, bmsg_length);

        cUDPClient.sendMSG(msg_ptr, json_msg.length()+1+bmsg_length);
        
        msg.release();
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
void sendJMSG (const std::string& targetPartyID, const Json& jmsg, const int& andruav_message_id, const bool& internal_message)
{
        
        Json fullMessage;

        std::string msgRoutingType = CMD_COMM_GROUP;
        if (internal_message == true)
        {
            msgRoutingType = CMD_TYPE_INTERMODULE;
            fullMessage[INTERMODULE_MODULE_KEY]             = ModuleKey;
        }
        else
        {
            if (targetPartyID.length() != 0 )
            {
                msgRoutingType = CMD_COMM_INDIVIDUAL;
                fullMessage[ANDRUAV_PROTOCOL_TARGET_ID]     = targetPartyID;
            }
        
        }
        
        fullMessage[INTERMODULE_COMMAND_TYPE]           = std::string(msgRoutingType);
        fullMessage[ANDRUAV_PROTOCOL_MESSAGE_TYPE]      = andruav_message_id;
        fullMessage[ANDRUAV_PROTOCOL_MESSAGE_CMD]       = jmsg;
        const std::string msg = fullMessage.dump();
        cUDPClient.sendMSG(msg.c_str(), msg.length());
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
 * @param reSend if true then server should reply with server JSONID
 * @return const Json 
 */
const Json createJSONID (bool reSend)
{
        const Json& jsonConfig = cConfigFile.GetConfigJSON();
        Json jsonID;        
        
        jsonID[INTERMODULE_COMMAND_TYPE] =  CMD_TYPE_INTERMODULE;
        jsonID[ANDRUAV_PROTOCOL_MESSAGE_TYPE] =  TYPE_AndruavModule_ID;
        Json ms;
              
        ms[JSON_INTERMODULE_MODULE_ID]              = jsonConfig["module_id"];
        ms[JSON_INTERMODULE_MODULE_CLASS]           = "fcb"; // module_class
        ms[JSON_INTERMODULE_MODULE_MESSAGES_LIST]   = jsonConfig["module_messages"];
        ms[JSON_INTERMODULE_MODULE_FEATURES]        = module_features;
        ms[JSON_INTERMODULE_MODULE_KEY]             = jsonConfig["module_key"]; 
        ms[JSON_INTERMODULE_HARDWARE_ID]            = hardware_serial; 
        ms[JSON_INTERMODULE_HARDWARE_TYPE]          = HARDWARE_TYPE_CPU; 
        ms[JSON_INTERMODULE_RESEND]                 = reSend;

        jsonID[ANDRUAV_PROTOCOL_MESSAGE_CMD] = ms;
        #ifdef DEBUG
            std::cout << jsonID.dump(4) << std::endl;              
        #endif
        return jsonID;
}


void onReceive (const char * message, int len)
{
    static bool bFirstReceived = false;
        
    #ifdef DEBUG        
        std::cout << _INFO_CONSOLE_TEXT << "RX MSG: " << message << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    
    
    Json jMsg = Json::parse(message);

    if (std::strcmp(jMsg[INTERMODULE_COMMAND_TYPE].get<std::string>().c_str(),CMD_TYPE_INTERMODULE)==0)
    {
        const Json cmd = jMsg[ANDRUAV_PROTOCOL_MESSAGE_CMD];
        const int messageType = jMsg[ANDRUAV_PROTOCOL_MESSAGE_TYPE].get<int>();
    
        if (messageType== TYPE_AndruavModule_ID)
        {
            const Json moduleID = cmd ["f"];
            PartyID = std::string(moduleID[ANDRUAV_PROTOCOL_SENDER].get<std::string>());
            GroupID = std::string(moduleID[ANDRUAV_PROTOCOL_GROUP_ID].get<std::string>());

            if (!bFirstReceived)
            { 
                // tell server you dont need to send ID again.
                std::cout << _SUCCESS_CONSOLE_TEXT_ << "Communicator Server Found " <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
                Json jsonID = createJSONID(false);
                cUDPClient.SetJSONID (jsonID.dump());
                bFirstReceived = true;
            }
            return ;
        }
    }
    
    cAndruavResalaParser.parseMessage(jMsg, message, len);
    
}


void initSerial()
{
    helpers::CUtil_Rpi::getInstance().get_cpu_serial(hardware_serial);
    hardware_serial.append(get_linux_machine_id());
}


void initArguments (int argc, char *argv[])
{
    int opt;
    const struct GetOptLong::option options[] = {
        {"config",         true,   0, 'c'},
        {"serial",         false,  0, 's'},
        {"version",        false,  0, 'v'},
        {"help",           false,  0, 'h'},
        {0, false, 0, 0}
    };
    // adding ':' means there is extra parameter needed
    GetOptLong gopt(argc, argv, "c:svh",
                    options);

    /*
      parse command line options
     */
    while ((opt = gopt.getoption()) != -1) {
        switch (opt) {
        case 'c':
            configName = gopt.optarg;
            break;
        case 'v':
            _version();
            exit(0);
            break;
        case 's':
            _displaySerial();
            exit(0);
            break;
        case 'h':
            _usage();
            exit(0);
        default:
            printf("Unknown option '%c'\n", (char)opt);
            exit(1);
        }
    }
}


/**
 * initialize components
 **/
void init (int argc, char *argv[]) 
{
    //initialize serial
    initSerial();

    initArguments (argc, argv);
    
    signal(SIGINT,quit_handler);
	
    // Reading Configuration
    std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ << "=================== " << "STARTING PLUGIN ===================" << _NORMAL_CONSOLE_TEXT_ << std::endl;


    
    // Define module features
    module_features.push_back("T");
    module_features.push_back("R");
        

    cConfigFile.initConfigFile (configName.c_str());
    const Json& jsonConfig = cConfigFile.GetConfigJSON();
    

    // UDP Server
    cUDPClient.init(jsonConfig["s2s_udp_target_ip"].get<std::string>().c_str(),
            std::stoi(jsonConfig["s2s_udp_target_port"].get<std::string>().c_str()),
            jsonConfig["s2s_udp_listening_ip"].get<std::string>().c_str() ,
            std::stoi(jsonConfig["s2s_udp_listening_port"].get<std::string>().c_str()));
    
    
    ModuleKey = jsonConfig["module_key"];
    Json jsonID = createJSONID(true);
    cUDPClient.SetJSONID (jsonID.dump());
    cUDPClient.SetMessageOnReceive (&onReceive);
    cUDPClient.start();

    cFCBMain.registerSendJMSG(sendJMSG);
    cFCBMain.registerSendBMSG(sendBMSG);
    cFCBMain.init();
    
}


void uninit ()
{

    #ifdef DEBUG
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: Unint" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

    cFCBMain.uninit();
    
    #ifdef DEBUG
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: Unint" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    
    cUDPClient.stop();

    #ifdef DEBUG
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: Unint_after Stop" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    
    // end program here
	exit(0);
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void quit_handler( int sig )
{
	std::cout << _INFO_CONSOLE_TEXT << std::endl << "TERMINATING AT USER REQUEST" <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
	
	try 
    {
        exit_me = true;
        uninit();
	}
	catch (int error){}

}


int main (int argc, char *argv[])
{
    init (argc, argv);

    while (!exit_me)
    {
       std::this_thread::sleep_for(std::chrono::seconds(1));
       
    }

}