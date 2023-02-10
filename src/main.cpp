#include <iostream>
#include <signal.h>
#include <ctime>

#include <plog/Log.h> 
#include "plog/Initializers/RollingFileInitializer.h"


#include "version.h"
#include "./helpers/colors.hpp"
#include "./helpers/helpers.hpp"
#include "./helpers/util_rpi.hpp"
#include "./helpers/getopt_cpp.hpp"
#include "./uavos_common/messages.hpp"
#include "./uavos_common/configFile.hpp"
#include "./uavos_common/udpClient.hpp"
#include "fcb_swarm_manager.hpp"
#include "./mission/missions.hpp"
#include "fcb_facade.hpp"
#include "fcb_traffic_optimizer.hpp"
#include "./uavos_common/uavos_module.hpp"
#include "fcb_main.hpp"
#include "fcb_andruav_message_parser.hpp"

using namespace uavos;

#define MESSAGE_FILTER {TYPE_AndruavMessage_RemoteExecute,\
                        TYPE_AndruavMessage_FlightControl,\
                        TYPE_AndruavMessage_GeoFence,\
                        TYPE_AndruavMessage_ExternalGeoFence,\
                        TYPE_AndruavMessage_Arm,\
                        TYPE_AndruavMessage_ChangeAltitude,\
                        TYPE_AndruavMessage_Land,\
                        TYPE_AndruavMessage_GuidedPoint,\
                        TYPE_AndruavMessage_CirclePoint,\
                        TYPE_AndruavMessage_DoYAW,\
                        TYPE_AndruavMessage_DistinationLocation, \
                        TYPE_AndruavMessage_ChangeSpeed, \
                        TYPE_AndruavMessage_TrackingTarget, \
                        TYPE_AndruavMessage_TrackingTargetLocation, \
                        TYPE_AndruavMessage_TargetLost, \
                        TYPE_AndruavMessage_UploadWayPoints, \
                        TYPE_AndruavMessage_RemoteControlSettings, \
                        TYPE_AndruavMessage_SET_HOME_LOCATION, \
                        TYPE_AndruavMessage_RemoteControl2, \
                        TYPE_AndruavMessage_LightTelemetry, \
                        TYPE_AndruavMessage_ServoChannel, \
                        TYPE_AndruavMessage_Sync_EventFire, \
                        TYPE_AndruavMessage_MAKE_SWARM,  \
                        TYPE_AndruavMessage_FollowHim_Request,  \
                        TYPE_AndruavMessage_MAVLINK, \
                        TYPE_AndruavMessage_UDPProxy_Info, \
                        TYPE_AndruavSystem_UdpProxy}

// This is a timestamp used as instance unique number. if changed then communicator module knows module has restarted.
std::time_t instance_time_stamp;

bool exit_me = false;

// UAVOS Current PartyID read from communicator
std::string  PartyID;
// UAVOS Current GroupID read from communicator
std::string  GroupID;
std::string  ModuleID;
std::string  ModuleKey;
int AndruavServerConnectionStatus = SOCKET_STATUS_FREASH;

uavos::fcb::CFCBMain& cFCBMain = uavos::fcb::CFCBMain::getInstance();
uavos::fcb::CFCBAndruavMessageParser cAndruavResalaParser = uavos::fcb::CFCBAndruavMessageParser();

uavos::CConfigFile& cConfigFile = CConfigFile::getInstance();

uavos::comm::CUDPClient cUDPClient = uavos::comm::CUDPClient(); 

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


        
void quit_handler( int sig );
void onReceive (const char * message, int len);
void uninit ();


/**
 * @brief display version info
 * 
 */
void _version (void)
{
    std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ "Drone-Engage FCB Module version " << _INFO_CONSOLE_TEXT << version_string << _NORMAL_CONSOLE_TEXT_ << std::endl;
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
 * @brief called when connection with ANdruavServer changed.
 * 
 * @param status 
 */
void _onConnectionStatusChanged (const int status)
{
    cFCBMain.OnConnectionStatusChangedWithAndruavServer(status);
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
 * @param message_cmd JSON message in ms section of JSON header. if null then pass Json()
 */
void sendBMSG (const std::string& targetPartyID, const char * bmsg, const int bmsg_length, const int& andruav_message_id, const bool& internal_message, const Json& message_cmd)
{
    Json fullMessage;

    std::string msg_routing_type = CMD_COMM_GROUP;
    if (internal_message == true)
    {
        msg_routing_type = CMD_TYPE_INTERMODULE;
        fullMessage[INTERMODULE_MODULE_KEY]             = ModuleKey;
    }
    else
    {
        if (targetPartyID.length() != 0 )
        {
                    
            msg_routing_type = CMD_COMM_INDIVIDUAL;
        }
        
    }
        
    fullMessage[ANDRUAV_PROTOCOL_TARGET_ID]         = targetPartyID; // targetID can exist even if routing is intermodule
    fullMessage[INTERMODULE_ROUTING_TYPE]           = std::string(msg_routing_type);
    fullMessage[ANDRUAV_PROTOCOL_MESSAGE_TYPE]      = andruav_message_id;
    fullMessage[ANDRUAV_PROTOCOL_MESSAGE_CMD]       = message_cmd;
    std::string json_msg = fullMessage.dump();
        
    // prepare an array for the whole message
    char * msg_ptr = new char[json_msg.length() + 1 + bmsg_length];
    std::unique_ptr<char []> msg = std::unique_ptr<char []> (msg_ptr);
    // copy json part
    strcpy(msg_ptr,json_msg.c_str());
    // add zero '0' delimeter
    msg_ptr[json_msg.length()] = 0;
    // copy binary message
    if (bmsg_length != 0)
    {
        // empty binary contents of a binary can exist if binary contents is optional
        // or will be filled by communicator module.
        memcpy(&msg[json_msg.length()+1], bmsg, bmsg_length);
    }

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

        /*////////
        // Route messages:
        //  Internally: i.e. UAVOS Communication module will handle it and will resend it to other modules
        //                  or modulated then forwarded to Cmmunication Server.
        //  Group: i.e. to all members of groups.
        //  Individual: i.e. to a given member or a certain type of members i.e. all vehicles or all GCS.
        /////////*/
        std::string msg_routing_type = CMD_COMM_GROUP;
        if (internal_message == true)
        {
            msg_routing_type = CMD_TYPE_INTERMODULE;
            fullMessage[INTERMODULE_MODULE_KEY]             = ModuleKey;
        }
        else
        {
            if (targetPartyID.length() != 0 )
            {
                msg_routing_type = CMD_COMM_INDIVIDUAL;
            }
        }
        
        fullMessage[ANDRUAV_PROTOCOL_TARGET_ID]         = targetPartyID; // targetID can exist even if routing is intermodule
        fullMessage[INTERMODULE_ROUTING_TYPE]           = std::string(msg_routing_type);
        fullMessage[ANDRUAV_PROTOCOL_MESSAGE_TYPE]      = andruav_message_id;
        fullMessage[ANDRUAV_PROTOCOL_MESSAGE_CMD]       = jmsg;
        const std::string& msg = fullMessage.dump();
        #ifdef DEBUG
        //std::cout << "sendJMSG:" << msg.c_str() << std::endl;
        #endif
        cUDPClient.sendMSG(msg.c_str(), msg.length());
}


void sendSYSMSG (const Json& jmsg, const int& andruav_message_id)
{
        
        Json fullMessage;

        fullMessage[ANDRUAV_PROTOCOL_TARGET_ID]         = std::string(SPECIAL_NAME_SYS_NAME); 
        fullMessage[INTERMODULE_ROUTING_TYPE]           = std::string(CMD_COMM_SYSTEM);
        fullMessage[ANDRUAV_PROTOCOL_MESSAGE_TYPE]      = andruav_message_id;
        fullMessage[ANDRUAV_PROTOCOL_MESSAGE_CMD]       = jmsg;
        const std::string& msg = fullMessage.dump();
        #ifdef DEBUG
        //std::cout << "sendJMSG:" << msg.c_str() << std::endl;
        #endif
        cUDPClient.sendMSG(msg.c_str(), msg.length());
}

/**
* @brief similar to Remote execute command but between modules.
* 
* @param command_type 
* @return const Json 
*/
void sendMREMSG(const int& command_type)
{
    Json json_msg;        
        
    json_msg[INTERMODULE_ROUTING_TYPE] =  CMD_TYPE_INTERMODULE;
    json_msg[ANDRUAV_PROTOCOL_MESSAGE_TYPE] =  TYPE_AndruavModule_RemoteExecute;
    Json ms;
    ms["C"] = command_type;
    json_msg[ANDRUAV_PROTOCOL_MESSAGE_CMD] = ms;
    const std::string msg = json_msg.dump();
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
 * @param reSend if true then server should reply with server json_msg
 * @return const Json 
 */
const Json createJSONID (bool reSend)
{
        const Json& jsonConfig = cConfigFile.GetConfigJSON();
        Json json_msg;        
        
        json_msg[INTERMODULE_ROUTING_TYPE] =  CMD_TYPE_INTERMODULE;
        json_msg[ANDRUAV_PROTOCOL_MESSAGE_TYPE] =  TYPE_AndruavModule_ID;
        Json ms;
              
        ms[JSON_INTERMODULE_MODULE_ID]              = jsonConfig["module_id"];
        ms[JSON_INTERMODULE_MODULE_CLASS]           = cFCBMain.getModuleClass();
        ms[JSON_INTERMODULE_MODULE_MESSAGES_LIST]   = Json::array(MESSAGE_FILTER);
        ms[JSON_INTERMODULE_MODULE_FEATURES]        = cFCBMain.getModuleFeatures();
        ms[JSON_INTERMODULE_MODULE_KEY]             = jsonConfig["module_key"]; 
        ms[JSON_INTERMODULE_HARDWARE_ID]            = hardware_serial; 
        ms[JSON_INTERMODULE_HARDWARE_TYPE]          = HARDWARE_TYPE_CPU; 
        ms[JSON_INTERMODULE_VERSION]                = version_string;
        ms[JSON_INTERMODULE_RESEND]                 = reSend;
        ms[JSON_INTERMODULE_TIMESTAMP_INSTANCE]     = instance_time_stamp;

        json_msg[ANDRUAV_PROTOCOL_MESSAGE_CMD] = ms;
        #ifdef DEBUG
            std::cout << json_msg.dump(4) << std::endl;              
        #endif
        return json_msg;
}


void onReceive (const char * message, int len)
{
    static bool bFirstReceived = false;
        
    #ifdef DEBUG        
        std::cout << _INFO_CONSOLE_TEXT << "RX MSG: " << message << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    
    
    Json jMsg = Json::parse(message);

    if (std::strcmp(jMsg[INTERMODULE_ROUTING_TYPE].get<std::string>().c_str(),CMD_TYPE_INTERMODULE)==0)
    {
        const Json cmd = jMsg[ANDRUAV_PROTOCOL_MESSAGE_CMD];
        const int messageType = jMsg[ANDRUAV_PROTOCOL_MESSAGE_TYPE].get<int>();
    
        if (messageType== TYPE_AndruavModule_ID)
        {
            const Json moduleID = cmd ["f"];
            PartyID = std::string(moduleID[ANDRUAV_PROTOCOL_SENDER].get<std::string>());
            GroupID = std::string(moduleID[ANDRUAV_PROTOCOL_GROUP_ID].get<std::string>());
            cFCBMain.setPartyID(PartyID, GroupID);
            
            const int status = cmd ["g"].get<int>();
            if (AndruavServerConnectionStatus != status)
            {
                _onConnectionStatusChanged (status);
            }
            AndruavServerConnectionStatus = status;
            if (!bFirstReceived)
            { 
                // tell server you dont need to send ID again.
                std::cout << _SUCCESS_CONSOLE_TEXT_ << "Communicator Server Found " <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
                Json jsonID = createJSONID(false);
                cUDPClient.setJsonId (jsonID.dump());
                bFirstReceived = true;
            }
            return ;
        }
    }
    
    cAndruavResalaParser.parseMessage(jMsg, message, len);
    
}


void initLogger()
{
    const Json& jsonConfig = cConfigFile.GetConfigJSON();
    
    if ((jsonConfig.contains("logger_enabled") == false) || (jsonConfig["logger_enabled"].get<bool>()==false))
    {
        std::cout  << _LOG_CONSOLE_TEXT_BOLD_ << "Logging is " << _ERROR_CONSOLE_BOLD_TEXT_ << "DISABLED" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        
        return ;
    }

    std::string log_filename = "log";
    bool debug_log = false; 
    if (jsonConfig.contains("logger_file_prfix"))
    {
        log_filename = jsonConfig["logger_file_prfix"].get<std::string>();
    }
    if (jsonConfig.contains("logger_debug"))
    {
        debug_log = jsonConfig["logger_debug"].get<bool>();
    }

    std::cout  << _LOG_CONSOLE_TEXT_BOLD_ << "Logging is " << _SUCCESS_CONSOLE_BOLD_TEXT_ << "ENABLED" << _NORMAL_CONSOLE_TEXT_ <<  std::endl;

    std::cout  << _LOG_CONSOLE_TEXT_BOLD_ << "Logging to " << _INFO_CONSOLE_TEXT << log_filename << _NORMAL_CONSOLE_TEXT_ << " detailed:" << debug_log <<  std::endl;
        

    plog::init(plog::debug, log_filename.c_str()); 

    PLOG(plog::info) << "Drone-Engage FCB Module version:" << version_string; 
    
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

void initUDPClient(int argc, char *argv[])
{
    const Json& jsonConfig = cConfigFile.GetConfigJSON();
    
    // UDP Server
    cUDPClient.init(jsonConfig["s2s_udp_target_ip"].get<std::string>().c_str(),
            std::stoi(jsonConfig["s2s_udp_target_port"].get<std::string>().c_str()),
            jsonConfig["s2s_udp_listening_ip"].get<std::string>().c_str() ,
            std::stoi(jsonConfig["s2s_udp_listening_port"].get<std::string>().c_str()));
    
    
    ModuleKey = jsonConfig["module_key"];
    Json jsonID = createJSONID(true);
    cUDPClient.setJsonId (jsonID.dump());
    cUDPClient.setMessageOnReceive (&onReceive);
    cUDPClient.start();

    
}


/**
 * initialize components
 **/
void init (int argc, char *argv[]) 
{
    instance_time_stamp = std::time(nullptr);
    
    //initialize serial
    initSerial();

    initArguments (argc, argv);
    
    signal(SIGINT,quit_handler);
	
    // Reading Configuration
    std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ << "=================== " << "STARTING PLUGIN ===================" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    _version();

    std::cout << std::asctime(std::localtime(&instance_time_stamp)) << instance_time_stamp << " seconds since the Epoch" << std::endl;

    
    cConfigFile.initConfigFile (configName.c_str());
    initLogger();
    
    
    const Json& jsonConfig = cConfigFile.GetConfigJSON();
    
    if (jsonConfig.contains("event_fire_channel") && jsonConfig.contains("event_wait_channel"))
    {
        cFCBMain.setEventChannel(jsonConfig["event_fire_channel"].get<int>(), jsonConfig["event_wait_channel"].get<int>());
    }
    
    
    cFCBMain.registerSendSYSMSG(sendSYSMSG);
    cFCBMain.registerSendJMSG(sendJMSG);
    cFCBMain.registerSendBMSG(sendBMSG);
    cFCBMain.registerSendMREMSG(sendMREMSG);
    cFCBMain.init();
    
    // should be last
    initUDPClient (argc,argv);

    
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
    instance_time_stamp = std::time(nullptr);
     
    init (argc, argv);

    while (!exit_me)
    {
       std::this_thread::sleep_for(std::chrono::seconds(1));
       
    }

}