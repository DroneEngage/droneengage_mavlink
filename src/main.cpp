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
#include "./de_common/messages.hpp"
#include "./de_common/configFile.hpp"
#include "./de_common/localConfigFile.hpp"
#include "./de_common/udpClient.hpp"
#include "./de_common/de_module.hpp"
#include "./swarm/fcb_swarm_manager.hpp"
#include "./mission/missions.hpp"
#include "fcb_facade.hpp"
#include "fcb_traffic_optimizer.hpp"
#include "./swarm/fcb_swarm_follower.hpp"
#include "fcb_main.hpp"
#include "fcb_andruav_message_parser.hpp"
#include "./mission/mission_manager.hpp"

using namespace de;

//TODO: CREATE NEW COMMAND TYPE_AndruavMessage_ACTION to collect all Drone commands.
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
                        TYPE_AndruavMessage_TrackingTarget_ACTION, \
                        TYPE_AndruavMessage_TrackingTargetLocation, \
                        TYPE_AndruavMessage_TargetTracking_STATUS, \
                        TYPE_AndruavMessage_UploadWayPoints, \
                        TYPE_AndruavMessage_RemoteControlSettings, \
                        TYPE_AndruavMessage_SET_HOME_LOCATION, \
                        TYPE_AndruavMessage_RemoteControl2, \
                        TYPE_AndruavMessage_LightTelemetry, \
                        TYPE_AndruavMessage_ServoChannel, \
                        TYPE_AndruavMessage_Sync_EventFire, \
                        TYPE_AndruavMessage_MAVLINK, \
                        TYPE_AndruavMessage_SWARM_MAVLINK, \
                        TYPE_AndruavMessage_Make_Swarm,  \
                        TYPE_AndruavMessage_FollowHim_Request,  \
                        TYPE_AndruavMessage_FollowMe_Guided, \
                        TYPE_AndruavMessage_UpdateSwarm, \
                        TYPE_AndruavMessage_UDPProxy_Info, \
                        TYPE_AndruavSystem_UDPProxy, \
                        TYPE_AndruavMessage_P2P_ACTION, \
                        TYPE_AndruavMessage_P2P_STATUS, \
                        TYPE_AndruavMessage_Upload_DE_Mission, \
                        TYPE_AndruavMessage_DUMMY}

// This is a timestamp used as instance unique number. if changed then communicator module knows module has restarted.
std::time_t instance_time_stamp;

bool exit_me = false;

// DroneEngage Current PartyID read from communicator
std::string  PartyID;
// DroneEngage Current GroupID read from communicator
std::string  GroupID;
std::string  ModuleID;
std::string  ModuleKey;
int AndruavServerConnectionStatus = SOCKET_STATUS_FREASH;

de::comm::CModule& cModule= de::comm::CModule::getInstance();

de::fcb::CFCBMain& cFCBMain = de::fcb::CFCBMain::getInstance();
de::fcb::CFCBAndruavMessageParser& cAndruavResalaParser = de::fcb::CFCBAndruavMessageParser::getInstance();

de::CConfigFile& cConfigFile = CConfigFile::getInstance();
de::CLocalConfigFile& cLocalConfigFile = de::CLocalConfigFile::getInstance();


/**
 * @brief hardware serial number
 * 
 */
static std::string hardware_serial;

/**
 * @brief configuraytion file path & name
 * 
 */
static std::string configName = "de_mavlink.config.module.json";
static std::string localConfigName = "de_mavlink.local";


        
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
 * @brief display version info
 * 
 */
void _versionOnly (void)
{
    std::cout << version_string << std::endl;
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
    std::cout << std::endl << _INFO_CONSOLE_TEXT "\t--config:          name and path of configuration file. default [./de_mavlink.config.module.json]" << _NORMAL_CONSOLE_TEXT_ << std::ends;
    std::cout << std::endl << _INFO_CONSOLE_TEXT "\t                   -c ./config.json" << _NORMAL_CONSOLE_TEXT_ << std::ends;
    std::cout << std::endl << _INFO_CONSOLE_TEXT "\t--bconfig:          name and path of configuration file. default [./de_mavlink.config.module.local]" << _NORMAL_CONSOLE_TEXT_ << std::ends;
    std::cout << std::endl << _INFO_CONSOLE_TEXT "\t                   -b ./config.local" << _NORMAL_CONSOLE_TEXT_ << std::ends;
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
 *  !NOT CALLED
 * @param status 
 */
void _onConnectionStatusChanged (const int status)
{
    cFCBMain.OnConnectionStatusChangedWithAndruavServer(status);
}


// called from: void de::comm::CModule::onReceive 
void onReceive (const char * message, int len, Json_de jMsg)
{
        
    #ifdef DEBUG        
        std::cout << _INFO_CONSOLE_TEXT << "RX MSG: :len " << std::to_string(len) << ":" << message <<   _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    
    try
    {
        const int messageType = jMsg[ANDRUAV_PROTOCOL_MESSAGE_TYPE].get<int>();

        if (std::strcmp(jMsg[INTERMODULE_ROUTING_TYPE].get<std::string>().c_str(),CMD_TYPE_INTERMODULE)==0)
        {
            const Json_de cmd = jMsg[ANDRUAV_PROTOCOL_MESSAGE_CMD];
            
        
            if (messageType== TYPE_AndruavModule_ID)
            {
                cFCBMain.setPartyID(cModule.getPartyId(), cModule.getGroupId());
                
                const int status = cmd ["g"].get<int>();
                // TODO: Status Message should be a separate message
                if (AndruavServerConnectionStatus != status)
                {
                    cFCBMain.OnConnectionStatusChangedWithAndruavServer(status);
                }
                AndruavServerConnectionStatus = status;
                
                return ;
            }

            
        }

        // /*
        // * Handles SWARM MESSAGE
        // * P2P Message SHOULD BE ENCAPSULATED
        // */
        // if (messageType == TYPE_AndruavMessage_SWARM_MAVLINK)
        //     {
        //         const std::string leader_sender = jMsg[ANDRUAV_PROTOCOL_SENDER].get<std::string>();
        //         de::fcb::swarm::CSwarmFollower& swarm_follower = de::fcb::swarm::CSwarmFollower::getInstance();
        //         swarm_follower.handle_leader_traffic(leader_sender, message, len);
            
        //         return ;
        //     }
        cAndruavResalaParser.parseMessage(jMsg, message, len);
    
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
}


void initLogger()
{
    const Json_de& jsonConfig = cConfigFile.GetConfigJSON();
    
    if ((jsonConfig.contains("logger_enabled") == false) || (jsonConfig["logger_enabled"].get<bool>()==false))
    {
        std::cout  << _LOG_CONSOLE_BOLD_TEXT<< "Logging is " << _ERROR_CONSOLE_BOLD_TEXT_ << "DISABLED" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        
        return ;
    }

    std::string log_filename = "log";
    bool debug_log = false; 
    if (jsonConfig.contains("logger_debug"))
    {
        debug_log = jsonConfig["logger_debug"].get<bool>();
    }

    std::cout  << _LOG_CONSOLE_BOLD_TEXT<< "Logging is " << _SUCCESS_CONSOLE_BOLD_TEXT_ << "ENABLED" << _NORMAL_CONSOLE_TEXT_ <<  std::endl;

        

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream log_filename_final;
    log_filename_final <<  "./logs/log_" << std::put_time(&tm, "%d-%m-%Y_%H-%M-%S") << ".log";
    mkdir("./logs/",0777);

    std::cout  << _LOG_CONSOLE_BOLD_TEXT<< "Logging to folder: " << _INFO_CONSOLE_TEXT << log_filename << _INFO_CONSOLE_BOLD_TEXT<< " filename:" << _INFO_CONSOLE_TEXT << log_filename_final.str() <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
    auto log_level = debug_log==true?plog::debug:plog::info;

    plog::init(log_level, log_filename_final.str().c_str()); 

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
        {"bconfig",        true,   0, 'b'},
        {"serial",         false,  0, 's'},
        {"version",        false,  0, 'v'},
        {"versiononly",    false,  0, 'o'},
        {"help",           false,  0, 'h'},
        {0, false, 0, 0}
    };
    // adding ':' means there is extra parameter needed
    GetOptLong gopt(argc, argv, "c:b:svoh",
                    options);

    /*
      parse command line options
     */
    while ((opt = gopt.getoption()) != -1) {
        switch (opt) {
        case 'c':
            configName = gopt.optarg;
            break;
        case 'b':
            localConfigName = gopt.optarg;
            break;
        case 'v':
            _version();
            exit(0);
            break;
        case 'o':
            _versionOnly();
            exit(0);
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

void initDEModule(int argc, char *argv[])
{
    const Json_de& jsonConfig = cConfigFile.GetConfigJSON();
    CLocalConfigFile& cLocalConfigFile = de::CLocalConfigFile::getInstance();
        
    cModule.defineModule(
        MODULE_CLASS_FCB,
        jsonConfig["module_id"],
        cLocalConfigFile.getStringField("module_key"),
        version_string,
        Json_de::array(MESSAGE_FILTER)
    );

    cModule.addModuleFeatures(MODULE_FEATURE_SENDING_TELEMETRY);
    cModule.addModuleFeatures(MODULE_FEATURE_RECEIVING_TELEMETRY);
    cModule.setHardware(hardware_serial, ENUM_HARDWARE_TYPE::HARDWARE_TYPE_CPU);
    
    

    cModule.setMessageOnReceive (&onReceive);
    
    
    int udp_chunk_size = DEFAULT_UDP_DATABUS_PACKET_SIZE;
    
    if (validateField(jsonConfig, "s2s_udp_packet_size",Json_de::value_t::string)) 
    {
        udp_chunk_size = std::stoi(jsonConfig["s2s_udp_packet_size"].get<std::string>());
    }
    else
    {
        std::cout << _INFO_CONSOLE_BOLD_TEXT << "WARNING:" << _INFO_CONSOLE_TEXT << " MISSING FIELD " << _ERROR_CONSOLE_BOLD_TEXT_ << "s2s_udp_packet_size " <<  _INFO_CONSOLE_TEXT << "is missing in config file. default value " << _ERROR_CONSOLE_BOLD_TEXT_  << std::to_string(DEFAULT_UDP_DATABUS_PACKET_SIZE) <<  _INFO_CONSOLE_TEXT <<  " is used." << _NORMAL_CONSOLE_TEXT_ << std::endl;    
    }

    // UDP Server
    cModule.init(jsonConfig["s2s_udp_target_ip"].get<std::string>().c_str(),
            std::stoi(jsonConfig["s2s_udp_target_port"].get<std::string>().c_str()),
            jsonConfig["s2s_udp_listening_ip"].get<std::string>().c_str() ,
            std::stoi(jsonConfig["s2s_udp_listening_port"].get<std::string>().c_str()),
            udp_chunk_size);
    
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

    std::cout << _LOG_CONSOLE_BOLD_TEXT << std::asctime(std::localtime(&instance_time_stamp)) << instance_time_stamp << _INFO_CONSOLE_BOLD_TEXT<< " seconds since the Epoch" << std::endl;
    
    cConfigFile.initConfigFile (configName.c_str());
    cLocalConfigFile.InitConfigFile (localConfigName.c_str());

    
    
    const Json_de& jsonConfig = cConfigFile.GetConfigJSON();
    const Json_de& jsonLocalConfig = cLocalConfigFile.GetConfigJSON();
    
    // Retrieve or Create unique ModuleKey
    ModuleKey = cLocalConfigFile.getStringField("module_key");
    if (ModuleKey=="")
    {
        
        ModuleKey = std::to_string(get_time_usec());
        cLocalConfigFile.addStringField("module_key",ModuleKey.c_str());
        cLocalConfigFile.apply();
    }

    // Retrieve or Create -if needed- tracking IDs
    if (jsonConfig.contains("follow_me"))
    {
        const Json_de& json_follow_me = jsonConfig["follow_me"];
        if (json_follow_me.contains("PID_P_X") && !jsonLocalConfig.contains("follow_me_PID_P_X"))
        {
            cLocalConfigFile.addDoubleField("follow_me_PID_P_X", json_follow_me["PID_P_X"].get<double>());
        }

        if (json_follow_me.contains("PID_P_Y") && !jsonLocalConfig.contains("follow_me_PID_P_Y"))
        {
            cLocalConfigFile.addDoubleField("follow_me_PID_P_Y", json_follow_me["PID_P_Y"].get<double>());
        }

        if (json_follow_me.contains("smoothing") && !jsonLocalConfig.contains("follow_me_smoothing"))
        {
            cLocalConfigFile.addDoubleField("follow_me_smoothing", json_follow_me["smoothing"].get<double>());
        }

        cLocalConfigFile.apply();
    }

    if (jsonConfig.contains("event_fire_channel") && jsonConfig.contains("event_wait_channel"))
    {
        de::fcb::mission::CMissionManager::getInstance().setEventChannel(jsonConfig["event_fire_channel"].get<int>(), jsonConfig["event_wait_channel"].get<int>());
    }
    
    
    initLogger();
    
    cFCBMain.init();
    
    // should be last
    initDEModule (argc,argv);

    
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
    
    cModule.uninit();

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
    #ifdef DDEBUG
        std::cout << _INFO_CONSOLE_BOLD_TEXT << " ========================== D-DEBUG ENABLED =========================="   << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
	
    init (argc, argv);

    while (!exit_me)
    {
       std::this_thread::sleep_for(std::chrono::seconds(1));
       
    }

}