#include <iostream>
#include <cstdlib>
#include <csignal>

#include <mavlink_waypoint_manager.h>
#include <mavlink_parameter_manager.h>
#include <mavlink_ftp_manager.h>

#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"

#include "./helpers/colors.hpp"
#include "./helpers/helpers.hpp"

#include "./de_common/configFile.hpp"
#include "./de_common/localConfigFile.hpp"

#include "fcb_modes.hpp"


#include "fcb_main.hpp"

#include "./geofence/fcb_geo_fence_base.hpp"
#include "./geofence/fcb_geo_fence_manager.hpp"
#include "./swarm/fcb_swarm_leader.hpp"

#include "./geofence/fcb_geo_fence_base.hpp"
#include "./geofence/fcb_geo_fence_manager.hpp"
#include "./mission/mission_manager.hpp"

using namespace de::fcb;

#define UDP_PROXY_TIMEOUT 5000000

/**
 * @brief Message received from UDP Proxy
 *
 * @param udp_proxy
 * @param message
 * @param len
 */
void CFCBMain::OnMessageReceived(const de::comm::CUDPProxy *udp_proxy, const char *message, int len)
{
    // Execute messages received by MP or QGC by forwarding it to FCB.

    if (!isUdpProxyMavlinkAvailable() || getAndruavVehicleInfo().is_gcs_blocked)
        return;

    const u_int64_t now = get_time_usec();
    m_last_access_telemetry = now;

    mavlink_status_t status;
    mavlink_message_t mavlink_message;
    for (int i = 0; i < len; ++i)
    {
        uint8_t msgReceived = mavlink_parse_char(MAVLINK_CHANNEL_TELEMETRY, message[i], &mavlink_message, &status);
        if (msgReceived != 0)
        {
            mavlinksdk::CMavlinkCommand::getInstance().sendNative(mavlink_message);
        }
    }
}


/**
 * @brief get connection type UDP or serial based on config file.
 *
 * @return int
 */
int CFCBMain::getConnectionType() const
{
    std::string connection = str_tolower(m_jsonConfig["fcb_connection_uri"]["type"].get<std::string>());

    std::size_t found = connection.find("serial");
    if (found != std::string::npos)
        return CONNECTION_TYPE_SERIAL;

    found = connection.find("udp");
    if (found != std::string::npos)
        return CONNECTION_TYPE_UDP;

    found = connection.find("tcp");
    if (found != std::string::npos)
        return CONNECTION_TYPE_TCP;

    return CONNECTION_TYPE_UNKNOWN;
}

/**
 * @brief actual connect to the board in UDP or serial.
 *
 * @return true
 * @return false
 */
bool CFCBMain::connectToFCB()
{
    m_connection_type = getConnectionType();

    switch (m_connection_type)
    {
    // TODO Add Checks here for JSON fields.
    case CONNECTION_TYPE_SERIAL:
    {
        bool dynamic = false;
        if (m_jsonConfig["fcb_connection_uri"].contains("dynamic"))
        {
            dynamic = m_jsonConfig["fcb_connection_uri"]["dynamic"].get<bool>();
        }
        std::cout << _INFO_CONSOLE_TEXT << "Serial Connection Initializing" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        m_mavlink_sdk.connectSerial((m_jsonConfig["fcb_connection_uri"])["port"].get<std::string>().c_str(),
                                    (m_jsonConfig["fcb_connection_uri"])["baudrate"].get<int>(), dynamic);
    }
        return true;

    case CONNECTION_TYPE_UDP:
        std::cout << _INFO_CONSOLE_TEXT << "UDP Connection Initializing" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        m_mavlink_sdk.connectUDP((m_jsonConfig["fcb_connection_uri"])["ip"].get<std::string>().c_str(),
                                 (m_jsonConfig["fcb_connection_uri"])["port"].get<int>());
        return true;

    case CONNECTION_TYPE_TCP:
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "TCP Connection is not supported" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        exit(0);
        break;

    default:
        throw "Connection to FCB is not in (serial, udp, tcp) ";
        break;
    }

    return false;
}

bool CFCBMain::init()
{

    m_exit_thread = false;

    // Define module features

    de::CConfigFile &cConfigFile = CConfigFile::getInstance();
    m_jsonConfig = cConfigFile.GetConfigJSON();

    initVehicleChannelLimits(true);

    bool ignore = false;
    if (m_jsonConfig.contains("ignore_loading_parameters"))
    {
        ignore = m_jsonConfig["ignore_loading_parameters"];
    }
    mavlinksdk::CMavlinkParameterManager::getInstance().ignoreLoadingParameters(ignore);

    m_mavlink_optimizer.init(m_jsonConfig["message_timeouts"]);

    if (m_jsonConfig.contains("default_optimization_level"))
    { // TODO: convert this to inline as validatefield
        m_mavlink_optimizer.setOptimizationLevel(m_jsonConfig["default_optimization_level"].get<int>());
        m_mavlink_optimizer.reset_timestamps();
    }

    if (m_jsonConfig.contains("udp_proxy_enabled"))
    { // TODO: convert this to inline as validatefield
        m_enable_udp_telemetry_in_config = m_jsonConfig["udp_proxy_enabled"].get<bool>();
    }

    if (m_jsonConfig.contains("only_allow_ardupilot_sysid") && m_jsonConfig["only_allow_ardupilot_sysid"].is_number())
    {
        const int sys_id = m_jsonConfig["only_allow_ardupilot_sysid"].get<int>();
        std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Variable " << _INFO_CONSOLE_BOLD_TEXT << " only_allow_ardupilot_sysid " << _SUCCESS_CONSOLE_BOLD_TEXT_ << " is set to " << _INFO_CONSOLE_BOLD_TEXT << sys_id << _NORMAL_CONSOLE_TEXT_ << std::endl;
        std::cout << _INFO_CONSOLE_BOLD_TEXT << "NOTE OTHER SYS-IDs will be" << _ERROR_CONSOLE_BOLD_TEXT_ << " IGNORED" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        
        m_vehicle.restrictMessageToSysID(sys_id);
    }

    m_udp_telemetry_fixed_port = 0;

    if (m_enable_udp_telemetry_in_config == true)
    {
        de::CLocalConfigFile &cLocalConfigFile = de::CLocalConfigFile::getInstance();

        std::cout << _LOG_CONSOLE_BOLD_TEXT << "Udp Proxy: " << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Enabled" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        PLOG(plog::info) << "Udp Proxy Enabled";

        u_int16_t udp_proxy_fixed_port = cLocalConfigFile.getNumericField("udp_proxy_fixed_port");
        if (udp_proxy_fixed_port == 0xffff)
        { // not found in local config ... not that local config has higher priority than the user config.

            if (validateField(m_jsonConfig, "udp_proxy_fixed_port", Json_de::value_t::number_unsigned))
            { // TODO: convert this to inline as validatefield
                udp_proxy_fixed_port = m_jsonConfig["udp_proxy_fixed_port"].get<int>();
                cLocalConfigFile.addNumericField("udp_proxy_fixed_port", udp_proxy_fixed_port);
                cLocalConfigFile.apply();
                m_udp_telemetry_fixed_port = udp_proxy_fixed_port;
            }
        }
        else
        {
            m_udp_telemetry_fixed_port = udp_proxy_fixed_port;
        }

        if (m_udp_telemetry_fixed_port != 0)
        {
            std::cout << _LOG_CONSOLE_BOLD_TEXT << "Udp Proxy fixed port is:" << _INFO_CONSOLE_TEXT << m_udp_telemetry_fixed_port << _NORMAL_CONSOLE_TEXT_ << std::endl;
            PLOG(plog::info) << "Udp Proxy fixed port is:" << m_udp_telemetry_fixed_port;
        }
        else
        {
            m_udp_telemetry_fixed_port = 0;
        }
    }
    else
    {
        std::cout << _LOG_CONSOLE_BOLD_TEXT << "Udp Proxy:" << _ERROR_CONSOLE_BOLD_TEXT_ << "Disabled" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        PLOG(plog::info) << "Udp Proxy Disabled";
    }

    if (connectToFCB() == true)
    {
        m_mavlink_sdk.start(this);
    }

    de::fcb::mission::CMissionManager::getInstance().getAndruavMission().clear();
    m_andruav_vehicle_info.rc_sub_action = RC_SUB_ACTION::RC_SUB_ACTION_RELEASED;

    m_scheduler_thread = std::thread{[&]()
                                     { loopScheduler(); }};

    return true;
}

bool CFCBMain::uninit()
{
    // exit thread.
    if (m_exit_thread == true)
    {
        std::cout << "m_exit_thread == true" << std::endl;
        return true;
    }

    m_exit_thread = true;

    m_scheduler_thread.join();

#ifdef DEBUG
    std::cout << __FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  " << _LOG_CONSOLE_TEXT << "DEBUG: ~CFCBMain  Scheduler Thread Off" << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif

    return true;
}

void CFCBMain::initVehicleChannelLimits(const bool display)
{
    de::CConfigFile &cConfigFile = CConfigFile::getInstance();
    if (!cConfigFile.fileUpdated()) return ;
    cConfigFile.reloadFile();
    m_jsonConfig = cConfigFile.GetConfigJSON();

    if (m_jsonConfig.contains("rc_block_channel"))
    {
        m_andruav_vehicle_info.rc_block_channel = m_jsonConfig["rc_block_channel"].get<int>();
        
        if (display && (m_andruav_vehicle_info.rc_block_channel >= 0))
        {
            std::cout << _LOG_CONSOLE_BOLD_TEXT << "RC Blocking is " << _ERROR_CONSOLE_BOLD_TEXT_ << "enabled " << _INFO_CONSOLE_BOLD_TEXT << "at channel: " << _INFO_CONSOLE_BOLD_TEXT << std::to_string(m_andruav_vehicle_info.rc_block_channel) << _ERROR_CONSOLE_BOLD_TEXT_ << " - IMPORTANT" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        }
    }
    else
    {
        m_andruav_vehicle_info.rc_block_channel = -1;
    }

    if (!m_jsonConfig.contains("rc_channels") || !m_jsonConfig["rc_channels"].contains("rc_channel_enabled") || !m_jsonConfig["rc_channels"].contains("rc_channel_reverse") || !m_jsonConfig["rc_channels"].contains("rc_channel_limits_max") || !m_jsonConfig["rc_channels"].contains("rc_channel_limits_min"))
    {
        if (display)
        {
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "RC Channels are not defined." << _NORMAL_CONSOLE_TEXT_ << std::endl;
            std::cout << _INFO_CONSOLE_TEXT << "..... Assuming default values." << _NORMAL_CONSOLE_TEXT_ << std::endl;
            std::cout << _INFO_CONSOLE_TEXT << "..... Please define values for safety." << _NORMAL_CONSOLE_TEXT_ << std::endl;
        }

        m_rcmap_channels_info.use_smart_rc = true;
        memset(m_andruav_vehicle_info.rc_channels_enabled, true, RC_CHANNELS_MAX * sizeof(bool));
        memset(m_andruav_vehicle_info.rc_channels_reverse, false, RC_CHANNELS_MAX * sizeof(bool));
        memset(m_andruav_vehicle_info.rc_channels_min, (int16_t)1000, RC_CHANNELS_MAX * sizeof(int16_t));
        memset(m_andruav_vehicle_info.rc_channels_max, (int16_t)2000, RC_CHANNELS_MAX * sizeof(int16_t));

        return;
    }

    // Apply RC Channels settings - non smart rc section.
    int index = 0;
    const Json_de rc_channels = m_jsonConfig["rc_channels"];
    Json_de values;

    values = rc_channels["rc_channel_enabled"];
    for (auto it = values.begin(); it != values.end(); ++it)
    {
        m_andruav_vehicle_info.rc_channels_enabled[index] = *it == 1 ? true : false;
        index++;
    }

    index = 0;
    values = rc_channels["rc_channel_reverse"];
    for (auto it = values.begin(); it != values.end(); ++it)
    {
        m_andruav_vehicle_info.rc_channels_reverse[index] = *it == 1 ? true : false;
        index++;
    }

    index = 0;
    values = rc_channels["rc_channel_limits_max"];
    for (auto it = values.begin(); it != values.end(); ++it)
    {
        m_andruav_vehicle_info.rc_channels_max[index] = *it;
        index++;
    }

    index = 0;
    values = rc_channels["rc_channel_limits_min"];
    for (auto it = values.begin(); it != values.end(); ++it)
    {
        m_andruav_vehicle_info.rc_channels_min[index] = *it;
        index++;
    }

    if (rc_channels.contains("rc_smart_channels"))
    {
        if (rc_channels["rc_smart_channels"].contains("active"))
        {
            m_rcmap_channels_info.use_smart_rc = rc_channels["rc_smart_channels"]["active"].get<bool>();
        }
        else
        {
            m_rcmap_channels_info.use_smart_rc = false;
            
            
        }

        if ((m_rcmap_channels_info.use_smart_rc == true) && (m_rcmap_channels_info.is_valid))
        {
            #ifdef DDEBUG
            std::cout << _LOG_CONSOLE_BOLD_TEXT << "RC Smart Channels are" << _SUCCESS_CONSOLE_BOLD_TEXT_ << " enabled." << _NORMAL_CONSOLE_TEXT_ << std::endl;
            #endif

            // re-adjust rcmapped channels
            if (rc_channels["rc_smart_channels"].contains("rc_channel_limits_max"))
            {
                values = rc_channels["rc_smart_channels"]["rc_channel_limits_max"];
                auto it = values.begin();
                m_andruav_vehicle_info.rc_channels_max[m_rcmap_channels_info.rcmap_roll] = *it++;
                m_andruav_vehicle_info.rc_channels_max[m_rcmap_channels_info.rcmap_pitch] = *it++;
                m_andruav_vehicle_info.rc_channels_max[m_rcmap_channels_info.rcmap_throttle] = *it++;
                m_andruav_vehicle_info.rc_channels_max[m_rcmap_channels_info.rcmap_yaw] = *it;
            }

            if (rc_channels["rc_smart_channels"].contains("rc_channel_limits_min"))
            {
                values = rc_channels["rc_smart_channels"]["rc_channel_limits_min"];
                auto it = values.begin();
                m_andruav_vehicle_info.rc_channels_min[m_rcmap_channels_info.rcmap_roll] = *it++;
                m_andruav_vehicle_info.rc_channels_min[m_rcmap_channels_info.rcmap_pitch] = *it++;
                m_andruav_vehicle_info.rc_channels_min[m_rcmap_channels_info.rcmap_throttle] = *it++;
                m_andruav_vehicle_info.rc_channels_min[m_rcmap_channels_info.rcmap_yaw] = *it;
            }

            if (rc_channels["rc_smart_channels"].contains("rc_channel_enabled"))
            {
                values = rc_channels["rc_smart_channels"]["rc_channel_enabled"];
                auto it = values.begin();
                m_andruav_vehicle_info.rc_channels_enabled[m_rcmap_channels_info.rcmap_roll] = *it++ == 1 ? true : false;
                m_andruav_vehicle_info.rc_channels_enabled[m_rcmap_channels_info.rcmap_pitch] = *it++ == 1 ? true : false;
                m_andruav_vehicle_info.rc_channels_enabled[m_rcmap_channels_info.rcmap_throttle] = *it++ == 1 ? true : false;
                m_andruav_vehicle_info.rc_channels_enabled[m_rcmap_channels_info.rcmap_yaw] = *it == 1 ? true : false;
            }
        }
        
    }

    if (m_rcmap_channels_info.use_smart_rc == false)
    {
        #ifdef DDEBUG
            std::cout << _LOG_CONSOLE_BOLD_TEXT << "RC Smart Channels are" << _INFO_CONSOLE_BOLD_TEXT << " disabled." << _NORMAL_CONSOLE_TEXT_ << std::endl;
        #endif
    }
            
}

/**
 * @brief Send periodic RCControl values to FCB board to avoid timeout.
 *
 *
 */
void CFCBMain::remoteControlSignal()
{
    if (!m_andruav_vehicle_info.rc_command_active)
        return;
    const u_int64_t now = get_time_usec();

    switch (m_andruav_vehicle_info.rc_sub_action)
    {
    case RC_SUB_ACTION::RC_SUB_ACTION_RELEASED:
    {
        // should ignore these values.
    }
    break;

    case RC_SUB_ACTION::RC_SUB_ACTION_CENTER_CHANNELS:
    {
        // should ignore these values.
        mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels(m_andruav_vehicle_info.rc_channels, RC_CHANNELS_MAX);
    }
    break;

    case RC_SUB_ACTION::RC_SUB_ACTION_FREEZE_CHANNELS:
    {
        // should ignore these values.
        mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels(m_andruav_vehicle_info.rc_channels, RC_CHANNELS_MAX);
    }
    break;

    case RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS:
    {
        // if no RC message has been received since a while change to BreakMode.
        // unless you are in LANDED mode.
        if (now - m_andruav_vehicle_info.rc_command_last_update_time > RCCHANNEL_OVERRIDES_TIMEOUT)
        {
            if (m_andruav_vehicle_info.flying_mode != VEHICLE_MODE_LAND)
            {
                m_andruav_vehicle_info.rc_command_active = false;

                releaseRemoteControl();

                // if RC Timeout then switch to brake or equivelant mode.
                uint32_t ardupilot_mode, ardupilot_custom_mode, ardupilot_custom_sub_mode;
                // TODO: VEHICLE_MODE_BRAKE mode should be generalized to work on different autopilot types.
                CFCBModes::getArduPilotMode(VEHICLE_MODE_BRAKE, m_andruav_vehicle_info.vehicle_type, ardupilot_mode, ardupilot_custom_mode, ardupilot_custom_sub_mode);
                if (ardupilot_mode == E_UNDEFINED_MODE)
                {
                    // TODO: Send Error Message
                    return;
                }

                mavlinksdk::CMavlinkCommand::getInstance().doSetMode(ardupilot_mode, ardupilot_custom_mode, ardupilot_custom_sub_mode);
            }
            return;
        }
        mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels(m_andruav_vehicle_info.rc_channels, RC_CHANNELS_MAX);
    }
    break;

    case RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED:
    {
        if (now - m_andruav_vehicle_info.rc_command_last_update_time > RCCHANNEL_OVERRIDES_TIMEOUT)
        {
            releaseRemoteControl();

            // if RC Timeout then switch to brake or equivelant mode.
            uint32_t ardupilot_mode, ardupilot_custom_mode, ardupilot_custom_sub_mode;
            // TODO: VEHICLE_MODE_BRAKE mode should be generalized to work on different autopilot types.
            CFCBModes::getArduPilotMode(VEHICLE_MODE_BRAKE, m_andruav_vehicle_info.vehicle_type, ardupilot_mode, ardupilot_custom_mode, ardupilot_custom_sub_mode);
            if (ardupilot_mode == E_UNDEFINED_MODE)
            {
                // TODO: Send Error Message
                return;
            }

            mavlinksdk::CMavlinkCommand::getInstance().doSetMode(ardupilot_mode, ardupilot_custom_mode, ardupilot_custom_sub_mode);
            return;
        }
    }
    break;
    }
}

/**
 * @brief A separate thread schedular for calling recurring tasks.
 *
 */
void CFCBMain::loopScheduler()
{
    swarm::CSwarmLeader &fcb_swarm_leader = swarm::CSwarmLeader::getInstance();

    while (!m_exit_thread)
    {
        // timer each 10m sec.
        wait_time_nsec(0, 10000000);

        m_counter++;

        if (m_counter % 10 == 0)
        { // each 100 msec
            fcb_swarm_leader.handleSwarmsAsLeader();
        }

        if (m_counter % 30 == 0)
        { // each 300 msec
            remoteControlSignal();
            m_fcb_facade.sendLocationInfo();
        }

        if (m_counter % 50 == 0)
        {
            m_fcb_facade.sendNavInfo(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS));
            geofence::CGeoFenceManager::getInstance().updateGeoFenceHitStatus();

            // called each second group #1
            m_fcb_facade.sendGPSInfo(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS));
            checkBlockedStatus();

            heartbeatCamera();

            if (m_vehicle.hasLidarAltitude())
            {
                m_fcb_facade.sendDistanceSensorInfo(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), m_vehicle.getLidarAltitude());
            }
        }

        // .................
        if (m_counter % 100 == 0)
        {
            m_fcb_facade.sendWindInfo(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS));
            m_fcb_facade.sendTerrainReport(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS));
            initVehicleChannelLimits(false);
        }

        if (m_counter % 500 == 0)
        { // 5 sec

            // no need to send it faster as it is sent by an event  if there is an important change in the values.
            m_fcb_facade.sendEKFInfo(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS));
            m_fcb_facade.sendVibrationInfo(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS));
            m_fcb_facade.sendPowerInfo(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS));
            const bool fcb_connected = mavlinksdk::CVehicle::getInstance().isFCBConnected();

            if (fcb_connected != m_fcb_connected)
            {
                m_fcb_connected = fcb_connected;
                m_fcb_facade.API_IC_sendID(std::string());
            }
        }

        if (m_counter % 1000 == 0)
        { // 10 sec
            m_fcb_facade.API_IC_sendID(std::string());
            m_fcb_facade.sendDistanceSensorInfo(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS));
            m_fcb_facade.sendMissionCurrent(std::string());

            const mavlink_mission_current_t mission_current = mavlinksdk::CMavlinkWayPointManager::getInstance().getMissionCurrent();
            m_fcb_facade.sendMissionItemSequence(std::to_string(mission_current.seq));
        }

        if (m_counter % 1500 == 0)
        { // 15 sec
        }
    }

    return;
}






/**
 * @brief Message received from FCB
 *
 * @param mavlink_message
 */
void CFCBMain::OnMessageReceived(const mavlink_message_t &mavlink_message)
{

    switch (mavlink_message.msgid)
    {
    case MAVLINK_MSG_ID_HEARTBEAT:
        // this mavlink_message can be from internal components such as camera or ADSB
        // check msg_heartbeat.type  before parsing or rely on the vehicle stored heartbeat message.
        OnHeartBeat();
        break;

    case MAVLINK_MSG_ID_COMMAND_LONG:
        mavlink_command_long_t command_long;
        mavlink_msg_command_long_decode(&mavlink_message, &(command_long));

        OnCommandLong(command_long);
        break;
    }

    // if streaming active check each message to forward.
    if (m_mavlink_optimizer.shouldForwardThisMessage(mavlink_message))
    {
        // UdpProxy
        const u_int64_t now = get_time_usec();
        const u_int64_t last_access_duration = (now - m_last_access_telemetry);
        if (isUdpProxyMavlinkAvailable())
        {
            // stop sending mavlink if no one is sending back. except heartbeat messages.
            if ((last_access_duration < UDP_PROXY_TIMEOUT) || (mavlink_message.msgid == MAVLINK_MSG_ID_HEARTBEAT))
            {
                m_fcb_facade.sendUdpProxyMavlink(mavlink_message, m_udp_proxy.udp_client);
            }
        }
        
    }

    return;
}

void CFCBMain::OnConnected(const bool &connected)
{
    if (m_andruav_vehicle_info.use_fcb != connected)
    {
        m_andruav_vehicle_info.use_fcb = connected;
    }

    return;
}

/**
 * @brief called locally used for ticking.
 *
 */
void CFCBMain::OnHeartBeat()
{
    if (m_andruav_vehicle_info.is_flying)
    {
        m_andruav_vehicle_info.flying_last_start_time = (get_time_usec() - m_last_start_flying);
    }

    return;
}

void CFCBMain::OnCommandLong(const mavlink_command_long_t &command_long)
{
    switch (command_long.command)
    {
    case MAV_CMD_DO_DIGICAM_CONTROL:
        m_fcb_facade.internalCommand_takeImage();
        break;

    default:
        break;
    }
}

/**
 * @brief Called when HearBeat is received for the first time.
 *
 * @param heartbeat
 */
void CFCBMain::OnHeartBeat_First(const mavlink_heartbeat_t &heartbeat)
{
    m_andruav_vehicle_info.vehicle_type = CFCBModes::getAndruavVehicleType(heartbeat.type);
    m_andruav_vehicle_info.autopilot = heartbeat.autopilot;

    m_andruav_vehicle_info.gps_mode = GPS_MODE_FCB;

    m_fcb_facade.API_IC_sendID(std::string());

    // request home location
    mavlinksdk::CMavlinkCommand::getInstance().requestHomeLocation();

    return;
}

void CFCBMain::OnHeartBeat_Resumed(const mavlink_heartbeat_t &heartbeat)
{
    m_andruav_vehicle_info.vehicle_type = CFCBModes::getAndruavVehicleType(heartbeat.type);
    m_andruav_vehicle_info.autopilot = heartbeat.autopilot;

    m_fcb_facade.API_IC_sendID(std::string());

    return;
}

void CFCBMain::OnBoardRestarted()
{
    std::cout << std::endl
              << _ERROR_CONSOLE_BOLD_TEXT_ << "Flight Controller Restarted" << _NORMAL_CONSOLE_TEXT_ << std::endl;

    m_fcb_facade.sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_ERROR, "FCB board has been restarted");

    return;
}

void CFCBMain::OnArmed(const bool &armed, const bool& ready_to_arm)
{
    std::cout << std::endl
              << _SUCCESS_CONSOLE_BOLD_TEXT_ << "OnArmed/ReadyToArm" << _NORMAL_CONSOLE_TEXT_ << std::endl;

    m_andruav_vehicle_info.is_armed = armed;
    m_andruav_vehicle_info.is_ready_to_arm = ready_to_arm;

    m_fcb_facade.API_IC_sendID(std::string());

    return;
}

void CFCBMain::OnFlying(const bool &is_flying)
{
    std::cout << std::endl
              << _SUCCESS_CONSOLE_BOLD_TEXT_ << "OnFlying" << _NORMAL_CONSOLE_TEXT_ << std::endl;

    if (m_andruav_vehicle_info.is_flying != is_flying)
    {
        if (is_flying == true)
        {
            m_last_start_flying = get_time_usec();
            // start capture a flying
            m_andruav_vehicle_info.flying_last_start_time = 0;
        }
        else
        {
            // add duration to total
            // m_andruav_vehicle_info.flying_last_start_time is updated in onheartbeat
            m_andruav_vehicle_info.flying_total_duration += m_andruav_vehicle_info.flying_last_start_time;
            m_last_start_flying = 0;
        }

        m_andruav_vehicle_info.is_flying = is_flying;
        m_fcb_facade.API_IC_sendID(std::string());
    }

    return;
}

void CFCBMain::OnHighLatencyModeChanged(const int &latency_mode)
{
    // TODO: TO BE Done
    if (latency_mode == 0)
    {
        std::cout << std::endl
                  << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Latency Mode: " << _NORMAL_CONSOLE_TEXT_ << "Exit" << std::endl;
    }
    else
    {
        std::cout << std::endl
                  << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Latency Mode: " << _NORMAL_CONSOLE_TEXT_ << "Enter" << std::endl;
    }
    return;
}

void CFCBMain::OnHighLatencyMessageReceived(const int &latency_mode)
{
    // TODO: BE Done
    m_fcb_facade.sendHighLatencyInfo(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS));
    return;
}

void CFCBMain::OnEKFStatusReportChanged(const mavlink_ekf_status_report_t &ekf_status_report)
{
    m_fcb_facade.sendEKFInfo(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS));
    return;
}

void CFCBMain::OnVibrationChanged(const mavlink_vibration_t &vibration)
{
    m_fcb_facade.sendVibrationInfo(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS));
    return;
}

void CFCBMain::OnADSBVechileReceived(const mavlink_adsb_vehicle_t &adsb_vehicle)
{
    m_fcb_facade.sendADSBVehicleInfo(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS));
    return;
}

void CFCBMain::OnDistanceSensorChanged(const mavlink_distance_sensor_t &distance_sensor)
{
    m_fcb_facade.sendDistanceSensorInfo(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), distance_sensor);
    return;
}

void CFCBMain::OnStatusText(const std::uint8_t &severity, const std::string &status)
{
    try
    {
        if (!is_ascii((const signed char *)status.c_str(), status.length()))
        {
            std::cout << std::endl
                      << _ERROR_CONSOLE_BOLD_TEXT_ << " -- OnStatusText -bad format-" << _NORMAL_CONSOLE_TEXT_ << status << std::endl;
            PLOG(plog::error) << " -- OnStatusText received a non UTF-8 message format:" << status;
            return;
        }

        std::cout << std::endl
                  << _SUCCESS_CONSOLE_BOLD_TEXT_ << " -- OnStatusText " << _NORMAL_CONSOLE_TEXT_ << status << std::endl;

        m_fcb_facade.sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_3DR, severity, status);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }

    return;
}

void CFCBMain::OnMissionACK(const int &result, const int &mission_type, const std::string &result_msg)
{
    int sevirity;

    switch (result)
    {
    case MAV_MISSION_ACCEPTED:
    case MAV_MISSION_OPERATION_CANCELLED:
        sevirity = NOTIFICATION_TYPE_INFO;
        break;

    case MAV_MISSION_UNSUPPORTED:
    case MAV_MISSION_UNSUPPORTED_FRAME:
        sevirity = NOTIFICATION_TYPE_WARNING;
        break;

    case MAV_MISSION_ERROR:
    case MAV_MISSION_NO_SPACE:
    case MAV_MISSION_INVALID:
    case MAV_MISSION_INVALID_PARAM1:
    case MAV_MISSION_INVALID_PARAM2:
    case MAV_MISSION_INVALID_PARAM3:
    case MAV_MISSION_INVALID_PARAM4:
    case MAV_MISSION_INVALID_PARAM5_X:
    case MAV_MISSION_INVALID_PARAM6_Y:
    case MAV_MISSION_INVALID_PARAM7:
    case MAV_MISSION_INVALID_SEQUENCE:
    case MAV_MISSION_DENIED:
        sevirity = NOTIFICATION_TYPE_ERROR;
        break;

    default:
        sevirity = NOTIFICATION_TYPE_WARNING;
        break;
    }

    m_fcb_facade.sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_3DR, sevirity, result_msg);
    return;
}

void CFCBMain::OnWaypointReached(const int &seq)
{
    m_andruav_vehicle_info.current_waypoint = seq;
    m_fcb_facade.sendWayPointReached(std::string(), seq);

    return;
}

void CFCBMain::OnWayPointReceived(const mavlink_mission_item_int_t &mission_item_int)
{
    if (mission_item_int.mission_type == MAV_MISSION_TYPE_MISSION)
    {
        mission::CMissionItem *mission_item = mission::CMissionItemBuilder::getClassByMavlinkCMD(mission_item_int);
        mission_item->decodeMavlink(mission_item_int);
        de::fcb::mission::CMissionManager::getInstance().getAndruavMission().mission_items.insert(std::make_pair(mission_item_int.seq, std::unique_ptr<mission::CMissionItem>(mission_item)));
    }

    return;
}

void CFCBMain::OnWayPointsLoadingCompleted()
{
    // ?Please check if we need to notify GCS.
    // notify that mission has been updated
    m_fcb_facade.sendWayPoints(std::string());
}

/**
 * @brief called when ACK receivied while writing messages.
 * if result is MAV_MISSION_RESULT::MAV_MISSION_ACCEPTED then mission has been successfully uploaded
 *
 * @param result
 * @param mission_type
 * @param result_msg
 */
void CFCBMain::OnMissionSaveFinished(const int &result, const int &mission_type, const std::string &result_msg)
{
    if (result == MAV_MISSION_RESULT::MAV_MISSION_ACCEPTED)
    {
        m_fcb_facade.sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_INFO, "mission saved successfully");
        // reloadWayPoints();
    }
    else
    {
        // broadcast error.
        OnMissionACK(result, mission_type, result_msg);
    }
}

void CFCBMain::OnMissionCurrentChanged(const mavlink_mission_current_t &mission_current)
{
    m_fcb_facade.sendMissionCurrent(std::string());

    // IMPORTANT: This is an internal only event and cannot be broadcasted to communication-server.
    // if you want to broadcast ato communication-server you need to fire event from servo channels.
    
    // IMPORTANT: here we sent a local event of every mission item. this mission item can be ANYTHING
    // it is just a SYNC with other modules that has waiting events linked to waypoints mission-items using.
    // so instead of Filtering message I have chosen to sent.

    // IMPORTANT: Ardupilot sends seq of mission items not mission commands.
    de::fcb::mission::CMissionManager::getInstance().handleMissionCurrentCount(mission_current);
    
}

void CFCBMain::OnACK(const int &acknowledged_cmd, const int &result, const std::string &result_msg)
{
    int sevirity;

    switch (result)
    {
    case MAV_RESULT_ACCEPTED:
    case MAV_RESULT_IN_PROGRESS:
        sevirity = NOTIFICATION_TYPE_INFO;
        break;

    case MAV_RESULT_TEMPORARILY_REJECTED:
    case MAV_RESULT_UNSUPPORTED:
        sevirity = NOTIFICATION_TYPE_WARNING;
        break;

    case MAV_RESULT_DENIED:
    case MAV_RESULT_FAILED:
        sevirity = NOTIFICATION_TYPE_ERROR;
        break;

    default:
        sevirity = NOTIFICATION_TYPE_WARNING;
        break;
    }

    if (result != MAV_RESULT_ACCEPTED)
    {
        // dont annoy the user if everything is OK.
        m_fcb_facade.sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_3DR, sevirity, result_msg);
    }

    return;
}

/**
 * Called when ardupilot flying mode changes.
 */
void CFCBMain::OnModeChanges(const uint32_t &custom_mode, const int &firmware_type, const MAV_AUTOPILOT &autopilot)
{
    // vehicle type is determined here
    m_andruav_vehicle_info.vehicle_type = CFCBModes::getAndruavVehicleType(firmware_type);
    m_andruav_vehicle_info.autopilot = autopilot;

    // vehicle mode is mapped here to andruav mode.
    m_andruav_vehicle_info.flying_mode = CFCBModes::getAndruavMode(custom_mode, CFCBModes::getAndruavVehicleType(firmware_type), autopilot);
    adjustRemoteJoystickByMode(m_andruav_vehicle_info.rc_sub_action);
    m_fcb_facade.API_IC_sendID(std::string());

    return;
}

void CFCBMain::OnHomePositionUpdated(const mavlink_home_position_t &home_position)
{
    m_fcb_facade.sendHomeLocation(std::string());

    return;
}



void CFCBMain::OnServoOutputRaw(const mavlink_servo_output_raw_t &servo_output_raw)
{

    m_event_time_divider = m_event_time_divider + 1;
    m_event_time_divider = m_event_time_divider % EVENT_TIME_DIVIDER;
    if (m_event_time_divider == 0)
    {
        de::fcb::mission::CMissionManager::getInstance().readFiredEventFromFCB(servo_output_raw);
    }

    de::fcb::mission::CMissionManager::getInstance().readWaitingEventFromFCB(servo_output_raw);

    return;
}

/**
 * @brief Called when parameter is recieved for first time or its value has changed.
 * @details Forward parameters only when changed = true. As parameters are sent in chunks to save bandwidth.
 * @param param_name parameter name
 * @param param_message parameter mavlink message
 * @param changed true if changed & false if new
 * @param bool load_parameters_1st_iteration
 */
void CFCBMain::OnParamReceived(const std::string &param_name, const mavlink_param_value_t &param_message, const bool &changed, const bool &load_parameters_1st_iteration)
{
#ifdef DEBUG
std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: "
	<< std::string(param_name) << " : " << "count: " << std::to_string(param_message.param_index) << " of " << std::to_string(param_message.param_count)
	<< " type: " << std::to_string(param_message.param_type) << " value: " << std::to_string(param_message.param_value)
	<< " changed: " << std::to_string(changed) <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif

    if (changed && !load_parameters_1st_iteration)
        m_fcb_facade.sendParameterValue(std::string(), param_message);
}

void CFCBMain::OnParamReceivedCompleted()
{
    m_fcb_facade.sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_INFO, std::string("Parameters Loaded Successfully"));

    update_rcmap_info();
}

/**
 * @brief called when websocket connection changed state
 * @details called when connection between communication_pro and andruav server changes.
 *
 * @param status
 */
void CFCBMain::OnConnectionStatusChangedWithAndruavServer(const int status)
{

    switch (status)
    {
    case SOCKET_STATUS_FREASH:
    {
    }
    break;

    case SOCKET_STATUS_CONNECTING:
    {
        PLOG(plog::info) << "Communicator Server Connection Status: SOCKET_STATUS_CONNECTING";
    }
    break;

    case SOCKET_STATUS_REGISTERED:
    {
        PLOG(plog::info) << "Communicator Server Connection Status: SOCKET_STATUS_REGISTERED";

        m_fcb_facade.callModule_reloadSavedTasks(TYPE_AndruavSystem_LoadTasks);

        // open or close -if open- udpProxy once connection with communication server is established.
        m_fcb_facade.requestUdpProxyTelemetry(m_enable_udp_telemetry_in_config, "0.0.0.0", 0, "0.0.0.0", m_udp_telemetry_fixed_port);
        // Json_de json_msg = sendMREMSG(TYPE_AndruavSystem_LoadTasks);
        // const std::string msg = json_msg.dump();
        // CUDPProxy.sendMSG(msg.c_str(), msg.length());
    }
    break;

    case SOCKET_STATUS_UNREGISTERED:
    {
        PLOG(plog::warning) << "Communicator Server Connection Status: SOCKET_STATUS_UNREGISTERED";
    }
    break;

    case SOCKET_STATUS_ERROR:
    {
        PLOG(plog::error) << "Communicator Server Connection Status: SOCKET_STATUS_ERROR";
        alertDroneEngageOffline();
    }
    break;

    case SOCKET_STATUS_DISCONNECTING:
    {
        PLOG(plog::warning) << "Communicator Server Connection Status: SOCKET_STATUS_DISCONNECTING";
    }
    break;

    case SOCKET_STATUS_DISCONNECTED:
    {
        PLOG(plog::warning) << "Communicator Server Connection Status: SOCKET_STATUS_DISCONNECTED";
        alertDroneEngageOffline();
    }
    break;

    default:
    {
        PLOG(plog::warning) << "Communicator Server Connection Status: Unknonwn State " << std::to_string(status);
        alertDroneEngageOffline();
    }
    break;
    }
}

void CFCBMain::alertDroneEngageOffline()
{

    PLOG(plog::error) << "Alert: DroneEngage is Offline";

    return;
}

/**
 * @brief This function get PWM signal from Andruav [0,1000] value and after applying reverse, deadband & limites.
 * @param scaled_channels [0,1000]
 * @param ignode_dead_band
 * @callgraph
 * @return int16_t [1000,2000]
 */
void CFCBMain::calculateChannels(const int16_t scaled_channels[18], const bool ignode_dead_band, int16_t *output)
{
	
    for (int i = 0; i < 18; ++i)
    {
        int scaled_channel = scaled_channels[i];

        // --- Stage 1: Handle Ignored/Disabled Channels ---
        if ((scaled_channel == SKIP_RC_CHANNEL) || (!m_andruav_vehicle_info.rc_channels_enabled[i]))
        {
            // https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
            if (i<8)
            {
                output[i] = 0; // release RC Channel
            }
            else
            {
                output[i] = UINT16_MAX-1; // release RC Channel
            }
            continue;
        }

       
        // --- Stage 2: Convert Range, Apply Deadband, Apply Reverse ---
        // convert range to [-500,500]
        scaled_channel = scaled_channel - 500; 
        // apply deadband
        if ((!ignode_dead_band) && (abs(scaled_channel) < 20))
        {
            // https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
            if (i<8)
            {
                scaled_channel = 0; // release RC Channel
            }
            else
            {
                scaled_channel = UINT16_MAX-1; // release RC Channel
            }
        }

        // Apply Reverse
        if (m_andruav_vehicle_info.rc_channels_reverse[i])
        {
            scaled_channel = -scaled_channel;
        }

        // --- Stage 3: Limit Min Max and Scale ---
        if (scaled_channel <= 0)
        {
            const int min_value = 1500 - m_andruav_vehicle_info.rc_channels_min[i];
            output[i] = int(min_value / 500.0f * scaled_channel);
        }
        else
        {
            const int max_value = m_andruav_vehicle_info.rc_channels_max[i] - 1500;
            output[i] = int(max_value / 500.0f * scaled_channel);
        }

        output[i] += 1500;
    }
}

/**
 * @brief disable rcchennel override and sends zero to all RC_CHANNELS_MAX channels of ardupilot.
 * @callgraph
 */
void CFCBMain::releaseRemoteControl()
{
    if (m_andruav_vehicle_info.rc_sub_action == RC_SUB_ACTION::RC_SUB_ACTION_RELEASED)
        return;

    memset(m_andruav_vehicle_info.rc_channels, 0, RC_CHANNELS_MAX * sizeof(int16_t));
    m_andruav_vehicle_info.rc_sub_action = RC_SUB_ACTION::RC_SUB_ACTION_RELEASED;
    m_andruav_vehicle_info.rc_command_active = false;

    mavlinksdk::CMavlinkCommand::getInstance().releaseRCChannels();

    m_fcb_facade.sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_RCCONTROL, NOTIFICATION_TYPE_WARNING, std::string("RX Released."));
}

/**
 * @brief Set first 4 FOUR channels or user smart Channels based on RCMAP to 1500 and release others.
 *
 */
void CFCBMain::centerRemoteControl()
{
    if (m_andruav_vehicle_info.rc_sub_action == RC_SUB_ACTION::RC_SUB_ACTION_CENTER_CHANNELS)
        return;

    if ((m_rcmap_channels_info.use_smart_rc) && (!m_rcmap_channels_info.is_valid))
    {
        m_fcb_facade.sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_ERROR, std::string("Error: RCMAP Channels not Ready."));
        return;
    }

    memset(m_andruav_vehicle_info.rc_channels, 0, RC_CHANNELS_MAX * sizeof(int16_t));
    if ((m_rcmap_channels_info.use_smart_rc) && (m_rcmap_channels_info.is_valid))
    {
        m_andruav_vehicle_info.rc_channels[m_rcmap_channels_info.rcmap_pitch] = 1500;
        m_andruav_vehicle_info.rc_channels[m_rcmap_channels_info.rcmap_roll] = 1500;
        m_andruav_vehicle_info.rc_channels[m_rcmap_channels_info.rcmap_throttle] = 1500;
        m_andruav_vehicle_info.rc_channels[m_rcmap_channels_info.rcmap_yaw] = 1500;
    }
    else
    {
        memset(m_andruav_vehicle_info.rc_channels, 1500, 4 * sizeof(int16_t));
    }

    m_andruav_vehicle_info.rc_sub_action = RC_SUB_ACTION::RC_SUB_ACTION_CENTER_CHANNELS;
    m_andruav_vehicle_info.rc_command_active = true;
    m_fcb_facade.sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_RCCONTROL, NOTIFICATION_TYPE_WARNING, std::string("RX Centered and Locked"));
}

/**
 * @brief Read the current 8 EIGHT channels of RC and save them.
 * Other channels are released.
 *
 */
void CFCBMain::freezeRemoteControl()
{

    if (m_andruav_vehicle_info.rc_sub_action == RC_SUB_ACTION::RC_SUB_ACTION_FREEZE_CHANNELS)
        return;

    if ((m_rcmap_channels_info.use_smart_rc) && (!m_rcmap_channels_info.is_valid))
    {
        m_fcb_facade.sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_ERROR, std::string("Error: RCMAP Channels not Ready."));
        return;
    }

    const mavlink_rc_channels_t &mavlink_rc_channels = mavlinksdk::CVehicle::getInstance().getRCChannels();

    m_andruav_vehicle_info.rc_channels[0] = mavlink_rc_channels.chan1_raw;
    m_andruav_vehicle_info.rc_channels[1] = mavlink_rc_channels.chan2_raw;
    m_andruav_vehicle_info.rc_channels[2] = mavlink_rc_channels.chan3_raw;
    m_andruav_vehicle_info.rc_channels[3] = mavlink_rc_channels.chan4_raw;
    m_andruav_vehicle_info.rc_channels[4] = mavlink_rc_channels.chan5_raw;
    m_andruav_vehicle_info.rc_channels[5] = mavlink_rc_channels.chan6_raw;
    m_andruav_vehicle_info.rc_channels[6] = mavlink_rc_channels.chan7_raw;
    m_andruav_vehicle_info.rc_channels[7] = mavlink_rc_channels.chan8_raw;
    m_andruav_vehicle_info.rc_channels[8] = mavlink_rc_channels.chan9_raw;
    m_andruav_vehicle_info.rc_channels[9] = mavlink_rc_channels.chan10_raw;
    m_andruav_vehicle_info.rc_channels[10] = mavlink_rc_channels.chan11_raw;
    m_andruav_vehicle_info.rc_channels[11] = mavlink_rc_channels.chan12_raw;
    m_andruav_vehicle_info.rc_channels[12] = mavlink_rc_channels.chan13_raw;
    m_andruav_vehicle_info.rc_channels[13] = mavlink_rc_channels.chan14_raw;
    m_andruav_vehicle_info.rc_channels[14] = mavlink_rc_channels.chan15_raw;
    m_andruav_vehicle_info.rc_channels[15] = mavlink_rc_channels.chan16_raw;
    m_andruav_vehicle_info.rc_channels[16] = mavlink_rc_channels.chan17_raw;
    m_andruav_vehicle_info.rc_channels[17] = mavlink_rc_channels.chan18_raw;

    m_andruav_vehicle_info.rc_sub_action = RC_SUB_ACTION::RC_SUB_ACTION_FREEZE_CHANNELS;
    m_andruav_vehicle_info.rc_command_active = true;

    m_fcb_facade.sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_RCCONTROL, NOTIFICATION_TYPE_WARNING, std::string("RX Freeze Mode"));
}

/**
 * @brief Override channels. Overridden channels are determined dynamically.
 *
 */
void CFCBMain::enableRemoteControl()
{
    if ((m_rcmap_channels_info.use_smart_rc) && (!m_rcmap_channels_info.is_valid))
    {
        m_fcb_facade.sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_ERROR, std::string("Error: RCMAP Channels not Ready."));
        return;
    }

    m_andruav_vehicle_info.rc_command_last_update_time = get_time_usec();
    m_andruav_vehicle_info.rc_command_active = false; // remote control data will enable it
    m_andruav_vehicle_info.rc_sub_action = RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS;
    memset(m_andruav_vehicle_info.rc_channels, 0, RC_CHANNELS_MAX * sizeof(int16_t)); // zero values. RC channels will be sent later

    m_fcb_facade.sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_RCCONTROL, NOTIFICATION_TYPE_WARNING, std::string("RX Joystick Mode"));
}

void CFCBMain::enableRemoteControlGuided()
{

    if ((m_rcmap_channels_info.use_smart_rc) && (!m_rcmap_channels_info.is_valid))
    {
        m_fcb_facade.sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_ERROR, std::string("Error: RCMAP Channels not Ready."));
        return;
    }

    m_andruav_vehicle_info.rc_command_last_update_time = get_time_usec();
    m_andruav_vehicle_info.rc_command_active = false; // remote control data will enable it

    // release channels
    memset(m_andruav_vehicle_info.rc_channels, 0, RC_CHANNELS_MAX * sizeof(int16_t));
    mavlinksdk::CMavlinkCommand::getInstance().releaseRCChannels();

    m_andruav_vehicle_info.rc_sub_action = RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED;

    m_fcb_facade.sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_RCCONTROL, NOTIFICATION_TYPE_WARNING, std::string("RX Joystick Guided Mode"));

    return;
}

/**
 * Change channel values based on rc_sub_action and flying modes.
 */
void CFCBMain::adjustRemoteJoystickByMode(RC_SUB_ACTION rc_sub_action)
{
    switch (rc_sub_action)
    {
    case RC_SUB_ACTION::RC_SUB_ACTION_RELEASED:
    {
        releaseRemoteControl();
    }
    break;

    case RC_SUB_ACTION::RC_SUB_ACTION_CENTER_CHANNELS:
    {
        centerRemoteControl();
    }
    break;

    case RC_SUB_ACTION::RC_SUB_ACTION_FREEZE_CHANNELS:
    {
        freezeRemoteControl();
    }
    break;

    case RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS:
    case RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED:
    {
        if (m_andruav_vehicle_info.flying_mode == VEHICLE_MODE_GUIDED)
        {
            // remote control in guided mode uses ctrlGuidedVelocityInLocalFrame for control.
            enableRemoteControlGuided();
        }
        else
        {
            // for non guided mode uses sendRCChannels()
            enableRemoteControl();
        }
    }
    break;
    }
}

/**
 * @brief called by external gcs to apply channel values.
 *
 * @param rc_channels values from [-500,500] ... -1 meanse release channel.
 */
void CFCBMain::updateRemoteControlChannels(const int16_t rc_channels[RC_CHANNELS_MAX])
{

    switch (m_andruav_vehicle_info.rc_sub_action)
    {
    case RC_SUB_ACTION::RC_SUB_ACTION_RELEASED:
    {
        // should ignore these values.
    }
    break;

    case RC_SUB_ACTION::RC_SUB_ACTION_CENTER_CHANNELS:
    {
        // should ignore these values...already predefined
    }
    break;

    case RC_SUB_ACTION::RC_SUB_ACTION_FREEZE_CHANNELS:
    {
        // should ignore these values...already predefined
    }
    break;

    case RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS:
    {
        // In this mode send values via sendRCChannels

        m_andruav_vehicle_info.rc_command_last_update_time = get_time_usec();
        m_andruav_vehicle_info.rc_command_active = true;

        int16_t rc_chammels_pwm[RC_CHANNELS_MAX] = {0};

        calculateChannels(rc_channels, true, rc_chammels_pwm);

        if (m_rcmap_channels_info.use_smart_rc == true)
        {

            // memset(m_andruav_vehicle_info.rc_channels, 0, 18 * sizeof(int16_t));
            m_andruav_vehicle_info.rc_channels[m_rcmap_channels_info.rcmap_pitch] = rc_chammels_pwm[m_rcmap_channels_info.rcmap_pitch];
            m_andruav_vehicle_info.rc_channels[m_rcmap_channels_info.rcmap_roll] = rc_chammels_pwm[m_rcmap_channels_info.rcmap_roll];
            m_andruav_vehicle_info.rc_channels[m_rcmap_channels_info.rcmap_throttle] = rc_chammels_pwm[m_rcmap_channels_info.rcmap_throttle];
            m_andruav_vehicle_info.rc_channels[m_rcmap_channels_info.rcmap_yaw] = rc_chammels_pwm[m_rcmap_channels_info.rcmap_yaw];
        }
        else
        {
            // just copy channels.
            for (int i = 0; i < RC_CHANNELS_MAX; ++i)
            {
                m_andruav_vehicle_info.rc_channels[i] = rc_chammels_pwm[i];
            }
        }

        mavlinksdk::CMavlinkCommand::getInstance().sendRCChannels(m_andruav_vehicle_info.rc_channels, RC_CHANNELS_MAX);
    }
    break;

    case RC_SUB_ACTION::RC_SUB_ACTION_JOYSTICK_CHANNELS_GUIDED:
    {
        // In this mode send values via ctrlGuidedVelocityInLocalFrame

        m_andruav_vehicle_info.rc_command_last_update_time = get_time_usec();
        m_andruav_vehicle_info.rc_command_active = true;

        int16_t rc_chammels_pwm[RC_CHANNELS_MAX] = {0};

        calculateChannels(rc_channels, true, rc_chammels_pwm);
        if ((m_rcmap_channels_info.use_smart_rc) && (m_rcmap_channels_info.is_valid))
        {
            mavlinksdk::CMavlinkCommand::getInstance().ctrlGuidedVelocityInLocalFrame(
                !m_andruav_vehicle_info.rc_channels_enabled[m_rcmap_channels_info.rcmap_pitch] ? 0 : (1500 - rc_chammels_pwm[m_rcmap_channels_info.rcmap_pitch]) / 100.0f,
                !m_andruav_vehicle_info.rc_channels_enabled[m_rcmap_channels_info.rcmap_roll] ? 0 : (rc_chammels_pwm[m_rcmap_channels_info.rcmap_roll] - 1500) / 100.0f,
                !m_andruav_vehicle_info.rc_channels_enabled[m_rcmap_channels_info.rcmap_throttle] ? 0 : (1500 - rc_chammels_pwm[m_rcmap_channels_info.rcmap_throttle]) / 100.0f,
                !m_andruav_vehicle_info.rc_channels_enabled[m_rcmap_channels_info.rcmap_yaw] ? 0 : (rc_chammels_pwm[m_rcmap_channels_info.rcmap_yaw] - 1500) / 1000.0f,
                MAV_FRAME_BODY_OFFSET_NED);
        }
        else
        {
            mavlinksdk::CMavlinkCommand::getInstance().ctrlGuidedVelocityInLocalFrame(
                (1500 - rc_chammels_pwm[1]) / 100.0f,
                (1500 - rc_chammels_pwm[0]) / 100.0f,
                (1500 - rc_chammels_pwm[2]) / 100.0f,
                (1500 - rc_chammels_pwm[3]) / 100.0f,
                MAV_FRAME_BODY_OFFSET_NED);
        }
    }
    break;
    }
}




void CFCBMain::setStreamingLevel(const int &streaming_level)
{
    if (streaming_level != -1)
    {
        // update streaming level globally.
        m_mavlink_optimizer.setOptimizationLevel(streaming_level);
        m_mavlink_optimizer.reset_timestamps();
    }
}


/**
 * @brief updates RCMAP_CHANNELS_MAP_INFO_STRUCT with valid values.
 * @details should be called after parmaters are all uploaded and available.
 *
 * @callgraph
 */
void CFCBMain::update_rcmap_info()
{
    mavlinksdk::CMavlinkParameterManager &parameter_manager = mavlinksdk::CMavlinkParameterManager::getInstance();
    if (!parameter_manager.isParametersListAvailable())
    {
        return;
    }

    switch (m_andruav_vehicle_info.autopilot)
    {
    case MAV_AUTOPILOT::MAV_AUTOPILOT_ARDUPILOTMEGA:
    case MAV_AUTOPILOT::MAV_AUTOPILOT_GENERIC:
    {
        m_rcmap_channels_info.is_valid = false;

        mavlink_param_value_t rcmap = parameter_manager.getParameterByName("RCMAP_PITCH");
        m_rcmap_channels_info.rcmap_pitch = (uint16_t)rcmap.param_value - 1;

        rcmap = parameter_manager.getParameterByName("RCMAP_ROLL");
        m_rcmap_channels_info.rcmap_roll = (uint16_t)rcmap.param_value - 1;

        rcmap = parameter_manager.getParameterByName("RCMAP_THROTTLE");
        m_rcmap_channels_info.rcmap_throttle = (uint16_t)rcmap.param_value - 1;

        rcmap = parameter_manager.getParameterByName("RCMAP_YAW");
        m_rcmap_channels_info.rcmap_yaw = (uint16_t)rcmap.param_value - 1;

        m_rcmap_channels_info.is_valid = true;
    }
    break;

    case MAV_AUTOPILOT::MAV_AUTOPILOT_PX4:
    {
        m_rcmap_channels_info.is_valid = false;

        mavlink_param_value_t rcmap = parameter_manager.getParameterByName("RC_MAP_PITCH");
        m_rcmap_channels_info.rcmap_pitch = (uint16_t)rcmap.param_value == 0 ? 2 : (uint16_t)rcmap.param_value - 1;

        rcmap = parameter_manager.getParameterByName("RC_MAP_ROLL");
        m_rcmap_channels_info.rcmap_roll = (uint16_t)rcmap.param_value == 0 ? 0 : (uint16_t)rcmap.param_value - 1;

        rcmap = parameter_manager.getParameterByName("RC_MAP_THROTTLE");
        m_rcmap_channels_info.rcmap_throttle = (uint16_t)rcmap.param_value == 0 ? 1 : (uint16_t)rcmap.param_value - 1;

        rcmap = parameter_manager.getParameterByName("RC_MAP_YAW");
        m_rcmap_channels_info.rcmap_yaw = (uint16_t)rcmap.param_value == 0 ? 3 : (uint16_t)rcmap.param_value - 1;

        m_rcmap_channels_info.is_valid = true;
    }
    default:
        break;
    };
}

void CFCBMain::checkBlockedStatus()
{
    // check if blocking channel is defined.
    if (m_andruav_vehicle_info.rc_block_channel == -1)
        return;

    const mavlink_rc_channels_t &mavlink_rc_channels = mavlinksdk::CVehicle::getInstance().getRCChannels();

    uint16_t *channel = (uint16_t *)&mavlink_rc_channels;

    // move channel pointer to defined channel
    channel += (1 + m_andruav_vehicle_info.rc_block_channel); //+1 skip time stamp.

    const bool is_gcs_blocked = (*channel > BLOCKING_CHANNEL_HIGH_ACTIVE_PWM);

    if (m_jsonConfig.contains("read_only_mode"))
    {
        if (m_jsonConfig["read_only_mode"] == true)
        { // if read only true the it is always read_only_mode = true
            m_andruav_vehicle_info.is_gcs_blocked = true;
            return;
        }
    }

    if (m_andruav_vehicle_info.is_gcs_blocked != is_gcs_blocked)
    {
        m_andruav_vehicle_info.is_gcs_blocked = is_gcs_blocked;

        std::string msg = "GCS is in Control.";
        if (is_gcs_blocked)
        {
            releaseRemoteControl();
            // TODO: release from swarm.
            msg = "GCS is Blocked.";
        }

        m_fcb_facade.API_IC_sendID(std::string());

        m_fcb_facade.sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_WARNING, msg);
    }
}

void CFCBMain::heartbeatCamera()
{
    /**
     * This is used to tell Ardupilot that there is a camera in the system acccessible from this mavlink module.
     * so when clicking on Camera from Mission Planner the command generated by Ardupilot it sent to mavlink module
     * that forwards it to DE_Camera.
     * see: internalCommand_takeImage
     */
    mavlinksdk::CMavlinkCommand::getInstance().sendHeartBeatOfComponent(MAV_COMP_ID_CAMERA);
}

void CFCBMain::updateUDPProxy(const bool &enabled, const std::string &udp_ip1, const int &udp_port1, const std::string &udp_ip2, const int &udp_port2)
{
    if (enabled == true)
    {
        if (m_udp_proxy.udp_client.isStarted())
        {
            m_udp_proxy.udp_client.stop();
        }
        m_udp_proxy.udp_client.setCallback(this);
        m_udp_proxy.udp_client.init(udp_ip1.c_str(), udp_port1, "0.0.0.0", 0);
        m_udp_proxy.udp_client.start();
    }
    else
    {
        m_udp_proxy.udp_client.stop();
    }

    m_udp_proxy.enabled = enabled;
    m_udp_proxy.paused = false;
    m_udp_proxy.udp_ip1 = udp_ip1;
    m_udp_proxy.udp_port1 = udp_port1;
    m_udp_proxy.udp_ip2 = udp_ip2;
    m_udp_proxy.udp_port2 = udp_port2;

    sendUdpProxyStatus(std::string());
    return;
}

void CFCBMain::sendUdpProxyStatus(const std::string &target_party_id)
{

    m_fcb_facade.sendUdpProxyStatus(std::string(), m_udp_proxy.enabled, m_udp_proxy.paused, m_udp_proxy.udp_ip2, m_udp_proxy.udp_port2, m_mavlink_optimizer.getOptimizationLevel());
    ;
}

bool CFCBMain::isUdpProxyMavlinkAvailable() const
{
    return (m_udp_proxy.enabled && (!m_udp_proxy.paused));
}

void CFCBMain::pauseUDPProxy(const bool paused)
{
    m_udp_proxy.paused = paused;
}

void CFCBMain::requestChangeUDPProxyClientPort(const uint16_t udp_proxy_fixed_port)
{
    if (m_jsonConfig.contains("udp_proxy_enabled"))
    {
        /**
         * @brief In some cases, the m_udp_proxy.enabled variable may be set to false when attempting to open a port that is already in use. When this happens,
         * the Communication Server will also set the enabled flag to false. However, if UDP proxy usage is allowed,
         * you can try to force enable the proxy or change the port to resolve the issue.
         * It's important to note that forcing the opening of the UDP proxy is not permitted unless it's allowed in the configuration file, due to security reasons.
         */
        m_udp_proxy.enabled = m_jsonConfig["udp_proxy_enabled"].get<bool>();
    }
    m_fcb_facade.requestUdpProxyTelemetry(m_udp_proxy.enabled, m_udp_proxy.udp_ip1, m_udp_proxy.udp_port1,
                                          m_udp_proxy.udp_ip2, udp_proxy_fixed_port);
}
