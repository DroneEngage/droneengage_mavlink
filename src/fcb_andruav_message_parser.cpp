#include <iostream>
#include <fstream>
#include "defines.hpp"
#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"
#include "./de_common/helpers/helpers.hpp"
#include "./de_common/helpers/colors.hpp"
#include "./de_common/de_databus/messages.hpp"
#include "./de_common/de_databus/configFile.hpp"
#include "./de_common/de_databus/localConfigFile.hpp"
#include "fcb_modes.hpp"
#include "fcb_andruav_message_parser.hpp"
#include "./geofence/fcb_geo_fence_base.hpp"
#include "./geofence/fcb_geo_fence_manager.hpp"

using namespace de::fcb;

void CFCBAndruavMessageParser::parseRemoteExecute(Json_de &andruav_message)
{
    const Json_de cmd = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_CMD];

    if (!validateField(cmd, "C", Json_de::value_t::number_unsigned))
        return;

    uint32_t permission = 0;
    if (validateField(andruav_message, ANDRUAV_PROTOCOL_MESSAGE_PERMISSION, Json_de::value_t::number_unsigned))
    {
        permission = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_PERMISSION].get<int>();
    }

    bool m_is_system = false;
    if ((validateField(andruav_message, ANDRUAV_PROTOCOL_SENDER, Json_de::value_t::string)) && (andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>().compare(ANDRUAV_PROTOCOL_SENDER_COMM_SERVER) == 0))
    {
        m_is_system = true;
    }

    if ((validateField(andruav_message, INTERMODULE_ROUTING_TYPE, Json_de::value_t::string)) && (andruav_message[INTERMODULE_ROUTING_TYPE].get<std::string>().compare(CMD_TYPE_INTERMODULE) == 0))
    {
        m_is_inter_module = true;
    }

    const int remoteCommand = cmd["C"].get<int>();
    std::cout << "cmd: " << remoteCommand << std::endl;
    switch (remoteCommand)
    {
    case TYPE_AndruavMessage_ServoChannel:
        if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked)
            break;
        CFCBFacade::getInstance().sendServoReadings(andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>());
        break;

    case RemoteCommand_REQUEST_PARA_LIST:
        if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked)
            break;
        CFCBFacade::getInstance().sendParameterList(andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>());
        break;

    case TYPE_AndruavMessage_HomeLocation:
        if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked)
            break;
        CFCBFacade::getInstance().sendHomeLocation(andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>());
        break;

    case RemoteCommand_MISSION_COUNT:
    case RemoteCommand_MISSION_CURRENT:
        CFCBFacade::getInstance().sendMissionCurrent(std::string());
        break;

    case RemoteCommand_RELOAD_WAY_POINTS_FROM_FCB:
        if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked)
            break;
        m_mission_manager.reloadWayPoints();
        break;

    case RemoteCommand_CLEAR_WAY_POINTS:
        if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked)
            break;
        if ((!m_is_system) && ((permission & PERMISSION_ALLOW_GCS_WP_CONTROL) != PERMISSION_ALLOW_GCS_WP_CONTROL))
        {
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "CLEAR_WAY_POINTS_FROM_FCB: " << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
            break;
        }
        m_mission_manager.clearWayPoints();
        break;

    case RemoteCommand_CLEAR_FENCE_DATA:
    {
        if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked)
            break;
        if ((!m_is_system) && ((permission & PERMISSION_ALLOW_GCS_WP_CONTROL) != PERMISSION_ALLOW_GCS_WP_CONTROL))
        {
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "CLEAR_FENCE_DATA: " << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
            break;
        }
        std::string fence_name;
        if (cmd.contains("fn") == true)
        {
            fence_name = cmd["fn"].get<std::string>();
        }
        geofence::CGeoFenceManager::getInstance().clearGeoFences(fence_name);
    }
    break;

    case RemoteCommand_SET_START_MISSION_ITEM:
        if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked)
            break;
        if (!validateField(cmd, "n", Json_de::value_t::number_unsigned))
            return;
        mavlinksdk::CMavlinkCommand::getInstance().setCurrentMission(cmd["n"].get<int>());
        break;

    case RemoteCommand_CONNECT_FCB:
        break;

    case TYPE_AndruavSystem_LoadTasks:
        CFCBFacade::getInstance().callModule_reloadSavedTasks(TYPE_AndruavSystem_LoadTasks);
        break;

    case TYPE_AndruavMessage_GeoFence:
    {
        std::string fence_name;
        geofence::GEO_FENCE_STRUCT *geo_fence_struct = nullptr;
        if (cmd.contains("fn") == true)
        {
            fence_name = cmd["fn"].get<std::string>();
            geo_fence_struct = geofence::CGeoFenceManager::getInstance().getFenceByName(fence_name);
        }
        if (geo_fence_struct != nullptr)
        {
            CFCBFacade::getInstance().sendGeoFenceToTarget(andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>(), geo_fence_struct);
        }
        else
        {
            std::vector<geofence::GEO_FENCE_STRUCT *> geo_fence_struct = geofence::CGeoFenceManager::getInstance().getFencesOfParty(m_fcbMain.getAndruavVehicleInfo().party_id);
            const std::size_t size = geo_fence_struct.size();
            for (int i = 0; i < size; i++)
            {
                CFCBFacade::getInstance().sendGeoFenceToTarget(andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>(), geo_fence_struct[i]);
            }
        }
    }
    break;

    case TYPE_AndruavMessage_GeoFenceAttachStatus:
    {
        std::string fence_name;
        if (cmd.contains("fn") == true)
        {
            fence_name = cmd["fn"].get<std::string>();
        }
        CFCBFacade::getInstance().sendGeoFenceAttachedStatusToTarget(andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>(), fence_name);
    }
    break;

    case TYPE_AndruavMessage_UDPProxy_Info:
    {
        if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked)
            break;
        if ((!m_is_system) && ((permission & PERMISSION_ALLOW_GCS_FULL_CONTROL) != PERMISSION_ALLOW_GCS_FULL_CONTROL))
        {
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "UDPProxy_Info: " << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
            break;
        }
        m_fcbMain.sendUdpProxyStatus(andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>());
    }
    break;

    case RemoteCommand_TELEMETRYCTRL:
    {
        if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked)
            break;
        if ((!m_is_inter_module) && (!m_is_system) && ((permission & PERMISSION_ALLOW_GCS_FULL_CONTROL) != PERMISSION_ALLOW_GCS_FULL_CONTROL))
        {
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "TELEMETRYCTRL: " << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
            break;
        }
        if (!validateField(cmd, "Act", Json_de::value_t::number_unsigned))
            return;
        const int request_type = cmd["Act"].get<int>();
        int streaming_level = -1;
        switch (request_type)
        {
        case CONST_TELEMETRY_ADJUST_RATE:
            if (validateField(cmd, "LVL", Json_de::value_t::number_unsigned))
            {
                streaming_level = cmd["LVL"].get<int>();
            }
            break;
        case CONST_TELEMETRY_REQUEST_PAUSE:
            m_fcbMain.pauseUDPProxy(true);
            break;
        case CONST_TELEMETRY_REQUEST_RESUME:
            m_fcbMain.pauseUDPProxy(false);
            break;
        default:
            return;
        }
        m_fcbMain.setStreamingLevel(streaming_level);
        if (m_is_inter_module == true)
        {
            m_fcbMain.sendUdpProxyStatus(std::string(""));
        }
    }
    break;

    case RemoteCommand_SET_UDPPROXY_CLIENT_PORT:
    {
        if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked)
            break;
        if ((!m_is_system) && ((permission & PERMISSION_ALLOW_GCS_FULL_CONTROL) != PERMISSION_ALLOW_GCS_FULL_CONTROL))
        {
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "SET_UDPPROXY_CLIENT_PORT: " << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
            break;
        }
        if (cmd.contains("P") == true)
        {
            uint32_t udp_proxy_fixed_port = cmd["P"].get<int>();
            if (udp_proxy_fixed_port >= 0xffff)
                break;
            de::CLocalConfigFile &cLocalConfigFile = de::CLocalConfigFile::getInstance();
            cLocalConfigFile.addNumericField("udp_proxy_fixed_port", udp_proxy_fixed_port);
            cLocalConfigFile.apply();
            std::cout << std::endl
                      << _ERROR_CONSOLE_BOLD_TEXT_ << "Change UDPProxy Port to " << _INFO_CONSOLE_TEXT << udp_proxy_fixed_port << std::endl;
            PLOG(plog::warning) << "SET_UDPPROXY_CLIENT_PORT: Change UDPProxy Port to:" << udp_proxy_fixed_port;
            m_fcb_facade.sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_WARNING, std::string("UDPProxy port update initiated."));
            m_fcbMain.requestChangeUDPProxyClientPort(udp_proxy_fixed_port);
        }
    }
    }
}

void CFCBAndruavMessageParser::parseCommand(Json_de &andruav_message, const char *full_message, const int &full_message_length, int messageType, uint32_t permission)
{
    const Json_de cmd = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_CMD];

    switch (messageType)
    {
    case TYPE_AndruavMessage_CONFIG_ACTION:
    {
        std::string module_key = "";
        if (!validateField(cmd, "a", Json_de::value_t::number_unsigned))
            return;
        if (validateField(cmd, "b", Json_de::value_t::string))
        {
            module_key = de::comm::CModule::getInstance().getModuleKey();
            if (module_key != cmd["b"].get<std::string>())
            {
                return;
            }
        }
        int action = cmd["a"].get<int>();
        switch (action)
        {
        case CONFIG_ACTION_Restart:
            exit(0);
            break;
        case CONFIG_ACTION_APPLY_CONFIG:
        {
            Json_de config = cmd["c"];
            std::cout << config << std::endl;
            de::CConfigFile& cConfigFile = de::CConfigFile::getInstance();
            cConfigFile.updateJSON(config.dump());
        }
        break;
        case CONFIG_REQUEST_FETCH_CONFIG_TEMPLATE:
        {
            if (!andruav_message.contains(ANDRUAV_PROTOCOL_SENDER)) return;
            std::string sender = andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>();
#ifdef DEBUG
            std::cout << std::endl << _INFO_CONSOLE_TEXT << "CONFIG_REQUEST_FETCH_CONFIG_TEMPLATE" << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif
            std::ifstream file("template.json");
            if (!file.is_open()) {
                std::cout << std::endl << _ERROR_CONSOLE_BOLD_TEXT_ << "cannot read template.json" << _NORMAL_CONSOLE_TEXT_ << std::endl;
                CFCBFacade::getInstance().sendErrorMessage(std::string(), 0, ERROR_3DR, NOTIFICATION_TYPE_ERROR, "cannot read template.json");
                Json_de empty_file_content_json = {};
                CFCBFacade::getInstance().API_sendConfigTemplate(sender, module_key, empty_file_content_json, true);
                return;
            }
            std::string file_content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
            file.close();
            Json_de file_content_json = Json_de::parse(file_content);
            CFCBFacade::getInstance().API_sendConfigTemplate(sender, module_key, file_content_json, true);
        }
        break;
        case CONFIG_REQUEST_FETCH_CONFIG:
            break;
        default:
            break;
        }
    }
    break;

    case TYPE_AndruavMessage_Arm:
    {
        if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked)
            break;
        if ((!m_is_system) && ((permission & PERMISSION_ALLOW_GCS_MODES_CONTROL) != PERMISSION_ALLOW_GCS_MODES_CONTROL))
        {
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "Arm: " << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
            break;
        }
        if (!validateField(cmd, "A", Json_de::value_t::boolean))
            return;
        bool arm = cmd["A"].get<bool>();
        bool force = false;
        if (cmd.contains("D") == true)
        {
            if (!validateField(cmd, "D", Json_de::value_t::boolean))
                return;
            force = cmd["D"].get<bool>();
        }
        mavlinksdk::CMavlinkCommand::getInstance().doArmDisarm(arm, force);
    }
    break;

    case TYPE_AndruavMessage_FlightControl:
    {
        if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked)
            break;
        if ((!m_is_system) && ((permission & PERMISSION_ALLOW_GCS_MODES_CONTROL) != PERMISSION_ALLOW_GCS_MODES_CONTROL))
        {
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "FlightControl: " << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
            break;
        }
        if (!validateField(cmd, "F", Json_de::value_t::number_unsigned))
            return;
        const int andruav_mode = cmd["F"].get<int>();
        uint32_t ardupilot_mode, ardupilot_custom_mode, ardupilot_custom_sub_mode;
        CFCBModes::getArduPilotMode(andruav_mode, m_fcbMain.getAndruavVehicleInfo().vehicle_type, ardupilot_mode, ardupilot_custom_mode, ardupilot_custom_sub_mode);
        if (ardupilot_mode == E_UNDEFINED_MODE)
        {
            return;
        }
        mavlinksdk::CMavlinkCommand::getInstance().doSetMode(ardupilot_mode, ardupilot_custom_mode, ardupilot_custom_sub_mode);
    }
    break;

    case TYPE_AndruavMessage_ChangeAltitude:
    {
        if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked)
            break;
        if ((!m_is_system) && ((permission & PERMISSION_ALLOW_GCS_MODES_CONTROL) != PERMISSION_ALLOW_GCS_MODES_CONTROL))
        {
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "ChangeAltitude: " << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
            break;
        }
        if ((!validateField(cmd, "a", Json_de::value_t::number_float)) && (!validateField(cmd, "a", Json_de::value_t::number_unsigned)))
            return;
        double altitude = cmd["a"].get<double>();
        if (mavlinksdk::CVehicle::getInstance().isFlying() == true)
        {
            mavlinksdk::CMavlinkCommand::getInstance().changeAltitude(altitude);
        }
        else
        {
            mavlinksdk::CMavlinkCommand::getInstance().takeOff(altitude);
        }
    }
    break;

    case TYPE_AndruavMessage_Land:
    {
        if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked)
            break;
        if ((!m_is_system) && ((permission & PERMISSION_ALLOW_GCS_MODES_CONTROL) != PERMISSION_ALLOW_GCS_MODES_CONTROL))
        {
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "Land: " << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
            break;
        }
        uint32_t ardupilot_mode, ardupilot_custom_mode, ardupilot_custom_sub_mode;
        CFCBModes::getArduPilotMode(VEHICLE_MODE_LAND, m_fcbMain.getAndruavVehicleInfo().vehicle_type, ardupilot_mode, ardupilot_custom_mode, ardupilot_custom_sub_mode);
        if (ardupilot_mode == E_UNDEFINED_MODE)
        {
            return;
        }
        mavlinksdk::CMavlinkCommand::getInstance().doSetMode(ardupilot_mode, ardupilot_custom_mode, ardupilot_custom_sub_mode);
    }
    break;

    case TYPE_AndruavMessage_GuidedPoint:
    {
        if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked)
            break;
        if ((!m_is_system) && ((permission & PERMISSION_ALLOW_GCS_MODES_CONTROL) != PERMISSION_ALLOW_GCS_MODES_CONTROL))
        {
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "GuidedPoint: " << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
            break;
        }
        if (!validateField(cmd, "a", Json_de::value_t::number_float))
            return;
        if (!validateField(cmd, "g", Json_de::value_t::number_float))
            return;
        double latitude = cmd["a"].get<double>();
        double longitude = cmd["g"].get<double>();
        double altitude = mavlinksdk::CVehicle::getInstance().getMsgGlobalPositionInt().relative_alt;
        if (cmd.contains("l") == true)
        {
            if ((!validateField(cmd, "l", Json_de::value_t::number_float)) && (!validateField(cmd, "l", Json_de::value_t::number_unsigned)))
                return;
            double alt = cmd["l"].get<double>();
            if (alt != 0.0)
            {
                altitude = alt * 1000.0;
            }
        }
        mavlinksdk::CMavlinkCommand::getInstance().gotoGuidedPoint(latitude, longitude, altitude / 1000.0);
        CFCBFacade::getInstance().sendFCBTargetLocation(andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>(), latitude, longitude, altitude, DESTINATION_GUIDED_POINT);
    }
    break;

    case TYPE_AndruavMessage_SET_HOME_LOCATION:
    {
        if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked)
            break;
        if ((!m_is_system) && ((permission & PERMISSION_ALLOW_GCS_WP_CONTROL) != PERMISSION_ALLOW_GCS_WP_CONTROL))
        {
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "SET_HOME_LOCATION: " << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
            break;
        }
        if (!validateField(cmd, "T", Json_de::value_t::number_float))
            return;
        if (!validateField(cmd, "O", Json_de::value_t::number_float))
            return;
        double latitude = cmd["T"].get<double>();
        double longitude = cmd["O"].get<double>();
        double altitude = cmd["A"].get<double>();
        if (altitude == 0)
        {
            altitude = mavlinksdk::CVehicle::getInstance().getMsgHomePosition().altitude / 1000.0f;
        }
        mavlinksdk::CMavlinkCommand::getInstance().setHome(0, latitude, longitude, altitude);
    }
    break;

    case TYPE_AndruavMessage_UDPProxy_Info:
    {
        if (!validateField(cmd, "a", Json_de::value_t::string))
            return;
        if (!validateField(cmd, "p", Json_de::value_t::number_integer))
            return;
        if (!validateField(cmd, "o", Json_de::value_t::number_integer))
            return;
        if (!validateField(cmd, "en", Json_de::value_t::boolean))
            return;
    }
    break;

    case TYPE_AndruavSystem_UDPProxy:
    {
        if (!m_is_system)
        {
            std::cout << _INFO_CONSOLE_BOLD_TEXT << "UdpProxy: " << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
            return;
        }
        if (!validateField(cmd, "socket1", Json_de::value_t::object))
            return;
        if (!validateField(cmd, "socket2", Json_de::value_t::object))
            return;
        const Json_de &socket1 = cmd["socket1"];
        const Json_de &socket2 = cmd["socket2"];
        if (!validateField(socket1, "address", Json_de::value_t::string))
            return;
        if (!validateField(socket1, "port", Json_de::value_t::number_unsigned))
            return;
        if (!validateField(socket2, "address", Json_de::value_t::string))
            return;
        if (!validateField(socket2, "port", Json_de::value_t::number_unsigned))
            return;
        if (!validateField(cmd, "en", Json_de::value_t::boolean))
            return;
        const bool enable = cmd["en"].get<bool>();
        const std::string my_address = socket1["address"].get<std::string>();
        const int my_port = socket1["port"].get<int>();
        const std::string others_address = socket2["address"].get<std::string>();
        const int others_port = socket2["port"].get<int>();
        m_fcbMain.updateUDPProxy(enable, my_address, my_port, others_address, others_port);
    }
    break;
    }
}