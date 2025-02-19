#include <iostream>
#include "defines.hpp"
#include <plog/Log.h> 
#include "plog/Initializers/RollingFileInitializer.h"

#include "./helpers/helpers.hpp"
#include "./helpers/colors.hpp"
#include "./de_common/messages.hpp"
#include "./de_common/localConfigFile.hpp"
#include "fcb_modes.hpp"
#include "./swarm/fcb_swarm_follower.hpp"
#include "fcb_andruav_message_parser.hpp"
#include "./geofence/fcb_geo_fence_base.hpp"
#include "./geofence/fcb_geo_fence_manager.hpp"

using namespace de::fcb;



/// @brief Parses & executes messages received from uavos_comm"
/// @param parsed JSON message received from uavos_comm 
/// @param full_message 
/// @param full_message_length 
void CFCBAndruavMessageParser::parseMessage (Json_de &andruav_message, const char * full_message, const int & full_message_length)
{
    const int messageType = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_TYPE].get<int>();
    bool is_binary = !(full_message[full_message_length-1]==125 || (full_message[full_message_length-2]==125));   // "}".charCodeAt(0)  IS TEXT / BINARY Msg  
    

    uint32_t permission = 0;
    if (validateField(andruav_message, ANDRUAV_PROTOCOL_MESSAGE_PERMISSION, Json_de::value_t::number_unsigned))
    {
        permission =  andruav_message[ANDRUAV_PROTOCOL_MESSAGE_PERMISSION].get<int>();
    }

    bool is_system = false;
    if ((validateField(andruav_message, ANDRUAV_PROTOCOL_SENDER, Json_de::value_t::string)) && (andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>().compare(SPECIAL_NAME_SYS_NAME)==0))
    {   // permission is not needed if this command sender is the communication server not a remote GCS or Unit.
        is_system = true;
    }

    bool is_inter_module = false;
    if ((validateField(andruav_message, INTERMODULE_ROUTING_TYPE, Json_de::value_t::string)) && (andruav_message[INTERMODULE_ROUTING_TYPE].get<std::string>().compare(CMD_TYPE_INTERMODULE)==0))
    {   // permission is not needed if this command sender is the communication server not a remote GCS or Unit.
        is_inter_module = true;
    }
    
    if (messageType == TYPE_AndruavMessage_RemoteExecute)
    {
        parseRemoteExecute(andruav_message);

        return ;
    }

    else
    {
        Json_de cmd = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_CMD];
        
        switch (messageType)
        {

            case TYPE_AndruavMessage_Arm:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                if ((!is_system) && ((permission & PERMISSION_ALLOW_GCS_MODES_CONTROL) != PERMISSION_ALLOW_GCS_MODES_CONTROL)) 
                {
                    std::cout << _INFO_CONSOLE_BOLD_TEXT << "Arm: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    break;
                }
                
                // A  : bool arm/disarm
                // [D]: bool force 
                if (!validateField(cmd, "A", Json_de::value_t::boolean)) return ;
                
                bool arm = cmd["A"].get<bool>();
                bool force = false;
                if (cmd.contains("D") == true)
                {
                    if (!validateField(cmd, "D", Json_de::value_t::boolean)) return ;
                
                    force = cmd["D"].get<bool>();
                }
                mavlinksdk::CMavlinkCommand::getInstance().doArmDisarm(arm,force);
            }
            break;

            case TYPE_AndruavMessage_FlightControl:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                if ((!is_system) && ((permission & PERMISSION_ALLOW_GCS_MODES_CONTROL) != PERMISSION_ALLOW_GCS_MODES_CONTROL)) 
                {
                    std::cout << _INFO_CONSOLE_BOLD_TEXT << "FlightControl: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    break;
                }
                
                // F  : andruav unit mode
                // [g]: longitude
                // [a]: latitude
                // [r]: radius 

                if (!validateField(cmd, "F", Json_de::value_t::number_unsigned)) return ;
                
                const int andruav_mode = cmd["F"].get<int>();
                //double langitude = 0.0f;
                // TODO: Missing circle and guided go to here mode.
                uint32_t ardupilot_mode, ardupilot_custom_mode, ardupilot_custom_sub_mode;
                CFCBModes::getArduPilotMode(andruav_mode, m_fcbMain.getAndruavVehicleInfo().vehicle_type, ardupilot_mode , ardupilot_custom_mode, ardupilot_custom_sub_mode);
                if (ardupilot_mode == E_UNDEFINED_MODE)
                {   
                    //TODO: Send Error Message
                    return ;
                }

                mavlinksdk::CMavlinkCommand::getInstance().doSetMode(ardupilot_mode, ardupilot_custom_mode, ardupilot_custom_sub_mode);
            }
            break;

            case TYPE_AndruavMessage_ChangeAltitude:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                if ((!is_system) && ((permission & PERMISSION_ALLOW_GCS_MODES_CONTROL) != PERMISSION_ALLOW_GCS_MODES_CONTROL)) 
                {
                    std::cout << _INFO_CONSOLE_BOLD_TEXT << "ChangeAltitude: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    break;
                }
                
                // a : altitude 
                if ((!validateField(cmd, "a", Json_de::value_t::number_float)) 
                && (!validateField(cmd, "a", Json_de::value_t::number_unsigned)))
                    return ;
                
                double altitude = cmd["a"].get<double>();
                
                if (mavlinksdk::CVehicle::getInstance().isFlying()== true)
                {
                    mavlinksdk::CMavlinkCommand::getInstance().changeAltitude (altitude );
                }
                else
                {
                    mavlinksdk::CMavlinkCommand::getInstance().takeOff (altitude );
                }
                
            }
            break;

            case TYPE_AndruavMessage_Land:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                if ((!is_system) && ((permission & PERMISSION_ALLOW_GCS_MODES_CONTROL) != PERMISSION_ALLOW_GCS_MODES_CONTROL)) 
                {
                    std::cout << _INFO_CONSOLE_BOLD_TEXT << "Land: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    break;
                }

                //TODO: could be included in change mode.

                uint32_t ardupilot_mode, ardupilot_custom_mode, ardupilot_custom_sub_mode;
                CFCBModes::getArduPilotMode(VEHICLE_MODE_LAND, m_fcbMain.getAndruavVehicleInfo().vehicle_type, ardupilot_mode , ardupilot_custom_mode, ardupilot_custom_sub_mode);
                if (ardupilot_mode == E_UNDEFINED_MODE)
                {   
                    //TODO: Send Error Message
                    return ;
                }
                
                mavlinksdk::CMavlinkCommand::getInstance().doSetMode(ardupilot_mode, ardupilot_custom_mode, ardupilot_custom_sub_mode);
            }
            break;

            case TYPE_AndruavMessage_GuidedPoint:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                if ((!is_system) && ((permission & PERMISSION_ALLOW_GCS_MODES_CONTROL) != PERMISSION_ALLOW_GCS_MODES_CONTROL)) 
                {
                    std::cout << _INFO_CONSOLE_BOLD_TEXT << "GuidedPoint: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    break;
                }
                
                //  a : latitude
                //  g : longitude
                // [l]: altitude
                // other fields are obscelete.
                
                if (!validateField(cmd, "a", Json_de::value_t::number_float)) return ;
                if (!validateField(cmd, "g", Json_de::value_t::number_float)) return ;
                
                double latitude  = cmd["a"].get<double>();
                double longitude = cmd["g"].get<double>();
                double altitude  = mavlinksdk::CVehicle::getInstance().getMsgGlobalPositionInt().relative_alt;
                if (cmd.contains("l") == true)
                {
                    if ((!validateField(cmd, "l", Json_de::value_t::number_float)) 
                    && (!validateField(cmd, "l", Json_de::value_t::number_unsigned)))
                        return ;
                
                    double alt = cmd["l"].get<double>();
                    if (alt != 0.0)
                    {
                        altitude = alt * 1000.0;
                    }
                }
                

                mavlinksdk::CMavlinkCommand::getInstance().gotoGuidedPoint (latitude, longitude, altitude / 1000.0);
                CFCBFacade::getInstance().sendFCBTargetLocation (andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>(), latitude, longitude, altitude, DESTINATION_GUIDED_POINT);
            }   
            break;

            case TYPE_AndruavMessage_SET_HOME_LOCATION:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                if ((!is_system) && ((permission & PERMISSION_ALLOW_GCS_WP_CONTROL) != PERMISSION_ALLOW_GCS_WP_CONTROL)) 
                {
                    std::cout << _INFO_CONSOLE_BOLD_TEXT << "SET_HOME_LOCATION: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    break;
                }
                
                // T: latitude
                // O: longitude
                // A: altitude
                if (!validateField(cmd, "T", Json_de::value_t::number_float)) return ;
                if (!validateField(cmd, "O", Json_de::value_t::number_float)) return ;
                //if (!validateField(message, "A", Json_de::value_t::number_float)) return ;

                double latitude  = cmd["T"].get<double>();
                double longitude = cmd["O"].get<double>();
                double altitude  = cmd["A"].get<double>();
                if (altitude == 0)
                {
                    altitude  = mavlinksdk::CVehicle::getInstance().getMsgHomePosition().altitude / 1000.0f; // convert to meter
                }
                
                mavlinksdk::CMavlinkCommand::getInstance().setHome(0, latitude, longitude, altitude);
            }
            break;

            case TYPE_AndruavMessage_DoYAW:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                if ((!is_system) && ((permission & PERMISSION_ALLOW_GCS_MODES_CONTROL) != PERMISSION_ALLOW_GCS_MODES_CONTROL)) 
                {
                    std::cout << _INFO_CONSOLE_BOLD_TEXT << "DoYAW: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    break;
                }
                
                // A : target_angle
                // R : turn_rate
                // C : is_clock_wise
                // L : is_relative
                if ((!validateField(cmd, "A", Json_de::value_t::number_unsigned)) 
                && (!validateField(cmd, "A", Json_de::value_t::number_integer)))
                    return ;
                if ((!validateField(cmd, "R", Json_de::value_t::number_unsigned)) 
                && (!validateField(cmd, "R", Json_de::value_t::number_integer)))
                    return ;
                if (!validateField(cmd, "C", Json_de::value_t::boolean)) return ;
                if (!validateField(cmd, "L", Json_de::value_t::boolean)) return ;

                double target_angle = cmd["A"].get<double>();
                double turn_rate    = cmd["R"].get<double>();
                bool is_clock_wise  = cmd["C"].get<bool>();
                bool is_relative    = cmd["L"].get<bool>();
                
                mavlinksdk::CMavlinkCommand::getInstance().setYawCondition( target_angle, turn_rate, is_clock_wise, is_relative);
            }
            break;

            case TYPE_AndruavMessage_ChangeSpeed:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                if ((!is_system) && ((permission & PERMISSION_ALLOW_GCS_MODES_CONTROL) != PERMISSION_ALLOW_GCS_MODES_CONTROL)) 
                {
                    std::cout << _INFO_CONSOLE_BOLD_TEXT << "ChangeSpeed: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    break;
                }
                
                // a : speed
                // b : is_ground_speed
                // c : throttle
                // d : is_relative
                if ((!validateField(cmd, "a", Json_de::value_t::number_float)) 
                && (!validateField(cmd, "a", Json_de::value_t::number_unsigned)))
                    return ;
                if (!validateField(cmd, "b", Json_de::value_t::boolean)) return ;
                if ((!validateField(cmd, "c", Json_de::value_t::number_float)) 
                && (!validateField(cmd, "c", Json_de::value_t::number_integer)))
                    return ;
                if (!validateField(cmd, "d", Json_de::value_t::boolean)) return ;

                double speed            = cmd["a"].get<double>();
                bool is_ground_speed    = cmd["b"].get<bool>();
                double throttle         = cmd["c"].get<double>();
                bool is_relative        = cmd["d"].get<bool>();
                
                const int speed_type = is_ground_speed?1:0;

                mavlinksdk::CMavlinkCommand::getInstance().setNavigationSpeed(speed_type, speed, throttle, is_relative);
            }
            break;

            case TYPE_AndruavMessage_UploadWayPoints:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;
                
                if ((!is_system) && ((permission & PERMISSION_ALLOW_GCS_WP_CONTROL) != PERMISSION_ALLOW_GCS_WP_CONTROL)) \
                {
                    std::cout << _INFO_CONSOLE_BOLD_TEXT << "UploadWayPoints: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    break;
                }
                
                // UDP communication for de_databus should allow any size of waypoints.
                
                
                /*
                    a : std::string serialized mission file
                */
                if (!validateField(cmd, "a", Json_de::value_t::string)) 
                {
                    CFCBFacade::getInstance().sendErrorMessage(std::string(), 0, ERROR_3DR, NOTIFICATION_TYPE_ERROR, "Bad input plan file");

                    break ;
                }

                std::string plan_text = cmd ["a"];
                geofence::CGeoFenceManager::getInstance().uploadFencesIntoSystem(plan_text);
                m_mission_manager.uploadMissionIntoSystem(plan_text);
                
                
                
            }
            break;

            case TYPE_AndruavMessage_Upload_DE_Mission:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;
                
                if ((!is_system) && ((permission & PERMISSION_ALLOW_GCS_WP_CONTROL) != PERMISSION_ALLOW_GCS_WP_CONTROL)) \
                {
                    std::cout << _INFO_CONSOLE_BOLD_TEXT << "UploadWayPoints: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    break;
                }
                
                // UDP communication for de_databus should allow any size of waypoints.
                
                
                /*
                    a : std::string serialized mission file
                */
                if (!validateField(cmd, "j", Json_de::value_t::object)) 
                {
                    CFCBFacade::getInstance().sendErrorMessage(std::string(), 0, ERROR_3DR, NOTIFICATION_TYPE_ERROR, "Bad input plan file");

                    break ;
                }

                if (validateField(cmd, "e", Json_de::value_t::boolean))
                {
                    geofence::CGeoFenceManager::getInstance().clearGeoFences("");
                }

                const Json_de plan_object = cmd ["j"];

                //CMissionManager::getInstance().
                geofence::CGeoFenceManager::getInstance().uploadFencesIntoSystem(plan_object);
                
                m_mission_manager.uploadMissionIntoSystem2(plan_object);
                
            }
            break;

            case TYPE_AndruavMessage_GeoFence:
            {
                // receive GeoFence info from drones.
                std::cout << "I AM HERE" << std::endl;
            }
            break;

            case TYPE_AndruavMessage_ExternalGeoFence:
            {
                // I am a drone and I need to update my fence info.
                // This could be the _SYS_ in this case it is my own fences.

                //if (!validateField(message, "t", Json_de::value_t::number_unsigned)) return ;
                std::unique_ptr<geofence::CGeoFenceBase> fence = geofence::CGeoFenceFactory::getInstance().getGeoFenceObject(cmd);
                geofence::CGeoFenceManager::getInstance().addFence(std::move(fence));
                geofence::CGeoFenceManager::getInstance().attachToGeoFence(m_fcbMain.getAndruavVehicleInfo().party_id, cmd["n"].get<std::string>());
                // geofence::GEO_FENCE_STRUCT * fence_struct = geofence::CGeoFenceManager::getInstance().getFenceByName(message["n"].get<std::string>());
                // std::vector<geofence::GEO_FENCE_STRUCT*> fence_struct_vect = geofence::CGeoFenceManager::getInstance().getFencesOfParty (m_fcbMain.getAndruavVehicleInfo().party_id);
                // if (fence_struct!=NULL)
                // {
                //     geofence::CGeoFenceManager::getInstance().detachFromGeoFence (m_fcbMain.getAndruavVehicleInfo().party_id, fence_struct);   
                // }
                // fence_struct_vect = geofence::CGeoFenceManager::getInstance().getFencesOfParty (m_fcbMain.getAndruavVehicleInfo().party_id);
                
            }
            break;

            case TYPE_AndruavMessage_ServoChannel:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;
                
                if ((!is_system) && ((permission & PERMISSION_ALLOW_GCS_MODES_SERVOS) != PERMISSION_ALLOW_GCS_MODES_SERVOS)) 
                {
                    std::cout << _INFO_CONSOLE_BOLD_TEXT << "ServoChannel: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    break;
                }
                
                // n : servo_channel
                // v : servo_value

                if (!validateField(cmd, "n",Json_de::value_t::number_unsigned)) return ;
                if (!validateField(cmd, "v",Json_de::value_t::number_unsigned)) return ;

                int servo_channel = cmd["n"].get<int>();
                int servo_value = cmd["v"].get<int>();
                if (servo_value == 9999) servo_value = 2000;
                if (servo_value == 0) servo_value = 1000;
                mavlinksdk::CMavlinkCommand::getInstance().setServo (servo_channel, servo_value);
            
            }
            break;

            case TYPE_AndruavMessage_RemoteControlSettings:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                if ((!is_system) && ((permission & PERMISSION_ALLOW_GCS_MODES_CONTROL) != PERMISSION_ALLOW_GCS_MODES_CONTROL)) 
                {
                    std::cout << _INFO_CONSOLE_BOLD_TEXT << "RemoteControlSettings: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    break;
                }
                
                // b: remote control setting
                
                if (!validateField(cmd, "b",Json_de::value_t::number_unsigned)) return ;
                
                int rc_sub_action = cmd["b"].get<int>();
                m_fcbMain.adjustRemoteJoystickByMode((RC_SUB_ACTION)rc_sub_action);

                // update status
                CFCBFacade::getInstance().API_IC_sendID(std::string(ANDRUAV_PROTOCOL_SENDER_ALL));
            }
            break;

            case TYPE_AndruavMessage_Sync_EventFire:
            {   // This can be an event from remote unit or from anothe module.

                // if (validateField(cmd, "a", Json_de::value_t::number_unsigned)) 
                // {
                //     // add mission-id event to event list.
                //     // dependent paused mission should be fired as a response.
                //     m_mission_manager.mavlinkMissionItemStartedEvent(cmd["a"].get<int>());
                // }

                if (validateField(cmd, "d",Json_de::value_t::string)) 
                {
                    // string droneengage event format.
                    m_mission_manager.deEventFiredExternally(cmd["d"].get<std::string>());
                }
            }
            break;

            case TYPE_AndruavMessage_RemoteControl2:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                if ((!is_system) && ((permission & PERMISSION_ALLOW_GCS_MODES_CONTROL) != PERMISSION_ALLOW_GCS_MODES_CONTROL)) 
                {
                    std::cout << _INFO_CONSOLE_BOLD_TEXT << "RemoteControl2: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    break;
                }
                
                // value: [0,1000] IMPORTANT: -999 means channel release
                // 'R': Rudder
                // 'T': Throttle
                // 'A': Aileron
                // 'E': Elevator
                // ['w']: Aux-1 optional 
                // ['x']: Aux-2 optional
                // ['y']: Aux-3 optional
                // ['z']: Aux-4 optional
                if (!validateField(cmd, "R",Json_de::value_t::number_unsigned)) return ;
                if (!validateField(cmd, "T",Json_de::value_t::number_unsigned)) return ;
                if (!validateField(cmd, "A",Json_de::value_t::number_unsigned)) return ;
                if (!validateField(cmd, "E",Json_de::value_t::number_unsigned)) return ;
                
                int16_t rc_channels[18] = {-999};
                const RCMAP_CHANNELS_MAP_INFO_STRUCT rc_map = m_fcbMain.getRCChannelsMapInfo();
                if ((rc_map.use_smart_rc) && (rc_map.is_valid))
                {
                    rc_channels[rc_map.rcmap_yaw]       = 1000 - cmd["R"].get<int>(); // to be aligned with default settings of Ardu
                    rc_channels[rc_map.rcmap_throttle]  = 1000 - cmd["T"].get<int>();
                    rc_channels[rc_map.rcmap_roll]      = 1000 - cmd["A"].get<int>(); // to be aligned with default settings of Ardu
                    rc_channels[rc_map.rcmap_pitch]     = 1000 - cmd["E"].get<int>();
                
                }
                else
                {
                    rc_channels[3] = 1000 - cmd["R"].get<int>(); // to be aligned with default settings of Ardu
                    rc_channels[2] = 1000 - cmd["T"].get<int>();
                    rc_channels[0] = 1000 - cmd["A"].get<int>(); // to be aligned with default settings of Ardu
                    rc_channels[1] = 1000 - cmd["E"].get<int>();
                    rc_channels[4] = validateField(cmd, "w",Json_de::value_t::number_integer)?cmd["w"].get<int>():-999;
                    rc_channels[5] = validateField(cmd, "x",Json_de::value_t::number_integer)?cmd["x"].get<int>():-999;
                    rc_channels[6] = validateField(cmd, "y",Json_de::value_t::number_integer)?cmd["y"].get<int>():-999;
                    rc_channels[7] = validateField(cmd, "z",Json_de::value_t::number_integer)?cmd["z"].get<int>():-999;
                }
                
                

                m_fcbMain.updateRemoteControlChannels(rc_channels);

            }
            
            // This message was using for Telemetry. It has been replaced with UDPProxy
            // However the message itself is still valid and can be used to send mavlink 
            // Again there is a message called TYPE_AndruavMessage_MAVLINK for exchanging mavlink.
            // So this is considered a redundant none used function.
            case TYPE_AndruavMessage_LightTelemetry:
            {   
                
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                if ((!is_system) && ((permission & PERMISSION_ALLOW_GCS_FULL_CONTROL) != PERMISSION_ALLOW_GCS_FULL_CONTROL)) 
                {
                    std::cout << _INFO_CONSOLE_BOLD_TEXT << "LightTelemetry: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    break;
                }
                
                // this is a binary message
                // search for char '0' and then binary message is the next byte after it.
                if (!is_binary)
                {
                    // corrupted message.
                    break;
                }
                const char * binary_message = (char *)(memchr (full_message, 0x0, full_message_length));
                int binary_length = binary_message==0?0:(full_message_length - (binary_message - full_message +1) );

                mavlink_status_t status;
	            mavlink_message_t mavlink_message;
                for (int i=0; i<binary_length; ++ i)
                {
		            uint8_t msgReceived = mavlink_parse_char(MAVLINK_CHANNEL_INTERMODULE, binary_message[i+ 1], &mavlink_message, &status);
                    if (msgReceived!=0)
                    {
                        //TODO: you can add logging or warning
                        //mavlinksdk::CMavlinkCommand::getInstance().sendNative(mavlink_message);
                        
                    }
                }
            }
            break;

            case TYPE_AndruavMessage_MAVLINK:
            {   
                /**
                * @brief  DANGER: Sending messages that is outpout of other agent's fcb can lead to 
                * unpredicted issues such as the MAVLINK_MSG_ID_MISSION_COUNT BUG.
                * DE-SERVER reject agents from broadcasting messages to agents unless agent id
                * is mention explicitly or target is _AGN_.
                *
                * GCS uses this command to send mavlink command such as parameters.
                * This command can be replaced by TYPE_AndruavMessage_LightTelemetry
                **/

                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                // this is a binary message
                // search for char '0' and then binary message is the next byte after it.
                const char * binary_message = (char *)(memchr (full_message, 0x0, full_message_length));
                int binary_length = binary_message==0?0:(full_message_length - (binary_message - full_message +1) );

                mavlink_status_t status;
	            mavlink_message_t mavlink_message;
                for (int i=0; i<binary_length; ++ i)
                {
		            uint8_t msgReceived = mavlink_parse_char(MAVLINK_CHANNEL_INTERMODULE, binary_message[i+ 1], &mavlink_message, &status);
                    if (msgReceived!=0)
                    {
                        #ifdef DEBUG        
                            std::cout << _INFO_CONSOLE_TEXT << "RX MAVLINK: " << std::to_string(mavlink_message.msgid) << _NORMAL_CONSOLE_TEXT_ << std::endl;
                        #endif
                        switch (mavlink_message.msgid)
                        {
                            case MAVLINK_MSG_ID_MISSION_COUNT:
                            case MAVLINK_MSG_ID_MISSION_ITEM_INT:
                            case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
                            case MAVLINK_MSG_ID_STATUSTEXT:
                            case MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED:
                                // internal processing not to be forwarded to FCB.
                                break;
                            
                            default:
                                if ((!is_system) && ((permission & PERMISSION_ALLOW_GCS_FULL_CONTROL) != PERMISSION_ALLOW_GCS_FULL_CONTROL)) 
                                {
                                    std::cout << _INFO_CONSOLE_BOLD_TEXT << "MAVLINK: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied " << _INFO_CONSOLE_TEXT << " - " << _ERROR_CONSOLE_BOLD_TEXT_<< permission << _NORMAL_CONSOLE_TEXT_ << std::endl;
                                    return;
                                }
                                mavlinksdk::CMavlinkCommand::getInstance().sendNative(mavlink_message);
                                break;
                        } 
                    }
                }
            }

            case TYPE_AndruavMessage_SWARM_MAVLINK:
            {
                /**
                 * @brief This message is mainly the TYPE_AndruavMessage_MAVLINK message.
                 * It is just used as a separate channel to avoid routing messages wrongly between units.
                 * Sender of this messages should have permission PERMISSION_ALLOW_SWARM 
                 */
                
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;


                if ((!is_system) && ((permission & PERMISSION_ALLOW_SWARM) != PERMISSION_ALLOW_SWARM)) 
                {
                    std::cout << _INFO_CONSOLE_BOLD_TEXT << "SWARM MAVLINK: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    break;
                }

                const std::string leader_sender = andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>();
                de::fcb::swarm::CSwarmFollower& swarm_follower = de::fcb::swarm::CSwarmFollower::getInstance();
                swarm_follower.handle_leader_traffic(leader_sender, full_message, full_message_length);
            }
            break;

            /**
             * @brief P2P SECTION
             * This should be a generic P2P as much as possible.
             * For now we will focus on ESP32-Mesh
             * 
             * We have two messages here:
             * 
             * 1- TYPE_AndruavMessage_P2P_ACTION:
             *  This message contains requests to connect/disconnect/search ...etc.
             * 
             * 
             * 
             * 2- TYPE_AndruavMessage_P2P_STATUS:
             *  This message contains information about P2P status.
             * 
             * 
            */

           case TYPE_AndruavMessage_P2P_ACTION:
           {
                /**
                 * @brief This is a general purpose message 
                 * 
                 * a: P2P_ACTION_ ... commands
                 * b: mac address
                 * p: wifi_password
                 * c: channel
                 * 
                */

                if (!cmd.contains("a") || !cmd["a"].is_number_integer()) return ;
                if (!cmd.contains("b") || !cmd["b"].is_string()) return ;
                

           }
           break;

           case TYPE_AndruavMessage_P2P_STATUS:
           {
                /**
                 * @brief This is a general purpose message 
                 * 
                 * a: P2P_STATUS_ ... commands
                 * b: mac address
                 * p: wifi_password
                 * c: channel
                 * 
                */

                if (!cmd.contains("a") || !cmd["a"].is_number_integer()) return ;
                if (!cmd.contains("b") || !cmd["b"].is_string()) return ;
                

           }
           break;

            /**
             * @brief SWARM SECTION
             * We have three messages here:
             * 
             * 1- TYPE_AndruavMessage_MAKE_SWARM:
             *  sent from gcs to a drone to promote/demote a drone to leader
             * 
             * 2- TYPE_AndruavMessage_FollowHim_Request:
             *  this is a message sent to a drone to make it follow a leader.
             *  IMPORTANT concept here is that a leader can follow a leader.
             * 
             * 3- TYPE_AndruavMessage_UpdateSwarm:
             * follower drone sends this message to a leader to request adding 
             * it as a follower.
             * 
             */
            case TYPE_AndruavMessage_MAKE_SWARM:
            {   
                /**
                * @brief This message is received by Leader Units to activate or deactivate swarm formation.
                * Leader needs to inform followers if it is not a leader anymore.
                **/

                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                // Become a leader drone.
                // a: formationID
                // b: partyID
                // if (b:partyID) is not me then treat it as an announcement.

                if (!cmd.contains("a") || !cmd["a"].is_number_integer()) return ;
                if (!cmd.contains("b") || !cmd["b"].is_string()) return ;
                
                if (m_fcbMain.getAndruavVehicleInfo().party_id.compare( cmd["b"].get<std::string>())!=0)
                {
                    // this is an announcement.
                    // other drone should handle this message.
                    // if this is my leader drone and it is being released then 
                    // I can release my self or wait for it to ell me to do so -which makes more sense-.
                    return ;
                }

                const int formation = cmd["a"].get<int>();

                m_fcb_swarm_manager.makeSwarm((de::fcb::swarm::ANDRUAV_SWARM_FORMATION)formation);
            }
            break;

            case TYPE_AndruavMessage_FollowHim_Request:
            {
                /**
                 * @brief tell a drone that another drone is in its team -a slave-.
                 * @details 
                 * This message can be sent from GCS or another Drone either a leader or not.
                 * This message requests from the receiver "Drone" to send @ref TYPE_AndruavMessage_UpdateSwarm
                 * to a third drone that is a LEADER requesting to join its swarm.
                 * The receiver can refuse to send @ref TYPE_AndruavMessage_UpdateSwarm
                 * and the third drone can also refuse the request to be followed by the receiver.
                 * 
                 * @note receiver should not assume it is a follower. It only should forward this request to the leader.
                 */

                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                // a: follower index -1 means any available location.   
                // b: leader partyID 
                // c: slave partyID
                // d: formation_id
                // f: SWARM_FOLLOW/SWARM_UNFOLLOW
                // if (c:partyID) is not me then treat it as an announcement.
                
                //if (!validateField(message, "f",Json_de::value_t::number_integer)) return ;
                if (!cmd.contains("f") || !cmd["f"].is_number_integer()) return ;
                if (!cmd.contains("c") || !cmd["c"].is_string()) return ;
                

                if (m_fcbMain.getAndruavVehicleInfo().party_id.compare(cmd["c"].get<std::string>())!=0)
                {
                    std::cout << "FollowHim_Request announcement by: " << cmd["c"].get<std::string>() << std::endl;

                    // this is an announcement.
                    // other drone should handle this message.
                    return ;
                }

                
                const int swar_action = cmd["f"].get<int>();
                if (swar_action==SWARM_UNFOLLOW)
                { 
                    // [b] can be null
                    std::string leader = m_fcb_swarm_manager.getLeader();
                    if (cmd.contains("b") && cmd["b"].is_string()) 
                    {
                        leader = cmd["b"].get<std::string>();
                    }
                    m_fcb_swarm_manager.unFollowLeader(leader, andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>());
                    break;
                }
                else if (swar_action==SWARM_FOLLOW)
                {   // follow leader in [b]
                    if (!cmd.contains("a") || !cmd["a"].is_number_integer()) return ;
                    if (!cmd.contains("b") || !cmd["b"].is_string()) return ;
                    
                    
                    const int follower_index = cmd["a"].get<int>();
                    const std::string leader_party_id = cmd["b"].get<std::string>();
                    swarm::ANDRUAV_SWARM_FORMATION follower_formation = swarm::ANDRUAV_SWARM_FORMATION::FORMATION_NO_SWARM;
                    if (cmd.contains("d") && cmd["d"].is_number_integer()) 
                    {
                        follower_formation = (swarm::ANDRUAV_SWARM_FORMATION) cmd["d"].get<int>();
                    }
                    
                    std::string sender = "";
                    if (!is_inter_module)
                    {
                        sender = andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>();
                    }
                    else
                    {
                        sender = de::comm::CModule::getInstance().getPartyId();
                    }
                    
                    m_fcb_swarm_manager.followLeader(leader_party_id, follower_index, follower_formation, sender);
                    break;
                }
            }
            break;

            case TYPE_AndruavMessage_UpdateSwarm:
            {   /**
                * @brief This message should be processed by leader to add or remove
                * followers. This message is sent from a follower.
                * Leader will send TYPE_AndruavMessage_FollowHim_Request to the follower in order to aknowledge
                * adding it to the swarm and updating it with formation and its index.
                * TYPE_AndruavMessage_FollowHim_Request messages also is sent to release a follower but with different parameters [f] = SWARM_UNFOLLOW
                */

                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                /*
                    a: action [SWARM_UPDATED, SWARM_DELETE]
                    b: follower index [mandatory with SWARM_UPDATED]
                    c: leader id - if this is not me then consider it a notification.
                    d: slave party id 
                */

                //if (!validateField(message, "d",Json_de::value_t::string)) return ;
                if (!cmd.contains("a") || !cmd["a"].is_number_integer()) return ;
                if (!cmd.contains("c") || !cmd["c"].is_string()) return ;
                if (!cmd.contains("d") || !cmd["d"].is_string()) return ;
                

                if (m_fcbMain.getAndruavVehicleInfo().party_id.compare(cmd["c"].get<std::string>())!=0)
                {
                    // this is an announcement.
                    // other drone should handle this message.
                    return ;
                }

                const int action = cmd["a"].get<int>();
                const std::string follower_party_id = cmd["d"].get<std::string>();
                    
                if (action == SWARM_UPDATED)
                {   
                    // add or modify swarm member
                    const int follower_index = cmd["b"].get<int>();
                    m_fcb_swarm_manager.addFollower(follower_party_id, follower_index);

                   return ;     
                }

                if (action == SWARM_DELETE)
                {
                    // remove a swarm member
                    m_fcb_swarm_manager.releaseSingleFollower(follower_party_id);

                    return ;     
                }
            }
            break;

            //////////////////////SWARM-END

            case TYPE_AndruavMessage_UDPProxy_Info:
            {
                /*
                    message received from another drone to communicate to it directly.
                */
                if (!validateField(cmd, "a",Json_de::value_t::string)) return ;
                if (!validateField(cmd, "p",Json_de::value_t::number_integer)) return ;
                if (!validateField(cmd, "o",Json_de::value_t::number_integer)) return ;
                if (!validateField(cmd, "en",Json_de::value_t::boolean)) return ;
                //TODO: COMPLETE
                
            }
            break;

            case TYPE_AndruavSystem_UDPProxy:
            {   
                /*
                    Received from communication_server that created a udp socket me.
                    In future version this can be received from a udp Server.

                    socket1: a udp socket that can be used.
                    socket2: a udp socket that can be used.
                    any socket can be used by this drone and the other by other parties.
                    the drone can determine this when sending TYPE_AndruavMessage_UDPProxy_Info to other parties.
                    normally socket 1 is drone's socket and socket 2 for external parties.
                */
                if (!is_system)
                {
                    std::cout << _INFO_CONSOLE_BOLD_TEXT << "UdpProxy: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    return;
                }

                if (!validateField(cmd, "socket1",Json_de::value_t::object)) return ;
                if (!validateField(cmd, "socket2",Json_de::value_t::object)) return ;

                const Json_de&  socket1 = cmd["socket1"];
                const Json_de&  socket2 = cmd["socket2"];
                
                if (!validateField(socket1, "address",Json_de::value_t::string)) return ;
                if (!validateField(socket1, "port",Json_de::value_t::number_unsigned)) return ;
                
                if (!validateField(socket2, "address",Json_de::value_t::string)) return ;
                if (!validateField(socket2, "port",Json_de::value_t::number_unsigned)) return ;

                if (!validateField(cmd, "en",Json_de::value_t::boolean)) return ;
                
                /*
                 * my address & other adress are arbitrary.
                 * you can switch between them
                 */
                const bool enable = cmd["en"].get<bool>();
                const std::string my_address = socket1["address"].get<std::string>();
                const int my_port = socket1["port"].get<int>();
                const std::string others_address = socket2["address"].get<std::string>();
                const int  others_port = socket2["port"].get<int>();
                
                m_fcbMain.updateUDPProxy (enable, my_address, my_port,
                    others_address, others_port);
            }
            break;


        }
    }
}

/**
 * @brief part of parseMessage that is responsible only for
 * parsing remote execute command.
 * 
 * @param andruav_message 
 */
void CFCBAndruavMessageParser::parseRemoteExecute (Json_de &andruav_message)
{
    const Json_de cmd = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_CMD];
    
    if (!validateField(cmd, "C", Json_de::value_t::number_unsigned)) return ;
                
    uint32_t permission = 0;
    if (validateField(andruav_message, ANDRUAV_PROTOCOL_MESSAGE_PERMISSION, Json_de::value_t::number_unsigned))
    {
        permission =  andruav_message[ANDRUAV_PROTOCOL_MESSAGE_PERMISSION].get<int>();
    }

    UNUSED (permission);
    
    bool is_system = false;
    
    if ((validateField(andruav_message, ANDRUAV_PROTOCOL_SENDER, Json_de::value_t::string)) && (andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>().compare(SPECIAL_NAME_SYS_NAME)==0))
    {   // permission is not needed if this command sender is the communication server not a remote GCS or Unit.
        is_system = true;
    }
    
    bool is_inter_module = false;
    if ((validateField(andruav_message, INTERMODULE_ROUTING_TYPE, Json_de::value_t::string)) && (andruav_message[INTERMODULE_ROUTING_TYPE].get<std::string>().compare(CMD_TYPE_INTERMODULE)==0))
    {   // permission is not needed if this command sender is the communication server not a remote GCS or Unit.
        is_inter_module = true;
    }
    
    const int remoteCommand = cmd["C"].get<int>();
    std::cout << "cmd: " << remoteCommand << std::endl;
    switch (remoteCommand)
    {
        case RemoteCommand_REQUEST_PARA_LIST:
            if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;
            
            CFCBFacade::getInstance().sendParameterList(andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>());
        break;

        case TYPE_AndruavMessage_HomeLocation:
            if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;
            
            CFCBFacade::getInstance().sendHomeLocation(andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>());
        break;

        case RemoteCommand_MISSION_COUNT:
            // mission cound and mission current are sent together in DE.
        case RemoteCommand_MISSION_CURRENT:
            CFCBFacade::getInstance().sendMissionCurrent(std::string());
        break;

        case RemoteCommand_RELOAD_WAY_POINTS_FROM_FCB:
            if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;
            
            m_mission_manager.reloadWayPoints();
        break;

        case RemoteCommand_CLEAR_WAY_POINTS_FROM_FCB:
            if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;
            
            if ((!is_system) && ((permission & PERMISSION_ALLOW_GCS_WP_CONTROL) != PERMISSION_ALLOW_GCS_WP_CONTROL))
            {
                std::cout << _INFO_CONSOLE_BOLD_TEXT << "CLEAR_WAY_POINTS_FROM_FCB: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                break;
            }
                
            m_mission_manager.clearWayPoints();
        break;
        

        case RemoteCommand_CLEAR_FENCE_DATA:
        {
            if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;
            if ((!is_system) && ((permission & PERMISSION_ALLOW_GCS_WP_CONTROL) != PERMISSION_ALLOW_GCS_WP_CONTROL)) 
            {
                std::cout << _INFO_CONSOLE_BOLD_TEXT << "CLEAR_FENCE_DATA: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                break;
            }
                
            std::string fence_name;
            if (cmd.contains("fn")==true)
            {
                fence_name = cmd["fn"].get<std::string>();
            }
            //clears all geo fence info for this unit and other units.
            geofence::CGeoFenceManager::getInstance().clearGeoFences(fence_name);
        }
        break;

        case RemoteCommand_SET_START_MISSION_ITEM:
            if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;
            
            if (!validateField(cmd, "n", Json_de::value_t::number_unsigned)) return ;
            mavlinksdk::CMavlinkCommand::getInstance().setCurrentMission(cmd["n"].get<int>());
        break;


        case RemoteCommand_CONNECT_FCB:
        {
            // This is an Andruav command and not applicable here till now.
        }
        break;

        case TYPE_AndruavSystem_LoadTasks:
        {
            CFCBFacade::getInstance().callModule_reloadSavedTasks(TYPE_AndruavSystem_LoadTasks);
        
        }
        break;

        case TYPE_AndruavMessage_GeoFence:
        {
            // receive GeoFence info from drones.
            std::string fence_name;
            geofence::GEO_FENCE_STRUCT * geo_fence_struct = nullptr;

            // capture a fence of a specific name.
            if (cmd.contains("fn")==true)
            {
                fence_name = cmd["fn"].get<std::string>();
                geo_fence_struct = geofence::CGeoFenceManager::getInstance().getFenceByName (fence_name);
            }
            
            // if fence name exists, send back info.
            if (geo_fence_struct != nullptr) 
            {
                CFCBFacade::getInstance().sendGeoFenceToTarget(andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>(), geo_fence_struct);
            } 
            else
            {
                // send all fences data. as cmd["fn"] is null or ""
                std::vector<geofence::GEO_FENCE_STRUCT*> geo_fence_struct = geofence::CGeoFenceManager::getInstance().getFencesOfParty (m_fcbMain.getAndruavVehicleInfo().party_id);
                const std::size_t size = geo_fence_struct.size();

                for(int i = 0; i < size; i++)
                {
                    CFCBFacade::getInstance().sendGeoFenceToTarget(andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>(), geo_fence_struct[i]);
                }
            }
        }
        break;

            
        case TYPE_AndruavMessage_GeoFenceAttachStatus:
        {
            std::string fence_name;
            if (cmd.contains("fn")==true)
            {
                fence_name = cmd["fn"].get<std::string>();
            }

            CFCBFacade::getInstance().sendGeoFenceAttachedStatusToTarget(andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>(), fence_name);
        }
        break;

        case TYPE_AndruavMessage_UDPProxy_Info:
        {
            if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;
            if ((!is_system) && ((permission & PERMISSION_ALLOW_GCS_FULL_CONTROL) != PERMISSION_ALLOW_GCS_FULL_CONTROL))
            {
                std::cout << _INFO_CONSOLE_BOLD_TEXT << "UDPProxy_Info: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                break;
            }
            
            m_fcbMain.sendUdpProxyStatus(andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>());         
        }
        break;

        case RemoteCommand_TELEMETRYCTRL:
        {
            if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;
            if ((!is_inter_module) && (!is_system) && ((permission & PERMISSION_ALLOW_GCS_FULL_CONTROL) != PERMISSION_ALLOW_GCS_FULL_CONTROL))
            {
                std::cout << _INFO_CONSOLE_BOLD_TEXT << "TELEMETRYCTRL: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                break;
            }
            
            if (!validateField(cmd, "Act", Json_de::value_t::number_unsigned)) return ;
            const int request_type = cmd["Act"].get<int>();
            int streaming_level = -1;
                    
            switch (request_type)
            {
                case CONST_TELEMETRY_ADJUST_RATE:
                {
                    if (validateField(cmd, "LVL", Json_de::value_t::number_unsigned))
                    {
                        streaming_level = cmd["LVL"].get<int>();
                    }
                }
                break;
                case CONST_TELEMETRY_REQUEST_PAUSE:
                    m_fcbMain.pauseUDPProxy(true);
                break;

                case CONST_TELEMETRY_REQUEST_RESUME:
                    m_fcbMain.pauseUDPProxy(false);
                break;

                default:
                return ;
            }
            
            m_fcbMain.setStreamingLevel(streaming_level);
            
            if (is_inter_module == true)
            {
                m_fcbMain.sendUdpProxyStatus(std::string(""));
            }
        }
        break;


        case RemoteCommand_SET_UDPPROXY_CLIENT_PORT:
        {   
            /**
             * @brief Notice that fixed port requires comunication server to accept this request. 
             * Communication server can choose to ignore this setting based on it settings in server.config
             * 
             */
            
            if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;
            if ((!is_system) && ((permission & PERMISSION_ALLOW_GCS_FULL_CONTROL) != PERMISSION_ALLOW_GCS_FULL_CONTROL))
            {
                std::cout << _INFO_CONSOLE_BOLD_TEXT << "SET_UDPPROXY_CLIENT_PORT: "  << _ERROR_CONSOLE_BOLD_TEXT_ << "Permission Denied." << _NORMAL_CONSOLE_TEXT_ << std::endl;
                break;
            }
            
            if (cmd.contains("P")==true)
            {
                // read client port
                uint32_t udp_proxy_fixed_port = cmd["P"].get<int>();
                if (udp_proxy_fixed_port >= 0xffff) break;
                
                de::CLocalConfigFile& cLocalConfigFile = de::CLocalConfigFile::getInstance();
                cLocalConfigFile.addNumericField("udp_proxy_fixed_port",udp_proxy_fixed_port);
                cLocalConfigFile.apply();

                std::cout << std::endl << _ERROR_CONSOLE_BOLD_TEXT_ << "Change UDPProxy Port to " << _INFO_CONSOLE_TEXT << udp_proxy_fixed_port <<  std::endl;
                PLOG(plog::warning) << "SET_UDPPROXY_CLIENT_PORT: Change UDPProxy Port to:" << udp_proxy_fixed_port;
            
                m_fcb_facade.sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_WARNING, std::string("UDPProxy port update initiated."));

                m_fcbMain.requestChangeUDPProxyClientPort(udp_proxy_fixed_port);
            }
        }

    } 
}