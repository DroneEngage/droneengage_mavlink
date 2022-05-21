#include <iostream>
#include "defines.hpp"
#include "./uavos_common/messages.hpp"
#include "fcb_modes.hpp"
#include "fcb_swarm_manager.hpp"
#include "fcb_andruav_message_parser.hpp"
#include "./mission/mission_translator.hpp"
#include "./geofence/fcb_geo_fence_base.hpp"
#include "./geofence/fcb_geo_fence_manager.hpp"

using namespace uavos::fcb;



/**
 * @brief Parse messages receuved from uavos_comm"
 * 
 * @param andruav_message message received from uavos_comm
 */
void CFCBAndruavMessageParser::parseMessage (Json &andruav_message, const char * full_message, const int & full_message_length)
{
    const int messageType = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_TYPE].get<int>();
    bool is_binary = !(full_message[full_message_length-1]==125 || (full_message[full_message_length-2]==125));   // "}".charCodeAt(0)  IS TEXT / BINARY Msg  
    
    if (messageType == TYPE_AndruavMessage_RemoteExecute)
    {
        parseRemoteExecute(andruav_message);

        return ;
    }

    else
    {
        Json message = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_CMD];
        
        switch (messageType)
        {

            case TYPE_AndruavMessage_Arm:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                // A  : bool arm/disarm
                // [D]: bool force 
                if (!validateField(message, "A", Json::value_t::boolean)) return ;
                
                bool arm = message["A"].get<bool>();
                bool force = false;
                if (message.contains("D") == true)
                {
                    if (!validateField(message, "D", Json::value_t::boolean)) return ;
                
                    force = message["D"].get<bool>();
                }
                mavlinksdk::CMavlinkCommand::getInstance().doArmDisarm(arm,force);
            }
            break;

            case TYPE_AndruavMessage_FlightControl:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                // F  : andruav unit mode
                // [g]: longitude
                // [a]: latitude
                // [r]: radius 

                if (!validateField(message, "F", Json::value_t::number_unsigned)) return ;
                
                const int andruav_mode = message["F"].get<int>();
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

                // a : altitude 
                if ((!validateField(message, "a", Json::value_t::number_float)) 
                && (!validateField(message, "a", Json::value_t::number_unsigned)))
                    return ;
                
                double altitude = message["a"].get<double>();
                
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

                //  a : latitude
                //  g : longitude
                // [l]: altitude
                // other fields are obscelete.
                
                if (!validateField(message, "a", Json::value_t::number_float)) return ;
                if (!validateField(message, "g", Json::value_t::number_float)) return ;
                
                double latitude  = message["a"].get<double>();
                double longitude = message["g"].get<double>();
                double altitude  = mavlinksdk::CVehicle::getInstance().getMsgGlobalPositionInt().relative_alt;
                if (message.contains("l") == true)
                {
                    if ((!validateField(message, "l", Json::value_t::number_float)) 
                    && (!validateField(message, "l", Json::value_t::number_unsigned)))
                        return ;
                
                    double alt = message["l"].get<double>();
                    if (alt != 0.0)
                    {
                        altitude = alt * 1000.0;
                    }
                }
                

                mavlinksdk::CMavlinkCommand::getInstance().gotoGuidedPoint (latitude, longitude, altitude / 1000.0);
                CFCBFacade::getInstance().sendFCBTargetLocation (andruav_message[ANDRUAV_PROTOCOL_SENDER], latitude, longitude, altitude);
            }   
            break;

            case TYPE_AndruavMessage_SET_HOME_LOCATION:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                // T: latitude
                // O: longitude
                // A: altitude
                if (!validateField(message, "T", Json::value_t::number_float)) return ;
                if (!validateField(message, "O", Json::value_t::number_float)) return ;
                //if (!validateField(message, "A", Json::value_t::number_float)) return ;

                double latitude  = message["T"].get<double>();
                double longitude = message["O"].get<double>();
                double altitude  = message["A"].get<double>();
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

                // A : target_angle
                // R : turn_rate
                // C : is_clock_wise
                // L : is_relative
                if ((!validateField(message, "A", Json::value_t::number_unsigned)) 
                && (!validateField(message, "A", Json::value_t::number_integer)))
                    return ;
                if ((!validateField(message, "R", Json::value_t::number_unsigned)) 
                && (!validateField(message, "R", Json::value_t::number_integer)))
                    return ;
                if (!validateField(message, "C", Json::value_t::boolean)) return ;
                if (!validateField(message, "L", Json::value_t::boolean)) return ;

                double target_angle  = message["A"].get<double>();
                double turn_rate = message["R"].get<double>();
                bool is_clock_wise  = message["C"].get<bool>();
                bool is_relative = message["L"].get<bool>();
                
                mavlinksdk::CMavlinkCommand::getInstance().setYawCondition( target_angle, turn_rate, is_clock_wise, is_relative);
            }
            break;

            case TYPE_AndruavMessage_ChangeSpeed:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                // a : speed
                // b : is_ground_speed
                // c : throttle
                // d : is_relative
                if ((!validateField(message, "a", Json::value_t::number_float)) 
                && (!validateField(message, "a", Json::value_t::number_unsigned)))
                    return ;
                if (!validateField(message, "b", Json::value_t::boolean)) return ;
                if ((!validateField(message, "c", Json::value_t::number_float)) 
                && (!validateField(message, "c", Json::value_t::number_integer)))
                    return ;
                if (!validateField(message, "d", Json::value_t::boolean)) return ;

                double speed = message["a"].get<double>();
                bool is_ground_speed = message["b"].get<bool>();
                double throttle = message["c"].get<double>();
                bool is_relative = message["d"].get<bool>();
                
                const int speed_type = is_ground_speed?1:0;

                mavlinksdk::CMavlinkCommand::getInstance().setNavigationSpeed(speed_type, speed, throttle, is_relative);
            }
            break;

            case TYPE_AndruavMessage_UploadWayPoints:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                //TODO: you should allow multiple messages to allow large file to be received.
                // !UDP packet has maximum size.
                
                /*
                    a : std::string serialized mission file
                */
                if (!validateField(message, "a", Json::value_t::string)) 
                {
                    CFCBFacade::getInstance().sendErrorMessage(std::string(), 0, ERROR_3DR, NOTIFICATION_TYPE_ERROR, "Bad input plan file");

                    break ;
                }

                std::string mission = message ["a"];
                mission::CMissionTranslator cMissionTranslator;
                
                std::unique_ptr<std::map <int, std::unique_ptr<mission::CMissionItem>>> new_mission_items = cMissionTranslator.translateMissionText(mission);
                if (new_mission_items == std::nullptr_t())
                {
                    CFCBFacade::getInstance().sendErrorMessage(std::string(), 0, ERROR_3DR, NOTIFICATION_TYPE_ERROR, "Bad input plan file");

                    break ;
                }
                m_fcbMain.clearWayPoints();
                mission::ANDRUAV_UNIT_MISSION& andruav_missions = m_fcbMain.getAndruavMission();                
                
             
                std::map<int, std::unique_ptr<mission::CMissionItem>>::iterator it;
                for (it = new_mission_items->begin(); it != new_mission_items->end(); it++)
                {
                    int seq = it->first;
                    
                    andruav_missions.mission_items.insert(std::make_pair( seq, std::move(it->second)));
                }

                new_mission_items->clear();
                
                m_fcbMain.saveWayPointsToFCB();
                
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

                //if (!validateField(message, "t", Json::value_t::number_unsigned)) return ;
                std::unique_ptr<geofence::CGeoFenceBase> fence = geofence::CGeoFenceFactory::getInstance().getGeoFenceObject(message);
                geofence::CGeoFenceManager::getInstance().addFence(std::move(fence));
                geofence::CGeoFenceManager::getInstance().attachToGeoFence(m_fcbMain.getAndruavVehicleInfo().party_id, message["n"].get<std::string>());
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

                // n : servo_channel
                // v : servo_value

                if (!validateField(message, "n",Json::value_t::number_unsigned)) return ;
                if (!validateField(message, "v",Json::value_t::number_unsigned)) return ;

                int servo_channel = message["n"].get<int>();
                int servo_value = message["v"].get<int>();
                if (servo_value == 9999) servo_value = 2000;
                if (servo_value == 0) servo_value = 1000;
                mavlinksdk::CMavlinkCommand::getInstance().setServo (servo_channel, servo_value);
            
            }
            break;

            case TYPE_AndruavMessage_RemoteControlSettings:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                // b: remote control setting
                
                if (!validateField(message, "b",Json::value_t::number_unsigned)) return ;
                
                int rc_sub_action = message["b"].get<int>();
                m_fcbMain.adjustRemoteJoystickByMode((RC_SUB_ACTION)rc_sub_action);

                // update status
                CFCBFacade::getInstance().sendID(std::string(ANDRUAV_PROTOCOL_SENDER_ALL));
            }
            break;

            case TYPE_AndruavMessage_Sync_EventFire:
            {
                if (!validateField(message, "a",Json::value_t::number_unsigned)) return ;
                
                m_fcbMain.insertIncommingEvent(message["a"].get<int>());
            }
            break;

            case TYPE_AndruavMessage_RemoteControl2:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                // value: [0,1000] IMPORTANT: -999 means channel release
                // 'R': Rudder
                // 'T': Throttle
                // 'A': Aileron
                // 'E': Elevator
                // ['w']: Aux-1 optional 
                // ['x']: Aux-2 optional
                // ['y']: Aux-3 optional
                // ['z']: Aux-4 optional
                if (!validateField(message, "R",Json::value_t::number_unsigned)) return ;
                if (!validateField(message, "T",Json::value_t::number_unsigned)) return ;
                if (!validateField(message, "A",Json::value_t::number_unsigned)) return ;
                if (!validateField(message, "E",Json::value_t::number_unsigned)) return ;
                
                int16_t rc_channels[18] = {-999};
                const RCMAP_CHANNELS_MAP_INFO_STRUCT rc_map = m_fcbMain.getRCChannelsMapInfo();
                if ((rc_map.use_smart_rc) && (rc_map.is_valid))
                {
                    rc_channels[rc_map.rcmap_yaw]       = 1000 - message["R"].get<int>(); // to be aligned with default settings of Ardu
                    rc_channels[rc_map.rcmap_throttle]  = 1000 - message["T"].get<int>();
                    rc_channels[rc_map.rcmap_roll]      = 1000 - message["A"].get<int>(); // to be aligned with default settings of Ardu
                    rc_channels[rc_map.rcmap_pitch]     = 1000 - message["E"].get<int>();
                
                }
                else
                {
                    rc_channels[3] = 1000 - message["R"].get<int>(); // to be aligned with default settings of Ardu
                    rc_channels[2] = 1000 - message["T"].get<int>();
                    rc_channels[0] = 1000 - message["A"].get<int>(); // to be aligned with default settings of Ardu
                    rc_channels[1] = 1000 - message["E"].get<int>();
                    rc_channels[4] = validateField(message, "w",Json::value_t::number_integer)?message["w"].get<int>():-999;
                    rc_channels[5] = validateField(message, "x",Json::value_t::number_integer)?message["x"].get<int>():-999;
                    rc_channels[6] = validateField(message, "y",Json::value_t::number_integer)?message["y"].get<int>():-999;
                    rc_channels[7] = validateField(message, "z",Json::value_t::number_integer)?message["z"].get<int>():-999;
                }
                
                

                m_fcbMain.updateRemoteControlChannels(rc_channels);

            }

            case TYPE_AndruavMessage_LightTelemetry:
            {   
                
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

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
		            uint8_t msgReceived = mavlink_parse_char(MAVLINK_COMM_1, binary_message[i+ 1], &mavlink_message, &status);
                    if (msgReceived==0)
                    {
                        //TODO: you can add logging or warning
                    }
                }

                 mavlinksdk::CMavlinkCommand::getInstance().sendNative(mavlink_message);
            }
            break;

            case TYPE_AndruavMessage_MAVLINK:
            {   
                
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                // this is a binary message
                // search for char '0' and then binary message is the next byte after it.
                const char * binary_message = (char *)(memchr (full_message, 0x0, full_message_length));
                int binary_length = binary_message==0?0:(full_message_length - (binary_message - full_message +1) );

                mavlink_status_t status;
	            mavlink_message_t mavlink_message;
                for (int i=0; i<binary_length; ++ i)
                {
		            uint8_t msgReceived = mavlink_parse_char(MAVLINK_COMM_1, binary_message[i+ 1], &mavlink_message, &status);
                    if (msgReceived==0)
                    {
                        //TODO: you can add logging or warning
                    }
                }

                 mavlinksdk::CMavlinkCommand::getInstance().sendNative(mavlink_message);
            }
            break;

            case TYPE_AndruavMessage_MAKE_SWARM:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                // a: formationID
                // b: partyID
                // if (b:partyID) is not me then treat it as an announcement.

                if (!validateField(message, "a",Json::value_t::number_unsigned)) return ;
                if (!validateField(message, "b",Json::value_t::string)) return ;

                if (m_fcbMain.getAndruavVehicleInfo().party_id.compare( message["b"].get<std::string>())!=0)
                {
                    // this is an announcement.
                    // other drone should handle this message.
                    return ;
                }
                const int formation = message["a"].get<int>();

                m_fcb_swarm_manager.makeSwarm((ANDRUAV_SWARM_FORMATION)formation);
            }
            break;

            case TYPE_AndruavMessage_FollowHim_Request:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                // a: slave index    
                // b: leader partyID - if null the unfollow
                // c: slave partyID
                // if (c:partyID) is not me then treat it as an announcement.
                
                if (!validateField(message, "a",Json::value_t::number_integer)) return ;
                if (!validateField(message, "c",Json::value_t::string)) return ;

                if (m_fcbMain.getAndruavVehicleInfo().party_id.compare(message["c"].get<std::string>())!=0)
                {
                    // this is an announcement.
                    // other drone should handle this message.
                    return ;
                }

                if (!validateField(message, "b",Json::value_t::string)) 
                {
                    // unfollow
                    m_fcb_swarm_manager.unFollow();
                }
                else
                {
                    const int slave_index = message["a"].get<int>();
                    const std::string leader_party_id = message["b"].get<std::string>();
                    m_fcb_swarm_manager.makeSlave(leader_party_id, slave_index);
                }

            }
            break;

            case TYPE_AndruavMessage_UpdateSwarm:
            {
                if (m_fcbMain.getAndruavVehicleInfo().is_gcs_blocked) break ;

                /*
                    a: action [SWARM_UPDATED, SWARM_DELETE]
                    b: slave index [mandatory with SWARM_UPDATED]
                    c: leader id - if this is not me then consider it a notification.
                    d: slave party id 
                */

                if (!validateField(message, "a",Json::value_t::number_integer)) return ;
                if (!validateField(message, "c",Json::value_t::string)) return ;
                if (!validateField(message, "d",Json::value_t::string)) return ;


                if (m_fcbMain.getAndruavVehicleInfo().party_id.compare(message["c"].get<std::string>())!=0)
                {
                    // this is an announcement.
                    // other drone should handle this message.
                    return ;
                }

                const int action = message["a"].get<int>();
                const std::string slave_party_id = message["d"].get<std::string>();
                    
                if (action == SWARM_UPDATED)
                {   
                    // add or modify swarm member
                    if (!validateField(message, "b",Json::value_t::number_integer)) return ;
                
                    const int slave_index = message["b"].get<int>();
                    m_fcb_swarm_manager.addSlave(slave_party_id, slave_index);

                   return ;     
                }

                if (action == SWARM_DELETE)
                {
                    // remove a swarm member
                    // TODO:NOT IMPLEMENTED.
                    //m_fcb_swarm_manager.removeSlave(slave_party_id);

                    return ;     
                }
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
void CFCBAndruavMessageParser::parseRemoteExecute (Json &andruav_message)
{
    const Json cmd = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_CMD];
    
    if (!validateField(cmd, "C", Json::value_t::number_unsigned)) return ;
                
    const int remoteCommand = cmd["C"].get<int>();
    std::cout << "cmd: " << remoteCommand << std::endl;
    switch (remoteCommand)
    {
        case RemoteCommand_REQUEST_PARA_LIST:
            CFCBFacade::getInstance().sendParameterList(andruav_message[ANDRUAV_PROTOCOL_SENDER]);
        break;

        case TYPE_AndruavMessage_SET_HOME_LOCATION:
            CFCBFacade::getInstance().sendHomeLocation(andruav_message[ANDRUAV_PROTOCOL_SENDER]);
        break;

        case RemoteCommand_RELOAD_WAY_POINTS_FROM_FCB:
            m_fcbMain.reloadWayPoints();
        break;

        case RemoteCommand_CLEAR_WAY_POINTS_FROM_FCB:
            m_fcbMain.clearWayPoints();
        break;
        

        case RemoteCommand_CLEAR_FENCE_DATA:
        {
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
            if (!validateField(cmd, "n", Json::value_t::number_unsigned)) return ;
            mavlinksdk::CMavlinkCommand::getInstance().setCurrentMission(cmd["n"].get<int>());
        break;


        case RemoteCommand_TELEMETRYCTRL:
        {
            if (!validateField(cmd, "Act", Json::value_t::number_unsigned)) return ;
            int request_type = cmd["Act"].get<int>();
            int streaming_level = -1;
            if (validateField(cmd, "LVL", Json::value_t::number_unsigned))
            {
                streaming_level = cmd["LVL"].get<int>();
            }
            
            m_fcbMain.toggleMavlinkStreaming(andruav_message[ANDRUAV_PROTOCOL_SENDER], request_type, streaming_level);
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
                CFCBFacade::getInstance().sendGeoFenceToTarget(andruav_message[ANDRUAV_PROTOCOL_SENDER], geo_fence_struct);
            } 
            else
            {
                // send all fences data. as cmd["fn"] is null or ""
                std::vector<geofence::GEO_FENCE_STRUCT*> geo_fence_struct = geofence::CGeoFenceManager::getInstance().getFencesOfParty (m_fcbMain.getAndruavVehicleInfo().party_id);
                const std::size_t size = geo_fence_struct.size();

                for(int i = 0; i < size; i++)
                {
                    CFCBFacade::getInstance().sendGeoFenceToTarget(andruav_message[ANDRUAV_PROTOCOL_SENDER], geo_fence_struct[i]);
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

            CFCBFacade::getInstance().sendGeoFenceAttachedStatusToTarget(andruav_message[ANDRUAV_PROTOCOL_SENDER], fence_name);
        }
        break;

    } 
}