#include <iostream>
#include "defines.hpp"
#include "messages.hpp"
#include "fcb_modes.hpp"
#include "fcb_andruav_message_parser.hpp"
#include "./mission/mission_translator.hpp"

/**
 * @brief Called by external scheduler at rate of 1Hz
 * */
void Scheduler_1Hz ()
{

}

/**
 * @brief Parse messages receuved from uavos_comm"
 * 
 * @param andruav_message message received from uavos_comm
 */
void uavos::fcb::CFCBAndruavResalaParser::parseMessage (Json &andruav_message)
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

            case TYPE_AndruavResala_Arm:
            {
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

            case TYPE_AndruavResala_FlightControl:
            {
                // F  : andruve unit mode
                // [g]: longitude
                // [a]: latitude
                // [r]: radius 

                if (!validateField(message, "F", Json::value_t::number_unsigned)) return ;
                
                const int andruav_mode = message["F"].get<int>();
                double langitude = 0.0f;
                // TODO: Missing circle and guided go to here mode.
                const int ardupilot_mode = uavos::fcb::CFCBModes::getArduPilotMode(andruav_mode, m_fcbMain.getAndruavVehicleInfo().vehicle_type);
                if (ardupilot_mode == E_UNDEFINED_MODE)
                {   
                    //TODO: Send Error Message
                    return ;
                }

                mavlinksdk::CMavlinkCommand::getInstance().doSetMode(ardupilot_mode);
            }
            break;

            case TYPE_AndruavResala_ChangeAltitude:
            {
                #ifdef DEBUG
                    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: TYPE_AndruavResala_ChangeAltitude " << _NORMAL_CONSOLE_TEXT_ << std::endl;
                #endif
                // a : altitude 
                if ((!validateField(message, "a", Json::value_t::number_float)) 
                && (!validateField(message, "a", Json::value_t::number_unsigned)))
                    return ;
                
                double altitude = message["a"].get<double>();
                
                mavlinksdk::CVehicle *vehicle =  m_mavlinksdk.getVehicle().get();

                if (vehicle->isFlying()== true)
                {
                    mavlinksdk::CMavlinkCommand::getInstance().changeAltitude (altitude );
                }
                else
                {
                    mavlinksdk::CMavlinkCommand::getInstance().takeOff (altitude );
                }
                
            }
            break;

            case TYPE_AndruavResala_Land:
            {
                //TODO: could be included in change mode.

                int ardupilot_mode = uavos::fcb::CFCBModes::getArduPilotMode(VEHICLE_MODE_LAND, m_fcbMain.getAndruavVehicleInfo().vehicle_type);
                if (ardupilot_mode == E_UNDEFINED_MODE)
                {   
                    //TODO: Send Error Message
                    return ;
                }

                mavlinksdk::CMavlinkCommand::getInstance().doSetMode(ardupilot_mode);
            }
            break;

            case TYPE_AndruavResala_GuidedPoint:
            {
                //  a : latitude
                //  g : longitude
                // [l]: altitude
                // other fields are obscelete.
                
                if (!validateField(message, "a", Json::value_t::number_float)) return ;
                if (!validateField(message, "g", Json::value_t::number_float)) return ;
                
                double latitude  = message["a"].get<double>();
                double longitude = message["g"].get<double>();
                double altitude  = m_mavlinksdk.getVehicle().get()->getMsgGlobalPositionInt().relative_alt;
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
            }   

            break;

            case TYPE_AndruavResala_SET_HOME_LOCATION:
            {
                // T: latitude
                // O: longitude
                // A: altitude
                if (!validateField(message, "T", Json::value_t::number_float)) return ;
                if (!validateField(message, "O", Json::value_t::number_float)) return ;
                if (!validateField(message, "A", Json::value_t::number_float)) return ;

                double latitude  = message["T"].get<double>();
                double longitude = message["O"].get<double>();
                double altitude  = message["A"].get<double>();
                
                mavlinksdk::CMavlinkCommand::getInstance().setHome(0, latitude, longitude, altitude);
            }
            break;

            case TYPE_AndruavResala_DoYAW:
            {
                // A : target_angle
                // R : turn_rate
                // C : is_clock_wise
                // L : is_relative
                if ((!validateField(message, "A", Json::value_t::number_float)) 
                && (!validateField(message, "A", Json::value_t::number_unsigned)))
                    return ;
                if ((!validateField(message, "R", Json::value_t::number_float)) 
                && (!validateField(message, "R", Json::value_t::number_unsigned)))
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

            case TYPE_AndruavResala_ChangeSpeed:
            {
                // a : speed
                // b : is_ground_speed
                // c : throttle
                // d : is_relative
                if ((!validateField(message, "a", Json::value_t::number_float)) 
                && (!validateField(message, "a", Json::value_t::number_unsigned)))
                    return ;
                if (!validateField(message, "b", Json::value_t::boolean)) return ;
                if ((!validateField(message, "c", Json::value_t::number_float)) 
                && (!validateField(message, "c", Json::value_t::number_unsigned)))
                    return ;
                if (!validateField(message, "d", Json::value_t::boolean)) return ;

                double speed = message["a"].get<double>();
                bool is_ground_speed = message["b"].get<bool>();
                double throttle = message["c"].get<double>();
                bool is_relative = message["d"].get<bool>();
                
                const int speed_type = is_ground_speed?1:0;

                mavlinksdk::CMavlinkCommand::getInstance().setNavigationSpeed(1, speed, throttle, is_relative);
            }
            break;

            case TYPE_AndruavResala_UploadWayPoints:
            {
                //TODO: you should allow multiple messages to allow large file to be received.
                // !UDP packet has maximum size.
                
                /*
                    a : std::string serialized mission file
                */
                if (!validateField(message, "a", Json::value_t::string)) 
                {
                    m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_3DR, NOTIFICATION_TYPE_ERROR, "Bad input plan file");

                    break ;
                }

                std::string mission = message ["a"];
                uavos::fcb::mission::CMissionTranslator cMissionTranslator;
                
                std::unique_ptr<std::map <int, std::unique_ptr<uavos::fcb::mission::CMissionItem>>> new_mission_items = cMissionTranslator.translateMissionText(mission);
                if (new_mission_items == std::nullptr_t())
                {
                    m_fcb_facade.sendErrorMessage(std::string(), 0, ERROR_3DR, NOTIFICATION_TYPE_ERROR, "Bad input plan file");

                    break ;
                }
                m_fcbMain.clearWayPoints();
                uavos::fcb::mission::ANDRUAV_UNIT_MISSION& andruav_missions = m_fcbMain.getAndruavMission();                
                
             
                std::map<int, std::unique_ptr<uavos::fcb::mission::CMissionItem>>::iterator it;
                for (it = new_mission_items->begin(); it != new_mission_items->end(); it++)
                {
                    int seq = it->first;
                    
                    andruav_missions.mission_items.insert(std::make_pair( seq, std::move(it->second)));
                }

                new_mission_items->clear();
                
                m_fcbMain.saveWayPointsToFCB();
                
            }
            break;

            case TYPE_AndruavResala_ServoChannel:
            {
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

            case TYPE_AndruavResala_RemoteControlSettings:
            {
                // b: remote control setting
                
                if (!validateField(message, "b",Json::value_t::number_unsigned)) return ;
                
                int rc_sub_action = message["b"].get<int>();
                m_fcbMain.adjustRemoteJoystickByMode((RC_SUB_ACTION)rc_sub_action);
            }
            break;

            case TYPE_AndruavResala_RemoteControl2:
            {
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
                
                int16_t rc_channels[16] = {0};
                rc_channels[3] = message["R"].get<int>();
                rc_channels[2] = message["T"].get<int>();
                rc_channels[0] = message["A"].get<int>();
                rc_channels[1] = message["E"].get<int>();
                rc_channels[4] = validateField(message, "w",Json::value_t::number_integer)?message["w"].get<int>():-999;
                rc_channels[5] = validateField(message, "x",Json::value_t::number_integer)?message["x"].get<int>():-999;
                rc_channels[6] = validateField(message, "y",Json::value_t::number_integer)?message["y"].get<int>():-999;
                rc_channels[7] = validateField(message, "z",Json::value_t::number_integer)?message["z"].get<int>():-999;

                for (int i=8;i<18;++i)
                {
                    rc_channels[i] = -999;
                }

                m_fcbMain.updateRemoteControlChannels(rc_channels);

            }

            case TYPE_AndruavResala_RemoteExecute:
            {
                parseRemoteExecute(andruav_message);
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
void uavos::fcb::CFCBAndruavResalaParser::parseRemoteExecute (Json &andruav_message)
{
    const Json cmd = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_CMD];
    
    if (!validateField(cmd, "C", Json::value_t::number_unsigned)) return ;
                
    const int remoteCommand = cmd["C"].get<int>();
    std::cout << "cmd: " << remoteCommand << std::endl;
    switch (remoteCommand)
    {
        case TYPE_AndruavResala_SET_HOME_LOCATION:
            
            m_fcb_facade.sendHomeLocation(andruav_message[ANDRUAV_PROTOCOL_SENDER]);
        break;

        case RemoteCommand_RELOAD_WAY_POINTS_FROM_FCB:
            m_fcbMain.reloadWayPoints();
            //mavlinksdk::CMavlinkCommand::getInstance().reloadWayPoints();
        break;

        case RemoteCommand_CLEAR_WAY_POINTS_FROM_FCB:
            m_fcbMain.clearWayPoints();
            //mavlinksdk::CMavlinkCommand::getInstance().clearWayPoints();
        break;
        

        case RemoteCommand_CLEAR_FENCE_DATA:
        break;

        case RemoteCommand_SET_START_MISSION_ITEM:
            if (!validateField(cmd, "n", Json::value_t::number_unsigned)) return ;
            mavlinksdk::CMavlinkCommand::getInstance().setCurrentMission(cmd["n"].get<int>());
        break;
    } 
}