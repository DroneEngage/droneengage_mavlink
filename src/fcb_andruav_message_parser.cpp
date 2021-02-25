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
                bool arm = message["A"].get<bool>();
                bool force = false;
                if (message.contains("D") == true)
                {
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

                int andruav_mode = message["F"].get<int>();
                double langitude = 0.0f;
                // TODO: Missing circle and guided go to here mode.
                int ardupilot_mode = uavos::fcb::CFCBModes::getArduPilotMode(andruav_mode, m_fcbMain.getAndruavVehicleInfo().vehicle_type);
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
                // a : altitude 
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
                double latitude  = message["a"].get<double>();
                double longitude = message["g"].get<double>();
                double altitude  = m_mavlinksdk.getVehicle().get()->getMsgGlobalPositionInt().relative_alt;
                if (message.contains("l") == true)
                {
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
                if ((message.contains("a")!=true) || (message["a"].type() != Json::value_t::string))
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
            mavlinksdk::CMavlinkCommand::getInstance().setCurrentMission(cmd["n"].get<int>());
        break;
    } 
}