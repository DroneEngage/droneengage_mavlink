#include <iostream>
#include "defines.hpp"
#include "messages.hpp"
#include "fcb_modes.hpp"
#include "fcb_andruav_message_parser.hpp"


/**
 * Called by external scheduler at rate of 1Hz
 * */
void Scheduler_1Hz ()
{

}



/**
 * Parse messages receuved from uavos_comm"
 * */
void uavos::fcb::CFCBAndruavMessageParser::parseMessage (Json &andruav_message)
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

            case TYPE_AndruavMessage_Arm:
            {
                // A  : bool arm/disarm
                // [D]: bool force 
                bool arm = message["A"].get<bool>();
                bool force = false;
                if (message.contains("D") == true)
                {
                    force = message["D"].get<bool>();
                }
                m_mavlinkCommand.doArmDisarm(arm,force);
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

                m_mavlinkCommand.doSetMode(ardupilot_mode);
            }
            break;

            case TYPE_AndruavResala_ChangeAltitude:
            {
                // a : altitude 
                const char *str_altitude = message["a"].get<std::string>().c_str();
                float altitude = atof(str_altitude);

                m_mavlinkCommand.changeAltitude (altitude);
                
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

                m_mavlinkCommand.doSetMode(ardupilot_mode);
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


void uavos::fcb::CFCBAndruavMessageParser::parseRemoteExecute (Json &andruav_message)
{
    const Json cmd = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_CMD];
    const int remoteCommand = cmd["C"].get<int>();
    std::cout << "cmd: " << remoteCommand << std::endl;
    // switch (remoteCommand)
    // {
        
    // } 
}