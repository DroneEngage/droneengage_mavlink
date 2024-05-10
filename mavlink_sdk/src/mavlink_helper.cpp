#include <ardupilotmega/mavlink.h>

#include "./helpers/colors.h"
#include "mavlink_helper.h"

using namespace mavlinksdk;
/**
 * @brief return description of MISSION_ACK
 * 
 * @param result MISSION_ACK value
 * @return std::string 
 */
std::string CMavlinkHelper::getMissionACKResult (const int& result)
{
    switch (result)
    {
        case MAV_MISSION_ACCEPTED:
        	return "mission accepted OK";

        case MAV_MISSION_ERROR:
            return 	"Generic error / not accepting mission commands at all right now.";

        case MAV_MISSION_UNSUPPORTED_FRAME:
            return "Coordinate frame is not supported.";
        
        case MAV_MISSION_UNSUPPORTED:
            return "Command is not supported.";
        

        case MAV_MISSION_NO_SPACE:
            return "Mission items exceed storage space.";

        case MAV_MISSION_INVALID:
            return "One of the parameters has an invalid value.";

        case MAV_MISSION_INVALID_PARAM1:
            return "param1 has an invalid value.";

        case MAV_MISSION_INVALID_PARAM2:
            return "param2 has an invalid value.";

        case MAV_MISSION_INVALID_PARAM3:
            return "param3 has an invalid value.";

        case MAV_MISSION_INVALID_PARAM4:
            return "param4 has an invalid value.";

        case MAV_MISSION_INVALID_PARAM5_X:
            return "x / param5 has an invalid value.";

        case MAV_MISSION_INVALID_PARAM6_Y:
            return "y / param6 has an invalid value.";

        case MAV_MISSION_INVALID_PARAM7:
            return "z / param7 has an invalid value.";

        case MAV_MISSION_INVALID_SEQUENCE:
            return "Mission item received out of sequence";

        case MAV_MISSION_DENIED:
            return "Not accepting any mission commands from this communication partner.";

        case MAV_MISSION_OPERATION_CANCELLED:
            return "Current mission operation cancelled (e.g. mission upload, mission download).";
    }


    return std::string();

}


/**
 * @brief Get string readable meaning of MAV_CMD_ACK error number.
 * 
 * @param result MAV_CMD_ACK value
 * @return std::string 
 */
std::string CMavlinkHelper::getACKError (const int& result)
{
    std::string err;
    switch (result)
    {
        //case 0:  // I found ZEO reply with success !!
        case MAV_RESULT::MAV_RESULT_ACCEPTED:
            err = "succeeded";
            break;
        
        case MAV_RESULT::MAV_RESULT_TEMPORARILY_REJECTED:
            err = "Command is valid, but cannot be executed at this time. This is used to indicate a problem that should be fixed just by waiting (e.g. a state machine is busy, can't arm because have not got GPS lock, etc.). Retrying later should work.";
            break;
        
        case MAV_RESULT::MAV_RESULT_DENIED:
            err = "Command is invalid (is supported but has invalid parameters). Retrying same command and parameters will not work.";
            break;
        
        case MAV_RESULT::MAV_RESULT_UNSUPPORTED:
            err = "Command is not supported (unknown).";
            break;
        
        case MAV_RESULT::MAV_RESULT_FAILED:
            err = "Command is valid, but execution has failed. This is used to indicate any non-temporary or unexpected problem, i.e. any problem that must be fixed before the command can succeed/be retried. For example, attempting to write a file when out of memory, attempting to arm when sensors are not calibrated, etc.";
            break;
        
        case MAV_RESULT::MAV_RESULT_IN_PROGRESS:
            err = "Command is valid and is being executed. This will be followed by further progress updates, i.e. the component may send further COMMAND_ACK messages with result MAV_RESULT_IN_PROGRESS (at a rate decided by the implementation), and must terminate by sending a COMMAND_ACK message with final result of the operation. The COMMAND_ACK.progress field can be used to indicate the progress of the operation. There is no need for the sender to retry the command, but if done during execution, the component will return MAV_RESULT_IN_PROGRESS with an updated progress.";
            break;
        
        case MAV_RESULT::MAV_RESULT_COMMAND_LONG_ONLY:
            err = "Command is only accepted when sent as a COMMAND_LONG.";
            break;
        
        case MAV_RESULT::MAV_RESULT_COMMAND_INT_ONLY:
            err = "Command is only accepted when sent as a COMMAND_INT.";
            break;
        
        default:
            err = "Unknown";
            break;
    };

    return err;
}


