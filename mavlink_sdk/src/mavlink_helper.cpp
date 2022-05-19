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
        case MAV_CMD_ACK_OK:
            err = "succeeded";
            break;
        
        case MAV_CMD_ACK_ERR_FAIL:
            err = "Generic error message if none of the other reasons fails or if no detailed error reporting is implemented.";
            break;

        case MAV_CMD_ACK_ERR_ACCESS_DENIED:
            err = "The system is refusing to accept this command from communication partner.";
            break;

        case MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED:
            err = "The coordinate frame of this command or mission item is not supported.";
            break;

        case MAV_CMD_ACK_ERR_NOT_SUPPORTED:
            err = "Command or mission item is not supported, other commands would be accepted.";
            break;

        case MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE:
            err = "The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible.";
            break;

        case MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE:
            err = "The X or latitude value is out of range.";
            break;
        
        case MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE:
            err = "The Y or longitude value is out of range.";
            break;
        
        case MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE:
            err = "The Z or altitude value is out of range.";
            break;

        default:
            err = "Unknown";
            break;
    };

    return err;
}


