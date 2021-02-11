#pragma once 

#include <iostream>
#include <string>
#include <common/mavlink.h>

/**
 * Get string readable meaning of CMD_ACK error number.
 * @param result value in CMD_ACK
 */
std::string getACKError (const int result)
{
    std::string err;
    switch (result)
    {
        case 0:  // I found ZEO reply with success !!
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


int modeMappingByNumber (int mav_type)
{

}


