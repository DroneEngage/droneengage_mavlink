#include <ardupilotmega/mavlink.h>

#include "./helpers/colors.h"
#include "mavlink_helper.h"

/**
 * Get string readable meaning of CMD_ACK error number.
 * @param result value in CMD_ACK
 */
std::string mavlinksdk::CMavlinkHelper::getACKError (const int result)
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


/**
 * Classify heartbeat.mav_type into main vehicle types to be able to extract modes.
 * @param mav_type  mavlink_heartbeat_t.type (MAV_TYPE)
 * @param autopilot_type mavlink_heartbeat_t.autopilot (MAV_AUTOPILOT)
 * */
mavlinksdk::FIRMWARE_TYPE mavlinksdk::CMavlinkHelper::getFirmewareType (int mav_type,int autopilot_type)
{
    if ((autopilot_type == MAV_AUTOPILOT_GENERIC) || (autopilot_type == MAV_AUTOPILOT_ARDUPILOTMEGA)) 
    {

        switch (mav_type)
        {
            case MAV_TYPE_QUADROTOR: /* Quadrotor | */
            case MAV_TYPE_HEXAROTOR: /* Hexarotor | */
            case MAV_TYPE_OCTOROTOR: /* Octorotor | */
            case MAV_TYPE_TRICOPTER: /* Tricopter | */
            case MAV_TYPE_COAXIAL: /* Coaxial helicopter | */
            case MAV_TYPE_HELICOPTER: /* Normal helicopter with tail rotor. | */
            #ifdef DEBUG
                std::cout << _LOG_CONSOLE_TEXT << "FIRMWARE_TYPE_ARDU_COPTER" << _NORMAL_CONSOLE_TEXT_ << std::endl; 
            #endif
                return mavlinksdk::FIRMWARE_TYPE_ARDU_COPTER;
    
            
            case MAV_TYPE_GROUND_ROVER: /* Ground rover | */
            case MAV_TYPE_SURFACE_BOAT: /* Surface vessel, boat, ship | */
            #ifdef DEBUG
                std::cout << _LOG_CONSOLE_TEXT << "FIRMWARE_TYPE_ARDU_ROVER" << _NORMAL_CONSOLE_TEXT_ << std::endl; 
            #endif
                return mavlinksdk::FIRMWARE_TYPE_ARDU_ROVER;

            case MAV_TYPE_SUBMARINE: /* Submarine | */
            #ifdef DEBUG
                std::cout << _LOG_CONSOLE_TEXT << "FIRMWARE_TYPE_ARDU_SUB" << _NORMAL_CONSOLE_TEXT_ << std::endl; 
            #endif
                return mavlinksdk::FIRMWARE_TYPE_ARDU_SUB;

            case MAV_TYPE_FLAPPING_WING: /* Flapping wing | */
            case MAV_TYPE_VTOL_DUOROTOR: /* Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter. | */
            case MAV_TYPE_VTOL_QUADROTOR: /* Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter. | */
            case MAV_TYPE_VTOL_TILTROTOR: /* Tiltrotor VTOL | */
            case MAV_TYPE_VTOL_RESERVED2: /* VTOL reserved 2 | */
            case MAV_TYPE_VTOL_RESERVED3: /* VTOL reserved 3 | */
            case MAV_TYPE_VTOL_RESERVED4: /* VTOL reserved 4 | */
            case MAV_TYPE_VTOL_RESERVED5: /* VTOL reserved 5 | */
            case MAV_TYPE_FIXED_WING:
            #ifdef DEBUG
                std::cout << _LOG_CONSOLE_TEXT << "FIRMWARE_TYPE_ARDU_PLANE" << _NORMAL_CONSOLE_TEXT_ << std::endl; 
            #endif
                return mavlinksdk::FIRMWARE_TYPE_ARDU_PLANE;
            default:
            #ifdef DEBUG
                std::cout << _LOG_CONSOLE_TEXT << "FIRMWARE_TYPE_UNKNOWN" << _NORMAL_CONSOLE_TEXT_ << std::endl; 
            #endif
                return mavlinksdk::FIRMWARE_TYPE_UNKNOWN;
        };
        
    }

    return mavlinksdk::FIRMWARE_TYPE_UNKNOWN;
}


/***
 * @param mode - int heartbeat.custom_mode
 * @param autopilot_type - mavlinksdk::FIRMWARE_TYPE 
 * */
std::string mavlinksdk::CMavlinkHelper::getMode (int mode, int autopilot_type)
{
    switch (autopilot_type)
    {
        case FIRMWARE_TYPE_ARDU_PLANE:
            return  mavlinksdk::CMavlinkHelper::getPlaneMode (mode);

        case FIRMWARE_TYPE_ARDU_COPTER:
            return  mavlinksdk::CMavlinkHelper::getCopterMode (mode);

        case FIRMWARE_TYPE_ARDU_SUB:
            return  mavlinksdk::CMavlinkHelper::getSubMode (mode);

        case FIRMWARE_TYPE_ARDU_ROVER:
            return  mavlinksdk::CMavlinkHelper::getRoverMode (mode);

    }

    return "unknown";
}

std::string mavlinksdk::CMavlinkHelper::getPlaneMode (int mode)
{
    switch (mode)
    {
        case PLANE_MODE_MANUAL:
            return "Manual";
        case PLANE_MODE_CIRCLE:
            return "Circle";
        case PLANE_MODE_STABILIZE:
            return "Stabilizer";
        case PLANE_MODE_TRAINING:
            return "Training";
        case PLANE_MODE_ACRO:
            return "Acro";
        case PLANE_MODE_FLY_BY_WIRE_A:
            return "FBWA";
        case PLANE_MODE_FLY_BY_WIRE_B:
            return "FBWB";
        case PLANE_MODE_CRUISE:
            return "Cruise";
        case PLANE_MODE_AUTOTUNE:
            return "AutoTune";
        case PLANE_MODE_AUTO:
            return "Auto";
        case PLANE_MODE_RTL:
            return "RTL";
        case PLANE_MODE_LOITER:
            return "Loiter";
        case PLANE_MODE_TAKEOFF:
            return "TakeOff";
        case PLANE_MODE_AVOID_ADSB:
            return "Avoid ADSB";
        case PLANE_MODE_GUIDED:
            return "Guided";
        case PLANE_MODE_INITIALIZING:
            return "Initializing";
        case PLANE_MODE_QSTABILIZE:
            return "QStabilize";
        case PLANE_MODE_QHOVER:
            return "QHover";
        case PLANE_MODE_QLOITER:
            return "QLoiter";
        case PLANE_MODE_QLAND:
            return "QLand";
        case PLANE_MODE_QRTL:
            return "QRTL";
        case PLANE_MODE_QAUTOTUNE:
            return "QAutoTune";
        case PLANE_MODE_QACRO:
            return "QACRO";
    }

    return "unknown";
}

std::string mavlinksdk::CMavlinkHelper::getCopterMode (int mode)
{
    switch (mode)
    {
        case COPTER_MODE_STABILIZE:
            return "Stabilize";
        case COPTER_MODE_ACRO:
            return "Acro";
        case COPTER_MODE_ALT_HOLD:
            return "Alt-Hold";
        case COPTER_MODE_AUTO:
            return "Auto";
        case COPTER_MODE_GUIDED:
            return "Guided";
        case COPTER_MODE_LOITER:
            return "Loiter";
        case COPTER_MODE_RTL:
            return "RTL";
        case COPTER_MODE_CIRCLE:
            return "Circle";
        case COPTER_MODE_LAND:
            return "Land";
        case COPTER_MODE_DRIFT:
            return "Drift";
        case COPTER_MODE_SPORT:
            return "Sport";
        case COPTER_MODE_FLIP:
            return "Flip";
        case COPTER_MODE_AUTOTUNE:
            return "AutoTune";
        case COPTER_MODE_POSHOLD:
            return "Pos-Hold";
        case COPTER_MODE_BRAKE:
            return "Brake";
        case COPTER_MODE_THROW:
            return "Throw";
        case COPTER_MODE_AVOID_ADSB:
            return "Avoid ADSB";
        case COPTER_MODE_GUIDED_NOGPS:
            return "No-GPS";
        case COPTER_MODE_SMART_RTL:
            return "S-RTL";
        case COPTER_MODE_FLOWHOLD:
            return "Flow Hold";
        case COPTER_MODE_FOLLOW:
            return "Follow";
        case COPTER_MODE_ZIGZAG:
            return "ZigZag";
        case COPTER_MODE_SYSTEMID:
            return "Sys-ID";
        case COPTER_MODE_AUTOROTATE:
            return "Auto Rotate";
    }

    return "unknown";
}

std::string mavlinksdk::CMavlinkHelper::getSubMode (int mode)
{
    switch (mode)
    {
        case SUB_MODE_STABILIZE:
            return "Stabilize";
        case SUB_MODE_ACRO:
            return "Acro";
        case SUB_MODE_ALT_HOLD:
            return "Alt-Hold";
        case SUB_MODE_AUTO:
            return "Auto";
        case SUB_MODE_GUIDED:
            return "Guided";
        case SUB_MODE_CIRCLE:
            return "Circle";
        case SUB_MODE_SURFACE:
            return "Surface";
        case SUB_MODE_POSHOLD:
            return "Pos-Hold";
        case SUB_MODE_MANUAL:
            return "Manual";
    }

    return "unknown";
}


std::string mavlinksdk::CMavlinkHelper::getRoverMode (int mode)
{

    switch (mode)
    {
        case ROVER_MODE_MANUAL:
            return "Manual";
        case ROVER_MODE_ACRO:
            return "Acro";
        case ROVER_MODE_STEERING:
            return "Steering";
        case ROVER_MODE_HOLD:
            return "Hold";
        case ROVER_MODE_LOITER:
            return "Loiter";
        case ROVER_MODE_FOLLOW:
            return "Follow";
        case ROVER_MODE_SIMPLE:
            return "Simple";
        case ROVER_MODE_AUTO:
            return "Auto";
        case ROVER_MODE_RTL:
            return "RTL";
        case ROVER_MODE_SMART_RTL:
            return "S-RTL";
        case ROVER_MODE_GUIDED:
            return "Guided";
        case ROVER_MODE_INITIALIZING:
            return "Initializing";
    }

    return "unknown";
}

