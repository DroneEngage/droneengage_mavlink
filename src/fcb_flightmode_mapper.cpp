#include <ardupilotmega/mavlink.h>

#include "./helpers/colors.h"
#include "fcb_flightmode_mapper.h"
#include "px4_modes.h"


/**
 * Classify heartbeat.mav_type into main vehicle types to be able to extract modes.
 * @param mav_type  mavlink_heartbeat_t.type (MAV_TYPE)
 * @param autopilot_type mavlink_heartbeat_t.autopilot (MAV_AUTOPILOT)
 * */
uavos::fcb::FIRMWARE_TYPE uavos::fcb::CFlightModeMapper::getFirmewareType (const int mav_type, const int autopilot_type)
{
    
    switch (mav_type)
    {
        case MAV_TYPE_QUADROTOR: /* Quadrotor | */
        case MAV_TYPE_HEXAROTOR: /* Hexarotor | */
        case MAV_TYPE_OCTOROTOR: /* Octorotor | */
        case MAV_TYPE_TRICOPTER: /* Tricopter | */
        case MAV_TYPE_COAXIAL:   /* Coaxial helicopter | */
        case MAV_TYPE_HELICOPTER: /* Normal helicopter with tail rotor. | */
            #ifdef DEBUG
                std::cout << _LOG_CONSOLE_TEXT << "FIRMWARE_TYPE_ARDU_COPTER" << _NORMAL_CONSOLE_TEXT_ << std::endl; 
            #endif
            return uavos::fcb::FIRMWARE_TYPE_ARDU_COPTER;
    
            
        case MAV_TYPE_GROUND_ROVER: /* Ground rover | */
        case MAV_TYPE_SURFACE_BOAT: /* Surface vessel, boat, ship | */
            #ifdef DEBUG
                std::cout << _LOG_CONSOLE_TEXT << "FIRMWARE_TYPE_ARDU_ROVER" << _NORMAL_CONSOLE_TEXT_ << std::endl; 
            #endif
            return uavos::fcb::FIRMWARE_TYPE_ARDU_ROVER;

        case MAV_TYPE_SUBMARINE: /* Submarine | */
            #ifdef DEBUG
                std::cout << _LOG_CONSOLE_TEXT << "FIRMWARE_TYPE_ARDU_SUB" << _NORMAL_CONSOLE_TEXT_ << std::endl; 
            #endif
            return uavos::fcb::FIRMWARE_TYPE_ARDU_SUB;

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
            return uavos::fcb::FIRMWARE_TYPE_ARDU_PLANE;
        
        default:
            #ifdef DEBUG
                std::cout << _LOG_CONSOLE_TEXT << "FIRMWARE_TYPE_UNKNOWN" << _NORMAL_CONSOLE_TEXT_ << std::endl; 
            #endif
            return uavos::fcb::FIRMWARE_TYPE_UNKNOWN;
            
        };
        
    return uavos::fcb::FIRMWARE_TYPE_UNKNOWN;
}


/***
 * @param mode - int heartbeat.custom_mode
 * @param autopilot_type - uavos::fcb::FIRMWARE_TYPE 
 * */
std::string uavos::fcb::CFlightModeMapper::getMode (const int mode, const int autopilot_type)
{
    switch (autopilot_type)
    {
        case FIRMWARE_TYPE_ARDU_PLANE:
            return  uavos::fcb::CFlightModeMapper::getPlaneMode (mode, autopilot_type);

        case FIRMWARE_TYPE_ARDU_COPTER:
            return  uavos::fcb::CFlightModeMapper::getCopterMode (mode, autopilot_type);

        case FIRMWARE_TYPE_ARDU_SUB:
            return  uavos::fcb::CFlightModeMapper::getSubMode (mode, autopilot_type);

        case FIRMWARE_TYPE_ARDU_ROVER:
            return  uavos::fcb::CFlightModeMapper::getRoverMode (mode, autopilot_type);

    }

    return "unknown";
}

std::string uavos::fcb::CFlightModeMapper::getPlaneMode (const int mode, const int autopilot_type)
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

/**
 * @brief convert autopilot flight mode into text
 * 
 * @param mode 
 * @param autopilot_type 
 * @return std::string 
 */
std::string uavos::fcb::CFlightModeMapper::getCopterMode (const int mode, const int autopilot_type)
{
    switch (autopilot_type)
    {
        case MAV_AUTOPILOT::MAV_AUTOPILOT_PX4:
            px4_custom_mode custom_mode;
            custom_mode.data = 0;
            
            switch (mode)
            {
                case PX4_CUSTOM_MAIN_MODE_MANUAL:
                    return "Manuak";
                case PX4_CUSTOM_MAIN_MODE_ALTCTL:
                    return "Alt-Ctrl";
                case PX4_CUSTOM_MAIN_MODE_POSCTL:
                    return "PosCtl";
                case PX4_CUSTOM_MAIN_MODE_AUTO:
                    return "Auto";
                case PX4_CUSTOM_MAIN_MODE_ACRO:
                    return "Acro";
                case PX4_CUSTOM_MAIN_MODE_OFFBOARD:
                    return "Offboard";
                case PX4_CUSTOM_MAIN_MODE_STABILIZED:
                    return "Stabilize";
                case PX4_CUSTOM_MAIN_MODE_RATTITUDE_LEGACY:
                    return "R-ATT";
                case PX4_CUSTOM_MAIN_MODE_SIMPLE:
                    return "Simple";
            }
        break;
        case MAV_AUTOPILOT::MAV_AUTOPILOT_GENERIC:
        default:
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
    }

    return "unknown";
}

std::string uavos::fcb::CFlightModeMapper::getSubMode (const int mode, const int autopilot_type)
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


std::string uavos::fcb::CFlightModeMapper::getRoverMode (const int mode, const int autopilot_type)
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

