#include <ardupilotmega/mavlink.h>
#include <all/mavlink.h>
#include <mavlink_helpers.h>
#include "fcb_modes.hpp"
#include "px4_modes.h"

using namespace de::fcb;

/**
 * Converts Ardupilot unit type to Andruav vehicle type.
 * @param mav_type  mavlink_heartbeat_t.type (MAV_TYPE)
 * @param autopilot_type mavlink_heartbeat_t.autopilot (MAV_AUTOPILOT)
 **/
ANDRUAV_UNIT_TYPE CFCBModes::getAndruavVehicleType(const int mav_type)
{
    switch (mav_type)
    {
        case MAV_TYPE_QUADROTOR: /* Quadrotor | */
        case MAV_TYPE_HEXAROTOR: /* Hexarotor | */
        case MAV_TYPE_OCTOROTOR: /* Octorotor | */
            return ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_QUAD;

        case MAV_TYPE_TRICOPTER: /* Tricopter | */
            return ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_TRI;

        case MAV_TYPE_COAXIAL:    /* Coaxial helicopter | */
        case MAV_TYPE_HELICOPTER: /* Normal helicopter with tail rotor. | */
            return ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_HELI;

        case MAV_TYPE_GROUND_ROVER: /* Ground rover | */
            return ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_ROVER;

        case MAV_TYPE_SURFACE_BOAT: /* Surface vessel, boat, ship | */
            return ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_BOAT;
        
        case MAV_TYPE_SUBMARINE: /* Submarine | */
            return ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_SUBMARINE;

        case MAV_TYPE_FLAPPING_WING:  /* Flapping wing | */
        case MAV_TYPE_VTOL_DUOROTOR:  /* Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter. | */
        case MAV_TYPE_VTOL_QUADROTOR: /* Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter. | */
        case MAV_TYPE_VTOL_TILTROTOR: /* Tiltrotor VTOL | */
        case MAV_TYPE_VTOL_RESERVED2: /* VTOL reserved 2 | */
        case MAV_TYPE_VTOL_RESERVED3: /* VTOL reserved 3 | */
        case MAV_TYPE_VTOL_RESERVED4: /* VTOL reserved 4 | */
        case MAV_TYPE_VTOL_RESERVED5: /* VTOL reserved 5 | */
        case MAV_TYPE_FIXED_WING:
            return ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_PLANE;

        default:
            return ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_UNKNOWN;
    };

    return ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_UNKNOWN;
}

/**
 * Converts ardupilot modes into Andruav modes.
 * Andruav modes are flat i.e. it is not categoriezed by vehicles.
 * So same mode means the same accross all vehicles.
 * */
ANDRUAV_UNIT_MODE CFCBModes::getAndruavMode(const uint32_t mode, const int andruav_vehicle_type, const MAV_AUTOPILOT autopilot_type)
{

    switch (andruav_vehicle_type)
    {
        case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_QUAD:
        case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_TRI:
        case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_HELI:
            return CFCBModes::getAndruavModeFromMavlinkCopterMode(mode, autopilot_type);
            break;

        case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_PLANE:
            return CFCBModes::getAndruavModeFromMavlinkPlaneMode(mode, autopilot_type);
            break;

        case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_ROVER:
        case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_BOAT:
            return CFCBModes::getAndruavModeFromMavlinkRoverMode(mode, autopilot_type);
            break;

        case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_SUBMARINE:
            return CFCBModes::getAndruavModeFromMavlinkSubMode(mode, autopilot_type);
            break;
    }
}

ANDRUAV_UNIT_MODE CFCBModes::getAndruavModeFromMavlinkPlaneMode(const uint32_t &mode, const MAV_AUTOPILOT autopilot_type)
{
    switch (autopilot_type)
    {
        case MAV_AUTOPILOT::MAV_AUTOPILOT_PX4:
            return CFCBModes::getAndruavModeFromPX4PlaneMode(mode, autopilot_type);

        case MAV_AUTOPILOT::MAV_AUTOPILOT_GENERIC:
        default:
            return CFCBModes::getAndruavModeFromArdupilotPlaneMode(mode, autopilot_type);
    };
}

ANDRUAV_UNIT_MODE CFCBModes::getAndruavModeFromMavlinkCopterMode(const uint32_t &mode, const MAV_AUTOPILOT autopilot_type)
{
    switch (autopilot_type)
    {
        case MAV_AUTOPILOT::MAV_AUTOPILOT_PX4:
            return CFCBModes::getAndruavModeFromPX4CopterMode(mode, autopilot_type);

        case MAV_AUTOPILOT::MAV_AUTOPILOT_GENERIC:
        default:
            return CFCBModes::getAndruavModeFromArdupilotCopterMode(mode, autopilot_type);
    };
}

ANDRUAV_UNIT_MODE CFCBModes::getAndruavModeFromMavlinkRoverMode(const uint32_t &mode, const MAV_AUTOPILOT autopilot_type)
{
    switch (autopilot_type)
    {
        case MAV_AUTOPILOT::MAV_AUTOPILOT_PX4:
            return CFCBModes::getAndruavModeFromPX4RoverMode(mode, autopilot_type);

        case MAV_AUTOPILOT::MAV_AUTOPILOT_GENERIC:
        default:
            return CFCBModes::getAndruavModeFromArdupilotRoverMode(mode, autopilot_type);
    };
}

ANDRUAV_UNIT_MODE CFCBModes::getAndruavModeFromMavlinkSubMode(const uint32_t &mode, const MAV_AUTOPILOT autopilot_type)
{
    switch (autopilot_type)
    {
        case MAV_AUTOPILOT::MAV_AUTOPILOT_PX4:
            return CFCBModes::getAndruavModeFromPX4SubMode(mode, autopilot_type);

        case MAV_AUTOPILOT::MAV_AUTOPILOT_GENERIC:
        default:
            return CFCBModes::getAndruavModeFromArdupilotSubMode(mode, autopilot_type);
    };
}

ANDRUAV_UNIT_MODE CFCBModes::getAndruavModeFromPX4PlaneMode(const uint32_t &mode, const MAV_AUTOPILOT autopilot_type)
{

    return VEHICLE_MODE_UNKNOWN;
}

ANDRUAV_UNIT_MODE CFCBModes::getAndruavModeFromArdupilotPlaneMode(const uint32_t &mode, const MAV_AUTOPILOT autopilot_type)
{
    switch (mode)
    {
    case PLANE_MODE_MANUAL:
        return VEHICLE_MODE_MANUAL;
    case PLANE_MODE_ACRO:
        return VEHICLE_MODE_ACRO;
    case PLANE_MODE_STABILIZE:
        return VEHICLE_MODE_STABILIZE;
    case PLANE_MODE_TRAINING:
        return VEHICLE_MODE_UNKNOWN;
    case PLANE_MODE_FLY_BY_WIRE_A:
        return VEHICLE_MODE_FBWA;
    case PLANE_MODE_FLY_BY_WIRE_B:
        return VEHICLE_MODE_FBWB;
    case PLANE_MODE_CRUISE:
        return VEHICLE_MODE_CRUISE;
    case PLANE_MODE_CIRCLE:
        return VEHICLE_MODE_CIRCLE;
    case PLANE_MODE_AUTOTUNE:
        return VEHICLE_MODE_UNKNOWN;
    case PLANE_MODE_AUTO:
        return VEHICLE_MODE_AUTO;
    case PLANE_MODE_RTL:
        return VEHICLE_MODE_RTL;
    case PLANE_MODE_LOITER:
        return VEHICLE_MODE_LOITER;
    case PLANE_MODE_TAKEOFF:
        return VEHICLE_MODE_TAKEOFF;
    case PLANE_MODE_AVOID_ADSB:
        return VEHICLE_MODE_UNKNOWN;
    case PLANE_MODE_GUIDED:
        return VEHICLE_MODE_GUIDED;
    case PLANE_MODE_INITIALIZING:
        return VEHICLE_MODE_INITALIZING;
    case PLANE_MODE_QAUTOTUNE:
        return VEHICLE_MODE_UNKNOWN;
    case PLANE_MODE_QACRO:
        // fix this
        return VEHICLE_MODE_MANUAL;
    case PLANE_MODE_QRTL:
        return VEHICLE_MODE_QRTL;
    case PLANE_MODE_QLAND:
        return VEHICLE_MODE_QLAND;
    case PLANE_MODE_QSTABILIZE:
        return VEHICLE_MODE_QSTABILIZE;
    case PLANE_MODE_QLOITER:
        return VEHICLE_MODE_QLOITER;
    case PLANE_MODE_QHOVER:
        return VEHICLE_MODE_QHOVER;
    };

    return VEHICLE_MODE_UNKNOWN;
}

ANDRUAV_UNIT_MODE CFCBModes::getAndruavModeFromPX4CopterMode(const uint32_t &mode, const MAV_AUTOPILOT autopilot_type)
{
    // see get_px4_custom_mode at https://github.com/PX4/PX4-Autopilot/blob/fb71e7587c76ac883928870a23789e6b182d46b9/src/modules/mavlink/mavlink_messages.cpp
    px4_custom_mode custom_mode;
    custom_mode.data = mode;

    switch (custom_mode.main_mode)
    {
        case PX4_CUSTOM_MAIN_MODE_MANUAL:
            return VEHICLE_MODE_PX4_MANUAL;
        case PX4_CUSTOM_MAIN_MODE_ALTCTL:
            return VEHICLE_MODE_PX4_ALT_HOLD;
        case PX4_CUSTOM_MAIN_MODE_POSCTL:
            switch (custom_mode.sub_mode)
            {
            case PX4_CUSTOM_SUB_MODE_POSCTL_POSCTL:
                return VEHICLE_MODE_PX4_POSCTL_POSCTL;
            case PX4_CUSTOM_SUB_MODE_POSCTL_ORBIT:
                return VEHICLE_MODE_PX4_POSCTL_ORBIT;
            }
            return VEHICLE_MODE_UNKNOWN;
        case PX4_CUSTOM_MAIN_MODE_AUTO:
            switch (custom_mode.sub_mode)
            {
            case PX4_CUSTOM_SUB_MODE_AUTO_MISSION:
                return VEHICLE_MODE_PX4_AUTO_MISSION;

            case PX4_CUSTOM_SUB_MODE_AUTO_LOITER:
                return VEHICLE_MODE_PX4_AUTO_HOLD;

            case PX4_CUSTOM_SUB_MODE_AUTO_RTL:
                return VEHICLE_MODE_PX4_AUTO_RTL;

            case PX4_CUSTOM_SUB_MODE_AUTO_LAND:
                return VEHICLE_MODE_PX4_AUTO_LAND;

            case PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET:
                return VEHICLE_MODE_PX4_AUTO_FOLLOW_TARGET;

            case PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND:
                return VEHICLE_MODE_PX4_AUTO_LAND;
                       
            case PX4_CUSTOM_SUB_MODE_AUTO_VTOL_TAKEOFF:
                return VEHICLE_MODE_PX4_AUTO_VTOL_TAKEOFF;
                       
            case PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF:
                return VEHICLE_MODE_PX4_AUTO_TAKEOFF;
            };
            return VEHICLE_MODE_UNKNOWN;
        case PX4_CUSTOM_MAIN_MODE_ACRO:
            return VEHICLE_MODE_PX4_ACRO;
        case PX4_CUSTOM_MAIN_MODE_OFFBOARD:
            return VEHICLE_MODE_PX4_OFF_BORAD;
        case PX4_CUSTOM_MAIN_MODE_STABILIZED:
            return VEHICLE_MODE_PX4_STABILIZE;
        case PX4_CUSTOM_MAIN_MODE_RATTITUDE_LEGACY:
            return VEHICLE_MODE_PX4_RATTITUDE;
        case PX4_CUSTOM_MAIN_MODE_SIMPLE:
            return VEHICLE_MODE_UNKNOWN;
    };

    return VEHICLE_MODE_UNKNOWN;
}

ANDRUAV_UNIT_MODE CFCBModes::getAndruavModeFromArdupilotCopterMode(const uint32_t &mode, const MAV_AUTOPILOT autopilot_type)
{
    switch (mode)
    {
        case COPTER_MODE_STABILIZE:
            return VEHICLE_MODE_STABILIZE;
        case COPTER_MODE_ACRO:
            return VEHICLE_MODE_ACRO;
        case COPTER_MODE_ALT_HOLD:
            return VEHICLE_MODE_ALT_HOLD;
        case COPTER_MODE_AUTO:
            return VEHICLE_MODE_AUTO;
        case COPTER_MODE_GUIDED:
            return VEHICLE_MODE_GUIDED;
        case COPTER_MODE_LOITER:
            return VEHICLE_MODE_LOITER;
        case COPTER_MODE_RTL:
            return VEHICLE_MODE_RTL;
        case COPTER_MODE_CIRCLE:
            return VEHICLE_MODE_CIRCLE;
        case COPTER_MODE_LAND:
            return VEHICLE_MODE_LAND;
        case COPTER_MODE_DRIFT:
            return VEHICLE_MODE_UNKNOWN;
        case COPTER_MODE_SPORT:
            return VEHICLE_MODE_UNKNOWN;
        case COPTER_MODE_FLIP:
            return VEHICLE_MODE_UNKNOWN;
        case COPTER_MODE_AUTOTUNE:
            return VEHICLE_MODE_UNKNOWN;
        case COPTER_MODE_POSHOLD:
            return VEHICLE_MODE_POS_HOLD;
        case COPTER_MODE_BRAKE:
            return VEHICLE_MODE_BRAKE;
        case COPTER_MODE_THROW:
            return VEHICLE_MODE_UNKNOWN;
        case COPTER_MODE_AVOID_ADSB:
            return VEHICLE_MODE_UNKNOWN;
        case COPTER_MODE_GUIDED_NOGPS:
            return VEHICLE_MODE_UNKNOWN;
        case COPTER_MODE_SMART_RTL:
            return VEHICLE_MODE_SMART_RTL;
        case COPTER_MODE_FLOWHOLD:
            return VEHICLE_MODE_UNKNOWN;
        case COPTER_MODE_FOLLOW:
            return VEHICLE_MODE_FOLLOW_ME;
        case COPTER_MODE_ZIGZAG:
            return VEHICLE_MODE_UNKNOWN;
        case COPTER_MODE_SYSTEMID:
            return VEHICLE_MODE_INITALIZING;
        case COPTER_MODE_AUTOROTATE:
            return VEHICLE_MODE_UNKNOWN;
    };

    return VEHICLE_MODE_UNKNOWN;
}

ANDRUAV_UNIT_MODE CFCBModes::getAndruavModeFromPX4RoverMode(const uint32_t &mode, const MAV_AUTOPILOT autopilot_type)
{

    return VEHICLE_MODE_UNKNOWN;
}

ANDRUAV_UNIT_MODE CFCBModes::getAndruavModeFromArdupilotRoverMode(const uint32_t &mode, const MAV_AUTOPILOT autopilot_type)
{
    switch (mode)
    {
        case ROVER_MODE_MANUAL:
            return VEHICLE_MODE_MANUAL;
        case ROVER_MODE_ACRO:
            /*
                the ACRO_TURN_RATE parameter controls the maximum turn rate the user’s steering stick can request.
                The turn rate varies linearly from zero to ACRO_TURN_RATE as the RC input varies from neutral to full deflection. Once the input returns to neutral,
                the vehicle will attempt to hold heading, compensating for external influences, ie. “heading hold”.
            */
            return VEHICLE_MODE_ACRO;
        case ROVER_MODE_STEERING:
            return VEHICLE_MODE_STABILIZE;
        case ROVER_MODE_HOLD:
            return VEHICLE_MODE_BRAKE;
        case ROVER_MODE_LOITER:
            return VEHICLE_MODE_LOITER;
        case ROVER_MODE_FOLLOW:
            return VEHICLE_MODE_FOLLOW_ME;
        case ROVER_MODE_SIMPLE:
            return VEHICLE_MODE_STABILIZE;
        case ROVER_MODE_AUTO:
            return VEHICLE_MODE_AUTO;
        case ROVER_MODE_RTL:
            return VEHICLE_MODE_RTL;
        case ROVER_MODE_SMART_RTL:
            return VEHICLE_MODE_SMART_RTL;
        case ROVER_MODE_GUIDED:
            return VEHICLE_MODE_GUIDED;
        case ROVER_MODE_INITIALIZING:
            return VEHICLE_MODE_INITALIZING;
    };

    return VEHICLE_MODE_UNKNOWN;
}

ANDRUAV_UNIT_MODE CFCBModes::getAndruavModeFromPX4SubMode(const uint32_t &mode, const MAV_AUTOPILOT autopilot_type)
{

    return VEHICLE_MODE_UNKNOWN;
}

ANDRUAV_UNIT_MODE CFCBModes::getAndruavModeFromArdupilotSubMode(const uint32_t &mode, const MAV_AUTOPILOT autopilot_type)
{
    
    switch (mode)
    {
        
        case SUB_MODE_MANUAL:
            return VEHICLE_MODE_MANUAL;
        case SUB_MODE_ACRO:
            return VEHICLE_MODE_ACRO;
        case SUB_MODE_STABILIZE:
            return VEHICLE_MODE_STABILIZE;
        case SUB_MODE_ALT_HOLD:
            return VEHICLE_MODE_ALT_HOLD;
        case SUB_MODE_GUIDED:
            return VEHICLE_MODE_GUIDED;
        case SUB_MODE_AUTO:
            return VEHICLE_MODE_AUTO;
        case SUB_MODE_CIRCLE:
            return VEHICLE_MODE_CIRCLE;
        case SUB_MODE_SURFACE:
            return VEHICLE_MODE_SURFACE;
        case SUB_MODE_POSHOLD:
            return VEHICLE_MODE_POS_HOLD;
    };

    return VEHICLE_MODE_UNKNOWN;
}

/**
 * Converts Andruav Modes to ArduPilot mode.
 * */
void CFCBModes::getArduPilotMode(const int &andruav_unit_mode, const int &andruav_unit_type, uint32_t &mode, uint32_t &custom_mode, uint32_t &custom_sub_mode)
{
    mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    custom_sub_mode = 0;
                   
    switch (andruav_unit_mode)
    {
        // PX4-Related Modes
        case VEHICLE_MODE_PX4_MANUAL:
            custom_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
            return;
        case VEHICLE_MODE_PX4_ALT_HOLD:
            custom_mode = PX4_CUSTOM_MAIN_MODE_ALTCTL;
            return;
        case VEHICLE_MODE_PX4_AUTO_MISSION:
            custom_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
            custom_sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
            return;
        case VEHICLE_MODE_PX4_AUTO_HOLD:
            custom_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
            custom_sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
            return;
        case VEHICLE_MODE_PX4_AUTO_RTL:
            custom_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
            custom_sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTL;
            return;
        case VEHICLE_MODE_PX4_AUTO_LAND:
            custom_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
            custom_sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LAND;
            return;
        case VEHICLE_MODE_PX4_AUTO_FOLLOW_TARGET:
            custom_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
            custom_sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET;
            return;
        case VEHICLE_MODE_PX4_AUTO_PRECLAND:
            custom_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
            custom_sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND;
            return;
        case VEHICLE_MODE_PX4_AUTO_VTOL_TAKEOFF:
            custom_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
            custom_sub_mode = VEHICLE_MODE_PX4_AUTO_VTOL_TAKEOFF;
            return;
        case VEHICLE_MODE_PX4_AUTO_TAKEOFF:
            custom_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
            custom_sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF;
            return;
        case VEHICLE_MODE_PX4_ACRO:
            custom_mode = PX4_CUSTOM_MAIN_MODE_ACRO;
            return;
        case VEHICLE_MODE_PX4_STABILIZE:
            custom_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
            return;
        case VEHICLE_MODE_PX4_OFF_BORAD:
            custom_mode = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
            return;
        case VEHICLE_MODE_PX4_RATTITUDE:
            custom_mode = PX4_CUSTOM_MAIN_MODE_RATTITUDE_LEGACY;
            return;
        case VEHICLE_MODE_PX4_POSCTL_POSCTL:
            custom_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
            custom_sub_mode = PX4_CUSTOM_SUB_MODE_POSCTL_POSCTL;
            return;
        case VEHICLE_MODE_PX4_POSCTL_ORBIT:
            custom_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
            custom_sub_mode = PX4_CUSTOM_SUB_MODE_POSCTL_POSCTL;
            return;

        // Ardupilot-Related Modes
        case VEHICLE_MODE_RTL:
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    custom_mode = COPTER_MODE_RTL;
                    return;

                case VEHICLE_TYPE_PLANE:
                    custom_mode = PLANE_MODE_RTL;
                    return;

                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                    custom_mode = ROVER_MODE_RTL;
                    return;

                case VEHICLE_TYPE_SUBMARINE:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            };

            break;

        case VEHICLE_MODE_FOLLOW_ME:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    custom_mode = COPTER_MODE_FOLLOW;
                    return;

                case VEHICLE_TYPE_PLANE:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                    custom_mode = ROVER_MODE_FOLLOW;
                    return;

                case VEHICLE_TYPE_SUBMARINE:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;

        case VEHICLE_MODE_AUTO:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    custom_mode = COPTER_MODE_AUTO;
                    return;

                case VEHICLE_TYPE_PLANE:
                    custom_mode = PLANE_MODE_AUTO;
                    return;

                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                    custom_mode = ROVER_MODE_AUTO;
                    return;

                case VEHICLE_TYPE_SUBMARINE:
                    custom_mode = SUB_MODE_AUTO;
                    return;

                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;

        case VEHICLE_MODE_STABILIZE:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    custom_mode = COPTER_MODE_STABILIZE;
                    return;

                case VEHICLE_TYPE_PLANE:
                    custom_mode = PLANE_MODE_STABILIZE;
                    return;

                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                    custom_mode = ROVER_MODE_STEERING;
                    return;

                case VEHICLE_TYPE_SUBMARINE:
                    custom_mode = SUB_MODE_STABILIZE;
                    return;

                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;

        case VEHICLE_MODE_ALT_HOLD:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    custom_mode = COPTER_MODE_ALT_HOLD;
                    return;

                case VEHICLE_TYPE_PLANE:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                case VEHICLE_TYPE_SUBMARINE:
                    custom_mode = SUB_MODE_ALT_HOLD;
                    return;

                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;

        case VEHICLE_MODE_MANUAL:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                case VEHICLE_TYPE_PLANE:
                    custom_mode = PLANE_MODE_MANUAL;
                    return;

                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                    custom_mode = ROVER_MODE_MANUAL;
                    return;

                case VEHICLE_TYPE_SUBMARINE:
                    custom_mode = SUB_MODE_MANUAL;
                    return;
                    break;

                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }
            break;

        case VEHICLE_MODE_ACRO:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    custom_mode = COPTER_MODE_ACRO;
                    return;

                case VEHICLE_TYPE_PLANE:
                    custom_mode = PLANE_MODE_ACRO;
                    return;

                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                    custom_mode = ROVER_MODE_ACRO;
                    return;

                case VEHICLE_TYPE_SUBMARINE:
                    custom_mode = SUB_MODE_ACRO;
                    return;
                    break;

                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;

        case VEHICLE_MODE_GUIDED:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    custom_mode = COPTER_MODE_GUIDED;
                    return;

                case VEHICLE_TYPE_PLANE:
                    custom_mode = PLANE_MODE_GUIDED;
                    return;

                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                    custom_mode = ROVER_MODE_GUIDED;
                    return;

                case VEHICLE_TYPE_SUBMARINE:
                    custom_mode = SUB_MODE_GUIDED;
                    return;

                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;

        case VEHICLE_MODE_LOITER:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    custom_mode = COPTER_MODE_LOITER;
                    return;

                case VEHICLE_TYPE_PLANE:
                    custom_mode = PLANE_MODE_LOITER;
                    return;

                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                    custom_mode = ROVER_MODE_LOITER;
                    return;

                case VEHICLE_TYPE_SUBMARINE:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;

        case VEHICLE_MODE_POS_HOLD:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    custom_mode = COPTER_MODE_POSHOLD;
                    return;

                case VEHICLE_TYPE_PLANE:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                case VEHICLE_TYPE_SUBMARINE:
                    custom_mode = SUB_MODE_POSHOLD;
                    return;

                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;

        case VEHICLE_MODE_LAND:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    custom_mode = COPTER_MODE_LAND;
                    return;

                case VEHICLE_TYPE_PLANE:
                    custom_mode = PLANE_MODE_QLAND;
                    return;

                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                case VEHICLE_TYPE_SUBMARINE:
                    custom_mode = VEHICLE_MODE_SURFACE;
                    return;

                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;

        case VEHICLE_MODE_CIRCLE:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    custom_mode = COPTER_MODE_CIRCLE;
                    return;

                case VEHICLE_TYPE_PLANE:
                    custom_mode = PLANE_MODE_CIRCLE;
                    return;

                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                case VEHICLE_TYPE_SUBMARINE:
                    custom_mode = SUB_MODE_CIRCLE;
                    return;

                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;

        case VEHICLE_MODE_FBWA:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                case VEHICLE_TYPE_PLANE:
                    custom_mode = PLANE_MODE_FLY_BY_WIRE_A;
                    return;

                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                case VEHICLE_TYPE_SUBMARINE:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;

        case VEHICLE_MODE_CRUISE:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                case VEHICLE_TYPE_PLANE:
                    custom_mode = PLANE_MODE_CRUISE;
                    return;

                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                case VEHICLE_TYPE_SUBMARINE:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;

        case VEHICLE_MODE_FBWB:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                case VEHICLE_TYPE_PLANE:
                    custom_mode = PLANE_MODE_FLY_BY_WIRE_B;
                    return;

                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                case VEHICLE_TYPE_SUBMARINE:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;

        case VEHICLE_MODE_BRAKE:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    custom_mode = COPTER_MODE_BRAKE;
                    return;

                case VEHICLE_TYPE_PLANE:
                    custom_mode = PLANE_MODE_LOITER;
                    return;

                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                    custom_mode = ROVER_MODE_HOLD;
                    return;

                case VEHICLE_TYPE_SUBMARINE:
                    custom_mode = SUB_MODE_POSHOLD;
                    return;

                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;

        case VEHICLE_MODE_SMART_RTL:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    custom_mode = COPTER_MODE_SMART_RTL;
                    return;

                case VEHICLE_TYPE_PLANE:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                    custom_mode = ROVER_MODE_SMART_RTL;
                    return;

                case VEHICLE_TYPE_SUBMARINE:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;

        case VEHICLE_MODE_TAKEOFF:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                case VEHICLE_TYPE_PLANE:
                    custom_mode = PLANE_MODE_TAKEOFF;
                    return;

                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                case VEHICLE_TYPE_SUBMARINE:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;

        case VEHICLE_MODE_SURFACE:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                case VEHICLE_TYPE_PLANE:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                case VEHICLE_TYPE_SUBMARINE:
                    custom_mode = SUB_MODE_SURFACE;
                    return;

                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;

        case VEHICLE_MODE_INITALIZING:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    custom_mode = COPTER_MODE_SYSTEMID;
                    return;

                case VEHICLE_TYPE_PLANE:
                    custom_mode = PLANE_MODE_INITIALIZING;
                    return;

                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                case VEHICLE_TYPE_SUBMARINE:
                    custom_mode = E_UNDEFINED_MODE;
                    return;

                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;

        case VEHICLE_MODE_QRTL:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_PLANE:
                    custom_mode = PLANE_MODE_QRTL;
                    return ;
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                case VEHICLE_TYPE_SUBMARINE:
                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;
            
        case VEHICLE_MODE_QLAND:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_PLANE:
                    custom_mode = PLANE_MODE_QLAND;
                    return ;
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                case VEHICLE_TYPE_SUBMARINE:
                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;
            
        case VEHICLE_MODE_QSTABILIZE:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_PLANE:
                    custom_mode = PLANE_MODE_QSTABILIZE;
                    return ;
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                case VEHICLE_TYPE_SUBMARINE:
                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;
            
        case VEHICLE_MODE_QLOITER:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_PLANE:
                    custom_mode = PLANE_MODE_QLOITER;
                    return ;
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                case VEHICLE_TYPE_SUBMARINE:
                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;
            
        case VEHICLE_MODE_QHOVER:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_PLANE:
                    custom_mode = PLANE_MODE_QHOVER;
                    return ;
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                case VEHICLE_TYPE_SUBMARINE:
                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;
            
        case VEHICLE_MODE_UNKNOWN:
            mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                case VEHICLE_TYPE_PLANE:
                case VEHICLE_TYPE_ROVER:
                case VEHICLE_TYPE_BOAT:
                case VEHICLE_TYPE_SUBMARINE:
                default:
                    custom_mode = E_UNDEFINED_MODE;
                    return;
            }

            break;
    }

    custom_mode = E_UNDEFINED_MODE;
    return;
}
