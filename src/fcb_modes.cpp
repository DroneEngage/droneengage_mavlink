#include <ardupilotmega/mavlink.h>
#include <common/mavlink.h>
#include <mavlink_helpers.h>
#include "fcb_modes.hpp"


/**
* Converts Ardupilot unit type to Andruav vehicle type.
* @param mav_type  mavlink_heartbeat_t.type (MAV_TYPE)
* @param autopilot_type mavlink_heartbeat_t.autopilot (MAV_AUTOPILOT)
**/
ANDRUAV_UNIT_TYPE uavos::fcb::CFCBModes::getAndruavVehicleType (const int mav_type, const int autopilot_type)
{
    if ((autopilot_type == MAV_AUTOPILOT_GENERIC) || (autopilot_type == MAV_AUTOPILOT_ARDUPILOTMEGA)) 
    {

        switch (mav_type)
        {
            case MAV_TYPE_QUADROTOR: /* Quadrotor | */
            case MAV_TYPE_HEXAROTOR: /* Hexarotor | */
            case MAV_TYPE_OCTOROTOR: /* Octorotor | */
                return ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_QUAD;
            
            case MAV_TYPE_TRICOPTER: /* Tricopter | */
                return ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_TRI;
            
            case MAV_TYPE_COAXIAL: /* Coaxial helicopter | */
            case MAV_TYPE_HELICOPTER: /* Normal helicopter with tail rotor. | */
                return ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_HELI;
    
            
            case MAV_TYPE_GROUND_ROVER: /* Ground rover | */
            case MAV_TYPE_SURFACE_BOAT: /* Surface vessel, boat, ship | */
                //TODO: Please define mode for boats
                return ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_ROVER;

            case MAV_TYPE_SUBMARINE: /* Submarine | */
                return ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_SUBMARINE;

            case MAV_TYPE_FLAPPING_WING: /* Flapping wing | */
            case MAV_TYPE_VTOL_DUOROTOR: /* Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter. | */
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
        
    }

    return ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_UNKNOWN;
}

/**
 * Converts ardupilot modes into Andruav modes.
 * Andruav modes are flat i.e. it is not categoriezed by vehicles.
 * So same mode means the same accross all vehicles.
 * */
ANDRUAV_UNIT_MODE uavos::fcb::CFCBModes::getAndruavMode (int mode, int andruav_vehicle_type)
{

    switch (andruav_vehicle_type)
    {
        case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_QUAD:
        case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_TRI:
        case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_HELI:
            return uavos::fcb::CFCBModes::getAndruavModeFromArdupilotCopterMode (mode);
        break;
        
        case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_PLANE:
            return uavos::fcb::CFCBModes::getAndruavModeFromArdupilotPlaneMode (mode);
        break;

        case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_ROVER:
            return uavos::fcb::CFCBModes::getAndruavModeFromArdupilotRoverMode (mode);
        break;


        case ANDRUAV_UNIT_TYPE::VEHICLE_TYPE_SUBMARINE:
            return uavos::fcb::CFCBModes::getAndruavModeFromArdupilotSubMode (mode);
        break;

    }
}



ANDRUAV_UNIT_MODE uavos::fcb::CFCBModes::getAndruavModeFromArdupilotPlaneMode (const int & mode)
{
    switch (mode)
    {
        case PLANE_MODE_MANUAL:
            return VEHICLE_MODE_MANUAL;
        case PLANE_MODE_CIRCLE:
            return VEHICLE_MODE_CIRCLE;
        case PLANE_MODE_STABILIZE:
            return VEHICLE_MODE_STABILIZE;
        case PLANE_MODE_TRAINING:
            return VEHICLE_MODE_UNKNOWN;
        case PLANE_MODE_ACRO:
            //TODO: fix me add Acro
            return VEHICLE_MODE_MANUAL;
        case PLANE_MODE_FLY_BY_WIRE_A:
            return VEHICLE_MODE_FBWA;
        case PLANE_MODE_FLY_BY_WIRE_B:
            return VEHICLE_MODE_FBWB;
        case PLANE_MODE_CRUISE:
            return VEHICLE_MODE_CRUISE;
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



ANDRUAV_UNIT_MODE uavos::fcb::CFCBModes::getAndruavModeFromArdupilotCopterMode (const int & mode)
{
    switch (mode)
    {
        case COPTER_MODE_STABILIZE:
            return VEHICLE_MODE_STABILIZE;
        case COPTER_MODE_ACRO:
            return VEHICLE_MODE_MANUAL;
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



ANDRUAV_UNIT_MODE uavos::fcb::CFCBModes::getAndruavModeFromArdupilotRoverMode (const int & mode)
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
           // TODO: please reintroduce ACRO again
            return VEHICLE_MODE_MANUAL;
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
    }

    return VEHICLE_MODE_UNKNOWN;
}



ANDRUAV_UNIT_MODE uavos::fcb::CFCBModes::getAndruavModeFromArdupilotSubMode (const int & mode)
{
    switch (mode)
    {
        case SUB_MODE_STABILIZE:
            return VEHICLE_MODE_STABILIZE;
        case SUB_MODE_ACRO:
            return VEHICLE_MODE_MANUAL;
        case SUB_MODE_ALT_HOLD:
            return VEHICLE_MODE_ALT_HOLD;
        case SUB_MODE_AUTO:
            return VEHICLE_MODE_AUTO;
        case SUB_MODE_GUIDED:
            return VEHICLE_MODE_GUIDED;
        case SUB_MODE_CIRCLE:
            return VEHICLE_MODE_CIRCLE;
        case SUB_MODE_SURFACE:
            return VEHICLE_MODE_SURFACE;
        case SUB_MODE_POSHOLD:
            return VEHICLE_MODE_POS_HOLD;
        case SUB_MODE_MANUAL:
            //TODO: MANUAL & ACRO for SUB .... Why is that ? isnt manual is acro and vice versa???? please check
            return VEHICLE_MODE_MANUAL;
    }

    return VEHICLE_MODE_UNKNOWN;
}



/**
 * Converts Andruav Modes to ArduPilot mode.
 * */
int uavos::fcb::CFCBModes::getArduPilotMode(const int& andruav_unit_mode, const int& andruav_unit_type)
{
    switch (andruav_unit_mode)
    {
        case VEHICLE_MODE_RTL:     
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    return COPTER_MODE_RTL;

                case VEHICLE_TYPE_PLANE:
                    return PLANE_MODE_RTL;

                case VEHICLE_TYPE_ROVER:
                    return ROVER_MODE_RTL;

                case VEHICLE_TYPE_SUBMARINE:
                    return E_UNDEFINED_MODE;

                default:
                return E_UNDEFINED_MODE;
            }

            break;
            
        case VEHICLE_MODE_FOLLOW_ME:     
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    return COPTER_MODE_FOLLOW;

                case VEHICLE_TYPE_PLANE:
                    return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_ROVER:
                    return ROVER_MODE_FOLLOW;

                case VEHICLE_TYPE_SUBMARINE:
                    return E_UNDEFINED_MODE;

                default:
                return E_UNDEFINED_MODE;
            }

            break;

        case VEHICLE_MODE_AUTO:     
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    return COPTER_MODE_AUTO;

                case VEHICLE_TYPE_PLANE:
                    return PLANE_MODE_AUTO;

                case VEHICLE_TYPE_ROVER:
                    return ROVER_MODE_AUTO;

                case VEHICLE_TYPE_SUBMARINE:
                    return SUB_MODE_AUTO;

                default:
                return E_UNDEFINED_MODE;
            }

            break;

        case VEHICLE_MODE_STABILIZE:     
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    return COPTER_MODE_STABILIZE;

                case VEHICLE_TYPE_PLANE:
                    return PLANE_MODE_STABILIZE;
                
                case VEHICLE_TYPE_ROVER:
                    return ROVER_MODE_STEERING;
                
                case VEHICLE_TYPE_SUBMARINE:
                    return SUB_MODE_STABILIZE;

                default:
                return E_UNDEFINED_MODE;
            }

            break;

        case VEHICLE_MODE_ALT_HOLD:     
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    return COPTER_MODE_ALT_HOLD;

                case VEHICLE_TYPE_PLANE:
                    return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_ROVER:
                    return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_SUBMARINE:
                    return SUB_MODE_ALT_HOLD;

                default:
                return E_UNDEFINED_MODE;
            }

            break;

        case VEHICLE_MODE_MANUAL:     
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    return COPTER_MODE_ACRO;

                case VEHICLE_TYPE_PLANE:
                    return PLANE_MODE_MANUAL;

                case VEHICLE_TYPE_ROVER:
                    return ROVER_MODE_MANUAL;

                case VEHICLE_TYPE_SUBMARINE:
                    return SUB_MODE_MANUAL;
                break;

                default:
                return E_UNDEFINED_MODE;
            }

            break;

        case VEHICLE_MODE_GUIDED:     
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    return COPTER_MODE_GUIDED;

                case VEHICLE_TYPE_PLANE:
                    return PLANE_MODE_GUIDED;

                case VEHICLE_TYPE_ROVER:
                    return ROVER_MODE_GUIDED;

                case VEHICLE_TYPE_SUBMARINE:
                    return SUB_MODE_GUIDED;

                default:
                return E_UNDEFINED_MODE;
            }

            break;

        case VEHICLE_MODE_LOITER:     
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    return COPTER_MODE_LOITER;

                case VEHICLE_TYPE_PLANE:
                    return PLANE_MODE_LOITER;

                case VEHICLE_TYPE_ROVER:
                    return ROVER_MODE_LOITER;

                case VEHICLE_TYPE_SUBMARINE:
                    return E_UNDEFINED_MODE;

                default:
                return E_UNDEFINED_MODE;
            }

            break;

        case VEHICLE_MODE_POS_HOLD:     
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    return COPTER_MODE_POSHOLD;

                case VEHICLE_TYPE_PLANE:
                    return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_ROVER:
                    return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_SUBMARINE:
                    return SUB_MODE_POSHOLD;

                default:
                return E_UNDEFINED_MODE;
            }

            break;

        case VEHICLE_MODE_LAND:     
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    return COPTER_MODE_LAND;

                case VEHICLE_TYPE_PLANE:
                    return PLANE_MODE_QLAND;

                case VEHICLE_TYPE_ROVER:
                    return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_SUBMARINE:
                    return VEHICLE_MODE_SURFACE;

                default:
                return E_UNDEFINED_MODE;
            }

            break;

        case VEHICLE_MODE_CIRCLE:     
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    return COPTER_MODE_CIRCLE;

                case VEHICLE_TYPE_PLANE:
                    return PLANE_MODE_CIRCLE;

                case VEHICLE_TYPE_ROVER:
                    return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_SUBMARINE:
                    return SUB_MODE_CIRCLE;

                default:
                return E_UNDEFINED_MODE;
            }

            break;

        case VEHICLE_MODE_FBWA:     
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_PLANE:
                    return PLANE_MODE_FLY_BY_WIRE_A;

                case VEHICLE_TYPE_ROVER:
                    return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_SUBMARINE:
                    return E_UNDEFINED_MODE;

                default:
                return E_UNDEFINED_MODE;
            }

            break;

        case VEHICLE_MODE_CRUISE:     
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_PLANE:
                    return PLANE_MODE_CRUISE;

                case VEHICLE_TYPE_ROVER:
                    return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_SUBMARINE:
                    return E_UNDEFINED_MODE;

                default:
                return E_UNDEFINED_MODE;
            }

            break;

        case VEHICLE_MODE_FBWB:     
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_PLANE:
                    return PLANE_MODE_FLY_BY_WIRE_B;

                case VEHICLE_TYPE_ROVER:
                return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_SUBMARINE:
                return E_UNDEFINED_MODE;

                default:
                return E_UNDEFINED_MODE;
            }

            break;

        case VEHICLE_MODE_BRAKE:     
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    return COPTER_MODE_BRAKE;

                case VEHICLE_TYPE_PLANE:
                    return PLANE_MODE_LOITER;

                case VEHICLE_TYPE_ROVER:
                    return ROVER_MODE_HOLD;

                case VEHICLE_TYPE_SUBMARINE:
                    return SUB_MODE_POSHOLD;

                default:
                return E_UNDEFINED_MODE;
            }

            break;

        case VEHICLE_MODE_SMART_RTL:     
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    return COPTER_MODE_SMART_RTL;

                case VEHICLE_TYPE_PLANE:
                    return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_ROVER:
                    return ROVER_MODE_SMART_RTL;

                case VEHICLE_TYPE_SUBMARINE:
                    return E_UNDEFINED_MODE;

                default:
                return E_UNDEFINED_MODE;
            }

            break;

        case VEHICLE_MODE_TAKEOFF:     
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_PLANE:
                    return PLANE_MODE_TAKEOFF;

                case VEHICLE_TYPE_ROVER:
                    return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_SUBMARINE:
                    return E_UNDEFINED_MODE;

                default:
                return E_UNDEFINED_MODE;
            }

            break;

        case VEHICLE_MODE_SURFACE:     
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_PLANE:
                    return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_ROVER:
                    return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_SUBMARINE:
                    return SUB_MODE_SURFACE;

                default:
                return E_UNDEFINED_MODE;
            }

            break;

        case VEHICLE_MODE_INITALIZING:     
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    return COPTER_MODE_SYSTEMID;

                case VEHICLE_TYPE_PLANE:
                    return PLANE_MODE_INITIALIZING;

                case VEHICLE_TYPE_ROVER:
                    return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_SUBMARINE:
                    return E_UNDEFINED_MODE;

                default:
                return E_UNDEFINED_MODE;
            }

            break;

        case VEHICLE_MODE_UNKNOWN:     
            switch (andruav_unit_type)
            {
                case VEHICLE_TYPE_HELI:
                case VEHICLE_TYPE_TRI:
                case VEHICLE_TYPE_QUAD:
                    return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_PLANE:
                    return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_ROVER:
                    return E_UNDEFINED_MODE;

                case VEHICLE_TYPE_SUBMARINE:
                    return E_UNDEFINED_MODE;

                default:
                return E_UNDEFINED_MODE;
            }

            break;
    }

    return E_UNDEFINED_MODE;
}



           