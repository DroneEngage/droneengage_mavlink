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