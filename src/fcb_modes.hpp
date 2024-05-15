#ifndef FCB_MODES_H_
#define FCB_MODES_H_

#include "defines.hpp"
#include <all/mavlink.h>

namespace de
{
namespace fcb
{

    
 
 /**
  * @brief This class handles tmapping between Andruav Modes and Ardupilot modes.
  * Andruav modes are global not related to a vehicle.
  * i.e. Guided mode is the same number for all vehicles. Although not all fields are allowable. 
  * 
  */
  class CFCBModes 
    {
        private: 
            // Disallow creating an instance of this class.
            CFCBModes(){};       

        public:
            
            
            static ANDRUAV_UNIT_TYPE getAndruavVehicleType (const int mav_type);
            static ANDRUAV_UNIT_MODE getAndruavMode (const uint32_t mode, const int andruav_vehicle_type, const MAV_AUTOPILOT autopilot_type);

            static void getArduPilotMode (const int& andruav_unit_mode, const int& andruav_unit_type, uint32_t& mode,  uint32_t& custom_mode, uint32_t &custom_sub_mode);
            static void getPX4Mode (const int& andruav_unit_mode, const int& andruav_unit_type, uint32_t& mode,  uint32_t& custom_mode); 
        private:
            
            static ANDRUAV_UNIT_MODE getAndruavModeFromMavlinkPlaneMode (const uint32_t& mode, const MAV_AUTOPILOT autopilot_type);
            static ANDRUAV_UNIT_MODE getAndruavModeFromMavlinkCopterMode (const uint32_t& mode, const MAV_AUTOPILOT autopilot_type);
            static ANDRUAV_UNIT_MODE getAndruavModeFromMavlinkRoverMode (const uint32_t& mode, const MAV_AUTOPILOT autopilot_type);
            static ANDRUAV_UNIT_MODE getAndruavModeFromMavlinkSubMode (const uint32_t& mode, const MAV_AUTOPILOT autopilot_type);


            static ANDRUAV_UNIT_MODE getAndruavModeFromArdupilotPlaneMode (const uint32_t& mode, const MAV_AUTOPILOT autopilot_type);
            static ANDRUAV_UNIT_MODE getAndruavModeFromArdupilotCopterMode (const uint32_t& mode, const MAV_AUTOPILOT autopilot_type);
            static ANDRUAV_UNIT_MODE getAndruavModeFromArdupilotRoverMode (const uint32_t& mode, const MAV_AUTOPILOT autopilot_type);
            static ANDRUAV_UNIT_MODE getAndruavModeFromArdupilotSubMode (const uint32_t& mode, const MAV_AUTOPILOT autopilot_type);

            static ANDRUAV_UNIT_MODE getAndruavModeFromPX4PlaneMode (const uint32_t& mode, const MAV_AUTOPILOT autopilot_type);
            static ANDRUAV_UNIT_MODE getAndruavModeFromPX4CopterMode (const uint32_t& mode, const MAV_AUTOPILOT autopilot_type);
            static ANDRUAV_UNIT_MODE getAndruavModeFromPX4RoverMode (const uint32_t& mode, const MAV_AUTOPILOT autopilot_type);
            static ANDRUAV_UNIT_MODE getAndruavModeFromPX4SubMode (const uint32_t& mode, const MAV_AUTOPILOT autopilot_type);


           
    };
}
}
#endif