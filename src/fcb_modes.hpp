#ifndef FCB_MODES_H_
#define FCB_MODES_H_

#include "defines.hpp"

namespace uavos
{
namespace fcb
{

    
 
 /**
  * @brief 
  * This class handles tmapping between Andruav Modes and Ardupilot modes.
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
            
            
            static ANDRUAV_UNIT_TYPE getAndruavVehicleType (const int mav_type, const int autopilot_type);
            static ANDRUAV_UNIT_MODE getAndruavMode (int mode, int andruav_vehicle_type);

            static int getArduPilotMode (const int& andruav_unit_mode, const int& andruav_unit_type); 

        private:
            static ANDRUAV_UNIT_MODE getAndruavModeFromArdupilotPlaneMode (const int & mode);
            static ANDRUAV_UNIT_MODE getAndruavModeFromArdupilotCopterMode (const int & mode);
            static ANDRUAV_UNIT_MODE getAndruavModeFromArdupilotRoverMode (const int & mode);
            static ANDRUAV_UNIT_MODE getAndruavModeFromArdupilotSubMode (const int & mode);


           
    };
}
}
#endif