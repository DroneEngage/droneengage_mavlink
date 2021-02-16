#ifndef FCB_MODES_H_
#define FCB_MODES_H_

#include "defines.hpp"

namespace uavos
{
namespace fcb
{

 class CFCBModes 
    {
        private: 
            // Disallow creating an instance of this class.
            CFCBModes(){};       

        public:
            
            
            static ANDRUAV_UNIT_TYPE getAndruavVehicleType (const int mav_type, const int autopilot_type);
            static ANDRUAV_UNIT_MODE getAndruavMode (int mode, int andruav_vehicle_type);
            static ANDRUAV_UNIT_MODE getAndruavModeFromArdupilotPlaneMode (const int & mode);
            static ANDRUAV_UNIT_MODE getAndruavModeFromArdupilotCopterMode (const int & mode);
            static ANDRUAV_UNIT_MODE getAndruavModeFromArdupilotRoverMode (const int & mode);
            static ANDRUAV_UNIT_MODE getAndruavModeFromArdupilotSubMode (const int & mode);

           
    };
}
}
#endif