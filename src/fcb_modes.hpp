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
            
           
    };
}
}
#endif