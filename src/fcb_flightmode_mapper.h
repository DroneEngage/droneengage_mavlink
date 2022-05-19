#ifndef FLIGHTMODE_MAPPER_H_
#define FLIGHTMODE_MAPPER_H_

#include <iostream>
#include <string>

namespace uavos
{
namespace fcb
{
    typedef enum FIRMWARE_TYPE
    {
        FIRMWARE_TYPE_ARDU_PLANE=0,
        FIRMWARE_TYPE_ARDU_COPTER=1,
        FIRMWARE_TYPE_ARDU_ROVER=2,
        FIRMWARE_TYPE_ARDU_SUB=3,
        FIRMWARE_TYPE_UNKNOWN = 9999,
    } FIRMWARE_TYPE;


    


    class CFlightModeMapper 
    {
        private: 
            // Disallow creating an instance of this class.
            CFlightModeMapper(){};       

        public:

            static uavos::fcb::FIRMWARE_TYPE getFirmewareType (const int mav_type, const int autopilot_type);

            static std::string getMode (const int mode, int autopilot_type);

            static std::string getPlaneMode (const int mode, const int autopilot_type);

            static std::string getCopterMode (const int mode, const int autopilot_type);

            static std::string getSubMode (const int mode, const int autopilot_type);

            static std::string getRoverMode (const int mode, const int autopilot_type);
    };

}
}
#endif //FLIGHTMODE_MAPPER_H_