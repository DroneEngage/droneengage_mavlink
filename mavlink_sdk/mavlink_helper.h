#ifndef MAVLINK_HELPER_H_
#define MAVLINK_HELPER_H_

#include <iostream>
#include <string>
#include <common/mavlink.h>

namespace mavlinksdk
{
    typedef enum FIRMWARE_TYPE
    {
        FIRMWARE_TYPE_ARDU_PLANE=0,
        FIRMWARE_TYPE_ARDU_COPTER=1,
        FIRMWARE_TYPE_ARDU_ROVER=2,
        FIRMWARE_TYPE_ARDU_SUB=3,
        FIRMWARE_TYPE_UNKNOWN = 9999,
    } FIRMWARE_TYPE;


    


    class CMavlinkHelper 
    {
        private: 
            // Disallow creating an instance of this class.
            CMavlinkHelper(){};       

        public:
            static std::string getACKError (const int result);        
            
            static mavlinksdk::FIRMWARE_TYPE getFirmewareType (const int mav_type, const int autopilot_type);

            static std::string getMode (int mode, int autopilot_type);

            static std::string getPlaneMode (int mode);

            static std::string getCopterMode (int mode);

            static std::string getSubMode (int mode);

            static std::string getRoverMode (int mode);
    };

}

#endif //MAVLINK_HELPER_H_