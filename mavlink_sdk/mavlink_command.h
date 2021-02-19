#ifndef MAVLINK_COMMAND_H_
#define MAVLINK_COMMAND_H_

#include "mavlink_sdk.h"


namespace mavlinksdk
{

/**
 * @brief This class holds methods to do actions on vehicle -mostly-.
 * So this is Vehicle.API
 * 
 */
class CMavlinkCommand
{

    public:
        
        void doSetMode   (const int& mode);
        void doArmDisarm (const bool& arm, const bool& force);
            
        void setHome (const float& yaw, const float& latitude, const float& longitude, const float& altitude);
        void setROI (const float& latitude, const float& longitude, const float& altitude);
        void resetROI ();
        void cmdTerminalFlight ();
        void changeAltitude (const float& altitude);
        void takeOff (const float& altitude);
        void gotoGuidedPoint (const double& latitude, const double& longitude, const double& altitude);
        void setYawCondition (const double& target_angle, const double& turn_rate, const bool& is_clock_wise, const bool& is_relative);
        void setNavigationSpeed ( const int& speed_type, const double& speed, const double& throttle, const bool& is_relative);
        void loadWayPoints ();
        void clearWayPoints();
        void setCurrentMission (const int& mission_number);

    protected:
        void sendLongCommand (const uint16_t& command,
                const bool& confirmation = false,
                const float& param1 = 0.0f,
                const float& param2 = 0.0f,
                const float& param3 = 0.0f,
                const float& param4 = 0.0f,
                const float& param5 = 0.0f,
                const float& param6 = 0.0f,
                const float& param7 = 0.0f);

    protected:

        mavlinksdk::CMavlinkSDK& m_mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
};

}

#endif