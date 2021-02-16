#ifndef MAVLINK_COMMAND_H_
#define MAVLINK_COMMAND_H_

#include "mavlink_sdk.h"


namespace mavlinksdk
{

class CMavlinkCommand
{

    public:
        
        void doSetMode   (const int& mode);
        void doArmDisarm (const bool& arm, const bool& force);
            
        void setHome (const float& latitude, const float& longitude, const float& altitude);
        void setROI (const float& latitude, const float& longitude, const float& altitude);
        void resetROI ();
        void cmdTerminalFlight ();
        void changeAltitude (const float& altitude);
        

        

    protected:

        mavlinksdk::CMavlinkSDK& m_mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
};

}

#endif