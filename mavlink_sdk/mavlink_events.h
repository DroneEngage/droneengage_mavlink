#ifndef MAVLINK_CEVENTS_H_
#define MAVLINK_CEVENTS_H_
#include <iostream>
#include <common/common.h>

namespace mavlinksdk
{
    class CMavlinkEvents 
    {
        
        
        
        public: 
        
        // CCallBack_Vehicle Related
        virtual void OnHeartBeat_First (const mavlink_heartbeat_t& heartbeat)       {};
        virtual void OnArmed  (const bool armed)                                    {};
        virtual void OnFlying (const bool isFlying)                                 {};
        virtual void OnACK    (const int result, const std::string& result_msg)     {};
        virtual void OnStatusText (const std::string& status)                       {};
        virtual void OnModeChanges(const int mode_number, const int firmware_type)  {};

        // CCallBack_Communicator Related

        virtual void OnMessageReceived (mavlink_message_t& mavlink_message) {};
        virtual void OnConnected (const bool connected) {};
    };
}

#endif