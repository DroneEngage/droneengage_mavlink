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
        virtual void OnHeartBeat_Resumed (const mavlink_heartbeat_t& heartbeat)     {};
        virtual void OnArmed  (const bool& armed)                                    {};
        virtual void OnFlying (const bool& isFlying)                                 {};
        virtual void OnMissionACK (const int& result, const int& mission_type, const std::string& result_msg){}; 
        virtual void OnACK    (const int& result, const std::string& result_msg)     {};
        virtual void OnStatusText (const std::uint8_t& severity, const std::string& status)                       {};
        /*
        * vehicle_mode = mavlinksdk::CMavlinkHelper::getMode (m_heartbeat.custom_mode, m_firmware_type)
        * */
        virtual void OnModeChanges(const int& custom_mode, const int& firmware_type)  {};
        virtual void OnHomePositionUpdated(const mavlink_home_position_t& home_position)  {};
    

        // CCallBack_Communicator Related

        virtual void OnMessageReceived (mavlink_message_t& mavlink_message) {};
        virtual void OnConnected (const bool& connected) {};
    };
}

#endif