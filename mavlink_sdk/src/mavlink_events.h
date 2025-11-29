#ifndef MAVLINK_CEVENTS_H_
#define MAVLINK_CEVENTS_H_
#include <iostream>

namespace mavlinksdk
{
    /**
     * @brief This class holds all exposed to outter useres callback functions from mavlink_sdk library.
     * All functions are virtual and user can override only needed functions.
     * 
     */
    class CMavlinkEvents 
    {
        
        
        
        public: 
        
        // CCallBack_Vehicle Related
        virtual void OnHeartBeat_First (const mavlink_heartbeat_t& heartbeat)                                               {};
        virtual void OnHeartBeat_Resumed (const mavlink_heartbeat_t& heartbeat)                                             {};
        virtual void OnBoardRestarted ()                                                                                    {};
        virtual void OnArmed  (const bool& armed, const bool& ready_to_arm)                                                                           {};
        virtual void OnFlying (const bool& isFlying)                                                                        {};
        virtual void OnACK    (const int& acknowledged_cmd, const int& result, const std::string& result_msg)               {};
        virtual void OnStatusText (const std::uint8_t& severity, const std::string& status)                                 {};
        /*
        * vehicle_mode = mavlinksdk::CMavlinkHelper::getMode (m_heartbeat.custom_mode, m_firmware_type)
        * */
        virtual void OnModeChanges(const uint32_t& custom_mode, const int& firmware_type, const MAV_AUTOPILOT& autopilot)   {};
        virtual void OnHomePositionUpdated(const mavlink_home_position_t& home_position)                                    {};
        virtual void OnServoOutputRaw(const mavlink_servo_output_raw_t& servo_output_raw, const bool& changed)              {};
        virtual void OnHighLatencyModeChanged (const int& latency_mode)                                                     {};
        virtual void OnHighLatencyMessageReceived (const int& latency_mode)                                                 {};
        virtual void OnEKFStatusReportChanged (const mavlink_ekf_status_report_t& ekf_status_report)                        {};    
        virtual void OnVibrationChanged (const mavlink_vibration_t& vibration)                                              {};    
        virtual void OnADSBVechileReceived (const mavlink_adsb_vehicle_t& adsb_vehicle)                                     {};    
        virtual void OnDistanceSensorChanged (const mavlink_distance_sensor_t& distance_sensor)                             {};

        virtual void OnParamReceived(const std::string& param_name, const mavlink_param_value_t& param_message, const bool& changed, const bool &load_parameters_1st_iteration)  {};
        virtual void OnParamReceivedCompleted()                                                                             {};
    

        //CCallBack_WayPoint
        virtual void OnWaypointReached(const int& seq)                                                                      {};
        virtual void OnWayPointReceived(const mavlink_mission_item_int_t& mission_item_int)                                 {};
        virtual void OnWayPointsLoadingCompleted ()                                                                         {};
        virtual void OnMissionSaveFinished (const int& result, const int& mission_type, const std::string& result_msg)      {};
        virtual void OnMissionACK (const int& result, const int& mission_type, const std::string& result_msg)               {}; 
        virtual void OnMissionCurrentChanged (const mavlink_mission_current_t& mission_current)                             {};

        // CCallBack_Communicator Related

        virtual void OnMessageReceived (const mavlink_message_t& mavlink_message) {};
        virtual void OnConnected (const bool& connected) {};
    };
}

#endif