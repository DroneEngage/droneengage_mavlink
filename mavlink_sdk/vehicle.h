#ifndef VEHICLE_H_
#define VEHICLE_H_


#include "mavlink_helper.h"

namespace mavlinksdk
{
    // 3 seconds
    #define HEART_BEAT_TIMEOUT 3000000l
    
    struct Time_Stamps
    {
        #define TIME_STAMP_MSG_LEN 1024

        Time_Stamps()
        {
            reset_timestamps();
        }

        uint64_t message_id[TIME_STAMP_MSG_LEN];
        
        void reset_timestamps()
        {
            for (int i=0; i< TIME_STAMP_MSG_LEN; ++i)
            {
                message_id[i] =0;
            }
        }

        #undef TIME_STAMP_MSG_LEN

    };


    class CCallBack_Vehicle
    {
        public:

        virtual void OnHeartBeat_First (const mavlink_heartbeat_t& heartbeat)                 {};
        virtual void OnHeartBeat_Resumed (const mavlink_heartbeat_t& heartbeat)                 {};
        virtual void OnArmed (const bool& armed)                                                {};
        virtual void OnFlying (const bool& isFlying)                                             {};
        virtual void OnACK (const int& result, const std::string& result_msg)                 {};
        virtual void OnStatusText (const std::uint8_t& severity, const std::string& status)      {};
        virtual void OnModeChanges(const int& custom_mode, const int& firmware_type)              {};
        virtual void OnHomePositionUpdated(const mavlink_home_position_t& home_position)        {};
    };

    class CVehicle
    {
        public:
        
            explicit CVehicle(mavlinksdk::CCallBack_Vehicle& callback_vehicle);
            ~CVehicle() {};
    
            void parseMessage       (const mavlink_message_t& mavlink_message);
    


        protected:

            inline void handle_heart_beat    (const mavlink_heartbeat_t& heartbeat);
            inline void handle_cmd_ack       (const mavlink_command_ack_t& command_ack);
            inline void handle_status_text   (const mavlink_statustext_t& status_text);
            inline void handle_home_position (const mavlink_home_position_t& home_position);

        // Vechile Methods
        public:
            inline const mavlinksdk::FIRMWARE_TYPE getFirmwareType()
            {
                return m_firmware_type;
            }

            inline const bool isArmed()
            {
                return m_armed;
            }
            
            inline const bool isFlying()
            {
                return m_is_flying;
            } 

            inline const mavlink_heartbeat_t& getMsgHeartBeat ()
            {
                return m_heartbeat;
            }

            inline const mavlink_sys_status_t& getMsgSysStatus ()
            {
                return m_sys_status;
            }

            inline const mavlink_battery_status_t& getMsgBatteryStatus ()
            {
                return m_battery_status;
            }

            inline const mavlink_radio_status_t& getMsgRadioStatus ()
            {
                return m_radio_status;
            }

            inline const mavlink_local_position_ned_t& getMsgLocalPositionNED ()
            {
                return m_local_position_ned;
            }

            inline const mavlink_global_position_int_t& getMsgGlobalPositionInt ()
            {
                return m_global_position_int;
            }

            inline const mavlink_position_target_local_ned_t& getMsgTargetPositionLocalNED ()
            {
                return m_position_target_local_ned;
            }

            inline const mavlink_position_target_global_int_t& getMsgTargetPositionGlobalInt ()
            {
                return m_position_target_global_int;
            }

            inline const mavlink_gps_raw_int_t& getMSGGPSRaw ()
            {
                return m_gps_raw_int;
            }

            inline const mavlink_attitude_t& getMsgAttitude ()
            {
                return m_attitude;
            }

            inline const mavlink_home_position_t& getMsgHomePosition ()
            {
                return m_home_position;
            }

            inline const mavlink_nav_controller_output_t& getMsgNavController()
            {
                return m_nav_controller;
            }

            inline const std::string& getLastStatusText ()
            {
                return m_status_text;
            }

        // Class Members
        protected:
            mavlinksdk::CCallBack_Vehicle& m_callback_vehicle;
            bool m_heart_beat_first = false;


        // Vehicle Attributes
        protected:
            // Heartbeat
            mavlink_heartbeat_t m_heartbeat;

            // System Status
            mavlink_sys_status_t m_sys_status;

            // Battery Status
            /**
             * @brief 
             *  voltage in x1000 array of 10
             *  current in x100
             *  temprature in x 1000
             */
            mavlink_battery_status_t m_battery_status;

            // Radio Status
            mavlink_radio_status_t m_radio_status;

            // Local Position
            mavlink_local_position_ned_t m_local_position_ned;

            // Global Position
            mavlink_global_position_int_t m_global_position_int;

            // Local Position Target
            mavlink_position_target_local_ned_t m_position_target_local_ned;

            // Global Position Target
            mavlink_position_target_global_int_t m_position_target_global_int;

            // GPS Raw
            mavlink_gps_raw_int_t m_gps_raw_int;

            // HiRes IMU
            mavlink_highres_imu_t m_highres_imu;

            // Attitude
            mavlink_attitude_t m_attitude;

            // System Parameters?

            // Home Position
            mavlink_home_position_t m_home_position;

            // Desired (pitch, roll, yaw, wp_dist, alt_error)
            mavlink_nav_controller_output_t m_nav_controller;

            // Status Text
            std::string m_status_text;
            std::uint8_t m_status_severity;

            // Time Stamps
            Time_Stamps time_stamps;

            // Vehicle is armed
            bool m_armed     = false;
            // Flying or Diving
            bool m_is_flying = false;
            // Firmware Type
            mavlinksdk::FIRMWARE_TYPE m_firmware_type = FIRMWARE_TYPE_UNKNOWN;
    };
}


#endif // VEHICLE_H_
