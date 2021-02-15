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

        virtual void OnHeartBeat_First   (const mavlink_heartbeat_t& heartbeat)     {};
        virtual void OnHeartBeat_Resumed (const mavlink_heartbeat_t& heartbeat)     {};
        virtual void OnArmed  (const bool armed)                                    {};
        virtual void OnFlying (const bool isFlying)                                 {};
        virtual void OnACK    (const int result, const std::string& result_msg)     {};
        virtual void OnStatusText (const std::uint8_t severity, const std::string& status)                       {};
        // On flying mode changed
        virtual void OnModeChanges(const int custom_mode, const int firmware_type)  {};
    };

    class CVehicle
    {
        public:
        
            explicit CVehicle(mavlinksdk::CCallBack_Vehicle& callback_vehicle);
            ~CVehicle();
    
            void parseMessage       (const mavlink_message_t& mavlink_message);
    


        protected:

            void handle_heart_beat  (const mavlink_heartbeat_t& heartbeat);
            void handle_cmd_ack     (const mavlink_command_ack_t& command_ack);
            void handle_status_text (const mavlink_statustext_t& status_text);

        // Vechile Methods
        public:
            const mavlinksdk::FIRMWARE_TYPE getFirmwareType()
            {
                return m_firmware_type;
            }

            const bool isArmed()
            {
                return m_armed;
            }
            
            const bool isFlying()
            {
                return m_is_flying;
            } 

            const mavlink_heartbeat_t& getMsgHeartBeat ()
            {
                return m_heartbeat;
            }

            const mavlink_sys_status_t& getMsgSysStatus ()
            {
                return m_sys_status;
            }

            const mavlink_battery_status_t& getMsgBatteryStatus ()
            {
                return m_battery_status;
            }

            const mavlink_radio_status_t& getMsgRadioStatus ()
            {
                return m_radio_status;
            }

            const mavlink_local_position_ned_t& getMsgLocalPositionNED ()
            {
                return m_local_position_ned;
            }

            const mavlink_global_position_int_t& getMsgGlobalPositionInt ()
            {
                return m_global_position_int;
            }

            const mavlink_position_target_local_ned_t& getMsgTargetPositionLocalNED ()
            {
                return m_position_target_local_ned;
            }

            const mavlink_position_target_global_int_t& getMsgTargetPositionGlobalInt ()
            {
                return m_position_target_global_int;
            }

            const mavlink_attitude_t& getMsgAttitude ()
            {
                return m_attitude;
            }

            const mavlink_home_position_t& getMsgHomePosition ()
            {
                return m_home_position;
            }

            const std::string& getLastStatusText ()
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

            // HiRes IMU
            mavlink_highres_imu_t m_highres_imu;

            // Attitude
            mavlink_attitude_t m_attitude;

            // System Parameters?

            // Home Position
            mavlink_home_position_t m_home_position;


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
