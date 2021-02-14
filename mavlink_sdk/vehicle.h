#ifndef VEHICLE_H_
#define VEHICLE_H_


#include "mavlink_helper.h"

namespace mavlinksdk
{
    
    struct Time_Stamps
    {
        #define TIME_STAMP_MSG_LEN 1024

        Time_Stamps()
        {
            reset_timestamps();
        }

        uint64_t message_id[1024];
        
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

        virtual void OnHeartBeat_First (const mavlink_heartbeat_t& heartbeat)       = 0;
        virtual void OnArmed  (const bool armed)                                    = 0;
        virtual void OnFlying (const bool isFlying)                                 = 0;
        virtual void OnACK    (const int result, const std::string& result_msg)     = 0;
        virtual void OnStatusText (const std::string& status)                       = 0;
        virtual void OnModeChanges(const int mode_number, const int firmware_type)  = 0;
    };

    class CVehicle
    {
        public:
        
            explicit CVehicle(mavlinksdk::CCallBack_Vehicle& callback_vehicle);
            ~CVehicle();
            
        public:

            void parseMessage       (mavlink_message_t& mavlink_message);
            void handle_heart_beat  (const mavlink_heartbeat_t& heartbeat);
            void handle_cmd_ack     (const mavlink_command_ack_t& command_ack);
            void handle_status_text (const mavlink_statustext_t& status_text);

        // Vechile Methods
        public:
            const int getVehicleType()
            {
                return m_heartbeat.type;
            }

            const bool isArmed()
            {
                return m_armed;
            }
            
            const bool isFlying()
            {
                return m_is_flying;
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

            // Time Stamps
            Time_Stamps time_stamps;

            // Vehicle is armed
            bool m_armed    = false;
            // Flying or Diving
            bool m_is_flying = false;
            // Firmware Type
            mavlinksdk::FIRMWARE_TYPE m_firmware_type;
    };
}


#endif // VEHICLE_H_
