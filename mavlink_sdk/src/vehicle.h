#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <map>

#include <all/mavlink.h>
#include <ardupilotmega/ardupilotmega.h>

#include "mavlink_helper.h"

namespace mavlinksdk
{
    // 3 seconds
    #define HEART_BEAT_TIMEOUT 5000000l
    
 
    struct Time_Stamps
    {
        #define TIME_STAMP_MSG_LEN 1024
        #define MESSAGE_UNPROCESSED     0
        #define MESSAGE_PROCESSED       1

        Time_Stamps()
        {
            reset_timestamps();
        }

        void setTimestamp (uint16_t message_id, uint64_t time_stamp)
        {
            if (message_id >= TIME_STAMP_MSG_LEN) return ;
            m_message_id[message_id] = time_stamp;
            m_pocessed_flag[message_id] = MESSAGE_UNPROCESSED;
        }

        uint64_t getMessageTime(uint16_t message_id) const
        {
            if (message_id >= TIME_STAMP_MSG_LEN) return 0;
            return m_message_id[message_id];
        }

        uint16_t getProcessedFlag(uint16_t message_id) const
        {
            if (message_id >= TIME_STAMP_MSG_LEN) return 0;
            return m_pocessed_flag[message_id];
        }

        void setProcessedFlag(uint16_t message_id, uint16_t flags)
        {
            if (message_id >= TIME_STAMP_MSG_LEN) return ;
            m_pocessed_flag[message_id] = flags;
        }

        void reset_timestamps()
        {
            for (uint16_t i=0; i< TIME_STAMP_MSG_LEN; ++i)
            {
               m_message_id[i] = 0;
               m_pocessed_flag[i] = MESSAGE_UNPROCESSED;
            }
        }

        private: 
            uint64_t m_message_id[TIME_STAMP_MSG_LEN];
            uint16_t m_pocessed_flag [TIME_STAMP_MSG_LEN];
        
        #undef TIME_STAMP_MSG_LEN

    };


    



    class CCallBack_Vehicle
    {
        public:

        virtual void OnBoardRestarted ()                                                                                                {};
        virtual void OnHeartBeat_First (const mavlink_heartbeat_t& heartbeat)                                                           {};
        virtual void OnHeartBeat_Resumed (const mavlink_heartbeat_t& heartbeat)                                                         {};
        virtual void OnArmed (const bool& armed)                                                                                        {};
        virtual void OnFlying (const bool& isFlying)                                                                                    {};
        virtual void OnACK (const int& acknowledged_cmd, const int& result, const std::string& result_msg)                              {};
        virtual void OnStatusText (const std::uint8_t& severity, const std::string& status)                                             {};
        virtual void OnModeChanges(const uint32_t& custom_mode, const int& firmware_type, const MAV_AUTOPILOT& autopilot)               {};
        virtual void OnHomePositionUpdated(const mavlink_home_position_t& home_position)                                                {};
        virtual void OnServoOutputRaw(const mavlink_servo_output_raw_t& servo_output_raw)                                               {};
        virtual void OnHighLatencyModeChanged (const int& latency_mode)                                                                 {};
        virtual void OnHighLatencyMessageReceived (const int& latency_mode)                                                             {};
        virtual void OnEKFStatusReportChanged (const mavlink_ekf_status_report_t& ekf_status_report)                                    {};
        virtual void OnVibrationChanged (const mavlink_vibration_t& vibration)                                                          {};
        virtual void OnADSBVechileReceived (const mavlink_adsb_vehicle_t& adsb_vehicle)                                                 {};
        virtual void OnDistanceSensorChanged (const mavlink_distance_sensor_t& distance_sensor)                                            {};
    };

    class CVehicle
    {
        
        public:
            //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
            static CVehicle& getInstance()
            {
                static CVehicle instance;

                return instance;
            }

            CVehicle(CVehicle const&)               = delete;
            void operator=(CVehicle const&)         = delete;

        
            // Note: Scott Meyers mentions in his Effective Modern
            //       C++ book, that deleted functions should generally
            //       be public as it results in better error messages
            //       due to the compilers behavior to check accessibility
            //       before deleted status

        private:

            CVehicle();

        public:
            
            ~CVehicle ()
            {

            }

        public:

            void set_callback_vehicle (mavlinksdk::CCallBack_Vehicle* callback_vehicle);
            void parseMessage (const mavlink_message_t& mavlink_message);


        protected:

            bool handle_heart_beat              (const mavlink_heartbeat_t& heartbeat);
            void handle_extended_system_state   (const mavlink_extended_sys_state_t& extended_system_state);
            void handle_cmd_ack                 (const mavlink_command_ack_t& command_ack);
            void handle_status_text             (const mavlink_statustext_t& status_text);
            void handle_home_position           (const mavlink_home_position_t& home_position);
            void handle_param_ext_value         (const mavlink_param_ext_value_t& param_message);
            void handle_param_value             (const mavlink_param_value_t& param_message);
            void handle_adsb_vehicle            (const mavlink_adsb_vehicle_t& adsb_vehicle);
            void handle_rc_channels_raw         (const mavlink_rc_channels_t& rc_channels);
            void handle_servo_output_raw        (const mavlink_servo_output_raw_t& servo_output_raw);
            void handle_system_time             (const mavlink_system_time_t& system_time);
            void handle_radio_status            (const mavlink_radio_status_t& radio_status);
            void handle_terrain_data_report     (const mavlink_terrain_report_t& terrain_report);
            void handle_ekf_status_report       (const mavlink_ekf_status_report_t& ekf_status_report);
            void handle_vibration_report        (const mavlink_vibration_t& vibration);
            void handle_distance_sensor         (const mavlink_distance_sensor_t& distance_sensor);
            
            void handle_high_latency            (const int message_id);
            void exit_high_latency              ();

        // Vechile Methods
        public:
            const bool isFCBConnected() const;

            inline const uint64_t getMessageTime(uint16_t message_id) const
            {
                return time_stamps.getMessageTime(message_id);
            }

            inline const uint64_t getProcessedFlag(uint16_t message_id) const
            {
                return time_stamps.getProcessedFlag(message_id);
            }

            inline void setProcessedFlag(uint16_t message_id, uint16_t flags)
            {
                return time_stamps.setProcessedFlag(message_id, flags);
            }

            inline const bool isArmed()
            {
                return m_armed;
            }
            
            inline const bool isFlying() const
            {
                return m_is_flying;
            } 

            inline const bool hasLidarAltitude() const 
            {
                return m_has_lidar_altitude;
            }

            inline const mavlink_heartbeat_t& getMsgHeartBeat () const
            {
                return m_heartbeat;
            }

            inline const mavlink_sys_status_t& getMsgSysStatus () const
            {
                return m_sys_status;
            }

            inline const mavlink_battery_status_t& getMsgBatteryStatus () const
            {
                return m_battery_status;
            }

            inline const mavlink_battery2_t& getMsgBattery2Status () const
            {
                return m_battery2;
            }

            inline const int getHighLatencyMode () const
            {
                return m_high_latency_mode;
            }

            inline const mavlink_high_latency_t& getHighLatency () const
            {
                return m_high_latency;
            }

            inline const mavlink_high_latency2_t& getHighLatency2 () const
            {
                return m_high_latency2;
            }

            inline const mavlink_local_position_ned_t& getMsgLocalPositionNED () const
            {
                return m_local_position_ned;
            }

            inline const mavlink_global_position_int_t& getMsgGlobalPositionInt () const
            {
                return m_global_position_int;
            }

            inline const mavlink_position_target_local_ned_t& getMsgTargetPositionLocalNED () const
            {
                return m_position_target_local_ned;
            }

            inline const mavlink_position_target_global_int_t& getMsgTargetPositionGlobalInt () const
            {
                return m_position_target_global_int;
            }

            inline const mavlink_gps_raw_int_t& getMSGGPSRaw () const
            {
                return m_gps_raw_int;
            }

            inline const mavlink_gps2_raw_t& getMSGGPS2Raw () const
            {
                return m_gps2_raw;
            }

            inline const mavlink_attitude_t& getMsgAttitude () const
            {
                return m_attitude;
            }

            inline const mavlink_vfr_hud_t& getMsgVFRHud () const
            {
                return m_vfr_hud;
            }

            inline const mavlink_wind_t& getMsgWind () const
            {
                return m_wind;
            }

            inline const mavlink_distance_sensor_t& getDistanceSensor (uint8_t direction)
            {
                return m_distance_sensors[direction];
            }

            inline const mavlink_distance_sensor_t& getLidarAltitude()
            {
                return m_distance_sensors[MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_270];
            }

            inline const mavlink_home_position_t& getMsgHomePosition () const
            {
                return m_home_position;
            }

            inline const mavlink_nav_controller_output_t& getMsgNavController() const
            {
                return m_nav_controller;
            }

            inline const mavlink_adsb_vehicle_t& getADSBVechile() const 
            {
                return m_adsb_vehicle;
            }

            inline const mavlink_rc_channels_t& getRCChannels () const
            {
                return m_rc_channels;
            }

            inline const mavlink_system_time_t& getSystemTime() const 
            {  
                return m_system_time;
            }

            inline const mavlink_radio_status_t& getRadioStatus() const 
            {
                return m_radio_status;
            }

            inline const mavlink_terrain_report_t& getTerrainReport() const 
            {  
                return m_terrain_report;
            }

            inline const mavlink_ekf_status_report_t& getEkf_status_report() const 
            {
                return m_ekf_status_report;   
            }

            inline const mavlink_vibration_t& getVibration() const 
            {
                return m_vibration;   
            }

            inline const std::string& getLastStatusText () const
            {
                return m_status_text;
            }

        // Class Members
        protected:
            mavlinksdk::CCallBack_Vehicle* m_callback_vehicle;
            bool m_heart_beat_first_recieved = false;


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

            /**
             * @brief 
             * only battery voltage
             */
            mavlink_battery2_t m_battery2;

            
            // High Latency
            int m_high_latency_mode = 0; // either equal to MAVLINK_MSG_ID_HIGH_LATENCY or MAVLINK_MSG_ID_HIGH_LATENCY2 or 0
            mavlink_high_latency_t m_high_latency;
            mavlink_high_latency2_t m_high_latency2;

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

            // GPS 2 Raw
            mavlink_gps2_raw_t m_gps2_raw;

            // HiRes IMU
            mavlink_highres_imu_t m_highres_imu;

            // Attitude
            mavlink_attitude_t m_attitude;

            // Attitude
            mavlink_vfr_hud_t m_vfr_hud;

            // Wind
            mavlink_wind_t m_wind;

            /**
            * @brief store all MAV_SENSOR_ORIENTATION predefined directions.
            * Distance Sensors
            */
            mavlink_distance_sensor_t m_distance_sensors[41];

            // System Parameters?

            // Home Position
            mavlink_home_position_t m_home_position;

            // Desired (pitch, roll, yaw, wp_dist, alt_error)
            mavlink_nav_controller_output_t m_nav_controller;

            //ADSB
            mavlink_adsb_vehicle_t  m_adsb_vehicle;
            
            // RCChannels
            mavlink_rc_channels_t   m_rc_channels;

            //Servo Output Raw
            mavlink_servo_output_raw_t   m_servo_output_raw;
            
            // System Time
            mavlink_system_time_t  m_system_time;

            // Radio Status
            mavlink_radio_status_t m_radio_status;

            // terrain report
            mavlink_terrain_report_t m_terrain_report;

            /**
             * @brief 
             * m_ekf_status_report.flags = EKF_STATUS_FLAGS contains status notification.
             */
            mavlink_ekf_status_report_t m_ekf_status_report; 

            // efk status
            mavlink_vibration_t m_vibration;
            
            /**
             * @brief true if m_vibration.clipping increased 
             * from last read.
             */
            bool m_clipping = false;

            // Status Text
            std::string m_status_text;
            std::uint8_t m_status_severity =0;

            // Time Stamps
            Time_Stamps time_stamps;

            // Vehicle is armed
            bool m_armed     = false;
            // Flying or Diving => Armed Ready to take-off. It is not necessary physically flying
            bool m_is_flying = false;

            // Valid only with PX4
            bool m_is_landing = false;
            
            bool m_has_lidar_altitude = false;

    };
}


#endif // VEHICLE_H_
