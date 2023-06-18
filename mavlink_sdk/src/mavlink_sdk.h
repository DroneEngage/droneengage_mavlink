#ifndef MAVLINK_SDK_H_
#define MAVLINK_SDK_H_


#include <memory>
#include "./helpers/colors.h"
#include "generic_port.h"
#include "mavlink_communicator.h"
#include "vehicle.h"
#include "mavlink_waypoint_manager.h"
#include "mavlink_parameter_manager.h"
#include "mavlink_events.h"


namespace mavlinksdk
{
    class CMavlinkSDK : protected mavlinksdk::comm::CCallBack_Communicator
                        , protected mavlinksdk::CCallBack_Vehicle
                        , protected mavlinksdk::CCallBack_WayPoint
                        , protected mavlinksdk::CCallBack_Parameter

    {
        public:
            //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
            static CMavlinkSDK& getInstance()
            {
                static CMavlinkSDK instance;

                return instance;
            }

            CMavlinkSDK(CMavlinkSDK const&)           = delete;
            void operator=(CMavlinkSDK const&)        = delete;

        
            // Note: Scott Meyers mentions in his Effective Modern
            //       C++ book, that deleted functions should generally
            //       be public as it results in better error messages
            //       due to the compilers behavior to check accessibility
            //       before deleted status

        private:

            CMavlinkSDK() 
            {
                m_mavlink_events    = (mavlinksdk::CMavlinkEvents*) this;
            }                    // Constructor? (the {} brackets) are needed here.

            
        public:
            
            ~CMavlinkSDK ();

        public:
            void start (mavlinksdk::CMavlinkEvents * mavlink_events);
            void connectUDP (const char *target_ip, const int udp_port);
            void connectSerial (const char *uart_name, const int baudrate, const bool dynamic);
            void stop();


        
        // Writing Status
        public:    
            
            void sendMavlinkMessage(const mavlink_message_t& mavlink_message);
            
        protected:
              mavlinksdk::CMavlinkEvents * m_mavlink_events; 
              mavlinksdk::CCallBack_Vehicle* m_callback_vehicle;
              mavlinksdk::CCallBack_WayPoint* m_callback_waypoint;
              std::shared_ptr<mavlinksdk::comm::GenericPort> m_port;
              std::unique_ptr<mavlinksdk::comm::CMavlinkCommunicator> m_communicator;
              bool m_stopped_called = false;
              

        // CCallback_Communicator inheritance
        protected:
            void OnMessageReceived (const mavlink_message_t& mavlink_message) override;
            void OnConnected (const bool& connected) override;

        // CCalback_WayPointManager inheritance
        protected:
            inline void OnMissionACK (const int& result, const int& mission_type, const std::string& result_msg)  override
            {
                std::cout << _INFO_CONSOLE_TEXT << "OnMissionACK " << std::to_string(result) << " - " << result_msg << _NORMAL_CONSOLE_TEXT_ << std::endl;    
                m_mavlink_events->OnMissionACK (result, mission_type, result_msg);
            }


            inline void OnMissionSaveFinished (const int& result, const int& mission_type, const std::string& result_msg) override 
            {
                std::cout << _INFO_CONSOLE_TEXT << "OnMissionSaveFinished " << std::to_string(result) << " - " << result_msg << _NORMAL_CONSOLE_TEXT_ << std::endl;    
                m_mavlink_events->OnMissionSaveFinished (result, mission_type, result_msg);
            }
        

            inline void OnWaypointReached (const int& sequence) override 
            {
                std::cout << _INFO_CONSOLE_TEXT << "OnWaypointReached " << std::to_string(sequence) << _NORMAL_CONSOLE_TEXT_ << std::endl;    
                m_mavlink_events->OnWaypointReached (sequence);
            }

            inline void OnWayPointReceived (const mavlink_mission_item_int_t& mission_item_int) override
            {
                std::cout << _INFO_CONSOLE_TEXT << "OnWayPointReceived " << std::to_string(mission_item_int.seq) << _NORMAL_CONSOLE_TEXT_ << std::endl;    
                m_mavlink_events->OnWayPointReceived (mission_item_int);
            }

            inline void OnMissionCurrentChanged (const mavlink_mission_current_t& mission_current) override
            {
                m_mavlink_events->OnMissionCurrentChanged (mission_current);
            }  

            inline void OnWayPointsLoadingCompleted () override 
            {
                m_mavlink_events->OnWayPointsLoadingCompleted();
            }
   
    
            
        // CCallback_Vehicle inheritance
        protected:
            inline void OnBoardRestarted () override 
            {
                m_mavlink_events->OnBoardRestarted ();
            };
            inline void OnHeartBeat_First (const mavlink_heartbeat_t& heartbeat) override 
            {
                m_mavlink_events->OnHeartBeat_First (heartbeat);
            };
            inline void OnHeartBeat_Resumed (const mavlink_heartbeat_t& heartbeat) override 
            {
                m_mavlink_events->OnHeartBeat_Resumed (heartbeat);
            };
            inline void OnArmed (const bool& armed) override 
            {
                #ifdef DEBUG
                    std::cout << _INFO_CONSOLE_TEXT << "OnArmed " << std::to_string(armed) << _NORMAL_CONSOLE_TEXT_ << std::endl;    
                #endif

                m_mavlink_events->OnArmed (armed);

            };
            inline void OnFlying (const bool& isFlying) override 
            {
                #ifdef DEBUG
                    std::cout << _INFO_CONSOLE_TEXT << "OnFlying " << std::to_string(isFlying) << _NORMAL_CONSOLE_TEXT_ << std::endl;    
                #endif

                m_mavlink_events->OnFlying (isFlying);
            };

            inline void OnACK (const int& acknowledged_cmd, const int& result, const std::string& result_msg) override 
            {
                #ifdef DEBUG
                    std::cout << _INFO_CONSOLE_TEXT << "OnACK  of CMD" << std::to_string(acknowledged_cmd) << " RES:" << std::to_string(result) << " - " << result_msg << _NORMAL_CONSOLE_TEXT_ << std::endl;    
                #endif

                m_mavlink_events->OnACK (acknowledged_cmd, result, result_msg);
            };

            inline void OnStatusText (const std::uint8_t& severity, const std::string& status) override 
            {
                #ifdef DEBUG
                    std::cout << _INFO_CONSOLE_TEXT << "Status " << status << _NORMAL_CONSOLE_TEXT_ << std::endl;
                #endif

                m_mavlink_events->OnStatusText (severity, status);
            };
            
            inline void OnModeChanges(const uint32_t& custom_mode, const int& firmware_type, const MAV_AUTOPILOT& autopilot)  override
            {
                m_mavlink_events->OnModeChanges (custom_mode, firmware_type, autopilot);
            };

            inline void OnHomePositionUpdated(const mavlink_home_position_t& home_position)  override
            {
                m_mavlink_events->OnHomePositionUpdated (home_position);
            };

            inline void OnServoOutputRaw(const mavlink_servo_output_raw_t& servo_output_raw)  override
            {
                m_mavlink_events->OnServoOutputRaw (servo_output_raw);
            };

            inline void OnHighLatencyModeChanged (const int& latency_mode) override 
            {
                m_mavlink_events->OnHighLatencyModeChanged (latency_mode);
            };
        
            inline void OnHighLatencyMessageReceived (const int& latency_mode) override
            {
                m_mavlink_events->OnHighLatencyMessageReceived (latency_mode);
            };

            inline void OnParamReceived(const std::string& param_name, const mavlink_param_value_t& param_message, const bool& changed) override 
            {
                m_mavlink_events->OnParamReceived (param_name, param_message, changed);
            };
    
            inline void OnParamReceivedCompleted() override 
            {
                m_mavlink_events->OnParamReceivedCompleted ();
            };
    
            inline void OnEKFStatusReportChanged (const mavlink_ekf_status_report_t& ekf_status_report)
            {
                m_mavlink_events->OnEKFStatusReportChanged (ekf_status_report);
            };   

            inline void OnVibrationChanged (const mavlink_vibration_t& vibration)
            {
                m_mavlink_events->OnVibrationChanged (vibration);
            };  

            inline void OnADSBVechileReceived (const mavlink_adsb_vehicle_t& adsb_vehicle)
            {
                m_mavlink_events->OnADSBVechileReceived (adsb_vehicle);
            };    
     
            inline void OnDistanceSensorChanged (const mavlink_distance_sensor_t& distance_sensor)
            {
                m_mavlink_events->OnDistanceSensorChanged (distance_sensor);
            };    
     
            

    };
}



#endif // MAVLINK_SDK_H_