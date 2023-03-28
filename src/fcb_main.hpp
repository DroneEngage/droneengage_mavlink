#ifndef FCB_MAIN_H_
#define FCB_MAIN_H_
#include <thread>         // std::thread
#include <memory>
#include <vector>
#include <all/mavlink.h>
#include <mavlink_sdk.h>
#include <mavlink_command.h>
#include <mavlink_events.h>

#include "./uavos_common/uavos_module.hpp"
#include "./uavos_common/udpProxy.hpp"
#include "./helpers/json.hpp"
using Json = nlohmann::json;


#include "defines.hpp"


#define EVENT_TIME_DIVIDER      5
namespace uavos
{
namespace fcb
{


    typedef struct 
    {
        std::string party_id;
        bool is_online;

    } ANDRUAV_UNIT_STRUCT;

    typedef struct 
    {
        std::string udp_ip1;
        int udp_port1;
        std::string udp_ip2;
        int udp_port2;
        bool enabled;
        uavos::comm::CUDPProxy udp_client;
    } ANDRUAV_UDP_PROXY;

    /**
     * @brief This class is the heart of FCB module. 
     * It handles logic and vehicle states that is related to Andruav.
     * It also communicates with physical FCB using mavlinksdk library.
     * 
     */
    class CFCBMain: public uavos::CMODULE,  mavlinksdk::CMavlinkEvents, uavos::comm::CCallBack_UdpProxy
    {
        public:
            //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
            static CFCBMain& getInstance()
            {
                static CFCBMain instance;

                return instance;
            }

            CFCBMain(CFCBMain const&)               = delete;
            void operator=(CFCBMain const&)         = delete;

        
            // Note: Scott Meyers mentions in his Effective Modern
            //       C++ book, that deleted functions should generally
            //       be public as it results in better error messages
            //       due to the compilers behavior to check accessibility
            //       before deleted status

        private:

            CFCBMain()
            {
                m_event_wait_channel = -1; // no event
                m_event_fired_by_me.clear(); // nothing fired

                m_module_features.push_back("T");
                m_module_features.push_back("R");

                m_module_class="fcb";
            };

        public:
            
            ~CFCBMain ()
            {
                if (m_exit_thread == false)
                {
                    uninit();
                }
            };
                

        public:

            bool init () override;
            bool uninit () override;

            void loopScheduler();

            /**
             * @details register callback function to send system message using it.
             * 
             * @param sendJMSG of type @link SEND_SYSMSG_CALLBACK @endlink 
             */
            void registerSendSYSMSG (SEND_SYSMSG_CALLBACK sendSYSMSG) override
            {
                m_fcb_facade.RegisterSendSYSMSG(sendSYSMSG);
            };            

            /**
             * @details register callback function to send message using it.
             * 
             * @param sendJMSG of type @link SEND_JMSG_CALLBACK @endlink 
             */
            void registerSendJMSG (SEND_JMSG_CALLBACK sendJMSG) override
            {
                m_fcb_facade.RegisterSendJMSG(sendJMSG);
            };            

            /**
             * @details register callback function to send message using it.
             * 
             * @param sendBMSG of type @link SEND_BMSG_CALLBACK @endlink 
             */
            void registerSendBMSG (SEND_BMSG_CALLBACK sendBMSG) override
            {
                m_fcb_facade.RegisterSendBMSG(sendBMSG);
            };            

            /**
             * @details register call back to send InterModule remote execute message.
             * 
             * @param sendMREMSG of type @link SEND_MREMSG_CALLBACK @endlink 
             */
            void registerSendMREMSG (SEND_MREMSG_CALLBACK sendMREMSG) override
            {
                m_fcb_facade.RegisterSendMREMSG(sendMREMSG);
            };            

            /* cannot connect to uavos comm*/
            void alertDroneEngageOffline ();
            
            const ANDRUAV_VEHICLE_INFO& getAndruavVehicleInfo ()
            {
                return m_andruav_vehicle_info;
            }

            uavos::fcb::mission::ANDRUAV_UNIT_MISSION& getAndruavMission()
            {
                return m_andruav_missions;      
            } 
            


        public:

            void clearWayPoints();
            void reloadWayPoints();
            void saveWayPointsToFCB();

            void releaseRemoteControl();
            void updateRemoteControlChannels(const int16_t rc_channels[18]);

            void adjustRemoteJoystickByMode(RC_SUB_ACTION rc_sub_action);
            void remoteControlSignal();
            void centerRemoteControl();
            void freezeRemoteControl();
            void enableRemoteControl();
            void enableRemoteControlGuided();
            void processIncommingEvent();
            void insertIncommingEvent(const int16_t event_id);
            /**
             * @brief Set the PartyID & GroupID
             * 
             * @param party_id 
             * @param group_id 
             */
            void setPartyID (const std::string& party_id, const std::string& group_id) override
            {
                m_andruav_vehicle_info.party_id = party_id;
                m_andruav_vehicle_info.group_id = group_id;
            }

            /**
             * @brief Set the Event Channels used for Fire & Wait Events
             * 
             * @param event_fire_channel 
             * @param event_wait_channel 
             */
            void setEventChannel (const int event_fire_channel, const int event_wait_channel)
            {
                m_event_fire_channel = event_fire_channel; 
                m_event_wait_channel = event_wait_channel;
            }

            /**
             * @deprecated
             */
            //void toggleMavlinkStreaming (const std::string& target_party_id, const int& request_type, const int& streaming_level);
            
            void setStreamingLevel (const std::string& target_party_id, const int& streaming_level);

            bool isFCBConnected () const { return m_fcb_connected;}; 
            
            const RCMAP_CHANNELS_MAP_INFO_STRUCT getRCChannelsMapInfo() const
            {
                return m_rcmap_channels_info;
            };

            /**
             * @brief called when receive {@link TYPE_AndruavSystem_UdpProxy @endlink} to 
             * add update udpProxy database and creates a UDP socket. 
             * 
             * @param enabled 
             * @param udp_ip1 
             * @param udp_port1 
             * @param udp_ip2 
             * @param udp_port2 
             */
            void updateUDPProxy(const bool& enabled, const std::string&udp_ip1, const int& udp_port1, const std::string&udp_ip2, const int& udp_port2);

            /**
             * @brief sends udpProxy socket info to another party to start communication with me.
             * send only one socket info not the one that I use here to communicate.
             * 
             * @param target_party_id 
             */
            void sendUdpProxyStatus (const std::string& target_party_id);

            /**
             * @brief in future there maybe multiple up for multiple purpose. for now
             * this message will return true of there is a single udpProxy enabled.
             * 
             * @return true 
             * @return false 
             */
            bool isUdpProxyMavlinkAvailable () const;


        public:
            void OnHeartBeat ();
            void OnCommandLong (const mavlink_command_long_t& command_long);
            
        // Events implementation of mavlinksdk::CMavlinkEvents
        public:
            void OnMessageReceived (const mavlink_message_t& mavlink_message) override;           
            void OnConnected (const bool& connected) override;
            void OnHeartBeat_First (const mavlink_heartbeat_t& heartbeat) override;
            void OnHeartBeat_Resumed (const mavlink_heartbeat_t& heartbeat) override ;
            void OnBoardRestarted () override;
            void OnArmed (const bool& armed) override;
            void OnFlying (const bool& isFlying) override;
            void OnStatusText (const std::uint8_t& severity, const std::string& status) override;
            void OnModeChanges(const uint32_t& custom_mode, const int& firmware_type, const MAV_AUTOPILOT& autopilot) override;
            void OnHomePositionUpdated(const mavlink_home_position_t& home_position)  override;
            void OnServoOutputRaw(const mavlink_servo_output_raw_t& servo_output_raw)  override;
            void OnMissionACK (const int& result, const int& mission_type, const std::string& result_msg)override;
            void OnACK (const int& acknowledged_cmd, const int& result, const std::string& result_msg) override;
            void OnWaypointReached(const int& seq) override;
            void OnWayPointReceived(const mavlink_mission_item_int_t& mission_item_int) override;
            void OnWayPointsLoadingCompleted ();
            void OnMissionSaveFinished (const int& result, const int& mission_type, const std::string& result_msg) override;            
            void OnMissionCurrentChanged (const mavlink_mission_current_t& mission_current) override;
            void OnParamReceived(const std::string& param_name, const mavlink_param_value_t& param_message, const bool& changed) override;
            void OnParamReceivedCompleted() override;
            void OnHighLatencyModeChanged (const int& latency_mode) override;
            void OnHighLatencyMessageReceived (const int& latency_mode) override;
            void OnEKFStatusReportChanged (const mavlink_ekf_status_report_t& ekf_status_report) override;
            void OnVibrationChanged (const mavlink_vibration_t& vibration);
            void OnADSBVechileReceived (const mavlink_adsb_vehicle_t& adsb_vehicle);
            void OnDistanceSensorChanged (const mavlink_distance_sensor_t& distance_sensor);        
            
            // called from main
            void OnConnectionStatusChangedWithAndruavServer (const int status) override;
        
        
        // Events implementation of uavos::comm::CCallBack_UdpProxy
        public:
            void OnMessageReceived (const uavos::comm::CUDPProxy * udp_proxy, const char *, int len) override;

        private: 
            void initVehicleChannelLimits(const bool display);
     
        private:
            mavlinksdk::CMavlinkSDK& m_mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
            uavos::fcb::CFCBFacade& m_fcb_facade = uavos::fcb::CFCBFacade::getInstance();
            uavos::fcb::CMavlinkTrafficOptimizer& m_mavlink_optimizer = uavos::fcb::CMavlinkTrafficOptimizer::getInstance();
            
        private:
            int getConnectionType () const; 
            bool connectToFCB ();
            
            void updateGeoFenceHitStatus();
            void takeActionOnFenceViolation(uavos::fcb::geofence::CGeoFenceBase * geo_fence);
            void calculateChannels(const int16_t scaled_channels[16], const bool ignode_dead_band, int16_t *output);
            void update_rcmap_info();
            void checkBlockedStatus();

            /**
             * @brief emulate a camera for ardupilot.
             * 
             */
            void heartbeatCamera ();

        private:
            Json m_jsonConfig;
            int m_connection_type;
            /**
             * @brief servo channel used for sending events
             * 
             */
            int m_event_fire_channel;
            int m_event_wait_channel;
            int m_event_time_divider=0; // wait

            std::vector<int>  m_event_fired_by_me;
            std::vector<int>  m_event_received_from_others;
            int m_event_waiting_for;
            ANDRUAV_VEHICLE_INFO m_andruav_vehicle_info;
            uavos::fcb::mission::ANDRUAV_UNIT_MISSION m_andruav_missions;      

            RCMAP_CHANNELS_MAP_INFO_STRUCT m_rcmap_channels_info;

            /**
             * @brief Andruav units subscribed in telemetry streaming.
             * 
             */
            std::vector<std::unique_ptr<uavos::fcb::ANDRUAV_UNIT_STRUCT>> m_TelemetryUnits;
            

            bool m_exit_thread = true;
            std::thread m_scheduler_thread;
            u_int64_t m_last_start_flying =0 ;
            u_int64_t m_counter =0;


            bool m_fcb_connected = false;
            bool m_enable_udp_telemetry = false; // defined in config file.
            uint16_t m_udp_telemetry_fixed_port = 0;
            uint64_t m_last_access_telemetry = 0;
            ANDRUAV_UDP_PROXY m_udp_proxy;
            

    };
}
}
#endif