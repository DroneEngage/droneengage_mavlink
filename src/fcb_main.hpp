#ifndef FCB_MAIN_H_
#define FCB_MAIN_H_
#include <thread>         // std::thread
#include <memory>
#include <common/mavlink.h>
#include <mavlink_sdk.h>
#include <mavlink_command.h>
#include <mavlink_events.h>

#include "./helpers/json.hpp"
using Json = nlohmann::json;


#include "defines.hpp"
#include "./mission/missions.hpp"
#include "fcb_facade.hpp"
#include "fcb_traffic_optimizer.hpp"
namespace uavos
{
namespace fcb
{


    typedef struct 
    {
        std::string party_id;
        bool is_online;

    } ANDRUAV_UNIT_STRUCT;

    /**
     * @brief This class is the heart of FCB module. 
     * It handles logic and vehicle states that is related to Andruav.
     * It also communicates with physical FCB using mavlinksdk library.
     * 
     */
    class CFCBMain: public mavlinksdk::CMavlinkEvents
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

            }

        public:
            
            ~CFCBMain ()
            {
                if (m_exit_thread == false)
                {
                    uninit();
                }
            };
                

        public:

            void init ();
            void uninit ();

            void loopScheduler();

            /**
             * @details register callback function to send message using it.
             * 
             * @param sendJMSG of type @link SEND_JMSG_CALLBACK @endlink 
             */
            void registerSendJMSG (SEND_JMSG_CALLBACK sendJMSG)
            {
                m_fcb_facade.setSendJMSG(sendJMSG);
            };            

            /**
             * @details register callback function to send message using it.
             * 
             * @param sendBMSG of type @link SEND_BMSG_CALLBACK @endlink 
             */
            void registerSendBMSG (SEND_BMSG_CALLBACK sendJMSG)
            {
                m_fcb_facade.setSendBMSG(sendJMSG);
            };            

            /* cannot connect to uavos comm*/
            void alertUavosOffline ();
            
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

            void toggleMavlinkStreaming (const std::string& target_party_id, const int& request_type, const int& streaming_level);

        // Events implementation of mavlinksdk::CMavlinkEvents
        public:
            void OnMessageReceived (const mavlink_message_t& mavlink_message) override;           
            void OnConnected (const bool& connected) override;
            void OnHeartBeat ();
            void OnHeartBeat_First (const mavlink_heartbeat_t& heartbeat) override;
            void OnHeartBeat_Resumed (const mavlink_heartbeat_t& heartbeat) override ;
            void OnArmed (const bool& armed) override;
            void OnFlying (const bool& isFlying) override;
            void OnStatusText (const std::uint8_t& severity, const std::string& status) override;
            void OnModeChanges(const int& custom_mode, const int& firmware_type) override;
            void OnHomePositionUpdated(const mavlink_home_position_t& home_position)  override;
            void onMissionACK (const int& result, const int& mission_type, const std::string& result_msg)override;
            void OnACK (const int& result, const std::string& result_msg) override;
            void OnWaypointReached(const int& seq) override;
            void OnWayPointReceived(const mavlink_mission_item_int_t& mission_item_int) override;
            void OnWayPointsLoadingCompleted ();
            void OnMissionSaveFinished (const int& result, const int& mission_type, const std::string& result_msg) override;            
            void OnParamChanged(const std::string& param_name, const mavlink_param_value_t& param_message, const bool& changed) override;


        private: 
            void initVehicleChannelLimits();
     
        private:
            mavlinksdk::CMavlinkSDK& m_mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
            uavos::fcb::CFCBFacade& m_fcb_facade = uavos::fcb::CFCBFacade::getInstance();
            uavos::fcb::CMavlinkTrafficOptimizer& m_mavlink_optimizer = uavos::fcb::CMavlinkTrafficOptimizer::getInstance();

        private:
            int getConnectionType ();
            bool connectToFCB ();
            
            void calculateChannels(const int16_t scaled_channels[16], const bool ignode_dead_band, int16_t *output);
            
        private:
            Json m_jsonConfig;
            int m_connection_type;
            ANDRUAV_VEHICLE_INFO m_andruav_vehicle_info;
            uavos::fcb::mission::ANDRUAV_UNIT_MISSION m_andruav_missions;      

            /**
             * @brief Andruav units subscribed in telemetry streaming.
             * 
             */
            std::vector<std::unique_ptr<uavos::fcb::ANDRUAV_UNIT_STRUCT>> m_TelemetryUnits;
            
            bool m_exit_thread = true;
            std::thread m_scheduler_thread;
            u_int64_t m_last_start_flying =0 ;
            u_int64_t m_counter =0;
            u_int64_t m_counter_sec =0;
    };
}
}
#endif