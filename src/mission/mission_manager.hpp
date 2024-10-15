#ifndef MISSION_MANAGER_H_
#define MISSION_MANAGER_H_

#include <iostream>
#include <vector>

#include <mavlink_command.h>
#include <mavlink_sdk.h>

#include "missions.hpp"
#include "../de_general_mission_planner/mission_manager_base.hpp"

#define AP_EVENT_DISABLED 0


namespace de
{
namespace fcb
{
namespace mission
{
    class CMissionManager : public de::mission::CMissionManagerBase
    {
        public:

            
            static CMissionManager& getInstance()
            {
                static CMissionManager instance;

                return instance;
            }

            CMissionManager(CMissionManager const&)             = delete;
            void operator=(CMissionManager const&)              = delete;

        
            private:

                CMissionManager()
                {
                    clearMissionItems(); // clear DroneEngage Events & Mission (NOT ARDUPILOT FCB)
                }

                
            public:
                
                ~CMissionManager ()
                {

                }



            public:

                void uploadMissionIntoSystem(const std::string& plan_text);
                
                void uploadMissionIntoSystem2(const Json_de& plan);

                void extractPlanMavlinkMission (const Json_de& plan);
                
                void extractPlanModule (const Json_de& plan) override;
                void deEventFiredExternally (const std::string de_event_sid) override;
                void deEventFiredInternally (const std::string de_event_sid);
            
            public:

                void processMyWaitingEvent();
                void reloadWayPoints();
                void saveWayPointsToFCB();
                void clearWayPoints();

            public:
                void readWaitingEventFromFCB(const mavlink_servo_output_raw_t &servo_output_raw);
                void readFiredEventFromFCB(const mavlink_servo_output_raw_t &servo_output_raw);

            public:

                inline void setCurrentWaitingEvent(const std::string event_id)
                {
                    m_event_waiting_for = event_id;
                    m_event_waiting_for_processed = false;
                }

                void clearMissionItems ();
                

                /**
                * @brief Set (define) the Event Channels used for Fire & Wait Events
                * 
                * @param event_fire_channel 
                * @param event_wait_channel 
                */
                void setEventChannel (const int event_fire_channel, const int event_wait_channel)
                {
                    m_event_fire_channel = event_fire_channel; 
                    m_event_wait_channel = event_wait_channel;
                }
                

                inline de::fcb::mission::ANDRUAV_UNIT_MISSION& getAndruavMission()
                {
                    return m_andruav_missions;      
                } 

            protected:

                inline void addMissionItem(int id, std::unique_ptr<CMissionItem> item) {
                    // Move the unique_ptr into the map
                    m_mission_items[id] = std::move(item);
                }

        
            private:

                std::string m_event_waiting_for;
                bool m_event_waiting_for_processed;
                std::map <int, std::unique_ptr<CMissionItem>> m_mission_items;
                de::fcb::mission::ANDRUAV_UNIT_MISSION m_andruav_missions;      

                
                std::vector<std::string>  m_event_received_from_others;
            
                //event sent from Ardupilot-Board (RCOUT) to indicate that it is waiting for it.
                int m_mavlink_event_waiting_for;
                std::vector<int>  m_event_fired_by_me;   


            private:
                /**
                * @brief servo channel used for sending events
                * 
                */
                int m_event_fire_channel;
                int m_event_wait_channel;
            
    };
}
}
}
#endif