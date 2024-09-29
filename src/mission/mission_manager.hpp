#ifndef MISSION_MANAGER_H_
#define MISSION_MANAGER_H_

#include <iostream>


#include "../de_general_mission_planner/mission_manager_base.hpp"

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
                   
                }

                
            public:
                
                ~CMissionManager ()
                {

                }



            public:

                void uploadMissionIntoSystem(const std::string& plan_text);
                
                void uploadMissionIntoSystem2(const Json_de& plan);

                void extractPlanMavlinkMission (const Json_de& plan);

            public:
            
                inline void clearMissionItems ()
                {
                    m_mission_items.clear();
                }
                
                

            protected:

                inline void addMissionItem(int id, std::unique_ptr<CMissionItem> item) {
                    // Move the unique_ptr into the map
                    m_mission_items[id] = std::move(item);
                    
                }

        
            private:

                std::map <int, std::unique_ptr<CMissionItem>> m_mission_items;
    };
}
}
}
#endif