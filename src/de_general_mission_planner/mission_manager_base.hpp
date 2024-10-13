#ifndef MISSION_MANAGER_BASE_H_
#define MISSION_MANAGER_BASE_H_


#include "../helpers/json_nlohmann.hpp"
using Json_de = nlohmann::json;

namespace de
{
namespace mission
{
    class CMissionManagerBase
    {
        public:

            
            static CMissionManagerBase& getInstance()
            {
                static CMissionManagerBase instance;

                return instance;
            }

            CMissionManagerBase(CMissionManagerBase const&)       = delete;
            void operator=(CMissionManagerBase const&)            = delete;

        
            // Note: Scott Meyers mentions in his Effective Modern
            //       C++ book, that deleted functions should generally
            //       be public as it results in better error messages
            //       due to the compilers behavior to check accessibility
            //       before deleted status

        protected:

            CMissionManagerBase()
            {
                
            }

            
        public:
            
            ~CMissionManagerBase ()
            {

            }

        public:

            virtual void extractPlanModule (const Json_de& plan); 
        
            
            virtual void deEventFiredExternally (const std::string de_event_sid);
    
        protected:

            inline void addModuleMissionItem(std::string id, Json_de item) {
                // Move the unique_ptr into the map
                m_module_missions[id] = item;
                
            }

            
            inline void addModuleMissionItemByEvent(std::string de_event_sid, Json_de item) {
                // Move the unique_ptr into the map
                m_module_missions_by_de_events[de_event_sid] = item;
                
            }

            
            virtual std::vector<Json_de> getCommandsAttachedToMavlinkMission(const int mission_i){};


        public:
            
                inline void clearModuleMissionItems ()
                {
                    m_module_missions.clear();
                    m_module_missions_by_de_events.clear();
                }
                
                

        protected:

            std::map <std::string, Json_de> m_module_missions; // = std::map <int, std::unique_ptr<Json_de>> (new std::map <int, std::unique_ptr<Json_de>>);
            std::map <std::string, Json_de> m_module_missions_by_de_events;
    };

}
}


#endif