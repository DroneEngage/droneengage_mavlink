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
        
            virtual void fireEvent (const std::string fire_event);

            virtual void mavlinkMissionItemStartedEvent (const int mission_id);

        
        protected:

            inline void addModuleMissionItem(std::string id, std::unique_ptr<Json_de> item) {
                // Move the unique_ptr into the map
                m_module_missions[id] = std::move(item);
                
            }

            
            virtual std::vector<Json_de> getCommandsAttachedToMavlinkMission(const int mission_i){};


        public:
            
                inline void clearModuleMissionItems ()
                {
                    m_module_missions.clear();
                }
                
                

        protected:

            std::map <std::string, std::unique_ptr<Json_de>> m_module_missions; // = std::map <int, std::unique_ptr<Json_de>> (new std::map <int, std::unique_ptr<Json_de>>);
    };

}
}


#endif