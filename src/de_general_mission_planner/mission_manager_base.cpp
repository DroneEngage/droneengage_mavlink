
#include "../helpers/colors.hpp"
#include "../helpers/helpers.hpp"
#include "../global.hpp"
#include "../de_common/messages.hpp"
#include "mission_manager_base.hpp"


using namespace de::mission;

void CMissionManagerBase::extractPlanModule (const Json_de& plan)
{

try
    {
        if (std::string(plan["fileType"]).find("de_plan") != std::string::npos)
        {
            if (plan.contains("de_mission"))
            {
                // there is a waypoint data
                const Json_de de_mission = plan["de_mission"];

                Json_de modules;
                if (validateField(de_mission, "modules",Json_de::value_t::array))
                {
                    const Json_de modules = de_mission["modules"];
                    
                    for (const auto& module_mission_item : modules) {
                        std::cout << "module_mission_item:" << module_mission_item << std::endl; 
                        Json_de module_mission_item_commands = module_mission_item["c"];
                        const std::string module_linked = module_mission_item["ls"];
                        for (auto& module_mission_item_single_command : module_mission_item_commands) {

                            // link me to mission
                            module_mission_item_single_command[LINKED_TO_STEP] = module_linked;
                            
                            // check if I am waiting for event.
                            if (validateField(module_mission_item,WAITING_EVENT,Json_de::value_t::string))
                            {
                                std::string de_event_id = module_mission_item[WAITING_EVENT].get<std::string>();
                                module_mission_item_single_command[WAITING_EVENT] = de_event_id;
                                addModuleMissionItemByEvent (de_event_id, module_mission_item_single_command);
                            }
                            std::cout << "module_mission_item_single_command:" << module_mission_item_single_command.dump() << std::endl; 

                            addModuleMissionItem(module_linked, module_mission_item_single_command); 
                        }   
                    }


                    std::cout << "m_module_missions:" << m_module_missions.size() << "  m_module_missions_by_de_events:" << m_module_missions_by_de_events.size() << std::endl;
                }
            }
        }
        
    }
    catch(const std::exception& e)
    {
        std::cerr << "ss" << e.what() << '\n';
    }

}

void CMissionManagerBase::deEventFiredExternally (const std::string de_event_sid)
{
    if (m_module_missions_by_de_events.find(de_event_sid) != m_module_missions_by_de_events.end()) 
    {
        const Json_de cmd = m_module_missions_by_de_events[de_event_sid];
        std::cout << "deEventFiredExternally:" << cmd.dump() << std::endl;
    }

    return ; 
}


void CMissionManagerBase::processMyWaitingEvent()
{
    return ;
}
