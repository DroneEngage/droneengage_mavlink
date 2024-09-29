
#include <vector>

#include <all/mavlink.h>

#include "../helpers/colors.hpp"
#include "../helpers/helpers.hpp"
#include "../helpers/json_nlohmann.hpp"
using Json_de = nlohmann::json;

#include "../de_common/messages.hpp"

#include "mission_translator.hpp"
#include "mission_manager.hpp"
#include "../geofence/fcb_geo_fence_manager.hpp"
#include "../fcb_facade.hpp"
#include "../fcb_main.hpp"

using namespace de::fcb::mission;

/**
 * @brief only Mavlink Mission Upload for Mission Planner & QGC files.
 * 
 * @param plan_text 
 */
void CMissionManager::uploadMissionIntoSystem(const std::string& plan_text)
{
    de::fcb::CFCBMain&  fcb_main = de::fcb::CFCBMain::getInstance();
            
    CMissionTranslator cMissionTranslator;

    std::unique_ptr<std::map <int, std::unique_ptr<mission::CMissionItem>>> new_mission_items = cMissionTranslator.translateMissionText(plan_text);
    
    if (new_mission_items == std::nullptr_t())
    {
        CFCBFacade::getInstance().sendErrorMessage(std::string(), 0, ERROR_3DR, NOTIFICATION_TYPE_ERROR, "Bad input plan file");
        return ;
    }
    
    fcb_main.clearWayPoints();
    mission::ANDRUAV_UNIT_MISSION& andruav_missions = fcb_main.getAndruavMission();                
                
             
    std::map<int, std::unique_ptr<mission::CMissionItem>>::iterator it;
    for (it = new_mission_items->begin(); it != new_mission_items->end(); it++)
    {
        int seq = it->first;
                    
        andruav_missions.mission_items.insert(std::make_pair( seq, std::move(it->second)));
    }

    new_mission_items->clear();
                
    fcb_main.saveWayPointsToFCB();                
}

/**
 * @brief Upload mission for DE Mission File
 * 
 * @param plan 
 */
void CMissionManager::uploadMissionIntoSystem2(const Json_de& plan)
{
    de::fcb::CFCBMain&  fcb_main = de::fcb::CFCBMain::getInstance();
            
    extractPlanMavlinkMission(plan);

    extractPlanModule(plan);

    if (m_mission_items.size()==0)
    {
        CFCBFacade::getInstance().sendErrorMessage(std::string(), 0, ERROR_3DR, NOTIFICATION_TYPE_ERROR, "Bad input plan file");
        return ;
    }

    fcb_main.clearWayPoints();
    mission::ANDRUAV_UNIT_MISSION& andruav_missions = fcb_main.getAndruavMission();                

    std::map<int, std::unique_ptr<mission::CMissionItem>>::iterator it;
    for (it = m_mission_items.begin(); it != m_mission_items.end(); it++)
    {
        int seq = it->first;
                    
        andruav_missions.mission_items.insert(std::make_pair( seq, std::move(it->second)));
    }

    fcb_main.saveWayPointsToFCB();       
}

void CMissionManager::extractPlanMavlinkMission (const Json_de& plan)
{
try
    {
        clearMissionItems();
        
        if (std::string(plan["fileType"]).find("de_plan") != std::string::npos)
        {
            if (plan.contains("de_mission"))
            {
                if (validateField(plan, "unit",Json_de::value_t::object))
                {
                    const Json_de unit = plan["unit"];

                    if (validateField(unit, "home",Json_de::value_t::object))
                    {
                        const Json_de home = unit["home"];

                        // mission zero is home position
                        mavlink_mission_item_int_t mavlink_mission_item;
                        mavlink_mission_item.seq = 0; // message zero is home
                        mavlink_mission_item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
                        mavlink_mission_item.autocontinue = 1;
                        mavlink_mission_item.current = false;
                        mavlink_mission_item.command = MAV_CMD_NAV_WAYPOINT;
                        mavlink_mission_item.param1 = 0;
                        mavlink_mission_item.param2 = 0;
                        mavlink_mission_item.param3 = 0;
                        mavlink_mission_item.param4 = 0;
                        mavlink_mission_item.x = home["lat"].get<double>() * 10000000;
                        mavlink_mission_item.y = home["lng"].get<double>() * 10000000;
                        mavlink_mission_item.z = home["alt"].get<int>();

                        de::fcb::mission::CMissionItem *mission_item = de::fcb::mission::CMissionItemBuilder::getClassByMavlinkCMD(mavlink_mission_item);
                        if (mission_item != nullptr)
                        {
                            mission_item->decodeMavlink (mavlink_mission_item);
                            addMissionItem(mavlink_mission_item.seq, std::unique_ptr<de::fcb::mission::CMissionItem>(mission_item));
                        }
                    }
                }
                // there is a waypoint data
                const Json_de de_mission = plan["de_mission"];
                
                if (validateField(de_mission, "mav_waypoints",Json_de::value_t::array))
                {
                    const Json_de waypoints = de_mission["mav_waypoints"];
                    
                    int item_id = 0;
                    for (const auto& waypoint : waypoints) {
                        const Json_de mavlink_params = waypoint["mv"];
                        item_id++;
                        mavlink_mission_item_int_t mavlink_mission_item;
                        mavlink_mission_item.seq = item_id;
                        mavlink_mission_item.frame = waypoint["ft"].get<int>();
                        mavlink_mission_item.autocontinue = 0;  // BUG: ???
                        mavlink_mission_item.current = (item_id == 1)?1:0;
                        mavlink_mission_item.command = waypoint["c"].get<int>();       // 183
                        mavlink_mission_item.param1 = mavlink_params[0].get<double>(); // [15,16]
                        mavlink_mission_item.param2 = mavlink_params[1].get<double>(); // event number
                        mavlink_mission_item.param3 = mavlink_params[2].get<double>();
                        mavlink_mission_item.param4 = mavlink_params[3].get<double>();
                        mavlink_mission_item.x = mavlink_params[4].get<double>() * 10000000;
                        mavlink_mission_item.y = mavlink_params[5].get<double>() * 10000000;
                        mavlink_mission_item.z = mavlink_params[6].get<double>();
                        mavlink_mission_item.mission_type = MAV_MISSION_TYPE_MISSION;

                        de::fcb::mission::CMissionItem *mission_item = de::fcb::mission::CMissionItemBuilder::getClassByMavlinkCMD(mavlink_mission_item);
                        if (mission_item != nullptr)
                        {
                            mission_item->decodeMavlink (mavlink_mission_item);
                            addMissionItem(mavlink_mission_item.seq, std::unique_ptr<de::fcb::mission::CMissionItem>(mission_item));
                        }
                    }
                }
            }
        }
        return ;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}