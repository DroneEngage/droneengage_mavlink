
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