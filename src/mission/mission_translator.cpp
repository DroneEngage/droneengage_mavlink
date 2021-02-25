#include "../helpers/colors.hpp"
#include "../helpers/json.hpp"
using Json = nlohmann::json;

#include "../messages.hpp"

#include <common/mavlink.h>
#include "mission_translator.hpp"

using namespace uavos::fcb::mission;

std::unique_ptr<std::map <int, std::unique_ptr<CMissionItem>>>  CMissionTranslator::translateMissionText (const std::string& mission_text)
{
    std::unique_ptr<std::map <int, std::unique_ptr<CMissionItem>>> mission_items = std::unique_ptr<std::map <int, std::unique_ptr<CMissionItem>>> (new std::map <int, std::unique_ptr<CMissionItem>>);
    //uavos::fcb::mission::ANDRUAV_UNIT_MISSION * andruav_mission_ptr = new (uavos::fcb::mission::ANDRUAV_UNIT_MISSION);
    std::unique_ptr <uavos::fcb::mission::ANDRUAV_UNIT_MISSION> andruav_mission = std::unique_ptr<uavos::fcb::mission::ANDRUAV_UNIT_MISSION> ();

    if (mission_text.find("QGC WPL 110") != std::string::npos)
    {
            #ifdef DEBUG
            std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: QGC WPL 110 " << _NORMAL_CONSOLE_TEXT_ << std::endl;
            #endif


    }
    else 
    {
        try
        {
            Json mission = Json::parse(mission_text);
            if (std::string(mission["fileType"]).find("Plan") != std::string::npos)
            {
                if ((mission.contains("mission") != true) || (mission["mission"].type() != Json::value_t::object))
                {
                    
                    return std::nullptr_t();
                }
                Json  homePosition = mission["mission"]["plannedHomePosition"]; 
                Json  missionlist  = mission["mission"]["items"];
                const int mission_item_count = missionlist.size();
                if (mission_item_count == 0)
                {
                    return std::nullptr_t();
                }

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
                mavlink_mission_item.x = homePosition[0].get<double>() * 10000000;
                mavlink_mission_item.y = homePosition[1].get<double>() * 10000000;
                mavlink_mission_item.z = homePosition[2].get<int>();

                uavos::fcb::mission::CMissionItem *mission_item = uavos::fcb::mission::CMissionItemBuilder::getClassByMavlinkCMD(mavlink_mission_item);
                if (mission_item != nullptr)
                {
                    mission_item->decodeMavlink (mavlink_mission_item);
                    mission_items.get()->insert(std::make_pair( mavlink_mission_item.seq, std::unique_ptr<uavos::fcb::mission::CMissionItem>(mission_item)));
                }

                // remaining messages
                for (int i=0; i< mission_item_count; ++i)
                {
                    Json mission_item_element = missionlist[i];
                    std::string type = mission_item_element["type"].get<std::string>();
                    bool auto_continue = mission_item_element["autoContinue"].get<bool>();
                    int command = mission_item_element["command"].get<int>();
                    int frame = mission_item_element["frame"].get<int>();
                    double param1 = mission_item_element["params"][0].get<double>();
                    double param2 = mission_item_element["params"][1].get<double>();
                    double param3 = mission_item_element["params"][2].get<double>();
                    double param4 = mission_item_element["params"][3].get<double>();
                    double x = mission_item_element["params"][4].get<double>();
                    double y = mission_item_element["params"][5].get<double>();
                    double z = mission_item_element["params"][6].get<double>();
                    
                    mavlink_mission_item_int_t mavlink_mission_item;
                    mavlink_mission_item.seq = i+1; // message zero is home
                    mavlink_mission_item.frame = frame;
                    mavlink_mission_item.autocontinue = auto_continue?1:0;
                    mavlink_mission_item.current = false;
                    mavlink_mission_item.command = command;
                    mavlink_mission_item.param1 = param1;
                    mavlink_mission_item.param2 = param2;
                    mavlink_mission_item.param3 = param3;
                    mavlink_mission_item.param4 = param4;
                    mavlink_mission_item.x = x * 10000000;
                    mavlink_mission_item.y = y * 10000000;
                    mavlink_mission_item.z = z;

                    uavos::fcb::mission::CMissionItem *mission_item = uavos::fcb::mission::CMissionItemBuilder::getClassByMavlinkCMD(mavlink_mission_item);
                    if (mission_item != nullptr)
                    {
                        mission_item->decodeMavlink (mavlink_mission_item);
                        mission_items.get()->insert(std::make_pair( mavlink_mission_item.seq, std::unique_ptr<uavos::fcb::mission::CMissionItem>(mission_item)));
                    }
                    
                    #ifdef DEBUG
                    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: translateMissionText " << type << " command:" << std::to_string(command) << _NORMAL_CONSOLE_TEXT_ << std::endl;
                    #endif
                }
            }

            return std::move(mission_items);
        
        }
            
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
    }
}

