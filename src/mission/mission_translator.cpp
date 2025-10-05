#include <vector>

#include <all/mavlink.h>

#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"
#include "../de_common/helpers/json_nlohmann.hpp"
using Json_de = nlohmann::json;

#include "../de_common/de_databus/messages.hpp"

#include <all/mavlink.h>
#include "mission_translator.hpp"
extern std::vector<std::string> split_string_by_newline(const std::string& str);
extern std::vector<std::string> split_string_by_delimeter(const std::string& str, const char& delimeter);

using namespace de::fcb::mission;

std::unique_ptr<std::map <int, std::unique_ptr<CMissionItem>>>   CMissionTranslator::translateQGCFormat (const std::string& mission_text)
{
    try
    {
        std::unique_ptr<std::map <int, std::unique_ptr<CMissionItem>>> mission_items = std::unique_ptr<std::map <int, std::unique_ptr<CMissionItem>>> (new std::map <int, std::unique_ptr<CMissionItem>>);
        //std::unique_ptr <de::fcb::mission::ANDRUAV_UNIT_MISSION> andruav_mission = std::unique_ptr<de::fcb::mission::ANDRUAV_UNIT_MISSION> ();

        Json_de mission = Json_de::parse(mission_text);
        if (std::string(mission["fileType"]).find("Plan") != std::string::npos)
        {
            if ((mission.contains("mission") != true) || (mission["mission"].type() != Json_de::value_t::object))
            {
                return std::nullptr_t();
            }
            
            Json_de  homePosition = mission["mission"]["plannedHomePosition"]; 
            Json_de  missionlist  = mission["mission"]["items"];
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

            de::fcb::mission::CMissionItem *mission_item = de::fcb::mission::CMissionItemBuilder::getClassByMavlinkCMD(mavlink_mission_item);
            if (mission_item != nullptr)
            {
                mission_item->decodeMavlink (mavlink_mission_item);
                mission_items.get()->insert(std::make_pair( mavlink_mission_item.seq, std::unique_ptr<de::fcb::mission::CMissionItem>(mission_item)));
            }

            // remaining messages
            for (int i=0; i< mission_item_count; ++i)
            {
                Json_de mission_item_element = missionlist[i];
                std::string type = mission_item_element["type"].get<std::string>();
                bool auto_continue = mission_item_element["autoContinue"].get<bool>();
                int command = mission_item_element["command"].get<int>();
                int frame = mission_item_element["frame"].get<int>();
                double param[7];

                for (int i=0; i<7; ++i)
                {
                    param[i] = mission_item_element["params"][i].is_null()?0:mission_item_element["params"][i].get<double>();    
                }

                    
                mavlink_mission_item_int_t mavlink_mission_item;
                mavlink_mission_item.seq = i+1; // message zero is home
                mavlink_mission_item.frame = frame;
                mavlink_mission_item.autocontinue = auto_continue?1:0;
                mavlink_mission_item.current = false;
                mavlink_mission_item.command = command;
                mavlink_mission_item.param1 = param[0];
                mavlink_mission_item.param2 = param[1];
                mavlink_mission_item.param3 = param[2];
                mavlink_mission_item.param4 = param[3];
                mavlink_mission_item.x = param[4] * 10000000;
                mavlink_mission_item.y = param[5] * 10000000;
                mavlink_mission_item.z = param[6];

                de::fcb::mission::CMissionItem *mission_item = de::fcb::mission::CMissionItemBuilder::getClassByMavlinkCMD(mavlink_mission_item);
                if (mission_item != nullptr)
                {
                    mission_item->decodeMavlink (mavlink_mission_item);
                    mission_items.get()->insert(std::make_pair( mavlink_mission_item.seq, std::unique_ptr<de::fcb::mission::CMissionItem>(mission_item)));
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

std::unique_ptr<std::map <int, std::unique_ptr<CMissionItem>>>   CMissionTranslator::translateMPFormat (const std::string& mission_text)
{
    std::unique_ptr<std::map <int, std::unique_ptr<CMissionItem>>> mission_items = std::unique_ptr<std::map <int, std::unique_ptr<CMissionItem>>> (new std::map <int, std::unique_ptr<CMissionItem>>);
    //std::unique_ptr <de::fcb::mission::ANDRUAV_UNIT_MISSION> andruav_mission = std::unique_ptr<de::fcb::mission::ANDRUAV_UNIT_MISSION> ();

    /**************
    // Mission Planner File Format
    // QGC WPL <VERSION>
    // <INDEX> <CURRENT WP> <COORD FRAME> <COMMAND> <PARAM1> <PARAM2> <PARAM3> <PARAM4> <PARAM5/X/LATITUDE> <PARAM6/Y/LONGITUDE> <PARAM7/Z/ALTITUDE> <AUTOCONTINUE>

        e.g.:
                QGC WPL 110
                0	1	0	0	0	0	0	0	0	0	0	1
                1	0	3	16	0.000000	0.000000	0.000000	0.000000	30.921076	32.431641	100.000000	1
                2	0	3	16	0.000000	0.000000	0.000000	0.000000	30.751278	32.541504	100.000000	1
                3	0	3	16	0.000000	0.000000	0.000000	0.000000	30.600094	32.651367	100.000000	1
                4	0	3	16	0.000000	0.000000	0.000000	0.000000	30.164126	32.783203	100.000000	1
                5	0	3	16	0.000000	0.000000	0.000000	0.000000	30.315988	33.178711	100.000000	1
                6	0	3	21	0.000000	0.000000	0.000000	0.000000	30.977609	33.420410	100.000000	1
         
    ***************/

    #ifdef DEBUG
        std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: QGC WPL 110 " << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    
    try
    {
        /* code */

        std::vector<std::string> v = split_string_by_newline(mission_text);
        const int length = v.size();
        if (length <= 1) 
        {
            // only QGC WPL 110 or empty
            return  std::nullptr_t();
        }
            
        for (int i=1; i < length; ++i)
        {
            std::vector<std::string> task = split_string_by_delimeter (v[i],'\t');
            if (task.size()<11) continue ; // extra line
            #ifdef DEBUG
                std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  
                << _LOG_CONSOLE_TEXT << "Line " << task.size() << _NORMAL_CONSOLE_TEXT_ << std::endl;
            #endif

            mavlink_mission_item_int_t mavlink_mission_item;
            mavlink_mission_item.seq = stoi(task[0]); //i-1; // message zero is home
            mavlink_mission_item.frame = stoi(task[2]);
            mavlink_mission_item.autocontinue = stoi(task[11]);
            mavlink_mission_item.current = stoi(task[1]);
            mavlink_mission_item.command = stoi(task[3]);
            mavlink_mission_item.param1 = std::stod(task[4]);
            mavlink_mission_item.param2 = std::stod(task[5]);
            mavlink_mission_item.param3 = std::stod(task[6]);
            mavlink_mission_item.param4 = std::stod(task[7]);
            mavlink_mission_item.x = std::stod(task[8]) * 10000000;
            mavlink_mission_item.y = std::stod(task[9]) * 10000000;
            mavlink_mission_item.z = std::stod(task[10]);
            mavlink_mission_item.mission_type = MAV_MISSION_TYPE_MISSION;

            de::fcb::mission::CMissionItem *mission_item = de::fcb::mission::CMissionItemBuilder::getClassByMavlinkCMD(mavlink_mission_item);
            if (mission_item != nullptr)
            {
                mission_item->decodeMavlink (mavlink_mission_item);
                mission_items.get()->insert(std::make_pair( mavlink_mission_item.seq, std::unique_ptr<de::fcb::mission::CMissionItem>(mission_item)));
            }
                    
        }

        return std::move(mission_items);
            
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return std::move(mission_items);
    }

}



std::unique_ptr<std::map <int, std::unique_ptr<CMissionItem>>>  CMissionTranslator::translateMissionText (const std::string& mission_text)
{
    
    if (mission_text.find("QGC WPL 110") != std::string::npos)
    {
        return translateMPFormat (mission_text);
        
    }
    else 
    {
        Json_de mission = Json_de::parse(mission_text);
        if (validateField(mission, "fileType",Json_de::value_t::string)) 
        {
            return translateQGCFormat (mission_text);
        }
    

        
    }
}

