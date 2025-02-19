
#include <vector>

#include <all/mavlink.h>

#include "../helpers/colors.hpp"
#include "../helpers/helpers.hpp"
#include "../helpers/json_nlohmann.hpp"
using Json_de = nlohmann::json;

#include <mavlink_waypoint_manager.h>

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
    CMissionTranslator cMissionTranslator;

    std::unique_ptr<std::map <int, std::unique_ptr<mission::CMissionItem>>> new_mission_items = cMissionTranslator.translateMissionText(plan_text);
    
    if (new_mission_items == std::nullptr_t())
    {
        CFCBFacade::getInstance().sendErrorMessage(std::string(), 0, ERROR_3DR, NOTIFICATION_TYPE_ERROR, "Bad input plan file");
        return ;
    }
    
    clearWayPoints();
    mission::ANDRUAV_UNIT_MISSION& andruav_missions = de::fcb::mission::CMissionManager::getInstance().getAndruavMission();                
                
             
    std::map<int, std::unique_ptr<mission::CMissionItem>>::iterator it;
    for (it = new_mission_items->begin(); it != new_mission_items->end(); it++)
    {
        int seq = it->first;
                    
        andruav_missions.mission_items.insert(std::make_pair( seq, std::move(it->second)));
    }

    new_mission_items->clear();
                
    saveWayPointsToFCB();                
}


/**
 * @brief clear way points from andruav and fcb.
 * * In FCB it clears all types of maps.
 * * But in andruav geo fence is not affected by this command.
 *
 */
void CMissionManager::clearWayPoints()
{
    
    clearMissionItems();
    m_andruav_missions.clear();
    mavlinksdk::CMavlinkCommand::getInstance().clearWayPoints();
    
    return;
}

/**
 * @brief Clear Mission from DroneEngage NOT Ardupilot Flight Controller.
 */
void CMissionManager::clearMissionItems ()
{
    m_event_waiting_for = "";
    m_event_waiting_for_has_processed = true;
    m_mavlink_event_waiting_for = 0;
    m_mavlink_mission_item_last_seq =0;

    m_mission_items.clear();

    if (m_event_fire_channel != AP_EVENT_DISABLED)
    {
        mavlinksdk::CMavlinkCommand::getInstance().setServo (m_event_fire_channel, AP_EVENT_DISABLED);
    }
    
    if (m_event_wait_channel != AP_EVENT_DISABLED)
    {
        mavlinksdk::CMavlinkCommand::getInstance().setServo (m_event_wait_channel, AP_EVENT_DISABLED);
    }
    
    m_event_fired_by_me = -1; // nothing fired
}


/**
 * @brief Upload mission for DE Mission File
 * 
 * @param plan 
 */
void CMissionManager::uploadMissionIntoSystem2(const Json_de& plan)
{
    extractPlanMavlinkMission(plan);

    extractPlanModule(plan);

    if (m_mission_items.size()==0)
    {
        CFCBFacade::getInstance().sendErrorMessage(std::string(), 0, ERROR_3DR, NOTIFICATION_TYPE_ERROR, "Bad input plan file");
        return ;
    }

    mission::ANDRUAV_UNIT_MISSION& andruav_missions = de::fcb::mission::CMissionManager::getInstance().getAndruavMission();                

    std::map<int, std::unique_ptr<mission::CMissionItem>>::iterator it;
    for (it = m_mission_items.begin(); it != m_mission_items.end(); it++)
    {
        int seq = it->first;
                    
        andruav_missions.mission_items.insert(std::make_pair( seq, std::move(it->second)));
    }

    saveWayPointsToFCB();       
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
                        mavlink_mission_item.autocontinue = 1;
                        mavlink_mission_item.current = false;
                        mavlink_mission_item.command = MAV_CMD_NAV_WAYPOINT;
                        mavlink_mission_item.param1 = 0;
                        mavlink_mission_item.param2 = 0;
                        mavlink_mission_item.param3 = 0;
                        mavlink_mission_item.param4 = 0;
                        if (home.contains("lat")
                            && home.contains("lng")
                            && home.contains("alt"))
                        {
                            mavlink_mission_item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
                            mavlink_mission_item.x = home["lat"].get<double>() * 10000000;
                            mavlink_mission_item.y = home["lng"].get<double>() * 10000000;
                            mavlink_mission_item.z = home["alt"].get<int>();
                        }
                        else
                        {
                            mavlinksdk::CVehicle &vehicle =  mavlinksdk::CVehicle::getInstance();
                            const mavlink_home_position_t& home_pos = vehicle.getMsgHomePosition();
        
                            mavlink_mission_item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
                            mavlink_mission_item.x = home_pos.latitude * 10000000;
                            mavlink_mission_item.y = home_pos.longitude * 10000000;
                            mavlink_mission_item.z = home_pos.altitude;
                        }
                            
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
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}


void CMissionManager::extractPlanModule (const Json_de& plan) 
{
    // do nothing.
}
                
/**
 * @brief
 *  add mission-id event to event list.
 *  dependent paused mission should be fired as a response.
 *  Important: Waiting events are created as mission items, so they are not listed
 *  here since the beginning of the mission.
 *  What we store ares the fired events that we keep test them to resume a waiting event.
 *  This is the OPPOSITE of Communication Module logic where DE events are pre-initialized.
 * @see CMissionManager::processMyWaitingEvent
 */
void CMissionManager::deEventFiredExternally(const std::string de_event_sid) 
{
    //TODO:: In future you may determine if the event will be used or not, so unneeded events dont need to be stored.
    if (std::find(m_event_received_from_others.begin(), m_event_received_from_others.end(), de_event_sid) == m_event_received_from_others.end())
    {
        m_event_received_from_others.push_back(de_event_sid);

        std::cout << _INFO_CONSOLE_TEXT << "Event Received (new): " << de_event_sid << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }

    // processMyWaitingEvent is called elsewhere and it will use this stored event to resume missions if needed.
}

void CMissionManager::deEventFiredInternally(const std::string de_event_sid) 
{
    deEventFiredExternally (de_event_sid);
}



/**
 * @brief checks if there is an action depends on an event and resumes it.
 *  This is the executor function where a waiting mission way-point resumes.
 * 
 */
void CMissionManager::processMyWaitingEvent()
{
    if (m_event_waiting_for_has_processed) return ;

    std::vector<std::string>::iterator it = std::find(m_event_received_from_others.begin(), m_event_received_from_others.end(), m_event_waiting_for);

    if (it != m_event_received_from_others.end())
    { // event that is [m_event_waiting_for] is fired by another unit or module found 
        
        std::cout << "event: " << m_event_waiting_for << " : found" << std::endl;

        if (de::fcb::CFCBMain::getInstance().getAndruavVehicleInfo().flying_mode == VEHICLE_MODE_AUTO)
        {
            // +3 because setServo & Delay does not appear in Mission Item Reached
            //      - SetServo (WAIT FOR EVENT)
            //      - MAV_CMD_NAV_DELAY()
            //      - Next Mission.
            // TODO: replace with break mode unless delay is used for timeout.
            const ANDRUAV_VEHICLE_INFO& andruav_vehicle_info = de::fcb::CFCBMain::getInstance().getAndruavVehicleInfo();
            
            // Iterating using range-based for loop
            for (const auto& pair : m_andruav_missions.mission_items) {
                const int key = pair.first;
                
                // Skip early missions.
                // remember the logic is: we are waiting for a single event to continue... 
                // .. not trigerringdifferent missions based on events.
                if (key < andruav_vehicle_info.current_waypoint) continue;
                
                const CMissionItem* item = pair.second.get();
                
                if (item->m_mission_command == MAV_CMD_DO_SET_SERVO)
                {
                    mavlink_mission_item_int_t cmd = item->getArdupilotMission();
                    if (cmd.param1 !=  m_event_wait_channel) 
                    {
                        // Not a SYNC Event
                        continue;
                    }
                    
                    if (cmd.param2 !=  std::stof(m_event_waiting_for)) 
                    {   // not the event we are looking for.
                        continue;
                    }
                    
                    m_event_waiting_for_has_processed = true;
                
                    mavlinksdk::CMavlinkCommand::getInstance().setCurrentMission(item->m_sequence+2);
                    
                    std::cout << "Continue from task Key: " << key << std::endl ;

                    // erase it 
                    m_event_received_from_others.erase(it);

                    return;
                }
                
                std::cout << "mission3: skip" << key << std::endl;
                
            }
        }
    }
}



/**
 * @brief Process RCOut signals to extract SYNC Event sent by Ardupilot.
 *  This event is FIRED by the board to other modules and units.
 */
void CMissionManager::readFiredEventFromFCB(const mavlink_servo_output_raw_t &servo_output_raw)
{
    // no need to get here with every event.
    int event_value;
    switch (m_event_fire_channel)
    {
    case 11:
        event_value = servo_output_raw.servo11_raw;
        break;
    case 12:
        event_value = servo_output_raw.servo12_raw;
        break;
    case 13:
        event_value = servo_output_raw.servo13_raw;
        break;
    case 14:
        event_value = servo_output_raw.servo14_raw;
        break;
    case 15:
        event_value = servo_output_raw.servo15_raw;
        break;
    case 16:
        event_value = servo_output_raw.servo16_raw;
        break;
    default:
        // Waiting event is disabled.
        return ;
    }

    if (m_event_fired_by_me != event_value)
    {   // event is not in the list ... i.e. has not been fired yet....
        m_event_fired_by_me = event_value;
        CFCBFacade::getInstance().sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_WARNING, std::string("Trigger Event:") + std::to_string(event_value));
        const std::string de_event_sid = std::to_string(event_value);

        //IMPORTANT: this is a event from servo channel. You need to broadcast it globally.
        CFCBFacade::getInstance().sendSyncFireEvent(ANDRUAV_PROTOCOL_SENDER_ALL_AGENTS, de_event_sid, false);
        
        de::fcb::mission::CMissionManager::getInstance().deEventFiredInternally(de_event_sid);
        std::cout << _INFO_CONSOLE_TEXT << "Event Triggered: " << std::to_string(event_value) << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }
}


void CMissionManager::handleMissionCurrentCount(const mavlink_mission_current_t &mission_current)
{
    // IMPORTANT: Ardupilot sends seq of mission items not mission commands.
    
    const uint16_t seq = mission_current.seq ;
    if (m_mavlink_mission_item_last_seq < seq)
    {
        for (uint16_t i=m_mavlink_mission_item_last_seq; i<= seq; ++i)
        {
            CFCBFacade::getInstance().sendMissionItemSequence(std::to_string(i));
        }

        m_mavlink_mission_item_last_seq = seq;
    }
}

/**
 * @brief Process RCOut signals to extract SYNC Event sent by Ardupilot.
 * The board is WAITING from other modules or units.
 */
void CMissionManager::readWaitingEventFromFCB(const mavlink_servo_output_raw_t &servo_output_raw)
{
    // Events that I am waiting for from other drones or modules.
    int event_value;
    switch (m_event_wait_channel)
    {
    case 11:
        event_value = servo_output_raw.servo11_raw;
        break;
    case 12:
        event_value = servo_output_raw.servo12_raw;
        break;
    case 13:
        event_value = servo_output_raw.servo13_raw;
        break;
    case 14:
        event_value = servo_output_raw.servo14_raw;
        break;
    case 15:
        event_value = servo_output_raw.servo15_raw;
        break;
    case 16:
        event_value = servo_output_raw.servo16_raw;
        break;
    default:
        return ;
    }

    if (m_mavlink_event_waiting_for != event_value)
    {   // new event has been fired.
        m_mavlink_event_waiting_for = event_value;
        de::fcb::mission::CMissionManager::getInstance().setCurrentWaitingEvent(std::to_string(event_value));
        CFCBFacade::getInstance().sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_WARNING, std::string("Wait Event:") + std::to_string(m_mavlink_event_waiting_for));
    }
    
    // Adding fired event to the list of fired events waiting to be processed
    // when fcb fire the correspondent waiting event.
    de::fcb::mission::CMissionManager::getInstance().processMyWaitingEvent();
}


/**
 * @brief uploads mavlink mission items to FCB.
 *
 */
void CMissionManager::saveWayPointsToFCB()
{
    // m_andruav_missions.clear();
    // m_fcb_facade.sendWayPoints(std::string());

    const std::size_t length = m_andruav_missions.mission_items.size();
    if (length == 0)
    {
        // ignore
        return;
    }

    std::map<int, mavlink_mission_item_int_t> mavlink_mission;
    for (int i = 0; i < length; ++i)
    {
        mavlink_mission.insert(std::make_pair(i, m_andruav_missions.mission_items.at(i).get()->getArdupilotMission()));
    }

    mavlinksdk::CMavlinkWayPointManager::getInstance().saveWayPoints(mavlink_mission, MAV_MISSION_TYPE_MISSION);

    return;
}


void CMissionManager::reloadWayPoints()
{
    m_andruav_missions.clear();
    mavlinksdk::CMavlinkCommand::getInstance().reloadWayPoints();

    return;
}