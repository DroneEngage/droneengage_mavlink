#include <iostream>
#include "./helpers/colors.h"
#include "mavlink_helper.h"
#include "mavlink_command.h"
#include "mavlink_waypoint_manager.h"





using namespace mavlinksdk;


void CMavlinkWayPointManager::setCallbackWaypoint (mavlinksdk::CCallBack_WayPoint* callback_waypoint)
{
	m_callback_waypoint = callback_waypoint;
}


void CMavlinkWayPointManager::reloadWayPoints ()
{
    m_mission_waiting_for_seq = 0;
    m_state = WAYPOINT_STATE_READ_REQUEST;
    mavlinksdk::CMavlinkCommand::getInstance().requestMissionList();
}


void CMavlinkWayPointManager::clearWayPoints ()
{
    m_mission_waiting_for_seq = 0;
    m_state = WAYPOINT_STATE_IDLE;
    m_callback_waypoint->OnWayPointsLoadingCompleted();

}

/**
 * @brief Upload Mission to Ardupilot.
 * This is the entery function.
 * 
 * @param mavlink_mission std::map of mavlink_missions
 * @param mission_type MAV_MISSION_TYPE
 */
void CMavlinkWayPointManager::saveWayPoints (std::map <int, mavlink_mission_item_int_t> mavlink_mission, const MAV_MISSION_TYPE& mission_type)
{
    m_state = WAYPOINT_STATE_WRITING_WP_COUNT;
    m_mavlink_mission = mavlink_mission;
    const int waypoint_count = mavlink_mission.size();
    m_mission_write_count = waypoint_count;
    mavlinksdk::CMavlinkCommand::getInstance().setMissionCount (waypoint_count, mission_type);
    //m_state = WAYPOINT_STATE_WRITING_WP;
    //writeMissionItems();
    m_state = WAYPOINT_STATE_WRITING_ACK;

}




void CMavlinkWayPointManager::handle_mission_ack (const mavlink_mission_ack_t& mission_ack)
{
	#ifdef DEBUG
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: handle_mission_ack "  << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    switch (m_state)
    {
        case WAYPOINT_STATE_WRITING_WP_COUNT:
            m_state = WAYPOINT_STATE_WRITING_ACK;
        break;

        case WAYPOINT_STATE_WRITING_ACK:
            m_callback_waypoint->OnMissionSaveFinished(mission_ack.type, mission_ack.mission_type, mavlinksdk::CMavlinkHelper::getMissionACKResult (mission_ack.type));        
        break;

        default:
        break;
    }

    // check that these responses is due to ANDRUAV request not a telemetry.
    if (m_state != WAYPOINT_STATE_IDLE)
    {   
        m_callback_waypoint->OnMissionACK (mission_ack.type, mission_ack.mission_type, mavlinksdk::CMavlinkHelper::getMissionACKResult (mission_ack.type));
    }
}


void CMavlinkWayPointManager::handle_mission_count (const mavlink_mission_count_t& mission_count)
{
    
    m_mission_count = mission_count;

    #ifdef DEBUG
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: m_mission_count:" << std::to_string(m_mission_count.count) << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

    switch (mission_count.mission_type)
    {
        case MAV_MISSION_TYPE_MISSION:
        {
            if (m_mission_count.count == 0)
            {
                //mavlinksdk::CMavlinkCommand::getInstance().sendMissionAck();
                return ;
            }
            
            switch (m_state)
            {
                case WAYPOINT_STATE_READ_REQUEST:
                    mavlinksdk::CMavlinkCommand::getInstance().getWayPointByNumber(0);
                    break;
                
                default:
                    break;
            } 
            
        }
        break;

        
    };
}


void CMavlinkWayPointManager::writeMissionItems ()
{
    const std::size_t length = m_mavlink_mission.size();
    for (int i=0; i<length; ++i)
    {
        writeMissionItem(m_mavlink_mission.at(i));
    }

    m_state = WAYPOINT_STATE_WRITING_ACK;
}

/**
 * @brief write individual mission item to Ardupilot
 * 
 * @param mission_item_int 
 */
void CMavlinkWayPointManager::writeMissionItem(mavlink_mission_item_int_t& mission_item_int)
{

    mavlinksdk::CMavlinkCommand::getInstance().writeMissionItem(mission_item_int);
}

void CMavlinkWayPointManager::handle_mission_current (const mavlink_mission_current_t& mission_current)
{
    // handle_mission_item_reached detects changes
    const mavlink_mission_current_t last_mission_current = m_mission_current;
    m_mission_current = mission_current;
    if (last_mission_current.seq != mission_current.seq)
    {
        m_callback_waypoint->OnMissionCurrentChanged(mission_current);
    }
	
}


void CMavlinkWayPointManager::handle_mission_item (const mavlink_message_t& mavlink_message) //const mavlink_mission_item_int_t& mission_item_int)
{
    if (m_state != WAYPOINT_STATE_READ_REQUEST) 
    {
        return ; //ignore
    }


    mavlink_mission_item_int_t mission_item_int;
    mavlink_msg_mission_item_int_decode (&mavlink_message, &mission_item_int);


    #ifdef DEBUG
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: mission_item_int.seq " << std::to_string(mission_item_int.seq) << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    if (mission_item_int.mission_type == MAV_MISSION_TYPE_MISSION)
    {
        if (m_state == WAYPOINT_STATE_READ_REQUEST) 
        {
            m_callback_waypoint->OnWayPointReceived (mission_item_int);
        }
    
        if (m_mission_waiting_for_seq == mission_item_int.seq)
        {
            m_mission_waiting_for_seq +=1;
            m_mission_waiting_for_seq %=m_mission_count.count+1;
            
        }

        const int next_seq = mission_item_int.seq + 1;
        
        if (m_mission_count.count == next_seq)
        {
            // inform FCB that you received missions.
            std::cout << _SUCCESS_CONSOLE_TEXT_ << "Mission Received Way points count: " << std::to_string(m_mission_count.count) << _NORMAL_CONSOLE_TEXT_ << std::endl;
            mavlinksdk::CMavlinkCommand::getInstance().sendMissionAck(mavlink_message.sysid, mavlink_message.compid, MAV_MISSION_ACCEPTED);
            if (m_state == WAYPOINT_STATE_READ_REQUEST) 
            {
                m_state = WAYPOINT_STATE_IDLE;
                m_callback_waypoint->OnWayPointsLoadingCompleted();
            }
        }
        else
        {
            mavlinksdk::CMavlinkCommand::getInstance().getWayPointByNumber(next_seq);
        }
        
    }
}


void CMavlinkWayPointManager::handle_mission_item_reached (const mavlink_mission_item_reached_t& mission_item_reached)
{
    m_callback_waypoint->OnWaypointReached(mission_item_reached.seq);
}


void CMavlinkWayPointManager::handle_mission_item_request (const mavlink_mission_request_int_t& mission_request_int)
{
    #ifdef DEBUG
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: handle_mission_item_request  I AM HERE"  << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
   

    if (mission_request_int.mission_type != MAV_MISSION_TYPE_MISSION ) return ;
    if (m_mavlink_mission.size() <= mission_request_int.seq) return ;
    
    mavlinksdk::CMavlinkCommand::getInstance().writeMissionItem(m_mavlink_mission.at(mission_request_int.seq));
}


void CMavlinkWayPointManager::handle_mission_item_request (const mavlink_mission_request_t& mission_request)
{
    
    #ifdef DEBUG
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: mission_request  "  << std::to_string(mission_request.seq) << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
   
    if (mission_request.mission_type != MAV_MISSION_TYPE_MISSION ) return ;
    if (m_mavlink_mission.size() <= mission_request.seq) return ;
    
    writeMissionItem(m_mavlink_mission.at(mission_request.seq));
    m_state = WAYPOINT_STATE_WRITING_ACK;
    //mavlinksdk::CMavlinkCommand::getInstance().writeMissionItem(m_mavlink_mission.at(mission_request.seq+1));

    // mavlink_mission_item_int_t t1 = m_mavlink_mission.at(mission_request.seq);
    
    // mavlink_mission_item_t t2;
    
    // t2.target_component = t1.target_component;
    // t2.target_system = t1.target_system;
    // t2.command = t1.command;
    // t2.autocontinue = t1.autocontinue;
    // t2.current = t1.current;
    
    // t2.seq = t1.seq;
    // t2.mission_type = t1.mission_type;
    // t2.frame = t1.frame;
    
    // t2.x = t1.x / 10000000;
    // t2.y = t1.y / 10000000;
    // t2.z = t1.z;
    // t2.param1 = t1.param1;
    // t2.param2 = t1.param2;
    // t2.param3 = t1.param3;
    // t2.param4 = t1.param4;
    
    // mavlinksdk::CMavlinkCommand::getInstance().writeMissionItem(t2);
    
}


