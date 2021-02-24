#include <iostream>
#include <common/mavlink.h>
#include "helpers/colors.h"
#include "mavlink_helper.h"
#include "mavlink_command.h"
#include "mavlink_waypoint_manager.h"





using namespace mavlinksdk;



CMavlinkWayPointManager::CMavlinkWayPointManager (mavlinksdk::CCallBack_WayPoint& callback_waypoint):m_callback_waypoint(callback_waypoint)
{
    
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
    m_callback_waypoint.onWayPointsLoadingCompleted();

}


void CMavlinkWayPointManager::saveWayPoints (std::map <int, mavlink_mission_item_int_t> mavlink_mission, const MAV_MISSION_TYPE& mission_type)
{
    m_state = WAYPOINT_STATE_WRITING_WP_COUNT;
    m_mavlink_mission = mavlink_mission;
    const int waypoint_count = mavlink_mission.size();
    m_mission_write_count = waypoint_count;
    mavlinksdk::CMavlinkCommand::getInstance().setMissionCount (waypoint_count, mission_type);
    m_state = WAYPOINT_STATE_WRITING_WP;
    writeMissionItems();
}




void CMavlinkWayPointManager::handle_mission_ack (const mavlink_mission_ack_t& mission_ack)
{
	#ifdef DEBUG
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: handle_mission_ack "  << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    switch (m_state)
    {
        case WAYPOINT_STATE_WRITING_WP_COUNT:
            m_state = WAYPOINT_STATE_WRITING_WP;
        break;

        case WAYPOINT_STATE_WRITING_ACK:
            m_callback_waypoint.onMissionSaveFinished(mission_ack.type, mission_ack.mission_type, mavlinksdk::CMavlinkHelper::getMissionACKResult (mission_ack.type));        
        break;

        default:
        break;
    }
    m_callback_waypoint.onMissionACK (mission_ack.type, mission_ack.mission_type, mavlinksdk::CMavlinkHelper::getMissionACKResult (mission_ack.type));
}


void CMavlinkWayPointManager::handle_mission_count (const mavlink_mission_count_t& mission_count)
{
    
    m_mission_count = mission_count.count;

    #ifdef DEBUG
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: m_mission_count:" << std::to_string(m_mission_count) << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

    switch (mission_count.mission_type)
    {
        case MAV_MISSION_TYPE_MISSION:
        {
            // if (m_mission_count == 0)
            // {
            //     mavlinksdk::CMavlinkCommand::getInstance().sendMissionAck();
            // }
            
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
	m_mission_current = mission_current.seq;
}


void CMavlinkWayPointManager::handle_mission_item (const mavlink_mission_item_int_t& mission_item_int)
{
    if (m_state != WAYPOINT_STATE_READ_REQUEST) 
    {
        return ; //ignore
    }

    #ifdef DEBUG
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: mission_item_int.seq " << std::to_string(mission_item_int.seq) << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    
    m_callback_waypoint.onWayPointReceived (mission_item_int);

    if (mission_item_int.mission_type == MAV_MISSION_TYPE_MISSION)
    {
            
        if (m_mission_count == mission_item_int.seq + 1)
        {
            // inform FCB that you received missions.
            std::cout << _SUCCESS_CONSOLE_TEXT_ << "Mission Received Way points count: " << std::to_string(m_mission_count) << _NORMAL_CONSOLE_TEXT_ << std::endl;
            m_state = WAYPOINT_STATE_IDLE;
            mavlinksdk::CMavlinkCommand::getInstance().sendMissionAck();
            m_callback_waypoint.onWayPointsLoadingCompleted();
        }
        else
        {
            if (m_mission_waiting_for_seq == mission_item_int.seq)
            {
                m_mission_waiting_for_seq +=1;
                mavlinksdk::CMavlinkCommand::getInstance().getWayPointByNumber(m_mission_waiting_for_seq);
            }
        }
    }
}


void CMavlinkWayPointManager::handle_mission_item_reached (const mavlink_mission_item_reached_t& mission_item_reached)
{
    m_callback_waypoint.onWaypointReached(mission_item_reached.seq);
}


void CMavlinkWayPointManager::parseMessage (const mavlink_message_t& mavlink_message)
{
    
    const int& msgid = mavlink_message.msgid;
	
    switch (msgid)
    {

        case MAVLINK_MSG_ID_MISSION_COUNT:
        {
            mavlink_mission_count_t mission_count;
            mavlink_msg_mission_count_decode (&mavlink_message, &mission_count);

            handle_mission_count (mission_count);

        }
        break;
        
        case MAVLINK_MSG_ID_MISSION_CURRENT:
        {
            mavlink_mission_current_t mission_current;
            mavlink_msg_mission_current_decode (&mavlink_message, &mission_current);

            handle_mission_current (mission_current);

        }
        break;

        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        {
            mavlink_mission_item_int_t mission_item_int;
            mavlink_msg_mission_item_int_decode (&mavlink_message, &mission_item_int);

            handle_mission_item (mission_item_int);
        }
        break;
        
        case MAVLINK_MSG_ID_MISSION_ACK:
		{
			mavlink_mission_ack_t mission_ack;
			mavlink_msg_mission_ack_decode(&mavlink_message, &mission_ack);

			handle_mission_ack(mission_ack);
		}
        break;

        case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
		{
			mavlink_mission_item_reached_t mission_item_reached;
			mavlink_msg_mission_item_reached_decode(&mavlink_message, &mission_item_reached);

			handle_mission_item_reached(mission_item_reached);
		}
        break;

    }
 
}