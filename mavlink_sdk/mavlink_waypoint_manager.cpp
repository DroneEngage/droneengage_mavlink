#include <iostream>

#include <common/mavlink.h>
#include "mavlink_helper.h"
#include "mavlink_waypoint_manager.h"









mavlinksdk::CMavlinkWayPointManager::CMavlinkWayPointManager (mavlinksdk::CCallBack_WayPoint& callback_waypoint):m_callback_waypoint(callback_waypoint)
{
    
}



void mavlinksdk::CMavlinkWayPointManager::loadWayPoints ()
{
    
    m_callback_waypoint.onWayPointsLoadingCompleted();
}


void mavlinksdk::CMavlinkWayPointManager::handle_mission_ack (const mavlink_mission_ack_t& mission_ack)
{
	m_callback_waypoint.OnMissionACK (mission_ack.type, mission_ack.mission_type, mavlinksdk::CMavlinkHelper::getMissionACKResult (mission_ack.type));
}

void mavlinksdk::CMavlinkWayPointManager::handle_mission_count (const mavlink_mission_count_t& mission_count)
{
	
}


void mavlinksdk::CMavlinkWayPointManager::handle_mission_current (const mavlink_mission_current_t& mission_current)
{
	
}

void mavlinksdk::CMavlinkWayPointManager::handle_mission_item (const mavlink_mission_item_int_t& mission_item_int)
{

}


void mavlinksdk::CMavlinkWayPointManager::handle_mission_item_reached (const mavlink_mission_item_reached_t& mission_item_reached)
{
    m_callback_waypoint.onWaypointReached(mission_item_reached);
}

void mavlinksdk::CMavlinkWayPointManager::parseMessage (const mavlink_message_t& mavlink_message)
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

        case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
		{
			mavlink_mission_item_reached_t mission_item_reached;
			mavlink_msg_mission_item_reached_decode(&mavlink_message, &mission_item_reached);

			handle_mission_item_reached(mission_item_reached);
		}

    }
}