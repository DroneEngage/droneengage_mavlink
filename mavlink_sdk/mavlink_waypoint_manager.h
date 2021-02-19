#ifndef WAYPOINT_MANAGER_H_
#define WAYPOINT_MANAGER_H_

#include "mavlink_helper.h"

namespace mavlinksdk
{



class CCallBack_WayPoint
{
    public:

    virtual void onWaypointReached(const mavlink_mission_item_reached_t& mission_item_reached)        {};
    virtual void onCurrentWaypointUpdated() {};
    virtual void onWayPointsLoadingCompleted() {}; 
    virtual void OnMissionACK (const int& result, const int& mission_type, const std::string& result_msg) {};
        
};

class CMavlinkWayPointManager
{
    public:
        
        explicit CMavlinkWayPointManager(mavlinksdk::CCallBack_WayPoint& callback_waypoint);
        ~CMavlinkWayPointManager() {};
    
        void parseMessage (const mavlink_message_t& mavlink_message);
    

    public:

            void loadWayPoints();


    protected:
            
            inline void handle_mission_ack   (const mavlink_mission_ack_t& mission_ack);
            inline void handle_mission_count (const mavlink_mission_count_t& mission_count);
            inline void handle_mission_current   (const mavlink_mission_current_t& mission_current);
            inline void handle_mission_item (const mavlink_mission_item_int_t& mission_item_int);
            inline void handle_mission_item_reached (const mavlink_mission_item_reached_t& mission_item_reached);


    // Class Members
    protected:
        mavlinksdk::CCallBack_WayPoint& m_callback_waypoint;
         

};

}


#endif