#ifndef WAYPOINT_MANAGER_H_
#define WAYPOINT_MANAGER_H_
#include <map>

#include <common/mavlink.h>
#include <ardupilotmega/ardupilotmega.h>



namespace mavlinksdk
{

#define WAYPOINT_STATE_IDLE                 0
#define WAYPOINT_STATE_READ_REQUEST         1
#define WAYPOINT_STATE_READ_WP              2    
#define WAYPOINT_STATE_WRITING_WP_COUNT     3    
#define WAYPOINT_STATE_WRITING_WP           4    
#define WAYPOINT_STATE_WRITING_ACK          5


/**
 * @brief This class manages waypoints
 * 
 */
class CCallBack_WayPoint
{
    public:

    virtual void OnWaypointReached(const int& sequence)                                                             {};
    virtual void OnWayPointsLoadingCompleted()                                                                      {}; 
    virtual void OnMissionACK (const int& result, const int& mission_type, const std::string& result_msg)           {};
    virtual void OnMissionSaveFinished (const int& result, const int& mission_type, const std::string& result_msg)  {};
    virtual void OnWayPointReceived (const mavlink_mission_item_int_t& mission_item_int)                            {};
};

class CMavlinkWayPointManager
{
    public:
            //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
            static CMavlinkWayPointManager& getInstance()
            {
                static CMavlinkWayPointManager instance;
                return instance;
            }

            CMavlinkWayPointManager(CMavlinkWayPointManager const&)               = delete;
            void operator=(CMavlinkWayPointManager const&)                        = delete;

        
            // Note: Scott Meyers mentions in his Effective Modern
            //       C++ book, that deleted functions should generally
            //       be public as it results in better error messages
            //       due to the compilers behavior to check accessibility
            //       before deleted status

    private:

        CMavlinkWayPointManager() {};

    public:
            
        ~CMavlinkWayPointManager ()
        {

        }

        
    public:
        
    
        void setCallbackWaypoint (mavlinksdk::CCallBack_WayPoint* callback_waypoint);
        inline int getCurrentStep () const { return m_mission_current;}

    public:

            void reloadWayPoints();
            void clearWayPoints();
            void saveWayPoints (std::map <int, mavlink_mission_item_int_t> mavlink_mission, const MAV_MISSION_TYPE& mission_type);

            void handle_mission_ack   (const mavlink_mission_ack_t& mission_ack);
            void handle_mission_count (const mavlink_mission_count_t& mission_count);
            void handle_mission_current   (const mavlink_mission_current_t& mission_current);
            void handle_mission_item (const mavlink_message_t& mavlink_message); //const mavlink_mission_item_int_t& mission_item_int);
            void handle_mission_item_reached (const mavlink_mission_item_reached_t& mission_item_reached);
            void handle_mission_item_request (const mavlink_mission_request_int_t& mission_request_int);
            void handle_mission_item_request (const mavlink_mission_request_t& mission_request);

    protected:
            void writeMissionItems ();  
            void writeMissionItem(mavlink_mission_item_int_t& mission_item_int);

    // Class Members
    protected:
        mavlinksdk::CCallBack_WayPoint* m_callback_waypoint;

        uint16_t  m_mission_current = 0;
        uint16_t  m_mission_count   = 0;
        uint16_t  m_mission_write_count   = 0;
        uint16_t  m_mission_write_current   = 0;
        uint16_t  m_mission_waiting_for_seq   = 0;
        std::map <int, mavlink_mission_item_int_t> m_mavlink_mission;

        int m_state = WAYPOINT_STATE_IDLE;
        
      
};

}


#endif