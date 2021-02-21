#ifndef MISSIONS_H_
#define MISSIONS_H_

#include <map>
#include <memory>


typedef enum ANDRUAV_MISSION_TYPE
{
        ANDRUAV_MISSION_UNKNOWN             = 0,
        ANDRUAV_MISSION_ARDUPILOT_WAYPOINTS = 1
} ANDRUAV_MISSION_TYPE;


class CMissionItem 
{
    public:
        int m_sequence;
        bool m_auto_continue;
        /**
         * @brief local: x position in meters * 1e4, global: latitude in degrees * 10^7
         * 
         */
        int x,y,z;


};

class CWayPoint_Step : public CMissionItem
{
    public:
        CWayPoint_Step (){};
        explicit CWayPoint_Step (const int& sequence)
        {
            m_sequence = sequence;
        }


    
};


/**
 * @brief holds andruav missions
 * Andruav missions can be ardupilot mission or andruav mission.
 * 
 * * Andruav Mission can contain swarm and waiting event actions.
 * * Ardupilot mission can contains embedded commands but no need 
 * * to save them here as they are not handled by Andruav right now.
 */
typedef struct ANDRUAV_UNIT_MISSION
{
    ANDRUAV_MISSION_TYPE mission_type;
    std::map <int, std::unique_ptr<CMissionItem>> mission_items;

    void clear ()
    {
        mission_type = ANDRUAV_MISSION_UNKNOWN;
        mission_items.clear();
    }

} ANDRUAV_UNIT_MISSION;


#endif