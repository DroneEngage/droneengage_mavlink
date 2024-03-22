
#include "fcb_swarm_leader.hpp"
#include "../fcb_facade.hpp"
#include "../helpers/helpers.hpp"

using namespace uavos::fcb::swarm; 

#define DEF_SWARM_LEADER_LOCATION_UPDATE_RATE 1000000



void CSwarmLeader::updateFollowersThreadFormation()
{
    mavlinksdk::CVehicle &vehicle =  mavlinksdk::CVehicle::getInstance();
        
    // get my attitude and position to share with followers.
    const mavlink_attitude_t& attitude = vehicle.getMsgAttitude();
    const mavlink_global_position_int_t&  my_gpos = vehicle.getMsgGlobalPositionInt();

    // loop on followers unit
    const std::vector<ANDRUAV_UNIT_FOLLOWER>& follower_units = m_fcb_swarm_manager.getFollowerUnits();
    for (const auto& item : follower_units) 
    {
        mavlink_message_t mavlink_message[2];
        
        const int sys_id = vehicle.getSysId();
        const int comp_id = vehicle.getCompId();

        // time_boot_ms - lat - lon - alt - relative - alt - vx - vy - vz - hdg
        mavlink_msg_global_position_int_encode(sys_id, comp_id, &mavlink_message[0], &my_gpos);
        // roll - pitch - yaw - rollspeed - pitchspeed - yawspeed
        mavlink_msg_attitude_encode(sys_id, comp_id, &mavlink_message[1], &attitude);
            
        // forward info to followers.
        uavos::fcb::CFCBFacade::getInstance().sendSWARM_M (item.party_id, mavlink_message, 2);
    }
} 


/**
 * @brief Send instructions for followers based on 
 * my active formation as a leader.
 * 
 */
void CSwarmLeader::updateFollowers()
{
    switch (m_fcb_swarm_manager.getFormationAsLeader())
    {
        case FORMATION_THREAD:
            updateFollowersThreadFormation();
            break;
        case FORMATION_VECTOR:
            break;
        case FORMATION_VECTOR_180:
            break;
        
        default:
            break;
    }
}


/**
 * @brief This function is called from Schdular in rate 10Hz.
 * This function is executed by the leader. The leader should deceide what data is sent to which drone.
 * IMPORTANT: Some modes the calculation of the folloer location is determined by follower to reduce 
 *      calculation effort on leader. Other formation may require leader to calculate locations of followers.
 */
void CSwarmLeader::handleSwarmsAsLeader()
{
    
    static u_int64_t previous = 0;
    if (!m_fcb_swarm_manager.isLeader()) return ;
    
    const u_int64_t now = get_time_usec();
    if ((now - previous) > DEF_SWARM_LEADER_LOCATION_UPDATE_RATE)
    {
        updateFollowers();
        
        previous = now;
    }
}