
#include "fcb_swarm_leader.hpp"
#include "../fcb_facade.hpp"
#include "../helpers/helpers.hpp"

using namespace uavos::fcb::swarm; 

/**
 * @brief This function is called from Schdular in rate 10Hz.
 * This function is executed by the leader. The leader should deceide what data is sent to which drone.
 */
void CSwarmManager::handle_swarm_as_leader()
{
    
    static u_int64_t previous = 0;
    if (!m_is_leader) return ;
    
    const u_int64_t now = get_time_usec();
    if ((now - previous) > 1000000)
    {
        mavlinksdk::CVehicle &vehicle =  mavlinksdk::CVehicle::getInstance();
        
        // loop on followers unit
        for (const auto& item : m_follower_units) 
        {
            

            mavlink_message_t mavlink_message[2];
        
            const int sys_id = vehicle.getSysId();
            const int comp_id = vehicle.getCompId();

            // get my attitude and position to share with followers.
            const mavlink_attitude_t& attitude = vehicle.getMsgAttitude();
            const mavlink_global_position_int_t&  my_gpos = vehicle.getMsgGlobalPositionInt();

            // time_boot_ms - lat - lon - alt - relative - alt - vx - vy - vz - hdg
            mavlink_msg_global_position_int_encode(sys_id, comp_id, &mavlink_message[0], &my_gpos);
            // roll - pitch - yaw - rollspeed - pitchspeed - yawspeed
            mavlink_msg_attitude_encode(sys_id, comp_id, &mavlink_message[1], &attitude);
            
            // forward info to followers.
            uavos::fcb::CFCBFacade::getInstance().sendSWARM_M (item.party_id, mavlink_message, 2);
            
        }

        previous = now;
    }
}