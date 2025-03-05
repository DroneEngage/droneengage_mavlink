
#include "fcb_swarm_follower.hpp"
#include "fcb_swarm_manager.hpp"
#include "../fcb_facade.hpp"
#include "../fcb_main.hpp"
#include "../helpers/gps.hpp"
#include "../helpers/helpers.hpp"

#include "fcb_swarm_manager.hpp"

using namespace de::fcb::swarm;


de::fcb::swarm::CSwarmManager& fcb_swarm_manager = de::fcb::swarm::CSwarmManager::getInstance();
    

/**
 * @brief Logic of thread formation for follower in implemented here.
 * Leader sends it own location, and I know the formation is Thread-Formation and my index into it 
 * and I generate my position accordingly.
 * Note that another formation my receive exact location of the unit. 
 * The logic varies from a formation to another.
 */
void CSwarmFollower::updateFollowerInThreadFormation()
{
    const uint32_t KNODE_LENGTH = 100; 

    // get my own location
    mavlinksdk::CVehicle &vehicle =  mavlinksdk::CVehicle::getInstance();
    const mavlink_global_position_int_t&  my_gpos = vehicle.getMsgGlobalPositionInt();
    const double leader_lat = m_leader_gpos_new.lat / 10000000.0f;
    const double leader_lon = m_leader_gpos_new.lon / 10000000.0f;

    // get current time
    const u_int64_t now = get_time_usec();

    #ifdef DEBUG
        std::cout << _INFO_CONSOLE_TEXT << "time_diff: " <<  ":" << (m_leader_last_access - now) << ":" <<_NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
                   
    // test if leader speed is very low then break.
    double speed_sq = m_leader_gpos_new.vx * m_leader_gpos_new.vx + m_leader_gpos_new.vy * m_leader_gpos_new.vy;
    if (speed_sq < 100) 
    {
        return ;
    }

    const double my_lat = my_gpos.lat / 10000000.0f;
    const double my_lon = my_gpos.lon / 10000000.0f;

    const int follower_index = fcb_swarm_manager.getFollowerIndex();
    const double base_distance = (follower_index + 1) * KNODE_LENGTH; // Base distance from leader

    // distance between me & leader
    const double distance_to_leader = calcGPSDistance(leader_lat,leader_lon, my_lat, my_lon);

    // the rope effect ... ignore when distance is less than robe length. Rope has nodes and each node has a length. of KNODE_LENGTH
    if (distance_to_leader <  base_distance)  
    {
        // rope effect ... ignore when distance is less than robe length.
        return ;
    }

    
                    
    const double leader_velocity_vector_bearing = getBearingOfVector (m_leader_gpos_new.vx, m_leader_gpos_new.vy); // bearing of leader velocity vector.
    UNUSED(leader_velocity_vector_bearing);
    const double bearing_with_leader = calculateBearing(leader_lat,leader_lon, my_lat, my_lon); // bearing between me and leader.

    POINT_2D p = get_point_at_bearing(leader_lat, leader_lon, bearing_with_leader, base_distance);  // getpoint using bearing and distance.

    // instruct follower to go to a target point.
    mavlinksdk::CMavlinkCommand::getInstance().gotoGuidedPoint(p.latitude , p.longitude , (m_leader_gpos_new.relative_alt + (follower_index +1) * 10000) / 1000.0);

    // broadcast target location or this follower.
    CFCBFacade::getInstance().sendFCBTargetLocation("", p.latitude , p.longitude, (double) m_leader_gpos_new.relative_alt, DESTINATION_SWARM_MY_LOCATION);
    
    // store latest readings.
    m_leader_last_access = now;
    m_leader_gpos_old = m_leader_gpos_new;
}


void CSwarmFollower::updateFollowerInArrowFormation()
{
    const uint32_t KNODE_LENGTH = 100;

    // Get my own location
    mavlinksdk::CVehicle &vehicle = mavlinksdk::CVehicle::getInstance();
    const mavlink_global_position_int_t &my_gpos = vehicle.getMsgGlobalPositionInt();
    const double leader_lat = m_leader_gpos_new.lat / 10000000.0f;
    const double leader_lon = m_leader_gpos_new.lon / 10000000.0f;

    // Get current time
    const u_int64_t now = get_time_usec();


    // Test if leader speed is very low, then break
    double speed_sq = m_leader_gpos_new.vx * m_leader_gpos_new.vx + m_leader_gpos_new.vy * m_leader_gpos_new.vy;
    if (speed_sq < 1000)
    {
        return;
    }

    const double my_lat = my_gpos.lat / 10000000.0f;
    const double my_lon = my_gpos.lon / 10000000.0f;

    const int follower_index = fcb_swarm_manager.getFollowerIndex();
    const bool is_left_side = (follower_index % 2 == 0); // Determine if the follower is on the left side

    // Adjust the base distance for symmetry
    const double base_distance = ((follower_index / 2) + 1) * KNODE_LENGTH; // Base distance from leader

    // Distance between me & leader
    const double distance_to_leader = calcGPSDistance(leader_lat, leader_lon, my_lat, my_lon);

    UNUSED(distance_to_leader);
    
    // Calculate the bearing of the leader's velocity vector
    const double leader_velocity_vector_bearing = getBearingOfVector(m_leader_gpos_new.vx, m_leader_gpos_new.vy);

    // Calculate the bearing between me and the leader
    const double bearing_with_leader = calculateBearing(leader_lat, leader_lon, my_lat, my_lon);
    UNUSED(bearing_with_leader);

    // Calculate the 45-degree offset for V formation
    const double angle_offset = M_PI + (is_left_side ? -M_PI / 4 : M_PI / 4); // 45 degrees in radians

    // Calculate target point for V formation
    POINT_2D p = get_point_at_bearing(leader_lat, leader_lon, leader_velocity_vector_bearing + angle_offset, base_distance);

    // Instruct follower to go to the target point
    mavlinksdk::CMavlinkCommand::getInstance().gotoGuidedPoint(p.latitude, p.longitude, (m_leader_gpos_new.relative_alt + (follower_index + 1) * 10000) / 1000.0);

    // Broadcast target location for this follower
    CFCBFacade::getInstance().sendFCBTargetLocation("", p.latitude, p.longitude, (double)m_leader_gpos_new.relative_alt, DESTINATION_SWARM_MY_LOCATION);

    // Store latest readings
    m_leader_last_access = now;
    m_leader_gpos_old = m_leader_gpos_new;

    #ifdef DEBUG
        std::cout << _INFO_CONSOLE_TEXT << "time_diff: " << ":" << (m_leader_last_access - now) << ":" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        std::cout << "Leader Velocity Bearing: " << leader_velocity_vector_bearing << std::endl;
        std::cout << "Angle Offset: " << angle_offset <<  "     bearing_with_leader: " << bearing_with_leader << std::endl;
        std::cout << "Target Point: " << p.latitude << ", " << p.longitude << std::endl;
    #endif
}

/**
 * @brief Update me as a follower based on the active formation.
 * 
 */
void CSwarmFollower::updateFollower()
{
    de::fcb::swarm::CSwarmManager& fcb_swarm_manager = de::fcb::swarm::CSwarmManager::getInstance();

    switch (fcb_swarm_manager.getFormationAsFollower())
    {
        case FORMATION_THREAD:
            updateFollowerInThreadFormation();
            break;
        case FORMATION_ARROW:
            updateFollowerInArrowFormation();
            break;
        case FORMATION_VECTOR:
            break;
        
        default:
            break;
    }
}


/**
 * @brief Receives location, speed,...etc. from leader SWARM INFO
 * it then uses it based on formation and order in formation to determine 
 * followr exact location, speed...etc.
 * * Received info could be follower exact location, or could be leader location
 * * then each foller calculate its location. It is all based on formation pattern.
 * @param leader_sender 
 * @param full_message 
 * @param full_message_length 
 */
void CSwarmFollower::handle_leader_traffic(const std::string & leader_sender, const char * full_message, const int & full_message_length)
{

    // suggest: send unfollow to it. but take care of message rate.
    // Not a follower
    
    if (!fcb_swarm_manager.isFollower()) return ;
    // This traffic is not from my leader.
    if (!fcb_swarm_manager.isMyLeader(leader_sender)) return ;


    // this is a binary message
    // search for char '0' and then binary message is the next byte after it.
    const char * binary_message = (char *)(memchr (full_message, 0x0, full_message_length));
    int binary_length = binary_message==0?0:(full_message_length - (binary_message - full_message +1) );

    bool valid = false;
    mavlink_status_t status;
	mavlink_message_t mavlink_message;
    for (int i=0; i<binary_length; ++ i)
    {
	    uint8_t msgReceived = mavlink_parse_char(MAVLINK_CHANNEL_INTERMODULE, binary_message[i+ 1], &mavlink_message, &status);
        if (msgReceived!=0)
        {
            valid = true;
            #ifdef DDEBUG        
            std::cout << _INFO_CONSOLE_TEXT << "RX SWARM MAVLINK: " << std::to_string(mavlink_message.msgid) << _NORMAL_CONSOLE_TEXT_ << std::endl;
            #endif
            switch (mavlink_message.msgid)
            {
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                {
                    // decode message
                    mavlink_msg_global_position_int_decode(&mavlink_message, &(m_leader_gpos_new));
                    
                    #ifdef DEBUG        
                        std::cout << _INFO_CONSOLE_TEXT << "RX SWARM MAVLINK: " << std::to_string(mavlink_message.msgid) << ":" << m_leader_gpos_new.lat << ":" << m_leader_gpos_new.lon << ":" << m_leader_gpos_new.relative_alt << ":" << m_leader_gpos_new.vx << ":" << m_leader_gpos_new.vy << ":" << m_leader_gpos_new.vz << ":" <<_NORMAL_CONSOLE_TEXT_ << std::endl;
                        //std::cout << _INFO_CONSOLE_TEXT << "RX SWARM MAVLINK: " << leader_velocity_vector_bearing << "   :   " << _SUCCESS_CONSOLE_TEXT_ << bearing_with_leader << ":" <<_NORMAL_CONSOLE_TEXT_ << std::endl;
                        //std::cout << _INFO_CONSOLE_TEXT << "gotoGuidedPoint: " <<  ":" << p.latitude << ":" << p.longitude << ":" <<_NORMAL_CONSOLE_TEXT_ << std::endl;
                    #endif

                }
                break;
                        
            }
            
        }
    }
    
    if (valid) 
    {
        updateFollower();
    }
}