
#include "fcb_swarm_follower.hpp"
#include "../fcb_facade.hpp"
#include "../fcb_main.hpp"
#include "../helpers/gps.hpp"
#include "../de_common/helpers/helpers.hpp"


using namespace de::fcb::swarm;


de::fcb::swarm::CSwarmManager& fcb_swarm_manager = de::fcb::swarm::CSwarmManager::getInstance();


#define SPEED_STATIONARY_THRESHOLD_SQ 2500.0  // 50 cm/s squared — below this leader is considered stationary
#define MAX_BEARING_SLEW_RATE_DEG 30.0        // max bearing change per update (degrees)
#define TARGET_SMOOTHING_ALPHA 0.3            // low-pass filter factor for target position
#define HDG_DEADZONE_DEG 5.0                  // ignore compass heading changes below this (degrees)
#define CROSS_TRACK_DEADZONE_M 3.0            // deadzone in meters for dynamic side switching
#define TARGET_JUMP_THRESHOLD_M 5.0           // if raw target jumps more than this, check direction before chasing


/**
 * @brief Logic of thread formation for follower in implemented here.
 * Leader sends it own location, and I know the formation is Thread-Formation and my index into it 
 * and I generate my position accordingly.
 * Note that another formation my receive exact location of the unit. 
 * The logic varies from a formation to another.
 */
void CSwarmFollower::updateFollowerInThreadFormation()
{
    
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
    // Raised threshold to 50 cm/s to avoid GPS noise causing false movement detection
    // when a copter is hovering and yawing in place.
    double speed_sq = m_leader_gpos_new.vx * m_leader_gpos_new.vx + m_leader_gpos_new.vy * m_leader_gpos_new.vy;
    if (speed_sq < SPEED_STATIONARY_THRESHOLD_SQ) 
    {
        return ;
    }

    const double my_lat = my_gpos.lat / 10000000.0f;
    const double my_lon = my_gpos.lon / 10000000.0f;

    const int follower_index = fcb_swarm_manager.getFollowerIndex();
    const double base_distance = (follower_index + 1) * m_min_horizontal_distance; // Base distance from leader

    // distance between me & leader
    const double distance_to_leader = calcGPSDistance(leader_lat,leader_lon, my_lat, my_lon);

    // the rope effect ... ignore when distance is less than robe length. Rope has nodes and each node has a length. of m_min_horizontal_distance
    if (distance_to_leader <  base_distance)  
    {
        // rope effect ... ignore when distance is less than robe length.
        return ;
    }

    
                    
    const double leader_velocity_vector_bearing = getBearingOfVector (m_leader_gpos_new.vx, m_leader_gpos_new.vy); // bearing of leader velocity vector.
    UNUSED(leader_velocity_vector_bearing);
    const double bearing_with_leader = calculateBearing(leader_lat,leader_lon, my_lat, my_lon); // bearing between me and leader.

    POINT_2D p = get_point_at_bearing(leader_lat, leader_lon, bearing_with_leader, base_distance);  // getpoint using bearing and distance.

    // Low-pass filter on target position to prevent step jumps.
    if (m_has_smoothed_target)
    {
        m_smoothed_target_lat = TARGET_SMOOTHING_ALPHA * p.latitude + (1.0 - TARGET_SMOOTHING_ALPHA) * m_smoothed_target_lat;
        m_smoothed_target_lon = TARGET_SMOOTHING_ALPHA * p.longitude + (1.0 - TARGET_SMOOTHING_ALPHA) * m_smoothed_target_lon;
    }
    else
    {
        m_smoothed_target_lat = p.latitude;
        m_smoothed_target_lon = p.longitude;
        m_has_smoothed_target = true;
    }

    // instruct follower to go to a target point.
    mavlinksdk::CMavlinkCommand::getInstance().gotoGuidedPoint(m_smoothed_target_lat , m_smoothed_target_lon , (m_leader_gpos_new.relative_alt + (follower_index +1) * m_min_vertical_distance * 1000) / 1000.0f);

    // broadcast target location or this follower.
    CFCBFacade::getInstance().sendFCBTargetLocation("", m_smoothed_target_lat , m_smoothed_target_lon, (double) m_leader_gpos_new.relative_alt, DESTINATION_SWARM_MY_LOCATION);
    
    // store latest readings.
    m_leader_last_access = now;
    m_leader_gpos_old = m_leader_gpos_new;
}


void CSwarmFollower::updateFollowerInArrowFormation(const bool is_dynamic)
{
    
    // Get my own location
    mavlinksdk::CVehicle &vehicle = mavlinksdk::CVehicle::getInstance();
    const mavlink_global_position_int_t &my_gpos = vehicle.getMsgGlobalPositionInt();
    const double leader_lat = m_leader_gpos_new.lat / 10000000.0f;
    const double leader_lon = m_leader_gpos_new.lon / 10000000.0f;

    // Get current time
    const u_int64_t now = get_time_usec();


    // Determine formation bearing:
    // - When leader is moving, use velocity bearing with slew-rate limiting.
    // - FORMATION_ARROW (static): when stationary, use compass heading (hdg).
    // - FORMATION_ARROW_DYNAMIC: when stationary, freeze last movement bearing.
    double speed_sq = m_leader_gpos_new.vx * m_leader_gpos_new.vx + m_leader_gpos_new.vy * m_leader_gpos_new.vy;
    double formation_bearing;

    if (speed_sq >= SPEED_STATIONARY_THRESHOLD_SQ)
    {
        // Leader is moving: use velocity bearing
        formation_bearing = getBearingOfVector(m_leader_gpos_new.vx, m_leader_gpos_new.vy);

        // Bearing latching: limit bearing change rate to prevent violent swings
        if (m_has_last_bearing)
        {
            double bearing_diff = formation_bearing - m_last_leader_bearing;
            // Normalize to [-PI, PI]
            while (bearing_diff > M_PI) bearing_diff -= 2.0 * M_PI;
            while (bearing_diff < -M_PI) bearing_diff += 2.0 * M_PI;

            const double max_slew = MAX_BEARING_SLEW_RATE_DEG * M_PI / 180.0;
            if (fabs(bearing_diff) > max_slew)
            {
                formation_bearing = m_last_leader_bearing + (bearing_diff > 0 ? max_slew : -max_slew);
            }
        }
        m_last_leader_bearing = formation_bearing;
        m_has_last_bearing = true;
    }
    else
    {
        if (is_dynamic)
        {
            // Dynamic V: formation depends ONLY on actual movement direction.
            // Freeze last movement bearing; skip if leader has never moved.
            if (!m_has_last_bearing)
            {
                return;
            }
            formation_bearing = m_last_leader_bearing;
        }
        else
        {
            // Static V: use compass heading (hdg) so the V points where
            // the leader faces, even when hovering or yawing in place.
            // hdg is in centi-degrees (0.01 deg). 65535 means unknown.
            if (m_leader_gpos_new.hdg != 65535 && m_leader_gpos_new.hdg < 36000)
            {
                double hdg_rad = m_leader_gpos_new.hdg / 100.0 * M_PI / 180.0;

                // Deadzone: only update bearing if heading changed more than HDG_DEADZONE_DEG
                if (m_has_last_bearing)
                {
                    double hdg_diff = hdg_rad - m_last_leader_bearing;
                    while (hdg_diff > M_PI) hdg_diff -= 2.0 * M_PI;
                    while (hdg_diff < -M_PI) hdg_diff += 2.0 * M_PI;

                    if (fabs(hdg_diff) > HDG_DEADZONE_DEG * M_PI / 180.0)
                    {
                        m_last_leader_bearing = hdg_rad;
                    }
                }
                else
                {
                    m_last_leader_bearing = hdg_rad;
                    m_has_last_bearing = true;
                }
                formation_bearing = m_last_leader_bearing;
            }
            else
            {
                // No hdg available: use last known bearing, or skip if none
                if (!m_has_last_bearing)
                {
                    return;
                }
                formation_bearing = m_last_leader_bearing;
            }
        }
    }

    const double my_lat = my_gpos.lat / 10000000.0f;
    const double my_lon = my_gpos.lon / 10000000.0f;

    const int follower_index = fcb_swarm_manager.getFollowerIndex();

    // Cross-track position relative to the V's backward heading (formation opens BACKWARD).
    // Kept for diagnostics/possible future use, but side is now fixed by follower_index
    // to guarantee a valid V shape. Decentralized physical-side assignment can put
    // two followers on the same wing when they happen to be on the same side of the leader.
    const double backward_north = cos(formation_bearing + M_PI);
    const double backward_east  = sin(formation_bearing + M_PI);
    const double dx_m = (my_lat - leader_lat) * 111000.0;
    const double dy_m = (my_lon - leader_lon) * 111000.0 * cos(leader_lat * M_PI / 180.0);
    const double cross_m = backward_north * dy_m - backward_east * dx_m;
    UNUSED(cross_m);

    // Side assignment: the V formation requires one follower on each wing at the
    // same tier. Each follower runs independently, so the only way to guarantee
    // separation without leader coordination is to use follower_index parity.
    // Even index (0, 2, ...) -> left wing; odd index (1, 3, ...) -> right wing.
    const bool is_left_side = (follower_index % 2 == 0);

    // Adjust the base distance for symmetry
    const double base_distance = ((follower_index / 2) + 1) * m_min_horizontal_distance; // Base distance from leader

    // Distance between me & leader
    const double distance_to_leader = calcGPSDistance(leader_lat, leader_lon, my_lat, my_lon);

    UNUSED(distance_to_leader);

    // Calculate the bearing between me and the leader
    const double bearing_with_leader = calculateBearing(leader_lat, leader_lon, my_lat, my_lon);
    UNUSED(bearing_with_leader);

    // Calculate the 45-degree offset for V formation
    const double angle_offset = M_PI + (is_left_side ? -M_PI / 4 : M_PI / 4); // 45 degrees in radians

    // Calculate target point for V formation using the stabilized formation bearing
    POINT_2D p = get_point_at_bearing(leader_lat, leader_lon, formation_bearing + angle_offset, base_distance);

    // Target jump detection: if the raw target suddenly jumped, check if it's coming
    // toward the follower. If so, wait — but only for a couple of cycles to avoid
    // getting permanently stuck.
    if (m_has_prev_raw_target)
    {
        const double jump_dx = (p.latitude - m_prev_raw_target_lat) * 111000.0;
        const double jump_dy = (p.longitude - m_prev_raw_target_lon) * 111000.0 * cos(leader_lat * M_PI / 180.0);
        const double jump_dist = sqrt(jump_dx * jump_dx + jump_dy * jump_dy);

        if (jump_dist > TARGET_JUMP_THRESHOLD_M && m_consecutive_skips < 2)
        {
            // Target jumped significantly. Check direction:
            // old_target -> follower vector
            const float to_follower_dx = (my_lat - m_prev_raw_target_lat) * 111000.0;
            const float to_follower_dy = (my_lon - m_prev_raw_target_lon) * 111000.0 * cos(leader_lat * M_PI / 180.0);
            // Dot product: if negative, target jumped toward the follower → wait
            const float dot = jump_dx * to_follower_dx + jump_dy * to_follower_dy;
            if (dot < 0.0)
            {
                // Target is coming toward follower — hold position for now
                m_prev_raw_target_lat = p.latitude;
                m_prev_raw_target_lon = p.longitude;
                m_consecutive_skips++;
                m_leader_last_access = now;
                m_leader_gpos_old = m_leader_gpos_new;
                return;
            }
        }
    }
    m_prev_raw_target_lat = p.latitude;
    m_prev_raw_target_lon = p.longitude;
    m_has_prev_raw_target = true;
    m_consecutive_skips = 0;

    // Low-pass filter on target position to prevent step jumps
    if (m_has_smoothed_target)
    {
        m_smoothed_target_lat = TARGET_SMOOTHING_ALPHA * p.latitude + (1.0 - TARGET_SMOOTHING_ALPHA) * m_smoothed_target_lat;
        m_smoothed_target_lon = TARGET_SMOOTHING_ALPHA * p.longitude + (1.0 - TARGET_SMOOTHING_ALPHA) * m_smoothed_target_lon;
    }
    else
    {
        m_smoothed_target_lat = p.latitude;
        m_smoothed_target_lon = p.longitude;
        m_has_smoothed_target = true;
    }

    // Instruct follower to go to the target point
    mavlinksdk::CMavlinkCommand::getInstance().gotoGuidedPoint(m_smoothed_target_lat, m_smoothed_target_lon, (m_leader_gpos_new.relative_alt + (follower_index + 1) * m_min_vertical_distance * 1000) / 1000.0f);

    // Broadcast target location for this follower
    CFCBFacade::getInstance().sendFCBTargetLocation("", m_smoothed_target_lat, m_smoothed_target_lon, (double)m_leader_gpos_new.relative_alt, DESTINATION_SWARM_MY_LOCATION);

    // Store latest readings
    m_leader_last_access = now;
    m_leader_gpos_old = m_leader_gpos_new;

    #ifdef DEBUG
        std::cout << _INFO_CONSOLE_TEXT << "time_diff: " << ":" << (m_leader_last_access - now) << ":" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        std::cout << "Formation Bearing: " << formation_bearing << " (speed_sq: " << speed_sq << ")" << std::endl;
        std::cout << "Angle Offset: " << angle_offset <<  "     bearing_with_leader: " << bearing_with_leader << std::endl;
        std::cout << "Target Point: " << m_smoothed_target_lat << ", " << m_smoothed_target_lon << std::endl;
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
            updateFollowerInArrowFormation(false);
            break;
        case FORMATION_ARROW_DYNAMIC:
            updateFollowerInArrowFormation(true);
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
    const char * binary_message = (const char *)(memchr(full_message, 0x0, full_message_length));
    int binary_length = 0;
    if (binary_message != nullptr && (binary_message - full_message) < full_message_length - 1) {
        // binary payload starts after the 0x0 byte
        binary_length = full_message_length - (binary_message - full_message + 1);
    }

    bool valid = false;
    mavlink_status_t status;
	mavlink_message_t mavlink_message;
    for (int i = 0; i < binary_length; ++i)
    {
	    uint8_t msgReceived = mavlink_parse_char(MAVLINK_CHANNEL_INTERMODULE, binary_message[i + 1], &mavlink_message, &status);
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
                    #endif
                }
                break;

                case MAVLINK_MSG_ID_ATTITUDE:
                {
                    mavlink_attitude_t attitude;
                    mavlink_msg_attitude_decode(&mavlink_message, &attitude);
                    m_leader_yaw = attitude.yaw;
                    m_leader_yawspeed = attitude.yawspeed;
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