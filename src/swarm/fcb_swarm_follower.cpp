
#include "fcb_swarm_follower.hpp"
#include "fcb_swarm_manager.hpp"
#include "../fcb_facade.hpp"
#include "../fcb_main.hpp"
#include "../helpers/gps.hpp"
#include "../helpers/helpers.hpp"


using namespace uavos::fcb::swarm;


void CSwarmFollower::handle_leader_traffic(const std::string & leader_sender, const char * full_message, const int & full_message_length)
{

    // suggest: send unfollow to it. but take care of message rate.
    // Not a follower
    if (!m_fcb_swarm_manager.isFollower()) return ;
    // This traffic is not from my leader.
    if (!m_fcb_swarm_manager.isMyLeader(leader_sender)) return ;


    // this is a binary message
    // search for char '0' and then binary message is the next byte after it.
    const char * binary_message = (char *)(memchr (full_message, 0x0, full_message_length));
    int binary_length = binary_message==0?0:(full_message_length - (binary_message - full_message +1) );


    mavlink_status_t status;
	mavlink_message_t mavlink_message;
    for (int i=0; i<binary_length; ++ i)
    {
	    uint8_t msgReceived = mavlink_parse_char(MAVLINK_COMM_0, binary_message[i+ 1], &mavlink_message, &status);
        if (msgReceived!=0)
        {
            #ifdef DEBUG        
            std::cout << _INFO_CONSOLE_TEXT << "RX SWARM MAVLINK: " << std::to_string(mavlink_message.msgid) << _NORMAL_CONSOLE_TEXT_ << std::endl;
            #endif
            switch (mavlink_message.msgid)
            {
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                {
                    // decode message
                    mavlink_global_position_int_t leader_gpos;
                    mavlink_msg_global_position_int_decode(&mavlink_message, &(leader_gpos));
                    
                    
                    // get my own location
                    mavlinksdk::CVehicle &vehicle =  mavlinksdk::CVehicle::getInstance();
                    const mavlink_global_position_int_t&  my_gpos = vehicle.getMsgGlobalPositionInt();
                    const double leader_lat = leader_gpos.lat / 10000000.0f;
                    const double leader_lon = leader_gpos.lon / 10000000.0f;

                    // get current time
                    const u_int64_t now = get_time_usec();
                    
                    std::cout << _INFO_CONSOLE_TEXT << "time_diff: " <<  ":" << (m_leader_last_access - now) << ":" <<_NORMAL_CONSOLE_TEXT_ << std::endl;
                    
                    // test if leader speed is very low then break.
                    double speed_sq = leader_gpos.vx * leader_gpos.vx + leader_gpos.vy * leader_gpos.vy;
                    if (speed_sq < 100) 
                    {
                        return ;
                    }
                    //const double leader_motion_bearing = calculateBearing (leader_lat,leader_lon, m_leader_gpos_latest.lat / 10000000.0f, m_leader_gpos_latest.lon / 10000000.0f);
                    
                    // store latest readings.
                    m_leader_last_access = now;
                    m_leader_gpos_latest = leader_gpos;

                    const double my_lat = my_gpos.lat / 10000000.0f;
                    const double my_lon = my_gpos.lon / 10000000.0f;


                    // distance between me & leader
                    const double distance_to_leader = calcGPSDistance(leader_lat,leader_lon, my_lat, my_lon);

                    if (distance_to_leader <  (m_fcb_swarm_manager.getFollowerIndex()+1)*100)  
                    {
                        // rope effect ... ignore when distance is less than robe length.
                        return ;
                    }

                    const double leader_vector_bearing = getBearingOfVector (leader_gpos.vx, leader_gpos.vy);
                    const double bearing_with_leader = calculateBearing(leader_lat,leader_lon, my_lat, my_lon);

                    POINT_2D p = get_point_at_bearing(leader_lat, leader_lon, bearing_with_leader, (m_fcb_swarm_manager.getFollowerIndex()+1)*100);

                    mavlinksdk::CMavlinkCommand::getInstance().gotoGuidedPoint(p.latitude , p.longitude , (leader_gpos.relative_alt + (m_fcb_swarm_manager.getFollowerIndex()+1) * 10000) / 1000.0);
                    CFCBFacade::getInstance().sendFCBTargetLocation("", p.latitude , p.longitude, (double) leader_gpos.relative_alt, DESTINATION_SWARM_MY_LOCATION);
                    #ifdef DEBUG        
                    std::cout << _INFO_CONSOLE_TEXT << "RX SWARM MAVLINK: " << std::to_string(mavlink_message.msgid) << ":" << leader_gpos.lat << ":" << leader_gpos.lon << ":" << leader_gpos.relative_alt << ":" << leader_gpos.vx << ":" << leader_gpos.vy << ":" << leader_gpos.vz << ":" <<_NORMAL_CONSOLE_TEXT_ << std::endl;
                    std::cout << _INFO_CONSOLE_TEXT << "RX SWARM MAVLINK: " << leader_vector_bearing << "   :   " << _SUCCESS_CONSOLE_TEXT_ << bearing_with_leader << ":" <<_NORMAL_CONSOLE_TEXT_ << std::endl;
                    //std::cout << _INFO_CONSOLE_TEXT << "gotoGuidedPoint: " <<  ":" << p.latitude << ":" << p.longitude << ":" <<_NORMAL_CONSOLE_TEXT_ << std::endl;
                    #endif
                }
                break;
                        
            }
            
        }
    }
}