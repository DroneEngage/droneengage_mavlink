
#include "fcb_swarm_manager.hpp"
#include "../fcb_facade.hpp"
#include "../fcb_main.hpp"
#include "../helpers/gps.hpp"
#include "../helpers/helpers.hpp"


using namespace uavos::fcb::swarm;


/**
 * @brief: 
 * UnFollow Leader: The follower informs the leader that it will unfolllow it and that is all.
 * 
 * Follow Leader: Normally this request is called from GCS. The drone is told to follow a leader.
 *                The follower can accept or reject. If it accepts it sends a request to the leader
 *                to ask it to join. The leader can accept or reject and send back location 
 *                and formation pattern to follower if accepted.
 * 
*/
void CSwarmManager::followLeader(const std::string& leader_party_id, const int follower_index, const ANDRUAV_SWARM_FORMATION follower_formation, const std::string& party_id_request)
{
    
    if (leader_party_id.empty()) 
    {
        // you ask me to follow nothing. This is bad.
        // I should get unFollowLeader message to unfollow leader.
        // this is a bad message so ignore.
        // maybe because of a racing condition.
        return ;
    }


    if ((!m_leader_party_id.empty()) && (m_leader_party_id!=leader_party_id))
    { 
        // I am following a leader but some one is asking be to switch leaders.
        // so send to my current leader informing that I am unfollowing it first

        std::cout << "UnFollow " << m_leader_party_id << " requested by: " << party_id_request << std::endl;
        unFollowLeader(m_leader_party_id, uavos::fcb::CFCBMain::getInstance().getAndruavVehicleInfo().party_id);
    }

    // now copy new leader info.
    m_leader_party_id = leader_party_id;
    // this can be -1 unless it is send from my leader
    m_follower_index = follower_index;
    // also this can be 0 unless it is send from my leader
    m_formation_as_follower = follower_formation;

    std::cout << "Follow " << leader_party_id << " index:" << m_follower_index << " m_formation_as_follower:" << m_formation_as_follower << " requested by: " << party_id_request << std::endl;
    if ((!party_id_request.empty()) && (party_id_request!=leader_party_id))
    { // if the requestor is not the leader then I need to ask the leader.
        uavos::fcb::CFCBFacade::getInstance().requestToFollowLeader (m_leader_party_id, m_follower_index);
    }

    //NOTE: MAYBE IN FUTURE WE SPLIT THIS INTO TWO MESSAGES.
    // REQUEST_TO_FOLLOW & DO_FOLLOW
    
    uavos::fcb::CFCBFacade::getInstance().API_IC_sendID(std::string());
}


/**
 * @brief: This function handles the request to stop following a leader drone. 
 * If the request is coming from a drone that is not following the specified leader, 
 * the function returns immediately without taking any action. 
 * If the request is coming from a drone that is following a different leader, 
 * the function requests that leader to be unfollowed. 
 * Otherwise, the function sets the m_leader_party_id variable to an empty string 
 * and sets the m_follower_index variable to -1. 
 * 
 * @param party_id_leader_to_unfollow 
 * @param party_id_request 
 **/
void CSwarmManager::unFollowLeader(const std::string& party_id_leader_to_unfollow, const std::string& party_id_request)
{
    std::cout << "UnFollow " << party_id_leader_to_unfollow << " requested by: " << party_id_request << std::endl;

    if ((!party_id_leader_to_unfollow.empty())
     && (party_id_leader_to_unfollow != m_leader_party_id))
    {
        // I am not following it anyway.
        // this is not an error. It can happen due to event racing.
        // also the unfollow request origin could be the follower so it is not following the leader anymore.
        std::cout << "Unfollow " << party_id_leader_to_unfollow << " but I am not following it" << std::endl;
        return ;
    }
    
    if (party_id_request.compare(m_leader_party_id)!=0)
    {
        // sender is NOT my leader so inform the leader.. 
        // Typical scenario here is GCS is asking me to unfollow a leader.
        
        std::cout << "Unfollow:" << party_id_leader_to_unfollow << " . Tell it that I am not following it anymore." << std::endl;
        uavos::fcb::CFCBFacade::getInstance().requestUnFollowLeader (m_leader_party_id);
    }
                    
    
    m_leader_party_id = std::string("");
    m_follower_index = -1;
    m_formation_as_follower = ANDRUAV_SWARM_FORMATION::FORMATION_NO_SWARM;

    std::cout << "UnFollow " << party_id_leader_to_unfollow << " requested by: " << party_id_request << " DONE." << std::endl;

    uavos::fcb::CFCBFacade::getInstance().API_IC_sendID(std::string());
}
            
ANDRUAV_SWARM_FORMATION CSwarmManager::getFormationAsFollower() const
{
    return m_formation_as_follower;    
}

ANDRUAV_SWARM_FORMATION CSwarmManager::getFormationAsLeader() const
{
    return m_formation_as_leader;
}

/**
 * @brief Leaders Activate/Deactivate Swarm Formation.
 * 
 * @param formation
 */
void CSwarmManager::makeSwarm(const ANDRUAV_SWARM_FORMATION formation)
{
    if (m_formation_as_leader == formation) return ;


    m_formation_as_leader = formation;

    if (formation == FORMATION_SERB_NO_SWARM)
    {
        std::cout << _INFO_CONSOLE_TEXT << std::endl << "Demoted from Leader" <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
        m_is_leader = false;
        // RoundRobin on all followers and release them.
        releaseFollowers();
        return ;
    }
    m_is_leader = true;

    std::cout << _INFO_CONSOLE_TEXT << std::endl << "Promoted to a Leader" <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
	
    //TODO: handle change formation for followers.
    //TODO: problems include formation transition, & max number of followers.
    uavos::fcb::CFCBFacade::getInstance().API_IC_sendID(std::string());
}


            

/**
 * @brief Check if a follower drone with the specified party ID exists in the swarm.
 * The function searches for a follower drone in the list of slave units based on the specified party ID. 
 * If the drone is found, the function returns its index in the list. Otherwise, it returns -1.
 * @param party_id The party ID of the follower drone to search for.
 * @return The index of the follower drone if it exists in the list, or -1 if it does not exist.
*/
int CSwarmManager::followersExist(const std::string& party_id) const {
    auto it = std::find_if(m_follower_units.begin(), m_follower_units.end(),
                [&](const ANDRUAV_UNIT_FOLLOWER& unit) { return unit.party_id == party_id; });
    if (it != m_follower_units.end()) {
        return static_cast<int>(std::distance(m_follower_units.begin(), it));
    } else {
        return -1;
    }
}


/**
 * @brief adds a vehicle as a follower -slave-.
 * @details adding a vehicle as a slave requires the leader that is (Me) to send it guides to follow (Me).
 * @param party_id 
 * @param follower_index index of the vehicle in the formation.
 */
void CSwarmManager::addFollower (const std::string& party_id, const int follower_index)
{
    if (followersExist(party_id) != -1) 
    {
        #ifdef DEBUG        
            std::cout << _INFO_CONSOLE_TEXT << "addFollower: " << party_id << " is already added." _NORMAL_CONSOLE_TEXT_ << std::endl;
        #endif
        return ;
    }

    int follower_idx;
    if (follower_index < (int)m_follower_units.size())
    {  //BUG: REVAMP this to avoid re-taking same location index.
        follower_idx = m_follower_units.size();
    }
    else
    {
        follower_idx = follower_index;
    }
    
    std::cout << _INFO_CONSOLE_TEXT << std::endl << "SWARM: " << "add Follower " << party_id << " at " << follower_idx <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
	
    // add a new one
    m_follower_units.push_back({party_id, follower_idx});
    uavos::fcb::CFCBFacade::getInstance().requestFromUnitToFollowMe (party_id, follower_idx);
    uavos::fcb::CFCBFacade::getInstance().API_IC_P2P_connectToMeshOnMac(party_id);
}

void CSwarmManager::releaseFollowers ()
{
    // Using a range-based for loop
    for (const auto& item : m_follower_units) {
        // Inform follower.
        // Follower will not send back because the requester is the leader.
        std::cout << _INFO_CONSOLE_TEXT << "Release Follower: " <<  item.party_id << _NORMAL_CONSOLE_TEXT_ << std::endl;
	    uavos::fcb::CFCBFacade::getInstance().requestFromUnitToUnFollowMe (item.party_id);
    }

        
    m_follower_units.clear();
    std::cout << _INFO_CONSOLE_TEXT << "SWARM: " <<  "I have no followers now." << _NORMAL_CONSOLE_TEXT_ << std::endl;
	
}

void CSwarmManager::releaseSingleFollower (const std::string& party_id)
{

    if (party_id.empty()) return ;

    auto it = std::find_if(m_follower_units.begin(), m_follower_units.end(),
        [&](const ANDRUAV_UNIT_FOLLOWER& unit) {
            return (unit.party_id == party_id);
        });
        

    if (it != m_follower_units.end()) {
        // erase the element at the iterator
        std::cout << _INFO_CONSOLE_TEXT << "SWARM: " << "Release Follower: " <<  it->party_id << _NORMAL_CONSOLE_TEXT_ << std::endl;
	    uavos::fcb::CFCBFacade::getInstance().requestFromUnitToUnFollowMe (it->party_id);
        m_follower_units.erase(it);
    }
}


