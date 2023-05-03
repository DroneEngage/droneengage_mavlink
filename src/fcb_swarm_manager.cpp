
#include "fcb_swarm_manager.hpp"
#include "fcb_facade.hpp"
#include "fcb_main.hpp"

using namespace uavos::fcb;


bool CSwarmManager::isLeader() const
{
    return getFormation() != ANDRUAV_SWARM_FORMATION::FORMATION_NO_SWARM;
}


bool CSwarmManager::isSlave() const
{
    return m_follower_index != -1;
}


void CSwarmManager::followLeader(const std::string& party_id, const int follower_index, const std::string& party_id_request)
{
    if (party_id.empty()) return ;
    if ((!m_leader_party_id.empty()) && (m_leader_party_id!=party_id))
    {
        std::cout << "UnFollow " << m_leader_party_id << " requested by: " << party_id_request << std::endl;
        unFollowLeader(m_leader_party_id, uavos::fcb::CFCBMain::getInstance().getAndruavVehicleInfo().party_id);
    }

    m_leader_party_id = party_id;
    m_follower_index = follower_index;

    std::cout << "Follow " << party_id << " index:" << m_follower_index << " requested by: " << party_id_request << std::endl;
    if ((!party_id_request.empty()) && (party_id_request!=party_id))
    {
        uavos::fcb::CFCBFacade::getInstance().requestToFollowLeader (m_leader_party_id, m_follower_index);
    }

    uavos::fcb::CFCBFacade::getInstance().sendID(std::string());
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
        // sender is NOT my leader so inform the leader
        std::cout << "Unfollow:" << party_id_leader_to_unfollow << " . Tell it that I am not following it anymore." << std::endl;
        uavos::fcb::CFCBFacade::getInstance().requestUnFollowLeader (m_leader_party_id);
    }
                    

    
    m_leader_party_id = std::string("");
    m_follower_index = -1;

    uavos::fcb::CFCBFacade::getInstance().sendID(std::string());
}
            

ANDRUAV_SWARM_FORMATION CSwarmManager::getFormation() const
{
    return m_formation;
}

/**
 * @brief Leaders Activate/Deactivate Swarm Formation.
 * 
 * @param formation
 */
void CSwarmManager::makeSwarm(const ANDRUAV_SWARM_FORMATION formation)
{
    if (m_formation == formation) return ;


    m_formation = formation;

    if (formation == FORMATION_SERB_NO_SWARM)
    {
        std::cout << _INFO_CONSOLE_TEXT << std::endl << "Demoted from Leader" <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
	
        // RoundRobin on all followers and release them.
        releaseFollowers();
        return ;
    }

    std::cout << _INFO_CONSOLE_TEXT << std::endl << "Promoted to a Leader" <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
	
    //TODO: handle change formation for followers.
    //TODO: problems include formation transition, & max number of followers.
    uavos::fcb::CFCBFacade::getInstance().sendID(std::string());
}


            

/**
 * @brief Check if a follower drone with the specified party ID exists in the swarm.
 * The function searches for a follower drone in the list of slave units based on the specified party ID. 
 * If the drone is found, the function returns its index in the list. Otherwise, it returns -1.
 * @param party_id The party ID of the follower drone to search for.
 * @return The index of the follower drone if it exists in the list, or -1 if it does not exist.
*/
int CSwarmManager::followersExist(const std::string& party_id) const {
    auto it = std::find_if(m_slave_units.begin(), m_slave_units.end(),
                [&](const ANDRUAV_UNIT_SALVE& unit) { return unit.party_id == party_id; });
    if (it != m_slave_units.end()) {
        return static_cast<int>(std::distance(m_slave_units.begin(), it));
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
    if (follower_index < (int)m_slave_units.size())
    {
        follower_idx = m_slave_units.size();
    }
    else
    {
        follower_idx = follower_index;
    }
    
    std::cout << _INFO_CONSOLE_TEXT << std::endl << "SWARM: " << "add Follower " << party_id << " at " << follower_idx <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
	
    // add a new one
    m_slave_units.push_back({party_id, follower_idx});
    uavos::fcb::CFCBFacade::getInstance().requestFromUnitToFollowMe (party_id, follower_idx);    
}

void CSwarmManager::releaseFollowers ()
{
    // Using a range-based for loop
    for (const auto& item : m_slave_units) {
        // Inform follower.
        // Follower will not send back because the requester is the leader.
        std::cout << _INFO_CONSOLE_TEXT << "Release Follower: " <<  item.party_id << _NORMAL_CONSOLE_TEXT_ << std::endl;
	    uavos::fcb::CFCBFacade::getInstance().requestFromUnitToUnFollowMe (item.party_id);
    }

        
    m_slave_units.clear();
    std::cout << _INFO_CONSOLE_TEXT << "SWARM: " <<  "I have no followers now." << _NORMAL_CONSOLE_TEXT_ << std::endl;
	
}

void CSwarmManager::releaseSingleFollower (const std::string& party_id)
{

    if (party_id.empty()) return ;

    auto it = std::find_if(m_slave_units.begin(), m_slave_units.end(),
        [&](const ANDRUAV_UNIT_SALVE& unit) {
            return (unit.party_id == party_id);
        });
        

    if (it != m_slave_units.end()) {
        // erase the element at the iterator
        std::cout << _INFO_CONSOLE_TEXT << "SWARM: " << "Release Follower: " <<  it->party_id << _NORMAL_CONSOLE_TEXT_ << std::endl;
	    uavos::fcb::CFCBFacade::getInstance().requestFromUnitToUnFollowMe (it->party_id);
        m_slave_units.erase(it);
    }
}