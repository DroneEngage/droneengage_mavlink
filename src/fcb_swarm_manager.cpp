
#include "fcb_swarm_manager.hpp"

using namespace uavos::fcb;


bool CSwarmManager::isLeader() const
{
    return getFormation() != ANDRUAV_SWARM_FORMATION::FORMATION_NO_SWARM;
}


bool CSwarmManager::isSlave() const
{
    return m_slave_index != -1;
}


void CSwarmManager::unFollow()
{
    m_leader_party_id = std::string("");
    m_slave_index = -1;
}
            

ANDRUAV_SWARM_FORMATION CSwarmManager::getFormation() const
{
    return m_formation;
}

void CSwarmManager::makeSwarm(const ANDRUAV_SWARM_FORMATION formation)
{
    m_formation = formation;

    //TODO: handle change formation for followers.
    //TODO: problems include formation transition, & max number of followers.
}


void CSwarmManager::makeSlave(const std::string& party_id, const int slave_index)
{
    m_leader_party_id = party_id;
    m_slave_index = slave_index;
}
            

/**
 * @brief return index in array of slaves -not index of formation-.
 * 
 * @param party_id 
 * @return int -1 if not found. n>=0 if found.
 */
int CSwarmManager::slaveExists (const std::string& party_id) const
{
    const std::size_t size = m_slave_units.size();

    for (std::size_t i=0; i<size; ++i)
    {
        if (m_slave_units[i].party_id.compare(party_id) == 0)
        {
            return i;
        }
    }

    return -1;
}


/**
 * @brief adds a vehicle as a follower -slave-.
 * @details adding a vehicle as a slave requires the leader that is (Me) to send it guides to follow (Me).
 * @param party_id 
 * @param slave_index index of the vehicle in the formation.
 */
void CSwarmManager::addSlave (const std::string& party_id, const int slave_index)
{
    if (slaveExists(party_id) != -1) return ;

    // add a new one
    m_slave_units.push_back({party_id, slave_index});
    
}


std::string CSwarmManager::getLeader() const
{
    return m_leader_party_id;
}