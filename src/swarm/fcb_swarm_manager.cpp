

#include "../fcb_facade.hpp"
#include "../fcb_main.hpp"
#include "../helpers/gps.hpp"
#include "../helpers/helpers.hpp"
#include "../fcb_main.hpp"

#include "fcb_swarm_manager.hpp"
#include "fcb_swarm_follower.hpp"

using namespace de::fcb::swarm;

/**
 * @brief: 
 * Follow Leader: Normally this request is called first by GCS. The drone is told to follow a leader.
 *                The follower can accept or reject. If it accepts it sends a request to the leader
 *                to ask it to join. The leader can accept or reject and send back location 
 *                and formation pattern to follower if accepted.
 * 
 *                The leader drone will call this function in the follower drone directly or as 
 *                a reply of requestToFollowLeader()
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
        unFollowLeader(m_leader_party_id, de::fcb::CFCBMain::getInstance().getAndruavVehicleInfo().party_id);
    }

    // now copy new leader info.
    m_leader_party_id = leader_party_id;
    // this can be -1 unless it is send from my leader
    m_follower_index = follower_index;
    // also this can be 0 unless it is send from my leader
    m_formation_as_follower = follower_formation;

    std::string event = "Follow " + leader_party_id + " index:" + std::to_string(m_follower_index) + " fromation#:" + std::to_string(m_formation_as_follower) + " requested by: " + party_id_request;
    std::cout << std::endl << _INFO_CONSOLE_TEXT << "Follow " << _SUCCESS_CONSOLE_BOLD_TEXT_ << leader_party_id 
        << _INFO_CONSOLE_TEXT << " index:" << _SUCCESS_CONSOLE_BOLD_TEXT_ << m_follower_index 
        << _INFO_CONSOLE_TEXT << " m_formation_as_follower:" << _SUCCESS_CONSOLE_BOLD_TEXT_ << m_formation_as_follower 
        << _INFO_CONSOLE_TEXT<< " requested by: " << _SUCCESS_CONSOLE_BOLD_TEXT_ << party_id_request << std::endl;
    
    de::fcb::CFCBFacade::getInstance().sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_WARNING, event);

    if ((!party_id_request.empty()) && (party_id_request!=leader_party_id))
    { // if the requestor is not the leader then I need to ask the leader. and then the leader will call back followLeader() again.
        de::fcb::CFCBFacade::getInstance().requestToFollowLeader (m_leader_party_id);
    }

    //NOTE: MAYBE IN FUTURE WE SPLIT THIS INTO TWO MESSAGES.
    // REQUEST_TO_FOLLOW & DO_FOLLOW
    
    de::fcb::CFCBFacade::getInstance().API_IC_sendID(std::string());
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
        std::string event = "Unfollow " + party_id_leader_to_unfollow + " but I am not following it";
        
        std::cout << std::endl << _INFO_CONSOLE_TEXT << "Unfollow " << _SUCCESS_CONSOLE_BOLD_TEXT_ << party_id_leader_to_unfollow
            << _INFO_CONSOLE_TEXT << " but I am not following it" << std::endl;
        
        //de::fcb::CFCBFacade::getInstance().sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_ERROR, event);

        return ;
    }
    
    if (party_id_request.compare(m_leader_party_id)!=0)
    {
        // sender is NOT my leader so inform the leader.. 
        // Typical scenario here is GCS is asking me to unfollow a leader.
        
        std::string event = "Unfollow " + party_id_leader_to_unfollow + " . Tell it that I am not following it anymore.";
        
        std::cout << std::endl << _INFO_CONSOLE_TEXT << "Unfollow:" << _SUCCESS_CONSOLE_BOLD_TEXT_ << party_id_leader_to_unfollow 
            << _INFO_CONSOLE_TEXT << " . Tell it that I am not following it anymore." << std::endl;
        
        de::fcb::CFCBFacade::getInstance().sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_WARNING, event);
        
        de::fcb::CFCBFacade::getInstance().requestUnFollowLeader (m_leader_party_id);
    }
                    
    
    m_leader_party_id = std::string("");
    m_follower_index = -1;
    m_formation_as_follower = ANDRUAV_SWARM_FORMATION::FORMATION_NO_SWARM;

    std::string event = "Unfollow " + party_id_leader_to_unfollow + " requested by: " + party_id_request + " DONE.";
    std::cout << std::endl << _INFO_CONSOLE_TEXT << "UnFollow " << _SUCCESS_CONSOLE_BOLD_TEXT_ << party_id_leader_to_unfollow 
        << _INFO_CONSOLE_TEXT << " requested by: " << _SUCCESS_CONSOLE_BOLD_TEXT_ << party_id_request 
        << _INFO_CONSOLE_TEXT << " DONE." << std::endl;

    de::fcb::CFCBFacade::getInstance().sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_WARNING, event);
        
    de::fcb::CFCBFacade::getInstance().API_IC_sendID(std::string());
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
 * This function can do three things:
 *  1- Activate Swarm Formation.
 *  2- Deactivate Swarm Formation.
 *  3- Change Swarm Formation.
 *  
 * @param formation 
 */
void CSwarmManager::handleMakeSwarm(const Json_de& andruav_message, const char * full_message, const int & full_message_length)
{
    // Become a leader drone.
    // a: formationID
    // b: partyID
    // if (b:partyID) is not me then treat it as an announcement.

    const Json_de cmd = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_CMD];
    de::fcb::CFCBMain&  fcbMain = de::fcb::CFCBMain::getInstance();
    

    if (!cmd.contains("a") || !cmd["a"].is_number_integer()) return ;
    if (!cmd.contains("b") || !cmd["b"].is_string()) return ;
                
    if (fcbMain.getAndruavVehicleInfo().party_id.compare( cmd["b"].get<std::string>())!=0)
    {
        // this is an announcement.
        // other drone should handle this message.
        // if this is my leader drone and it is being released then 
        // I can release my self or wait for it to ell me to do so -which makes more sense-.
        return ;
    }

    const int formation = cmd["a"].get<int>();

    if (m_formation_as_leader == formation) return ;


    m_formation_as_leader = (ANDRUAV_SWARM_FORMATION)formation;

    if (formation == FORMATION_SERB_NO_SWARM)
    {
        std::cout << _INFO_CONSOLE_TEXT << std::endl << "Demoted from Leader" <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
        m_is_leader = false;
        // RoundRobin on all followers and release them.
        releaseFollowers();

    }
    else
    {   

        if (m_is_leader)
        {
            // already leader... then this is a change formation request.
            // I need to inform all followers about the change.
            
            
            for (const auto& follower : m_follower_units)
            {
                de::fcb::CFCBFacade::getInstance().requestFromUnitToChangeFormation(follower.party_id, follower.follower_index);
            }  
        }
        else
        {
            m_is_leader = true;
        }

        std::cout << _INFO_CONSOLE_TEXT << std::endl << "Promoted to a Leader with formation:" << m_formation_as_leader <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
        
        //TODO: handle change formation for followers.
        //TODO: problems include formation transition, & max number of followers.

    }

    de::fcb::CFCBFacade::getInstance().API_IC_sendID(std::string());
}


            

/**
 * @brief Check if a follower drone with the specified party ID exists in the swarm.
 * The function searches for a follower drone in the list of slave units based on the specified party ID. 
 * If the drone is found, the function returns its index in the list. Otherwise, it returns -1.
 * @param party_id The party ID of the follower drone to search for.
 * @return The index of the follower drone if it exists in the list, or -1 if it does not exist.
*/
int CSwarmManager::followerExist(const std::string& party_id) const {
    auto it = std::find_if(m_follower_units.begin(), m_follower_units.end(),
                [&](const ANDRUAV_UNIT_FOLLOWER& unit) { return unit.party_id == party_id; });
    if (it != m_follower_units.end()) {
        return static_cast<int>(std::distance(m_follower_units.begin(), it));
    } else {
        return -1;
    }
}


/**
 * @brief 
 * 
 * @param party_id 
 * @return int if you return (-1) then you reject adding.
 */
int CSwarmManager::insertFollowerInSwarmFormation(const std::string& party_id) {

    int follower_idx = 0;

    // Find the smallest missing index by iterating from 1 onwards
    for(const ANDRUAV_UNIT_FOLLOWER& i : m_follower_units) 
    {
        if (i.follower_index <= follower_idx)
        {
            ++follower_idx;
        }
    }
    
    std::cout << _INFO_CONSOLE_TEXT << std::endl << "SWARM: " << "add Follower " << party_id << " at " << follower_idx <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
	
    m_follower_units.push_back({party_id, follower_idx});

    return follower_idx;
}


/**
 * @brief adds a vehicle as a follower -slave-.
 * @details adding a vehicle as a slave requires the leader that is (Me) to send it guides to follow (Me).
 * @param party_id 
 */
void CSwarmManager::addFollower (const std::string& party_id)
{
    int follower_idx = followerExist(party_id);
    if ( follower_idx == -1) 
    {
        #ifdef DEBUG        
            std::cout << _INFO_CONSOLE_TEXT << "addFollower: " << party_id << " is already added." _NORMAL_CONSOLE_TEXT_ << std::endl;
        #endif
        
   

    
        follower_idx = insertFollowerInSwarmFormation (party_id);
        if (follower_idx==-1)
        {
            // rejected;
            //TODO: send a rejection message.
            return ;
        }
    }
    de::fcb::CFCBFacade::getInstance().requestFromUnitToFollowMe (party_id, follower_idx);
    de::fcb::CFCBFacade::getInstance().API_IC_P2P_accessMac(party_id);
}

void CSwarmManager::releaseFollowers ()
{
    // Using a range-based for loop
    for (const auto& item : m_follower_units) {
        // Inform follower.
        // Follower will not send back because the requester is the leader.
        std::cout << _INFO_CONSOLE_TEXT << "Release Follower: " <<  item.party_id << _NORMAL_CONSOLE_TEXT_ << std::endl;
	    de::fcb::CFCBFacade::getInstance().requestFromUnitToUnFollowMe (item.party_id);
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
	    de::fcb::CFCBFacade::getInstance().requestFromUnitToUnFollowMe (it->party_id);
        m_follower_units.erase(it);
    }
}


/**
 * @brief: This function is called by a GCS or leader and received by a follower.
 * 
 */
void CSwarmManager::handleFollowHimRequest(const Json_de& andruav_message, const char * full_message, const int & full_message_length)
{
    Json_de cmd = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_CMD];
        
    bool is_inter_module = false;
    if ((validateField(andruav_message, INTERMODULE_ROUTING_TYPE, Json_de::value_t::string)) && (andruav_message[INTERMODULE_ROUTING_TYPE].get<std::string>().compare(CMD_TYPE_INTERMODULE)==0))
    {   // permission is not needed if this command sender is the communication server not a remote GCS or Unit.
        is_inter_module = true;
    }

    // a: follower index -1 means any available location.   
    // b: leader partyID 
    // c: slave partyID
    // d: formation_id
    // f: SWARM_FOLLOW/SWARM_UNFOLLOW
    // if (c:partyID) is not me then treat it as an announcement.
                
    if (!cmd.contains("f") || !cmd["f"].is_number_integer()) return ;
    if (!cmd.contains("c") || !cmd["c"].is_string()) return ;
                
    de::fcb::CFCBMain&  fcbMain = de::fcb::CFCBMain::getInstance();
            
    if (fcbMain.getAndruavVehicleInfo().party_id.compare(cmd["c"].get<std::string>())!=0)
    {   // I am not the follower in this message... This is just an info message.

        std::cout << "FollowHim_Request announcement by: " << cmd["c"].get<std::string>() << std::endl;

        // this is an announcement.
        // other drone should handle this message.
        return ;
    }

                
    const int swarm_action = cmd["f"].get<int>();

    std::string sender = "";
    if (!is_inter_module)
    {
        sender = andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>();
    }
    else
    {
        sender = de::comm::CModule::getInstance().getPartyId();
    }

    switch (swarm_action)
    {
        case SWARM_UNFOLLOW:
        {
            // [b] can be null
            std::string leader = getLeader();
            if (cmd.contains("b") && cmd["b"].is_string()) 
            {
                leader = cmd["b"].get<std::string>();
            }
            unFollowLeader(leader, andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>());
        }
        break;
        case SWARM_FOLLOW:
        {
            // follow leader in [b]
            if (!cmd.contains("a") || !cmd["a"].is_number_integer()) return ;
            if (!cmd.contains("b") || !cmd["b"].is_string()) return ;
                        
                        
            const int follower_index = cmd["a"].get<int>();
            const std::string leader_party_id = cmd["b"].get<std::string>();
            swarm::ANDRUAV_SWARM_FORMATION follower_formation = swarm::ANDRUAV_SWARM_FORMATION::FORMATION_NO_SWARM;
            if (cmd.contains("d") && cmd["d"].is_number_integer()) 
            {
                follower_formation = (swarm::ANDRUAV_SWARM_FORMATION) cmd["d"].get<int>();
            }
                        
            followLeader(leader_party_id, follower_index, follower_formation, sender);
        }
        return ;
        case SWARM_CHANGE_FORMATION:
        {
            // this request should be from leader only.
            if (sender!=getLeader()) return ;

            if (!cmd.contains("d") || !cmd["d"].is_number_integer()) 
            {
                break ;
            }
            swarm::ANDRUAV_SWARM_FORMATION follower_formation = (swarm::ANDRUAV_SWARM_FORMATION) cmd["d"].get<int>();
            m_formation_as_follower = follower_formation;
            de::fcb::CFCBFacade::getInstance().API_IC_sendID(std::string());
        }
        break;
        default:
        break;
    }
}

void CSwarmManager::handleSwarmMavlink(const Json_de& andruav_message, const char * full_message, const int & full_message_length)
{
    const std::string leader_sender = andruav_message[ANDRUAV_PROTOCOL_SENDER].get<std::string>();
    de::fcb::swarm::CSwarmFollower& swarm_follower = de::fcb::swarm::CSwarmFollower::getInstance();
    swarm_follower.handle_leader_traffic(leader_sender, full_message, full_message_length);
}


void CSwarmManager::handleUpdateSwarm(const Json_de& andruav_message, const char * full_message, const int & full_message_length)
{
    /*
        a: action [SWARM_UPDATED, SWARM_DELETE]
        b: follower index [mandatory with SWARM_UPDATED]
        c: leader id - if this is not me then consider it a notification.
        d: slave party id 
    */

    const Json_de cmd = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_CMD];
    de::fcb::CFCBMain&  fcbMain = de::fcb::CFCBMain::getInstance();
                
    if (!cmd.contains("a") || !cmd["a"].is_number_integer()) return ;
    if (!cmd.contains("c") || !cmd["c"].is_string()) return ;
    if (!cmd.contains("d") || !cmd["d"].is_string()) return ;
                

    if (fcbMain.getAndruavVehicleInfo().party_id.compare(cmd["c"].get<std::string>())!=0)
    {
        // this is an announcement, other drone should handle this message.
        return ;
    }

    const int action = cmd["a"].get<int>();
    const std::string follower_party_id = cmd["d"].get<std::string>();
                    
    if (action == SWARM_ADD)
    {   
        // add or modify swarm member
        addFollower(follower_party_id);

        return ;     
    }

    if (action == SWARM_DELETE)
    {
        // remove a swarm member
        releaseSingleFollower(follower_party_id);

        return ;     
    }
}
