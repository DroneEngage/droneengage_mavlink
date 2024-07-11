#include "../helpers/colors.hpp"
#include "../helpers/helpers.hpp"

#include "../de_common/messages.hpp"

#include "../fcb_modes.hpp"
#include "../geofence/fcb_geo_fence_manager.hpp"
#include "../fcb_traffic_optimizer.hpp"
#include "../mission/missions.hpp"
#include "../fcb_facade.hpp"
#include "../fcb_main.hpp"


using namespace de::fcb::geofence;


de::fcb::CFCBMain&  m_fcbMain = de::fcb::CFCBMain::getInstance();
de::fcb::CFCBFacade& m_fcb_facade = de::fcb::CFCBFacade::getInstance();
            

void CGeoFenceManager::addFence (std::unique_ptr<de::fcb::geofence::CGeoFenceBase> geo_fence)
{
    std::string geo_fence_name = geo_fence.get()->getName();
    std::unique_ptr<de::fcb::geofence::GEO_FENCE_STRUCT> fence_sturct = std::unique_ptr<de::fcb::geofence::GEO_FENCE_STRUCT>(new de::fcb::geofence::GEO_FENCE_STRUCT);
    fence_sturct.get()->geoFence = std::move(geo_fence);
    m_geo_fences.get()->insert(std::make_pair(geo_fence_name, std::move(fence_sturct)));
}

void CGeoFenceManager::attachToGeoFence (const std::string& party_id, const std::string& geo_fence_name)
{
    GEO_FENCE_STRUCT * geo_fence_struct = getFenceByName(geo_fence_name);
    
    if (geo_fence_struct == nullptr) return ;

    attachToGeoFence(party_id, geo_fence_struct);
}


/**
 * @brief attaches a party into a geo_fence.
 * 
 * @param party_id 
 * @param geo_fence_struct 
 */
void CGeoFenceManager::attachToGeoFence (const std::string party_id, GEO_FENCE_STRUCT *geo_fence_struct)
{
    if (party_id.empty()) return ;

    std::unique_ptr<de::fcb::geofence::GEO_FENCE_PARTY_STATUS> geo_fence_party_status = std::unique_ptr<de::fcb::geofence::GEO_FENCE_PARTY_STATUS>(new de::fcb::geofence::GEO_FENCE_PARTY_STATUS);
    geo_fence_party_status.get()->party_id = party_id;
    geo_fence_struct->parties.push_back(std::move(geo_fence_party_status));
    geo_fence_struct->local_index = getIndexOfPartyInGeoFence(m_fcbMain.getAndruavVehicleInfo().party_id, geo_fence_struct);
    //TODO: Test if inzone or violates 
    //TODO: send fence
    //TODO: send Hit Status
    #ifdef DEBUG
        std::cout << geo_fence_struct->geoFence.get()->getName() << std::endl;
    #endif
    // inform I am attached to fence.
    de::fcb::CFCBFacade::getInstance().sendGeoFenceAttachedStatusToTarget(std::string(""), geo_fence_struct->geoFence.get()->getName());
}


/**
 * @brief removes a partyid from a fence object.
 * @details use @link getFenceByName() @endlink to get object.
 * @param party_id 
 * @param geo_fence_struct 
 */
void CGeoFenceManager::detachFromGeoFence (const std::string& party_id, GEO_FENCE_STRUCT *geo_fence_struct)
{
    const std::size_t size = geo_fence_struct->parties.size();

    for(std::size_t i = 0; i < size; i++){

        if(geo_fence_struct->parties[i].get()->party_id.find(party_id)!=std::string::npos)
        {
            geo_fence_struct->parties.erase(geo_fence_struct->parties.begin() + i);
            geo_fence_struct->local_index = getIndexOfPartyInGeoFence(m_fcbMain.getAndruavVehicleInfo().party_id, geo_fence_struct);
            return ;
        }
    }
}                

/**
 * @brief retreives geo-fence record @link GEO_FENCE_STRUCT @endlink
 * @details geo-fence with related partyID's. _sys_ partyID or my partyID name means it is MINE.. same meaning.
 * 
 * @param geoFenceName 
 * @return GEO_FENCE_STRUCT* 
 */
GEO_FENCE_STRUCT * CGeoFenceManager::getFenceByName (const std::string& geo_fence_name) const
{
    if (geo_fence_name.empty()) return nullptr;

    std::map<std::string,std::unique_ptr<de::fcb::geofence::GEO_FENCE_STRUCT>>::iterator it;
                
    for (it = m_geo_fences.get()->begin(); it != m_geo_fences.get()->end(); it++)
    {
        if (it->first.find(geo_fence_name)!=std::string::npos)
        {
            return  it->second.get();   
        }
    }

    return nullptr;
}


/**
 * @brief std::vector of fences where party is one of the owners.
 * 
 * @return std::vector<GEO_FENCE_STRUCT*> 
 */
std::vector<GEO_FENCE_STRUCT*> CGeoFenceManager::getFencesOfParty (const std::string& party_id)
{
    std::vector<GEO_FENCE_STRUCT*> fence_list;
    if (party_id.empty()) return fence_list;

    std::map<std::string,std::unique_ptr<de::fcb::geofence::GEO_FENCE_STRUCT>>::iterator it;
                
    for (it = m_geo_fences.get()->begin(); it != m_geo_fences.get()->end(); it++)
    {
        GEO_FENCE_STRUCT* geo_fence_struct = it->second.get();
        
        if (getIndexOfPartyInGeoFence (party_id, geo_fence_struct) >= 0)
        {
            fence_list.push_back (geo_fence_struct);   
        }
    }

    return fence_list;

}


/**
 * @brief returns index if a party is member of fence owners else returns -1.
 * 
 * @param party_id 
 * @param geo_fence_struct 
 * @return index of party or -1
 */
int CGeoFenceManager::getIndexOfPartyInGeoFence (const std::string& party_id, const GEO_FENCE_STRUCT *geo_fence_struct) const
{
    const std::size_t size = geo_fence_struct->parties.size();

    for(int i = 0; i < size; i++){

        if(geo_fence_struct->parties[i].get()->party_id.find(party_id)!=std::string::npos)
        {
            
            return i;
        }
    }

    return -1;
}


/**
 * @brief empty all geo fences.
 * 
 */
void CGeoFenceManager::clearGeoFences (const std::string& geo_fence_name)
{

   std::map<std::string,std::unique_ptr<de::fcb::geofence::GEO_FENCE_STRUCT>>::iterator it;
                
    for (it = m_geo_fences.get()->begin(); it != m_geo_fences.get()->end(); it++)
    {
        if ((geo_fence_name.empty()==true) || (it->first.find(geo_fence_name)!=std::string::npos))
        {
            GEO_FENCE_STRUCT* geo_fence_struct = it->second.get();
            geo_fence_struct->geoFence.release();
            geo_fence_struct->parties.clear();
            geo_fence_struct->local_index = -1;
        }
    }

    if (geo_fence_name.empty()==false)
    {
        m_geo_fences.get()->erase(geo_fence_name);
    }
    else
    {
        // otherwise remove all
        m_geo_fences.get()->clear();
    }
}


void CGeoFenceManager::uploadFencesIntoSystem (const std::string& mission_text)
{
    try
    {
        auto fence_items = std::make_unique<std::map<int, std::unique_ptr<de::fcb::geofence::CGeoFenceBase>>>();
        Json_de plan = Json_de::parse(mission_text);
        if (std::string(plan["fileType"]).find("de_plan") != std::string::npos)
        {
            if (plan.contains("fences"))
            {
                // there is a fence data.
                Json_de json_fences = plan["fences"];
                // Iterate through the array
                for (const auto& json_fence : json_fences) {
                    // Access individual elements of the fence object
                    std::cout << "Fence property: " << json_fence << std::endl;
                    std::unique_ptr<geofence::CGeoFenceBase> fence = geofence::CGeoFenceFactory::getInstance().getGeoFenceObject(json_fence);
                    addFence(std::move(fence));
                    std::cout << m_fcbMain.getAndruavVehicleInfo().party_id << std::endl;
                    attachToGeoFence(m_fcbMain.getAndruavVehicleInfo().party_id, json_fence["n"].get<std::string>());
                }
            }
       }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}


/**
 * @brief review status of each attached geo fence
 * 
 * @details each attached geo fence is called isInside() and based on result global fence status is calculated.
 * also status is sent to other parties using update hit status. Actions is taken when hard fences are violated.
 * 
 */
void CGeoFenceManager::updateGeoFenceHitStatus()
{
    /* 
	    bit 0: out of green zone
		bit 1: in bad zone
		bit 2: in good zone
	*/
	int total_violation = 0b000;

    std::vector<geofence::GEO_FENCE_STRUCT*> geo_fence_struct_list = geofence::CGeoFenceManager::getInstance().getFencesOfParty(m_fcbMain.getAndruavVehicleInfo().party_id);
        
    const std::size_t size = geo_fence_struct_list.size();

    mavlinksdk::CVehicle&  vehicle =  mavlinksdk::CVehicle::getInstance();

    const mavlink_global_position_int_t&  gpos = vehicle.getMsgGlobalPositionInt();

    // test each fence and check if inside or not.
    for(int i = 0; i < size; i++)
    {
        de::fcb::geofence::GEO_FENCE_STRUCT * g = geo_fence_struct_list[i];
        de::fcb::geofence::CGeoFenceBase * geo_fence = g->geoFence.get();
        const int local_index = geo_fence_struct_list[i]->local_index;
        double current_position_in_zone = geo_fence->isInside(gpos.lat / 10000000.0f, gpos.lon / 10000000.0f, gpos.alt);
        double previous_position_in_zone = g->parties[local_index].get()->in_zone;
        
        if ((previous_position_in_zone == -INFINITY) || (signum(current_position_in_zone) != signum(previous_position_in_zone)))
        {
            // change status
            //TODO: Alert & Act
            std::cout << "current_position_in_zone" << std::to_string(current_position_in_zone) << std::endl;
            g->parties[local_index].get()->in_zone = current_position_in_zone;
            if ((current_position_in_zone<=0) && geo_fence->shouldKeepOutside()) 
            {
                // violate should be OUTSIDE
                total_violation = total_violation | 0b010; //bad 

                handleFenceViolation(geo_fence);
            }
            else if ((current_position_in_zone>0) && !geo_fence->shouldKeepOutside()) 
            {
                // multiple allowed fences may exist so a viuolation for one is not a violation.
                // a safe green fence but I am not inside it.
                // unless this is the only one.
                total_violation = total_violation | 0b001; // not in greed zone  

            }
            else if  ((current_position_in_zone<=0) && !geo_fence->shouldKeepOutside()) 
            {
                // green fence and I am in.
                total_violation = total_violation | 0b100; // good
                handleFenceEntry(geo_fence);
            }
            
            
            m_fcb_facade.sendGeoFenceHit(std::string(""), 
                                        geo_fence->getName(),
                                        current_position_in_zone, 
                                        current_position_in_zone<=0,
                                        geo_fence->shouldKeepOutside());
        }

    }
}

void CGeoFenceManager::handleFenceViolation(geofence::CGeoFenceBase* geo_fence) {
    std::string error_str = "violate fence " + std::string(geo_fence->getName());
    m_fcb_facade.sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_GEO_FENCE_ERROR, NOTIFICATION_TYPE_ERROR, error_str);
    // Note that action is taken once when state changes.
    // This is important to allow user to reverse action or take other actions.
    // For example if you break you need a mechanize to land or take drone out ...etc.
    takeActionOnFenceViolation(geo_fence);
}

void CGeoFenceManager::handleFenceEntry(geofence::CGeoFenceBase* geo_fence) {
    std::string error_str = "safe fence " + std::string(geo_fence->getName());
    m_fcb_facade.sendErrorMessage(std::string(ANDRUAV_PROTOCOL_SENDER_ALL_GCS), 0, ERROR_GEO_FENCE_ERROR, NOTIFICATION_TYPE_NOTICE, error_str);
}

void CGeoFenceManager::takeActionOnFenceViolation(de::fcb::geofence::CGeoFenceBase * geo_fence)
{
    const int fence_action = geo_fence->hardFenceAction();
    switch (fence_action)
    {
        case CONST_FENCE_ACTION_SOFT:
        {
            return ;
        }
        break;

        case CONST_FENCE_ACTION_RTL:
        case CONST_FENCE_ACTION_LAND:
        case CONST_FENCE_ACTION_LOITER:
        case CONST_FENCE_ACTION_BRAKE:
        case CONST_FENCE_ACTION_SMART_RTL:
        {
            uint32_t ardupilot_mode, ardupilot_custom_mode, ardupilot_custom_sub_mode;
            //TODO: fence_action mode should be generalized to work on different autopilot types.
            CFCBModes::getArduPilotMode(fence_action, m_fcbMain.getAndruavVehicleInfo().vehicle_type, ardupilot_mode , ardupilot_custom_mode, ardupilot_custom_sub_mode);
            if (ardupilot_mode == E_UNDEFINED_MODE)
            {   
                //TODO: Send Error Message
                return ;
            }

            mavlinksdk::CMavlinkCommand::getInstance().doSetMode(ardupilot_mode, ardupilot_custom_mode, ardupilot_custom_sub_mode);
            return ;
        }
        break;

    }
}