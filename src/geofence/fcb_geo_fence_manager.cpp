#include "../de_common/messages.hpp"
#include "../geofence/fcb_geo_fence_manager.hpp"
#include "../fcb_traffic_optimizer.hpp"
#include "../mission/missions.hpp"
#include "../fcb_facade.hpp"
#include "../fcb_main.hpp"


using namespace de::fcb::geofence;

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
    geo_fence_struct->local_index = getIndexOfPartyInGeoFence(de::fcb::CFCBMain::getInstance().getAndruavVehicleInfo().party_id, geo_fence_struct);
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
            geo_fence_struct->local_index = getIndexOfPartyInGeoFence(de::fcb::CFCBMain::getInstance().getAndruavVehicleInfo().party_id, geo_fence_struct);
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
                de::fcb::CFCBMain&  fcbMain = de::fcb::CFCBMain::getInstance();
                for (const auto& json_fence : json_fences) {
                    // Access individual elements of the fence object
                    std::cout << "Fence property: " << json_fence << std::endl;
                    std::unique_ptr<geofence::CGeoFenceBase> fence = geofence::CGeoFenceFactory::getInstance().getGeoFenceObject(json_fence);
                    addFence(std::move(fence));
                    std::cout << fcbMain.getAndruavVehicleInfo().party_id << std::endl;
                    attachToGeoFence(fcbMain.getAndruavVehicleInfo().party_id, json_fence["n"].get<std::string>());
                }
            }
       }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}


