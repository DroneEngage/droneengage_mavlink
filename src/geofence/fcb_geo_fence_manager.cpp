#include "../geofence/fcb_geo_fence_manager.hpp"


using namespace uavos::fcb::geofence;

void CGeoFenceManager::addFence (std::unique_ptr<uavos::fcb::geofence::CGeoFenceBase> geo_fence)
{
    std::string geo_fence_name = geo_fence.get()->getName();
    std::unique_ptr<uavos::fcb::geofence::GEO_FENCE_STRUCT> fence_sturct = std::unique_ptr<uavos::fcb::geofence::GEO_FENCE_STRUCT>(new uavos::fcb::geofence::GEO_FENCE_STRUCT);
    fence_sturct.get()->geoFence = std::move(geo_fence);
    m_geo_fences.get()->insert(std::make_pair(geo_fence_name, std::move(fence_sturct)));
}

void CGeoFenceManager::attachToGeoFence (const std::string& party_id, const std::string& geo_fence_name)
{
    GEO_FENCE_STRUCT * geo_fence_struct = getFenceByName(geo_fence_name);
    
    if (geo_fence_struct == NULL) return ;

    attachToGeoFence(party_id, geo_fence_struct);
}


void CGeoFenceManager::attachToGeoFence (const std::string party_id, GEO_FENCE_STRUCT *geo_fence)
{
    if (party_id.empty()) return ;
    geo_fence->parties.push_back(party_id);
}


/**
 * @brief removes a partyid from a fence object.
 * @details use @link getFenceByName() @endlink to get object.
 * @param party_id 
 * @param geo_fence 
 */
void CGeoFenceManager::detachFromGeoFence (const std::string& party_id, GEO_FENCE_STRUCT *geo_fence)
{
    const std::size_t size = geo_fence->parties.size();

    for(int i = 0; i < size; i++){

        if(geo_fence->parties[i].find(party_id)!=std::string::npos)
        {
            geo_fence->parties.erase(geo_fence->parties.begin() + i);
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
    if (geo_fence_name.empty()) return NULL;

    std::map<std::string,std::unique_ptr<uavos::fcb::geofence::GEO_FENCE_STRUCT>>::iterator it;
                
    for (it = m_geo_fences.get()->begin(); it != m_geo_fences.get()->end(); it++)
    {
        if (it->first.find(geo_fence_name)!=std::string::npos)
        {
            return  it->second.get();   
        }
    }

    return NULL;
}



void CGeoFenceManager::clearGeoFences ()
{
   m_geo_fences.get()->clear();
}
