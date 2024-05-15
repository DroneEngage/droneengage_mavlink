#ifndef FCB_GEO_FENCE_MANAGER_H_
#define FCB_GEO_FENCE_MANAGER_H_


#include <map>
#include <vector>

#include "../geofence/fcb_geo_fence_base.hpp"

namespace de
{
namespace fcb
{
namespace geofence
{
    /**
     * @brief Attached unit status
     * 
     */
    typedef struct 
    {
        std::string party_id; 
        /**
         * @brief <=0 then touch >0 distance to center or edge depends on the shape.
         * initial value -INFINITY used to trigeer testing
         */
        double in_zone=-INFINITY; 
        bool violation = false;
    } GEO_FENCE_PARTY_STATUS;

    /**
     * @brief Holds geo fence object with its attached units.
     * 
     */
    typedef struct 
    {
        std::unique_ptr<de::fcb::geofence::CGeoFenceBase> geoFence;
        std::vector<std::unique_ptr<GEO_FENCE_PARTY_STATUS>> parties;
        int local_index;
    } GEO_FENCE_STRUCT;

    class CGeoFenceManager
    {
        public:

            
            //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
            static CGeoFenceManager& getInstance()
            {
                static CGeoFenceManager instance;

                return instance;
            }

            CGeoFenceManager(CGeoFenceManager const&)            = delete;
            void operator=(CGeoFenceManager const&)              = delete;

        
            // Note: Scott Meyers mentions in his Effective Modern
            //       C++ book, that deleted functions should generally
            //       be public as it results in better error messages
            //       due to the compilers behavior to check accessibility
            //       before deleted status

            private:

                CGeoFenceManager() 
                {
                   m_geo_fences = std::unique_ptr <std::map<std::string,std::unique_ptr<de::fcb::geofence::GEO_FENCE_STRUCT>>>(new std::map<std::string,std::unique_ptr<de::fcb::geofence::GEO_FENCE_STRUCT>>);
                }

                
            public:
                
                ~CGeoFenceManager ()
                {

                }


            
            public:

                void addFence (std::unique_ptr<de::fcb::geofence::CGeoFenceBase> geo_fence);
                void attachToGeoFence (const std::string& party_id, const std::string& geo_fence_name);
                void attachToGeoFence (const std::string party_id, GEO_FENCE_STRUCT *geo_fence_struct);
                
                void detachFromGeoFence (const std::string& party_id, GEO_FENCE_STRUCT *geo_fence_struct);
                void clearGeoFences (const std::string& geo_fence_name);
                GEO_FENCE_STRUCT * getFenceByName (const std::string& geo_fence_name) const;
                std::vector<GEO_FENCE_STRUCT*> getFencesOfParty (const std::string& party_id);
                int getIndexOfPartyInGeoFence (const std::string& party_id, const GEO_FENCE_STRUCT *geo_fence_struct) const;
                
                
            
            protected:

                std::unique_ptr <std::map<std::string,std::unique_ptr<GEO_FENCE_STRUCT>>>  m_geo_fences;
    };
}
}
}

#endif