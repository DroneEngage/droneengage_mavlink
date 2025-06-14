#ifndef FCB_TRACKING_MANAGER_H_
#define FCB_TRACKING_MANAGER_H_

#include <iostream>
#include <mavlink_sdk.h>




#include "../helpers/json_nlohmann.hpp"
using Json_de = nlohmann::json;


namespace de
{
namespace fcb
{
namespace tracking
{
    class CTrackingManager
    {
        public:
            static CTrackingManager& getInstance()
                {
                    static CTrackingManager instance;

                    return instance;
                }

                CTrackingManager(CTrackingManager const&)            = delete;
                void operator=(CTrackingManager const&)            = delete;

            
                
            private:

                CTrackingManager()
                {
                    
                }

                
            public:
                
                ~CTrackingManager ()
                {

                }


            public:

                void enableTracking(const bool detected);
                void onTrack(const double x, const double yz, const bool is_xy);


            public:

                void onTargetAccuired(const bool detected);
                
            private:

                
            
    };
}
}
}

#endif