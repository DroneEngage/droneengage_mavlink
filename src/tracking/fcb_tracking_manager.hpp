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

                void onTrack(const double x, const double yz, const bool is_xy);
                void onStatusChanged(const int status);

                
            private:

                bool m_tracking_running = false;
                bool m_object_detected = false;
            
    };
}
}
}

#endif