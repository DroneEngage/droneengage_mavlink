#ifndef FCB_TRACKING_MANAGER_H_
#define FCB_TRACKING_MANAGER_H_

#include <chrono> // For high-resolution timing
#include <iostream>
#include <mavlink_sdk.h>

#include "pic_controller.hpp"


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
                
                void init();
                void onTrack(const double x, const double yz, const bool is_xy);
                void onStatusChanged(const int status);
            
            public:
                inline void setParameters(const double x_PID_P, const double yz_PID_P, const double x_PID_I, const double yz_PID_I, const double alpha)
                {
                    m_x_PID_P = x_PID_P;
                    m_yz_PID_P = yz_PID_P;
                    m_x_PID_I = x_PID_I;
                    m_yz_PID_I = yz_PID_I;
                    m_alpha = alpha;
                }
                
            private:

                bool m_tracking_running = false;
                bool m_object_detected = false;

                
                CPIDController m_PID_X;
                CPIDController m_PID_YZ;
                
                double m_x;
                double m_yz;
                double m_alpha = 0.1;
                double m_x_PID_P = 1.0;
                double m_yz_PID_P = 1.0;
                double m_x_PID_I = 1.0;
                double m_yz_PID_I = 1.0;
                      
                std::chrono::high_resolution_clock::time_point m_last_message_time;
                const std::chrono::milliseconds m_target_frame_time_ms = std::chrono::milliseconds(500); 
    };
}
}
}

#endif