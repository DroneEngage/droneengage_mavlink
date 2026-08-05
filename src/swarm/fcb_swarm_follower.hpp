#ifndef FCB_SWARM_FOLLOWER_H_
#define FCB_SWARM_FOLLOWER_H_

#include <iostream>
#include <mavlink_sdk.h>
#include "fcb_swarm_manager.hpp"
namespace de
{
namespace fcb
{
namespace swarm
{
    class CSwarmFollower
    {
        public:

            
            static CSwarmFollower& getInstance()
            {
                static CSwarmFollower instance;

                return instance;
            }

            CSwarmFollower(CSwarmFollower const&)            = delete;
            void operator=(CSwarmFollower const&)            = delete;

        
            // Note: Scott Meyers mentions in his Effective Modern
            //       C++ book, that deleted functions should generally
            //       be public as it results in better error messages
            //       due to the compilers behavior to check accessibility
            //       before deleted status

        private:

            CSwarmFollower()
            {
                
            }

            
        public:
            
            ~CSwarmFollower ()
            {

            }

        public:

            void handle_leader_traffic(const std::string & leader_sender, const char * full_message, const int & full_message_length);

        public: 

            void setMinVerticalDistance (const int min_vertical_distance)
            {
                m_min_vertical_distance = min_vertical_distance;
            };
            void setMinHorizontalDistance (const int min_horizontal_distance)
            {
                m_min_horizontal_distance = min_horizontal_distance;
            };
            int getMinVerticalDistance ()
            {
                return m_min_vertical_distance;
            };
            int getMinHorizontalDistance ()
            {
                return m_min_horizontal_distance;
            };

        private:

            void updateFollower();
            void updateFollowerInThreadFormation();
            void updateFollowerInArrowFormation();

        private:
            
            mavlink_global_position_int_t m_leader_gpos_new;
            mavlink_global_position_int_t m_leader_gpos_old;
            u_int64_t m_leader_last_access;

            // Bearing latching for arrow formation — prevents violent swings
            double m_last_leader_bearing = 0.0;
            bool m_has_last_bearing = false;

            // Target position smoothing — low-pass filter on goto target
            double m_smoothed_target_lat = 0.0;
            double m_smoothed_target_lon = 0.0;
            bool m_has_smoothed_target = false;

            // Leader attitude (parsed from ATTITUDE message)
            float m_leader_yaw = 0.0f;
            float m_leader_yawspeed = 0.0f;

            // Dynamic side assignment for arrow formation (hysteresis)
            int m_last_side = 0;  // -1 = left, +1 = right, 0 = unassigned

        private:
            
            int m_min_vertical_distance = KNODE_LENGTH;
            int m_min_horizontal_distance = KNODE_LENGTH;
    };
}
}
}
#endif