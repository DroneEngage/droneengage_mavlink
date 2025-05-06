
#ifndef FCB_SWARM_LEADER_H_
#define FCB_SWARM_LEADER_H_

#include <iostream>
#include <mavlink_sdk.h>
#include "fcb_swarm_manager.hpp"

namespace de
{
namespace fcb
{
namespace swarm
{
    class CSwarmLeader
    {
        public:

            
            static CSwarmLeader& getInstance()
            {
                static CSwarmLeader instance;

                return instance;
            }

            CSwarmLeader(CSwarmLeader const&)            = delete;
            void operator=(CSwarmLeader const&)          = delete;

        
            // Note: Scott Meyers mentions in his Effective Modern
            //       C++ book, that deleted functions should generally
            //       be public as it results in better error messages
            //       due to the compilers behavior to check accessibility
            //       before deleted status

        private:

            CSwarmLeader()
            {
                
            }

            
        public:
            
            ~CSwarmLeader ()
            {

            }

        public:

            void handleSwarmsAsLeader();


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

            void updateFollowers();
            void updateFollowersThreadFormation();
            void updateFollowerInArrowFormation();

        private:
            
            mavlink_global_position_int_t m_leader_gpos_latest;
            u_int64_t m_leader_last_access;

        private:
            
            de::fcb::swarm::CSwarmManager& m_fcb_swarm_manager = de::fcb::swarm::CSwarmManager::getInstance();
          
            int m_min_vertical_distance = KNODE_LENGTH;
            int m_min_horizontal_distance = KNODE_LENGTH;
        
    };
}
}
}
#endif