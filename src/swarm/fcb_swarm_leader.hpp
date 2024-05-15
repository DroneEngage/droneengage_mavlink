
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


        private:

            void updateFollowers();
            void updateFollowersThreadFormation();


        private:
            
            mavlink_global_position_int_t m_leader_gpos_latest;
            u_int64_t m_leader_last_access;

        private:
            
            de::fcb::swarm::CSwarmManager& m_fcb_swarm_manager = de::fcb::swarm::CSwarmManager::getInstance();

        
    };
}
}
}
#endif