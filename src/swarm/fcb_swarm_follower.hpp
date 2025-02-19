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

        private:

            void updateFollower();
            void updateFollowerInThreadFormation();

        private:
            
            mavlink_global_position_int_t m_leader_gpos_new;
            mavlink_global_position_int_t m_leader_gpos_old;
            u_int64_t m_leader_last_access;

        private:
            
            de::fcb::swarm::CSwarmManager& m_fcb_swarm_manager = de::fcb::swarm::CSwarmManager::getInstance();
        
    };
}
}
}
#endif