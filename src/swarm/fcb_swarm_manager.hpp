#ifndef FCB_SWARM_MANAGER_H_
#define FCB_SWARM_MANAGER_H_

#include <iostream>
#include <vector>
#include <unordered_map>

namespace uavos
{
namespace fcb
{
namespace swarm
{
// Swarm Formation
#define FORMATION_SERB_NO_SWARM     0
#define FORMATION_SERB_THREAD       1
#define FORMATION_SERB_VECTOR       2 // requires angle
#define FORMATION_SERB_VECTOR_180   3

    typedef enum
    {
        FORMATION_NO_SWARM      = 0, 
        FORMATION_THREAD        = 1, 
        FORMATION_VECTOR        = 2, 
        FORMATION_VECTOR_180    = 3, 

    } ANDRUAV_SWARM_FORMATION;

    typedef struct 
    {
        std::string party_id;
        int follower_index;

    } ANDRUAV_UNIT_FOLLOWER;


    class CSwarmManager
    {

        public:

            
            //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
            static CSwarmManager& getInstance()
            {
                static CSwarmManager instance;

                return instance;
            }

            CSwarmManager(CSwarmManager const&)            = delete;
            void operator=(CSwarmManager const&)            = delete;

        
            // Note: Scott Meyers mentions in his Effective Modern
            //       C++ book, that deleted functions should generally
            //       be public as it results in better error messages
            //       due to the compilers behavior to check accessibility
            //       before deleted status

        private:

            CSwarmManager():m_leader_party_id(std::string(""))
            {
                
            }

            
        public:
            
            ~CSwarmManager ()
            {

            }


        public:

            void followLeader(const std::string& leader_party_id, const int follower_index, const ANDRUAV_SWARM_FORMATION follower_formation, const std::string& party_id_request);
            void unFollowLeader(const std::string& party_id_leader_to_unfollow, const std::string& party_id_request);
            ANDRUAV_SWARM_FORMATION getFormationAsLeader() const;
            ANDRUAV_SWARM_FORMATION getFormationAsFollower() const;
            void makeSwarm(const ANDRUAV_SWARM_FORMATION formation);
            void addFollower (const std::string& party_id, const int follower_index);
            void releaseSingleFollower (const std::string& party_id);
            void releaseFollowers ();
            int followerExist (const std::string& party_id) const;
            int insertFollowerinSwarmFormation(const std::string& party_id);

        public:

            void handle_swarm_as_leader();
            

        public:

            inline std::string getLeader() const
            {
                return m_leader_party_id;
            }

            inline bool isMyLeader(const std::string& leader_party_id) const
            {
                return  (m_leader_party_id.compare (leader_party_id)==0);
            }
        
            /**
             * @brief a unit can be a follower and a leader at the same time
             * 
             * @return true 
             * @return false 
             */
            inline bool isLeader() const
            {
                return m_is_leader;
            }

            /**
             * @brief a unit can be a follower and a leader at the same time.
             * Note: m_follower_formation can be 0 ubtill leader confirms.
             * but m_follower_index can be set by GCS as initial request.
             * @return true 
             * @return false 
             */
            inline bool isFollower() const
            {
                return m_follower_index != -1;
            }

            inline int getFollowerIndex() const
            {
                return m_follower_index;
            }

            
        private:
            ANDRUAV_SWARM_FORMATION m_formation_as_leader;
            bool m_is_leader = false;
            //std::unordered_map<uint16_t, ANDRUAV_UNIT_FOLLOWER> m_follower_units; 
            std::vector <ANDRUAV_UNIT_FOLLOWER> m_follower_units;
            
            std::string m_leader_party_id;
            /**
             * @brief valid if I am a follower.
             * 
             */
            int m_follower_index=-1;
            ANDRUAV_SWARM_FORMATION m_formation_as_follower;
            
    } ;
}// namespace
}// namespace
}// namespace

#endif