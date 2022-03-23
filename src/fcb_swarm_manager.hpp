#ifndef FCB_SWARM_MANAGER_H_
#define FCB_SWARM_MANAGER_H_

#include <iostream>
#include <vector>

namespace uavos
{
namespace fcb
{
// Swarm Formation
#define FORMATION_SERB_NO_SWARM     0
#define FORMATION_SERB_THREAD       1
#define FORMATION_SERB_VECTOR       2 // requires angle
#define FORMATION_SERB_VECTOR_180   3

    typedef enum
    {
        FORMATION_NO_SWARM      = 0, 
        FORMATION_THREAD        = 0, 
        FORMATION_VECTOR        = 0, 
        FORMATION_VECTOR_180    = 0, 

    } ANDRUAV_SWARM_FORMATION;

    typedef struct 
    {
        std::string party_id;
        int slave_index;

    } ANDRUAV_UNIT_SALVE;


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

            bool isLeader() const;
            bool isSlave() const;
            void unFollow();
            ANDRUAV_SWARM_FORMATION getFormation() const;
            void makeSwarm(const ANDRUAV_SWARM_FORMATION formation);
            void makeSlave(const std::string& party_id, const int slave_index);
            void addSlave (const std::string& party_id, const int slave_index);
            int slaveExists (const std::string& party_id) const;
            std::string getLeader() const;
            
        private:
            ANDRUAV_SWARM_FORMATION m_formation;
            std::vector <ANDRUAV_UNIT_SALVE> m_slave_units;
            std::string m_leader_party_id;
            /**
             * @brief valid if I am a slave -follower-.
             * 
             */
            int m_slave_index=-1;
            ANDRUAV_SWARM_FORMATION m_slave_formation;
    } ;
}// namespace
}// namespace

#endif