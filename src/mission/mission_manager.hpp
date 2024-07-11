#ifndef MISSION_MANAGER_H_
#define MISSION_MANAGER_H_

#include <iostream>


namespace de
{
namespace fcb
{
namespace mission
{
    class CMissionManager
    {
        public:

            
            static CMissionManager& getInstance()
            {
                static CMissionManager instance;

                return instance;
            }

            CMissionManager(CMissionManager const&)            = delete;
            void operator=(CMissionManager const&)              = delete;

        
            private:

                CMissionManager() 
                {
                   
                }

                
            public:
                
                ~CMissionManager ()
                {

                }



            public:

                void uploadMissionIntoSystem(const std::string& plan_text);

    };
}
}
}
#endif