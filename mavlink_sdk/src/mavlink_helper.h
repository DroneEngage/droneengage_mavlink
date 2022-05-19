#ifndef MAVLINK_HELPER_H_
#define MAVLINK_HELPER_H_

#include <iostream>
#include <string>

namespace mavlinksdk
{


    class CMavlinkHelper 
    {
        private: 
            // Disallow creating an instance of this class.
            CMavlinkHelper(){};       

        public:

            static std::string getMissionACKResult (const int& result);

            static std::string getACKError (const int& result);        
            
    };

}

#endif //MAVLINK_HELPER_H_