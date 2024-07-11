#ifndef MISSION_TRANSLATOR_H_
#define MISSION_TRANSLATOR_H_

#include <iostream>
#include <memory>

#include "missions.hpp"

namespace de
{
namespace fcb
{
namespace mission
{
    class CMissionTranslator 
    {
        public:

            std::unique_ptr<std::map <int, std::unique_ptr<CMissionItem>>> translateMissionText (const std::string& mission_text);

        protected:

            std::unique_ptr<std::map <int, std::unique_ptr<CMissionItem>>> translateQGCFormat (const std::string& mission_text);
            std::unique_ptr<std::map <int, std::unique_ptr<CMissionItem>>> translateMPFormat (const std::string& mission_text);
            std::unique_ptr<std::map <int, std::unique_ptr<CMissionItem>>> translateDEFormat (const std::string& mission_text);


    };
}
}
}

#endif