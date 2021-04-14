#ifndef CCONFIGFILE_H

#define CCONFIGFILE_H

#include <sstream>
#include "./helpers/json.hpp"
using Json = nlohmann::json;

namespace uavos
{
    class CConfigFile 
    {

        public:
            //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
            static CConfigFile& getInstance()
            {
                static CConfigFile    instance; // Guaranteed to be destroyed.
                                                // Instantiated on first use.
                return instance;
            }
            CConfigFile(CConfigFile const&)             = delete;
            void operator=(CConfigFile const&)          = delete;

            // Note: Scott Meyers mentions in his Effective Modern
            //       C++ book, that deleted functions should generally
            //       be public as it results in better error messages
            //       due to the compilers behavior to check accessibility
            //       before deleted status
        private:
            CConfigFile() {}                    // Constructor? (the {} brackets) are needed here.

            // C++ 11
            // =======
            // We can use the better technique of deleting the methods
            // we don't want.
            

        public:
            void InitConfigFile (const char* fileURL);
            const Json& GetConfigJSON();

        protected:
            void ReadFile (const char * fileURL);
            void ParseData (std::string jsonString);
            

        private:
            std::stringstream m_fileContents;
            Json m_ConfigJSON;
        

    };
}

#endif
