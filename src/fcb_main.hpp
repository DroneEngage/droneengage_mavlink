#ifndef FCB_MAIN_H_
#define FCB_MAIN_H_

#include "./helpers/json.hpp"
using Json = nlohmann::json;

namespace uavos::FCB
{
    class CFCBMain
    {
        public:
            //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
            static CFCBMain& getInstance()
            {
                static CFCBMain instance;

                return instance;
            }

            CFCBMain(CFCBMain const&)               = delete;
            void operator=(CFCBMain const&)         = delete;

        private:

            CFCBMain()
            {

            }

        public:
            
            ~CFCBMain ();

        public:

            void init (const Json &jsonConfig);

    };
}
#endif