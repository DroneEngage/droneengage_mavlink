#ifndef FCB_MAIN_H_
#define FCB_MAIN_H_
#include <common/mavlink.h>
#include <mavlink_sdk.h>
#include "defines.hpp"
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

            /* cannot connect to uavos comm*/
            void alertUavosOffline ();
            

        protected:
            mavlinksdk::CMavlinkSDK& m_mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
            
        
        protected:
            int getConnectionType ();
            bool connectToFCB ();


        protected:
            Json m_jsonConfig;
            int m_connection_type;
    };
}
#endif