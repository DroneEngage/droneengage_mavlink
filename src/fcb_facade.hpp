#ifndef FCB_FACADE_H_
#define FCB_FACADE_H_

#include <iostream>

#include <mavlink_sdk.h>

#include "global.hpp"
#include "defines.hpp"



namespace uavos
{
namespace fcb
{
    /**
     * @brief Mainly this class handles communication received from FCB or andruav fcb unit internal status
     * and sends it to communicator and server.
     * 
     */
    class CFCBFacade
    {

        public:

            public:
            //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
            static CFCBFacade& getInstance()
            {
                static CFCBFacade instance;

                return instance;
            }

            CFCBFacade(CFCBFacade const&)            = delete;
            void operator=(CFCBFacade const&)        = delete;

        
            // Note: Scott Meyers mentions in his Effective Modern
            //       C++ book, that deleted functions should generally
            //       be public as it results in better error messages
            //       due to the compilers behavior to check accessibility
            //       before deleted status

        private:

            CFCBFacade() 
            {
            }

            
        public:
            
            ~CFCBFacade ()
            {

            }


        public:

            void sendID(const std::string&target_party_id);
            void requestID(const std::string&target_party_id);
            void sendTelemetryPanic(const std::string&target_party_id);
            void sendErrorMessage(const std::string&target_party_id, const int& error_number, const int& info_type, const int& notification_type, const std::string& description);
            void sendGPSInfo(const std::string&target_party_id);
            void sendNavInfo(const std::string&target_party_id);
            void sendIMUInfo(const std::string&target_party_id);
            void sendPowerInfo(const std::string&target_party_id);
            void sendHomeLocation(const std::string&target_party_id);
            void sendWayPoints(const std::string&target_party_id);
            void sendTelemetryData(const std::string&target_party_id);
            void sendServoReadings(const std::string&target_party_id);
            void sendWayPointReached (const std::string&target_party_id, const int& mission_sequence);
        
            void setSendJMSG (SENDJMSG_CALLBACK sendJMSG)
            {
                m_sendJMSG = sendJMSG;
            };

        protected:
            mavlinksdk::CMavlinkSDK& m_mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
            
            SENDJMSG_CALLBACK m_sendJMSG = NULL;
    };
}
}
#endif