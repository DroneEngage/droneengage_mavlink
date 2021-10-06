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

            void sendID(const std::string&target_party_id) const;
            void requestID(const std::string&target_party_id) const;
            void sendTelemetryPanic(const std::string&target_party_id) const;
            void sendErrorMessage(const std::string&target_party_id, const int& error_number, const int& info_type, const int& notification_type, const std::string& description) const;
            void sendGPSInfo(const std::string&target_party_id) const;
            void sendNavInfo(const std::string&target_party_id) const;
            void sendIMUInfo(const std::string&target_party_id) const;
            void sendPowerInfo(const std::string&target_party_id) const;
            void sendHomeLocation(const std::string&target_party_id) const;
            void sendFCBTargetLocation(const std::string&target_party_id, const double &latitude, const double &longitude, const double &altitude) const;
            void sendWayPoints(const std::string&target_party_id) const;
            void sendTelemetryData(const std::string&target_party_id, const mavlink_message_t& mavlink_message) const;
            void sendMavlinkData(const std::string&target_party_id, const mavlink_message_t& mavlink_message)  const;
            void sendMavlinkData_2(const std::string&target_party_id, const mavlink_message_t& mavlink_message1, const mavlink_message_t& mavlink_message2)  const;
            void sendServoReadings(const std::string&target_party_id) const;
            void sendWayPointReached (const std::string&target_party_id, const int& mission_sequence) const;
            
            void setSendJMSG (SEND_JMSG_CALLBACK sendJMSG)
            {
                m_sendJMSG = sendJMSG;
            };
            
            void setSendBMSG (SEND_BMSG_CALLBACK sendBMSG)
            {
                m_sendBMSG = sendBMSG;
            };

        private:
            mavlinksdk::CMavlinkSDK& m_mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
            
            SEND_JMSG_CALLBACK m_sendJMSG = NULL;
            SEND_BMSG_CALLBACK m_sendBMSG = NULL;
    };
}
}
#endif