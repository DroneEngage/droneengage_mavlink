#ifndef FCB_FACADE_H_
#define FCB_FACADE_H_

#include <iostream>

#include "global.hpp"
#include "defines.hpp"


namespace uavos
{
namespace fcb
{
    class CFCBFacade
    {

        public:

            CFCBFacade ()
            {
            
            }


        public:

            void  sendID(const char *target_party_id, const ANDRUAV_VEHICLE_INFO& andruav_vehicle_info);
            void  requestID(const char *target_party_id);
            void  sendTelemetryPanic(const char *target_party_id);
            void  sendErrorMessage(const char *target_party_id, const std::uint8_t severity, const std::string& status);
            void  sendGPSInfo(const char *target_party_id);
            void  sendNavInfo(const char *target_party_id);
            void  sendIMUInfo(const char *target_party_id);
            void  sendPowerInfo(const char *target_party_id);
            void  sendHomeLocation(const char *target_party_id);
            void  sendWayPoints(const char *target_party_id);
            void  sendTelemetryData(const char *target_party_id);
            void  sendServoReadings(const char *target_party_id);

        
            void setSendJMSG (SENDJMSG_CALLBACK sendJMSG)
            {
                m_sendJMSG = sendJMSG;
            }

        private:

            SENDJMSG_CALLBACK m_sendJMSG = NULL;
    };
}
}
#endif