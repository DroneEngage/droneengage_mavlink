#ifndef FCB_FACADE_H_
#define FCB_FACADE_H_

#include <iostream>

#include <mavlink_sdk.h>

#include "global.hpp"
#include "defines.hpp"
#include "./uavos_common/uavos_module.hpp"
#include "./uavos_common/udpProxy.hpp"
#include "./geofence/fcb_geo_fence_base.hpp"
#include "./geofence/fcb_geo_fence_manager.hpp"



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
            void sendHighLatencyInfo(const std::string&target_party_id) const;
            void sendEKFInfo(const std::string&target_party_id) const;
            void sendVibrationInfo(const std::string&target_party_id) const;
            void sendGPSInfo(const std::string&target_party_id) const;
            void sendNavInfo(const std::string&target_party_id) const;
            void sendWindInfo (const std::string&target_party_id) const;
            void sendTerrainReport (const std::string&target_party_id) const;
            void sendLocationInfo() const; 
            void sendParameterList (const std::string&target_party_id) const;
            void sendParameterValue (const std::string&target_party_id, const mavlink_param_value_t& param_message) const;
            void sendPowerInfo(const std::string&target_party_id) const;
            void sendHomeLocation(const std::string&target_party_id) const;
            void sendFCBTargetLocation(const std::string&target_party_id, const double &latitude, const double &longitude, const double &altitude) const;
            void sendWayPoints(const std::string&target_party_id) const;
            void sendTelemetryData(const std::string&target_party_id, const mavlink_message_t& mavlink_message) const;
            void sendMavlinkData(const std::string&target_party_id, const mavlink_message_t& mavlink_message)  const;
            void sendMavlinkData_2(const std::string&target_party_id, const mavlink_message_t& mavlink_message1, const mavlink_message_t& mavlink_message2)  const;
            void sendMavlinkData_3(const std::string&target_party_id, const mavlink_message_t& mavlink_message1, const mavlink_message_t& mavlink_message2, const mavlink_message_t& mavlink_message3)  const;
            void sendMavlinkData_M(const std::string&target_party_id, const mavlink_message_t* mavlink_message, const uint16_t count)  const;
            void sendServoReadings(const std::string&target_party_id) const;
            void sendWayPointReached (const std::string&target_party_id, const int& mission_sequence) const;
            void sendGeoFenceAttachedStatusToTarget(const std::string&target_party_id, const std::string&fence_name) const;
            void sendGeoFenceToTarget(const std::string&target_party_id, const geofence::GEO_FENCE_STRUCT * geo_fenct_struct) const;
            void sendGeoFenceHit(const std::string&target_party_id, const std::string fence_name, const double distance, const bool in_zone, const bool should_keep_outside) const;
            void sendSyncEvent(const std::string&target_party_id, const int event_id ) const;
            void requestToFollowLeader(const std::string&target_party_id, const int slave_index) const;
            void requestUnFollowLeader(const std::string&target_party_id) const;
            
            void requestUdpProxyTelemetry(const bool start, const std::string&udp_ip1, const int& udp_port1, const std::string&udp_ip2, const int& udp_port2);
            void sendUdpProxyStatus(const std::string&target_party_id, const bool& start, const std::string&udp_ip_other, const int& udp_port_other, const int& optimization_level);
            // Inter Module Remote Execute Commands - commands executed by other modules in droneengage.
            void callModule_reloadSavedTasks (const int& inter_module_command);
            void internalCommand_takeImage () const;
            void sendUdpProxyMavlink(const mavlink_message_t& mavlink_message, uavos::comm::CUDPProxy& udp_client) const;
                 

            void RegisterSendSYSMSG (SEND_SYSMSG_CALLBACK sendSYSMSG)
            {
                m_sendSYSMSG = sendSYSMSG;
            };
            
            /**
             * @brief call back function to send JSON message either 
             * to communicator or other modules or server.
             * 
             * @param sendJMSG 
             */
            void RegisterSendJMSG (SEND_JMSG_CALLBACK sendJMSG)
            {
                m_sendJMSG = sendJMSG;
            };
            
            /**
             * @brief callback function to send binary message
             * 
             * @param sendBMSG 
             */
            void RegisterSendBMSG (SEND_BMSG_CALLBACK sendBMSG)
            {
                m_sendBMSG = sendBMSG;
            };
            
            /**
             * @brief callback function to send remote-module-execute messages
             * 
             * @param sendMREMSG 
             */
            void RegisterSendMREMSG (SEND_MREMSG_CALLBACK sendMREMSG)
            {
                m_sendMREMSG = sendMREMSG;
            };

        private:
            mavlinksdk::CMavlinkSDK& m_mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
            SEND_SYSMSG_CALLBACK    m_sendSYSMSG    = NULL;
            SEND_JMSG_CALLBACK      m_sendJMSG      = NULL;
            SEND_BMSG_CALLBACK      m_sendBMSG      = NULL;
            SEND_MREMSG_CALLBACK    m_sendMREMSG    = NULL;
    };
}
}
#endif