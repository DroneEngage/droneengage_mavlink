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



typedef void (*SEND_SYSMSG_CALLBACK)(const Json_de&, const int& );

/**
 * @brief sends JSON packet
 * 
 * @param targetPartyID 
 * @param jmsg 
 * @param andruav_message_id 
 * @param internal_message if true @link INTERMODULE_MODULE_KEY @endlink equaqls to Module key
 */
typedef void (*SEND_JMSG_CALLBACK)(const std::string& targetPartyID, const Json_de&, const int&, const bool& );

typedef void (*SEND_BMSG_CALLBACK)(const std::string& targetPartyID, const char *, const int bmsg_length, const int& , const bool&, const Json_de& );
typedef void (*SEND_MREMSG_CALLBACK)(const int& );



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

            void API_IC_sendID(const std::string&target_party_id) const;
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
            void sendADSBVehicleInfo(const std::string&target_party_id) const;
            void sendDistanceSensorInfo(const std::string&target_party_id) const;
            void sendDistanceSensorInfo(const std::string&target_party_id,const mavlink_distance_sensor_t& distance_sensor) const;
            void sendLocationInfo() const; 
            void sendParameterList (const std::string&target_party_id) const;
            void sendParameterValue (const std::string&target_party_id, const mavlink_param_value_t& param_message) const;
            void sendPowerInfo(const std::string&target_party_id) const;
            void sendHomeLocation(const std::string&target_party_id) const;
            void sendFCBTargetLocation(const std::string&target_party_id, const double &latitude, const double &longitude, const double &altitude, const int &target_type) const;
            void sendWayPoints(const std::string&target_party_id) const;
            void sendTelemetryData(const std::string&target_party_id, const mavlink_message_t& mavlink_message) const;
            void sendMavlinkData(const std::string&target_party_id, const mavlink_message_t& mavlink_message)  const;
            void sendMavlinkData_3(const std::string&target_party_id, const mavlink_message_t& mavlink_message1, const mavlink_message_t& mavlink_message2, const mavlink_message_t& mavlink_message3)  const;
            void sendMavlinkData_M(const std::string&target_party_id, const mavlink_message_t* mavlink_message, const uint16_t count)  const;
            void sendServoReadings(const std::string&target_party_id) const;
            void sendWayPointReached(const std::string&target_party_id, const int& mission_sequence) const;
            void sendMissionCurrent(const std::string&target_party_id) const;
            void sendGeoFenceAttachedStatusToTarget(const std::string&target_party_id, const std::string&fence_name) const;
            void sendGeoFenceToTarget(const std::string&target_party_id, const geofence::GEO_FENCE_STRUCT * geo_fenct_struct) const;
            void sendGeoFenceHit(const std::string&target_party_id, const std::string fence_name, const double distance, const bool in_zone, const bool should_keep_outside) const;
            void sendSyncEvent(const std::string&target_party_id, const int event_id ) const;
            
            // SWARM API
            void requestToFollowLeader(const std::string&target_party_id, const int follower_index) const;
            void requestUnFollowLeader(const std::string&target_party_id) const;
            void requestFromUnitToFollowMe(const std::string&target_party_id, const int follower_index) const;
            void requestFromUnitToUnFollowMe(const std::string&target_party_id) const;
            void sendSWARM_M(const std::string&target_party_id, const mavlink_message_t* mavlink_message, const uint16_t count)  const;
            // END SWARM API
            
            void requestUdpProxyTelemetry(const bool enable, const std::string&udp_ip1, const int& udp_port1, const std::string&udp_ip2, const int& udp_port2);
            void sendUdpProxyStatus(const std::string&target_party_id, const bool& enabled, const bool& paused, const std::string&udp_ip_other, const int& udp_port_other, const int& optimization_level);
            // Inter Module Remote Execute Commands - commands executed by other modules in droneengage.
            void callModule_reloadSavedTasks (const int& inter_module_command);
            void internalCommand_takeImage () const;
            void sendUdpProxyMavlink(const mavlink_message_t& mavlink_message, uavos::comm::CUDPProxy& udp_client) const;
                 

        public:

            void API_IC_P2P_connectToMeshOnMac (const std::string& target_party_id) const;
            void API_IC_P2P_accessMac (const std::string& target_party_id) const;

        private:
            mavlinksdk::CVehicle&    m_vehicle      =  mavlinksdk::CVehicle::getInstance();

            uavos::comm::CModule &m_module = uavos::comm::CModule::getInstance();            
    };
}
}
#endif