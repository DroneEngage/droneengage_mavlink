#include <math.h>       /* floor */
#include <memory>
#include <thread>
#include <mutex>
#include "./helpers/json.hpp"
using Json = nlohmann::json;
#include "./uavos_common/messages.hpp"

#include "./mission/missions.hpp"
#include "fcb_traffic_optimizer.hpp"
#include "fcb_swarm_manager.hpp"
#include "fcb_facade.hpp"
#include "fcb_main.hpp"



using namespace uavos::fcb;


           
void CFCBFacade::sendID(const std::string&target_party_id)  const
{
    /*
         VT : vehicle type
         FM : flying mode
         GM : gps mode
         FI : use FCB
        [AR]: is armed                            // optional 
        [FL]: is flying                           // optional
         SD : shutdown
         TP : telemetry protocol
         [z]: flying last start time              // optional 
         [a]: flying total duration.              // optional 
         [b]: is in cv tracking mode.             // optional 
          C : manual TX blocked mode subAction    
          B : is GCSBlocked
          o : leader formation -if I am a leader.
          q : partyID I follow. -if I am following someone.

    */
    CFCBMain&  fcbMain = CFCBMain::getInstance();
    CSwarmManager& fcb_swarm_manager = CSwarmManager::getInstance();

    mavlinksdk::CVehicle &vehicle =  mavlinksdk::CVehicle::getInstance();
        
    const ANDRUAV_VEHICLE_INFO& andruav_vehicle_info = fcbMain.getAndruavVehicleInfo();

    Json message =
        {
            {"VT", andruav_vehicle_info.vehicle_type},     
            {"FM", andruav_vehicle_info.flying_mode},     
            {"AP", andruav_vehicle_info.autopilot},
            {"GM", andruav_vehicle_info.gps_mode},
            {"FI", andruav_vehicle_info.use_fcb},
            {"AR", vehicle.isArmed()},
            {"FL", vehicle.isFlying()},
            {"TP", fcbMain.isFCBConnected()?TelemetryProtocol_DroneKit_Telemetry:TelemetryProtocol_No_Telemetry},
            //{"SD", false},
            {"z", andruav_vehicle_info.flying_last_start_time / 1000},
            {"a", andruav_vehicle_info.flying_total_duration / 1000},
            {"b", andruav_vehicle_info.is_tracking_mode},
            {"C", andruav_vehicle_info.rc_sub_action},
            {"B", andruav_vehicle_info.is_gcs_blocked},

            {"o", (int)fcb_swarm_manager.getFormation()},
            {"q", fcb_swarm_manager.getLeader()}
        };
        

        

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message, TYPE_AndruavMessage_ID, true);
    }
    
    return ;
}


void CFCBFacade::requestID(const std::string&target_party_id)  const
{
    Json message = 
        {
            {"C", TYPE_AndruavMessage_ID}
        };
        

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message, TYPE_AndruavMessage_RemoteExecute, true);
    }
    
    return ;
}

void CFCBFacade::sendErrorMessage (const std::string&target_party_id, const int& error_number, const int& info_type, const int& notification_type, const std::string& description)  const
{
    /*
        EN : error number  "not currently processed".
        IT : info type indicate what component is reporting the error.
        NT : sevirity and com,pliant with ardupilot.
        DS : description message.
    */
    Json message =
        {
            {"EN", error_number},
            {"IT", info_type},
            {"NT", notification_type},
            {"DS", description}
        };

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message, TYPE_AndruavMessage_Error, false);
    }
 
    std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ << "sendErrorMessage " << _NORMAL_CONSOLE_TEXT_ << description << std::endl;
    
    return ;
}

void CFCBFacade::sendTelemetryPanic(const std::string& target_party_id)  const
{

}


void CFCBFacade::sendGPSInfo(const std::string&target_party_id)  const
{
    /*
        3D          : int 3D fix
        GS          : bool GPS internal from FCB or external "injected"
        SATC        : satellite count
        [g]         : ground altitude
        [p]         : location exists
        [la]        : latitude
        [ln]        : longitude
        [a]         : relative altitude
        [pa]        : predicted latitude
        [pn]        : predicted longitude
        [s]         : speed m/s
        [b]         : bearing
        [c]         : accuracy
    */

    if (m_sendJMSG == NULL) return ;
    
    mavlinksdk::CVehicle&  vehicle =  mavlinksdk::CVehicle::getInstance();

    const mavlink_gps_raw_int_t& gps = vehicle.getMSGGPSRaw();
    const mavlink_global_position_int_t&  gpos = vehicle.getMsgGlobalPositionInt();

    const int sys_id = m_mavlink_sdk.getSysId();
    const int comp_id = m_mavlink_sdk.getCompId();

    mavlink_message_t mavlink_message1,mavlink_message2;
    mavlink_msg_gps_raw_int_encode(sys_id, comp_id, &mavlink_message1, &gps);
        
	mavlink_msg_global_position_int_encode(sys_id, comp_id, &mavlink_message2, &gpos);
    sendMavlinkData_2 (target_party_id, mavlink_message1, mavlink_message2);
    
    return ;
}


void CFCBFacade::sendLocationInfo () const 
{
    /*
        la          : latitude   [degE7]
        ln          : longitude  [degE7]
        a           : absolute altitude
        r           : relative altitude
    */
    if (m_sendJMSG == NULL) return ;
    
    mavlinksdk::CVehicle&  vehicle =  mavlinksdk::CVehicle::getInstance();
    const mavlink_global_position_int_t&  gpos = vehicle.getMsgGlobalPositionInt();
    const mavlink_gps_raw_int_t& gps = vehicle.getMSGGPSRaw();
    
    Json message=
    {
        {"la", gpos.lat},           // latitude   [degE7]
        {"ln", gpos.lon},           // longitude  [degE7]
        {"a", gpos.alt},            // absolute altitude in mm
        {"r", gpos.relative_alt},    // relative altitude in mm
        {"ha", gps.h_acc},          // position uncertanty in mm
        {"y", gps.yaw},             // yaw in cdeg
        // {"gs", groundspeed}, // ground speed
        // {"ws", windspeed}   
    };

    m_sendJMSG ("", message, TYPE_AndruavModule_Location_Info, true);
    
    
}

/**
 * @brief sends mavlink @link mavlink_attitude_t @endlink @link mavlink_nav_controller_output_t @endlink
 * 
 * @param target_party_id 
 */
void CFCBFacade::sendNavInfo(const std::string&target_party_id)  const
{
    /*
        a : nav_roll
        b : nav_pitch
        y : nav_yaw
        d : target_bearing 
        e : wp_dist
        f : alt_error
    */

    if (m_sendJMSG == NULL) return ;
    
    mavlinksdk::CVehicle &vehicle =  mavlinksdk::CVehicle::getInstance();
    const mavlink_attitude_t& attitude = vehicle.getMsgAttitude();
    const mavlink_nav_controller_output_t& nav_controller = vehicle.getMsgNavController();
    
    // Obsolete
    // Json message =
    // {
    //     {"a", attitude.roll},
    //     {"b", attitude.pitch},
    //     {"y", attitude.yaw},
    //     {"d", nav_controller.target_bearing},
    //     {"e", nav_controller.wp_dist},
    //     {"f", nav_controller.alt_error},
    // };

    // Send Mavlink
    const int sys_id = m_mavlink_sdk.getSysId();
    const int comp_id = m_mavlink_sdk.getCompId();

    mavlink_message_t mavlink_message1,mavlink_message2;
    
    mavlink_msg_attitude_encode(sys_id, comp_id, &mavlink_message1, &attitude);
    mavlink_msg_nav_controller_output_encode(sys_id, comp_id, &mavlink_message2, &nav_controller);
    
    sendMavlinkData_2 (target_party_id, mavlink_message1, mavlink_message2);
    
    return ;
}

/**
 * @brief Send complete list of parameters in chunks.
 * 
 * @param target_party_id 
 */
void CFCBFacade::sendParameterList (const std::string&target_party_id) const 
{
    if (m_sendJMSG == NULL) return ;
    
    const int sys_id = m_mavlink_sdk.getSysId();
    const int comp_id = m_mavlink_sdk.getCompId();

    mavlink_message_t mavlink_message;
    	
    mavlinksdk::CMavlinkParameterManager &parameter_manager =  mavlinksdk::CMavlinkParameterManager::getInstance();
    if (!parameter_manager.isParametersListAvailable())
    {
        sendErrorMessage(std::string(), 0, ERROR_TYPE_LO7ETTA7AKOM, NOTIFICATION_TYPE_WARNING, std::string("Still Loading Parameters."));
        return ;
    }
    const std::map<std::string, mavlink_param_value_t>& parameters_list = parameter_manager.getParametersList();
    
    char buf[600];
    unsigned total_length =0;

    for (auto it = parameters_list.begin(); it != parameters_list.end(); it++)
    {
        mavlink_param_value_t param_message = it->second;
        mavlink_msg_param_value_encode(sys_id, comp_id, &mavlink_message, &param_message);
        unsigned len = mavlink_msg_to_send_buffer((uint8_t*)&buf[total_length], &mavlink_message);
        total_length += len;

        std::cout << it->first << " len:" << std::to_string(len) << " value:" <<std::to_string(param_message.param_value) << std::endl;

        if (total_length > 500)
        {
            m_sendBMSG (target_party_id, buf, total_length, TYPE_AndruavMessage_MAVLINK, false, Json());
            total_length = 0;
        }
    }

    if (total_length >0)
    {
        m_sendBMSG (target_party_id, buf, total_length, TYPE_AndruavMessage_MAVLINK, false, Json());
    }
}

/**
 * @brief Send single parameter.
 * 
 * @param target_party_id 
 * @param param_message 
 */
void CFCBFacade::sendParameterValue (const std::string&target_party_id, const mavlink_param_value_t& param_message) const 
{
    if (m_sendJMSG == NULL) return ;
    
    const int sys_id = m_mavlink_sdk.getSysId();
    const int comp_id = m_mavlink_sdk.getCompId();

    mavlink_message_t mavlink_message;
    
    mavlink_msg_param_value_encode(sys_id, comp_id, &mavlink_message, &param_message);

    sendMavlinkData (target_party_id, mavlink_message);

    return ;
}

void CFCBFacade::sendPowerInfo(const std::string&target_party_id)  const
{

    if (m_sendJMSG == NULL) return ;
    
    mavlinksdk::CVehicle &vehicle =  mavlinksdk::CVehicle::getInstance();
    
    const mavlink_battery_status_t& battery_status = vehicle.getMsgBatteryStatus();

    const int sys_id = m_mavlink_sdk.getSysId();
    const int comp_id = m_mavlink_sdk.getCompId();

    mavlink_message_t mavlink_message;
    mavlink_msg_battery_status_encode(sys_id, comp_id, &mavlink_message, &battery_status);
    sendMavlinkData(target_party_id, mavlink_message);
    
    return ;
}


void CFCBFacade::sendHomeLocation(const std::string&target_party_id)  const
{
    
    if (m_sendJMSG == NULL) return ;
    
    mavlinksdk::CVehicle &vehicle =  mavlinksdk::CVehicle::getInstance();
    const mavlink_home_position_t& home = vehicle.getMsgHomePosition();
    
    /*
        T : latitude in xx.xxxxx
        O : longitude in xx.xxxxx
        A : altitude in meters
    */
    Json message=
    {
        {"T", home.latitude / 10000000.0f},
        {"O", home.longitude / 10000000.0f},
        {"A", home.altitude / 1000.0f}
    };

    m_sendJMSG (target_party_id, message, TYPE_AndruavMessage_HomeLocation, false);
    
    return ;
}

/**
* @brief Send points of destination guided point.
* @details This function sends destination points after confirmed from FCB.
* GCS sends these points to uavos and then oavos sends it to FCB then uavos should send it back to gcs as a confirmation.
*/
void CFCBFacade::sendFCBTargetLocation(const std::string&target_party_id, const double &latitude, const double &longitude, const double &altitude) const
{
    
    if (m_sendJMSG == NULL) return ;
    
    /*
        T : latitude in xx.xxxxx
        O : longitude in xx.xxxxx
        A : altitude in meters
    */
    Json message=
    {
        {"T", latitude},
        {"O", longitude},
        {"A", altitude}
    };

    m_sendJMSG (target_party_id, message, TYPE_AndruavMessage_DistinationLocation, false);
    
    return ;
}


std::mutex g_pages_mutex;

/**
* @details Chunked Waypoint version
* In this version field "i" is mandatory
* "i" = WAYPOINT_CHUNK except the last one is WAYPOINT_LAST_CHUNK
* A chunk may contain zero or more waypoints.
* "n" represents number of waypoint in the chunk.
* waypoints are numbered zero-based in each chunk.
*/
void CFCBFacade::sendWayPoints(const std::string&target_party_id) const
{

    if (m_sendJMSG == NULL) return ;
    
    std::lock_guard<std::mutex> guard(g_pages_mutex);
    
    CFCBMain&  fcbMain = CFCBMain::getInstance();
    const mission::ANDRUAV_UNIT_MISSION& andruav_missions = fcbMain.getAndruavMission(); 
    const std::size_t length = andruav_missions.mission_items.size();
    

    if (length ==0)
    {
        Json message =
        {
            {"n", 0}
        };

        m_sendJMSG (target_party_id, message, TYPE_AndruavMessage_WayPoints, false);
        
        return ;
    }

    #define MAX_WAYPOINT_CHUNK  2

    for (int i=0; i< length; i+=MAX_WAYPOINT_CHUNK)
    {
        Json message;
        int lastsentIndex = 0;
        for (int j =0; (j<MAX_WAYPOINT_CHUNK) && (j+i < length); ++j)
        {
            auto it = andruav_missions.mission_items.find(i+j);
            mission::CMissionItem *mi= it->second.get();
            
            Json message_item = mi->getAndruavMission();
            message[std::to_string(lastsentIndex)] = message_item;
            
            lastsentIndex++;   
        }

        message ["n"] = lastsentIndex;
        if (lastsentIndex + i >= length )
        {
            message["i"] = WAYPOINT_LAST_CHUNK;
        }
        else
        {
            message["i"] = WAYPOINT_CHUNK;
        }

        m_sendJMSG (target_party_id, message, TYPE_AndruavMessage_WayPoints, false);
        
    }

    #undef MAX_WAYPOINT_CHUNK
 
    return ;
}

void CFCBFacade::sendTelemetryData(const std::string&target_party_id, const mavlink_message_t& mavlink_message)  const
{
    char buf[300];
    // Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &mavlink_message);
	if (len >= 300) 
	{
		std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "ERROR LEN = " << std::to_string(len) << _NORMAL_CONSOLE_TEXT_ << std::endl;
		return ;
	}

    m_sendBMSG (target_party_id, buf, len, TYPE_AndruavMessage_LightTelemetry, false, Json());
    
    return ;
}

void CFCBFacade::sendMavlinkData(const std::string&target_party_id, const mavlink_message_t& mavlink_message)  const
{
    char buf[300];
    // Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &mavlink_message);
	if (len >= 300) 
	{
		std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "ERROR LEN = " << std::to_string(len) << _NORMAL_CONSOLE_TEXT_ << std::endl;
		return ;
	}

    m_sendBMSG (target_party_id, buf, len, TYPE_AndruavMessage_MAVLINK, false, Json());
    
    return ;
}


void CFCBFacade::sendMavlinkData_2(const std::string&target_party_id, const mavlink_message_t& mavlink_message1, const mavlink_message_t& mavlink_message2)  const
{
    char buf[600];
    // Translate message to buffer
	unsigned len1 = mavlink_msg_to_send_buffer((uint8_t*)buf, &mavlink_message1);
	unsigned len2 = mavlink_msg_to_send_buffer((uint8_t*)&buf[len1], &mavlink_message2);
    unsigned len = len1+len2;
	if (len >= 600) 
	{
		std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "ERROR LEN = " << std::to_string(len) << _NORMAL_CONSOLE_TEXT_ << std::endl;
		return ;
	}

    m_sendBMSG (target_party_id, buf, len, TYPE_AndruavMessage_MAVLINK, false, Json());
    
    return ;
}

void CFCBFacade::sendServoReadings(const std::string&target_party_id)  const
{
    Json message;

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message, TYPE_AndruavMessage_LightTelemetry, false);;
    }

    return ;
}


void CFCBFacade::sendWayPointReached (const std::string&target_party_id, const int& mission_sequence)  const
{
    if (m_sendJMSG == NULL) return ;
    
    /*
        R: Report Type
        P: Parameter1
    */

    Json message =
    {
        {"R", Drone_Report_NAV_ItemReached},
        {"P", mission_sequence}
    };

    m_sendJMSG (target_party_id, message, TYPE_AndruavMessage_DroneReport, false);
    
    return ;
}

/**
 * @brief 
 * 
 * @param target_party_id 
 * @param fence_name null means all fences of party
 */
void CFCBFacade::sendGeoFenceAttachedStatusToTarget(const std::string&target_party_id, const std::string&fence_name) const
{
    if (m_sendJMSG == NULL) return ;
    
    
    if (fence_name.empty()==true)
    {   //walk through all fences.
        std::vector<geofence::GEO_FENCE_STRUCT*> geo_fence_struct_list = geofence::CGeoFenceManager::getInstance().getFencesOfParty(uavos::fcb::CFCBMain::getInstance().getAndruavVehicleInfo().party_id);
        
        const std::size_t size = geo_fence_struct_list.size();

        for(int i = 0; i < size; i++)
        {
            Json message =
                {
                    {"n", geo_fence_struct_list[i]->geoFence.get()->getName()},
                    {"a", true}
                };
                
                m_sendJMSG (target_party_id, message, TYPE_AndruavMessage_GeoFenceAttachStatus, false);
        }
    }
    else
    {
        Json message =
                {
                    {"n", fence_name}
                };
                
        geofence::GEO_FENCE_STRUCT * geo_fence_struct = geofence::CGeoFenceManager::getInstance().getFenceByName(fence_name);
        message["a"] = ((geo_fence_struct != nullptr) && (geofence::CGeoFenceManager::getInstance().getIndexOfPartyInGeoFence(uavos::fcb::CFCBMain::getInstance().getAndruavVehicleInfo().party_id, geo_fence_struct)>=0));
        
        m_sendJMSG (target_party_id, message, TYPE_AndruavMessage_GeoFenceAttachStatus, false);
    }

        
    
    return ;
}


/**
 * @brief Sends fence shape, dimension, actions to a target.
 * 
 * @param target_party_id 
 * @param geo_fenct_struct 
 */
void CFCBFacade::sendGeoFenceToTarget(const std::string&target_party_id, const geofence::GEO_FENCE_STRUCT * geo_fenct_struct) const
{
    uavos::fcb::geofence::CGeoFenceBase* geo_fence_base = geo_fenct_struct->geoFence.get();

    if (geo_fence_base == nullptr) return ;
    
    Json message = geo_fence_base->getMessage();
    

    m_sendJMSG (target_party_id, message, TYPE_AndruavMessage_GeoFence, false);
    
}


/**
 * @brief Send hit status
 * 
 * @param target_party_id 
 * @param fence_name 
 * @param distance 
 * @param in_zone 
 * @param should_keep_outside 
 */
void CFCBFacade::sendGeoFenceHit(const std::string&target_party_id, const std::string fence_name, const double distance, const bool in_zone, const bool should_keep_outside) const
{
    Json message =
            {
                {"n", fence_name},
                {"z", in_zone},
                {"d", distance},
                {"o", should_keep_outside}
            };
                
    m_sendJMSG (target_party_id, message, TYPE_AndruavMessage_GEOFenceHit, false);
        
}


/**
 * @brief sends SYNC Event
 * 
 * @param target_party_id 
 * @param event_id 
 */
void CFCBFacade::sendSyncEvent(const std::string&target_party_id, const int event_id ) const
{
    Json message =
            {
                {"a", event_id}
            };
                
    m_sendJMSG (target_party_id, message, TYPE_AndruavMessage_Sync_EventFire, false);
   
}


void CFCBFacade::requestToFollowLeader(const std::string&target_party_id, const int slave_index) const
{
    /*
        a: action
        b: slave index
        c: leader id
        d: slave party id 
    */
    Json message =
            {
                {"a", SWARM_UPDATED},
                {"b", slave_index},
                {"c", target_party_id},
                {"d", uavos::fcb::CFCBMain::getInstance().getAndruavVehicleInfo().party_id}
            };
                
    m_sendJMSG (target_party_id, message, TYPE_AndruavMessage_UpdateSwarm, false);

}


/**
 * @brief sends a message to my leader informing it that I am no longer following it.
 * @details this message is sent from a drone folower to a leader drone. It can also be sent from a third party.
 * It is up to the leader to approve or reject the request.
 * 
 * @param target_party_id 
 */
void CFCBFacade::requestUnFollowLeader(const std::string&target_party_id) const
{
    /*
        a: action
        b: slave index
        c: leader id
        d: slave party id 
    */
    Json message =
            {
                {"a", SWARM_DELETE},
                {"c", target_party_id},
                {"d", uavos::fcb::CFCBMain::getInstance().getAndruavVehicleInfo().party_id}
            };
                
    m_sendJMSG (target_party_id, message, TYPE_AndruavMessage_UpdateSwarm, false);

}

/**
 * @brief send inter-module-command to communication module to load tasks.
 * @details it is up to communication module to handle this command. 
 * this is NOT LoadTasksCommand that is sent to server. It is a request to communication 
 * and it deceides how and when it executes it.
 * 
 * @param inter_module_command 
 */
void CFCBFacade::callModule_reloadSavedTasks(const int& inter_module_command)
{
    m_sendMREMSG(inter_module_command);
}


void CFCBFacade::internalCommand_takeImage()
{
    /*
        a: channelName  []
        b: numberOfImages
        c: timeBetweenShots
        d: distanceBetweenShots
    */
    Json message =
            {
                {"a", ""},  //first available camera
                {"b", 1},
                {"c", 0},
                {"d",0}
            };
            
    m_sendJMSG (std::string(), message, TYPE_AndruavMessage_Ctrl_Cameras, true);
    std::cout << "TYPE_AndruavMessage_Ctrl_Cameras" << std::endl;
}