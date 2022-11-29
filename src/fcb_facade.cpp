#include <math.h>       /* floor */
#include <memory>
#include <thread>
#include <mutex>
#include "./helpers/json.hpp"
using Json = nlohmann::json;
#include "helpers/helpers.hpp"
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

void CFCBFacade::sendHighLatencyInfo(const std::string&target_party_id) const 
{
    if (m_sendJMSG == NULL) return ;
    
    mavlinksdk::CVehicle&  vehicle =  mavlinksdk::CVehicle::getInstance();

    
    const int sys_id = m_mavlink_sdk.getSysId();
    const int comp_id = m_mavlink_sdk.getCompId();
    mavlink_message_t mavlink_message;

    const int mode = vehicle.getHighLatencyMode();
    
    switch (mode)
    {
        case 0:
        {   // no high latency info ... construct one from available info.

            const mavlink_heartbeat_t& heartbeat = vehicle.getMsgHeartBeat();
            const mavlink_gps_raw_int_t& gps = vehicle.getMSGGPSRaw();
            const mavlink_global_position_int_t&  gpos = vehicle.getMsgGlobalPositionInt();
            const mavlink_nav_controller_output_t& nav_controller = vehicle.getMsgNavController();
            const mavlink_attitude_t& attitude = vehicle.getMsgAttitude();
            const mavlink_battery_status_t& battery_status = vehicle.getMsgBatteryStatus();
            const mavlink_vfr_hud_t& vfr_hud = vehicle.getMsgVFRHud();

            nav_controller.target_bearing;
            mavlink_high_latency2_t high_latency2;
            memset(&high_latency2,0,sizeof(mavlink_high_latency2_t));
            
            high_latency2.timestamp=(get_time_usec()/1000) & 0xffffffff; //millis
            high_latency2.type = heartbeat.type;
            high_latency2.autopilot = heartbeat.autopilot;
            high_latency2.custom_mode = heartbeat.custom_mode;
            high_latency2.airspeed = vfr_hud.airspeed;
            high_latency2.groundspeed = vfr_hud.groundspeed;
            high_latency2.latitude = gpos.lat;
            high_latency2.longitude = gpos.lon;
            high_latency2.altitude = gpos.alt;
            high_latency2.heading = vfr_hud.heading;
            high_latency2.battery = battery_status.battery_remaining;
            high_latency2.wp_num = uavos::fcb::CFCBMain::getInstance().getAndruavVehicleInfo().current_waypoint;
            high_latency2.target_distance = nav_controller.wp_dist;
            mavlink_msg_high_latency2_encode(sys_id, comp_id, &mavlink_message, &high_latency2);

            mavlink_high_latency_t high_latency;
            memset(&high_latency,0,sizeof(mavlink_high_latency_t));
            
    
            high_latency.custom_mode = heartbeat.custom_mode;
            high_latency.airspeed = vfr_hud.airspeed;
            high_latency.groundspeed = vfr_hud.groundspeed;
            high_latency.latitude = gpos.lat;
            high_latency.longitude = gpos.lon;
            high_latency.altitude_amsl = gpos.alt;
            high_latency.heading = vfr_hud.heading;
            high_latency.gps_nsat = gps.satellites_visible;
            high_latency.gps_fix_type = gps.fix_type;
            high_latency.battery_remaining = battery_status.battery_remaining;
            high_latency.wp_distance = nav_controller.wp_dist;
            high_latency.wp_num = uavos::fcb::CFCBMain::getInstance().getAndruavVehicleInfo().current_waypoint;
            high_latency.roll = attitude.roll;
            high_latency.pitch = attitude.pitch;
            
            
            mavlink_msg_high_latency_encode(sys_id, comp_id, &mavlink_message, &high_latency);
        };
        break;

        case MAVLINK_MSG_ID_HIGH_LATENCY:
        {
            if (vehicle.getProcessedFlag(MAVLINK_MSG_ID_HIGH_LATENCY) != MESSAGE_UNPROCESSED) return ;
            vehicle.setProcessedFlag(MAVLINK_MSG_ID_HIGH_LATENCY,MESSAGE_PROCESSED);
            const mavlink_high_latency_t& high_latency = vehicle.getHighLatency();
            mavlink_msg_high_latency_encode(sys_id, comp_id, &mavlink_message, &high_latency);
        }
        break;

        case MAVLINK_MSG_ID_HIGH_LATENCY2:
        {
            if (vehicle.getProcessedFlag(MAVLINK_MSG_ID_HIGH_LATENCY2) != MESSAGE_UNPROCESSED) return ;
            vehicle.setProcessedFlag(MAVLINK_MSG_ID_HIGH_LATENCY2,MESSAGE_PROCESSED);
            const mavlink_high_latency2_t& high_latency2 = vehicle.getHighLatency2();   
            mavlink_msg_high_latency2_encode(sys_id, comp_id, &mavlink_message, &high_latency2);
        }
        break;
    }
    
    sendMavlinkData (target_party_id, mavlink_message);
}


void CFCBFacade::sendEKFInfo(const std::string&target_party_id) const
{
    if (m_sendJMSG == NULL) return ;
    mavlinksdk::CVehicle&  vehicle =  mavlinksdk::CVehicle::getInstance();
    
    const int sys_id = m_mavlink_sdk.getSysId();
    const int comp_id = m_mavlink_sdk.getCompId();

    mavlink_message_t mavlink_message;
    
    if (vehicle.getProcessedFlag(MAVLINK_MSG_ID_EKF_STATUS_REPORT) != MESSAGE_UNPROCESSED) 
    return ;
   
    vehicle.setProcessedFlag(MAVLINK_MSG_ID_EKF_STATUS_REPORT,MESSAGE_PROCESSED);

    const mavlink_ekf_status_report_t&  ekf_status_report = vehicle.getEkf_status_report();
    mavlink_msg_ekf_status_report_encode(sys_id, comp_id, &mavlink_message, &ekf_status_report);
    
    sendMavlinkData (target_party_id, mavlink_message);

    return ;
}

void CFCBFacade::sendVibrationInfo(const std::string&target_party_id) const
{
    if (m_sendJMSG == NULL) return ;
    mavlinksdk::CVehicle&  vehicle =  mavlinksdk::CVehicle::getInstance();
    
    const int sys_id = m_mavlink_sdk.getSysId();
    const int comp_id = m_mavlink_sdk.getCompId();

    mavlink_message_t mavlink_message;
    
    if (vehicle.getProcessedFlag(MAVLINK_MSG_ID_VIBRATION) != MESSAGE_UNPROCESSED) 
    return ;

    vehicle.setProcessedFlag(MAVLINK_MSG_ID_VIBRATION,MESSAGE_PROCESSED);

    const mavlink_vibration_t&  vibration = vehicle.getVibration();
    mavlink_msg_vibration_encode(sys_id, comp_id, &mavlink_message, &vibration);
    
    sendMavlinkData (target_party_id, mavlink_message);

    return ;
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
    
    if (vehicle.getHighLatencyMode()!=0) return ;
    
    const int sys_id = m_mavlink_sdk.getSysId();
    const int comp_id = m_mavlink_sdk.getCompId();

    mavlink_message_t mavlink_message[3];
    int i=0;

    if (vehicle.getProcessedFlag(MAVLINK_MSG_ID_GLOBAL_POSITION_INT) == MESSAGE_UNPROCESSED) 
    {
        vehicle.setProcessedFlag(MAVLINK_MSG_ID_GLOBAL_POSITION_INT,MESSAGE_PROCESSED);
        const mavlink_global_position_int_t&  gpos = vehicle.getMsgGlobalPositionInt();
        mavlink_msg_global_position_int_encode(sys_id, comp_id, &mavlink_message[i], &gpos);
        ++i;
    }
   
    if (vehicle.getProcessedFlag(MAVLINK_MSG_ID_GPS_RAW_INT) == MESSAGE_UNPROCESSED) 
    {
        vehicle.setProcessedFlag(MAVLINK_MSG_ID_GPS_RAW_INT,MESSAGE_PROCESSED);
        const mavlink_gps_raw_int_t& gps = vehicle.getMSGGPSRaw();
        mavlink_msg_gps_raw_int_encode(sys_id, comp_id, &mavlink_message[i], &gps);
        ++i;
    }
    if (vehicle.getProcessedFlag(MAVLINK_MSG_ID_GPS2_RAW) == MESSAGE_UNPROCESSED)
    {
        vehicle.setProcessedFlag(MAVLINK_MSG_ID_GPS2_RAW,MESSAGE_PROCESSED);
        const mavlink_gps2_raw_t& gps2 = vehicle.getMSGGPS2Raw();
        mavlink_msg_gps2_raw_encode(sys_id, comp_id, &mavlink_message[i], &gps2);
        ++i;
    }
    
    
    sendMavlinkData_M (target_party_id, mavlink_message, i);

    return ;
}


void CFCBFacade::sendWindInfo (const std::string&target_party_id) const
{
    if (m_sendJMSG == NULL) return ;
    
    mavlinksdk::CVehicle&  vehicle =  mavlinksdk::CVehicle::getInstance();

    if (vehicle.getHighLatencyMode()!=0) return ;
    
    if (vehicle.getProcessedFlag(MAVLINK_MSG_ID_WIND) != MESSAGE_UNPROCESSED) return ;
    vehicle.setProcessedFlag(MAVLINK_MSG_ID_WIND,MESSAGE_PROCESSED);

    const int sys_id = m_mavlink_sdk.getSysId();
    const int comp_id = m_mavlink_sdk.getCompId();

    mavlink_message_t mavlink_message;
    const mavlink_wind_t& wind = vehicle.getMsgWind();
    mavlink_msg_wind_encode(sys_id, comp_id, &mavlink_message, &wind);
    
    sendMavlinkData (target_party_id, mavlink_message);

    return ;
}


void CFCBFacade::sendTerrainReport (const std::string&target_party_id) const
{
    if (m_sendJMSG == NULL) return ;
    
    mavlinksdk::CVehicle&  vehicle =  mavlinksdk::CVehicle::getInstance();

    if (vehicle.getHighLatencyMode()!=0) return ;
    
    if (vehicle.getProcessedFlag(MAVLINK_MSG_ID_TERRAIN_REPORT) != MESSAGE_UNPROCESSED) return ;
    vehicle.setProcessedFlag(MAVLINK_MSG_ID_TERRAIN_REPORT,MESSAGE_PROCESSED);

    const int sys_id = m_mavlink_sdk.getSysId();
    const int comp_id = m_mavlink_sdk.getCompId();

    mavlink_message_t mavlink_message;
    const mavlink_terrain_report_t& terrain_report = vehicle.getTerrainReport();
    mavlink_msg_terrain_report_encode(sys_id, comp_id, &mavlink_message, &terrain_report);
    
    sendMavlinkData (target_party_id, mavlink_message);

    return ;
    
}


void CFCBFacade::sendADSBVehicleInfo(const std::string&target_party_id) const
{
    if (m_sendJMSG == NULL) return ;
    
    mavlinksdk::CVehicle&  vehicle =  mavlinksdk::CVehicle::getInstance();

    if (vehicle.getHighLatencyMode()!=0) return ;
    
    if (vehicle.getProcessedFlag(MAVLINK_MSG_ID_ADSB_VEHICLE) != MESSAGE_UNPROCESSED) return ;
    vehicle.setProcessedFlag(MAVLINK_MSG_ID_WIND,MESSAGE_PROCESSED);

    const int sys_id = m_mavlink_sdk.getSysId();
    const int comp_id = m_mavlink_sdk.getCompId();

    mavlink_message_t mavlink_message;
    const mavlink_adsb_vehicle_t& adsb_vehicle = vehicle.getADSBVechile();
    mavlink_msg_adsb_vehicle_encode(sys_id, comp_id, &mavlink_message, &adsb_vehicle);
    
    sendMavlinkData (target_party_id, mavlink_message);

    return ;
}

void CFCBFacade::sendDistanceSensorInfo(const std::string&target_party_id,const mavlink_distance_sensor_t& distance_sensor) const 
{
    if (m_sendJMSG == NULL) return ;
    
    mavlinksdk::CVehicle&  vehicle =  mavlinksdk::CVehicle::getInstance();

    if (vehicle.getHighLatencyMode()!=0) return ;
    
    // this is not accurate check as there are many messages of different orientation with the same id
    // but if we send them all then it is safe to check this.
    if (vehicle.getProcessedFlag(MAVLINK_MSG_ID_DISTANCE_SENSOR) != MESSAGE_UNPROCESSED) return ;
    vehicle.setProcessedFlag(MAVLINK_MSG_ID_DISTANCE_SENSOR,MESSAGE_PROCESSED);

    const int sys_id = m_mavlink_sdk.getSysId();
    const int comp_id = m_mavlink_sdk.getCompId();

    mavlink_message_t mavlink_message;
    mavlink_msg_distance_sensor_encode(sys_id, comp_id, &mavlink_message, &distance_sensor);
    
    sendMavlinkData (target_party_id, mavlink_message);

    return ;
}

void CFCBFacade::sendDistanceSensorInfo(const std::string&target_party_id) const
{

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
    if (mavlinksdk::CVehicle::getInstance().getHighLatencyMode()!=0) return ;
               
    mavlinksdk::CVehicle &vehicle =  mavlinksdk::CVehicle::getInstance();
    const mavlink_attitude_t& attitude = vehicle.getMsgAttitude();
    const mavlink_nav_controller_output_t& nav_controller = vehicle.getMsgNavController();
    const mavlink_vfr_hud_t& vfr_hud = vehicle.getMsgVFRHud();
    
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

    mavlink_message_t mavlink_message1, mavlink_message2, mavlink_message3;
    
    mavlink_msg_attitude_encode(sys_id, comp_id, &mavlink_message1, &attitude);
    mavlink_msg_nav_controller_output_encode(sys_id, comp_id, &mavlink_message2, &nav_controller);
    mavlink_msg_vfr_hud_encode(sys_id, comp_id, &mavlink_message3, &vfr_hud);

    sendMavlinkData_3 (target_party_id, mavlink_message1, mavlink_message2, mavlink_message3);
    
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
    if (mavlinksdk::CVehicle::getInstance().getHighLatencyMode()!=0) return ;
    
    mavlinksdk::CVehicle &vehicle =  mavlinksdk::CVehicle::getInstance();
    
    const int sys_id = m_mavlink_sdk.getSysId();
    const int comp_id = m_mavlink_sdk.getCompId();

    
    mavlink_message_t mavlink_message[2];
    int i=0;

    if (vehicle.getProcessedFlag(MAVLINK_MSG_ID_BATTERY_STATUS) == MESSAGE_UNPROCESSED) 
    {
        vehicle.setProcessedFlag(MAVLINK_MSG_ID_BATTERY_STATUS,MESSAGE_PROCESSED);
        const mavlink_battery_status_t& battery_status = vehicle.getMsgBatteryStatus();    
        mavlink_msg_battery_status_encode(sys_id, comp_id, &mavlink_message[i], &battery_status);
        ++i;
    }


    if (vehicle.getProcessedFlag(MAVLINK_MSG_ID_BATTERY2) == MESSAGE_UNPROCESSED) 
    {
        vehicle.setProcessedFlag(MAVLINK_MSG_ID_BATTERY2,MESSAGE_PROCESSED);
        const mavlink_battery2_t& battery2 = vehicle.getMsgBattery2Status();    
        mavlink_msg_battery2_encode(sys_id, comp_id, &mavlink_message[i], &battery2);
        ++i;
    }


    sendMavlinkData_M(target_party_id, mavlink_message, i);
    
    return ;
}




void CFCBFacade::sendMissionCurrent(const std::string&target_party_id, const mavlink_mission_current_t& mission_current) const
{
    if (m_sendJMSG == NULL) return ;

    mavlinksdk::CVehicle &vehicle =  mavlinksdk::CVehicle::getInstance();
    
    const int sys_id = m_mavlink_sdk.getSysId();
    const int comp_id = m_mavlink_sdk.getCompId();

    
    mavlink_message_t mavlink_message[2];
    

    // you do not need to do thefollowing check as you can resent this message to different GCS once then become online.
    // there is no obsolete message here.
    //if (vehicle.getProcessedFlag(MAVLINK_MSG_ID_MISSION_COUNT) == MESSAGE_UNPROCESSED) 
    vehicle.setProcessedFlag(MAVLINK_MSG_ID_MISSION_COUNT,MESSAGE_PROCESSED);
    mavlink_msg_mission_current_encode(sys_id, comp_id, &mavlink_message[0], &mission_current);
    
    const mavlink_mission_count_t mission_count = mavlinksdk::CMavlinkWayPointManager::getInstance().getMissionCount();
    mavlink_msg_mission_count_encode(sys_id, comp_id, &mavlink_message[1], &mission_count);
    

    sendMavlinkData_M(target_party_id, mavlink_message, 2);
    
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

void CFCBFacade::sendUdpProxyMavlink(const mavlink_message_t& mavlink_message, uavos::comm::CUDPProxy& udp_client) const
{

     char buf[300];
    // Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &mavlink_message);
	if (len >= 300) 
	{
		std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "ERROR LEN = " << std::to_string(len) << _NORMAL_CONSOLE_TEXT_ << std::endl;
		return ;
	}

    
    udp_client.sendMSG (buf, len);
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
	uint16_t len = mavlink_msg_to_send_buffer((uint8_t*)buf, &mavlink_message);
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
	uint16_t len = mavlink_msg_to_send_buffer((uint8_t*)buf, &mavlink_message1);
	len += mavlink_msg_to_send_buffer((uint8_t*)&buf[len], &mavlink_message2);
    
    if (len >= 600) 
	{
		std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "ERROR LEN = " << std::to_string(len) << _NORMAL_CONSOLE_TEXT_ << std::endl;
		return ;
	}

    m_sendBMSG (target_party_id, buf, len, TYPE_AndruavMessage_MAVLINK, false, Json());
    
    return ;
}


void CFCBFacade::sendMavlinkData_3(const std::string&target_party_id, const mavlink_message_t& mavlink_message1, const mavlink_message_t& mavlink_message2, const mavlink_message_t& mavlink_message3)  const
{
    char buf[600];
    // Translate message to buffer
    memset (buf,0,sizeof(buf));

    uint16_t len = mavlink_msg_to_send_buffer((uint8_t*)buf, &mavlink_message1);
	len += mavlink_msg_to_send_buffer((uint8_t*)(&buf[len]), &mavlink_message2);
    len += mavlink_msg_to_send_buffer((uint8_t*)(&buf[len]), &mavlink_message3);
    
    if (len >= 600) 
	{
		std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "ERROR LEN = " << std::to_string(len) << _NORMAL_CONSOLE_TEXT_ << std::endl;
		return ;
	}

    m_sendBMSG (target_party_id, buf, len, TYPE_AndruavMessage_MAVLINK, false, Json());
    
    return ;
}


void CFCBFacade::sendMavlinkData_M(const std::string&target_party_id, const mavlink_message_t* mavlink_message, const uint16_t count)  const
{
    if (count==0) return ;

    char buf[600];
    // Translate message to buffer
    memset (buf,0,sizeof(buf));

    uint16_t len=0;

    for (int i=0;i<count;++i)
    {
        len += mavlink_msg_to_send_buffer((uint8_t*)(&buf[len]), &mavlink_message[i]);
    }

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


void CFCBFacade::internalCommand_takeImage() const
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
}

/**
 * @brief start/stop udp proxy. <b>This is a system command.</b>
 * 
 * @param start 
 * @param udp_ip1 
 * @param udp_port1
 * @param udp_ip2 
 * @param udp_port2 
 */
void CFCBFacade::requestUdpProxyTelemetry(const bool start, const std::string&udp_ip1, const int& udp_port1, const std::string&udp_ip2, const int& udp_port2)
{
    /*
        {
            en: enabled (true/false)
            socket1: {"address":"x.x.x.x", "port":nn}
            socket2: {"address":"x.x.x.x", "port":nn}
        }
    */

    Json address1 = 
            {
                {"address",udp_ip1},
                {"port",udp_port1}
            };

    Json address2 =
            {
                {"address",udp_ip2},
                {"port",udp_port2}
            };
    Json message =
            {
                {"en",start},
                {"socket1", address1},          // socket1 
                {"socket2",  address2}          // socket2
            };
            
    m_sendSYSMSG (message, TYPE_AndruavSystem_UdpProxy);
}



void CFCBFacade::sendUdpProxyStatus(const std::string&target_party_id, const bool& start, const std::string&udp_ip_other, const int& udp_port_other, const int& optimization_level)
{
    /*
        {
            en: enabled (true/false)
            a: ip address_"x.x.x.x",
            p: port in integer
            o: optimization_level in integer
        }
    */
    
    Json message =
            {
                {"a", udp_ip_other},
                {"p", udp_port_other}, 
                {"o", optimization_level},
                {"en", start}
            };
            
    m_sendJMSG (target_party_id, message, TYPE_AndruavMessage_UDPProxy_Info, false);
}


