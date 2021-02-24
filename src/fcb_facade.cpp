#include <math.h>       /* floor */
#include <memory>
#include <thread>
#include <mutex>
#include "./helpers/json.hpp"
using Json = nlohmann::json;
#include "messages.hpp"
#include "fcb_main.hpp"
#include "fcb_facade.hpp"



void uavos::fcb::CFCBFacade::sendID(const std::string&target_party_id)
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

    */
    uavos::fcb::CFCBMain&  fcbMain = uavos::fcb::CFCBMain::getInstance();
    mavlinksdk::CVehicle *vehicle =  m_mavlink_sdk.getVehicle().get();
        
    const ANDRUAV_VEHICLE_INFO& andruav_vehicle_info = fcbMain.getAndruavVehicleInfo();

    Json message =
        {
            {"VT", andruav_vehicle_info.vehicle_type},     
            {"FM", andruav_vehicle_info.flying_mode},
            {"GM", andruav_vehicle_info.gps_mode},
            {"FI", andruav_vehicle_info.use_fcb},
            {"AR", vehicle->isArmed()},
            {"FL", vehicle->isFlying()},
            {"TP", TelemetryProtocol_DroneKit_Telemetry},
            {"SD", false},
            {"z", andruav_vehicle_info.flying_last_start_time},
            {"a", andruav_vehicle_info.flying_total_duration},
            {"b", andruav_vehicle_info.is_tracking_mode},
            {"C", andruav_vehicle_info.manual_TX_blocked_mode},
            {"B", andruav_vehicle_info.is_gcs_blocked}
        };
        

        

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message, TYPE_AndruavResala_ID, true);
    }
    
    return ;
}


void uavos::fcb::CFCBFacade::requestID(const std::string&target_party_id)
{
    Json message = 
        {
            {"C", TYPE_AndruavResala_ID}
        };
        

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message, TYPE_AndruavResala_RemoteExecute, true);
    }
    
    return ;
}

void uavos::fcb::CFCBFacade::sendErrorMessage (const std::string&target_party_id, const int& error_number, const int& info_type, const int& notification_type, const std::string& description)
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
        m_sendJMSG (target_party_id, message, TYPE_AndruavResala_Error, false);
    }
 
    return ;
}

void uavos::fcb::CFCBFacade::sendTelemetryPanic(const std::string& target_party_id)
{

}


void uavos::fcb::CFCBFacade::sendGPSInfo(const std::string&target_party_id)
{
    /*
        3D           : int 3D fix
        GS           : bool GPS internal from FCB or external "injected"
        SATC         : satellite count
        [g]          : ground altitude
        [p]          : location exists
            [la]     : latitude
            [ln]     : longitude
            [a]      : relative altitude
            [pa]     : predicted latitude
            [pn]     : predicted longitude
        [s]          : speed m/s
        [b]          : bearing
        [c]          : accuracy


    */


    uavos::fcb::CFCBMain&  fcbMain = uavos::fcb::CFCBMain::getInstance();
            
    mavlinksdk::CVehicle *vehicle =  m_mavlink_sdk.getVehicle().get();
    const mavlink_gps_raw_int_t& gps = vehicle->getMSGGPSRaw();
    const mavlink_global_position_int_t&  gpos = vehicle->getMsgGlobalPositionInt();
    const ANDRUAV_VEHICLE_INFO& andruav_vehicle_info = fcbMain.getAndruavVehicleInfo();

    
    Json message =
        {
            
            {"3D", gps.fix_type}, 
            {"GS", andruav_vehicle_info.gps_mode},
            {"SATC", gps.satellites_visible},
            {"g", gpos.alt},
            {"p", "FCB"}, // source og this reading as if mode is auto you can read from multiple gps
            {"la", gpos.lat / 10000000.0},
            {"ln", gpos.lon / 10000000.0},  
            {"a",  floor (gpos.relative_alt / 1000.0f)}, 
            {"c", gps.h_acc},
            {"b", gps.yaw},
            {"s", gps.vel / 100.0}
        };

        


    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message, TYPE_AndruavResala_GPS, false);
    }
 
    return ;
}

void uavos::fcb::CFCBFacade::sendNavInfo(const std::string&target_party_id)
{
    /*
        a : nav_roll
        b : nav_pitch
        y : nav_yaw
        d : target_bearing 
        e : wp_dist
        f : alt_error
    */

    mavlinksdk::CVehicle *vehicle =  m_mavlink_sdk.getVehicle().get();
    const mavlink_attitude_t& attitude = vehicle->getMsgAttitude();
    const mavlink_nav_controller_output_t& nav_controller = vehicle->getMsgNavController();
    Json message =
    {
        {"a", attitude.roll},
        {"b", attitude.pitch},
        {"y", attitude.yaw},
        {"d", nav_controller.target_bearing},
        {"e", nav_controller.wp_dist},
        {"f", nav_controller.alt_error},
    };

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message, TYPE_AndruavResala_NAV_INFO, false);
    }
 
    return ;
}

void uavos::fcb::CFCBFacade::sendIMUInfo(const std::string&target_party_id)
{
    Json message;

    if (m_sendJMSG != NULL)
    {
        //m_sendJMSG (target_party_id, message, true);
    }
 
    return ;
}

void uavos::fcb::CFCBFacade::sendPowerInfo(const std::string&target_party_id)
{

    mavlinksdk::CVehicle *vehicle =  m_mavlink_sdk.getVehicle().get();
    
    int voltage = 0.0f;

    for (int i=0; i<=10 ; ++i)
    {
        if ( vehicle->getMsgBatteryStatus().voltages[i] != 65535)
            voltage += vehicle->getMsgBatteryStatus().voltages[i];
    }

    Json message =
    {
        /*
            BL : Device battery level
             V : Device voltage
            BT : Device battery temprature
            [H]: Device battery health - string
           [PS]: Plug Status - string e.g. charging, unplugged
           [FV]: FCB_Battery voltage (mV x1000 )
           [FI]: FCB_Battry  current charge (mAmp x100). Battery current, -1: autopilot does not measure the current
           [FR]: FCB_Battery remaining Remaining battery energy. Values: [0-100], -1: autopilot does not estimate the remaining battery.
            
            Extra Parameters:
            [T]: Battery temprature in mC-deg x1000
            [C]: Battery Current Consumed in mAmp
            : Battery Type MAV_BATTERY_TYPE
        */

        
        {"BL", 0.0},
        {"V", 0.0},
        {"BT", 0.0},
        {"HBL", 0.0},
        {"FV", voltage},
        {"FI", vehicle->getMsgBatteryStatus().current_battery * 10 },
        {"FR", vehicle->getMsgBatteryStatus().battery_remaining},
        {"T", vehicle->getMsgBatteryStatus().temperature},
        {"C", vehicle->getMsgBatteryStatus().current_consumed}
    };

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message, TYPE_AndruavResala_POWER, true);
    }
 
    return ;
}

void uavos::fcb::CFCBFacade::sendHomeLocation(const std::string&target_party_id)
{
    
    mavlinksdk::CVehicle *vehicle =  m_mavlink_sdk.getVehicle().get();
    const mavlink_home_position_t& home = vehicle->getMsgHomePosition();
    
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

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message, Type_AndruavResala_HomeLocation, false);
    }
 
    return ;
}

std::mutex g_pages_mutex;

/**
* @brief Chunked Waypoint version
* In this version field "i" is mandatory
* "i" = WAYPOINT_CHUNK except the last one is WAYPOINT_LAST_CHUNK
* A chunk may contain zero or more waypoints.
* "n" represents number of waypoint in the chunk.
* waypoints are numbered zero-based in each chunk.
*/
void uavos::fcb::CFCBFacade::sendWayPoints(const std::string&target_party_id)
{

    std::lock_guard<std::mutex> guard(g_pages_mutex);
    
    uavos::fcb::CFCBMain&  fcbMain = uavos::fcb::CFCBMain::getInstance();
    const uavos::fcb::mission::ANDRUAV_UNIT_MISSION& andruav_missions = fcbMain.getAndruavMission(); 
    const std::size_t length = andruav_missions.mission_items.size();
    

    if (length ==0)
    {
        Json message =
        {
            {"n", 0}
        };

        if (m_sendJMSG != NULL)
        {
            m_sendJMSG (target_party_id, message, TYPE_AndruavResala_WayPoints, false);
        }
        
        return ;
    }

    #define MAX_WAYPOINT_CHUNK  2

    for (int i=0, j=0; i< length; i+=MAX_WAYPOINT_CHUNK)
    {
        Json message;
        int lastsentIndex = 0;
        for (int j =0; (j<MAX_WAYPOINT_CHUNK) && (j+i < length); ++j)
        {
            auto it = andruav_missions.mission_items.find(i+j);
            uavos::fcb::mission::CMissionItem *mi= it->second.get();
            
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

        if (m_sendJMSG != NULL)
        {
            m_sendJMSG (target_party_id, message, TYPE_AndruavResala_WayPoints, false);
        }
    }

    #undef MAX_WAYPOINT_CHUNK
 
    return ;
}

void uavos::fcb::CFCBFacade::sendTelemetryData(const std::string&target_party_id)
{
    Json message;

    if (m_sendJMSG != NULL)
    {
        //m_sendJMSG (target_party_id, message);
    }
 
    return ;
}

void uavos::fcb::CFCBFacade::sendServoReadings(const std::string&target_party_id)
{
    Json message;

    if (m_sendJMSG != NULL)
    {
        //m_sendJMSG (target_party_id, message);
    }

    return ;
}


void uavos::fcb::CFCBFacade::sendWayPointReached (const std::string&target_party_id, const int& mission_sequence)
{
    /*
        R: Report Type
        P: Parameter1
    */


    Json message =
    {
        {"R", Drone_Report_NAV_ItemReached},
        {"P", mission_sequence}
    };

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message, TYPE_AndruavMessage_DroneReport, false);
    }
 
    return ;
}
