
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
            
    const ANDRUAV_VEHICLE_INFO& andruav_vehicle_info = fcbMain.getAndruavVehicleInfo();

    Json message =
        {
            {"VT", andruav_vehicle_info.vehicle_type},     
            {"FM", andruav_vehicle_info.flying_mode},
            {"GM", andruav_vehicle_info.gps_mode},
            {"FI", andruav_vehicle_info.use_fcb},
            {"AR", andruav_vehicle_info.is_armed},
            {"FL", andruav_vehicle_info.is_flying},
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
            {"a",  gpos.relative_alt},
            {"c", gps.h_acc},
            {"b", gps.yaw}
        };

        


    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message, TYPE_AndruavResala_GPS, false);
    }
 
    return ;
}

void uavos::fcb::CFCBFacade::sendNavInfo(const std::string&target_party_id)
{
    Json message;

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message, TYPE_AndruavResala_NAV_INFO, true);
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
    Json message;

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message, TYPE_AndruavResala_POWER, true);
    }
 
    return ;
}

void uavos::fcb::CFCBFacade::sendHomeLocation(const std::string&target_party_id)
{
    Json message;

    if (m_sendJMSG != NULL)
    {
        //m_sendJMSG (target_party_id, message, true);
    }
 
    return ;
}

void uavos::fcb::CFCBFacade::sendWayPoints(const std::string&target_party_id)
{
    Json message;

    if (m_sendJMSG != NULL)
    {
        //m_sendJMSG (target_party_id, message);
    }
 
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
