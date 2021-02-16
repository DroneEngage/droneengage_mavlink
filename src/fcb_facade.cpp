
#include "./helpers/json.hpp"
using Json = nlohmann::json;
#include "messages.hpp"
#include "fcb_facade.hpp"



void uavos::fcb::CFCBFacade::sendID(const std::string&target_party_id, const ANDRUAV_VEHICLE_INFO& andruav_vehicle_info)
{
    /*
        VT: vehicle type
        FM: flying mode
        GM: gps mode
        FI: use FCB
        AR: is armed                            // optional 
        FL: is flying                           // optional
        SD: shutdown
        TP: telemetry protocol
         z: flying last start time              // optional 
         a: flying total duration.              // optional 
         b: is in cv tracking mode.             // optional 
         C: manual TX blocked mode subAction    
         B: is GCSBlocked

    */
   
   
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
    Json message = {{
            "C", TYPE_AndruavResala_ID}};
        

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message, TYPE_AndruavResala_RemoteExecute, true);
    }
    
    return ;
}

void uavos::fcb::CFCBFacade::sendErrorMessage(const std::string&target_party_id, const std::uint8_t severity, const std::string& status)
{
    Json message;

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message, TYPE_AndruavResala_Error, true);
    }
 
    return ;
}

void uavos::fcb::CFCBFacade::sendTelemetryPanic(const std::string&target_party_id)
{

}

void uavos::fcb::CFCBFacade::sendGPSInfo(const std::string&target_party_id)
{
    Json message;

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message, TYPE_AndruavResala_GPS, true);
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
