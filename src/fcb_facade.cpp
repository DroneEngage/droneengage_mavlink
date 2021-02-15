
#include "./helpers/json.hpp"
using Json = nlohmann::json;

#include "fcb_facade.hpp"



void uavos::fcb::CFCBFacade::sendID(const char *target_party_id, const ANDRUAV_VEHICLE_INFO& andruav_vehicle_info)
{
    /*
        VT: vehicle type
        FM: flying mode
        GM: gps mode
        FI: use FCB
        AR: is armed
        FL: is flying
        SD: shutdown
        TP: telemetry protocol
         z: flying last start time
         a: flying total duration.
         b: is in cv tracking mode.
         C: manual TX blocked mode subAction
         B: is GCSBlocked

    */
    Json message = R"(
        {
            "VT": true,     
            "FM": ,
            "GM": ,
            "FI": ,
            "AR": ,
            "FL": ,
            "SD": ,
            "TP": ,
            "z": ,
            "a": ,
            "b": ,
            "C": ,
            "B":    
        }
        )"_json;

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message);
    }
    
    return ;
}


void uavos::fcb::CFCBFacade::requestID(const char *target_party_id)
{

}

void uavos::fcb::CFCBFacade::sendErrorMessage(const char *target_party_id, const std::uint8_t severity, const std::string& status)
{
    Json message;

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message);
    }
 
    return ;
}

void uavos::fcb::CFCBFacade::sendTelemetryPanic(const char *target_party_id)
{

}

void uavos::fcb::CFCBFacade::sendGPSInfo(const char *target_party_id)
{
    Json message;

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message);
    }
 
    return ;
}

void uavos::fcb::CFCBFacade::sendNavInfo(const char *target_party_id)
{
    Json message;

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message);
    }
 
    return ;
}

void uavos::fcb::CFCBFacade::sendIMUInfo(const char *target_party_id)
{
    Json message;

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message);
    }
 
    return ;
}

void uavos::fcb::CFCBFacade::sendPowerInfo(const char *target_party_id)
{
    Json message;

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message);
    }
 
    return ;
}

void uavos::fcb::CFCBFacade::sendHomeLocation(const char *target_party_id)
{
    Json message;

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message);
    }
 
    return ;
}

void uavos::fcb::CFCBFacade::sendWayPoints(const char *target_party_id)
{
    Json message;

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message);
    }
 
    return ;
}

void uavos::fcb::CFCBFacade::sendTelemetryData(const char *target_party_id)
{
    Json message;

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message);
    }
 
    return ;
}

void uavos::fcb::CFCBFacade::sendServoReadings(const char *target_party_id)
{
    Json message;

    if (m_sendJMSG != NULL)
    {
        m_sendJMSG (target_party_id, message);
    }

    return ;
}
