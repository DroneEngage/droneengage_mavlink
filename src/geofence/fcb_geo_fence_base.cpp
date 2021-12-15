#include "../geofence/fcb_geo_fence_base.hpp"





using namespace uavos::fcb::geofence;


CGeoFenceBase::CGeoFenceBase() 
{

}

CGeoFenceBase::~CGeoFenceBase() 
{
    
}

void CGeoFenceBase::parse (const Json& message)
{
   m_fence_name = message["n"].get<std::string>();
   m_should_keep_outside = message["o"].get<int>()==1; 
   m_hard_fence_action = message["a"].get<int>();
}
            

//**************************** CGeoFenceCylinder

CGeoFenceCylinder::CGeoFenceCylinder() 
{

}

CGeoFenceCylinder::~CGeoFenceCylinder() 
{
    
}

void CGeoFenceCylinder::parse (const Json& message)
{
    CGeoFenceBase::parse(message);
    Json circle = message["0"];
    m_latitude = circle["a"].get<double>();
    m_longitude = circle["g"].get<double>();
    if (circle.contains("l")==true)
    {
        m_altitude = circle["l"].get<double>();
    }
    else
    {
        m_altitude = 0;
    }
    m_radius = message["r"].get<int>();
}
  
void CGeoFenceCylinder::evaluate(double lat, double lng, double alt)
{
    
}

//**************************** CGeoFencePolygon
CGeoFencePolygon::CGeoFencePolygon() 
{

}

CGeoFencePolygon::~CGeoFencePolygon() 
{
    
}

void CGeoFencePolygon::parse (const Json& message)
{
    CGeoFenceBase::parse(message);
}
  
void CGeoFencePolygon::evaluate(double lat, double lng, double alt)
{
    
}

//**************************** CGeoFenceLine

CGeoFenceLine::CGeoFenceLine() 
{

}

CGeoFenceLine::~CGeoFenceLine() 
{
    
}

void CGeoFenceLine::parse (const Json& message)
{
    CGeoFenceBase::parse(message);
}
            
void CGeoFenceLine::evaluate(double lat, double lng, double alt)
{
    
}


//******************************** FACTORY

std::unique_ptr<uavos::fcb::geofence::CGeoFenceBase> CGeoFenceFactory::getGeoFenceObject (const Json& message) const
{
    std::unique_ptr<uavos::fcb::geofence::CGeoFenceBase> cGeoFenceBase  (new uavos::fcb::geofence::CGeoFenceLine());

    int m_geofence_type = message["t"].get<int>();
    switch (m_geofence_type)
    {
        case ENUM_GEOFENCE_TYPE::LinearFence:
            cGeoFenceBase = std::unique_ptr<uavos::fcb::geofence::CGeoFenceBase> (new uavos::fcb::geofence::CGeoFenceLine());
        break;

        case ENUM_GEOFENCE_TYPE::PolygonFence:
            cGeoFenceBase = std::unique_ptr<uavos::fcb::geofence::CGeoFenceBase> (new uavos::fcb::geofence::CGeoFencePolygon());
        break;

        case ENUM_GEOFENCE_TYPE::CylindersFence:
            cGeoFenceBase = std::unique_ptr<uavos::fcb::geofence::CGeoFenceBase> (new uavos::fcb::geofence::CGeoFenceCylinder());
        break;

        default:
            cGeoFenceBase = std::unique_ptr<uavos::fcb::geofence::CGeoFenceBase> (new uavos::fcb::geofence::CGeoFenceBase());
        break;

    }

    cGeoFenceBase.get()->parse(message);
    
                
            
    return std::move(cGeoFenceBase);

}