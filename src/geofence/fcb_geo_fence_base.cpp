#include <iostream>

#include "../helpers/colors.hpp"
#include "../helpers/gps.hpp"
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

   m_message = message;
}
            

Json CGeoFenceBase::getMessage()
{
    return m_message;
}


//**************************** CGeoFenceCylinder

CGeoFenceCylinder::CGeoFenceCylinder() 
{
    m_geofence_type = ENUM_GEOFENCE_TYPE::CylindersFence;
}

CGeoFenceCylinder::~CGeoFenceCylinder() 
{
    
}

void CGeoFenceCylinder::parse (const Json& message)
{
    CGeoFenceBase::parse(message);
    Json circle = message["0"];
    m_latitude = circle["a"].get<double>() ;  //E7 format to match mavlink gps data
    m_longitude = circle["g"].get<double>() ;  //E7 format to match mavlink gps data
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


Json CGeoFenceCylinder::getMessage() 
{
    return CGeoFenceBase::getMessage();
}

/**
 * @brief distance from edge
 * 
 * @param lat 
 * @param lng 
 * @param alt 
 * @return double 
 */
double CGeoFenceCylinder::isInside(double lat, double lng, double alt) const
{
    double distance = calcGPSDistance (m_latitude , m_longitude , lat , lng );
    
    #ifdef DEBUG
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: lat " << std::to_string(lat) << " lng " << std::to_string(lng) << " alt " << std::to_string(alt) << " distance: " << std::to_string(distance) << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif

    return distance - m_radius;
}



//**************************** CGeoFencePolygon
CGeoFencePolygon::CGeoFencePolygon() 
{
    m_geofence_type = ENUM_GEOFENCE_TYPE::PolygonFence;
}

CGeoFencePolygon::~CGeoFencePolygon() 
{
    
}

void CGeoFencePolygon::parse (const Json& message)
{
    CGeoFenceBase::parse(message);

    m_vertex.clear();

    const int vertex_count = message["c"].get<int>();

    for (int i=0; i<vertex_count; ++i)
    {
        Json vertex = message[std::to_string(i)];
        m_vertex.push_back(POINT_3D());
        m_vertex[i].latitude = vertex["a"].get<double>() ;
        m_vertex[i].longitude = vertex["g"].get<double>() ;
        double alt; 
        if (vertex.contains("l")==true)
        {
            alt = vertex["l"];
        }
        else
        {
            alt = 0;
        }
        m_vertex[0].altitude = alt;
    }
}
  
Json CGeoFencePolygon::getMessage() 
{
    return CGeoFenceBase::getMessage();
}

/**
 * @brief return 0 if inside or distance to center if outside.
 * 
 * @param lat 
 * @param lng 
 * @param alt 
 * @return double 
 */
double CGeoFencePolygon::isInside(double lat, double lng, double alt) const
{
    #ifdef DEBUG
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: lat " << std::to_string(lat) << " lng " << std::to_string(lng) << " alt " << std::to_string(alt) << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    
    if (inPolygon (lat, lng, m_vertex) == 1)
    {
        return 0; //
    }
     
    
    return  calcGPSDistance (m_latitude , m_longitude , lat , lng );
}

//**************************** CGeoFenceLine

CGeoFenceLine::CGeoFenceLine() 
{
    m_geofence_type = ENUM_GEOFENCE_TYPE::LinearFence;
}

CGeoFenceLine::~CGeoFenceLine() 
{
    
}

void CGeoFenceLine::parse (const Json& message)
{
    CGeoFenceBase::parse(message);

    m_vertex.clear();

    const int vertex_count = message["c"].get<int>();

    for (int i=0; i<vertex_count; ++i)
    {
        Json vertex = message[std::to_string(i)];
        m_vertex.push_back(POINT_3D());
        m_vertex[i].latitude = vertex["a"].get<double>() ;
        m_vertex[i].longitude = vertex["g"].get<double>() ;
        double alt; 
        if (vertex.contains("l")==true)
        {
            alt = vertex["l"];
        }
        else
        {
            alt = 0;
        }
        m_vertex[0].altitude = alt;
    }

    m_width = message["r"].get<int>();
}
            
Json CGeoFenceLine::getMessage() 
{
    return CGeoFenceBase::getMessage();
}

double CGeoFenceLine::isInside(double lat, double lng, double alt) const
{
    #ifdef DEBUG
    std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: lat " << std::to_string(lat) << " lng " << std::to_string(lng) << " alt " << std::to_string(alt) << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    const std::size_t size = m_vertex.size();
    double total_distance = 9999999;
    for(int i = 0; i < size -1 ; i++)
    {
        const double distance = findDistanceToSegment (lat , lng, 
                            m_vertex[i].latitude, m_vertex[i].longitude,
                            m_vertex[i+1].latitude, m_vertex[i+1].longitude);
        if (distance < total_distance)
        {
            total_distance = distance;
        }
    }
    
    return (total_distance - m_width);
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