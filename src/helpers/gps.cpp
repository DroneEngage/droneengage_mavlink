

#include <iostream>
#include <math.h>
#include <algorithm>

#include "gps.hpp"

#define PI 3.14159265358979323846
#define RADIO_TERRESTRE 6372797.56085
#define GRADOS_RADIANES PI / 180
#define RADIANES_GRADOS 180 / PI
using namespace std;


/**
 * @brief return distanbce in meters
 * 
 * @param latitude_new 
 * @param longitude_new 
 * @param latitude_old 
 * @param longitude_old 
 * @return double distance in meters
 */
double calcGPSDistance(double latitude_new, double longitude_new, double latitude_old, double longitude_old)
{
    double  lat_new = latitude_old * GRADOS_RADIANES;
    double  lat_old = latitude_new * GRADOS_RADIANES;
    double  lat_diff = (latitude_new-latitude_old) * GRADOS_RADIANES;
    double  lng_diff = (longitude_new-longitude_old) * GRADOS_RADIANES;

    double  a = sin(lat_diff/2) * sin(lat_diff/2) +
                cos(lat_new) * cos(lat_old) *
                sin(lng_diff/2) * sin(lng_diff/2);
    double  c = 2 * atan2(sqrt(a), sqrt(1-a));

    double  distance = RADIO_TERRESTRE * c;
    
    // std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "
    // << "latitude_new:" << std::to_string(latitude_new)
    // << "longitude_new:" << std::to_string(longitude_new)
    // << "latitude_old:" << std::to_string(latitude_old)
    // << "longitude_old:" << std::to_string(longitude_old)
    // << "distance:" << std::to_string(distance)
    // << std::endl;

    return distance;
}


double calculateBearing(double lon1, double lat1, double lon2, double lat2)
{
    double longitude1 = lon1;
    double longitude2 = lon2;
    double latitude1 = lat1 * GRADOS_RADIANES;
    double latitude2 = lat2 * GRADOS_RADIANES;
    double longDiff= (longitude2-longitude1) * GRADOS_RADIANES;
    double y= sin(longDiff) * cos(latitude2);
    double x= cos(latitude1) * sin(latitude2) - sin(latitude1) * cos(latitude2) * cos(longDiff);
    
    // std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "
    // << "lon1:" << std::to_string(lon1)
    // << "lat1:" << std::to_string(lat1)
    // << "lon2:" << std::to_string(lon2)
    // << "lat2:" << std::to_string(lat2)
    // << std::endl;

    return fmod(((RADIANES_GRADOS *(atan2(y, x)))+360),360);
}



double inPolygon(double lat1, double lon1, std::vector<POINT_3D> points)
{
    int nvert= points.size();
    int i, j, c = 0;
    for (i = 0, j = nvert-1; i < nvert; j = i++) {
        // std::cout << std::to_string(points[i].longitude) << " " << std::to_string(points[j].longitude) << " " << std::to_string(lon1) << "(points[i].longitude>lon1)" << std::to_string((points[i].longitude>lon1))
        //         << "(points[j].longitude>lon1)" << std::to_string((points[j].longitude>lon1)) << std::endl;
        
        if ((points[i].longitude>lon1) != (points[j].longitude > lon1)) 
        {
         if ((lat1 < (points[j].latitude-points[i].latitude) * (lon1-points[i].longitude) / (points[j].longitude-points[i].longitude) + points[i].latitude) )
         {
            c = !c;
         }
        }
        
    }
    return c;
}

POINT_2D findIntersectionPoint(double ptx, double pty, double p1x, double p1y, double p2x, double p2y)
{
    POINT_2D  closest;
    double dx = p2x - p1x;
    double dy = p2y - p1y;
    if ((dx == 0) && (dy == 0))
    {
        // It's a point not a line segment.
        closest.latitude  = p1x;
        closest.longitude = p1y;
        
        return closest;
    }

    // Calculate the t that minimizes the distance.
    double t = ((ptx - p1x) * dx + (pty - p1y) * dy) /
                (dx * dx + dy * dy);

    // See if this represents one of the segment's
    // end points or a point in the middle.
    if (t < 0)
    {
        closest.latitude  = p1x;
        closest.longitude = p1y;
    }
    else if (t > 1)
    {
        closest.latitude  = p2x;
        closest.longitude = p2y;
    }
    else
    {
        closest.latitude  = p1x + t * dx;
        closest.longitude = p1y + t * dy;
    }

    return closest;
}

double findDistanceToSegment(double ptx, double pty, double p1x, double p1y, double p2x, double p2y)
{
    POINT_2D  closest = findIntersectionPoint(ptx, pty, p1x,p1y,p2x,p2y);
    return calcGPSDistance(ptx,pty,closest.latitude,closest.longitude);
}