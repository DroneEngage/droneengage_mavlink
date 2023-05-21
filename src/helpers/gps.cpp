

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
 * @param latitude_new  format 1e-7
 * @param longitude_new format 1e-7
 * @param latitude_old  format 1e-7
 * @param longitude_old format 1e-7
 * @return double distance in meters
 */
double calcGPSDistance(const double& latitude_new, const double& longitude_new, const double& latitude_old, const double& longitude_old)
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

// Function to get the point at a given bearing and distance from a given point.
//
// Args:
//   lat1: The latitude of the first point.
//   lon1: The longitude of the first point.
//   bearing: The bearing in degrees.
//   distance: The distance in kilometers.
//
// Returns:
//   A tuple of (latitude, longitude) of the point at the given bearing and distance.

POINT_2D get_point_at_bearing(const double&  lat1, const double&  lon1, const double&  bearing_deg, const double&  distance_m) {

    // Convert the latitude and longitude to radians.
    double lat1_radians = lat1 * GRADOS_RADIANES;
    double lon1_radians = lon1 * GRADOS_RADIANES;

    // Calculate the bearing in radians.
    double bearing_radians = bearing_deg * GRADOS_RADIANES;

    // Calculate the latitude and longitude of the new point.
    const double lat2 = asin(
        sin(lat1_radians) * cos(distance_m / RADIO_TERRESTRE) +
        cos(lat1_radians) * sin(distance_m / RADIO_TERRESTRE) * cos(bearing_radians));
    const double lon2 = lon1_radians + atan2(
        sin(bearing_radians) * sin(distance_m / RADIO_TERRESTRE) * cos(lat1_radians),
        cos(distance_m / RADIO_TERRESTRE) - sin(lat1_radians) * sin(lat2));

    // Convert the latitude and longitude to degrees.
    double lat2_degrees = lat2 * RADIANES_GRADOS;
    double lon2_degrees = lon2 * RADIANES_GRADOS;

    POINT_2D  closest;
    closest.latitude  = lat2_degrees;
    closest.longitude = lon2_degrees;
    
    return closest;
}


/**
 * @brief Get the Bearing object
 * IMPORTANT: if you are using vx & vy of GPS then use (vx, vy) 
 * @param velocityY 
 * @param velocityX 
 * @return double 
 */
double getBearingOfVector(double velocityX, double velocityY) {

  // Normalize the velocity vector.
  double velocityMagnitude = sqrt(velocityX * velocityX + velocityY * velocityY);
  double velocityUnitX = velocityX / velocityMagnitude;
  double velocityUnitY = velocityY / velocityMagnitude;

  // Get the angle of the normalized velocity vector.
  double angle = atan2(velocityUnitY, velocityUnitX);

  // Convert the angle to degrees.
  return fmod((angle * 180.0 / M_PI + 360),360);
}

double calculateBearing(const double& lat1, const double& lon1, const double& lat2, const double& lon2)
{
    const double longitude1 = lon1;
    const double longitude2 = lon2;
    const double latitude1 = lat1 * GRADOS_RADIANES;
    const double latitude2 = lat2 * GRADOS_RADIANES;
    const double longDiff= (longitude2-longitude1) * GRADOS_RADIANES;
    const double y= sin(longDiff) * cos(latitude2);
    const double x= cos(latitude1) * sin(latitude2) - sin(latitude1) * cos(latitude2) * cos(longDiff);
    
    return fmod(((RADIANES_GRADOS *(atan2(y, x)))+360),360);
}



double inPolygon(const double&  lat1, const double&  lon1, const std::vector<POINT_3D> points)
{
    const int nvert= points.size();
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

POINT_2D findIntersectionPoint(const double& ptx, const double& pty, const double& p1x, const double& p1y, const double& p2x, const double& p2y)
{
    POINT_2D  closest;
    const double dx = p2x - p1x;
    const double dy = p2y - p1y;
    if ((dx == 0) && (dy == 0))
    {
        // It's a point not a line segment.
        closest.latitude  = p1x;
        closest.longitude = p1y;
        
        return closest;
    }

    // Calculate the t that minimizes the distance.
    const double t = ((ptx - p1x) * dx + (pty - p1y) * dy) /
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

double findDistanceToSegment(const double& ptx, const double& pty, const double& p1x, const double& p1y, const double& p2x, const double& p2y)
{
    POINT_2D  closest = findIntersectionPoint(ptx, pty, p1x,p1y,p2x,p2y);
    return calcGPSDistance(ptx,pty,closest.latitude,closest.longitude);
}