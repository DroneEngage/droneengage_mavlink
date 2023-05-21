#ifndef GPS_H_
#define GPS_H_

#include <vector>

typedef struct {
        
        double latitude;
        double longitude;
        
    } POINT_2D;



typedef struct :POINT_2D{
        
        double altitude;

    } POINT_3D;

POINT_2D get_point_at_bearing(const double&  lat1, const double&  lon1, const double&  bearing_deg, const double&  distance_m);

double calcGPSDistance(const double& latitude_new, const double& longitude_new, const double& latitude_old, const double& longitude_old);
double inPolygon(const double&  lat1, const double&  lon1, const std::vector<POINT_3D> points);
POINT_2D findIntersectionPoint(const double& ptx, const double& pty, const double& p1x, const double& p1y, const double& p2x, const double& p2y);
double findDistanceToSegment(const double& ptx, const double& pty, const double& p1x, const double& p1y, const double& p2x, const double& p2y);

double calculateBearing(const double& lat1, const double& lon1, const double& lat2, const double& lon2);
double getBearingOfVector(double velocityX, double velocityY);

#endif
