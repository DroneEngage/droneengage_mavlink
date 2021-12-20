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

double calcGPSDistance(double latitude_new, double longitude_new, double latitude_old, double longitude_old);
double inPolygon(double lon1, double lat1, std::vector<POINT_3D> points);
POINT_2D findIntersectionPoint(double ptx, double pty, double p1x, double p1y, double p2x, double p2y);
double findDistanceToSegment(double ptx, double pty, double p1x, double p1y, double p2x, double p2y);
double calculateBearing(double lat1, double lon1, double lon2, double lat2);

#endif
