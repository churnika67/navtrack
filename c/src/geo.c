#include "geo.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double rad(double deg) { return deg * M_PI / 180.0; }
static double deg(double r) { return r * 180.0 / M_PI; }

double geo_distance_m(double lat1, double lon1, double lat2, double lon2) {
    double dlat = rad(lat2 - lat1);
    double dlon = rad(lon2 - lon1);
    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(rad(lat1)) * cos(rad(lat2)) * sin(dlon / 2) * sin(dlon / 2);
    if (a > 1.0) a = 1.0; /* guard against rounding past 1 */
    return 2.0 * GEO_EARTH_RADIUS_M * asin(sqrt(a));
}

double geo_bearing_deg(double lat1, double lon1, double lat2, double lon2) {
    double p1 = rad(lat1), p2 = rad(lat2), dl = rad(lon2 - lon1);
    double y = sin(dl) * cos(p2);
    double x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
    double b = fmod(deg(atan2(y, x)) + 360.0, 360.0);
    return b;
}

void geo_destination(double lat, double lon, double bearing_deg, double distance_m,
                     double *out_lat, double *out_lon) {
    double d = distance_m / GEO_EARTH_RADIUS_M;
    double b = rad(bearing_deg);
    double p1 = rad(lat), l1 = rad(lon);
    double p2 = asin(sin(p1) * cos(d) + cos(p1) * sin(d) * cos(b));
    double l2 = l1 + atan2(sin(b) * sin(d) * cos(p1), cos(d) - sin(p1) * sin(p2));
    *out_lat = deg(p2);
    *out_lon = fmod(deg(l2) + 540.0, 360.0) - 180.0;
}
