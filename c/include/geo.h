/* Great-circle math on a spherical Earth model. */
#ifndef NAVTRACK_GEO_H
#define NAVTRACK_GEO_H

#ifdef __cplusplus
extern "C" {
#endif

#define GEO_EARTH_RADIUS_M 6371008.8

double geo_distance_m(double lat1, double lon1, double lat2, double lon2);
double geo_bearing_deg(double lat1, double lon1, double lat2, double lon2);
void geo_destination(double lat, double lon, double bearing_deg, double distance_m,
                     double *out_lat, double *out_lon);

#ifdef __cplusplus
}
#endif

#endif
