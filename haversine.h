#include <math.h>

#define EarthRadius 6371000 // Earth's radius in meters

double toRadians(double degrees);
double toDegrees(double radians);
double calculateAzimuth(double lat1, double lon1, double lat2, double lon2);
double calculateDistance(double lat1, double lon1, double lat2, double lon2);
