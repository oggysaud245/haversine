#include <math.h>
#include "haversine.h"

double toRadians(double degrees) {
  return degrees * M_PI / 180.0;
}

double toDegrees(double radians) {
  return radians * 180.0 / M_PI;
}

double calculateAzimuth(double lat1, double lon1, double lat2, double lon2) {
  double phi1 = toRadians(lat1);
  double phi2 = toRadians(lat2);
  double lambda1 = toRadians(lon1);
  double lambda2 = toRadians(lon2);

  double deltaLambda = lambda2 - lambda1;

  double y = sin(deltaLambda) * cos(phi2);
  double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltaLambda);

  double azimuthRad = atan2(y, x);
  if (azimuthRad < 0) {
    azimuthRad += 2 * M_PI; // Convert negative angles to positive
  }

  double azimuthDeg = toDegrees(azimuthRad);
  return azimuthDeg;
}
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  double phi1 = toRadians(lat1);
  double phi2 = toRadians(lat2);
  double deltaPhi = toRadians(lat2 - lat1);
  double deltaLambda = toRadians(lon2 - lon1);

  double a = sin(deltaPhi / 2) * sin(deltaPhi / 2) +
             cos(phi1) * cos(phi2) * sin(deltaLambda / 2) * sin(deltaLambda / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  double distance = EarthRadius * c;
  return distance;
}
