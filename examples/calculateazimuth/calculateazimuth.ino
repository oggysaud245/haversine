/*This examples show the calculation of azimuth angle between two gps cordinates.
 * Author: Ramesh Saud aka oggy
 */
#include"haversine.h"

double lat1 = 27.0;   // Latitude of first point
double lon1 = 85.0;   // Longititude of first point
double lat2 = 28.0    // Latitude of second point
double lon2 = 85.0    // Longititude of second point
double azimuthangle = 0;

void setup(){
  Serial.begin(9600);
  azimuthangle = calculateAngle(lat1, lon1, lat2, lon2);
  Serial.print("Azimuth angle between given two cordinates is ");
  Serial.print(azimuthangle);
  Serial.println("deg");
}

void loop(){
  ;
}
