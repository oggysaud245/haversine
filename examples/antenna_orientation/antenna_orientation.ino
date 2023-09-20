// This example show to orient the position of antenna.
// The antenna orient its position based on the gps position of transmitter.
// To test the code a radiosonde device is used any gps incorporated device with communication link capabilities will work.
// The data of radiosonde device is received on third party software on pc and only GPS NEMA sentences are sent to arduino via serial communication.
// At this point TinyGPS++ library plays a crucial role to decode the NMEA sentences to decode gps cordinates.
// Since ground station will be stationary the gps cordinate of ground station is constant.
// Now the available two different gps (one is stationary othe is mobile i.e. radiosonde device) are calculated to find the azimuth anlge between them.
// From received data of radiosonde there is another parameter i.e. Altitude of radiosonde 
// Using this altitude and pythagoras theorem, elevation angle is calculated.
// Antenna orientaion mechanism may differ one to one based on the design, and calculated angle should be mapped accordingly to drive steper motor to get 
// accurate result.
// https://github.com/oggysaud245/antenna_buddy this python code will help in mapping angle and step size of motor.
// Author: Ramesh Saud

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "haversine.h"
// for stepper motor, changes should be made if necessary
#define y_dirPin 6
#define x_dirPin 5
#define y_stepPin 3
#define x_stepPin 2
#define stepsPerRevolution 200  // steps per one revolution of a stepper motor

void x_forward(int steps = 1);
void y_forward(int steps = 5);
void x_reverse(int steps = 1);
void y_reverse(int steps = 5);
int inc_azimuth = 0;
int inc_elevation = 0;
int azimuth_steps = 0;
int elevation_steps = 0;

static const int RXPin = 9, TXPin = 10;   
double baseStationAlt = 0; /// altitude of ground station
double lat1 = 27.0;   // Latitude of first point
double lon1 = 85.0;   // Longititude of first point

// The TinyGPS++ object
TinyGPSPlus gps;

// since default serial is used for pc communication, secondary serial communication is used for debugging 
SoftwareSerial ss(RXPin, TXPin);
bool config_mode = false;

void setup()
{
  Serial.begin(9600);
  ss.begin(9600);

  pinMode(x_stepPin, OUTPUT);
  pinMode(x_dirPin, OUTPUT);
  pinMode(y_stepPin, OUTPUT);
  pinMode(y_dirPin, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(8, LOW);
  while (millis() < 5000) {
    String c = Serial.readStringUntil('\n');
    delay(2000);
    if (c == "x") {
      config_mode = true;
      digitalWrite(13, HIGH);
      delay(1000);
      digitalWrite(13, LOW);
      delay(1000);
      digitalWrite(13, HIGH);
      delay(1000);
      digitalWrite(13, LOW);
    }
  }
  while (config_mode) {
    motor();    // to drive stepper motor using antenna buddy software given link above.
  }

}
void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.

  while (Serial.available() > 0)
    if (gps.encode(Serial.read()))
      displayInfo();
    else {
      motor();
    }

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    ss.println(F("No GPS detected: check wiring."));
    //    while (true);
  }

}

void displayInfo()
{
  if (gps.location.isValid())
  {
    ss.print(gps.location.lat(), 6);
    ss.print(F(","));
    ss.print(gps.location.lng(), 6);
    ss.print(F(","));
    ss.print(gps.altitude.meters());
    ss.println();
    calculateAngle(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
  }
  else
  {
    ss.print(F("INVALID"));
  }

}
void calculateAngle(double lat2, double lon2, double alt) {
  static int old_azimuth = 0;
  static int old_elevation = 0;

  alt = alt - baseStationAlt;
  int azimuth =  round(calculateAzimuth(lat1, lon1, lat2, lon2));
  int distance = round(calculateDistance(lat1, lon1, lat2, lon2));
  //  Serial.println(alt);
  //  Serial.println(distance);
  ss.print("Azimuth angle: ");
  ss.println(azimuth);
  float elevationRadian = atan(alt / distance);  // Calculate the angle in radians using atan
  int elevation = round(elevationRadian * 180.0 / PI);          // Convert angle from radians to degrees
  //  Serial.println(elevation);
  elevation = constrain(elevation, 0, 80);
  ss.print("Elevation angle: ");
  ss.println(elevation);
  inc_azimuth = azimuth - old_azimuth;
  ss.print("increased azimuth: ");
  ss.println(inc_azimuth);
  inc_elevation = elevation - old_elevation;
  ss.print("increased elevation: ");
  ss.println(inc_elevation);
  azimuth_steps = round(inc_azimuth * (200/90)); /// degree to steps.05556
  elevation_steps = round(inc_elevation * 5 *(450/90)); /// degree to steps
  ss.print("azimuth steps: ");
  ss.println(azimuth_steps);
  ss.print("elevation steps: ");
  ss.println(elevation_steps);
  if (azimuth_steps > 0) {
    x_forward(abs(azimuth_steps));
    old_azimuth = azimuth;
  }
  else {
    x_reverse(abs(azimuth_steps));
    old_azimuth = azimuth;
  }
  if (elevation_steps > 0) {
    y_forward(abs(elevation_steps));
    old_elevation = elevation;
  }
  else {
    y_reverse(abs(elevation_steps));
    old_elevation = elevation;
  }
ss.println("\n--------------------------------------");
}

void motor() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd == "#1") {
      x_forward();
    }
    else if (cmd == "#2") {
      x_reverse();
    }
    else if (cmd == "#3") {
      y_forward();
    }
    else if (cmd == "#4") {
      y_reverse();
    }
  }

}

void x_forward(int steps = 1) {
  digitalWrite(x_dirPin, HIGH);

  // Spin the stepper motor 1 revolution slowly:
  for (int i = 0; i < steps; i++) {
    // These four lines result in 1 step:
    digitalWrite(x_stepPin, HIGH);
    delayMicroseconds(10000);
    digitalWrite(x_stepPin, LOW);
    delayMicroseconds(10000);
  }
}
void x_reverse(int steps = 1) {
  digitalWrite(x_dirPin, LOW);
  // Spin the stepper motor 1 revolution quickly:
  for (int i = 0; i < steps; i++) {
    // These four lines result in 1 step:
    digitalWrite(x_stepPin, HIGH);
    delayMicroseconds(10000);
    digitalWrite(x_stepPin, LOW);
    delayMicroseconds(10000);
  }
}
void y_forward(int steps = 5) {
  digitalWrite(y_dirPin, HIGH);

  // Spin the stepper motor 1 revolution slowly:
  for (int i = 0; i < steps; i++) {
    // These four lines result in 1 step:
    digitalWrite(y_stepPin, HIGH);
    delayMicroseconds(10000);
    digitalWrite(y_stepPin, LOW);
    delayMicroseconds(10000);
  }
}
void y_reverse(int steps = 5) {
  digitalWrite(y_dirPin, LOW);
  // Spin the stepper motor 1 revolution quickly:
  for (int i = 0; i < steps; i++) {
    // These four lines result in 1 step:
    digitalWrite(y_stepPin, HIGH);
    delayMicroseconds(10000);
    digitalWrite(y_stepPin, LOW);
    delayMicroseconds(10000);
  }
}
