//necessary library's
#include <Arduino.h>
#include <math.h>   
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Servo.h>
// #include <MCUFRIEND_kbv.h>
#include <compass.h>
//#include <GPSNeo6.h>
#include <CompassDraw.h>
#include <GPSNeom8n.h>
#include <Data.h>


byte PWM_PIN = 3;
Servo Lenkung;
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
//////Compass stuff

#define Task_t 10          // Task Time in milli seconds






float geoDistance(double lat, double lon, double lat2, double lon2) {
  const float R = 6371000; // km
  float p1 = lat * DEGTORAD;
  float p2 = lat2 * DEGTORAD;
  float dp = (lat2-lat) * DEGTORAD;
  float dl = (lon2-lon) * DEGTORAD;

  float x = sin(dp/2) * sin(dp/2) + cos(p1) * cos(p2) * sin(dl/2) * sin(dl/2);
  float y = 2 * atan2(sqrt(x), sqrt(1-x));

  return R * y;
}

void setup()
{
 
 //CompassSetup
  Wire.begin();
  compass_x_offset = 122.17;
  compass_y_offset = 230.08;
  compass_z_offset = 389.85;
  compass_x_gainError = 1.12;
  compass_y_gainError = 1.13;
  compass_z_gainError = 1.03;
  
  
  
  compass_init(2);
  compass_debug = 1;
  compass_offset_calibration(3);
  //Initialize I2C communications
  
  // Put the HMC5883 IC into the correct operating mode
  // last_dx = centreX;
  // last_dy = centreY;
  // last_dx1 = centreX1;
  // last_dy1 = centreY1;
  
  ////////////////////////////////////////////////////////////////
  setupGPSm8n();
  ///////////////////////////////////////////////////////////////

  Lenkung.attach(9);
  Lenkung.write(90);

///
   initializeData();

  //first waypoint
  destinationlat = way1.lat;
  destinationlon = way1.lon;

}

void loop()
{

  //calibrating the Compass
   t = millis();
 
  float load;
 
  if (gps.available( gpsPort )) {
    gps_fix fix = gps.read();
    lat = fix.latitude ();    
    lon = fix.longitude();
    if(geoDistance(lat,lon,way1.lat,way1.lon) < 1){
     destinationlat = way2.lat;
      destinationlon = way2.lon; 
          }
    if(geoDistance(lat,lon,way2.lat,way2.lon) < 1){
      destinationlat = way3.lat;
      destinationlon = way3.lon; 
      }
       if(geoDistance(lat,lon,way3.lat,way3.lon) < 1){
      destinationlat = way4.lat;
      destinationlon = way4.lon; 
      }
       if(geoDistance(lat,lon,way4.lat,way4.lon) < 1){
      destinationlat = way5.lat;
      destinationlon = way5.lon; 
      }
    float dist = geoDistance(lat,lon,destinationlat,destinationlon);
    Serial.print("dist =");
    Serial.println(dist);
    

  
  // Serial.println(targetHeading);
  }
    compass_heading();
     double targetHeading = getBearing(lat,lon,destinationlat,destinationlon);
    


  float turn = targetHeading - bearing;
  while (turn < -180) turn += 360;
  while (turn >  180) turn -= 360;

  int autoSteer = map(turn, 180, -180, 180, 0);
  autoSteer = constrain(autoSteer, 50, 130);

  float angleError = abs(turn);
  // ''Serial.print("angelError =");
  // Serial.println(angleError);

  // Serial.print("AutoSteer");
  // Serial.println(autoSteer);''

  if(autoSteer > 92){
Serial.println("Links");
  }
  else if(autoSteer < 88){
    Serial.println("Rechts");
  }
  else{
    Serial.println("Geradeaus");
  }
  Lenkung.write(autoSteer);
 
}
//calculate the Bearing 
double getBearing(double lat1, double lng1, double lat2, double lng2)
{
  lat1 = radians(lat1);
  lng1 = radians(lng1);
  lat2 = radians(lat2);
  lng2 = radians(lng2);

  double dLng = lng2 - lng1;
  double dPhi = log(tan(lat2 / 2.0 + PI / 4.0) / tan(lat1 / 2.0 + PI / 4.0));

  if (abs(dLng) > PI)
  {
    if (dLng > 0.0)
      dLng = -(2.0 * PI - dLng);
    else
      dLng = (2.0 * PI + dLng);
  }

  return fmod((degrees(atan2(dLng, dPhi)) + 360.0), 360.0);
}

