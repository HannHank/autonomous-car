//necessary library's
#include <Arduino.h>
#include <SPI.h>
#include <math.h>   
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Servo.h>

#include <oled.h>

#include <HMC5883L.h>//
#include <ADXL345.h>

//#include <GPSNeo6.h>
//#include <Compass.h>
#include <GPSNeom8n.h>
#include <Data.h>


byte PWM_PIN = 3;
Servo Lenkung;

//////Compass stuff
// compass compass;
HMC5883L compass;
ADXL345 accelerometer;

float heading1;
float heading2;



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
 

  
  ////////////////////////////////////////////////////////////////
  setupGPSm8n();
  ///////////////////////////////////////////////////////////////

  if (!accelerometer.begin())
  {
    delay(500);
  }

  accelerometer.setRange(ADXL345_RANGE_2G);

  // Initialize Initialize HMC5883L
  while (!compass.begin())
  {
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(-177,-257); 

  start_up();

  Lenkung.attach(9);
  Lenkung.write(90);

///
   initializeData();

  //first waypoint
  destinationlat = way1.lat;
  destinationlon = way1.lon;

}
// Tilt compensation
float noTiltCompensate(Vector mag)
{
  float heading = atan2(mag.YAxis, mag.XAxis);
  return heading;
}
 
float tiltCompensate(Vector mag, Vector normAccel)
{
  // Pitch & Roll 

  float roll;
  float pitch;

  roll = asin(normAccel.YAxis);
  pitch = asin(-normAccel.XAxis);

  if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78)
  {
    return -1000;
  }

  // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
  float cosRoll = cos(roll);
  float sinRoll = sin(roll);  
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  // Tilt compensation
  float Xh = mag.XAxis * cosPitch + mag.ZAxis * sinPitch;
  float Yh = mag.XAxis * sinRoll * sinPitch + mag.YAxis * cosRoll - mag.ZAxis * sinRoll * cosPitch;

  float heading = atan2(Yh, Xh);

  return heading;
}

// Correct angle
float correctAngle(float heading)
{
  if (heading < 0) { heading += 2 * PI; }
  if (heading > 2 * PI) { heading -= 2 * PI; }

  return heading;
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
    int gps = fix.satellites;
    Serial.print("GPS = ");
    Serial.println(gps);
    display.clearDisplay();
    display.setTextColor(WHITE);ö
    display.setCursor(0,50);
    display.println("SATS =");
    display.setCursor(20,50);
    display.println(gps);
    display.display();
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
        if(geoDistance(lat,lon,way5.lat,way5.lon) < 1){
      destinationlat = way6.lat;
      destinationlon = way6.lon; 
      }
       destinationlat = way6.lat;
      destinationlon = way6.lon; 
    float dist = geoDistance(lat,lon,destinationlat,destinationlon);
    // Serial.print("dist =");
    // Serial.println(dist);
    

  
  // Serial.println(targetHeading);
  }
  
  Vector mag = compass.readNormalize();
  Vector acc = accelerometer.readScaled();

  // Calculate headings
  heading1 = noTiltCompensate(mag);
  heading2 = tiltCompensate(mag, acc);

  if (heading2 == -1000)
  {
    heading2 = heading1;
  }

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading1 += declinationAngle;
  heading2 += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  heading1 = correctAngle(heading1);
  heading2 = correctAngle(heading2);

  // Convert to degrees
  heading1 = heading1 * 180/M_PI; 
  heading2 = heading2 * 180/M_PI; 
  Serial.print("No = ");
  Serial.print(heading1);
  Serial.print("with = ");
  Serial.println(heading2);

  double targetHeading = getBearing(lat,lon,destinationlat,destinationlon);
    


  float turn = targetHeading - heading2;
  while (turn < -180) turn += 360;
  while (turn >  180) turn -= 360;

  int autoSteer = map(turn, 180, -180, 180, 0); //Hier habe ich Veränderungen vorgenommen
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

