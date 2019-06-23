//necessary library's
#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Servo.h>

#include <oled.h>

#include <HMC5883L.h> //
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
#include <offset.h>

float heading1;
float heading2;
int point = 0;
//gps
int gpsSat = 0;
float dist = 0.00;
//Funktion zum berechnen der Distanz
float geoDistance(double lat, double lon, double lat2, double lon2)
{
  const float R = 6371000; // km
  float p1 = lat * DEGTORAD;
  float p2 = lat2 * DEGTORAD;
  float dp = (lat2 - lat) * DEGTORAD;
  float dl = (lon2 - lon) * DEGTORAD;

  float x = sin(dp / 2) * sin(dp / 2) + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
  float y = 2 * atan2(sqrt(x), sqrt(1 - x));

  return R * y;
}
//Berechnen der Kompass Daten OHNE die Neigung zu korrigieren 
float noTiltCompensate(Vector mag)
{
  float heading = atan2(mag.YAxis, mag.XAxis);
  return heading;
}
//Berechnen der Kompass Daten MIT Neigung korrigiert 
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

//Der Winkel des Magnetsensors auslesen
float correctAngle(float heading)
{
  if (heading < 0)
  {
    heading += 2 * PI;
  }
  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  return heading;
}
//Den Winkel zwischen aktuelle Position und Ziel berechnen
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
void setup()
{
  //Pin defin
  pinMode(4, INPUT_PULLUP);
  //end
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
   // Serial.println("Compass begin");
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
  //
  //compass.setOffset(0,0)
  calculate_offsets();
  compass.setOffset(offX, offY);

  start_up();

  Lenkung.attach(9);
  Lenkung.write(90);

  ///
  initializeData();

  //first waypoint
  destinationlat = way1.lat;
  destinationlon = way1.lon;
  point = 1;
}


void loop()
{
  //clear display
  
  display.clearDisplay();
  //calibrating the Compass
  t = millis();

  if (gps.available(gpsPort))
  {
    gps_fix fix = gps.read();
    lat = fix.latitude();
    lon = fix.longitude();
    print(             fix.satellites       , fix.valid.satellites, 3             );
    print(             fix.latitude ()      , fix.valid.location  , 10, 6         );
    print(             fix.longitude()      , fix.valid.location  , 11, 6         );
    int pin = digitalRead(4);
    if(pin == LOW){
      way1.lat = lat;
      way1.lon = lon;
      Serial.println("Set homepoint");
     }
    // print(dist  , 123,3);
    DEBUG_PORT.println();
    gpsSat = fix.satellites;
    dist = geoDistance(lat, lon,51.001256 ,13.682089); 
    Serial.print(" Dist ");
    Serial.println(dist);
    //Serial.print("GPS = ");
    // Serial.println(gpsSat);

    // if (geoDistance(lat, lon, way1.lat, way1.lon) < 1)
    // {
    //   destinationlat = way2.lat;
    //   destinationlon = way2.lon;
    //   point = 1;
    // }
    // if (geoDistance(lat, lon, way2.lat, way2.lon) < 1)
    // {
    //   destinationlat = way3.lat;
    //   destinationlon = way3.lon;
    //   point = 2;
    // }
    // if (geoDistance(lat, lon, way3.lat, way3.lon) < 1)
    // {
    //   destinationlat = way4.lat;
    //   destinationlon = way4.lon;
    //   point = 3;
    // }
    // if (geoDistance(lat, lon, way4.lat, way4.lon) < 1)
    // {
    //   destinationlat = way5.lat;
    //   destinationlon = way5.lon;
    //   point = 4;
    // }
    // if (geoDistance(lat, lon, way5.lat, way5.lon) < 1)
    // {
    //   destinationlat = way6.lat;
    //   destinationlon = way6.lon;
    //   point = 5;
    // }
    // int val = digitalRead(4);
    // if(val == 1){
    //      destinationlat =lat;
    //      destinationlon = lon;
    // }

    //dist = geoDistance(lat, lon, destinationlat, destinationlon);
  
    


    // Serial.print("dist =");
    // Serial.println(dist);

    // Serial.println(targetHeading);
  }

  Vector mag = compass.readNormalize();
  Vector acc = accelerometer.readScaled();

//   // Calculate headings
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
  float declinationAngle = (3.0 + (59.0 / 60.0)) / (180 / M_PI);
  heading1 += declinationAngle;
  heading2 += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  heading1 = correctAngle(heading1);
  heading2 = correctAngle(heading2);

  // Convert to degrees
  heading1 = heading1 * 180 / M_PI;
  heading2 = heading2 * 180 / M_PI;
 
 
  
    // Serial.println(gpsSat);
    // Serial.println(gpsSat);
  double targetHeading = getBearing(lat, lon,51.001256,  13.68208);
  // Serial.print("Heading =");
  // Serial.println(heading1);
  // Serial.print("targetHeading =");
  // Serial.println(targetHeading);


  float turn = targetHeading - heading1;
  while (turn < -180) turn += 360;
  while (turn > 180)  turn -= 360;
  if(turn < 0){
    // Serial.print("kleiner Null");
    // Serial.print(turn);
    turn = turn *-1;
    // Serial.print("jetzt =");
    // Serial.println(turn);
  }
  else{
  //  Serial.print("größer Null");
   // Serial.print(turn);
    turn = turn *-1;
    //Serial.print("jetzt =");
   // Serial.println(turn);
  }
  int autoSteer = map(turn, 180, -180, 180, 0); //Hier habe ich Veränderungen vorgenommen
  autoSteer = constrain(autoSteer, 50, 130);

  float angleError = abs(turn);
  // ''Serial.print("angelError =");
  // Serial.println(angleError);

  // Serial.print("AutoSteer");
  // Serial.println(autoSteer);''

  if (autoSteer > 92)
  {
    //Serial.println("Links");
    // display.setTextColor(WHITE);
    // display.setCursor(0, 20);
    // display.print("links");
  }
  else if (autoSteer < 88)
  {
    //Serial.println("rechts");
    // display.setTextColor(WHITE);
    // display.setCursor(0, 20);
    // display.print("rechts");
  }
  else
  {
    //Serial.println("Geradeaus");
    // display.setTextColor(WHITE);
    // display.setCursor(0, 20);
    // display.print("Geradeaus");
  }
  Lenkung.write(autoSteer);

  //print GPS Data
  // display.setTextColor(WHITE);
  // display.setCursor(0, 0);
  // display.print("SATS =");
  // display.print(gpsSat);
  // display.setCursor(0, 10);
  // display.print("dist =");
  // display.print(dist);
  // display.setCursor(40, 20);
  // display.print(point);
  // display.display();
}

