# 1 "c:\\users\\johann~1\\appdata\\local\\temp\\tmpn_scqz"
#include <Arduino.h>
# 1 "C:/Users/Johann Hanke/Documents/PlatformIO/Projects/autonomous-car/src/KICar.ino"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Servo.h>

#include <oled.h>
#include <Path.h>
#include <HMC5883L.h>
#include <ADXL345.h>



#include <GPSNeom8n.h>
#include <ultrasonic_Sensors.h>
#include <Data.h>

byte PWM_PIN = 3;
Servo Lenkung;



HMC5883L compass;
ADXL345 accelerometer;
#include <offset.h>

float heading1;
float heading2;
int point = 0;

int gpsSat = 0;
long dist = 0;
float geoDistance(double lat, double lon, double lat2, double lon2);
float noTiltCompensate(Vector mag);
float tiltCompensate(Vector mag, Vector normAccel);
float correctAngle(float heading);
double getBearing(double lat1, double lng1, double lat2, double lng2);
void setup();
void loop();
#line 37 "C:/Users/Johann Hanke/Documents/PlatformIO/Projects/autonomous-car/src/KICar.ino"
float geoDistance(double lat, double lon, double lat2, double lon2)
{
  const float R = 6371000;
  float p1 = lat * DEGTORAD;
  float p2 = lat2 * DEGTORAD;
  float dp = (lat2 - lat) * DEGTORAD;
  float dl = (lon2 - lon) * DEGTORAD;

  float x = sin(dp / 2) * sin(dp / 2) + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
  float y = 2 * atan2(sqrt(x), sqrt(1 - x));

  return R * y;
}

float noTiltCompensate(Vector mag)
{
  float heading = atan2(mag.YAxis, mag.XAxis);
  return heading;
}

float tiltCompensate(Vector mag, Vector normAccel)
{


  float roll;
  float pitch;

  roll = asin(normAccel.YAxis);
  pitch = asin(-normAccel.XAxis);

  if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78)
  {
    return -1000;
  }


  float cosRoll = cos(roll);
  float sinRoll = sin(roll);
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);


  float Xh = mag.XAxis * cosPitch + mag.ZAxis * sinPitch;
  float Yh = mag.XAxis * sinRoll * sinPitch + mag.YAxis * cosRoll - mag.ZAxis * sinRoll * cosPitch;

  float heading = atan2(Yh, Xh);

  return heading;
}


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

  pinMode(4, INPUT_PULLUP);


  setupGPSm8n();


  if (!accelerometer.begin())
  {
    delay(500);
  }

  accelerometer.setRange(ADXL345_RANGE_2G);


  while (!compass.begin())
  {
    delay(500);

  }


  compass.setRange(HMC5883L_RANGE_1_3GA);


  compass.setMeasurementMode(HMC5883L_CONTINOUS);


  compass.setDataRate(HMC5883L_DATARATE_30HZ);


  compass.setSamples(HMC5883L_SAMPLES_8);
# 171 "C:/Users/Johann Hanke/Documents/PlatformIO/Projects/autonomous-car/src/KICar.ino"
  calculate_offsets();
  compass.setOffset(offX, offY);

  start_up();

  Lenkung.attach(9);
  Lenkung.write(90);


  initializeData();







}

void loop()
{


  display.clearDisplay();

  t = millis();
  if(Serial.available()){
    String Route = Serial.readStringUntil('\n');
    Serial.print("Daten bekommen = ");
    Serial.println(Route);
    if(Route == "clear_Route"){
        deletList();
        initializePath();
        point = 0;
    }
    else{
        DynamicJsonBuffer jsonBuffer;
        JsonObject& root = jsonBuffer.parseObject(Route);
        double dataLat = root[String("location")][String("lat")];
        double dataLon = root[String("location")][String("lng")];
        insert(dataLat,dataLon);
        initializePath();
        displayPath();
    }
  }
   if(ptr != NULL){
  if (gps.available(gpsPort))
  {
    gps_fix fix = gps.read();
    lat = fix.latitude();
    lon = fix.longitude();
    dist = geoDistance(lat, lon, ptr->lat, ptr->lon);
    print(fix.satellites, fix.valid.satellites, 3);
    print(fix.latitude(), fix.valid.location, 10, 6);
    print(fix.longitude(), fix.valid.location, 11, 6);
    print(dist, true, 4);
    print(point,true, 3);
# 236 "C:/Users/Johann Hanke/Documents/PlatformIO/Projects/autonomous-car/src/KICar.ino"
    DEBUG_PORT.println();
# 245 "C:/Users/Johann Hanke/Documents/PlatformIO/Projects/autonomous-car/src/KICar.ino"
    if (dist <= 1)
    {
      ptr = ptr->next;
      destinationlat = ptr->lat;
      destinationlon = ptr->lon;
      point += 1;

    }
# 291 "C:/Users/Johann Hanke/Documents/PlatformIO/Projects/autonomous-car/src/KICar.ino"
  }

  Vector mag = compass.readNormalize();
  Vector acc = accelerometer.readScaled();


  heading1 = noTiltCompensate(mag);
  heading2 = tiltCompensate(mag, acc);

  if (heading2 == -1000)
  {
    heading2 = heading1;
  }






  float declinationAngle = (3.0 + (59.0 / 60.0)) / (180 / M_PI);
  heading1 += declinationAngle;
  heading2 += declinationAngle;


  heading1 = correctAngle(heading1);
  heading2 = correctAngle(heading2);


  heading1 = heading1 * 180 / M_PI;
  heading2 = heading2 * 180 / M_PI;



  double targetHeading = getBearing(lat, lon, ptr->lat, ptr->lon);





  float turn = targetHeading - heading1;
  while (turn < -180)
    turn += 360;
  while (turn > 180)
    turn -= 360;
# 349 "C:/Users/Johann Hanke/Documents/PlatformIO/Projects/autonomous-car/src/KICar.ino"
  int autoSteer = map(turn, 180, -180, 180, 0);
  autoSteer = constrain(autoSteer, 50, 130);

  float angleError = abs(turn);






  if (autoSteer > 92)
  {




  }
  else if (autoSteer < 88)
  {




  }
  else
  {




  }
  Lenkung.write(autoSteer);
# 393 "C:/Users/Johann Hanke/Documents/PlatformIO/Projects/autonomous-car/src/KICar.ino"
   }
   else{
      Lenkung.write(90);
       if (gps.available(gpsPort))
  {
    gps_fix fix = gps.read();
    lat = fix.latitude();
    lon = fix.longitude();
    dist = 0;
    print(fix.satellites, fix.valid.satellites, 3);
    print(fix.latitude(), fix.valid.location, 10, 6);
    print(fix.longitude(), fix.valid.location, 11, 6);
    print(dist, true, 3);
    print(point,true, 3);
    DEBUG_PORT.println();
   }
}
}