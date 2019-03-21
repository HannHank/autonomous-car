//necessary library's
#include <Arduino.h>

#include <SoftwareSerial.h>

#include <Adafruit_GFX.h> 
// #include <MCUFRIEND_kbv.h>
#include <Compass.h>
 
#include <GPSNeo6.h.h>
#include <CompassDraw.h>
//end
MAG3110 mag = MAG3110();

#define TURN_LEFT 1
#define TURN_RIGHT 2
#define TURN_STRAIGHT 99
#define WAYPOINT_DIST_TOLERANE 5
#define HEADING_TOLERANCE 5
enum directions
{
  left = TURN_LEFT,
  right = TURN_RIGHT,
  straight = TURN_STRAIGHT
};
directions turnDirection = straight;
int headingError;
int sat = 0;
float kmh = 0;
long olddistance = 0;



int status = 0;

// Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
// If using the shield, all control and data lines are fixed, and
// a simpler declaration can optionally be used:

void setup()
{

  Serial.begin(9600);
  tft.reset(); //reset display
  uint16_t identifier = tft.readID(); // Found ILI9325 LCD driver
  tft.begin(identifier);
  tft.setRotation(1);
  tft.fillScreen(BLACK);
  //Initialize I2C communications
  Wire.begin(); // for the Compass
  mag.initialize(); // start Compass
  // Put the HMC5883 IC into the correct operating mode
  // last_dx = centreX;
  // last_dy = centreY;
  // last_dx1 = centreX1;
  // last_dy1 = centreY1;
  DrawCompassSetup();
  ////////////////////////////////////////////////////////////////
  ss.begin(GPSBaud); //gps start's
  ///////////////////////////////////////////////////////////////
}

void loop()
{

  //calibrating the Compass
   int x, y, z;
   while(!mag.isCalibrated()) //If we're not calibrated
  {
    if(!mag.isCalibrating()) //And we're not currently calibrating
    {
      Serial.println("Entering calibration mode");
      mag.enterCalMode(); //This sets the output data rate to the highest possible and puts the mag sensor in active mode
    }
    else
    {
      //Must call every loop while calibrating to collect calibration data
      //This will automatically exit calibration
      //You can terminate calibration early by calling mag.exitCalMode();
      mag.calibrate(); 
    }
      mag.readMag(&x, &y, &z);
      Serial.print("X: ");
      Serial.print(x);
      Serial.print(", Y: ");
      Serial.print(y);
      Serial.print(", Z: ");
      Serial.println(z);
      Serial.print("Heading: ");
      Serial.println(mag.readHeading());
      Serial.println("--------");
  }
    Serial.println("Calibrated!");
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(", Y: ");
  Serial.print(y);
  Serial.print(", Z: ");
  Serial.println(z);
  Serial.print("Heading: ");
  Serial.println(mag.readHeading());
  

  Serial.println("--------");
  delay(100);
  if(mag.readHeading() < 0.00){
    status = 1;
  }
//end 
//read heading 
mag.readMag(&x, &y, &z);
  GPS();
  
  int azimuth;
  int currentHeading;
  int targetHeading;
  double lat = 51.000832; // seting test position
  double lng = 13.682485;
  Draw_Compass_Rose(); //draw compass Rose at the tft
  float heading = atan2((double)x, (double)y);//caculating the heading 
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.0;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if (heading < 0){
    heading += 2 * PI;
   }

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI){
    heading -= 2 * PI;
  }
  //
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180 / M_PI;
  currentHeading = headingDegrees;

  dx = (diameter * cos((currentHeading - 90) * 3.14 / 180)) + centreX; // calculate X position
  dy = (diameter * sin((currentHeading - 90) * 3.14 / 180)) + centreY; // calculate Y position
  arrow(last_dx, last_dy, centreX, centreY, 20, 20, BLACK);            // Erase last arrow
  arrow(dx, dy, centreX, centreY, 20, 20, CYAN);                       // Draw arrow in new position
  last_dx = dx;
  last_dy = dy;
  delay(25);
  targetHeading = getBearing(gps.location.lat(), gps.location.lng(), lat, lng);
  Serial.print("currentHeading = ");
  Serial.println(currentHeading);
  Serial.print("targetHeading = ");
  Serial.println(targetHeading);
  // calculate where we need to turn to head to destination
  headingError = targetHeading - currentHeading;

  // adjust for compass wrap
  if (headingError < -180)
  {
    headingError += 360;
  }
  if (headingError > 180)
  {
    headingError -= 360;
  }
  // calculate which way to turn to intercept the targetHeading
  if (abs(headingError) <= HEADING_TOLERANCE)
  { // if within tolerance, don't turn
    turnDirection = straight;
    Serial.println("Straight");
    tft.setCursor(300, 50);
    tft.setTextColor(BLACK);
    tft.print("R");
    delay(50);
    tft.print("L");
    delay(50);
    tft.print("S");
    tft.setCursor(300, 50);
    tft.setTextColor(WHITE);
    tft.print("S");
    
  }
  else if (headingError > 0)
  {
    turnDirection = left;
    tft.setCursor(300, 50);
    tft.setTextColor(BLACK);
    tft.print("R");
    delay(50);
    tft.print("L");
    delay(50);
    tft.print("S");
    tft.setCursor(300, 50);
    tft.setTextColor(WHITE);
    tft.print("L");
    Serial.println("Nach Links");
  }
  else if (headingError < 0)
  {
    turnDirection = right;
    tft.setCursor(300, 50);
    tft.setTextColor(BLACK);
    tft.print("L");
    delay(50);
    tft.print("R");
    tft.setCursor(300, 50);
    tft.setTextColor(WHITE);
    tft.print("R");
    Serial.println("Nach Rechts");
  }
  else
  {
    turnDirection = straight;
  }

  // calcDesiredTurn()

  if (targetHeading >= currentHeading)
  {
    Serial.print("currentHeading = ");
    Serial.println(currentHeading);
    Serial.print("targetHeading = ");
    Serial.println(targetHeading);
    Serial.println("Nach Rechts");
  }
  if (targetHeading <= currentHeading)
  {
    Serial.print("currentHeading = ");
    Serial.println(currentHeading);
    Serial.print("targetHeading = ");
    Serial.println(targetHeading);
    Serial.println("Nach Links");
  }

  dx1 = (diameter1 * cos((targetHeading - 90) * 3.14 / 180)) + centreX1; // calculate X position
  dy1 = (diameter1 * sin((targetHeading - 90) * 3.14 / 180)) + centreY1; // calculate Y position
  arrow(last_dx1, last_dy1, centreX1, centreY1, 20, 20, BLACK);          // Erase last arrow
  arrow(dx1, dy1, centreX1, centreY1, 20, 20, MAGENTA);                  // Draw arrow in new position
  last_dx1 = dx1;
  last_dy1 = dy1;
  delay(25);

  //printing GPS stuff
  gps_log();
 //end 
 //calculate the distance to the destination 
  unsigned long distanceKmToLondon = //*have to look why I wrote this line XD
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      lat,
      lng);
  tft.setCursor(300, 0);
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.print(olddistance);
  tft.setCursor(300, 0);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print(distanceKmToLondon);
  olddistance = distanceKmToLondon;
  //printInt(distanceKmToLondon, gps.location.isValid(), 9);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();

  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    // Serial.println(F("No GPS data received: check wiring"));
    Serial.println(getBearing(gps.location.lat(), gps.location.lng(), 51.000832, 13.682485));
  //Serial.println("aktuell");
  ///  Serial.println(headingDegrees);
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

void GPS()
{
  tft.setCursor(0, 0);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("SAT =");
  tft.setCursor(200, 0);
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.print(kmh);
  tft.setCursor(70, 0);
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.print(sat);
  tft.setCursor(70, 0);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print((gps.satellites.value()));
  sat = gps.satellites.value();

  tft.setCursor(200, 0);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print(gps.speed.kmph());
  kmh = gps.speed.kmph();

  // tft.print( gps.location.lat());
  //tft.print("gps.location.lat()");

  //tft.setCursor(0,300);
  //tft.print(gps.location.lng());
  // printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
}

