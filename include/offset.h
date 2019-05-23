unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 20000;           // interval at which to blink (milliseconds)
bool calibrate_compass = true;


int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int offX = 0;
int offY = 0;

void calculate_offsets(){
while(calibrate_compass){
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    
    previousMillis = currentMillis;
    calibrate_compass = false;
  }


   Vector mag = compass.readRaw();

  // Determine Min / Max values
  if (mag.XAxis < minX) minX = mag.XAxis;
  if (mag.XAxis > maxX) maxX = mag.XAxis;
  if (mag.YAxis < minY) minY = mag.YAxis;
  if (mag.YAxis > maxY) maxY = mag.YAxis;

  // Calculate offsets
  offX = (maxX + minX)/2;
  offY = (maxY + minY)/2;
//  Serial.print(offX);
//  Serial.print(" : ");
//  Serial.println(offY);
  // Serial.print(mag.XAxis);
  // Serial.print(":");
  // Serial.print(mag.YAxis);
  // Serial.print(":");
  // Serial.print(minX);
  // Serial.print(":");
  // Serial.print(maxX);
  // Serial.print(":");
  // Serial.print(minY);
  // Serial.print(":");
  // Serial.print(maxY);
  // Serial.print(":");
  // Serial.print(offX);
  // Serial.print(":");
  // Serial.print(offY);
  // Serial.print("\n");

}


}