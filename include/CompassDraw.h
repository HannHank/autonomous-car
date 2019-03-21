#include <math.h>
#include <MCUFRIEND_kbv.h>



MCUFRIEND_kbv tft;

// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double
//up the pins with the touch screen (see the TFT paint example).
#define DHTPIN 51
#define DHTTYPE DHT11

#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7
//   Assign human-readable names to some common 16-bit colour values:
const int centreX = 160;
const int centreY = 120;
const int diameter = 100;
const int centreX1 = 160;
const int centreY1 = 120;
const int diameter1 = 100;
const int x_offset = 30;
const int y_offset = 128;
const int z_offset = 0;
int last_dx, last_dy, dx, dy;
int last_dx1, last_dy1, dx1, dy1;


void DrawCompassSetup(){
    last_dx = centreX;
    last_dy = centreY;
    last_dx1 = centreX1;
    last_dy1 = centreY1;
}
void display_item(int x, int y, String token, int txt_colour, int txt_size)
{
  tft.setCursor(x, y);
  tft.setTextColor(txt_colour);
  tft.setTextSize(txt_size);
  tft.print(token);
  tft.setTextSize(2); // Back to default text size
}
void arrow(int x2, int y2, int x1, int y1, int alength, int awidth, int colour)
{
  float distance;
  int dx, dy, x2o, y2o, x3, y3, x4, y4, k;
  distance = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
  dx = x2 + (x1 - x2) * alength / distance;
  dy = y2 + (y1 - y2) * alength / distance;
  k = awidth / alength;
  x2o = x2 - dx;
  y2o = dy - y2;
  x3 = y2o * k + dx;
  y3 = x2o * k + dy;
  //
  x4 = dx - y2o * k;
  y4 = dy - x2o * k;
  tft.drawLine(x1, y1, x2, y2, colour);
  tft.drawLine(x1, y1, dx, dy, colour);
  tft.drawLine(x3, y3, x4, y4, colour);
  tft.drawLine(x3, y3, x2, y2, colour);
  tft.drawLine(x2, y2, x4, y4, colour);
}

void Draw_Compass_Rose()
{
  int dxo, dyo, dxi, dyi;
  tft.drawCircle(centreX, centreY, diameter, YELLOW); // Draw compass circle
  for (float i = 0; i < 360; i = i + 22.5)
  {
    dxo = diameter * cos((i - 90) * 3.14 / 180);
    dyo = diameter * sin((i - 90) * 3.14 / 180);
    dxi = dxo * 0.9;
    dyi = dyo * 0.9;
    tft.drawLine(dxi + centreX, dyi + centreY, dxo + centreX, dyo + centreY, YELLOW);
  }
  display_item((centreX - 5), (centreY - 85), "N", RED, 2);
  display_item((centreX - 5), (centreY + 70), "S", RED, 2);
  display_item((centreX + 80), (centreY - 5), "E", RED, 2);
  display_item((centreX - 85), (centreY - 5), "W", RED, 2);
}