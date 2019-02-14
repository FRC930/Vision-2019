// Adafruit_NeoPixel - Version: Latest 
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

#define PIN 6

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(144, PIN, NEO_GRB + NEO_KHZ800);

uint32_t yellow = strip.Color(50, 40, 0);
uint32_t yellow_orange = strip.Color(50, 30, 0);
uint32_t red = strip.Color(50, 0, 0);
uint32_t red_orange = strip.Color(50, 27, 0);
uint32_t orange = strip.Color(50, 32, 0);
uint32_t none = strip.Color(0, 0, 0);
int patternID = 0;

void setup() {
  Wire.begin(84);                // join i2c bus with address #84
  Wire.onReceive(receiveEvent);  // register event. Connects the receiveEvent function to the .onReceive event.
  
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  Serial.begin(9600);            // start serial for output
  
  strip.begin();
  strip.show(); 
}

void loop() {
  if(patternID == 1)
  {
    halfAndHalf(strip.Color(0,0,100), strip.Color(100,100,0), 30); 
  }
  if(patternID == 2)
  {
    colorWipe(strip.Color(0,0,100), 30); 
  }
}

void receiveEvent(int howMany) {
  while (Wire.available()) { // loop through all but the last
    char c = Wire.read();
    if(c == 72) digitalWrite(13, HIGH);
    else if (c == 76) digitalWrite(13, LOW);
  }
  int x = Wire.read();    // receive byte as an integer
  patternID = x;
}

void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void colorSlide(uint32_t c, uint8_t wait)
{
  for(uint16_t i = 0; i<strip.numPixels(); i++)
  {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
  for(uint16_t i = 0; i<strip.numPixels(); i++)
  {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
    strip.show();
    delay(wait);
  }
}

void halfAndHalf(uint32_t c1, uint32_t c2, uint8_t wait)
{
  for(uint16_t i = 0; i < strip.numPixels() / 2; i++)
  {
    strip.setPixelColor(i, c1);
    strip.show();
    delay(wait);
  }
  for(uint16_t i = (strip.numPixels() / 2); i < (strip.numPixels() / 2); i++)
  {
    strip.setPixelColor(i, c2);
    strip.show();
    delay(wait);
  }
}
