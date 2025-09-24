#include <FastLED.h>
#define NUM_LEDS 45
#define DATA_PIN 45


// Define the array of leds
CRGB leds[NUM_LEDS];


void setup() {
 FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
 FastLED.setBrightness(50);
}


void loop() {
 // Turn on red LED
 for(int i = 0; i<45; i++){
 leds[i] = CRGB::White;
 }
 FastLED.show();


}
