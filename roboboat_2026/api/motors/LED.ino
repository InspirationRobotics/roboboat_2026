#include <Adafruit_NeoPixel.h>

// ======================
// LED Configuration
// ======================
#define LED_PIN     11      // D11
#define NUM_LEDS    144
#define BRIGHTNESS  75

Adafruit_NeoPixel strip(
  NUM_LEDS,
  LED_PIN,
  NEO_GRB + NEO_KHZ800
);

// ======================
// Variables
// ======================
int led_state = 0;
unsigned long lastAnimTime = 0;
int red_pos = 0;

// ======================
// LED Helpers
// ======================
void setStripColor(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

void turnOffStrip() {
  strip.clear();
  strip.show();
}

void flashOnce(uint8_t r, uint8_t g, uint8_t b) {
  setStripColor(r, g, b);
  delay(200);
  turnOffStrip();
}

void flashTwice(uint8_t r, uint8_t g, uint8_t b) {
  flashOnce(r, g, b);
  delay(100);
  flashOnce(r, g, b);
}

// ======================
// Red Running Animation
// ======================
void redAnimation() {
  if (millis() - lastAnimTime < 20) return;
  lastAnimTime = millis();

  strip.clear();
  strip.setPixelColor(red_pos, strip.Color(255, 0, 0));
  strip.show();

  red_pos++;
  if (red_pos >= NUM_LEDS) red_pos = 0;
}

// ======================
// Setup
// ======================
void setup() {
  Serial.begin(115200);

  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.show();

  Serial.println("Arduino Mini LED Controller Ready");
}

// ======================
// Loop
// ======================
void loop() {
  // ---------- Serial Read ----------
  if (Serial.available()) {
    led_state = Serial.parseInt();
    while (Serial.available()) Serial.read(); // clear buffer
  }

  // ---------- LED State Machine ----------
  if (led_state == 0) {
    turnOffStrip();
  }
  else if (led_state == 1) {
    flashOnce(0, 255, 0);
    led_state = 0;  // auto-reset
  }
  else if (led_state == 2) {
    flashTwice(0, 255, 0);
    led_state = 0;  // auto-reset
  }
  else if (led_state == 9) {
    redAnimation(); // special mode
  }
  
}
