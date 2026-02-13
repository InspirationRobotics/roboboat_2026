// Teensy control code with complementary filter, debug mode, and remapped ESCs for holonomic boat

#include <Servo.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#define LED_PIN     23        // Data pin (change if needed)  A8
#define NUM_LEDS    60       // Number of LEDs
#define BRIGHTNESS  75      // 0–255 (start lower to be safe)

Adafruit_NeoPixel strip(
    NUM_LEDS,
    LED_PIN,
    NEO_GRB + NEO_KHZ800   // WS2812B = GRB + 800kHz
);

// ======================
// ESC objects
// ======================
Servo forward_port;   // ESC1
Servo forward_starboard; // ESC2
Servo aft_port;       // ESC3
Servo aft_starboard;  // ESC4
Servo pump;

const int escPin1 = 38;
const int escPin2 = 37;
const int escPin3 = 36;
const int escPin4 = 35;
const int pumpPin = 2;
const int statePin = 22; // A9
const int minPulse = 1100;
const int maxPulse = 1900;

float desire_surge, desire_sway, desire_yaw;
int pwm1, pwm2, pwm3, pwm4;
int led_state = 0;
bool activate_pump = false;


float dt = 0.01;  // 100Hz for IMU
const unsigned long loopInterval = dt * 1000; // milliseconds

unsigned long last_gps_time = 0;

// ======================
// Setup
// ======================
void setup(){
    Serial.begin(115200);

    forward_port.attach(escPin4);
    forward_starboard.attach(escPin1);
    aft_port.attach(escPin3);
    aft_starboard.attach(escPin2);
    pump.attach(pumpPin);
    pinMode(statePin, INPUT);

    strip.begin();
    strip.setBrightness(BRIGHTNESS);
    strip.show();

    Serial.println("Teensy started Teensy Light");
}
/// LED functions
void setStripColor(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

void turnOffStrip() {
  strip.clear();   // sets all pixels to 0
  strip.show();
}

void flashOnce(uint8_t r, uint8_t g, uint8_t b, int onTime = 200) {
  setStripColor(r, g, b);
  delay(onTime);
  turnOffStrip();
}

void flashTwice(uint8_t r, uint8_t g, uint8_t b, int onTime = 200, int gap = 150) {
  flashOnce(r, g, b, onTime);
  delay(gap);
  flashOnce(r, g, b, onTime);
}

void redAnimation(int delayTime = 20) {
  static int position = 0;

  strip.clear();

  strip.setPixelColor(position, strip.Color(255, 0, 0));
  strip.show();

  position++;
  if (position >= NUM_LEDS) {
    position = 0;
  }

  delay(delayTime);
}

// ======================
// Thruster mixer for holonomic boat
// ======================
void moveThrusters(double surge, double sway, double yaw){
    // TODO fix motor mixing.

    double m1 =  - surge - sway - yaw; // forward_port
    double m2 =  - surge + sway + yaw; // forward_starboard
    double m3 =  surge - sway + yaw; // aft_port
    double m4 =  - surge - sway + yaw; // aft_starboard

    pwm1 = 1500 + m1*400;      // forward_port
    pwm2 = 1500 + m2*400;      // forward_starboard
    pwm3 = 1500 + m3*400;      // aft_port
    pwm4 = 1500 + m4*400;      // aft_starboard - FIXED this one

    pwm1 = constrain(pwm1, minPulse, maxPulse);
    pwm2 = constrain(pwm2, minPulse, maxPulse);
    pwm3 = constrain(pwm3, minPulse, maxPulse);
    pwm4 = constrain(pwm4, minPulse, maxPulse);

    forward_port.writeMicroseconds(pwm1);
    forward_starboard.writeMicroseconds(pwm2);
    aft_port.writeMicroseconds(pwm3);
    aft_starboard.writeMicroseconds(pwm4);

    
}

// ======================
// Serial parsing
// ======================
void parseSerial(String data){
    int c1 = data.indexOf(',');
    int c2 = data.indexOf(',', c1 + 1);
    int c3 = data.indexOf(',', c2 + 1);
    int c4 = data.indexOf(',', c3 + 1);   // may be -1

    // Required fields (must exist)
    if (c1 < 0 || c2 < 0 || c3 < 0) {
        return; // malformed packet
    }

    desire_surge = data.substring(0, c1).toFloat();
    desire_sway  = data.substring(c1 + 1, c2).toFloat();
    desire_yaw   = data.substring(c2 + 1, c3).toFloat();

    // Pump field (last required)
    int pump_reading;
    if (c4 == -1) {
        // No optional fields
        pump_reading = data.substring(c3 + 1).toInt();
    } else {
        pump_reading = data.substring(c3 + 1, c4).toInt();
    }

    activate_pump = (pump_reading == 1);

    // -------- Optional fields --------
    if (c4 != -1) {
        int c5 = data.indexOf(',', c4 + 1);

        // Example: optional LED field
        
        if (c5 == -1) {
            led_state = data.substring(c4 + 1).toInt();
        } else {
            led_state = data.substring(c4 + 1, c5).toInt();
        }
    }
}



void loop(){
    unsigned long currentTime = millis();

    // Parse Serial input
    // Read Serial
    while(Serial.available() > 0){
        static String line = "";
        char c = Serial.read();
        
        if(c == '\n'){
            parseSerial(line);
            line = "";
        } else if(c != '\r') {
            line += c;
        }
    }

    moveThrusters(desire_surge,desire_sway,desire_yaw);

    if(activate_pump){
        pump.writeMicroseconds(1800);
    }else{
        pump.writeMicroseconds(1500);
    }
    
    int statePWM = analogRead(statePin);
    if(statePWM>1700){
        Serial.println(1);
    }else{
        Serial.println(0);
        redAnimation();
    }

    if(led_state==0){ // LED off
        turnOffStrip();
    }
    if(led_state==1){
        flashOnce(0,255,0);
    }
    if(led_state==2){
        flashTwice(0,255,0);
    }


    unsigned long elapsed = millis() - currentTime;
    if (elapsed < loopInterval) delay(loopInterval - elapsed);
}
