// Teensy 4.1 optimized control code with hardware timer for LEDs

#include <Servo.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#include <IntervalTimer.h>

#define LED_PIN     23        // Data pin (change if needed)  A9
#define NUM_LEDS    144       // Number of LEDs
#define BRIGHTNESS  75        // 0–255 (start lower to be safe)

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
const int statePin = 22; // A8
const int minPulse = 1100;
const int maxPulse = 1900;

float desire_surge = 0.0;
float desire_sway = 0.0;
float desire_yaw = 0.0;
int pwm1, pwm2, pwm3, pwm4;
bool activate_pump = false;

float dt = 0.01;  // 100Hz for IMU
const unsigned long loopInterval = dt * 1000; // milliseconds

// ======================
// LED control with hardware timer
// ======================
IntervalTimer ledTimer;
volatile int current_led_state = 0;
volatile int red_anim_pos = 0;
volatile bool state_pin_high = true;

// LED update function called by hardware timer at 50Hz (every 20ms)
void updateLEDs() {
    if(!state_pin_high) {
        // Red animation when state pin is low
        strip.clear();
        strip.setPixelColor(red_anim_pos, strip.Color(255, 0, 0));
        strip.show();
        red_anim_pos = (red_anim_pos + 1) % NUM_LEDS;
    } else {
        // Handle led_state when state pin is high
        static int last_state = -1;
        static unsigned long flash_timer = 0;
        static int flash_phase = 0;
        
        if(current_led_state != last_state) {
            last_state = current_led_state;
            flash_timer = millis();
            flash_phase = 0;
        }
        
        unsigned long elapsed = millis() - flash_timer;
        
        if(current_led_state == 0) {
            // LED off
            if(flash_phase != 99) {  // Only clear once
                strip.clear();
                strip.show();
                flash_phase = 99;
            }
        }
        else if(current_led_state == 1) {
            // Flash once
            if(elapsed < 200 && flash_phase == 0) {
                for(int i = 0; i < NUM_LEDS; i++) {
                    strip.setPixelColor(i, strip.Color(0, 255, 0));
                }
                strip.show();
                flash_phase = 1;
            } else if(elapsed >= 200 && flash_phase == 1) {
                strip.clear();
                strip.show();
                flash_phase = 2;
            }
        }
        else if(current_led_state == 2) {
            // Flash twice
            if(elapsed < 200 && flash_phase == 0) {
                // First flash ON
                for(int i = 0; i < NUM_LEDS; i++) {
                    strip.setPixelColor(i, strip.Color(0, 255, 0));
                }
                strip.show();
                flash_phase = 1;
            } else if(elapsed >= 200 && elapsed < 350 && flash_phase == 1) {
                // First flash OFF
                strip.clear();
                strip.show();
                flash_phase = 2;
            } else if(elapsed >= 350 && elapsed < 550 && flash_phase == 2) {
                // Second flash ON
                for(int i = 0; i < NUM_LEDS; i++) {
                    strip.setPixelColor(i, strip.Color(0, 255, 0));
                }
                strip.show();
                flash_phase = 3;
            } else if(elapsed >= 550 && flash_phase == 3) {
                // Second flash OFF
                strip.clear();
                strip.show();
                flash_phase = 4;
            }
        }
    }
}

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

    // Start hardware timer for LEDs at 50Hz (every 20ms)
    ledTimer.begin(updateLEDs, 20000); // 20000 microseconds = 20ms

    Serial.println("Teensy started Teensy Light");
}

// ======================
// Thruster mixer for holonomic boat
// ======================
void moveThrusters(double surge, double sway, double yaw){
    double m1 = -surge - sway - yaw;  // forward_port
    double m2 = -surge + sway + yaw;  // forward_starboard
    double m3 =  surge - sway + yaw;  // aft_port
    double m4 = -surge - sway + yaw;  // aft_starboard

    pwm1 = 1500 + m1 * 300;  // forward_port
    pwm2 = 1500 + m2 * 400;  // forward_starboard
    pwm3 = 1500 + m3 * 400;  // aft_port
    pwm4 = 1500 + m4 * 400;  // aft_starboard

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

        // Optional LED field
        if (c5 == -1) {
            current_led_state = data.substring(c4 + 1).toInt();
        } else {
            current_led_state = data.substring(c4 + 1, c5).toInt();
        }
    }
}

// ======================
// Main Loop
// ======================
void loop(){
    unsigned long currentTime = millis();

    // Parse Serial input
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

    // Update thruster commands
    moveThrusters(desire_surge, desire_sway, desire_yaw);

    // Update pump
    if(activate_pump){
        pump.writeMicroseconds(1800);
    } else {
        pump.writeMicroseconds(1500);
    }
    
    // Read state pin and update variables for LED timer
    int statePWM = analogRead(statePin);
    state_pin_high = (statePWM > 1700);
    
    if(state_pin_high){
        Serial.println(1);
    } else {
        Serial.println(0);
    }

    // Maintain loop timing
    unsigned long elapsed = millis() - currentTime;
    if (elapsed < loopInterval) {
        delay(loopInterval - elapsed);
    }
}
