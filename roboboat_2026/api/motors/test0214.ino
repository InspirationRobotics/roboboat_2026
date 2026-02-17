// Teensy 4.1 optimized control code - LED updates in main loop (non-blocking)

#include <Servo.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN     23
#define NUM_LEDS    144
#define BRIGHTNESS  75

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ESC objects
Servo forward_port;
Servo forward_starboard;
Servo aft_port;
Servo aft_starboard;
Servo pump;

const int escPin1 = 38;
const int escPin2 = 37;
const int escPin3 = 36;
const int escPin4 = 35;
const int pumpPin = 2;
const int statePin = 22;
const int minPulse = 1100;
const int maxPulse = 1900;

float desire_surge = 0.0;
float desire_sway = 0.0;
float desire_yaw = 0.0;
int pwm1, pwm2, pwm3, pwm4;
bool activate_pump = false;

float dt = 0.01;
const unsigned long loopInterval = dt * 1000;

// LED control variables (non-blocking)
int current_led_state = 0;
int red_anim_pos = 0;
bool state_pin_high = true;
unsigned long last_led_update = 0;
unsigned long led_flash_start = 0;
int led_flash_phase = 0;
int last_led_state = -1;

// Safety watchdog
unsigned long last_serial_time = 0;
const unsigned long SERIAL_TIMEOUT = 500;

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

void moveThrusters(double surge, double sway, double yaw){
    double m1 = -surge - sway - yaw;
    double m2 = -surge + sway + yaw;
    double m3 =  surge - sway + yaw;
    double m4 = -surge - sway + yaw;

    pwm1 = 1500 + m1 * 300;
    pwm2 = 1500 + m2 * 400;
    pwm3 = 1500 + m3 * 400;
    pwm4 = 1500 + m4 * 400;

    pwm1 = constrain(pwm1, minPulse, maxPulse);
    pwm2 = constrain(pwm2, minPulse, maxPulse);
    pwm3 = constrain(pwm3, minPulse, maxPulse);
    pwm4 = constrain(pwm4, minPulse, maxPulse);

    forward_port.writeMicroseconds(pwm1);
    forward_starboard.writeMicroseconds(pwm2);
    aft_port.writeMicroseconds(pwm3);
    aft_starboard.writeMicroseconds(pwm4);
}

void parseSerial(String data){
    int c1 = data.indexOf(',');
    int c2 = data.indexOf(',', c1 + 1);
    int c3 = data.indexOf(',', c2 + 1);
    int c4 = data.indexOf(',', c3 + 1);

    if (c1 < 0 || c2 < 0 || c3 < 0) {
        return;
    }

    desire_surge = data.substring(0, c1).toFloat();
    desire_sway  = data.substring(c1 + 1, c2).toFloat();
    desire_yaw   = data.substring(c2 + 1, c3).toFloat();

    // Clamp to safe range
    desire_surge = constrain(desire_surge, -1.0, 1.0);
    desire_sway = constrain(desire_sway, -1.0, 1.0);
    desire_yaw = constrain(desire_yaw, -1.0, 1.0);

    int pump_reading;
    if (c4 == -1) {
        pump_reading = data.substring(c3 + 1).toInt();
    } else {
        pump_reading = data.substring(c3 + 1, c4).toInt();
    }

    activate_pump = (pump_reading == 1);

    if (c4 != -1) {
        int c5 = data.indexOf(',', c4 + 1);
        if (c5 == -1) {
            current_led_state = data.substring(c4 + 1).toInt();
        } else {
            current_led_state = data.substring(c4 + 1, c5).toInt();
        }
    }
    
    last_serial_time = millis();
}

// Non-blocking LED update (called from main loop)
void updateLEDs(unsigned long currentTime) {
    if(!state_pin_high) {
        // Red animation - update every 20ms
        if(currentTime - last_led_update >= 20) {
            last_led_update = currentTime;
            strip.clear();
            strip.setPixelColor(red_anim_pos, strip.Color(255, 0, 0));
            strip.show();
            red_anim_pos = (red_anim_pos + 1) % NUM_LEDS;
        }
    } else {
        // Normal LED states
        if(current_led_state != last_led_state) {
            last_led_state = current_led_state;
            led_flash_start = currentTime;
            led_flash_phase = 0;
        }
        
        unsigned long elapsed = currentTime - led_flash_start;
        
        if(current_led_state == 0) {
            if(led_flash_phase == 0) {
                strip.clear();
                strip.show();
                led_flash_phase = 99;
            }
        }
        else if(current_led_state == 1) {
            if(elapsed < 200 && led_flash_phase == 0) {
                for(int i = 0; i < NUM_LEDS; i++) {
                    strip.setPixelColor(i, strip.Color(0, 255, 0));
                }
                strip.show();
                led_flash_phase = 1;
            } else if(elapsed >= 200 && led_flash_phase == 1) {
                strip.clear();
                strip.show();
                led_flash_phase = 2;
            }
        }
        else if(current_led_state == 2) {
            if(elapsed < 200 && led_flash_phase == 0) {
                for(int i = 0; i < NUM_LEDS; i++) {
                    strip.setPixelColor(i, strip.Color(0, 255, 0));
                }
                strip.show();
                led_flash_phase = 1;
            } else if(elapsed >= 200 && elapsed < 350 && led_flash_phase == 1) {
                strip.clear();
                strip.show();
                led_flash_phase = 2;
            } else if(elapsed >= 350 && elapsed < 550 && led_flash_phase == 2) {
                for(int i = 0; i < NUM_LEDS; i++) {
                    strip.setPixelColor(i, strip.Color(0, 255, 0));
                }
                strip.show();
                led_flash_phase = 3;
            } else if(elapsed >= 550 && led_flash_phase == 3) {
                strip.clear();
                strip.show();
                led_flash_phase = 4;
            }
        }
    }
}

void loop(){
    unsigned long currentTime = millis();

    // Parse Serial
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

    // Safety: Serial timeout watchdog
    if(millis() - last_serial_time > SERIAL_TIMEOUT) {
        desire_surge = 0.0;
        desire_sway = 0.0;
        desire_yaw = 0.0;
        activate_pump = false;
    }

    // Read state pin
    int statePWM = analogRead(statePin);
    state_pin_high = (statePWM > 1700);
    
    // Safety: Force stop when state pin is low
    if(!state_pin_high) {
        desire_surge = 0.0;
        desire_sway = 0.0;
        desire_yaw = 0.0;
        activate_pump = false;
    }

    // Update thrusters
    moveThrusters(desire_surge, desire_sway, desire_yaw);

    // Update pump
    if(activate_pump){
        pump.writeMicroseconds(1800);
    } else {
        pump.writeMicroseconds(1500);
    }
    
    // Update LEDs (non-blocking)
    updateLEDs(currentTime);
    
    // Status output
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
