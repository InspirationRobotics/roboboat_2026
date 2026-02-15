// =====================================================
// Teensy 4.1 Direct ESC Control + LED Timer
// Serial format:
// [esc1, esc2, esc3, esc4, pump, led]\n
// =====================================================

#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <IntervalTimer.h>

// ======================
// LED Configuration
// ======================
#define LED_PIN     23      // A9
#define NUM_LEDS    144
#define BRIGHTNESS  75

Adafruit_NeoPixel strip(
    NUM_LEDS,
    LED_PIN,
    NEO_GRB + NEO_KHZ800
);

// ======================
// ESC & IO Configuration
// ======================
Servo forward_port;        // ESC1
Servo forward_starboard;   // ESC2
Servo aft_port;            // ESC3
Servo aft_starboard;       // ESC4
Servo pump;

const int escPin1 = 3;
const int escPin2 = 37;
const int escPin3 = 36;
const int escPin4 = 4;
const int pumpPin = 2;

const int statePin = 20;   // A8

const int minPulse = 1400;
const int maxPulse = 1600;
const int neutralPulse = 1500;

// ======================
// Control Variables
// ======================
float esc_cmd[4] = {0, 0, 0, 0};   // [-1, 1]
bool activate_pump = false;
volatile int current_led_state = 0;
volatile bool state_pin_high = true;

// PWM outputs
int pwm1, pwm2, pwm3, pwm4;

// ======================
// Timing
// ======================
const float dt = 0.01;  // 100 Hz
const unsigned long loopInterval = dt * 1000;

// ======================
// LED Timer
// ======================
IntervalTimer ledTimer;

// =====================================================
// LED Update ISR (50 Hz)
// =====================================================
void updateLEDs() {
    if (!state_pin_high) {
        strip.clear();
        strip.show();
        return;
    }

    static int last_state = -1;
    static unsigned long flash_timer = 0;
    static int phase = 0;

    if (current_led_state != last_state) {
        last_state = current_led_state;
        flash_timer = millis();
        phase = 0;
    }

    unsigned long elapsed = millis() - flash_timer;

    if (current_led_state == 0) {
        Serial.println("LED State 0");
        if (phase != 99) {
            strip.clear();
            strip.show();
            phase = 99;
        }
    }
    else if (current_led_state == 1) {
        Serial.println("LED State 1");
        if (elapsed < 200 && phase == 0) {
            strip.fill(strip.Color(0, 255, 0));
            strip.show();
            phase = 1;
        }
        else if (elapsed >= 200 && phase == 1) {
            strip.clear();
            strip.show();
            phase = 2;
        }
    }
    else if (current_led_state == 2) {
        Serial.println("LED State 2");
        if (elapsed < 200 && phase == 0) {
            strip.fill(strip.Color(0, 255, 0));
            strip.show();
            phase = 1;
        }
        else if (elapsed < 350 && phase == 1) {
            strip.clear();
            strip.show();
            phase = 2;
        }
        else if (elapsed < 550 && phase == 2) {
            strip.fill(strip.Color(0, 255, 0));
            strip.show();
            phase = 3;
        }
        else if (elapsed >= 550 && phase == 3) {
            strip.clear();
            strip.show();
            phase = 4;
        }
    }
}

// =====================================================
// Write ESCs directly
// =====================================================
void writeESCs() {
    pwm1 = neutralPulse + esc_cmd[0] * 300;
    pwm2 = neutralPulse + esc_cmd[1] * 400;
    pwm3 = neutralPulse + esc_cmd[2] * 400;
    pwm4 = neutralPulse + esc_cmd[3] * 400;

    pwm1 = constrain(pwm1, minPulse, maxPulse);
    pwm2 = constrain(pwm2, minPulse, maxPulse);
    pwm3 = constrain(pwm3, minPulse, maxPulse);
    pwm4 = constrain(pwm4, minPulse, maxPulse);

    Serial.print(pwm1); Serial.print(" ");
    Serial.print(pwm2); Serial.print(" ");
    Serial.print(pwm3); Serial.print(" ");
    Serial.print(pwm4); Serial.println(" ");
    
    forward_port.writeMicroseconds(pwm1);
    forward_starboard.writeMicroseconds(pwm2);
    aft_port.writeMicroseconds(pwm3);
    aft_starboard.writeMicroseconds(pwm4);
}

// =====================================================
// Serial Parser
// =====================================================
void parseSerial(String data) {
    int idx[6];
    int last = 0;

    for (int i = 0; i < 6; i++) {
        idx[i] = data.indexOf(',', last);
        if (idx[i] == -1 && i < 5) return;
        last = idx[i] + 1;
    }

    esc_cmd[0] = data.substring(0, idx[0]).toFloat();
    esc_cmd[1] = data.substring(idx[0] + 1, idx[1]).toFloat();
    esc_cmd[2] = data.substring(idx[1] + 1, idx[2]).toFloat();
    esc_cmd[3] = data.substring(idx[2] + 1, idx[3]).toFloat();

    activate_pump = data.substring(idx[3] + 1, idx[4]).toInt() == 1;
    current_led_state = data.substring(idx[4] + 1).toInt();

    for (int i = 0; i < 4; i++) {
        esc_cmd[i] = constrain(esc_cmd[i], -1.0, 1.0);
    }
}

// =====================================================
// Setup
// =====================================================
void setup() {
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

    ledTimer.begin(updateLEDs, 20000); // 50 Hz

    Serial.println("Teensy 4.1 Direct ESC Control Ready");
}

// =====================================================
// Main Loop
// =====================================================
void loop() {
    unsigned long start = millis();

    // ---------- Serial Read ----------
    while (Serial.available()) {
        static String line = "";
        char c = Serial.read();

        if (c == '\n') {
            parseSerial(line);
            line = "";
        }
        else if (c != '\r') {
            line += c;
        }
    }

    // ---------- Outputs ----------
    writeESCs();

    if (activate_pump) {
        pump.writeMicroseconds(1900);
    } else {
        pump.writeMicroseconds(1500);
    }

    state_pin_high = digitalRead(statePin);
    
    // ---------- Loop timing ----------
    unsigned long elapsed = millis() - start;
    if (elapsed < loopInterval) {
        delay(loopInterval - elapsed);
    }
}
