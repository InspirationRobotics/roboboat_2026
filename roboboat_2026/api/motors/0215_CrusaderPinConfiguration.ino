// =====================================================
// Teensy 4.1 Direct ESC Control + LED Timer
// Serial format:
// [esc1, esc2, esc3, esc4, pump, led]\n
// =====================================================

#include <Servo.h>

// ======================
// ESC & IO Configuration
// ======================
Servo forward_port;        // ESC2
Servo forward_starboard;   // ESC3
Servo aft_port;            // ESC4
Servo aft_starboard;       // ESC1

const int escPin1 = 2; //Aft Starboard
const int escPin2 = 3; //Forward port
const int escPin3 = 4; //Forward starboard
const int escPin4 = 5; //aft port
const int pumpPin = 1;

const int statePin = 6;   // A8

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


// =====================================================
// Write ESCs directly
// =====================================================
void writeESCs() {
    pwm1 = neutralPulse + esc_cmd[0] * 400;
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
    int idx[4];
    int last = 0;

    // Find 4 commas (5 fields total)
    for (int i = 0; i < 4; i++) {
        idx[i] = data.indexOf(',', last);
        if (idx[i] == -1) return;  // malformed packet
        last = idx[i] + 1;
    }

    esc_cmd[0] = data.substring(0, idx[0]).toFloat();
    esc_cmd[1] = data.substring(idx[0] + 1, idx[1]).toFloat();
    esc_cmd[2] = data.substring(idx[1] + 1, idx[2]).toFloat();
    esc_cmd[3] = data.substring(idx[2] + 1, idx[3]).toFloat();

    activate_pump = data.substring(idx[3] + 1).toInt() == 1;

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

    pinMode(statePin, INPUT);

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

    // ---------- Read State Pin ----------
    int stateValue = digitalRead(statePin);   // since you're using pinMode(INPUT)

    // ---------- Outputs ----------
    writeESCs();

    if (activate_pump) {
      digitalWrite(pumpPin,HIGH);
    } else {
      digitalWrite(pumpPin,LOW);
    }

    // ---------- Debug ----------
    Serial.println("==== LOOP DEBUG ====");

    Serial.print("StatePin (digital): ");
    Serial.println(stateValue);

    Serial.print("PWM1: "); Serial.println(pwm1);
    Serial.print("PWM2: "); Serial.println(pwm2);
    Serial.print("PWM3: "); Serial.println(pwm3);
    Serial.print("PWM4: "); Serial.println(pwm4);

    Serial.print("Pump Command: ");
    if (activate_pump) Serial.println("ON (1900)");
    else Serial.println("OFF (1500)");

    unsigned long elapsed = millis() - start;
    Serial.print("Loop Time (ms): ");
    Serial.println(elapsed);

    Serial.println("====================\n");

    // ---------- Loop timing ----------
    if (elapsed < loopInterval) {
        delay(loopInterval - elapsed);
    }
}
