// Teensy control code with complementary filter, debug mode, and remapped ESCs for holonomic boat

#include <Servo.h>
#include <math.h>

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
const int minPulse = 1100;
const int maxPulse = 1900;

float desire_surge, desire_sway, desire_yaw;
int pwm1, pwm2, pwm3, pwm4;
bool activate_pump = false;



// ======================
// Individual motor unit-test (RAW override)
// ======================
// Commands over Serial (115200):
//   RAW,pwm1,pwm2,pwm3,pwm4        -> direct PWM (us) to each thruster (1-4)
//   MOTOR,idx,pwm                  -> set only thruster idx (1-4) to pwm, others to 1500
//   RAW_OFF  (or STOP)             -> disable RAW mode, return to mixer control
//
// Thruster index mapping:
//   1 = forward_port
//   2 = forward_starboard
//   3 = aft_port
//   4 = aft_starboard
bool rawMode = false;
int rawPwm1 = 1500, rawPwm2 = 1500, rawPwm3 = 1500, rawPwm4 = 1500;
unsigned long raw_last_ms = 0;
const unsigned long raw_timeout_ms = 600; // safety stop if commands stop arriving
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

    Serial.println("Teensy started Teensy Light");

    // Arm ESCs at neutral
    setAllNeutral();
}


// Write direct PWM to thrusters (RAW mode)
void writeRawThrusters() {
    forward_port.writeMicroseconds(rawPwm1);
    forward_starboard.writeMicroseconds(rawPwm2);
    aft_port.writeMicroseconds(rawPwm3);
    aft_starboard.writeMicroseconds(rawPwm4);
}

void setAllNeutral() {
    rawPwm1 = rawPwm2 = rawPwm3 = rawPwm4 = 1500;
    writeRawThrusters();
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
    data.trim();

    // --- RAW / unit-test commands ---
    if (data.startsWith("RAW,")) {
        // Format: RAW,pwm1,pwm2,pwm3,pwm4   (microseconds)
        int c1 = data.indexOf(',', 4);
        int c2 = data.indexOf(',', c1 + 1);
        int c3 = data.indexOf(',', c2 + 1);

        if (c1 > 0 && c2 > 0 && c3 > 0) {
            rawPwm1 = constrain(data.substring(4, c1).toInt(),  minPulse, maxPulse);
            rawPwm2 = constrain(data.substring(c1 + 1, c2).toInt(), minPulse, maxPulse);
            rawPwm3 = constrain(data.substring(c2 + 1, c3).toInt(), minPulse, maxPulse);
            rawPwm4 = constrain(data.substring(c3 + 1).toInt(),      minPulse, maxPulse);

            rawMode = true;
            raw_last_ms = millis();
        }
        return;
    }

    if (data.startsWith("MOTOR,")) {
        // Format: MOTOR,idx,pwm   (idx 1-4)
        int c1 = data.indexOf(',', 6);
        if (c1 > 0) {
            int idx = data.substring(6, c1).toInt();
            int pwm = constrain(data.substring(c1 + 1).toInt(), minPulse, maxPulse);

            rawPwm1 = rawPwm2 = rawPwm3 = rawPwm4 = 1500;
            if      (idx == 1) rawPwm1 = pwm;
            else if (idx == 2) rawPwm2 = pwm;
            else if (idx == 3) rawPwm3 = pwm;
            else if (idx == 4) rawPwm4 = pwm;

            rawMode = true;
            raw_last_ms = millis();
        }
        return;
    }

    if (data == "RAW_OFF" || data == "STOP") {
        rawMode = false;
        setAllNeutral();
        return;
    }

    // --- Normal control command ---
    // Expected format: surge,sway,yaw,pump
    int c1 = data.indexOf(',');
    int c2 = data.indexOf(',', c1 + 1);
    int c3 = data.indexOf(',', c2 + 1);
    int c4 = data.indexOf(',', c3 + 1);

    if (c1 < 0 || c2 < 0 || c3 < 0) {
        return; // malformed line
    }

    desire_surge = data.substring(0, c1).toFloat();
    desire_sway  = data.substring(c1 + 1, c2).toFloat();
    desire_yaw   = data.substring(c2 + 1, c3).toFloat();

    int pump_reading = 0;
    if (c4 > 0) pump_reading = data.substring(c3 + 1, c4).toInt();
    else        pump_reading = data.substring(c3 + 1).toInt();

    if (pump_reading == 1) {
        activate_pump = true;
    } else {
        activate_pump = false;
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

    
    // RAW unit-test mode overrides mixer control (with safety timeout)
    if (rawMode) {
        if (millis() - raw_last_ms > raw_timeout_ms) {
            rawMode = false;
            setAllNeutral();
        } else {
            writeRawThrusters();
        }

        // Keep pump control available in RAW mode
        if(activate_pump){
            pump.writeMicroseconds(1800);
        }else{
            pump.writeMicroseconds(1500);
        }

        unsigned long elapsed = millis() - currentTime;
        if (elapsed < loopInterval) delay(loopInterval - elapsed);
        return;
    }

    moveThrusters(desire_surge,desire_sway,desire_yaw);

    if(activate_pump){
        pump.writeMicroseconds(1800);
    }else{
        pump.writeMicroseconds(1500);
    }

    unsigned long elapsed = millis() - currentTime;
    if (elapsed < loopInterval) delay(loopInterval - elapsed);
}
