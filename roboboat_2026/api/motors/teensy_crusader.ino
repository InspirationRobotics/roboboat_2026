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


const int escPin1 = 5;
const int escPin2 = 4;
const int escPin3 = 3;
const int escPin4 = 2;
const int pumpPin = 1;
const int statePin = 0;
const int minPulse = 1100;
const int maxPulse = 1900;

float desire_surge, desire_sway, desire_yaw;
int pwm1, pwm2, pwm3, pwm4;
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
    pinMode(pumpPin, OUTPUT);
    pinMode(statePin, INPUT);

    Serial.println("Teensy started Teensy Light");
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
    int c4 = data.indexOf(',', c3 + 1);

    desire_surge        = data.substring(0, c1).toFloat();
    desire_sway         = data.substring(c1 + 1, c2).toFloat();
    desire_yaw          = data.substring(c2 + 1, c3).toFloat();
    int pump_reading        = data.substring(c3 + 1, c4).toFloat();

    if(pump_reading==1){
        // Serial.println("Water pump activated");
        activate_pump = true;
    }else{
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

    moveThrusters(desire_surge,desire_sway,desire_yaw);

    if(activate_pump){
        digitalWrite(pumpPin, HIGH);
    }else{
        digitalWrite(pumpPin, LOW);
    }

    Serial.println(digitalRead(statePin));


    unsigned long elapsed = millis() - currentTime;
    if (elapsed < loopInterval) delay(loopInterval - elapsed);
}