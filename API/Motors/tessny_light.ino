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

const int escPin1 = 38;
const int escPin2 = 37;
const int escPin3 = 36;
const int escPin4 = 35;

const int minPulse = 1100;
const int maxPulse = 1900;

float desire_surge, desire_sway, desire_yaw;
int pwm1, pwm2, pwm3, pwm4;
// ======================
// Variables for GPS
// ======================
float raw_lat, raw_lon, raw_heading;
float raw_speed_knots = 0;

float dt = 0.01;  // 100Hz for IMU
const unsigned long loopInterval = dt * 1000; // milliseconds
bool debugMode = true;
unsigned long last_gps_time = 0;

// ======================
// Setup
// ======================
void setup(){
    Serial.begin(115200);
    Serial5.begin(115200);


    forward_port.attach(escPin4);
    forward_starboard.attach(escPin1);
    aft_port.attach(escPin3);
    aft_starboard.attach(escPin2);

    Serial.println("Teensy started with complementary filter");
}

// ======================
// Thruster mixer for holonomic boat
// ======================
void moveThrusters(double surge, double sway, double yaw){
    // TODO fix motor mixing.

    double m1 =  surge - sway + yaw; // forward_port
    double m2 =  surge + sway - yaw; // forward_starboard
    double m3 =  surge - sway - yaw; // aft_port
    double m4 =  surge + sway + yaw; // aft_starboard

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
// GPS parsing
// ======================
double DM_to_DD(String dm) {
    if (dm.length() < 4) return 0;
    double v = dm.toFloat();
    int deg = (int)(v / 100);
    double minutes = v - deg * 100;
    return deg + minutes / 60.0;
}

// ======================
// Serial parsing
// ======================
void parseSerial(String data){
    int c1 = data.indexOf(',');
    int c2 = data.indexOf(',', c1 + 1);
    int c3 = data.indexOf(',', c2 + 1);

    desire_surge        = data.substring(0, c1).toFloat();
    desire_sway         = data.substring(c1 + 1, c2).toFloat();
    desire_yaw          = data.substring(c2 + 1, c3).toFloat();

    if(debugMode){
    Serial.print("surge: "); Serial.print(desire_surge);
    Serial.print("lateral: "); Serial.print(desire_sway);
    Serial.print("yaw: "); Serial.print(desire_yaw);
    Serial.println("");
    }
    if(debugMode){
        Serial.print("PWMs: ");
        Serial.print(pwm1); Serial.print(" ");
        Serial.print(pwm2); Serial.print(" ");  
        Serial.print(pwm3); Serial.print(" ");
        Serial.print(pwm4); Serial.println();
    }
}

void parseRMC(String line) {
    if (!line.startsWith("$GNRMC")) return;

    String field[15];
    int idx = 0;
    int start = 0;
    
    for (int i = 0; i < line.length() && idx < 15; i++) {
        if (line[i] == ',' || line[i] == '*') {
            field[idx++] = line.substring(start, i);
            start = i + 1;
        }
    }

    if (field[2] != "A") return; // No valid fix

    // Latitude
    if (field[3].length() > 0) {
        double lat = DM_to_DD(field[3]);
        if (field[4] == "S") lat = -lat;
        raw_lat = lat;
    }

    // Longitude  
    if (field[5].length() > 0) {
        double lon = DM_to_DD(field[5]);
        if (field[6] == "W") lon = -lon;
        raw_lon = lon;
    }

    // Speed (knots to m/s)
    if (field[7].length() > 0) {
        raw_speed_knots = field[7].toFloat();
    }

    // Heading
    if (field[8].length() > 0) {
        raw_heading = field[8].toFloat();
    }
    
    last_gps_time = millis();
}

void loop(){
    unsigned long currentTime = millis();
    
    // Read GPS
    while(Serial5.available() > 0){
        static String line = "";
        char c = Serial5.read();
        
        if(c == '\n'){
            parseRMC(line);
            line = "";
        } else if(c != '\r') {
            line += c;
        }
    }


    // Print data
    // Serial.print(raw_lat, 8);
    // Serial.print(","); Serial.print(raw_lon, 8);
    // Serial.print(","); Serial.print(raw_heading, 1);
    // Serial.print(","); Serial.print(raw_speed_knots);
    // Serial.println("");

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

    unsigned long elapsed = millis() - currentTime;
    if (elapsed < loopInterval) delay(loopInterval - elapsed);
}