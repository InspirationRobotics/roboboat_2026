// Teensy control code with complementary filter, debug mode, and remapped ESCs for holonomic boat

#include <Adafruit_BNO08x.h>
#include <Servo.h>
#include <PID_v1_bc.h>
#include <math.h>

// ======================
// ESC objects
// ======================
Servo forward_port;   // ESC1
Servo forward_starboard; // ESC2
Servo aft_port;       // ESC3
Servo aft_starboard;  // ESC4

const int escPin1 = 3;
const int escPin2 = 4;
const int escPin3 = 5;
const int escPin4 = 6;

const int minPulse = 1100;
const int maxPulse = 1900;

// PID variables
double input_surge, output_surge, setpoint_surge;
double input_sway, output_sway, setpoint_sway;
double input_yaw, output_yaw, setpoint_yaw;

double Kp = 2.0, Ki = 0.0, Kd = 0.5;
PID pid_surge(&input_surge, &output_surge, &setpoint_surge, Kp, Ki, Kd, DIRECT);
PID pid_sway(&input_sway, &output_sway, &setpoint_sway, Kp, Ki, Kd, DIRECT);
PID pid_yaw(&input_yaw, &output_yaw, &setpoint_yaw, Kp, Ki, Kd, DIRECT);

// BNO08x IMU
#define BNO08X_CS 10
#define BNO08X_INT 9
#define BNO08X_RESET -1
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// ======================
// Simple Fusion Variables
// ======================
float fused_lat = 0, fused_lon = 0, fused_heading = 0;
float fused_vN = 0, fused_vE = 0;  // North/East velocity

// Filter coefficients (0-1, smaller = more smoothing)
float alpha_pos = 0.7f;    // Position filter
float alpha_vel = 0.3f;    // Velocity filter  
float alpha_heading = 0.05f; // Heading filter

// ======================
// Variables for GPS/IMU
// ======================
float raw_lat, raw_lon, raw_heading;
float raw_speed_knots = 0;
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float quat_w, quat_x, quat_y, quat_z;
bool valid_GPS;

// ======================
// Variables for control message
// ======================
float desire_lat, desire_lon, desire_heading;
float desire_surge, desire_sway, desire_yaw;


float dt = 0.01;  // 100Hz for IMU
const unsigned long loopInterval = dt * 1000; // milliseconds
bool debugMode = true;
unsigned long last_gps_time = 0;
const unsigned long gps_timeout = 2000; // 2 seconds

// ======================
// Setup
// ======================
void setup(){
    Serial.begin(115200);
    Serial1.begin(115200);

    if (!bno08x.begin_I2C()) {
        Serial.println("BNO08x not found!");
        while(1);
    }
    
    bno08x.enableReport(SH2_LINEAR_ACCELERATION);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED);
    bno08x.enableReport(SH2_ROTATION_VECTOR);

    forward_port.attach(escPin1);
    forward_starboard.attach(escPin2);
    aft_port.attach(escPin3);
    aft_starboard.attach(escPin4);

    // Initialize PID
    pid_surge.SetMode(AUTOMATIC);
    pid_sway.SetMode(AUTOMATIC);
    pid_yaw.SetMode(AUTOMATIC);
    pid_surge.SetOutputLimits(-1.0, 1.0);
    pid_sway.SetOutputLimits(-1.0, 1.0);
    pid_yaw.SetOutputLimits(-1.0, 1.0);

    // initialize pid
    input_surge = 0;
    input_sway = 0;
    input_yaw = 0;

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

    int pwm1 = 1500 + m1*400;      // forward_port
    int pwm2 = 1500 + m2*400;      // forward_starboard
    int pwm3 = 1500 + m3*400;      // aft_port
    int pwm4 = 1500 + m4*400;      // aft_starboard - FIXED this one

    pwm1 = constrain(pwm1, minPulse, maxPulse);
    pwm2 = constrain(pwm2, minPulse, maxPulse);
    pwm3 = constrain(pwm3, minPulse, maxPulse);
    pwm4 = constrain(pwm4, minPulse, maxPulse);

    forward_port.writeMicroseconds(pwm1);
    forward_starboard.writeMicroseconds(pwm2);
    aft_port.writeMicroseconds(pwm3);
    aft_starboard.writeMicroseconds(pwm4);

    if(debugMode){
        Serial.print(surge); Serial.print(sway); Serial.print(yaw);
        Serial.println("");
        Serial.print("PWMs: ");
        Serial.print(pwm1); Serial.print(" ");
        Serial.print(pwm2); Serial.print(" ");  
        Serial.print(pwm3); Serial.print(" ");
        Serial.print(pwm4); Serial.println();
    }
}

// ======================
// PID Control using global variables
// ======================
void updatePIDControl() {

    // --------------------------------------------------------
    // 1. Compute spatial error in meters
    // --------------------------------------------------------
    float dx = (desire_lat - fused_lat) * 111320.0f;  // North
    float dy = (desire_lon - fused_lon) * 111320.0f * cos(fused_lat * DEG_TO_RAD); // East

    // Normalize if necessary (optional, depending on PID scale)
    input_surge = dx;   // forward/back
    setpoint_surge = 0;

    input_sway = dy;    // left/right
    setpoint_sway = 0;

    // --------------------------------------------------------
    // 2. Compute heading error [-180, 180]
    // --------------------------------------------------------
    float yaw_error = desire_heading - fused_heading;
    while (yaw_error > 180) yaw_error -= 360;
    while (yaw_error < -180) yaw_error += 360;
    if(debugMode){
        Serial.print("yaw error: ");Serial.println(yaw_error);
    }

    input_yaw = yaw_error;
    setpoint_yaw = 0;

    // --------------------------------------------------------
    // 3. Compute PID outputs
    // --------------------------------------------------------
    pid_surge.Compute();
    pid_sway.Compute();
    pid_yaw.Compute();

    // --------------------------------------------------------
    // 4. Constrain PID outputs to [-1,1] or desired range
    // --------------------------------------------------------
    output_surge = constrain(output_surge, -1.0, 1.0);
    output_sway  = constrain(output_sway,  -1.0, 1.0);
    output_yaw   = constrain(output_yaw,   -1.0, 1.0);

    // --------------------------------------------------------
    // 5. fuse pid with desire pwm
    // --------------------------------------------------------
    if(debugMode){
        Serial.print("PID inputs: ");
        Serial.print(input_surge); Serial.print(", ");
        Serial.print(input_sway); Serial.print(", ");
        Serial.print(input_yaw); Serial.println("");
        Serial.print("PID output: ");
        Serial.print(output_surge); Serial.print(", ");
        Serial.print(output_sway); Serial.print(", ");
        Serial.print(output_yaw); Serial.println("");
    }

    if(valid_GPS){
        moveThrusters(output_surge+ desire_surge , output_sway+desire_sway, output_yaw+desire_yaw);
    }
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
    int c4 = data.indexOf(',', c3 + 1);
    int c5 = data.indexOf(',', c4 + 1);

    desire_lat            = data.substring(0, c1).toFloat();
    desire_lon            = data.substring(c1 + 1, c2).toFloat();
    desire_heading        = data.substring(c2 + 1, c3).toFloat();
    desire_surge          = data.substring(c3 + 1, c4).toFloat();
    desire_sway        = data.substring(c4 + 1, c5).toFloat();
    desire_yaw            = data.substring(c5 + 1).toFloat();

    if(desire_lat==0){
        desire_lat = fused_lat;
        desire_lon = fused_lon;
        desire_heading = fused_heading;
    }
    
    if(debugMode){
        Serial.print("desire lat: "); Serial.print(desire_lat);
        Serial.print("lon: "); Serial.print(desire_lon);
        Serial.print("heading: "); Serial.print(desire_heading);
        Serial.print("surge: "); Serial.print(desire_surge);
        Serial.print("lateral: "); Serial.print(desire_sway);
        Serial.print("yaw: "); Serial.print(desire_yaw);
        Serial.println("");
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

    if(raw_lat >10){
        valid_GPS = true;
        desire_lat = raw_lat;
        desire_lon = raw_lon;
        desire_heading = raw_heading;
    }
}

// ======================
// Simple Complementary Filter
// ======================
void updateFusion() {
    static unsigned long last_fusion_time = 0;
    unsigned long current_time = millis();
    
    if (last_fusion_time == 0) {
        fused_lat = raw_lat;
        fused_lon = raw_lon;
        fused_heading = raw_heading;
        last_fusion_time = current_time;
        return;
    }
    
    float dt_fusion = (current_time - last_fusion_time) / 1000.0f;
    if (dt_fusion <= 0) dt_fusion = 0.01f;
    
    // Check if GPS is fresh
    bool gps_fresh = (millis() - last_gps_time) < gps_timeout;
    
    // Position fusion (only when GPS is available)
    if (gps_fresh) {
        if (fused_lat == 0 && fused_lon == 0) {
            // First GPS reading
            fused_lat = raw_lat;
            fused_lon = raw_lon;
        } else {
            fused_lat = (1 - alpha_pos) * fused_lat + alpha_pos * raw_lat;
            fused_lon = (1 - alpha_pos) * fused_lon + alpha_pos * raw_lon;
        }
        
        // Calculate velocity from GPS (simple method)
        // In real implementation, you'd use GPS course and speed
        float speed_ms = raw_speed_knots * 0.514444f; // knots to m/s
        
        // Simple velocity from GPS position change (you could improve this)
        static float prev_gps_lat = 0, prev_gps_lon = 0;
        static unsigned long prev_gps_time = 0;
        
        if (prev_gps_time > 0) {
            float gps_dt = (current_time - prev_gps_time) / 1000.0f;
            if (gps_dt > 0) {
                // Rough velocity estimate from position change
                float gps_vN = (raw_lat - prev_gps_lat) * 111320.0f / gps_dt;
                float gps_vE = (raw_lon - prev_gps_lon) * (111320.0f * cos(fused_lat * DEG_TO_RAD)) / gps_dt;
                
                // Fuse velocity
                fused_vN = (1 - alpha_vel) * fused_vN + alpha_vel * gps_vN;
                fused_vE = (1 - alpha_vel) * fused_vE + alpha_vel * gps_vE;
            }
        }
        
        prev_gps_lat = raw_lat;
        prev_gps_lon = raw_lon;
        prev_gps_time = current_time;
        
        // Heading fusion (GPS + IMU)
        fused_heading = (1 - alpha_heading) * fused_heading + alpha_heading * raw_heading;
    } else {
        // Dead reckoning with IMU only (basic integration)
        // Convert velocity to position change
        fused_lat += (fused_vN / 111320.0f) * dt_fusion;
        fused_lon += (fused_vE / (111320.0f * cos(fused_lat * DEG_TO_RAD))) * dt_fusion;
        
        // TODO: Integrate gyro for heading when GPS is lost
    }
    
    // Use IMU accelerations to update velocity (basic integration)
    // Rotate body accelerations to global frame
    float heading_rad = fused_heading * DEG_TO_RAD;
    float ax_global = cos(heading_rad) * accel_x - sin(heading_rad) * accel_y;
    float ay_global = sin(heading_rad) * accel_x + cos(heading_rad) * accel_y;
    
    // Integrate acceleration to get velocity
    fused_vN += ax_global * dt_fusion;
    fused_vE += ay_global * dt_fusion;
    
    // Add some damping to velocity (simplified drag model)
    fused_vN *= 0.98f;
    fused_vE *= 0.98f;
    
    last_fusion_time = current_time;
}

// ======================
// Get heading from quaternion
// ======================
float getHeadingFromQuaternion(float qw, float qx, float qy, float qz) {
    // Convert quaternion to Euler angles (simplified heading calculation)
    float yaw = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
    return yaw * RAD_TO_DEG;
}

void loop(){
    unsigned long currentTime = millis();
    // Read IMU
    if(bno08x.getSensorEvent(&sensorValue)){
        if(sensorValue.sensorId == SH2_LINEAR_ACCELERATION){
            accel_x = sensorValue.un.linearAcceleration.x;
            accel_y = sensorValue.un.linearAcceleration.y;
            accel_z = sensorValue.un.linearAcceleration.z;
        } 
        else if(sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED){
            gyro_x = sensorValue.un.gyroscope.x;
            gyro_y = sensorValue.un.gyroscope.y; 
            gyro_z = sensorValue.un.gyroscope.z;
        }
        else if(sensorValue.sensorId == SH2_ROTATION_VECTOR){
            quat_w = sensorValue.un.rotationVector.real;
            quat_x = sensorValue.un.rotationVector.i;
            quat_y = sensorValue.un.rotationVector.j;
            quat_z = sensorValue.un.rotationVector.k;
            
            // Update heading from IMU when GPS is not available
            if(millis() - last_gps_time > gps_timeout) {
                float imu_heading = getHeadingFromQuaternion(quat_w, quat_x, quat_y, quat_z);
                fused_heading = (1 - alpha_heading) * fused_heading + alpha_heading * imu_heading;
            }
        }
    }

    // Read GPS
    while(Serial1.available() > 0){
        static String line = "";
        char c = Serial1.read();
        
        if(c == '\n'){
            parseRMC(line);
            line = "";
        } else if(c != '\r') {
            line += c;
        }
    }

    if(valid_GPS){
        // Update sensor fusion
        updateFusion();
    
    
        // Print fused data
        Serial.print(fused_lat, 8);
        Serial.print(","); Serial.print(fused_lon, 8);
        Serial.print(","); Serial.print(fused_heading, 1);
        Serial.print(","); Serial.print(sqrt(fused_vN*fused_vN + fused_vE*fused_vE), 2);
        Serial.println("");
    
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
        // Merge PID with desire pwm
        updatePIDControl();
    }

    unsigned long elapsed = millis() - currentTime;
    if (elapsed < loopInterval) delay(loopInterval - elapsed);
}