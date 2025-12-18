// Code to go on Barco Polo's Arduino. Gets PWM values for each motor from a serial connection.
// Writes the PWM value to each motor accordingly.

#include <Servo.h>

byte servoPin_forward_port = 9;
byte servoPin_forward_starboard = 3;
byte servoPin_aft_port = 6;
byte servoPin_aft_starboard = 5;

// Define servo objects.
Servo servo_forward_port;
Servo servo_forward_starboard;
Servo servo_aft_port;
Servo servo_aft_starboard;

int forward_port_PWM = 1500;
int forward_starboard_PWM = 1500;
int aft_port_PWM = 1500;
int aft_starboard_PWM = 1500;

// Setup function (runs during initialization)
void setup(){
    // Initialize Serial Communication at 9600 Baud Rate
    Serial.begin(9600);

    // Attach the right servo object to the right pin
    servo_forward_port.attach(servoPin_forward_port);
    servo_forward_starboard.attach(servoPin_forward_starboard);
    servo_aft_port.attach(servoPin_aft_port);
    servo_aft_starboard.attach(servoPin_aft_starboard);

    // Write initial PWM values to the servos (motors)
    servo_forward_port.write(1500);
    servo_forward_starboard.write(1500);
    servo_aft_port.write(1500);
    servo_aft_starboard.write(1500);

    delay(1000);
}

void parseData(String data){
    // NOTE: Can make it more graceful later.
    // Finds the each comma in the data, stores the value before the comma in a variable

    // Find index of comma
    int commaIndex1 = data.indexOf(',');
    int commaIndex2 = data.indexOf(',', commaIndex1 + 1);
    int commaIndex3 = data.indexOf(',', commaIndex2 + 1);

    // Extract value, convert it to an integer from a string
    forward_port_PWM = data.substring(0, commaIndex1).toInt();
    forward_starboard_PWM = data.substring(commaIndex1 + 1, commaIndex2).toInt();
    aft_port_PWM = data.substring(commaIndex2 + 1, commaIndex3).toInt();
    aft_starboard_PWM = data.substring(commaIndex3 + 1).toInt();
}

void sendMotorCommands(){
    servo_forward_port.write(forward_port_PWM);
    servo_forward_starboard.write(forward_starboard_PWM);
    servo_aft_port.write(aft_port_PWM);
    servo_aft_starboard.write(aft_starboard_PWM);
}

void debugDisplay(){
    Serial.print("PWM Value[forward Port]: ");
    Serial.println(forward_port_PWM);
    Serial.print("PWM Value[forward Starboard]: ");
    Serial.println(forward_starboard_PWM);
    Serial.print("PWM Value[Aft Port]: ");
    Serial.println(aft_port_PWM);
    Serial.print("PWM Value[Aft Starboard]: ");
    Serial.println(aft_starboard_PWM);
}

void loop(){
    // Check for serial commands and write those commands to the servo.
    // The commands should be in the form of a list: [forward_port, forward_starboard, aft_port, aft_starboard]
    
    if (Serial.available() > 0){
        String receivedData = Serial.readStringUntil('\n');
        parseData(receivedData);
        // debugDisplay();
        sendMotorCommands();
    }


    // Make a shorter delay time to prevent Arduino from being overloaded
    delay(10);
}
