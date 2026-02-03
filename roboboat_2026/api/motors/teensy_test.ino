#include <Arduino.h>

// Buffer for storing serial data
char nmeaBuffer[256];
int bufferIndex = 0;
bool newData = false;

// Variables to store parsed data
float latitude = 0.0;
float longitude = 0.0;
float altitude = 0.0;
float heading = 0.0;
char latDirection = 'N';
char lonDirection = 'W';
bool headingValid = false;
bool gpsFix = false;

// Function prototypes
void parseNMEA(char* nmeaSentence);
void parseGNGGA(char* data);
void parseGNTHS(char* data);
float dmToDecimalDegrees(float dm, char direction);
void sendDataToSerial();

void setup() {
  // Start default Serial for output
  Serial.begin(115200);
  
  // Start Serial3 for GPS input
  Serial3.begin(9600);
  
  Serial.println("GPS Parser Started");
  Serial.println("Waiting for GPS data...");
}

void loop() {
  // Read data from Serial3 (GPS)
  while (Serial3.available() > 0) {
    char c = Serial3.read();
    
    // Check for start of NMEA sentence
    if (c == '$') {
      bufferIndex = 0;
      nmeaBuffer[bufferIndex++] = c;
    }
    // Check for end of NMEA sentence
    else if (c == '\n') {
      if (bufferIndex > 0) {
        nmeaBuffer[bufferIndex] = '\0'; // Null terminate the string
        newData = true;
      }
    }
    // Store character in buffer if there's space
    else if (bufferIndex < sizeof(nmeaBuffer) - 1) {
      nmeaBuffer[bufferIndex++] = c;
    }
    
    // If we have a complete sentence, parse it
    if (newData) {
      parseNMEA(nmeaBuffer);
      newData = false;
      bufferIndex = 0;
    }
  }
  
  // Send data to default serial periodically
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime > 1000) { // Send every second
    sendDataToSerial();
    lastSendTime = millis();
  }
}

void parseNMEA(char* nmeaSentence) {
  // Check if it's a GNGGA sentence
  if (strncmp(nmeaSentence, "$GNGGA", 6) == 0) {
    parseGNGGA(nmeaSentence);
  }
  // Check if it's a GNTHS sentence
  else if (strncmp(nmeaSentence, "$GNTHS", 6) == 0) {
    parseGNTHS(nmeaSentence);
  }
}

void parseGNGGA(char* data) {
  // Split the NMEA sentence by commas
  char* tokens[20];
  int tokenCount = 0;
  
  char* token = strtok(data, ",");
  while (token != NULL && tokenCount < 20) {
    tokens[tokenCount++] = token;
    token = strtok(NULL, ",");
  }
  
  // Check if we have enough tokens (minimum 15 for GNGGA)
  if (tokenCount >= 15) {
    // Check GPS fix status (token 6)
    int fixQuality = atoi(tokens[6]);
    gpsFix = (fixQuality > 0);
    
    if (gpsFix) {
      // Parse latitude (token 2) and direction (token 3)
      float latDM = atof(tokens[2]);
      latDirection = tokens[3][0];
      latitude = dmToDecimalDegrees(latDM, latDirection);
      
      // Parse longitude (token 4) and direction (token 5)
      float lonDM = atof(tokens[4]);
      lonDirection = tokens[5][0];
      longitude = dmToDecimalDegrees(lonDM, lonDirection);
      
      // Parse altitude (token 9)
      altitude = atof(tokens[9]);
    }
  }
}

void parseGNTHS(char* data) {
  // Split the NMEA sentence by commas
  char* tokens[10];
  int tokenCount = 0;
  
  char* token = strtok(data, ",");
  while (token != NULL && tokenCount < 10) {
    tokens[tokenCount++] = token;
    token = strtok(NULL, ",");
  }
  
  // Check if we have enough tokens (minimum 3 for GNTHS)
  if (tokenCount >= 3) {
    // Parse heading (token 1)
    heading = atof(tokens[1]);
    
    // Check heading status (token 2, before checksum)
    // Remove checksum from status token if present
    char statusToken[10];
    strncpy(statusToken, tokens[2], sizeof(statusToken));
    statusToken[sizeof(statusToken) - 1] = '\0';
    
    // Find and remove checksum (starts with *)
    char* starPos = strchr(statusToken, '*');
    if (starPos != NULL) {
      *starPos = '\0';
    }
    
    // Check if heading is valid
    headingValid = (statusToken[0] == 'A'); // 'A' = valid
  }
}

float dmToDecimalDegrees(float dm, char direction) {
  // Convert degrees and minutes (DDMM.MMMMM) to decimal degrees
  int degrees = (int)(dm / 100);
  float minutes = dm - (degrees * 100);
  float decimalDegrees = degrees + (minutes / 60.0);
  
  // Adjust for direction
  if (direction == 'S' || direction == 'W') {
    decimalDegrees = -decimalDegrees;
  }
  
  return decimalDegrees;
}

void sendDataToSerial() {
  Serial.print("GPS Status: ");
  Serial.println(gpsFix ? "Fix" : "No Fix");
  
  if (gpsFix) {
    Serial.print("Latitude: ");
    Serial.print(latitude, 8);
    Serial.print(" ");
    Serial.println(latDirection);
    
    Serial.print("Longitude: ");
    Serial.print(longitude, 8);
    Serial.print(" ");
    Serial.println(lonDirection);
    
    Serial.print("Altitude: ");
    Serial.print(altitude, 4);
    Serial.println(" m");
  }
  
  Serial.print("Heading: ");
  if (headingValid) {
    Serial.print(heading, 4);
    Serial.println(" degrees");
  } else {
    Serial.println("Invalid");
  }
  
  Serial.println("--------------------");
}
