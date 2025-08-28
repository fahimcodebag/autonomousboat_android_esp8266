#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>
#include <Servo.h>

// GPS Configuration - Using SoftwareSerial
SoftwareSerial gpsSerial(0, 14); // RX=GPIO0, TX=GPIO14

// WiFi Credentials
const char* ssid = "Xperia_3278";
const char* password = "00008888";
const char* server = "http://192.168.51.20:8080"; //Android server IP

// GPS and MPU6050 DMP objects
TinyGPSPlus gps;
MPU6050 mpu;

// DMP Variables
bool dmpReady = false;          // Set true if DMP init was successful
uint8_t mpuIntStatus;           // Holds actual interrupt status byte from MPU
uint8_t devStatus;              // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;            // Expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64];         // FIFO storage buffer

// Orientation/Motion Variables
Quaternion q;                   // [w, x, y, z] quaternion container
VectorFloat gravity;            // [x, y, z] gravity vector
float ypr[3];                   // [yaw, pitch, roll] container

// Interrupt detection
volatile bool mpuInterrupt = false;
void IRAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

// Servo configuration
Servo steeringServo;
const int SERVO_PIN = 15; // GPIO5 (D1) for servo control
const int SERVO_CENTER = 90; // Center position (boat goes straight)
const int SERVO_MIN = 30; // Maximum left turn
const int SERVO_MAX = 150; // Maximum right turn

// Navigation Variables
double targetLat = 0.0, targetLon = 0.0;
bool destinationSet = false;
bool autonomousMode = false;

// PID Controller Variables
float pidKp = 2.0; // Proportional gain
float pidKi = 0.1; // Integral gain
float pidKd = 0.5; // Derivative gain
float pidIntegral = 0.0;
float pidPrevError = 0.0;
unsigned long pidLastTime = 0;
const float PID_OUTPUT_LIMIT = 60.0; // Max servo angle deviation from center

// WiFiClient object for HTTP requests
WiFiClient client;

// Motor control
const int MOTOR_PIN = 12; // GPIO12 (D6) for motor control

// Current yaw from DMP
float currentYaw = 0.0;

// Function declarations
void sendCurrentLocation();
void fetchTargetCoordinates();
void sendRequest(String endpoint);
double calculateDistance(double lat1, double lon1, double lat2, double lon2);
double calculateBearing(double lat1, double lon1, double lat2, double lon2);
double toradian(double deg);
double todegree(double rad);
double normalizeAngle(double angle);
float pidController(float targetHeading, float currentHeading);
void controlSteering(float targetHeading);
void forward();
void stopMotor();
bool updateDMPYaw();

void setup() {
    Serial.begin(115200);
    
    // Initialize I2C
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    // Initialize MPU6050
    Serial.println(F("Initializing MPU6050..."));
    mpu.initialize();
    pinMode(13, INPUT); // Using GPIO13 for interrupt (change from original GPS TX)
    pinMode(LED_BUILTIN, OUTPUT);
    // Note: This changes GPS to use different pins since GPIO2 is needed for interrupt
    // You may need to adjust GPS pins or use a different interrupt pin
    
    // Test MPU6050 connection
    Serial.println(F("Testing MPU6050 connection..."));
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while(1);
    } else {
        Serial.println("MPU6050 connection successful");
    }
    
    // Initialize DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    // Supply your gyro offsets here (you may need to calibrate these)
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    
    // Make sure DMP initialization was successful
    if (devStatus == 0) {
        // Auto-calibrate
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println("Active offsets:");
        mpu.PrintActiveOffsets();
        
        // Enable DMP
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        
        // Enable interrupt detection
        Serial.println(F("Enabling interrupt detection..."));
        attachInterrupt(digitalPinToInterrupt(13), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        
        // Set DMP ready flag
        Serial.println(F("DMP ready!"));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        while(1); // Stop if DMP fails
    }
    
    // Initialize servo
    steeringServo.attach(SERVO_PIN, 500, 2400);
    steeringServo.write(SERVO_CENTER); // Start with straight steering
    
    // Initialize GPS on SoftwareSerial
    // Note: Changed pins due to interrupt requirement
    gpsSerial.begin(9600);
    
    // Initialize motor pin
    pinMode(MOTOR_PIN, OUTPUT);
    
    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("WiFi connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Notify server of connection
    sendRequest("/connect");
    
    Serial.println("Autonomous boat control with DMP ready!");
    Serial.println("PID Parameters - Kp: " + String(pidKp) + ", Ki: " + String(pidKi) + ", Kd: " + String(pidKd));
}

void loop() {
    // Update DMP yaw data
    digitalWrite(LED_BUILTIN, LOW);
   
    
    // Read GPS Data
    while (gpsSerial.available()) {
        ESP.wdtFeed();
        yield();
        char c = gpsSerial.read();
        ESP.wdtFeed();
        yield();
        gps.encode(c);
        ESP.wdtFeed();
        yield();
        
    }
    
    // Fetch target coordinates from the Android server
    fetchTargetCoordinates();

    // Send current location to the Android server every 5 seconds
    static unsigned long lastSent = 0;
    const unsigned long sendInterval = 5000; // 5 seconds
    if (millis() - lastSent >= sendInterval) {
        delay(0);
        sendCurrentLocation();
        lastSent = millis();
        ESP.wdtFeed();
        yield();
    }
    
    if (!updateDMPYaw()) {
        ESP.wdtFeed();
        yield();
        return; // Skip this loop iteration if DMP data not ready
    }

    // Autonomous Navigation Logic
    if (gps.location.isValid()) {
        yield();
        double currentLat = gps.location.lat();
        double currentLon = gps.location.lng();
        ESP.wdtFeed();
        yield();
        if (destinationSet) {
            double distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
            double bearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
            
            Serial.printf("Lat: %.6f, Lon: %.6f, DMP Yaw: %.1f°\n", currentLat, currentLon, currentYaw);
            Serial.printf("Distance: %.2fm, Target Bearing: %.1f°\n", distance, bearing);
        ESP.wdtFeed();
        yield();
            if (distance <= 2.0) {
                // Destination reached
                stopMotor();
                steeringServo.write(SERVO_CENTER);
                sendRequest("/reached");
                destinationSet = false;
                autonomousMode = false;
                Serial.println("Destination reached!");
                ESP.wdtFeed();
                yield();
            } else {
                // Continue navigation
                if (!autonomousMode) {
                    autonomousMode = true;
                    Serial.println("Starting autonomous navigation...");
                }
                
                // Control steering using PID
                controlSteering(bearing);
                
                // Move forward
                forward();
                
                // Debug output
                Serial.printf("Steering angle: %d°, Error: %.1f°\n", 
                             steeringServo.read(), normalizeAngle(bearing - currentYaw));
            ESP.wdtFeed();
            yield();
            }
        } else {
            // No destination set - stop and center steering
            if (autonomousMode) {
                stopMotor();
                steeringServo.write(SERVO_CENTER);
                autonomousMode = false;
                Serial.println("No destination - stopping autonomous mode");
            }
        }
    } else {
        Serial.println("Waiting for GPS signal...");
    }
    ESP.wdtFeed();
    yield();
     digitalWrite(LED_BUILTIN, HIGH);
    delay(50); // 20Hz update rate
}



bool updateDMPYaw() {
    if (!dmpReady) return false;
    
    // Read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        // Get yaw, pitch, roll values
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        yield();
        mpu.dmpGetGravity(&gravity, &q);
        yield();
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        yield();
        // Convert yaw from radians to degrees and normalize to 0-360
        currentYaw = ypr[0] * 180.0 / M_PI;
        currentYaw = normalizeAngle(currentYaw);
        yield();
        return true;
    }
    ESP.wdtFeed();
    yield();
    return false;
}

// ================== AUTONOMOUS CONTROL FUNCTIONS ================== //

// Calculate bearing from current position to target
double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    double dLon = toradian(lon2 - lon1);
    double lat1Rad = toradian(lat1);
    double lat2Rad = toradian(lat2);
    
    double y = sin(dLon) * cos(lat2Rad);
    double x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(dLon);
    
    double bearing = todegree(atan2(y, x));
    return normalizeAngle(bearing);
}

// Normalize angle to 0-360 degrees
double normalizeAngle(double angle) {
    while (angle < 0) angle += 360;
    while (angle >= 360) angle -= 360;
    return angle;
}

// PID Controller for heading control
float pidController(float targetHeading, float currentHeading) {
    ESP.wdtFeed();
    yield();
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - pidLastTime) / 1000.0; // Convert to seconds
    ESP.wdtFeed();
    yield();
    if (deltaTime <= 0) return 0; // Prevent division by zero
    
    // Calculate error with proper angle wrapping
    float error = normalizeAngle(targetHeading) - normalizeAngle(currentHeading);
    
    // Handle angle wraparound (shortest path)
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
    // Proportional term
    float proportional = pidKp * error;
    ESP.wdtFeed();
    yield();
    // Integral term with windup protection
    pidIntegral += error * deltaTime;
    if (pidIntegral > PID_OUTPUT_LIMIT/pidKi) pidIntegral = PID_OUTPUT_LIMIT/pidKi;
    if (pidIntegral < -PID_OUTPUT_LIMIT/pidKi) pidIntegral = -PID_OUTPUT_LIMIT/pidKi;
    float integral = pidKi * pidIntegral;
    
    // Derivative term
    float derivative = pidKd * (error - pidPrevError) / deltaTime;
    
    // Calculate PID output
    float output = proportional + integral + derivative;
    ESP.wdtFeed();
    yield();
    // Limit output
    if (output > PID_OUTPUT_LIMIT) output = PID_OUTPUT_LIMIT;
    if (output < -PID_OUTPUT_LIMIT) output = -PID_OUTPUT_LIMIT;
    
    // Update for next iteration
    pidPrevError = error;
    pidLastTime = currentTime;
    
    Serial.printf("PID: Error=%.1f°, P=%.1f, I=%.1f, D=%.1f, Output=%.1f\n", 
                  error, proportional, integral, derivative, output);
    ESP.wdtFeed();
    yield();
    return output;
}

// Control steering based on target heading
void controlSteering(float targetHeading) {
    ESP.wdtFeed();
    yield();
    float pidOutput = pidController(targetHeading, currentYaw);
    
    // Convert PID output to servo angle
    int servoAngle = SERVO_CENTER + (int)pidOutput;
    
    // Constrain servo angle to safe limits
    servoAngle = constrain(servoAngle, SERVO_MIN, SERVO_MAX);
    
    // Apply steering
    steeringServo.write(servoAngle);
    ESP.wdtFeed();
    yield();
}

// Motor control function
void forward() {
    digitalWrite(MOTOR_PIN, HIGH);
    ESP.wdtFeed();
    yield();
}

// Stop motor function
void stopMotor() {
    digitalWrite(MOTOR_PIN, LOW);
    Serial.println("Motor stopped");
    ESP.wdtFeed();
    yield();
}

// Convert degrees to radians
double toradian(double deg) {
    return deg * (PI / 180.0);
}

// Convert radians to degrees
double todegree(double rad) {
    return rad * (180.0 / PI);
}

// ================== COMMUNICATION FUNCTIONS ================== //

// Function to send USV's current location to the Android app
void sendCurrentLocation() {
    ESP.wdtFeed();
    yield();
    if (gps.location.isValid()) {
        ESP.wdtFeed();
        String url = String(server) + "/current_location?lat=" + 
                    String(gps.location.lat(), 6) + "&lon=" + 
                    String(gps.location.lng(), 6) + "&yaw=" + 
                    String(currentYaw, 1);
                    
        HTTPClient http;
        http.begin(client, url);
        http.setTimeout(5000);
        int httpCode = http.GET();

        if (httpCode > 0) {
            String payload = http.getString(); // Clear buffer
        }

        http.end();

        if (httpCode == HTTP_CODE_OK) {
            Serial.println("Location and yaw sent");
        } else {
            Serial.printf("Send failed, error: %d\n", httpCode);
        }
    } else {
        Serial.println("GPS invalid - skip update");
    }
    ESP.wdtFeed();
    yield();
}

// Fetch destination coordinates from Android server
void fetchTargetCoordinates() {
    ESP.wdtFeed();
    yield();
    HTTPClient http;
    String url = String(server) + "/update";
    http.begin(client, url);
    ESP.wdtFeed();
    yield();
    int httpCode = http.GET();
    ESP.wdtFeed();
    yield();
    if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        int commaIndex = payload.indexOf(',');
        
        if (commaIndex != -1) {
            double newTargetLat = payload.substring(0, commaIndex).toDouble();
            double newTargetLon = payload.substring(commaIndex + 1).toDouble();
            
            // Only update if coordinates have changed
            if (newTargetLat != targetLat || newTargetLon != targetLon) {
                targetLat = newTargetLat;
                targetLon = newTargetLon;
                destinationSet = true;
                
                // Reset PID controller for new target
                pidIntegral = 0.0;
                pidPrevError = 0.0;
                pidLastTime = millis();
                
                Serial.printf("New target: %.6f, %.6f\n", targetLat, targetLon);
            }
        }
    }
    ESP.wdtFeed();
    yield();
     if (httpCode > 0) {
        String  payload = http.getString(); // Clear buffer
        }
    ESP.wdtFeed();
    yield();
    http.end();

}

// Send requests to the Android app
void sendRequest(String endpoint) {
    HTTPClient http;
    http.begin(client, server + endpoint);
    ESP.wdtFeed();
    yield();
    http.GET();
    http.end();
    Serial.println("Request sent: " + String(server) + endpoint);
    yield();
}

// ================== NAVIGATION FUNCTIONS ================== //

// Haversine formula to calculate distance between two coordinates
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000; // Earth radius in meters
    double dLat = toradian(lat2 - lat1);
    double dLon = toradian(lon2 - lon1);
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(toradian(lat1)) * cos(toradian(lat2)) *
               sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    yield();
    return R * c;
}