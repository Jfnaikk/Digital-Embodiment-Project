// Author: Jawad Farooq Naik
// Date Created: April 18, 2025, 10:45 AM EDT
// Location: Boston, MA, USA
// Project: Digital Embodiment
// Institution: Massachusetts College of Art and Design
// Department: Dynamic Media Institute
// Project Description:
// Digital Embodiment is an interactive project designed to bridge the emotional gap caused by long‑distance relationships
// by simulating the physical presence of a person away from home. The system consists of two key components: a sensing band
// and a homing beacon. The sensing band (worn by the remote user) detects motion data across X, Y, Z axes and transmits it.
// The homing beacon receives that data via ESP‑NOW and drives three servos to recreate movement—rotating cubes or similar
// elements—in real time. When no data arrives for more than 1 second, the servos return to their home position, indicating
// a temporary loss of presence.
//
// Features:
// - ESP‑NOW receive callback for ultra‑low‑latency, peer‑to‑peer motion data transfer
// - Three‑axis servo actuation (pins 4, 5, 6) mapping acceleration to angles 0°–180°
// - Movement threshold gating to ignore noise (|axis| > 0.5 g)
// - Automatic “homing” of servos to 0° if no packet is received within 1 s
//
// Notes:
// - Servos should be powered from a stable 5–6 V supply; do not draw heavy load from the ESP32’s 3.3 V regulator.
// - Adjust MOVEMENT_THRESHOLD, TIMEOUT_MS, and mapping ranges to fine‑tune responsiveness and idle behavior.

#include <esp_now.h>       // ESP‑NOW peer‑to‑peer comms
#include <WiFi.h>          // Required by ESP‑NOW
#include <ESP32Servo.h>    // Servo control library for ESP32

// —— Pin Definitions —— 
#define SERVO_X_PIN 4      // Servo X‑axis → GPIO4
#define SERVO_Y_PIN 5      // Servo Y‑axis → GPIO5
#define SERVO_Z_PIN 6      // Servo Z‑axis → GPIO6

Servo servoX, servoY, servoZ;    // Servo objects for each axis

// —— Servo & Motion Constants —— 
const int   SERVO_MIN_ANGLE    = 0;     // Minimum physical angle
const int   SERVO_MAX_ANGLE    = 180;   // Maximum physical angle
const float MOVEMENT_THRESHOLD = 0.5f;  // g‑units; ignore moves below this
const int   SERVO_HOME_ANGLE   = 0;     // Angle for “home” on timeout
const unsigned long TIMEOUT_MS = 1000;  // ms without data before homing

// —— Data Packet Structure —— 
// Must match sender’s definition exactly.
struct SensorData {
    float x, y, z;
};

// —— Utility Function —— 
// Maps a float from one range to another.
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min)
           / (in_max - in_min) + out_min;
}

// —— State Tracking —— 
unsigned long lastRecvTime = 0;  // Timestamp of last valid packet

// —— ESP‑NOW Receive Callback ——
// Fires whenever a SensorData packet arrives.
void onDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *incomingData, int len) {
    // 1) Validate packet size
    if (len != sizeof(SensorData)) return;

    // 2) Reset timeout timer
    lastRecvTime = millis();

    // 3) Deserialize incoming bytes
    SensorData data;
    memcpy(&data, incomingData, sizeof(data));

    // 4) Gate by movement threshold
    if (fabs(data.x) > MOVEMENT_THRESHOLD ||
        fabs(data.y) > MOVEMENT_THRESHOLD ||
        fabs(data.z) > MOVEMENT_THRESHOLD) {

        // 5) Map each axis value from [-10,10] to servo angle [0,180]
        int angleX = constrain((int)mapFloat(data.x, -10, 10,
                          SERVO_MIN_ANGLE, SERVO_MAX_ANGLE),
                          SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

        int angleY = constrain((int)mapFloat(data.y, -10, 10,
                          SERVO_MIN_ANGLE, SERVO_MAX_ANGLE),
                          SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

        int angleZ = constrain((int)mapFloat(data.z, -10, 10,
                          SERVO_MIN_ANGLE, SERVO_MAX_ANGLE),
                          SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

        // 6) Actuate servos
        servoX.write(angleX);
        servoY.write(angleY);
        servoZ.write(angleZ);
    }
}

void setup() {
    // —— 1) Initialize WiFi for ESP‑NOW —— 
    WiFi.mode(WIFI_STA);

    // —— 2) Attach and center servos —— 
    servoX.attach(SERVO_X_PIN);
    servoY.attach(SERVO_Y_PIN);
    servoZ.attach(SERVO_Z_PIN);
    servoX.write(SERVO_HOME_ANGLE);
    servoY.write(SERVO_HOME_ANGLE);
    servoZ.write(SERVO_HOME_ANGLE);

    // —— 3) Initialize ESP‑NOW and register callback —— 
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP‑NOW init error");
        return;
    }
    esp_now_register_recv_cb(onDataRecv);

    // —— 4) Seed timer so we don’t immediately home on startup —— 
    lastRecvTime = millis();
}

void loop() {
    // —— 5) Check for timeout —— 
    if (millis() - lastRecvTime > TIMEOUT_MS) {
        // Home all servos if no data recently
        servoX.write(SERVO_HOME_ANGLE);
        servoY.write(SERVO_HOME_ANGLE);
        servoZ.write(SERVO_HOME_ANGLE);
    }
    // Otherwise, servo positions are updated in onDataRecv()
}
