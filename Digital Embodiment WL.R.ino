// Author: Jawad Farooq Naik
// Date Created: April 18, 2025, 10:30 AM EDT
// Location: Boston, MA, USA
// Project: Digital Embodiment
// Institution: Massachusetts College of Art and Design
// Department: Dynamic Media Institute
// Project Description:
// This sketch implements the “homing beacon” component of Digital Embodiment.
// It uses ESP‑NOW to receive remote motion data (X, Y, Z acceleration) from the sensing band,
// then maps that data to three servos to recreate the user’s gestures in real time.
//
// Features:
// - Registers an ESP‑NOW receive callback to handle incoming SensorData packets.
// - Uses a movement threshold to gate servo updates, preventing noise from triggering motion.
// - Maps float acceleration values into servo angles across three axes (pins 4, 5, 6).
// - Executes all updates inside the callback for minimal loop overhead.
//
// Notes:
// - Ensure your ESP32 homing beacon is paired with the sensing band via ESP‑NOW.
// - Power servos from a separate 5–6 V supply to avoid drawing too much from the board.
// - Adjust MOVEMENT_THRESHOLD and mapping ranges for your specific motion profile.

#include <esp_now.h>      // Peer‑to‑peer wireless communication
#include <WiFi.h>         // Required by ESP‑NOW
#include <ESP32Servo.h>   // Servo library for ESP32

// —— Pin Definitions —— 
#define SERVO_X_PIN 4     // X‑axis servo → D4
#define SERVO_Y_PIN 5     // Y‑axis servo → D5
#define SERVO_Z_PIN 6     // Z‑axis servo → D6

Servo servoX, servoY, servoZ; // Servo objects for each axis

// —— Motion & Mapping Constants —— 
const int   SERVO_MIN_ANGLE     = 0;    // Minimum servo angle
const int   SERVO_MAX_ANGLE     = 180;  // Maximum servo angle
const float MOVEMENT_THRESHOLD  = 0.5f; // G‑units; only actuate if |accel| > threshold

// —— Data Packet Structure —— 
// Must match the struct used on the sender side.
struct SensorData {
    float x, y, z;
};

// —— Utility: map a float range to an integer range —— 
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// —— ESP‑NOW Receive Callback —— 
// Triggered whenever a SensorData packet arrives.
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    if (len != sizeof(SensorData)) return;  // Validate packet size

    // Deserialize incoming bytes into our struct
    SensorData receivedData;
    memcpy(&receivedData, incomingData, sizeof(receivedData));

    // Gate motion by threshold to avoid noise
    if (fabs(receivedData.x) > MOVEMENT_THRESHOLD ||
        fabs(receivedData.y) > MOVEMENT_THRESHOLD ||
        fabs(receivedData.z) > MOVEMENT_THRESHOLD) {

        // Map each axis to a servo angle, then constrain
        int angleX = constrain((int)mapFloat(receivedData.x, -10, 10, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE),
                               SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        int angleY = constrain((int)mapFloat(receivedData.y, -10, 10, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE),
                               SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        int angleZ = constrain((int)mapFloat(receivedData.z, -10, 10, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE),
                               SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

        // Actuate servos to mirror the remote movement
        servoX.write(angleX);
        servoY.write(angleY);
        servoZ.write(angleZ);
    }
}

void setup() {
    // Initialize Wi‑Fi in station mode for ESP‑NOW
    WiFi.mode(WIFI_STA);

    // Attach each servo to its pin and center at 90°
    servoX.attach(SERVO_X_PIN);
    servoY.attach(SERVO_Y_PIN);
    servoZ.attach(SERVO_Z_PIN);
    servoX.write(90);
    servoY.write(90);
    servoZ.write(90);

    // Initialize ESP‑NOW and register the receive callback
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP‑NOW");
        return;
    }
    esp_now_register_recv_cb(onDataRecv);
}

void loop() {
    // All real‑time work happens in onDataRecv(); the loop remains idle.
}
