// Author: Jawad Farooq Naik
// Date Created: April 18, 2025
// Location: Boston, MA, USA
// Project: Digital Embodiment
// Institution: Massachusetts College of Art and Design
// Department: Dynamic Media Institute
// Project Description:
// This sketch powers the “sensing band” component of Digital Embodiment. It reads
// acceleration (and optional gyro/temperature) data from an MPU6050 over I²C, applies
// a simple movement threshold, and—when movement is detected—transmits the X/Y/Z data
// via ESP‑NOW to a paired homing beacon device for remote haptic/visual feedback.
//
// Features:
// - Uses Adafruit_MPU6050 library for sensor initialization and filtering.
// - Custom I²C pin mapping (SDA → D6, SCL → D7) to free hardware defaults.
// - ESP‑NOW peer setup to wirelessly send `SensorData` structs.
// - Movement gating: only sends when one of the axes exceeds MOVEMENT_THRESHOLD.
// - Rate‑limiting: enforces a minimum interval of SEND_INTERVAL ms between sends.
//
// Notes:
// - Replace `receiverMac[]` with your beacon’s MAC address.
// - Ensure both ESP32 devices are on the same Wi‑Fi channel for ESP‑NOW to work.
// - Tune MOVEMENT_THRESHOLD and SEND_INTERVAL to balance responsiveness vs. noise/power.

#include <Wire.h>  
#include <Adafruit_MPU6050.h>    // Driver for the MPU6050 accelerometer/gyro
#include <Adafruit_Sensor.h>     // Common sensor interface
#include <esp_now.h>             // ESP‑NOW protocol for low‑power peer‑to‑peer
#include <WiFi.h>                // Needed to initialize ESP‑NOW’s Wi‑Fi mode

// —— I²C PIN OVERRIDES ——
// On some boards we remap I²C SDA/SCL away from defaults:
#define MPU_SDA 6   // Connect the MPU6050 SDA pin to digital pin 6
#define MPU_SCL 7   // Connect the MPU6050 SCL pin to digital pin 7

Adafruit_MPU6050 mpu;            // MPU6050 instance

// —— Remote Peer MAC Address ——
// The 6‑byte MAC of the receiving ESP32 (homing beacon)
uint8_t receiverMac[] = {0x40, 0x4C, 0xCA, 0x55, 0xD7, 0x20};

// —— Data Packet Structure ——
// We send only the fields we care about (float for ease on receiver side).
struct SensorData {
    float x, y, z;
};

// —— Movement & Timing Constants —— 
const float   MOVEMENT_THRESHOLD = 0.5f;  // G‑units; only send if |accel| > this
unsigned long lastSendTime       = 0;     // Timestamp of last transmission
const unsigned long SEND_INTERVAL = 10;   // Minimum ms between ESP‑NOW sends

void setup() {
  Serial.begin(115200);

  // —— 1) Configure Wi‑Fi for ESP‑NOW Peer‑to‑Peer —— 
  WiFi.mode(WIFI_STA);           // Station mode, necessary for ESP‑NOW

  // —— 2) Initialize I²C on custom pins —— 
  Wire.begin(MPU_SDA, MPU_SCL);

  // —— 3) Begin MPU6050, configure ranges & filter —— 
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // —— 4) Initialize ESP‑NOW and add peer —— 
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP‑NOW");
    while (1) delay(10);
  }
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;         // use current Wi‑Fi channel
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP‑NOW peer");
    while (1) delay(10);
  }

  Serial.println("Setup complete.");
}

void loop() {
  // —— 5) Read a full sensor event —— 
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Package only the acceleration axes we need
  SensorData dataToSend = {
    accel.acceleration.x,
    accel.acceleration.y,
    accel.acceleration.z
  };

  // —— 6) Movement gating —— Only send if any axis exceeds threshold
  if (fabs(dataToSend.x) > MOVEMENT_THRESHOLD ||
      fabs(dataToSend.y) > MOVEMENT_THRESHOLD ||
      fabs(dataToSend.z) > MOVEMENT_THRESHOLD) {

    unsigned long now = millis();
    // —— 7) Rate limiting —— ensure at least SEND_INTERVAL ms between sends
    if (now - lastSendTime >= SEND_INTERVAL) {
      esp_err_t result = esp_now_send(receiverMac, (uint8_t *)&dataToSend, sizeof(dataToSend));
      if (result == ESP_OK) {
        Serial.println("Data sent successfully");
      } else {
        Serial.println("Error sending data");
      }
      lastSendTime = now;
    }
  }

  // Small delay to avoid hammering the bus (optional)
  delay(1);
}
