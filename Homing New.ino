/*
  Digital Embodiment · Homing Beacon (Receiver)
  --------------------------------------------
  • ESP32 DevKit (classic)                      • Servos on 27 / 26 / 25
  • Listens on ESP‑NOW channel 1
  • Maps ±1 g → 0–180 ° for each axis
  • Snaps servos home if motion < 0.3 g
  • Homes servos if no packets for 1 s
*/

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <math.h>
#include <esp_wifi.h>            // low‑level channel API

// ---------- configuration ----------
#define WIFI_CHANNEL    1        // keep in sync with the sender

#define SERVO_X_PIN     27       // D27  → Servo X
#define SERVO_Y_PIN     26       // D26  → Servo Y
#define SERVO_Z_PIN     25       // D25  → Servo Z

const float MOVEMENT_G        = 0.30f;   // gate & snap threshold
const int   SERVO_MIN_ANGLE   = 0;
const int   SERVO_MAX_ANGLE   = 180;
const int   HOME_ANGLE        = 0;
const unsigned long TIMEOUT_MS = 1000;   // silence before homing
// -----------------------------------

struct SensorData { float x, y, z; };

Servo servoX, servoY, servoZ;
unsigned long lastPacketTime = 0;

// simple float mapper
static inline float mapF(float v, float i0, float i1, float o0, float o1) {
  return (v - i0) * (o1 - o0) / (i1 - i0) + o0;
}

// ESP‑NOW receive callback
void onReceive(const esp_now_recv_info_t*, const uint8_t* data, int len) {
  if (len != sizeof(SensorData)) return;

  SensorData d;
  memcpy(&d, data, sizeof(d));
  lastPacketTime = millis();

  // If motion is tiny, park servos immediately
  if (fabs(d.x) < MOVEMENT_G &&
      fabs(d.y) < MOVEMENT_G &&
      fabs(d.z) < MOVEMENT_G) {
    servoX.write(HOME_ANGLE);
    servoY.write(HOME_ANGLE);
    servoZ.write(HOME_ANGLE);
    return;
  }

  int ax = constrain((int)mapF(d.x, -1, 1, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE),
                     SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  int ay = constrain((int)mapF(d.y, -1, 1, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE),
                     SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  int az = constrain((int)mapF(d.z, -1, 1, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE),
                     SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  servoX.write(ax);
  servoY.write(ay);
  servoZ.write(az);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Force radio onto fixed channel
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  // Attach servos and home them
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);
  servoZ.attach(SERVO_Z_PIN);
  servoX.write(HOME_ANGLE);
  servoY.write(HOME_ANGLE);
  servoZ.write(HOME_ANGLE);

  // Start ESP‑NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP‑NOW init failed");
    while (true);
  }
  esp_now_register_recv_cb(onReceive);

  lastPacketTime = millis();
  Serial.println("Beacon ready (channel 1, servos 27/26/25)");
}

void loop() {
  // Auto‑home if no packets recently
  if (millis() - lastPacketTime > TIMEOUT_MS) {
    servoX.write(HOME_ANGLE);
    servoY.write(HOME_ANGLE);
    servoZ.write(HOME_ANGLE);
  }
}
