#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

#define SERVO_X_PIN 4
#define SERVO_Y_PIN 5
#define SERVO_Z_PIN 6

Servo servoX, servoY, servoZ;

const int   SERVO_MIN_ANGLE    = 0;
const int   SERVO_MAX_ANGLE    = 180;
const float MOVEMENT_THRESHOLD = 0.5f;
const int   SERVO_HOME_ANGLE   = 0;      // 0° on disconnect
const unsigned long TIMEOUT_MS = 1000;   // 1 second timeout

struct SensorData {
    float x, y, z;
};

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// tracks last valid packet time
unsigned long lastRecvTime = 0;

void onDataRecv(const esp_now_recv_info_t *, const uint8_t *incomingData, int len) {
    if (len != sizeof(SensorData)) return;

    // reset disconnect timer
    lastRecvTime = millis();

    SensorData receivedData;
    memcpy(&receivedData, incomingData, sizeof(receivedData));

    // only drive servos when movement exceeds threshold
    if (abs(receivedData.x) > MOVEMENT_THRESHOLD ||
        abs(receivedData.y) > MOVEMENT_THRESHOLD ||
        abs(receivedData.z) > MOVEMENT_THRESHOLD) {

        int angleX = constrain(
            mapFloat(receivedData.x, -10, 10, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE),
            SERVO_MIN_ANGLE, SERVO_MAX_ANGLE
        );
        int angleY = constrain(
            mapFloat(receivedData.y, -10, 10, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE),
            SERVO_MIN_ANGLE, SERVO_MAX_ANGLE
        );
        int angleZ = constrain(
            mapFloat(receivedData.z, -10, 10, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE),
            SERVO_MIN_ANGLE, SERVO_MAX_ANGLE
        );

        servoX.write(angleX);
        servoY.write(angleY);
        servoZ.write(angleZ);
    }
}

void setup() {
    WiFi.mode(WIFI_STA);

    servoX.attach(SERVO_X_PIN);
    servoY.attach(SERVO_Y_PIN);
    servoZ.attach(SERVO_Z_PIN);

    esp_now_init();
    esp_now_register_recv_cb(onDataRecv);

    // initialize timer so we don't immediately timeout
    lastRecvTime = millis();
}

void loop() {
    // if no packet received for >1 s, send home position
    if (millis() - lastRecvTime > TIMEOUT_MS) {
        servoX.write(SERVO_HOME_ANGLE);
        servoY.write(SERVO_HOME_ANGLE);
        servoZ.write(SERVO_HOME_ANGLE);
    }
    // otherwise, let onDataRecv() drive the servos
}
