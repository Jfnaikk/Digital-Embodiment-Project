#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <esp_now.h>
#include <WiFi.h>

#define MPU_SDA 6
#define MPU_SCL 7

Adafruit_MPU6050 mpu;

uint8_t receiverMac[] = {0x40, 0x4C, 0xCA, 0x55, 0xD7, 0x20};

struct SensorData {
    float x, y, z;
};

const float MOVEMENT_THRESHOLD = 0.5;
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 10;  // ms

void setup() {
    WiFi.mode(WIFI_STA);
    Wire.begin(MPU_SDA, MPU_SCL);

    mpu.begin();
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    esp_now_init();
    esp_now_peer_info_t peerInfo{};
    memcpy(peerInfo.peer_addr, receiverMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
}

void loop() {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    SensorData dataToSend = {accel.acceleration.x, accel.acceleration.y, accel.acceleration.z};

    if (abs(dataToSend.x) > MOVEMENT_THRESHOLD ||
        abs(dataToSend.y) > MOVEMENT_THRESHOLD ||
        abs(dataToSend.z) > MOVEMENT_THRESHOLD) {
        
        unsigned long now = millis();
        if (now - lastSendTime > SEND_INTERVAL) {
            esp_now_send(receiverMac, (uint8_t *)&dataToSend, sizeof(dataToSend));
            lastSendTime = now;
        }
    }
}
