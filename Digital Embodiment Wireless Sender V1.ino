#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

#define MPU_SDA 6
#define MPU_SCL 7
#define LED_PIN 8  
#define NUM_PIXELS 1  

Adafruit_NeoPixel rgbLED(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_MPU6050 mpu;

uint8_t receiverMac[] = {0x40, 0x4C, 0xCA, 0x55, 0xD7, 0x20};  

struct SensorData {
    float x, y, z;
};

const float MOVEMENT_THRESHOLD = 0.5;  

void blinkGreenLED(int times, int delayMs) {
    for (int i = 0; i < times; i++) {
        rgbLED.setPixelColor(0, rgbLED.Color(0, 255, 0));  
        rgbLED.show();
        delay(delayMs);
        rgbLED.setPixelColor(0, 0);  
        rgbLED.show();
        delay(delayMs);
    }
}

void onDataSent(const uint8_t *macAddr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        blinkGreenLED(1, 200);
    }
}

void setup() {
    WiFi.mode(WIFI_STA);
    Wire.begin(MPU_SDA, MPU_SCL);
    rgbLED.begin();
    rgbLED.clear();
    rgbLED.show();

    if (!mpu.begin()) {
        while (1);
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    if (esp_now_init() != ESP_OK) {
        return;
    }

    esp_now_register_send_cb(onDataSent);
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, receiverMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
}

void loop() {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    SensorData dataToSend = {accel.acceleration.x, accel.acceleration.y, accel.acceleration.z};

    if (abs(dataToSend.x) > MOVEMENT_THRESHOLD || abs(dataToSend.y) > MOVEMENT_THRESHOLD || abs(dataToSend.z) > MOVEMENT_THRESHOLD) {
        esp_now_send(receiverMac, (uint8_t *)&dataToSend, sizeof(dataToSend));
    }

    delay(50);
}
