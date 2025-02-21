#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>

#define SERVO_X_PIN 4
#define SERVO_Y_PIN 5
#define SERVO_Z_PIN 6
#define LED_PIN 8  
#define NUM_PIXELS 1  

Adafruit_NeoPixel rgbLED(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
Servo servoX, servoY, servoZ;

const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;
const float MOVEMENT_THRESHOLD = 0.5;

struct SensorData {
    float x, y, z;
};

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

void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
    SensorData receivedData;
    memcpy(&receivedData, incomingData, sizeof(receivedData));

    if (abs(receivedData.x) > MOVEMENT_THRESHOLD || abs(receivedData.y) > MOVEMENT_THRESHOLD || abs(receivedData.z) > MOVEMENT_THRESHOLD) {
        int angleX = map(receivedData.x, -10, 10, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        int angleY = map(receivedData.y, -10, 10, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        int angleZ = map(receivedData.z, -10, 10, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

        servoX.write(angleX);
        servoY.write(angleY);
        servoZ.write(angleZ);
        blinkGreenLED(1, 200);
    } else {
        servoX.write(0);
        servoY.write(0);
        servoZ.write(0);
    }
}

void setup() {
    WiFi.mode(WIFI_STA);
    rgbLED.begin();
    rgbLED.clear();
    rgbLED.show();

    servoX.attach(SERVO_X_PIN);
    servoY.attach(SERVO_Y_PIN);
    servoZ.attach(SERVO_Z_PIN);

    if (esp_now_init() != ESP_OK) {
        return;
    }

    esp_now_register_recv_cb(onDataRecv);
}

void loop() {
}
