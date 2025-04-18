/* 
 * Date: January 28, 2025
 * Author: Jawad F. Naik
 * Class/Professor: Elements of Media / Fred Wolflink
 * Institution: Massachusetts College of Art and Design
 * Location: Boston, Massachusetts, USA
 * 
 * Project: Dynamic Embodiment
 *
 * Description:
 *
 * This program is part of the **Dynamic Embodiment** project, an interactive 
 * tactile system exploring motion-driven physical feedback in remote communication. 
 * The system uses an MPU6050 accelerometer to detect movement and dynamically map 
 * motion data to three servos, providing real-time haptic feedback. 
 * 
 * Features:
 * - Three servos controlled via MPU6050 (pins 3, 5, and 10).
 * - Homing feature: If no motion is detected, servos return to 0°.
 * - Dynamic speed adjustment based on acceleration changes.
 * - Auto-recovery: If MPU6050 disconnects, the system attempts to reset it.
 * 
 * Notes:
 * - Ensure servos are powered externally (5-6V) to prevent Arduino power issues.
 * - This project investigates embodied interaction and tangible computing.
 */

#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo servos[3]; // Using 3 servos

// Servo pins (Only using 3, 5, and 10)
const int servoPins[3] = {3, 5, 10};

// Servo movement limits
const int minAngle = 30;
const int maxAngle = 150;

// Motion detection thresholds
const int noMotionThreshold = 500;   // Threshold for detecting no motion
const int rapidMovementThreshold = 5000;  // Adjust sensitivity if needed
const int fastDelay = 30;   // Delay for rapid movement
const int slowDelay = 100;  // Delay for slow movement

// Previous acceleration values
int16_t prevAx = 0, prevAy = 0, prevAz = 0;

// Calibration offsets
int16_t offsetAx = 0, offsetAy = 0, offsetAz = 0;

// MPU6050 connection status
bool mpuConnected = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize MPU6050
  mpu.initialize();
  mpuConnected = mpu.testConnection();

  if (!mpuConnected) {
    Serial.println("MPU6050 connection failed! Retrying...");
  } else {
    Serial.println("MPU6050 connected.");
    calibrateMPU();
  }

  // Attach servos to pins
  for (int i = 0; i < 3; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(90); // Start at neutral position
  }

  Serial.println("Setup complete.");
}

// Function to calibrate MPU6050
void calibrateMPU() {
  int16_t ax, ay, az;
  Serial.println("Calibrating MPU6050...");
  
  for (int i = 0; i < 100; i++) {
    if (!mpuConnected) return; // Stop calibration if MPU6050 is disconnected
    mpu.getAcceleration(&ax, &ay, &az);
    offsetAx += ax;
    offsetAy += ay;
    offsetAz += az;
    delay(10);
  }

  offsetAx /= 100;
  offsetAy /= 100;
  offsetAz /= 100;
  Serial.println("MPU6050 Calibration Complete.");
}

void resetMPU6050() {
  Serial.println("Resetting MPU6050...");
  Wire.begin();
  mpu.initialize();
  mpuConnected = mpu.testConnection();

  if (mpuConnected) {
    Serial.println("MPU6050 successfully reconnected.");
    calibrateMPU();
  } else {
    Serial.println("MPU6050 reconnection failed.");
  }
}

void loop() {
  if (!mpuConnected) {
    resetMPU6050(); // Try to reset MPU6050 if disconnected
    delay(500);
    return;
  }

  int16_t ax, ay, az;

  // Read acceleration data
  mpu.getAcceleration(&ax, &ay, &az);

  // Apply calibration offsets
  ax -= offsetAx;
  ay -= offsetAy;
  az -= offsetAz;

  // Calculate change in acceleration
  int dA = abs(ax - prevAx) + abs(ay - prevAy) + abs(az - prevAz);

  // Check for no motion
  if (dA < noMotionThreshold) {
    // Move all servos to home position (0 degrees)
    for (int i = 0; i < 3; i++) {
      servos[i].write(0);
    }
    Serial.println("No motion detected. Servos moving to home position.");
  } else {
    // Map acceleration data to servo angles
    int angleX = constrain(map(ax, -17000, 17000, minAngle, maxAngle), minAngle, maxAngle);
    int angleY = constrain(map(ay, -17000, 17000, minAngle, maxAngle), minAngle, maxAngle);
    int angleZ = constrain(map(az, -17000, 17000, minAngle, maxAngle), minAngle, maxAngle);

    // Assign mapped angles to servos
    servos[0].write(angleX); // Servo on pin 3
    servos[1].write(angleY); // Servo on pin 5
    servos[2].write(angleZ); // Servo on pin 10

    // Debugging output
    Serial.print("AX: "); Serial.print(ax);
    Serial.print(" AY: "); Serial.print(ay);
    Serial.print(" AZ: "); Serial.print(az);
    Serial.print(" | dA: "); Serial.println(dA);
  }

  // Adjust delay based on movement speed
  if (dA > rapidMovementThreshold) {
    delay(fastDelay);  // Rapid movement detected; faster response
  } else {
    delay(slowDelay);  // Slow movement detected; slower response
  }

  // Update previous acceleration values
  prevAx = ax;
  prevAy = ay;
  prevAz = az;
}
