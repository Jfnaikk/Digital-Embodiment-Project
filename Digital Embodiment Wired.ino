// Author: Jawad Farooq Naik
// Date Created: January 28, 2025
// Location: Boston, MA, USA
// Project: Digital Embodiment
// Institution: Massachusetts College of Art and Design
// Department: Dynamic Media Institute
// Project Description:
// Digital Embodiment is an interactive project designed to bridge the emotional gap caused by long-distance relationships
// by simulating the physical presence of a person away from home. The system consists of two key components: a sensing band
// and a homing beacon. The sensing band is worn by the person who is away from home and detects their motion data across
// the X, Y, and Z axes. This data is transmitted over the internet to the homing beacon, which is placed in the home of
// their loved ones. The homing beacon features three rotating cubes that respond to the received motion data. Positioned
// in front of these cubes is a horizontal line inspired by a heart rate monitor. When motion is detected, the cubes rotate,
// causing the line to break—symbolizing movement and presence. Conversely, when no motion is detected, the cubes align,
// forming an unbroken line that represents stillness and the absence of activity, evoking the visual metaphor of a flatline.
// By translating motion into visual and kinetic feedback, Digital Embodiment offers a meaningful way to maintain a sense of
// presence and connection across physical distances.
//
// Features:
// - Three servos(pins 3, 5, and 10) controlled via MPU6050 Accelerometer sensor(pins SDA/SCL).
// - Homing feature: If no motion is detected, servos return to 0°.
// - Dynamic speed adjustment based on acceleration changes.
// - Auto-recovery: If MPU6050 disconnects, the system attempts to reset it.
//
// Notes:
// - Ensure servos are powered externally (5–6V) to prevent Arduino power issues.
// - This project investigates embodied interaction and tangible computing.

// —— Library Includes —— 
#include <Wire.h>             // I2C communication
#include <MPU6050.h>          // MPU6050 driver
#include <Servo.h>            // Servo control

// —— Object & Array Declarations —— 
MPU6050 mpu;                   // MPU6050 instance
Servo   servos[3];             // Array managing three servos

// —— Pin Definitions —— 
const int servoPins[3] = {3, 5, 10};  
  // servos[0] → D3, servos[1] → D5, servos[2] → D10

// —— Servo Motion Limits —— 
const int minAngle = 30;       // Minimum servo angle
const int maxAngle = 150;      // Maximum servo angle

// —— Motion Detection Thresholds —— 
const int noMotionThreshold      = 500;   // Sum of Δaccel below ⇒ no motion
const int rapidMovementThreshold = 5000;  // Sum of Δaccel above ⇒ rapid movement

// —— Response Speed Settings —— 
const int fastDelay = 30;      // ms delay when movement is rapid
const int slowDelay = 100;     // ms delay when movement is slow

// —— Previous Acceleration Storage —— 
int16_t prevAx = 0, prevAy = 0, prevAz = 0;

// —— Calibration Offsets —— 
int16_t offsetAx = 0, offsetAy = 0, offsetAz = 0;

// —— MPU6050 Connection Flag —— 
bool mpuConnected = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();                // Initialize I2C bus

  // —— Initialize MPU6050 and verify connection —— 
  mpu.initialize();
  mpuConnected = mpu.testConnection();
  if (!mpuConnected) {
    Serial.println("MPU6050 connection failed! Retrying...");
  } else {
    Serial.println("MPU6050 connected.");
    calibrateMPU();            // Perform offset calibration
  }

  // —— Attach each servo and center at 90° —— 
  for (int i = 0; i < 3; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(90);
  }

  Serial.println("Setup complete.");
}

// —— Calibrate the MPU6050 ——
// Reads 100 samples to compute average offsets for X, Y, Z axes.
void calibrateMPU() {
  int16_t ax, ay, az;
  Serial.println("Calibrating MPU6050...");
  for (int i = 0; i < 100; i++) {
    if (!mpuConnected) return;           // Abort if disconnected
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

// —— Attempt to Reset MPU6050 on Disconnect ——
// Reinitializes I2C and re-calibrates if reconnection succeeds.
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
  // —— If MPU disconnected, keep retrying reset —— 
  if (!mpuConnected) {
    resetMPU6050();
    delay(500);
    return;
  }

  // —— 1) Read raw acceleration values —— 
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // —— 2) Apply calibration offsets —— 
  ax -= offsetAx;
  ay -= offsetAy;
  az -= offsetAz;

  // —— 3) Compute total change in acceleration —— 
  int dA = abs(ax - prevAx) + abs(ay - prevAy) + abs(az - prevAz);

  // —— 4) No‑motion homing: move servos to 0° if below threshold —— 
  if (dA < noMotionThreshold) {
    for (int i = 0; i < 3; i++) {
      servos[i].write(0);
    }
    Serial.println("No motion detected. Servos homing to 0°.");
  } 
  else {
    // —— 5) Map acceleration to servo angles —— 
    int angleX = constrain(map(ax, -17000, 17000, minAngle, maxAngle), minAngle, maxAngle);
    int angleY = constrain(map(ay, -17000, 17000, minAngle, maxAngle), minAngle, maxAngle);
    int angleZ = constrain(map(az, -17000, 17000, minAngle, maxAngle), minAngle, maxAngle);

    // —— 6) Apply mapped angles to servos —— 
    servos[0].write(angleX);
    servos[1].write(angleY);
    servos[2].write(angleZ);

    // —— 7) Debug output of sensor and computed Δ —— 
    Serial.print("AX: "); Serial.print(ax);
    Serial.print(" AY: "); Serial.print(ay);
    Serial.print(" AZ: "); Serial.print(az);
    Serial.print(" | dA: "); Serial.println(dA);
  }

  // —— 8) Choose response speed based on movement intensity —— 
  if (dA > rapidMovementThreshold) {
    delay(fastDelay);
  } else {
    delay(slowDelay);
  }

  // —— 9) Store current acceleration for next Δ calculation —— 
  prevAx = ax;
  prevAy = ay;
  prevAz = az;
}

  prevAy = ay;
  prevAz = az;
}
