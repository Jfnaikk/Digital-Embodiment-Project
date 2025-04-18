#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;

Servo servos[6];

// Servo pins
const int servoPins[6] = {3, 5, 6, 9, 10, 11};

// Angle limits to protect servos
const int minAngle = 30;
const int maxAngle = 150;

// Previous acceleration values to detect movement speed
int16_t prevAx = 0, prevAy = 0, prevAz = 0;

// Movement thresholds
const int rapidMovementThreshold = 5000;  // Adjust sensitivity if needed
const int fastDelay = 50;   // Delay for rapid movement
const int slowDelay = 150;  // Delay for slow movement

void setup() {
  Serial.begin(9600);
  
  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  
  // Attach servos to respective pins
  for (int i = 0; i < 6; i++) {
    servos[i].attach(servoPins[i]);
  }
  
  Serial.println("Setup complete.");
}

void loop() {
  int16_t ax, ay, az;
  
  // Read acceleration data from MPU6050
  mpu.getAcceleration(&ax, &ay, &az);
  
  // Calculate change in acceleration
  int dA = abs(ax - prevAx) + abs(ay - prevAy) + abs(az - prevAz);
  
  // Map acceleration data to limited servo angles
  int angleX = constrain(map(ax, -17000, 17000, minAngle, maxAngle), minAngle, maxAngle);
  int angleY = constrain(map(ay, -17000, 17000, minAngle, maxAngle), minAngle, maxAngle);
  int angleZ = constrain(map(az, -17000, 17000, minAngle, maxAngle), minAngle, maxAngle);

  // Print raw accelerometer data, mapped angles, and dA for debugging
  Serial.print("AX: "); Serial.print(ax); 
  Serial.print(" AY: "); Serial.print(ay); 
  Serial.print(" AZ: "); Serial.print(az);
  Serial.print(" | AngleX: "); Serial.print(angleX);
  Serial.print(" AngleY: "); Serial.print(angleY);
  Serial.print(" AngleZ: "); Serial.print(angleZ);
  Serial.print(" | dA: "); Serial.println(dA);

  // Move servos directly according to mapped angles
  servos[0].write(angleX);
  servos[1].write(angleY);
  servos[2].write(angleZ);
  
  // Assign other servos with combinations of angles
  servos[3].write((angleX + angleY) / 2);
  servos[4].write((angleY + angleZ) / 2);
  servos[5].write((angleZ + angleX) / 2);

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
