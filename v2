#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

// Servo pins
const int servoPins[6] = {3, 5, 6, 9, 10, 11};

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
  servo1.attach(servoPins[0]);
  servo2.attach(servoPins[1]);
  servo3.attach(servoPins[2]);
  servo4.attach(servoPins[3]);
  servo5.attach(servoPins[4]);
  servo6.attach(servoPins[5]);
  
  Serial.println("Setup complete.");
}

void loop() {
  int16_t ax, ay, az;
  
  // Read acceleration data from MPU6050
  mpu.getAcceleration(&ax, &ay, &az);
  
  // Map acceleration data to servo angles (0-180 degrees)
  int angleX = map(ax, -17000, 17000, 0, 180);
  int angleY = map(ay, -17000, 17000, 0, 180);
  int angleZ = map(az, -17000, 17000, 0, 180);
  
  // Print raw accelerometer data and mapped angles for debugging
  Serial.print("AX: "); Serial.print(ax); 
  Serial.print(" AY: "); Serial.print(ay); 
  Serial.print(" AZ: "); Serial.print(az);
  Serial.print(" | AngleX: "); Serial.print(angleX);
  Serial.print(" AngleY: "); Serial.print(angleY);
  Serial.print(" AngleZ: "); Serial.println(angleZ);
  
  // Control servos based on mapped angles
  servo1.write(angleX);
  servo2.write(angleY);
  servo3.write(angleZ);
  
  // Assign other servos with combinations or variations if needed
  servo4.write((angleX + angleY) / 2);
  servo5.write((angleY + angleZ) / 2);
  servo6.write((angleZ + angleX) / 2);

  delay(100);  // Small delay to stabilize movement
}
