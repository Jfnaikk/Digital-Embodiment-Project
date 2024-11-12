// This code simiply tests the servo signal & movements 

#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

// Define servo pins
const int servoPins[6] = {3, 5, 6, 9, 10, 11};

void setup() {
  // Attach each servo to its corresponding pin
  servo1.attach(servoPins[0]);
  servo2.attach(servoPins[1]);
  servo3.attach(servoPins[2]);
  servo4.attach(servoPins[3]);
  servo5.attach(servoPins[4]);
  servo6.attach(servoPins[5]);
  
  Serial.begin(9600);
  Serial.println("Starting servo test...");
}

void loop() {
  // Sweep each servo from 0 to 180 and back to 0
  for (int angle = 0; angle <= 180; angle += 10) {
    servo1.write(angle);
    delay(15);  // Small delay for smooth movement
  }
  for (int angle = 180; angle >= 0; angle -= 10) {
    servo1.write(angle);
    delay(15);
  }
  delay(500);  // Pause before moving to next servo
  
  // Repeat for each servo
  for (int angle = 0; angle <= 180; angle += 10) {
    servo2.write(angle);
    delay(15);
  }
  for (int angle = 180; angle >= 0; angle -= 10) {
    servo2.write(angle);
    delay(15);
  }
  delay(500);
  
  for (int angle = 0; angle <= 180; angle += 10) {
    servo3.write(angle);
    delay(15);
  }
  for (int angle = 180; angle >= 0; angle -= 10) {
    servo3.write(angle);
    delay(15);
  }
  delay(500);
  
  for (int angle = 0; angle <= 180; angle += 10) {
    servo4.write(angle);
    delay(15);
  }
  for (int angle = 180; angle >= 0; angle -= 10) {
    servo4.write(angle);
    delay(15);
  }
  delay(500);
  
  for (int angle = 0; angle <= 180; angle += 10) {
    servo5.write(angle);
    delay(15);
  }
  for (int angle = 180; angle >= 0; angle -= 10) {
    servo5.write(angle);
    delay(15);
  }
  delay(500);
  
  for (int angle = 0; angle <= 180; angle += 10) {
    servo6.write(angle);
    delay(15);
  }
  for (int angle = 180; angle >= 0; angle -= 10) {
    servo6.write(angle);
    delay(15);
  }
  delay(500);
  
  // After completing all, add a long delay before looping
  Serial.println("All servos have been tested.");
  delay(2000);
}
