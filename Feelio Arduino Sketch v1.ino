//Arduino Sketch code by: Jawad Farooq Naik
//Date: Tuesday, Nov 5 2024
//Location: Boston, MA, USA 
//Project: Feelio Installation. (DMI Thesis)
//Hardware: 4servos, PIR Sensor, Arduino UNO

#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

const int pirPin = 12;          // PIR sensor pin
const int delayAfterMotion = 0; // Delay after motion detected (1 minute)

void setup() {
    Serial.begin(9600); // Start serial communication for debugging

    // Attach each servo to its corresponding pin
    servo1.attach(3);
    servo2.attach(11);
    servo3.attach(6);
    servo4.attach(9);

    pinMode(pirPin, INPUT); // Set PIR sensor pin as input

    Serial.println("System initialized. Waiting for motion...");
}

void loop() {
    // Read the PIR sensor state
    int motionDetected = digitalRead(pirPin);

    if (motionDetected == HIGH) { // If motion is detected
        Serial.println("Motion detected! Moving servos...");

        // Move all servos in varied patterns
        for (int angle = 0; angle <= 180; angle += 10) {
            servo1.write(angle);          // Servo 1 sweeps from 0 to 180
            servo2.write(angle);    // Servo 2 moves opposite of Servo 1
            servo3.write(angle);      // Servo 3 has a slower movement range
            servo4.write(angle); // Servo 4 has an offset start position
            delay(30); // Adjust the delay to control servo speed
        }

        // Hold all servos briefly
        delay(500);

        // Move all servos back to 0 in varied patterns
        for (int angle = 180; angle >= 0; angle -= 10) {
            servo1.write(angle);          // Servo 1 sweeps back from 180 to 0
            servo2.write(angle);    // Servo 2 moves opposite of Servo 1
            servo3.write(angle);      // Servo 3 continues its slower movement
            servo4.write(angle); // Servo 4 returns to starting offset
            delay(30); // Adjust the delay to control servo speed
        }

        Serial.println("Motion sequence complete. Waiting 1 minute...");

        // Wait 1 minute (60,000 milliseconds) before checking for motion again
        delay(delayAfterMotion);

        Serial.println("Resuming motion detection...");
    }
}
