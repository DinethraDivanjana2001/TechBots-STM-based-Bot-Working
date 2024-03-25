#include <Servo.h>

//150 25

Servo servo1;  // servo object for servo connected to pin D1
Servo servo2;  // servo object for servo connected to pin D2
Servo servo3;  // servo object for servo connected to pin D3


int servoNum;
int angle;

void setup() {
  // Attach servos to their respective pins
  servo1.attach(9);
  servo2.attach(10);
  servo3.attach(11);

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // servo2.writeMicroseconds(400);
  if (Serial.available() > 0) {
    // Read the input from Serial Monitor
    String input = Serial.readStringUntil('\n');

    // Parse the input to get servo number and angle
    servoNum = input.substring(0, 1).toInt();
    angle = input.substring(2).toInt();

    // Move the corresponding servo to the specified angle if (servoNum == 1) {
    // servo1.write(angle);
    if (servoNum == 1) {
      servo1.write(angle);
    } else if (servoNum == 2) {
      servo2.write(angle);
    } else if (servoNum == 3) {
      servo3.write(angle);
    }
  }
}