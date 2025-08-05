/*
  Course:   Bionics and Wearable Robotics 0360108 
  Authors:  Elad Siman Tov, Eitan Gerber
  Date:     August 5th, 2025
*/
#include <Servo.h>
Servo indexFinger, middleFinger, ringFinger, pinkyFinger, thumb, twistWrist;

void setup() {
  indexFinger.attach(2);  // Servo_1
  middleFinger.attach(3); // Servo_2
  ringFinger.attach(4);   // Servo_3
  pinkyFinger.attach(5);  // Servo_4
  thumb.attach(6);        // Servo_5
  twistWrist.attach(9);   // Servo_6

  // Move to resting position
  indexFinger.write(0);  // Servo_1 == top right servo
  middleFinger.write(0); // Servo_2 == top middle servo
  ringFinger.write(0);   // Servo_3 == top left servo
  pinkyFinger.write(0);  // Servo_4 == bottom left servo
  thumb.write(0);        // Servo_5 == bottom right servo
  twistWrist.write(90);  // Servo_6

  // Do this for the hand supination / pronation motor: it is already at 90 so immidiately position at 90!
  // motor.write(90); // Immidiately move to the current physical angle position to avoid it to reset to 90 during init process
  // Do this for the rest of the motors which are positioned at 0.
  // Immidiately move to the current physical angle position to avoid it to reset to 90 during init process
  Serial.begin(115200);
}

void loop() {
  // Define Hardcoded Ranges of Motion per finger.
  // Pinky  ROM: 0-80 (bottom left servo)
  // Ring   ROM: 0-80 (top left servo)
  // Middle ROM: 0-90 (top middle servo)
  // Index  ROM: 0-90 (top right servo)
  // Thumb  ROM: 0-45 (bottom right servo)

  // Read the potentiometer from A0. If you want to use Hall sensor it is A9 but use 3.3V to avoid saturating.
  int potValue = analogRead(A0);

  int indexAngle = map(potValue, 0, 1023, 0, 90); // scale to ROM of index finger
  int middleAngle = map(potValue, 0, 1023, 0, 90);
  int ringAngle = map(potValue, 0, 1023, 0, 80);
  int pinkyAngle = map(potValue, 0, 1023, 0, 80);
  int thumbAngle = map(potValue, 0, 1023, 0, 45);
  int wristAngle = map(potValue, 0, 1023, 40, 140); // scale to mid range of 90 and range of 100.
  
  // Write command to all motors
  indexFinger.write(indexAngle);  // Servo_1 == top right servo
  middleFinger.write(middleAngle); // Servo_2 == top middle servo
  ringFinger.write(ringAngle);   // Servo_3 == top left servo
  pinkyFinger.write(pinkyAngle);  // Servo_4 == bottom left servo
  thumb.write(thumbAngle);        // Servo_5 == bottom right servo
  twistWrist.write(wristAngle);  // Servo_6

  Serial.print("index:"); Serial.print(indexAngle); Serial.print(",");
  Serial.print("middle:"); Serial.print(middleAngle); Serial.print(",");
  Serial.print("ring:"); Serial.print(ringAngle); Serial.print(",");
  Serial.print("pinky:"); Serial.print(pinkyAngle); Serial.print(",");
  Serial.print("thumb:"); Serial.print(thumbAngle); Serial.print(",");
  Serial.print("wrist:"); Serial.println(wristAngle);

  delay(100);
}