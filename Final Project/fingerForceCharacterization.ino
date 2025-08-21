/*
  Course:   Bionics and Wearable Robotics 0360108 
  Authors:  Elad Siman Tov, Eitan Gerber
  Date:     August 17th, 2025
*/

// mapping PCB pin names to teensyboard pins
#define H1 A9 // thumb
#define H2 A8 // index
#define H3 A7 // middle
#define H4 A6 // ring
#define H5 A5 // pinky

// declaring the necessary servos
#include <Servo.h>
Servo indexFinger, middleFinger, ringFinger, pinkyFinger, thumb, twistWrist;



void setup() {

  // initializing servos
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

  Serial.begin(115200);
  
  // set ADC resolution to 12 bits (0–4095) instead of default 10 bits (0–1023)
  analogReadResolution(12);
}

void loop() {

  // Define Hardcoded Ranges of Motion per finger.
  // Pinky  ROM: 0-80 (bottom left servo)
  // Ring   ROM: 0-80 (top left servo)
  // Middle ROM: 0-90 (top middle servo)
  // Index  ROM: 0-90 (top right servo)
  // Thumb  ROM: 0-65 (bottom right servo)

  // Read sensors
  int hallThumb = analogRead(H1);
  int hallIndex = analogRead(H2);
  int hallMiddle = analogRead(H3);
  int hallRing = analogRead(H4);
  int hallPinky = analogRead(H5);

  // Read the potentiometer from A0
  int potValue = analogRead(A0);


  int thumbAngle = map(potValue, 0, 4095, 0, 65); // scale to ROM of index finger
  int indexAngle = map(potValue, 0, 4095, 0, 90); 
  int middleAngle = map(potValue, 0, 4095, 0, 90);
  int ringAngle = map(potValue, 0, 4095, 0, 85);
  int pinkyAngle = map(potValue, 0, 4095, 0, 85);
  // int wristAngle = map(potValue, 0, 4095, 40, 140); // scale to mid range of 90 and range of 100.
  
  // Write command to servo - comment-out reupload between tests, to allow characterization of different fingers
  // thumb.write(thumbAngle);        // Servo_5 == bottom right servo
  // indexFinger.write(indexAngle);  // Servo_1 == top right servo
  // middleFinger.write(middleAngle); // Servo_2 == top middle servo
  // ringFinger.write(ringAngle);   // Servo_3 == top left servo
  pinkyFinger.write(pinkyAngle);  // Servo_4 == bottom left servo
  // twistWrist.write(wristAngle);  // Servo_6

  // Serial.print("Thumb Servo Angle:"); Serial.print(thumbAngle); Serial.print(",\t");
  // Serial.print("Index Servo Angle, "); Serial.print(indexAngle); Serial.print(",\t");
  // Serial.print("Middle Servo Angle, "); Serial.print(middleAngle); Serial.print(",\t");
  // Serial.print("Ring Servo Angle, "); Serial.print(ringAngle); Serial.print(",\t");
  // Serial.print("Pinky Servo Angle, "); Serial.print(pinkyAngle); Serial.print(",\t");
  
  // Serial.print("Wrist Servo Angle, "); Serial.print(wristAngle); Serial.print(",\t");

  // Serial.print("Pot. Val, "); Serial.print(potValue); Serial.print(",\t");
  // Serial.print("Thumb Hall Sensor Val, "); Serial.println(hallThumb);
  // Serial.print("Index Hall Sensor Val, "); Serial.println(hallIndex);
  // Serial.print("Middle Hall Sensor Val, "); Serial.println(hallMiddle);
  // Serial.print("Ring Hall Sensor Val, "); Serial.println(hallRing);
  Serial.print("Pinky Hall Sensory Val, "); Serial.println(hallPinky);

  delay(50);
}
