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



void setup() {
  // pinMode(H1, OUTPUT);
  // pinMode(H2, OUTPUT);
  // pinMode(H3, OUTPUT);
  // pinMode(H4, OUTPUT);
  // pinMode(H5, OUTPUT);
  Serial.begin(115200);
  
  // set ADC resolution to 12 bits (0–4095) instead of default 10 bits (0–1023)
  analogReadResolution(12);
}

void loop() {
  // Read sensors
  int hallThumb = analogRead(H1);
  int hallIndex = analogRead(H2);
  int hallMiddle = analogRead(H3);
  int hallRing = analogRead(H4);
  int hallPinky = analogRead(H5);

  Serial.print("Thumb:"); Serial.print(hallThumb); Serial.print(",\t");
  Serial.print("Index:"); Serial.print(hallIndex); Serial.print(",\t");
  Serial.print("Middle:"); Serial.print(hallMiddle); Serial.print(",\t");
  Serial.print("Ring:"); Serial.print(hallRing); Serial.print(",\t");
  Serial.print("Pinky:"); Serial.println(hallPinky);

  delay(100);
}