// For controlling the servo motor
#include <Servo.h> 

// Pre existing filter libraries
#include <Filters.h> 
#include <AH/Timing/MillisMicrosTimer.hpp>
#include <Filters/Butterworth.hpp>
#include <Filters/SMA.hpp>
#include <AH/STL/cmath>

// Init a Servo object
Servo motor;

const int redPin(10);
const int greenPin(11);
bool red_ledState=false;
bool green_ledState=false;
unsigned long now_millis(0);

void setup() {
  Serial1.begin(115200);
  computeNotchCoeffs(); // <- added
  motor.attach(2);  // Corresponds to Servo 1
  pinMode(redPin,OUTPUT);
  pinMode(greenPin,OUTPUT);
}

int emgSig[3] = {0, 0, 0};
char buffer[50];

// --- Notch filter & smoothing config ---
const float fs = 1000.0;
const float notchFreq = 50.0;
const float Q = 30.0;

float b0, b1, b2, a0, a1, a2;
float x_hist[2][3] = {0}, y_hist[2][3] = {0};

const int winSize = 10;
float smoothWin[2][winSize] = {0};
int winIdx = 0;

// --- Filters config ----
Timer<micros> timer = std::round(1e6 / fs);
const double f_low = 10.0;          // lower cutoff
const double f_high = 200.0;         // upper cutoff
const double f_low_n = 2 * f_low / fs;
const double f_high_n = 2 * f_high / fs;
SMA<10> moving_average0 = {512};
SMA<10> moving_average1 = {512};
auto butterhigh0 = butter<1>(f_high_n);
auto butterhigh1 = butter<1>(f_high_n);
auto butterlow0 = butter<1>(f_low_n);
auto butterlow1 = butter<1>(f_low_n);
float notched0, notched1;

// --- Direction and threshold config ---
const float threshold = 25.0;
int direction = 0; // +1, -1, or 0

void computeNotchCoeffs() {
  float w0 = 2.0 * PI * notchFreq / fs;
  float alpha = sin(w0) / (2.0 * Q);
  b0 = 1;
  b1 = -2 * cos(w0);
  b2 = 1;
  a0 = 1 + alpha;
  a1 = -2 * cos(w0);
  a2 = 1 - alpha;
}

float notchFilter(int ch, float x) {
  x_hist[ch][2] = x_hist[ch][1];
  x_hist[ch][1] = x_hist[ch][0];
  x_hist[ch][0] = x;

  y_hist[ch][2] = y_hist[ch][1];
  y_hist[ch][1] = y_hist[ch][0];

  y_hist[ch][0] = (b0/a0)*x_hist[ch][0] + (b1/a0)*x_hist[ch][1] + (b2/a0)*x_hist[ch][2]
                - (a1/a0)*y_hist[ch][1] - (a2/a0)*y_hist[ch][2];

  return y_hist[ch][0];
}

float movingAverage(int ch, float val) {
  smoothWin[ch][winIdx] = val;
  float sum = 0;
  for (int i = 0; i < winSize; i++) sum += smoothWin[ch][i];
  return sum / winSize;
}

void loop() {
  if (timer)
    // Unpack Raw EMG Signals
    emgSig[0] = analogRead(A0) - 512;
    emgSig[1] = analogRead(A1) - 512;
    // emgSig[2] = analogRead(A4);
    // if (timer)
    float notched0 = notchFilter(0, emgSig[0]);
    float notched1 = notchFilter(1, emgSig[1]);

    float buttered0 = butterhigh0(butterlow0(notched0));
    float buttered1 = butterhigh1(butterlow1(notched1));

    // float smoothed0 = moving_average0(buttered0);
    // float smoothed1 = moving_average1(buttered1);

    float smoothed0 = 23*movingAverage(0,abs(buttered0));
    float smoothed1 = 10*movingAverage(1,abs(buttered1));

    if (smoothed0 > threshold || smoothed1 > threshold) {
    if (smoothed0 > smoothed1) {
        direction = +1; // e.g., flexion
      } else {
        direction = -1; // e.g., extension
      }
    } else {
      direction = 0; // No voluntary contraction
    }

    now_millis = millis();
    if (direction == +1) {
      motor.write(150);  // Adjust for forward spin
      digitalWrite(greenPin,HIGH);
      digitalWrite(redPin,LOW);
    } else if (direction == -1) {
      motor.write(50);   // Adjust for reverse spin
      digitalWrite(redPin,HIGH);
      digitalWrite(greenPin,LOW);
    }
    Serial1.print("rawA0:");
    Serial1.print(emgSig[0]);
    Serial1.print(",");
    Serial1.print("rawA1:");
    Serial1.print(emgSig[1]);
    Serial1.print("\t");

    Serial1.print("notchA0:");
    Serial1.print(notched0, 1);
    Serial1.print(",");
    Serial1.print("notchA1:");
    Serial1.print(notched1, 1);
    Serial1.print("\t");

    Serial1.print("butA0:");
    Serial1.print(buttered0, 1);
    Serial1.print(",");
    Serial1.print("butA1:");
    Serial1.print(buttered1, 1);
    Serial1.print("\t");

    Serial1.print("envA0:");
    Serial1.print(smoothed0, 1);
    Serial1.print(",");
    Serial1.print("envA1:");
    Serial1.print(smoothed1, 1);
    Serial1.print("\t");
    Serial1.print("Dir:");
    Serial1.print(direction);

    Serial1.print("\t");
    Serial1.print("Fix:");
    Serial1.println(100);

  
    winIdx = (winIdx + 1) % winSize;
}
