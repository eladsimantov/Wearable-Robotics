/*
  Course:   Bionics and Wearable Robotics 0360108 
  Authors:  Elad Siman Tov, Eitan Gerber
  Date:     August 5th, 2025
*/



// ---------------- Servo setup ---------------- //
#include <Servo.h>
Servo indexFinger, middleFinger, ringFinger, pinkyFinger, thumb, twistWrist;
// double label = 0;

// Structure to hold one complete grasp profile
struct GraspProfile {
  int thumbAngle;
  int indexAngle;
  int middleAngle;
  int ringAngle;
  int pinkyAngle;
  int wristAngle;
};

// Define 6 different grasp profiles and an initial rest position.
// Replace these with your actual desired servo angles for each grasp.
const GraspProfile graspProfiles[7] = {
  // Index 0: Resting position
  {0, 0, 0, 0, 0, 90},
  // Index 1: Grasp 1 - Adducted Thumb
  {10, 90, 90, 85, 85, 90 + 45},
  // Index 2: Grasp 2 - LArge Diameter
  {40, 50, 50, 50, 50, 90},
  // Index 3: Grasp 3 - Medium Wrap
  {50, 70, 70, 70, 70, 90},
  // Index 4: Grasp 4 - Ring
  {50, 85, 0, 0, 0, 90},
  // Index 5: Grasp 5 - Ventral
  {0, 0, 90, 85, 85, 90 + 45},
  // Index 6: Grasp 6 - Rude
  {60, 90, 0, 85, 85, 90 - 45}
};


// ---------------- Timer setup ---------------- //
#include "TeensyTimerTool.h"
using namespace TeensyTimerTool;
PeriodicTimer emg_sample_timer;
const int us_InSecond = 1000000;
const int requiredSampleFrequencyHz = 50;
const int emgTimerSleepPeriod_us = us_InSecond / requiredSampleFrequencyHz; 
unsigned long lastPredict = 0;
unsigned long lastMotor = 0;
const unsigned long predictInterval_ms = 10;
const unsigned long motorInterval_ms = 2500;

// ---------------- Windowing setup ---------------- //
const int window_ms = 800;
const int fs = requiredSampleFrequencyHz;
const int window_samples = (fs * window_ms) / 1000; 

// A single set of circular buffers
volatile uint16_t bufA0[window_samples];
volatile uint16_t bufA1[window_samples];
volatile uint16_t bufA4[window_samples];

// The single variable that tracks the buffer position
volatile int sample_index = 0; 

// -------------------------- Classifier setup -------------------- //
// NOTE YOU MUST INSTALL OUR .ZIP PACKAGE TO YOUR IDE TO INCLUDE ALL THESE FUNCTIONS.
#include <CholeskyDiscriminant.h>
#include <CompactClassificationDiscriminant.h>
#include <predictGraspLD.h>
#include <predictGraspLD_data.h>
#include <predictGraspLD_initialize.h>
#include <predictGraspLD_terminate.h>
#include <predictGraspLD_types.h>
#include <rtGetInf.h>
#include <rtGetNaN.h>
#include <rt_nonfinite.h>
#include <rtwtypes.h>


void setup() {
  // Attach the Servos
  indexFinger.attach(2);  // Servo_1
  middleFinger.attach(3); // Servo_2
  ringFinger.attach(4);   // Servo_3
  pinkyFinger.attach(5);  // Servo_4
  thumb.attach(6);        // Servo_5
  twistWrist.attach(9);   // Servo_6

  // Move to resting position at start
  thumb.write(0);        // Servo_5 == bottom right servo
  indexFinger.write(0);  // Servo_1 == top right servo
  middleFinger.write(0); // Servo_2 == top middle servo
  ringFinger.write(0);   // Servo_3 == top left servo
  pinkyFinger.write(0);  // Servo_4 == bottom left servo
  twistWrist.write(90);  // Servo_6

  emg_sample_timer.begin([] { ISR_callback_readEmg(); }, emgTimerSleepPeriod_us);
  Serial.begin(115200);
  Serial1.begin(115200);
  predictGraspLD_initialize();
}

void loop() {
  // The Hardcoded Ranges of Motion per finger are defined here:
  // Pinky  ROM: 0-80 (bottom left servo)
  // Ring   ROM: 0-80 (top left servo)
  // Middle ROM: 0-90 (top middle servo)
  // Index  ROM: 0-90 (top right servo)
  // Thumb  ROM: 0-45 (bottom right servo)


  unsigned long now = millis();

  // Array to store labels predicted within the motorInterval
  static double labelHistory[30]; 
  static int historyIndex = 0;

  if (now - lastPredict >= predictInterval_ms) {
    lastPredict = now;
    double features[18];
    extractFeatures(features);
    double label = predictGraspLD(features);

    // Store the exact predicted label
    labelHistory[historyIndex] = label;
    historyIndex = (historyIndex + 1) % 30; // 30 is the number of labels predicted in 3 seconds (3000ms / 100ms)

    // Serial.println(label);
    // Serial1.println(label);
  }

  if (now - lastMotor >= motorInterval_ms) {
    lastMotor = now;

    // Find the most frequent label (the mode) in the last 3 seconds
    // A more robust way to handle floating-point values for mode
    double modeLabel = 0;
    int maxCount = 0;

    for (int i = 0; i < 30; ++i) {
      int count = 1;
      for (int j = i + 1; j < 30; ++j) {
        if (labelHistory[i] == labelHistory[j]) {
          count++;
        }
      }
      if (count > maxCount) {
        maxCount = count;
        modeLabel = labelHistory[i];
      }
    }

    // Convert the mode label to an integer for the grasp profile index (_.5 mapped to 0)
    int g = (int)modeLabel;
    int label = (int)modeLabel;
    if (label >= 0 && label <= 6 && (label - (int)label == 0)) g = (int)label; // integers 0–6
    else if (label >= 1.5 && label <= 6.5 && (label - (int)label == 0.5)) g = 0; // .5 values -> rest
    Serial1.println(g);
    // Serial1.println(label);
    thumb.write(graspProfiles[g].thumbAngle);
    indexFinger.write(graspProfiles[g].indexAngle);
    middleFinger.write(graspProfiles[g].middleAngle);
    ringFinger.write(graspProfiles[g].ringAngle);
    pinkyFinger.write(graspProfiles[g].pinkyAngle);
    twistWrist.write(graspProfiles[g].wristAngle);
  }
}

void ISR_callback_readEmg() {
  bufA0[sample_index] = analogRead(A0);
  bufA1[sample_index] = analogRead(A1);
  bufA4[sample_index] = analogRead(A4);
  sample_index = (sample_index + 1) % window_samples;
}

void extractFeatures(double *feat) {
  noInterrupts();
  // Directly process the circular buffers
  computeChannelFeatures(bufA0, window_samples, &feat[0]);
  computeChannelFeatures(bufA1, window_samples, &feat[6]);
  computeChannelFeatures(bufA4, window_samples, &feat[12]);
  interrupts();
}

void computeChannelFeatures(volatile uint16_t *x, int N, double *out) {
  double sum = 0, sumSq = 0, peak = 0;
  for (int i = 0; i < N; i++) {
    double v = (double)x[i];
    sum += v;
    sumSq += v * v;
    if (v > peak) {
      peak = v;
    }
  }
  double mean = sum / N;
  double rms = sqrt(sumSq / N);
  double stdv = sqrt(sumSq / N - mean * mean);
  double shape = rms / mean;
  double crest = peak / rms;
  out[0] = mean; out[1] = rms; out[2] = stdv; out[3] = peak; out[4] = shape; out[5] = crest;
}

