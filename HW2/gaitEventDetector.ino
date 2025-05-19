#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#define BNO055_SAMPLERATE_DELAY_MS (20)
#define DT (BNO055_SAMPLERATE_DELAY_MS / 1000.0)
#define ALPHA 0.95  // Complementary filter coefficient
#define GREEN_LED_PIN 8
#define RED_LED_PIN 7

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29, &Wire);

float phi_hat = 0.0, theta_hat = 0.0;
float accelFilterAlpha = 0.2;
float accelAmpFiltered = 0.0; 
bool HS = 1;
bool TO = 0; 
bool GYRO_SW = 0;
bool gyroSignPrev = 0;
bool gyroSign = 0;

unsigned long lastEventTime = 0;
unsigned long eventCooldown = 150; // milliseconds between events

void setup(void) {
  Serial.begin(115200);
  // Serial.println("IMU\tComp");
  
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  while (!Serial) delay(10);
  if (!bno.begin()) {
    Serial.print("No BNO055 detected. Check wiring.");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}

void loop(void) {
    // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2

  // Grab the raw gyro and accel data
  imu::Vector<3> gyros_rad_sec = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); // rad/s
  imu::Vector<3> accels_m_s2 = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // m/s^2

  // Compute the euler inertial frame angles using the accelerometer measurements and gravity (roll = phi, pitch = theta, without yaw!)
  // See https://control.asu.edu/Classes/MMAE441/Aircraft/441Lecture9.pdf for details
  float phi_acc = atan2(accels_m_s2.y(), sqrt(accels_m_s2.x()*accels_m_s2.x() + accels_m_s2.z()*accels_m_s2.z()));
  float theta_acc = atan2(-accels_m_s2.x(), sqrt(accels_m_s2.y()*accels_m_s2.y() + accels_m_s2.z()*accels_m_s2.z()));

  // Unpack raw angular velocities from gyro (to transform into Euler angles in fixed inertial frame)
  float roll_vel_body_frame = gyros_rad_sec.x(); // roll in the body frame 
  float pitch_vel_body_frame = gyros_rad_sec.y();
  float yaw_vel_body_frame = gyros_rad_sec.z();

  // Transform Body frame angular velocities into Euler frame angular velocities (Based on previous step angles estimation)
  float phi_dot = roll_vel_body_frame + sin(phi_hat) * tan(theta_hat) * pitch_vel_body_frame + cos(phi_hat) * tan(theta_hat) * yaw_vel_body_frame;
  float theta_dot = cos(phi_hat) * pitch_vel_body_frame - sin(phi_hat) * yaw_vel_body_frame;

  // Apply complementary filter onto angle estimate from accelerometer and from angular velocity integration.
  phi_hat = (1 - ALPHA) * (phi_hat + DT * phi_dot) + ALPHA * phi_acc;
  theta_hat = (1 - ALPHA) * (theta_hat + DT * theta_dot) + ALPHA * theta_acc;

  // Convert into degrees for reading output
  float angleX = phi_hat * 180.0 / PI;
  float angleY = theta_hat * 180.0 / PI;

  float accelAmp = sqrt(accels_m_s2.x()*accels_m_s2.x() +accels_m_s2.y()*accels_m_s2.y() + accels_m_s2.z()*accels_m_s2.z());
  accelAmpFiltered = accelFilterAlpha * accelAmp + (1 - accelFilterAlpha) * accelAmpFiltered;
  

  gyroSign = bool(gyros_rad_sec.z()>0);
  if (gyroSignPrev != gyroSign){
    GYRO_SW = 1;
    gyroSignPrev = gyroSign;
  }
  
  unsigned long now = millis();

  // if ((now - lastEventTime > eventCooldown) && accelAmpFiltered > 11.5 && angleX < -10.0 && gyroZFiltered > 5) {
  if ((now - lastEventTime > eventCooldown) && accelAmpFiltered > 11 && angleX < -5.0 && GYRO_SW==HIGH) {
    HS = 1;
    TO = 0;
    GYRO_SW = 0;
    lastEventTime = now;

    // Serial.println("Heel Strike");
  }
  // if ((now - lastEventTime > eventCooldown) && accelAmpFiltered > 10.5 && angleX > 30.0  && gyroZFiltered < -5) {
  // if ((now - lastEventTime > eventCooldown) && accelAmpFiltered > 10.5 && angleX > 30.0  && GYRO_SW==HIGH) {
  if ((now - lastEventTime > eventCooldown) && accelAmpFiltered > 11.5 && angleX > 30.0 && GYRO_SW==HIGH) {
    HS = 0;
    TO = 1;
    GYRO_SW = 0;
    lastEventTime = now;

    // Serial.println("Toe Off");
  }
  digitalWrite(GREEN_LED_PIN, HS);
  digitalWrite(RED_LED_PIN, TO);


  // Serial.print("Roll (X): "); Serial.println(angleX);
  // Serial.print("Pitch (Y): "); Serial.println(angleY);
  // Serial.println();

  // The roll angle (X) and pitch angle (Y) are according to the IMU axes printed on the board. 
  // The Adafruit axes are not in the same direction and printed on board, rather X-> -z, Y-> y, Z-> -x
  // That is why we compare our angleX with Adafruit (-1)*euler.z

  // Printing values:
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print(-euler.z());
  Serial.print(", \t");
  Serial.print(euler.y());
  Serial.print(", \t");
  Serial.print(angleX);
  Serial.print(", \t");
  Serial.print(angleY);
  Serial.print(", \t");
  Serial.print(accelAmpFiltered);
  Serial.print(", \t");
  Serial.println(HS);
  // Serial.print('\t');
  // Serial.print(accels_m_s2.z());

  // Serial.print(euler.y());
  // Serial.print('\t');
  // Serial.println(angleY);
  // End Printing

  // Detecting gait events
   




  delay(BNO055_SAMPLERATE_DELAY_MS);
}
