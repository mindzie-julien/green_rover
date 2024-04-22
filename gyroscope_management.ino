#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMI160.h>
 
Adafruit_BMI160 bmi;
 
float angleAcc, angleGyro, angle;
unsigned long tPrev;
float dt;
 
void setup() {
  Serial.begin(9600);
 
  if (!bmi.begin()) {
    Serial.println("Could not find a valid BMI160 sensor, check wiring!");
    while (1);
  }
 
  bmi.setAccelerometerRange(2);
  bmi.setAccelerometerRate(1600); // Hz
  bmi.setGyroRange(2000);
  bmi.setGyroRate(1600); // Hz
 
  tPrev = micros();
}
 
void loop() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
 
  bmi.getEvent(&accel, &gyro, &temp);
 
  // Calculate dt
  unsigned long tNow = micros();
  dt = (tNow - tPrev) / 1000000.0;
  tPrev = tNow;
 
  // Calculate accelerometer angle
  angleAcc = atan2(accel.acceleration.y, accel.acceleration.z) * 180 / PI;
 
  // Calculate gyro angle
  angleGyro += gyro.gyro.x * dt;
 
  // Complementary filter
  angle = 0.98 * angleGyro + 0.02 * angleAcc;
 
  Serial.print("Angle: "); Serial.println(angle);
 
  delay(100);
}
