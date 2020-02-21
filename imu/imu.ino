#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

float t; // Time
float r, p, y; // Euler angles
float ax, ay, az; // Acceleration
int calStatus; // BNO055 calibration status

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  Serial.begin(9600);
  delay(100);
  Serial.println("\n------------------------------\n");

  /* BNO055 9-Axis IMU */
  Serial.println("Initializing BNO055 9-Axis IMU...");
  if (!bno.begin()) {
    Serial.println("Failed!\n");
    while(1);
  }
  else {
    Serial.println("Success!\n");
    bno.setExtCrystalUse(true);
    delay(1000);
  }
}

void loop() {
  t = (double) millis() / (double) 1000.0;
  Serial.print("t: ");
  Serial.print(t);
  Serial.print("\t");

  /* Orientation (BNO055 9-Axis IMU) */
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  r = euler.x(); p = euler.y(); y = euler.z();
  ax = accel.x(); ay = accel.y(); az = accel.z();

  Serial.print("\tr: "); Serial.print(r, 4);
  Serial.print("\tp: "); Serial.print(p, 4);
  Serial.print("\ty: "); Serial.print(y, 4);
  Serial.print("\tax: "); Serial.print(ax, 2);
  Serial.print("\tay: "); Serial.print(ay, 2);
  Serial.print("\taz: "); Serial.print(az, 2);
  calStatus = displayCalStatus();
  Serial.println("");

  delay(100);
}

int displayCalStatus(void) {
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  Serial.print("\t");
  if (!system) {
    Serial.print("! ");
  }

  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
  return system;
}
