#include <Wire.h>
#include <SPI.h>
#include <TimeLib.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <RH_RF95.h>

#define SD_CS BUILTIN_SDCARD
#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
#define RF95_FREQ 915.0
#define GPSBaud 9600

float t; // Time
float a; // Altitude
float x, y, z; // Orientation
float ax, ay, az; // Acceleration
float latitude, longitude; // GPS Coordinates
int calStatus; // BNO055 calibration status
File data; // SD file
char FileName[] = "flight.csv";
int packetcount; // Radio packet counter
uint8_t packet[50];

Adafruit_BMP280 bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
TinyGPSPlus gps;
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  Serial.begin(9600);
  delay(100);
  Serial.println("\n------------------------------\n");


  /* BMP280 Pressure Sensor */
  Serial.println("Initializing BMP280 Pressure Sensor...");
  if (!bmp.begin()) {
    Serial.println("Failed!\n");
    while(1);
  }
  else {
    Serial.println("Success!\n");
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    delay(1000);
  }


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


  /* NEO-6M GPS */
  Serial.println("Intializing NEO-6M GPS...");
  Serial1.begin(GPSBaud);
  Serial.println("Success!\n");
  delay(1000);


  /* RFM95x LoRa Radio */
  Serial.println("Initializing RFM95 LoRa Radio Transmitter...");
  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(RFM95_RST, LOW); delay(10);
  digitalWrite(RFM95_RST, HIGH); delay(10);
  
  if (!rf95.init()) {
    Serial.println("Failed!\n");
//    while(1);
  } else {
    Serial.print("Setting frequency to: "); Serial.println(RF95_FREQ);
    if (!rf95.setFrequency(RF95_FREQ)) {
      Serial.println("Failed!\n");
      while(1);
    } else {
      Serial.println("Success!\n");
      packetcount = 0;
    }
  }
  delay(1000);


  /* SD Card Reader */
  Serial.println("Initializing SD Card Reader...");
  if (!SD.begin(SD_CS)) {
    Serial.println("Failed!\n");
//    while(1);
  }
  else {
    Serial.println("Success!\n");
    data = SD.open(FileName, FILE_WRITE);
    if (!data) {
      Serial.println("Error opening file!\n");
//      while(1);
    }
    data.println("count,time,altitude,roll,pitch,yaw,ax,ay,az,lat,long");
    data.close();
    delay(1000);
  }

  Serial.println("\nFC Ready!");
}

void loop() {
  t = (double) millis() / (double) 1000.0;
  Serial.print("t: ");
  Serial.print(t);
  Serial.print("\t");

  /* Altimeter (BMP280) */
  a = bmp.readAltitude(1013.25);
  Serial.print("a: ");
  if (a >= 0) Serial.print(" ");
  Serial.print(a);


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
  x = euler.x(); y = euler.y(); z = euler.z();
  ax = accel.x(); ay = accel.y(); az = accel.z();

  Serial.print("\tX: "); Serial.print(x, 4);
  Serial.print("\tY: "); Serial.print(y, 4);
  Serial.print("\tZ: "); Serial.print(z, 4);
  calStatus = displayCalStatus();
  Serial.println("");


  /* GPS */
  if (Serial1.available() > 0) {
    gps.encode(Serial1.read());
    if (gps.location.isUpdated()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
      Serial.print("Latitude: ");
      Serial.print(latitude, 6);
      Serial.print("\tLongitude: ");
      Serial.println(longitude, 6);
    }
  }


  /* Datalogging (SD Card) */
  data = SD.open(FileName, FILE_WRITE);
  if (!data) {
    Serial.println("Error opening file!\n");
//    while(1);
  } else {
    data.print(packetcount); data.print(",");
    data.print(t); data.print(",");
    data.print(a); data.print(",");
    data.print(x, 4); data.print(",");
    data.print(y, 4); data.print(",");
    data.print(z, 4); data.print(",");
    data.print(ax, 4); data.print(",");
    data.print(ay, 4); data.print(",");
    data.print(az, 4); data.print(",");
    data.print(latitude, 6); data.print(",");
    data.print(longitude, 6); data.println("");
//    data.print(s1_command); data.print(",");
//    data.print(s2_command); data.print(",");
//    data.print(s3_command); data.print(",");
//    data.print(s4_command); data.println("");
    data.close();
  }


  /* Radio (RFM95x) */
//  Serial.print("Transmitting Packet #"); Serial.println(packetcount);
//  memcpy(packet, &packetcount, sizeof(packetcount));
//  memcpy(packet + 2, &t, sizeof(t));
//  memcpy(packet + 6, &a, sizeof(a));
//  memcpy(packet + 10, &x, sizeof(x));
//  memcpy(packet + 14, &y, sizeof(y));
//  memcpy(packet + 18, &z, sizeof(z));
//  memcpy(packet + 22, &ax, sizeof(ax));
//  memcpy(packet + 26, &ay, sizeof(ay));
//  memcpy(packet + 30, &az, sizeof(az));
//  memcpy(packet + 34, &latitude, sizeof(latitude));
//  memcpy(packet + 38, &longitude, sizeof(longitude));
//  memcpy(packet + 42, &s1_command, sizeof(s1_command));
//  memcpy(packet + 44, &s1_command, sizeof(s1_command));
//  memcpy(packet + 46, &s1_command, sizeof(s1_command));
//  memcpy(packet + 48, &s1_command, sizeof(s1_command));
//  rf95.send((uint8_t *) packet, sizeof(packet)); delay(10);
//  rf95.waitPacketSent();
//  packetcount++;
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
