#include <Wire.h>
#include <Adafruit_BMP280.h>

float t; // Time
float a; // Altitude

Adafruit_BMP280 bmp;

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
  Serial.println(a);
  delay(100);
}
