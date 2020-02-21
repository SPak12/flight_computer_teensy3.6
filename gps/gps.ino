#include <TinyGPS++.h>

#define GPSBaud 9600

float latitude, longitude;

TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);
  delay(100);
  Serial.println("\n------------------------------\n");

  /* NEO-6M GPS */
  Serial.println("Intializing NEO-6M GPS...");
  Serial1.begin(GPSBaud);
  Serial.println("Success!\n");
  delay(1000);
}

void loop() {
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
}
