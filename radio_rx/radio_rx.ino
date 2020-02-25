#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);
float t; // Time
float a; // Altitude
float r, p, y; // Orientation
float ax, ay, az; // Acceleration
float latitude, longitude; // GPS Coordinates
int packetcount; // Radio packet counter
//int s1, s2, s3, s4; // Servo commands

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing RFM95 LoRa Radio Receiver...");

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(RFM95_RST, LOW); delay(10);
  digitalWrite(RFM95_RST, HIGH); delay(10);

  if (!rf95.init()) {
    Serial.println("Failed!");
    while(1);
  } else {
    Serial.print("Setting frequency to: "); Serial.print(RF95_FREQ); Serial.println(" MHz");
    if (!rf95.setFrequency(RF95_FREQ)) {
      Serial.println("Failed!");
      while(1);
    } else {
      Serial.println("Success!\n");
    }
  }
  
  delay(1000);
  
}

void loop() {
  if (rf95.available()) {
    if (rf95.recv(buf, &len)) {
      Serial.println("Received data!");
      Serial.print("RSSI: "); Serial.print(rf95.lastRssi(), DEC);
      memcpy(&packetcount, buf, sizeof(packetcount));
      memcpy(&t, buf + 2, sizeof(t));
      memcpy(&a, buf + 6, sizeof(a));
      memcpy(&r, buf + 10, sizeof(r));
      memcpy(&p, buf + 14, sizeof(p));
      memcpy(&y, buf + 18, sizeof(y));
      memcpy(&ax, buf + 22, sizeof(ax));
      memcpy(&ay, buf + 26, sizeof(ay));
      memcpy(&az, buf + 30, sizeof(az));
      memcpy(&latitude, buf + 34, sizeof(latitude));
      memcpy(&longitude, buf + 38, sizeof(longitude));
//      memcpy(&s1, buf + 42, sizeof(s1));
//      memcpy(&s2, buf + 44, sizeof(s2));
//      memcpy(&s3, buf + 46, sizeof(s3));
//      memcpy(&s4, buf + 48, sizeof(s4));
      
      Serial.print("\tc: "); Serial.print(packetcount);
      Serial.print("\tt: "); Serial.print(t);
      Serial.print("\ta: "); Serial.print(a);
      Serial.print("\tr: "); Serial.print(r);
      Serial.print("\tp: "); Serial.print(p);
      Serial.print("\ty: "); Serial.print(y);
      Serial.print("\tax: "); Serial.print(ax);
      Serial.print("\tay: "); Serial.print(ay);
      Serial.print("\taz: "); Serial.print(az);
      Serial.print("\tlat: "); Serial.print(latitude);
      Serial.print("\tlong: "); Serial.print(longitude);
//      Serial.print("\ts1: "); Serial.print(s1);
//      Serial.print("\ts2: "); Serial.print(s2);
//      Serial.print("\ts3: "); Serial.print(s3);
//      Serial.print("\ts4: "); Serial.println(s4);
    } else {
      Serial.println("Receive failed!\n");
    }
  }
}
