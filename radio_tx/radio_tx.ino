#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 10
#define RFM95_RST 3
#define RFM95_INT 2
#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);
int packetcount;
uint8_t packet[];

void setup() {
  Serial.begin(9600);
  delay(100);
  Serial.println("Initializing RFM95 LoRa Radio Transmitter...");

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(RFM95_RST, LOW); delay(10);
  digitalWrite(RFM95_RST, HIGH); delay(10);

  if (!rf95.init()) {
    Serial.println("Failed!");
    while(1);
  } else {
    Serial.println("Success!\n");
    delay(1000);
  }

  Serial.print("Setting frequency to: "); Serial.println(RF95_FREQ);
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("Failed!");
    while(1);
  } else {
    Serial.println("Success!\n");
    delay(1000);
  }
}

void loop() {
  Serial.println("Transmitting...");
  packet[] = "Hello world!";
  rf95.send((uint8_t *) packet, sizeof(packet));
  rf95.waitPacketSent();

  Serial.println("Sent!");

  delay(100);
}
