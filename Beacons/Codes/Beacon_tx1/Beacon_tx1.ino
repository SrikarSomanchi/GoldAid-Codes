 #include <SPI.h>
#include <LoRa.h>

const int buttonPin = 21;
int currentButtonState; 

#define LORA_SS 5
#define LORA_RST 33
#define LORA_DI0 32

bool sendHi = false;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(buttonPin, INPUT_PULLUP);

  
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DI0);

  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa initialization failed. Check your wiring.");
    while (1);
  }
  LoRa.setTxPower(20); 
  LoRa.setSpreadingFactor(10); 
  LoRa.setSyncWord(0xA5);
  LoRa.setSignalBandwidth(62.5E3);    
  LoRa.setCodingRate4(8);

  Serial.println("TX1 is ready.");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  currentButtonState = digitalRead(buttonPin);
  
  
     if (currentButtonState != HIGH) {
      LoRa.beginPacket();
      LoRa.print("Fall detected in TX1");
      LoRa.endPacket();

      Serial.println("Message sent from TX1: Fall detected");
     }
     
  
  else {
  if (packetSize) {
    String received = "";

    while (LoRa.available()) {
      received += (char)LoRa.read();

    }
      Serial.println();
      Serial.println(received);
      Serial.println();
      
    if (received == "001") {
      Serial.println("Received beacon signal for TX1. Sending 'hi' message.");
      sendHi = true;
    } else {
      Serial.println("Beacon signal not for TX1. Stopping 'hi' messages.");
      sendHi = false;
    }
  }

  if (sendHi) {
    LoRa.beginPacket();
    LoRa.print("hi from WSD1");
    LoRa.endPacket();

    Serial.println("Message sent from TX1: hi from WSD1");
  }
  sendHi = false;
  }
}
