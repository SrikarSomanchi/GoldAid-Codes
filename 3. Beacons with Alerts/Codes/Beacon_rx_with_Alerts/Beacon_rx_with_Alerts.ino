#include <SPI.h>
#include <LoRa.h>

#define LORA_SS 5
#define LORA_RST 33
#define LORA_DI0 32



unsigned long startMillis = 0;
const unsigned long beaconInterval = 30000; // Beacon interval: 20 seconds for each TX
const int numTXDevices = 2; // Number of TX devices

int currentTX = 0; // Index of the current TX device

void setup() {
  Serial.begin(115200);
  while (!Serial);

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

  
  startMillis = millis();
  Serial.println("Receiver is ready.");
}

void loop() {
  unsigned long currentMillis = millis();

  String received_urg = " ";

   while (LoRa.available()) {
      received_urg += (char)LoRa.read();

    }
    
  if(received_urg.indexOf("Fall detected") != -1)
  {
  Serial.println(" " + received_urg);
  Serial.println("***ALERT Fall Detected***");
  }


  
 else {
  if (currentMillis - startMillis >= beaconInterval) {
    startMillis = currentMillis;

    sendBeaconSignal();
    currentTX = (currentTX + 1) % numTXDevices; // Switch to the next TX device
  }

  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }
    
    Serial.println("Received from TX: " + received);
  }

 
}


}

 void sendBeaconSignal() {
  if (currentTX == 0) {
    LoRa.beginPacket();
    LoRa.print("001");
    LoRa.endPacket();
    Serial.println("Beacon signal sent to TX1.");
  } else if (currentTX == 1) {
    LoRa.beginPacket();
    LoRa.print("002");
    LoRa.endPacket();
    Serial.println("Beacon signal sent to TX2.");
  } //else if (currentTX == 2) {
//    LoRa.beginPacket();
//    LoRa.print("ICW003");
//    LoRa.endPacket();
//    Serial.println("Beacon signal sent to TX3.");
//  }
}
