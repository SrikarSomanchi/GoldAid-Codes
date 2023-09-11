#include <LoRa.h>

unsigned long startMillis = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  LoRa.setPins(5, 33, 32);  // Set LoRa module pins (CS, RST, IRQ)
  
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  startMillis = millis();
  Serial.println("Tx Ready");
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Check for incoming messages
  if (LoRa.parsePacket()) {
    String receivedMessage = LoRa.readString();
    
    
    if (receivedMessage.equals("Send IDs")) {
      delay(5000);
      // Respond with device ID
      String deviceID = "ID001"; // Replace with actual ID
      currentMillis = millis();//edit
      
      while(currentMillis - startMillis <= 30000){
      LoRa.beginPacket();
      LoRa.print(deviceID);
      LoRa.endPacket();
      Serial.println("ID Sent: " +deviceID);
      delay(100);//edit
      currentMillis=millis();//edit
      }
      startMillis = currentMillis;
      
    }
  }
}
