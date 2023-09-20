#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>

#define ss 5
#define rst 33
#define dio0 32

#include <WiFi.h>
#include "ThingSpeak.h"     // always include thingspeak header file after other header files and custom macros

char ssid[] = "gabru";      // your network SSID (name) 
char pass[] = "gabru597";   // your network password
int keyIndex = 0;           // your network key Index number (needed only for WEP)
WiFiClient  client;

unsigned long myChannelNumber = 1868059;
const char * myWriteAPIKey = "XFE6ZU03PE9F9FKZ";

char str[100];
char value[30];

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("BSD to ThinkSpeak");

  LoRa.setPins(ss, rst, dio0);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  LoRa.setSyncWord(0xA5);
  LoRa.setSpreadingFactor(10);          
  LoRa.setSignalBandwidth(62.5E3);  
  
  WiFi.mode(WIFI_STA);   
  ThingSpeak.begin(client);  // Initialize ThingSpeak

  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay(5000);     
    } 
    Serial.println("\nConnected.");
  }
}

void loop() {
  
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.println("INSIDE");
    uint8_t count = 0;
    while (LoRa.available()) {
    str[count++] = (char)LoRa.read();
    }
    str[count] = '\0';

    Serial.print("String Received : ");
    Serial.println(str);

    /*** Now Starting extracting data from string ***/
      int i = 0;
      int j = 0;
      int flag = 1;
      
      while(str[i]!='\0')
      {
        if(str[i]=='_') {
          value[j] = '\0';
          ThingSpeak.setField(flag, value);
          Serial.print(flag);
          Serial.print(", ");
          Serial.println(value);
          flag++;
          j=0;
        }
        else value[j++] = str[i];
        i++;
      }
      
        Serial.print(flag);
        Serial.print(", ");
        Serial.println(value);
      int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
      if(x == 200){
      Serial.println("Channel update successful.");
    }
    else{
      Serial.println("Problem updating channel. HTTP error code " + String(x));
    }
  }
}
