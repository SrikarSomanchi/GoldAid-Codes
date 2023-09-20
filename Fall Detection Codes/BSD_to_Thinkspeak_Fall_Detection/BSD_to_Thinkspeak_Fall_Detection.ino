#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>

#define ss 5
#define rst 33
#define dio0 32

#include <WiFi.h>
#include "ThingSpeak.h"     // always include thingspeak header file after other header files and custom macros

char ssid[] = "project20";      // your network SSID (name) 
char pass[] = "12345678";   // your network password
int keyIndex = 0;           // your network key Index number (needed only for WEP)
WiFiClient  client;

unsigned long myChannelNumber = 2224492;
const char * myWriteAPIKey = "V8PXX1SKENGVXBKI";

char str[50];
char value[10];

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\n\nBSD to ThinkSpeak");

  LoRa.setPins(ss, rst, dio0);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  LoRa.setSyncWord(0xA5);
  LoRa.setSpreadingFactor(20);          
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
    str[count] = (char)LoRa.read();

    //Writing the next 2 lines, just to get update of fall detection on the dashboard
    if(str[count]=='#')  {
      Serial.println("INSIDE #");
      int x = ThingSpeak.writeField(myChannelNumber, 7, 0, myWriteAPIKey);
      if(x == 200) Serial.println("Channel update successful.");
      else         Serial.println("Problem updating channel. HTTP error code " + String(x));
    }
    if(str[count]=='*')  {
      Serial.println("INSIDE *");
      int x = ThingSpeak.writeField(myChannelNumber, 7, 1, myWriteAPIKey);
      if(x == 200) Serial.println("Channel update successful.");
      else         Serial.println("Problem updating channel. HTTP error code " + String(x));
    }
    count++;
    }
    str[count] = '\0';
    Serial.print("\n\nString Received2: ");
    Serial.println(str);

    /*** Now Starting extracting data from string ***/
/*      int i = 0;
      int j = 0;
      int flag = 1;
      while(str[i]!='\0')
      {
        if(str[i]==',') {
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
*/      /*int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
      if(x == 200){
      Serial.println("Channel update successful.");
    }
    else{
      Serial.println("Problem updating channel. HTTP error code " + String(x));
    }*/
  }
}
