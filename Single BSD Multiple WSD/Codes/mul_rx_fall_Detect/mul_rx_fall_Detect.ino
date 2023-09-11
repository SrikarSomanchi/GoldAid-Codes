#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const int LORA_CS_PIN = 5;     // LoRa chip select pin
const int LORA_RST_PIN = 33;    // LoRa reset pin
const int LORA_IRQ_PIN = 32;    // LoRa interrupt pin

#define OLED_ADDR   0x3C
#define OLED_SDA    21
#define OLED_SCL    22

Adafruit_SSD1306 display(128, 64, &Wire, OLED_ADDR);

bool fallDetected = false;  // Flag to track if a fall is detected

void setup() {
  Serial.begin(9600);

  while (!Serial);

  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_IRQ_PIN);
  if (!LoRa.begin(433E6)) {  // Set the frequency of your LoRa module
    Serial.println("LoRa initialization failed. Check your wiring.");
    while (true);
  }

  Wire.begin(OLED_SDA, OLED_SCL);  // Initialize I2C communication

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 allocation failed. Check your wiring.");
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
}

void loop() {
  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    while (LoRa.available()) {
      String receivedMessage = LoRa.readStringUntil('\0');

      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Received message:");
      display.println(receivedMessage);

      if (receivedMessage == "Fall Detected") {
        fallDetected = true;
      }

      display.println("Fall Status:");
      display.println(fallDetected ? "Fall Detected" : "No Fall Detected");

      display.display();

      Serial.println("Received message:");
      Serial.println(receivedMessage);
      Serial.println("Fall Status:");
      Serial.println(fallDetected ? "Fall Detected" : "No Fall Detected");
    }
  }
}
