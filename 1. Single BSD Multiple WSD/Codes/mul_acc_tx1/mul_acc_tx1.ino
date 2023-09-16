#include <Wire.h>
#include <MPU6050.h>
#include <LoRa.h>

MPU6050 mpu;

const int MPU_ADDR = 0x68;  // MPU6050 I2C address
const int FALL_THRESHOLD = 1;  // Adjust this value based on your requirements

const int csPin = 5;           // LoRa radio chip select
const int resetPin = 33;       // LoRa radio reset
const int irqPin = 32;         // change for your board; must be a hardware interrupt pin

struct Vector3D {
  float x;
  float y;
  float z;
};

void setup() {
  Serial.begin(9600);

  Wire.begin();
  mpu.initialize();
  LoRa.setPins(csPin, resetPin, irqPin);

  while (!Serial);
  
  if (!LoRa.begin(433E6)) {  // Set the frequency of your LoRa module
    Serial.println("LoRa initialization failed. Check your wiring.");
    while (true);
  }
}

void loop() {
  Vector3D accel = readAccelerometer();
  
  if (detectFall(accel)) {
    sendFallDetected();
  } else {
    sendNoFallDetected();
  }
  
  delay(2000);  // Adjust the delay as needed
}

Vector3D readAccelerometer() {
  int16_t accelX, accelY, accelZ;
  mpu.getAcceleration(&accelX, &accelY, &accelZ);
  
  Vector3D accel;
  accel.x = accelX / 16384.0;
  accel.y = accelY / 16384.0;
  accel.z = accelZ / 16384.0;
  
  return accel;
}

bool detectFall(Vector3D accel) {
  // Calculate the total acceleration magnitude
  float totalAccel = sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
  
  // Check if the total acceleration exceeds the threshold
  if (totalAccel > FALL_THRESHOLD) {
    return true;
  }
  
  return false;
}

void sendFallDetected() {
  Serial.println("Fall detected from Device 2!");
  LoRa.beginPacket();
  LoRa.print("Fall detected from Device 2");
  LoRa.endPacket();
}

void sendNoFallDetected() {
  Serial.println("No fall detected from Device 2");
  LoRa.beginPacket();
  LoRa.print("No fall detected from Device 2");
  LoRa.endPacket();
}
