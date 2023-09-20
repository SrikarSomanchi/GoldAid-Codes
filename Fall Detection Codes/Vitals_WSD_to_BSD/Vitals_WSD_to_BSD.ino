/*
 * Vitals - HR, SpO2 from Protocentral, TEMPERATURE from TMP117
 * LoRa communication
 * Acc + Gyro Logging
 * Ext. button for reseting
 * Ext. LED for status
 */

int USER_ID = 1068;

#include <Wire.h> //I2C library
#include "SPI.h" //SPI library
#include "SD.h" //SD Card
#include "max32664.h" //PulseOx
#include <EEPROM.h>
#include <LoRa.h>

#define RESET_PIN 14
#define MFIO_PIN 12
#define RAWDATA_BUFFLEN 250

String tx_string;

const long frequency = 433E6;  // LoRa frequency
const int csPin = 5;          // LoRa radio chip select
const int resetPin = 33;       // LoRa radio reset
const int irqPin = 32;          // change for your board; must be a hardware interrupt pin

long old_millis;
uint8_t acc_sampling_period = 3; //4ms, that means sampling freq = 250 Hz

long old_millis_pulseox;
int pulseox_sampling_period = 100;
boolean LoRa_send_flag = 0;

long old_millis_LoRa;
int LoRa_transmit_period = 30000;

uint16_t dataReadyFlag, responseFromI2C;
float temp;

long counter = 0;
int  interval = 1000;

max32664 MAX32664(RESET_PIN, MFIO_PIN, RAWDATA_BUFFLEN);

void mfioInterruptHndlr(){
  Serial.println("i");
}
 
void enableInterruptPin(){
  //pinMode(mfioPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MAX32664.mfioPin), mfioInterruptHndlr, FALLING);
}

void loadAlgomodeParameters(){

  algomodeInitialiser algoParameters;
  /*  Replace the predefined values with the calibration values taken with a reference spo2 device in a controlled environt.
      Please have a look here for more information, https://pdfserv.maximintegrated.com/en/an/an6921-measuring-blood-pressure-MAX32664D.pdf
      https://github.com/Protocentral/protocentral-pulse-express/blob/master/docs/SpO2-Measurement-Maxim-MAX32664-Sensor-Hub.pdf
  */

  algoParameters.calibValSys[0] = 120;
  algoParameters.calibValSys[1] = 122;
  algoParameters.calibValSys[2] = 125;

  algoParameters.calibValDia[0] = 80;
  algoParameters.calibValDia[1] = 81;
  algoParameters.calibValDia[2] = 82;

  algoParameters.spo2CalibCoefA = 1.5958422;
  algoParameters.spo2CalibCoefB = -34.659664;
  algoParameters.spo2CalibCoefC = 112.68987;

  MAX32664.loadAlgorithmParameters(&algoParameters);
}


void setup(void) {  

  Serial.begin(115200);
  Serial.println("STARTED");
  
  Wire.begin();
    
  /**********PulseOx setup starts**********/
    loadAlgomodeParameters();

    if(MAX32664.hubBegin() != CMD_SUCCESS)
    {
      Serial.println("MAX32664 not initialised");
      while(1);
    }
  
    bool ret;
//    = MAX32664.startBPTcalibration();
//    while(!ret){
//      delay(10000);
//      Serial.println("failed calib, please retsart");
//      //ret = MAX32664.startBPTcalibration();
//    }
//  
//    delay(1000);
  
    Serial.println("start in estimation mode");
    ret = MAX32664.configAlgoInEstimationMode();
    while(!ret){
  
      Serial.println("failed est mode");
      ret = MAX32664.configAlgoInEstimationMode();
      delay(10000);
    }
  
    //MAX32664.enableInterruptPin();
    Serial.println("Getting the device ready..");
    delay(1000); 
/***********PulseOx setup complete***************/
  
  
  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    while(1); // if failed, do nothing
  }
  
  LoRa.setSyncWord(0xA5);
  LoRa.setTxPower(10);
  LoRa.setSpreadingFactor(10);          
  LoRa.setSignalBandwidth(62.5E3);     
  LoRa.setCodingRate4(8);

  /* The below lines are commented because we are using LoRa sender code not the node/gateway code
   *LoRa.onReceive(onReceive);
   *LoRa.onTxDone(onTxDone);
   *LoRa_rxMode();
   */
  
  Wire.beginTransmission(0x48);
  if(Wire.endTransmission()==0) Serial.println("TMP117 Found");
  else Serial.println("TMP117 Not Found");

  Wire.beginTransmission(0x48);
  Wire.write(0x01);
  Wire.endTransmission(false);
  Wire.requestFrom(0x48, 2, true);
  uint8_t highbyte = Wire.read();
  uint8_t lowbyte  = Wire.read();
  highbyte |= 0b00001100; // for setting one shot mode
  
  Wire.beginTransmission(0x48);
  Wire.write(0x01);
  Wire.write(highbyte);
  Wire.write(lowbyte);
  Wire.endTransmission();
  
  delay(100);
  LoRa.beginPacket();
  LoRa.print("Hello from WSD" + String(USER_ID));
  LoRa.endPacket();
  
}

void loop(void) 
{
  if((millis()-old_millis_LoRa) > LoRa_transmit_period){
      old_millis_LoRa = millis();

        char TXLoRaChar[20];
        tx_string.toCharArray(TXLoRaChar, 20);
        
        LoRa.beginPacket();
        LoRa.print(TXLoRaChar);
        LoRa.endPacket();
  }
  
  if((millis()-old_millis_pulseox) > pulseox_sampling_period){
      old_millis_pulseox = millis();
 
      Wire.beginTransmission(0x48);
      Wire.write(0x01);
      Wire.endTransmission(false);
      Wire.requestFrom(0x48, 2, true);
      responseFromI2C = ( Wire.read() << 8| Wire.read());
      dataReadyFlag = (responseFromI2C >> 13) & 0x0001;
      
        if(dataReadyFlag){
          Wire.beginTransmission(0x48);
          Wire.write(0x00);
          Wire.endTransmission(false);
          Wire.requestFrom(0x48, 2, true);
          responseFromI2C = ( Wire.read() << 8| Wire.read());
          temp = responseFromI2C * 0.0078125; //Resolution = 0.0078125

        Wire.beginTransmission(0x48);
        Wire.write(0x01);
        Wire.endTransmission(false);
        Wire.requestFrom(0x48, 2, true);
        uint8_t highbyte = Wire.read();
        uint8_t lowbyte  = Wire.read();
        highbyte |= 0b00001100; // for setting one shot mode
        
        Wire.beginTransmission(0x48);
        Wire.write(0x01);
        Wire.write(highbyte);
        Wire.write(lowbyte);
        Wire.endTransmission();
        }
        
      uint8_t num_samples = MAX32664.readSamples();

      if(num_samples){
        Serial.print("Millis = ");
        Serial.print(millis());              
        Serial.print(", hr = ");
        Serial.print(MAX32664.max32664Output.hr);
        Serial.print(", spo2 = ");
        Serial.print(MAX32664.max32664Output.spo2);
        Serial.print(", Temp = ");
        Serial.println(temp);
        
//        tx_string = "";
//        tx_string += String(MAX32664.max32664Output.hr);
//        tx_string += ",";
//        tx_string += String(MAX32664.max32664Output.spo2);
//        tx_string += ",";
//        tx_string += String(MAX32664.max32664Output.sys);
//        tx_string += ",";
//        tx_string += String(MAX32664.max32664Output.dia);
//        tx_string += ",";
//        tx_string += String(temp);

        tx_string = "";
        tx_string += String(temp);
        tx_string += ",";
        tx_string += String(MAX32664.max32664Output.sys);
        tx_string += ",";
        tx_string += String(MAX32664.max32664Output.dia);
        tx_string += ",";
        tx_string += String(MAX32664.max32664Output.spo2);
        tx_string += ",";
        tx_string += String(MAX32664.max32664Output.hr);
        
      }
  }
}
