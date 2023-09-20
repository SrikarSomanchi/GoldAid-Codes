#include <Wire.h> //I2C library
#include "SPI.h" //SPI library
#include <LoRa.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h> //#include <Adafruit_ADXL345_U.h>
#include <math.h>

bool led_flag = 0;

const long frequency = 433E6;  // LoRa frequency
const int csPin = 5;           // LoRa radio chip select
const int resetPin = 33;       // LoRa radio reset
const int irqPin = 32;         // change for your board; must be a hardware interrupt pin

long old_millis;
long counter = 0;
int  interval = 5000;
float confidence2;

long old_millis_LoRa = 0;
int thinkspeak_reset_period = 30000;
uint8_t flag_thinkspeak = 1;

#define FALL_EXPECTED_COUNT 10 //How many falls the algorithm will develop in 3000 samples (around 10 seconds)
#define STATUS 25 //LED PIN for showing the status of the µC (logging has completed/ error indication) [WS2812 can be integrated to give the exact status]
#define EEPROM_SIZE 12 

/* Assign a unique ID to this sensor at the same time */
Adafruit_MPU6050 mpu; //Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);


/*variables*/
uint8_t measure_flag;  
/* 
 * Flag to determine what µC is doing
 * 1: Measuring the acc values and saving them in SRAM
 * 0: Logging the values from SRAM into SD Card (Slow process but only used in prototyping for development of algorithm)
 * 2: Comes out of both the functions for now, & do nothing
 */

boolean fall_flag = 0;
boolean fall_certain = 0; 
/* 
 * Its kind of a fall portent,   
 * In detail, after freefall has been detected, if FALL CONFIDENCE value > fall_confidence for TIME > fall_min_time, then FALL is sure
 * Just waiting for a peak in acceleration value > tap_thresh
 */
  
boolean fall_confirm = 0;       // This flag confirms the fall
uint8_t fall_confirm_count = 0; // Counter for number of falls detected
long measure_count = 0;
/* 
 *  It is count of number of samples 
 *  Just needed for prototyping purposes, as to limit the amount of data to be logged
 *  As SRAM size limitation & Direct logging into SD card was pretty slow, (around 20ms) therefore only 50Hz sampling frequency is achieved
 */
 
long fall_count = 0;
long not_fall_count = 0;
long total_count = 0;
long start_millis, end_millis;

/*FALL PARAMETERS*/
float fall_thresh = 0.9; // This is the value below which the µC assumes the body is going into free-fall
float tap_thresh = 6;
/*
 * This is the value required at the impact of the body on the surface which create an high acc value
 * But there is a catch, as high acc value can also be achieved by the worker in some other acitivies also, such as hammering, drilling and other laborious tasks
 * Therefore our algorithm will not consider the peak acceleration achieved at the impact but also the free fall period,
 * for eg a free fall period of 500ms will give the height of fall of around 1.225m, which is calculated from Newton's equation of motion
 */
 
float not_fall_thresh = 1.1; //Currently not used
float fall_min_time = 100; //500 ms, more info is given in the above comments
uint8_t fall_confidence = 70;
/*
 * Confidence in percentage for fall (no. of samples below fall_thresh/ total samples) after the free fall has started
 */
uint8_t not_fall_confidence = 10;

long MILLIS;
float x, y, z, xg, yg, zg;

void setup(void) {  

  Serial.begin(115200);
  Serial.println("STARTED");
  
  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    //while (true);                       // if failed, do nothing
  }
  LoRa.setSyncWord(0xA5);
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(12);          
  LoRa.setSignalBandwidth(62.5E3);     
  LoRa.setCodingRate4(8);
  //Serial.print("TX POWER: "); Serial.println(singleTransfer());

  /* The below lines are commented because we are using LoRa sender code not the node/gateway code
   *LoRa.onReceive(onReceive);
   *LoRa.onTxDone(onTxDone);
   *LoRa_rxMode();
   */
  
  pinMode(STATUS, OUTPUT);
  digitalWrite(STATUS, HIGH);
  delay(200);
  /*
   * Setting the digitalPin as output, and glowing it
   */
  
  if (!mpu.begin()) {//if(!accel.begin()) while(1); //if not intialised, code will not go further ahead
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  } 

  digitalWrite(STATUS, LOW);  
  measure_flag = 1;

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G); //accel.setRange(ADXL345_RANGE_16_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // send packet for acknowledging the completion of initialisation
  LoRa.beginPacket();
  LoRa.print("Hello from WSD fall detector");
  LoRa.endPacket();
}

void loop(void) 
{  
    if((millis()-old_millis_LoRa) > thinkspeak_reset_period){ //This if condition is to just reset the fall flag on Thinkspeak

          if(flag_thinkspeak == 1)
        {
          Serial.println("Inside updation");
          flag_thinkspeak = 0;
          LoRa.beginPacket();
          LoRa.print("Reseting Flag*");
          LoRa.endPacket();
        }
        
  }

  
    sensors_event_t event, g, temp; //sensors_event_t event; 
    mpu.getEvent(&event, &g, &temp); //accel.getEvent(&event);
     
    MILLIS = millis();
    x = event.acceleration.x/9.8;
    y = event.acceleration.y/9.8;
    z = event.acceleration.z/9.8;

    //xg[measure_count] = g.gyro.x;
    //yg[measure_count] = g.gyro.y;
    //zg[measure_count] = g.gyro.z;

    /*Calculating the Sum Vector Magnitude of the acceleration values of all 3-axis*/
    float mag = pow(x, 2) + pow(y, 2) + pow(z, 2); 
          mag = pow(mag, 0.5);

if(mag > 6){
    Serial.print(x); 
    Serial.print(" "); 
    Serial.print(y); 
    Serial.print(" "); 
    Serial.print(z); 
    Serial.print(" "); 
    Serial.print(mag); 
    Serial.print(" "); 
    Serial.println(mag);
}
    if(mag < fall_thresh){

      if(fall_flag == 0) {
        start_millis = millis();
        fall_count = 0;
        total_count = 0;
      }

      fall_flag = 1;
      
      fall_count++;
      if((millis() - start_millis) > fall_min_time){
        
        float confidence = float(fall_count)/float(total_count);
              confidence *= 100.0;

        if(((fall_count/total_count)*100.0) > fall_confidence){
          fall_certain = 1;
        }
        else{
          fall_flag = 0;
          fall_count = 0;
          total_count = 0;
        }
      }
    }
    total_count++;

          confidence2 = float(fall_count)/float(total_count);
          confidence2 *= 100.0;


    if((confidence2 <= not_fall_confidence) && (fall_flag == 1)) {
      fall_flag = 0;
      Serial.println("********NOT FALL********");
    }
    
    if((fall_certain == 1) && (mag > 6)) {
      fall_confirm = 1;
      end_millis = millis();

      Serial.println();
      Serial.println("*******************FALL DETECTED*******************");\
      Serial.println();
      digitalWrite(STATUS, HIGH);

      long height = 0.5*9.8*pow((end_millis - start_millis), 2);
      float height_actual = float(height)/1000000;

      String fallLoRa  = "";
             fallLoRa  = "FALL DETECTED @ ";
             fallLoRa += String(height_actual);
             fallLoRa += "m#";

      char fallLoRaChar[30];

      fallLoRa.toCharArray(fallLoRaChar, 30);

      //Serial.println(fallBT);
      Serial.println(fallLoRa);
      Serial.println(fallLoRaChar);

      flag_thinkspeak = 1;
      old_millis_LoRa = millis();
      
      LoRa.beginPacket();
      //LoRa.print("#");  //For Thinkspeak
      LoRa.print(fallLoRaChar); //For Serial Monitor
      LoRa.endPacket();
      
      fall_flag = 0;
      fall_certain = 0;
      fall_confirm = 0; 
      }
}
