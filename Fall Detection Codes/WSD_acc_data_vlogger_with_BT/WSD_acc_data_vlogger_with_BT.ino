#include <Wire.h> //I2C library
#include "SPI.h" //SPI library
#include "SD.h" //SD Card
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h> //#include <Adafruit_ADXL345_U.h>

#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;
char a = 'F';

#define TOTAL 2500 //total no. of samples
#define FALL_EXPECTED_COUNT 10 //How many falls the algorithm will develop in 3000 samples (around 10 seconds)
#define STATUS 25 //LED PIN for showing the status of the µC (logging has completed/ error indication) [WS2812 can be integrated to give the exact status]
#define EEPROM_SIZE 12 

/* Assign a unique ID to this sensor at the same time */
Adafruit_MPU6050 mpu; //Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

long old_millis;
uint8_t acc_sampling_period = 3; //4ms, that means sampling freq = 250 Hz

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
float fall_min_time = 500; //500 ms, more info is given in the above comments
uint8_t fall_confidence = 80;
/*
 * Confidence in percentage for fall (no. of samples below fall_thresh/ total samples) after the free fall has started
 */
uint8_t not_fall_confidence = 10;

long MILLIS[TOTAL];
long start_millis_fall[FALL_EXPECTED_COUNT], end_millis_fall[FALL_EXPECTED_COUNT];  
float x[TOTAL], y[TOTAL], z[TOTAL], xg[TOTAL], yg[TOTAL], zg[TOTAL];

/*LOGGING FILE PARAMETERS*/
uint8_t addr = 1; //Address of the EEPROM from where the value of FILENAME is taken
uint8_t fileNo;
String loggingString = "";
String fileName;
String fallInfo = "FALL INFO:\n";
String fallBT;


void setup(void) {  
//#ifndef ESP8266
//  while (!Serial); // for Leonardo/Micro/Zero
//#endif

  SerialBT.begin("Gold-Aid_ML");
  /*
   * Bluetooth device name of our fall detection setup
   * The µC gives around 10sec in the starting (after reseting the module) to the user to connect the "Serial Bluetooth Terminal" app in smartphone to the µC's BT Classic
   */
  
  pinMode(STATUS, OUTPUT);
  digitalWrite(STATUS, HIGH);
  /*
   * Setting the digitalPin as output, and glowing it
   */
  
  Serial.begin(115200);
  Serial.println("STARTED");
  
  if(!SD.begin()){
        Serial.println("Card Mount Failed");
        while(1){
          SerialBT.println("SDCARD FAILED");
          digitalWrite(STATUS, HIGH); delay(100);
          digitalWrite(STATUS, LOW); delay(100);
        }
        return;
    }
  
  if (!mpu.begin()) {//if(!accel.begin()) while(1); //if not intialised, code will not go further ahead
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
          SerialBT.println("ACCELEROMETER FAILED");
          digitalWrite(STATUS, HIGH); delay(100);
          digitalWrite(STATUS, LOW); delay(100);
        }
  } 

  delay(10000);
  digitalWrite(STATUS, LOW);
  SerialBT.println("****GO*****");
  measure_flag = 1;

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G); //accel.setRange(ADXL345_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG); //8.73 is rad/sec of 500 degree/sec
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  /*Setting the fileName of the logged file*/
  EEPROM.begin(EEPROM_SIZE);
  fileNo = EEPROM.read(addr)+1;
  EEPROM.write(addr, fileNo);
  EEPROM.commit();
  fileName = "/ADL_v"; //For ESP32
  fileName += fileNo;
  fileName += ".txt";
  
}

void loop(void) 
{    

  if((millis()-old_millis) > acc_sampling_period){
    old_millis = millis();
    
    if(measure_flag == 1){

    sensors_event_t event, g, temp; //sensors_event_t event; 
    mpu.getEvent(&event, &g, &temp); //accel.getEvent(&event);
     
    MILLIS[measure_count] = millis();
    x[measure_count] = event.acceleration.x/9.8;
    y[measure_count] = event.acceleration.y/9.8;
    z[measure_count] = event.acceleration.z/9.8;

    xg[measure_count] = g.gyro.x;
    yg[measure_count] = g.gyro.y;
    zg[measure_count] = g.gyro.z;

    /*Calculating the Sum Vector Magnitude of the acceleration values of all 3-axis*/
    float mag = pow(x[measure_count],2); + pow(y[measure_count],2) + pow(z[measure_count],2); 
          mag = pow(mag,0.5);
    
    measure_count++;
          
    if(mag < fall_thresh){
      //Serial.println("INSIDE FREE FALL");
      if(fall_flag == 0) {
        start_millis = millis();
        fall_count = 0;
        total_count = 0;
      }
      fall_flag = 1;
      fall_count++;
      if((millis() - start_millis) > fall_min_time){
        if((fall_count/total_count)*100 > fall_confidence){
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
    
    if(((fall_count/total_count)*100 <= not_fall_confidence) && (fall_flag == 1)) {
      fall_flag = 0;
    }
    
    if((fall_certain == 1) && (mag > 6)) {
      fall_confirm = 1;
      end_millis = millis();
      
      start_millis_fall[fall_confirm_count] = start_millis;
      end_millis_fall[fall_confirm_count] = end_millis;
      
      
      Serial.println("*******************FALL DETECTED*******************");

      long height = 0.5*9.8*pow((end_millis - start_millis), 2);
      float height_actual = float(height)/1000000;
    
      fallBT = "";
      fallBT += "Start Millis = ";
      fallBT += String(start_millis);
      fallBT += "End Millis = ";
      fallBT += String(end_millis);
      fallBT += "Height = ";
      fallBT += height_actual;
      fallBT += "m*";

      for(int i=0; i<100; i++){
        if(fallBT[i] == '*') break;
        SerialBT.write(fallBT[i]);  // Command used to send the fall information on the smartphone app
      }
      SerialBT.write('\n');
      
      fall_confirm_count++;
      fall_flag = fall_certain = fall_confirm = 0;
    }

    /*The below if condition will blink the status led twice to indicate that measuring is done, now logging will start*/
   if(measure_count == TOTAL) {
    measure_flag = 0;
    for(int i=0; i<5; i++){
          digitalWrite(STATUS, HIGH); delay(300);
          digitalWrite(STATUS, LOW);  delay(100);
        }
     }
  }
}


  /*
   * Measuring phase over 
   * Now the saved value of accleration in SRAM is loggen into SD Card
   * Below is the code for logging of data
   */
   
  else if(measure_flag == 0){

    /*Do logging*/
    SerialBT.print("FILE NO:");
    SerialBT.print(fileNo);
    SerialBT.print("\n");
    
    for(int i=0; i<TOTAL; i++){
      loggingString = "";
      loggingString += String(MILLIS[i]); loggingString += ",";
      loggingString += String(x[i]); loggingString += ",";
      loggingString += String(y[i]); loggingString += ",";
      loggingString += String(z[i]); loggingString += ",";
      loggingString += String(xg[i]); loggingString += ",";
      loggingString += String(yg[i]); loggingString += ",";
      loggingString += String(zg[i]); loggingString += "\n";
  
      Serial.print(loggingString); //to print the logged line onto the serial monitor, can be commented to increase the speed
             
      char fileNameArray[25], loggingStringArray[100];
      fileName.toCharArray(fileNameArray, 25);
      loggingString.toCharArray(loggingStringArray, 100); 
      appendFile(SD, fileNameArray, loggingStringArray);
    }

    for(int i=0; i<fall_confirm_count; i++){

      
      long height = 0.5*9.8*pow((end_millis_fall[i] - start_millis_fall[i]), 2);
      float height_actual = float(height)/1000000;

//      fallInfo = "";
//      fallInfo += "FALL DETECTED @ ";
//      fallInfo += "START MILLIS = ";
//      fallInfo += String(start_millis_fall[i]);
//      fallInfo += " END MILLIS = ";
//      fallInfo += String(end_millis_fall[i]);
//      fallInfo += " FALL HEIGHT = ";
//      fallInfo += String(height_actual);
//      fallInfo += "\n";

        fallInfo = "";
        fallInfo += String(start_millis_fall[i]);
        fallInfo += "\n";
        fallInfo += String(end_millis_fall[i]);
        fallInfo += "\n";
        fallInfo += String(height_actual);
        fallInfo += "\n";
      
      Serial.println(fallInfo);
      
      char fileNameArray[25], loggingFallInfo[150];
      fileName.toCharArray(fileNameArray, 25);
      fallInfo.toCharArray(loggingFallInfo, 150);
      appendFile(SD, fileNameArray, loggingFallInfo);  
    }
    
    measure_flag = 2;
    digitalWrite(STATUS, HIGH); //After the logging has been completed, the status LED starts to glow   
    SerialBT.print("READY");
  }
}

/*Function to append the file into SD card*/
void appendFile(fs::FS &fs, const char * path, const char * message){
    File file = fs.open(path, FILE_APPEND);
    file.print(message);
    file.close();
}
