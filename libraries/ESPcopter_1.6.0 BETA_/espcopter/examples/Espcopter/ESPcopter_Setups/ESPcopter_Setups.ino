/*
 * www.espcopter.com/learn
 * 
*/

//select usage mode manually 
#define STANDALONE
//#define REMOTE_XY_OWN 
//#define FREECONTROL

//write your wifi SSID and pass manually 
#define ESPCOPTER_WIFI_SSID "your-ssid"
#define ESPCOPTER_WIFI_PASSWORD "your-pass" // it should be more than 8 character, Otherwise, there will be no password
 
//select connection mode manually 
#define AP_Mode 
//#define STA_Mode 


#include "espcopter.h" // library


void setup() {

  mainSetup(); // main flying setup

  ahrs.calibraiton(gyro_Accel_Calibraiton); // manually calibration for accelerometer and gyroscope 
  
  ahrs.resetAutoCalibraitons(); // reset automatic calibration data
  //optical flow  sensor is working with automatic calibration And then, it use
  //drone is collecting optical flow data in the first of the 5 second of the flight each time.
  //It is using this data next take off to make more stable  take off
  
}

void loop() {
   mainLoop ();  // main flying loop
 }

