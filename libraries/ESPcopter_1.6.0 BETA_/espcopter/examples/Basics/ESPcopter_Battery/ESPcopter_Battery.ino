
//#define STANDALONE

#define FREECONTROL // no wiffi connection


#include <espcopter.h> // library


void setup() {
  mainSetup(); // main flying setup
}

void loop() {
   mainLoop ();  // main flying loop
   
   Serial.print("Battery Voltage: ");
   Serial.print(getBatteryVoltage());
   Serial.print("Battery Level: ");
   Serial.println(getBatteryLevel());
  
}