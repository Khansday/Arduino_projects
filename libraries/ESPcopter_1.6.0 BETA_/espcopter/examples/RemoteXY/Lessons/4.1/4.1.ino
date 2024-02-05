// ESPcopter RemoteXY control RGB led when ESPcopter is flying

#define REMOTE_XY_OWN // custom RemoteXY App mode 

#include "remotexy.h" // RemoteXY design code
#include <espcopter.h>// ESPcopter library

void setup() {
  mainSetup();// main setup
}

void loop() {
   mainLoop();  // main flying loop
   
   if(RemoteXY.switch_1 == 1){ // if switch_1 is on do here

   if(RemoteXY.button_2 == 1){  // control blue led
   esp.blueLed_Digital(1);
   }else{
   esp.blueLed_Digital(0);
   }

   if(RemoteXY.button_3 == 1){ // control red led
   esp.redLed_Digital(1);
   }else{
   esp.redLed_Digital(0);
   }

   if(RemoteXY.button_4 == 1){ // control green led
   esp.greenLed_Digital(1);
   }else{
   esp.greenLed_Digital(0);
   }

   if(RemoteXY.button_1 == 1){  // if button_1 is on do here
   takeOff(500, 10000); //Take off 500 altitude for 25 seconds 
   land();//land 
   }
   
   }else{ // if switch_1 is off do here
   setArmControl(false);// close the motors
}
} 
