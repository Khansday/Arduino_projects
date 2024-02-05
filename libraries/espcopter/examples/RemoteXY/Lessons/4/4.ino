// ESPcopter RemoteXY control LED when ESPcopter is flying

#define REMOTE_XY_OWN // custom RemoteXY App mode 

#include "remotexy.h" // RemoteXY design code
#include <espcopter.h>// ESPcopter library

void setup() {
  mainSetup(); // main setup
}

void loop() {
   mainLoop();  // main flying loop
   
   if(RemoteXY.switch_1 == 1){ // if switch_1 is on do here
    
   if(RemoteXY.button_2 == 1){  // control LED 
   esp.greenLed_Digital(1);
   }else{
   esp.greenLed_Digital(0);
   }
   
   if(RemoteXY.button_1 == 1){  // if button_1 is on do here
   takeOff(500, 25000); //Take off 500 altitude for 25 seconds 
   land();//land 
   }
   
   }else{ // if switch_1 is off do here
   setArmControl(false);// close the motors
}
} 