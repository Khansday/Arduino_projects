// ESPcopter RemoteXY. Control Forward and backward motion with 2 buttons 

#define REMOTE_XY_OWN // custom RemoteXY App mode 

#include "remotexy.h" // RemoteXY design code
#include <espcopter.h>// ESPcopter library


void setup() {
  mainSetup(); // main setup 
}

void loop() {
   
   mainLoop();  // main flying loop
   
   if(RemoteXY.switch_1 == 1){ // if switch_1 is on do here

    if(RemoteXY.button_2 == 1){ // go forward
    SetOptPoint_Y(15); // set motion 
    }else if(RemoteXY.button_3 == 1){  // go back
    SetOptPoint_Y(-15); // set motion 
    }else{
	SetOptPoint_Y(0); // stay
    }

   if(RemoteXY.button_1 == 1){  // if button_1 is on do here
   takeOff(500, 10000); //Take off 500 altitude for 25 seconds 
   land();//land 
   }
   
   
   }else{ // if switch_1 is off do here
   setArmontrol(false);// close the motors
}
} 
