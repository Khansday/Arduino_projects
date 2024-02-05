// ESPcopter RemoteXY RGB Led Control  Example

#define REMOTE_XY_OWN // custom RemoteXY App mode 

#include "remotexy.h" // RemoteXY design code
#include <espcopter.h>// ESPcopter library

void setup() {
  mainSetup(); // main setup for escopter 
}

void loop() {
	
   mainLoop();  // main flying loop
   
   if(RemoteXY.switch_1 == 1){
   setMotorSpeedFL(map(RemoteXY.slider_1,0,100,0,255)); // 0-255 
   setMotorSpeedFR(map(RemoteXY.slider_2,0,100,0,255)); // 0-255 
   setMotorSpeedRL(map(RemoteXY.slider_3,0,100,0,255)); // 0-255 
   setMotorSpeedRR(map(RemoteXY.slider_4,0,100,0,255)); // 0-255 
   }else {
   setMotorSpeedFL(0); // 0-255 
   setMotorSpeedFR(0); // 0-255 
   setMotorSpeedRL(0); // 0-255 
   setMotorSpeedRR(0); // 0-255    
   }
    
} 