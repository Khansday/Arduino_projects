// ESPcopter RemoteXY RGB Led Control  Example

#define REMOTE_XY_OWN // custom RemoteXY App mode 

#include "remotexy.h" // RemoteXY design code
#include <espcopter.h>// ESPcopter library

void setup() {
  mainSetup(); // main setup for escopter 
  esp.greenLed_Digital(0); // turn off green Led
}

void loop() {
	
   mainLoop();  // main flying loop
   
   if(RemoteXY.button_1 == 1){ // if switch_1 is on do here
    esp.redLed_Digital(1); // turn on red Led
   }else{ 
    esp.redLed_Digital(0);// turn off red Led
   }
   
   if(RemoteXY.button_2 == 1){ // if switch_1 is on do here
    esp.greenLed_Digital(1); // turn on green Led
   }else{ 
    esp.greenLed_Digital(0);// turn off green Led
   }
   
   if(RemoteXY.button_3 == 1){ // if switch_1 is on do here
    esp.blueLed_Digital(1); // turn on blue Led
   }else{ 
    esp.blueLed_Digital(0);// turn off blue Led
   }
   
} 
