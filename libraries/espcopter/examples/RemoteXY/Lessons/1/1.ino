// ESPcopter RemoteXY single Led Control Example

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
   esp.blueLed_Digital(1); // turn off blue Led
   }else{ 
   esp.blueLed_Digital(0);// turn off blue Led
  } 
} 