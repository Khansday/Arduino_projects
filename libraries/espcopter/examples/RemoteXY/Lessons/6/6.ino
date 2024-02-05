// ESPcopter RemoteXY. Control Neopixel Led with phone app, When ESPcopter is flying 

#define REMOTE_XY_OWN // custom RemoteXY App mode  

#include "remotexy.h" // RemoteXY design code
#include <espcopter.h>// ESPcopter library


void setup() {
  mainSetup();// main setup
}

void loop() {
   mainLoop();  // main flying loop
   
   if(RemoteXY.switch_1 == 1){ // if switch_1 is on do here
   
   int red = RemoteXY.rgb_1_r; // get red value
   int blue = RemoteXY.rgb_1_b; // get blue value
   int green =  RemoteXY.rgb_1_g; // get green value
   
   for(int i ; i < 12; i++){ //control 12 leds
    pixels.setPixelColor(i, pixels.Color(red,green,blue));
   }
   pixels.show(); 

   if(RemoteXY.button_1 == 1){  // if button_1 is on do here
   takeOff(500, 25000); //Take off 500 altitude for 25 seconds 
   land();//land 
   }
   
   }else{ // if switch_1 is off do here
   setArmControl(false);// close the motors
}
} 