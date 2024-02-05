// ESPcopter RemoteXY Control altitude with 2 bottom when it is flying

#define REMOTE_XY_OWN // custom RemoteXY App mode  

#include "remotexy.h" // RemoteXY design code
#include <espcopter.h>// ESPcopter library

int controlButton_1 = 0; // button control checker 
int controlButton_2 = 0; // button control checker 

int changeAltitute = 500; // starting altitude 


void setup() {
  mainSetup(); // main setup
}

void loop() {
   mainLoop();  // main flying loop
   
   if(RemoteXY.switch_1 == 1){ // if switch_1 is on do here

     if(RemoteXY.button_2 == 1){
     if(controlButton_1 == 0){
     changeAltitute = changeAltitute + 100; // increase 100mm
     setTargetOto(changeAltitute);//set altitude  
     controlButton_1 =1; 
     }
     }else{
     controlButton_1 =0; 
     }
     
     if(RemoteXY.button_3 == 1){
     if(controlButton_2 == 0){
     changeAltitute = changeAltitute - 100; // decrease 100mm
     setTargetOto(changeAltitute);//set altitude  
     controlButton_2 = 1; 
     }
     }else{
     controlButton_2 = 0; 
     }

   if(RemoteXY.button_1 == 1){  // if button_1 is on do here
   takeOff(500, 25000); //Take off 500 altitude for 25 seconds 
   land();//land 
   }
   }else{ // if switch_1 is off do here
   setArmControl(false);// close the motors
}
} 