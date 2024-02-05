// ESPcopter RemoteXY. ESPcopter custom app and obstacle avoidance code with multi-ranger module

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
     if(Distance_Y_1() > 500){  // check y+ sensor data 
     SetOptPoint_Y(-15); // if distance is more than 500mm go forward 
     }
     }else if(RemoteXY.button_3 == 1){ // go back
     if(Distance_Y_0() > 500){ // check y- sensor data
     SetOptPoint_Y(15); // if distance is more than 500mm go back
     }
     }else{
     SetOptPoint_Y(0); // stay
     }
    
   
    if(RemoteXY.button_4 == 1){ // go right
    if(Distance_X_0() > 500){ // check x+ sensor data 
    SetOptPoint_X(-15); // if distance is more than 500mm go right 
    }
    }else if(RemoteXY.button_5 == 1){ // go left
    if( Distance_X_1() > 500){ // check x- sensor data 
    SetOptPoint_X(15); // if distance is more than 500mm go left 
    }
    }else{
    SetOptPoint_X(0); // stay
   }
  
   takeOff(500, 30000); //Take off 500 altitude for 25 seconds 
   land();//land 

   }else{ // if switch_1 is off do here
   setArmontrol(false);// close the motors
}
} 
