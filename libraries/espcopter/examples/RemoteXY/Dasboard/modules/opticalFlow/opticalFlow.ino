// ESPcopter RemoteXY Optical Flow Sensor Dashboard

#define REMOTE_XY_OWN // custom RemoteXY App mode 

#include "remotexy.h" // RemoteXY design code
#include <espcopter.h>// ESPcopter library


void setup() {
  mainSetup(); // main setup
}

void loop() {
   mainLoop();  // main flying loop

   int xPositive =  constrain(getFilterOptData_X(),0,50000);  // Convert data to negative and positive part
   int xNegative=  -1*constrain(getFilterOptData_X(),-50000,0);  // Convert data to negative and positive part

   int yPositive =  constrain(getFilterOptData_Y(),0,50000);    // Convert data to negative and positive part
   int yNegative=  -1*constrain(getFilterOptData_Y(),-50000,0);  // Convert data to negative and positive part


   RemoteXY.level_1 = map(yPositive,0,50000,0,100);  // convert sensor range and send data to app
   RemoteXY.level_2 = map(xPositive,0,50000,0,100);  // convert sensor range and send data to app
   RemoteXY.level_3 = map(yNegative,0,50000,0,100);  // convert sensor range and send data to app
   RemoteXY.level_4 = map(xNegative,0,50000,0,100);  // convert sensor range and send data to app
} 
