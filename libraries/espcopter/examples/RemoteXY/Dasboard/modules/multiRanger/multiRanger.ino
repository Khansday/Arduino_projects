// ESPcopter RemoteXY Multi-Ranger Sensor Dashboard

#define REMOTE_XY_OWN // custom RemoteXY App mode 

#include "remotexy.h" // RemoteXY design code
#include <espcopter.h>// ESPcopter library


void setup() {
  mainSetup(); // main setup
}

void loop() {
   mainLoop();  // main flying loop
   RemoteXY.level_1 = map(Distance_X_0(),0,1200,0,100); // convert sensor range and send data to app
   RemoteXY.level_2 = map(Distance_Y_0(),0,1200,0,100); // convert sensor range and send data to app
   RemoteXY.level_3 = map(Distance_X_1(),0,1200,0,100); // convert sensor range and send data to app
   RemoteXY.level_4 = map(Distance_Y_1(),0,1200,0,100); // convert sensor range and send data to app
} 

