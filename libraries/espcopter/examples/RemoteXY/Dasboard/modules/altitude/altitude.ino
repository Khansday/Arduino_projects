// ESPcopter RemoteXY Altitude Sensor Dashboard

#define REMOTE_XY_OWN // custom RemoteXY App mode 

#include "remotexy.h" // RemoteXY design code
#include <espcopter.h>// ESPcopter library


void setup() {
  mainSetup(); // main setup
}

void loop() {
   mainLoop();  // main flying loop
   RemoteXY.level_1 = map(getMeasureAltitude(),0,1200,0,100);  // convert data range and send to app
} 