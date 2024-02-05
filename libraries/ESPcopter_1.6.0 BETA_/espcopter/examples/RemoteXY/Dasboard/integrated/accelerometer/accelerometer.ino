// ESPcopter RemoteXY Accelerometer Sensor Dashboard

#define REMOTE_XY_OWN // custom RemoteXY App mode     

#include "remotexy.h" // RemoteXY design code
#include <espcopter.h>// ESPcopter library


void setup() {
  mainSetup(); // main setup
}

void loop() {
   mainLoop();  // main flying loop
   RemoteXY.onlineGraph_1_var1 = getMpuAccelX(); // Get X Accelerometer Data and send to app
   RemoteXY.onlineGraph_1_var2 = getMpuAccelY(); // Get Y Accelerometer Data and send to app
   RemoteXY.onlineGraph_1_var3 = getMpuAccelZ(); // Get Z Accelerometer Data and send to app
} 