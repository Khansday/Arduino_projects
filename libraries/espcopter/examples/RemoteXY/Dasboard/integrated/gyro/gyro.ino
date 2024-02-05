// ESPcopter RemoteXY Gyro Sensor Dashboard

#define REMOTE_XY_OWN // custom RemoteXY App mode   

#include "remotexy.h" // RemoteXY design code
#include <espcopter.h>// ESPcopter library


void setup() {
  mainSetup(); // main setup
}

void loop() {
   mainLoop();  // main flying loop
   RemoteXY.onlineGraph_1_var1 = getMpuGyroX(); // Get X Gyro Data and send to app
   RemoteXY.onlineGraph_1_var2 = getMpuGyroY(); // Get Y Gyro Data and send to app
   RemoteXY.onlineGraph_1_var3 = getMpuGyroZ(); // Get Z Gyro Data and send to app
} 

