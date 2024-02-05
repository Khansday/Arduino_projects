// ESPcopter IMU Sensor temperature Sensor Dashboard

#define REMOTE_XY_OWN // custom RemoteXY App mode   

#include "remotexy.h" // RemoteXY design code
#include <espcopter.h>// ESPcopter library

void setup() {
  mainSetup(); // main setup
}

void loop() {
   mainLoop();  // main flying loop
   float val = getMpuTemp(); // get temperature data
   dtostrf(val, 0, 2, RemoteXY.text_1); // convert and send data to remoteXY app 
} 

