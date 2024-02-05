// ESPcopter RemoteXY BME280  Sensor Dashboard

#define REMOTE_XY_OWN // custom RemoteXY App mode  

#include "remotexy.h" // RemoteXY design code
#include <espcopter.h>// ESPcopter library


void setup() {
  mainSetup(); // main setup
}

void loop() {
   mainLoop();  // main flying loop
   RemoteXY.onlineGraph_1 = getBmeTemp(); // get temperature data and send to app
   RemoteXY.onlineGraph_2 = getBmePressure(); // get pressure data and send to app
   RemoteXY.onlineGraph_3 = getBmeHumidity(); // get humidity data and send to app
} 