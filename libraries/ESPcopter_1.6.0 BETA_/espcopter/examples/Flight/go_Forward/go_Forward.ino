//  8 second flight code
//#define STANDALONE// 

#define FREECONTROL // no wiffi connection

#include "espcopter.h" // library

void setup() {
mainSetup(); // main flying setup
delay(5000); // wait 5 second before flight
}

void loop() {
mainLoop (); // main flying loop

takeOff(500,8000); // 500 mm alittute total flight time is 8000 milisecond
delay_(5000); // 3 second wait
goForward(1000); // 1 second forward 
delay_(2000); // 3 second wait
land() ; //  land
}