//  6 second flight code
//#define STANDALONE// 

#define FREECONTROL // no wiffi connection

#include "espcopter.h" // library

void setup() {
mainSetup(); // main flying setup
delay(5000); // wait 5 second before flight
}

void loop() {
mainLoop (); // main flying loop
setFlyMode_1(true);
takeOff(500,14000); // 500 mm alittute total flight time is 14000 milisecond
delay_(5000); // 3 second wait
turnRight(90);
delay_(5000); // 3 second wait
land(); //  land

}