//  5 second flight code
//#define STANDALONE// 

#define FREECONTROL // no wifi connection
#include "espcopter.h" // library
void setup() {
mainSetup(); // main flying setup
delay(5000); // wait 5 second after before flight
}

void loop() {
mainLoop (); // main flying loop

takeOff(500,5000); // 500 mm alittute total flight time is 5000 milisecond
delay_(5000); // 5 second wait
land() ; //  land
}