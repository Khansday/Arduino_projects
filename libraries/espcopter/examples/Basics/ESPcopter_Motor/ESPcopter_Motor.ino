// Control espcopter Motors

#define STANDALONE

#include <espcopter.h> // library


void setup() {
  mainSetup(); // main flying setup
}

void loop() {
 mainLoop ();  // main flying loop
 
 setMotorSpeedFL(50); // 0-255 %20 power
 delay(1000);
 setMotorSpeedFL(0); //  %0 power
 setMotorSpeedFR(50); // 0-255 %20 power
 delay(1000);
 setMotorSpeedFR(0); //  %0 power
 setMotorSpeedRL(50); // 0-255 %20 power
 delay(1000); 
 setMotorSpeedRL(0); //  %0 power
 setMotorSpeedRR(50); // 0-255 %20 power
 delay(1000);
 setMotorSpeedRR(0); //  %0 power
 
 delay(1000);
 setMotorSpeedFL(50); // 0-255 %20 power
 setMotorSpeedFR(50); // 0-255 %20 power
 setMotorSpeedRL(50); // 0-255 %20 power
 setMotorSpeedRR(50); // 0-255 %20 power
 delay(1000);
 
 // Exit the loop 
 exit(0); 
}