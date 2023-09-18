#include <Servo.h>      //include the Servo library
Servo robotNeck;         //create servo object to control a servo motor
int robotNeck_pin = 9;

void setup() {
  pinMode(robotNeck_pin , OUTPUT);
  robotNeck.attach(robotNeck_pin); //connect the servo motor to the library
  robotNeck.write(90);             //control the anlge of the shaft which range is 0 to 180 
  delay(500);                      //give time to the shaft to move around                                  
}
void loop() {
}


