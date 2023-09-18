#define DECODE_NEC
#include <Arduino.h>
#include "PinDefinitionsAndMore.h" // Define macros for input and output pin etc.
#include <IRremote.hpp>

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
int neckPin = 3;
int a = analogRead(0); 

void setup() {
  // Serial.begin(115200);
  // Serial.println(F("PRGRAM STARTED"));
  // IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
}

void loop() {
  myservo.attach(neckPin);
  delay(1000); 
  myservo.write(135);
  delay(1000); 
  myservo.detach();
  delay(1000); 
  
  // IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  // IrReceiver.resume();
  // if (IrReceiver.decode()) {
  //   // Print a short summary of received data
  //   IrReceiver.printIRResultShort(&Serial);
  //   IrReceiver.printIRSendUsage(&Serial);
  //   if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
  //     Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
  //     // We have an unknown protocol here, print more info
  //     IrReceiver.printIRResultRawFormatted(&Serial, true);
  //   }
  myservo.attach(neckPin);
  delay(1000); 
  myservo.write(35);
  delay(1000); 
  myservo.detach();
  delay(1000); 
  
}
