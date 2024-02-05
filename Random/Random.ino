int M1_p = 7;  //pins to control motor 1
int M1_n = 8;
int M1_s = 6;
int M2_p = 12;  //pins to control motor 2
int M2_n = 11;
int M2_s = 5;


#define DECODE_NEC          // Includes Apple and Onkyo

#define IR_RECEIVE_PIN      2
#define IR_SEND_PIN        13
#define TONE_PIN            4
#define APPLICATION_PIN     5
#define ALTERNATIVE_IR_FEEDBACK_LED_PIN 6 // E.g. used for examples which use LED_BUILDIN for example output.
#define _IR_TIMING_TEST_PIN 7

/*
 * Helper macro for getting a macro definition as string
 */
#if !defined(STR_HELPER)
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#endif

#include <Arduino.h>

/*
 * This include defines the actual pin number for pins like IR_RECEIVE_PIN, IR_SEND_PIN for many different boards and architectures
 */
//#include "PinDefinitionsAndMore.h"
#include <IRremote.hpp> // include the library
int button;

void go_Forward(){
  Serial.println("Car forward");  //show this message on the screen
  digitalWrite(M1_p , HIGH);      //motor 1 fwd
  digitalWrite(M1_n , LOW);
  analogWrite(M1_s , 180);     

  digitalWrite(M2_p , HIGH);      //mnotor 2 fwd
  digitalWrite(M2_n , LOW);
  analogWrite(M2_s , 180); 
}

void setup() {
  pinMode(M1_p , OUTPUT);  //setting the pins going to the motor driver as output
  pinMode(M1_n , OUTPUT);
  pinMode(M1_s , OUTPUT);
  pinMode(M2_p , OUTPUT);
  pinMode(M2_n , OUTPUT);
  pinMode(M2_s , OUTPUT);

  Serial.begin(9600);     //start the serial monitor

      // Just to know which program is running on my Arduino
   // Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

    // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

   // Serial.print(F("Ready to receive IR signals of protocols: "));
    printActiveIRProtocols(&Serial);
   // Serial.println(F("at pin " STR(IR_RECEIVE_PIN)));

  go_Forward();           //call function to move forward
}

void loop() {
    IrReceiver.decode();
  IrReceiver.resume(); // Enable receiving of the next value
  button = IrReceiver.decodedIRData.command;
  switch(button){
    case (0x45):    //button 1
    //do something
  go_Forward();           //call function to move forward
    break;
    case (0x9):    //button 9
    //do something
  go_Left();
    break;
  }
}    




  void go_Backward(){
  Serial.println("Car backward");
  digitalWrite(M1_p , LOW); //motor1 bkwd
  digitalWrite(M1_n , HIGH);
  analogWrite(M1_s , 80);  //low speed

  digitalWrite(M2_p , LOW); //motor 2 bkwd
  digitalWrite(M2_n , HIGH);
  analogWrite(M2_s , 80); //low speed
}

void go_Left(){
  Serial.println("Car left");
  digitalWrite(M1_p , LOW); //motor1 stop
  digitalWrite(M1_n , LOW);
  analogWrite(M1_s , 80);  //low speed

  digitalWrite(M2_p , HIGH); //mnotor 2 fwd
  digitalWrite(M2_n , LOW);
  analogWrite(M2_s , 80); //low speed
}

void go_Right(){
  Serial.println("Car right");
  digitalWrite(M1_p , HIGH); //motor1 fwd
  digitalWrite(M1_n , LOW);
  analogWrite(M1_s , 80);  //low speed

  digitalWrite(M2_p , LOW); //mnotor 2 stop
  digitalWrite(M2_n , LOW);
  analogWrite(M2_s , 80); //low speed
}

void stop(){
  Serial.println("Car stop");
  digitalWrite(M1_p , LOW); //motor1 fwd
  digitalWrite(M1_n , LOW);
  analogWrite(M1_s , 80);  //low speed

  digitalWrite(M2_p , LOW); //mnotor 2 stop
  digitalWrite(M2_n , LOW);
  analogWrite(M2_s , 80); //low speed
}   
