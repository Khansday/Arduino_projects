
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

void setup() {
    Serial.begin(115200);
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

    // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

    Serial.print(F("Ready to receive IR signals of protocols: "));
    printActiveIRProtocols(&Serial);
    Serial.println(F("at pin " STR(IR_RECEIVE_PIN)));
}

void loop() {
  IrReceiver.decode();
  IrReceiver.resume(); 
  button = IrReceiver.decodedIRData.command;

  switch(button){
    case (0x45):    //button 1
    //do something
    // MAKE THE ROBOT SELF DRIVING
    break;
    case (0x9):    //button 9
    //do something
    //MAKE THE ROBOT LINE FOLLOWER
    break; 
  }
}




