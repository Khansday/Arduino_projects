/*
 * SimpleReceiver.cpp
 *
 * Demonstrates receiving NEC IR codes with IRrecv
 *
 *  This file is part of Arduino-IRremote https://github.com/Arduino-IRremote/Arduino-IRremote.
 *
 ************************************************************************************
 * MIT License
 *
 * Copyright (c) 2020-2022 Armin Joachimsmeyer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ************************************************************************************
 */

/*
 * Specify which protocol(s) should be used for decoding.
 * If no protocol is defined, all protocols are active.
 */
//#define DECODE_DENON        // Includes Sharp
//#define DECODE_JVC
//#define DECODE_KASEIKYO
//#define DECODE_PANASONIC    // the same as DECODE_KASEIKYO
//#define DECODE_LG
#define DECODE_NEC          // Includes Apple and Onkyo
//#define DECODE_SAMSUNG
//#define DECODE_SONY
//#define DECODE_RC5
//#define DECODE_RC6

//#define DECODE_BOSEWAVE
//#define DECODE_LEGO_PF
//#define DECODE_MAGIQUEST
//#define DECODE_WHYNTER

//#define DECODE_DISTANCE     // universal decoder for pulse distance protocols
//#define DECODE_HASH         // special decoder for all protocols

//#define DEBUG               // Activate this for lots of lovely debug output from the decoders.

#include <Arduino.h>

#include "PinDefinitionsAndMore.h" // Define macros for input and output pin etc.
#include <IRremote.hpp>

int M1A = 8 , M1B = 9, M2A = 10, M2B = 11  ;


void M1_forward(){
  digitalWrite(M1A , HIGH);
  digitalWrite(M1B , LOW);

}

void M2_forward(){
  digitalWrite(M2A , HIGH);
  digitalWrite(M2B , LOW);
}

void M1_backwards(){
  digitalWrite(M1A , LOW);
  digitalWrite(M1B , HIGH);
}

void M2_backwards(){
  digitalWrite(M2A , LOW);
  digitalWrite(M2B , HIGH);
}

void M1_stop(){
  digitalWrite(M1A , LOW);
  digitalWrite(M1B , LOW);
}

void M2_stop(){
  digitalWrite(M2A , LOW);
  digitalWrite(M2B , LOW);
}

void setup() {
    Serial.begin(115200);
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

    // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

    Serial.print(F("Ready to receive IR signals of protocols: "));
    printActiveIRProtocols(&Serial);
    Serial.println(F("at pin " STR(IR_RECEIVE_PIN)));

    
    pinMode( M1A , OUTPUT);
    digitalWrite(M1A , HIGH);
    pinMode( M1B , OUTPUT);
    digitalWrite(M1B , HIGH);
    delay(2000);
    
    digitalWrite(M1A , LOW);
    digitalWrite(M1B , LOW);

    pinMode( M2A , OUTPUT);
    pinMode( M2B , OUTPUT);
}

void loop() {
    /*
     * Check if received data is available and if yes, try to decode it.
     * Decoded result is in the IrReceiver.decodedIRData structure.
     *
     * E.g. command is in IrReceiver.decodedIRData.command
     * address is in command is in IrReceiver.decodedIRData.address
     * and up to 32 bit raw data in IrReceiver.decodedIRData.decodedRawData
     */
    if (IrReceiver.decode()) {

        // Print a short summary of received data
        IrReceiver.printIRResultShort(&Serial);
        IrReceiver.printIRSendUsage(&Serial);
        if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
            Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
            // We have an unknown protocol here, print more info
            IrReceiver.printIRResultRawFormatted(&Serial, true);
        }
        Serial.println();

        /*
         * !!!Important!!! Enable receiving of the next value,
         * since receiving has stopped after the end of the current received data packet.
         */
        IrReceiver.resume(); // Enable receiving of the next value

        /*
         * Finally, check the received data and perform actions according to the received command
         */
        if (IrReceiver.decodedIRData.command == 0x18) {  //arrow up remote
            M1_forward();

            M2_forward();

        } else if (IrReceiver.decodedIRData.command == 0x52) { //arrow down remote
            M1_backwards();

            M2_backwards();
        
        } else if (IrReceiver.decodedIRData.command == 0x8) { //arrow left remote
            M1_stop();

            M2_forward();

        } else if (IrReceiver.decodedIRData.command == 0x5A) { //arrow right remote
            M1_forward();

            M2_stop();
        } else if (IrReceiver.decodedIRData.command == 0x1C) { //OK button remote
            M1_stop();

            M2_stop();
        }
    }
}


