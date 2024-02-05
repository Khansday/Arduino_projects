#define switchPin PB4
#define LEDPin PB3

#include <Wire.h>

void setup() {
  // Set PB3 as an output
  DDRB |= (1 << LEDPin);   
  DDRB &= ~(1 << switchPin);
  Wire.begin(8);                // join i2c bus with address #8
  Wire.setClock(100000);  //set speed of 1KHz

  Wire.onRequest(requestEvent); // register event
}

void loop() {
  // Read the state of the switch
  byte switchState = (PINB & (1 << switchPin)) ? HIGH : LOW;
   // Control the LED based on the switch state
  if (switchState == HIGH) {
    // Switch is ON, turn on the LED
    PORTB |= (1 << LEDPin);
  } else {
    // Switch is OFF, turn off the LED
    PORTB &= ~(1 << LEDPin);
  }
  delay(100);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  byte ledState = (PINB & (1 << PB3)) ? HIGH : LOW; // Read the state of PB3
  Wire.write(ledState); // respond with message of 6 bytes
  // as expected by master
  
}
