uint8_t a = 101;   //message

#include <Wire.h>

void setup() {
  // Set PB3 as an output
  DDRB |= (1 << DDB3);

  Wire.begin(10);                // join i2c bus with address #
  //Wire.setClock(100000);  //set speed of 1KHz

  Wire.onRequest(requestEvent); // register event
}

void loop() {
  //PORTB ^= (1 << PB3);  //toggle LED
  delay(1);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {

  Wire.write(a); // respond with message 
  // as expected by master
  
}
