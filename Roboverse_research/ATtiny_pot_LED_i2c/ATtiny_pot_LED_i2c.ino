#include <Wire.h>

#define potPin PB3
#define ledPin PB1
#define WATCHDOG_TIMEOUT 5000  // Timeout in milliseconds (adjust as needed)

unsigned long lastCommunicationTime = 0;

void setup() {
  // Set PB1 (LED) as an output
  DDRB |= (1 << ledPin);   
  
  // Start I2C communication with address 9
  Wire.begin(9);

  // Register the requestEvent function to be called when master requests data
  Wire.onRequest(requestEvent);
}

void loop() {
  // Read the analog value from the potentiometer
  int potValue = analogRead(potPin);

  // Map the potentiometer reading to the LED brightness (0-255)
  int brightness = map(potValue, 0, 1023, 0, 255);

  // Set the brightness of the LED
  analogWrite(ledPin, brightness);

  // Check if communication has occurred within the timeout period
  if (millis() - lastCommunicationTime > WATCHDOG_TIMEOUT) {
    // If no communication, reset I2C and update lastCommunicationTime
    Wire.end();
    delay(10);
    Wire.begin(9);
    lastCommunicationTime = millis();
  }

  // Delay for stability
  delay(10);
}

// Function that executes whenever data is requested by the master
void requestEvent() {
  // Read the analog value from the potentiometer
  int potValue = analogRead(potPin);
  uint8_t potValue_H = potValue / 256;
  uint8_t potValue_L = potValue - (potValue_H*256);
  //uint8_t dataArray [] = {potValue_H,potValue_L};
  uint8_t dataArray [] = {2};
  // Send the potentiometer reading to the master
  //Wire.write(dataArray, sizeof(dataArray));
  Wire.write(50); // respond with message of 6 bytes
  // Update the lastCommunicationTime when communication occurs
  lastCommunicationTime = millis();
}
