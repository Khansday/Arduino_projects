
uint8_t a;
#include <Wire.h>

void setup() {
  Wire.begin(0x9);        // join i2c bus (address optional for master)
  
  delay (2000);
}

void loop() {

  delay(500);
}
