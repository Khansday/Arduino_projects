
uint8_t a;
uint8_t addrs = 0x0a;
#include <Wire.h>

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Wire.setClock(100000);  //set speed of 1KHz
  Serial.begin(115200);  // start serial for output
  
  Serial.println("Program started");         // print the character
  delay (2000);
}

void loop() {
  Wire.requestFrom(addrs, 1);    // request 1 bytes from slave device #8

  while (Wire.available()) { // slave may send less than requested
    a = Wire.read(); // receive a byte as character
    Serial.println(a);         // print the character
  }

  delay(500);
}
