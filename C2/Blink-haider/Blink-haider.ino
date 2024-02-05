/*
THIS CODE switches ON and OFF an LED 
*/
int LED = 13;               //store the pin number where the LED is connected

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(LED, OUTPUT);     //set pin 5 to send signal
 // pinMode(PIN NUMBER, MODE / OUT/IN);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(500);               // wait for half second
  digitalWrite(LED, LOW);   // turn the LED off by making the voltage LOW
  delay(500);               // wait for half second
}
