/*
THIS CODE switches ON and OFF an LED 
*/
int LED_blue = 13;               //store the pin number where the LED is connected
tches// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(LED, OUTPUT);     //set pin 5 to send signal
  pinMode(LED_red, OUTPUT);     //set pin 3 to send signal

 // pinMode(PIN NUMBER, MODE / OUT/IN);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(500);               // wait for half second
  digitalWrite(LED, LOW);   // turn the LED off by making the voltage LOW
  delay(500);               // wait for half second
  digitalWrite(3, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(500);               // wait for half second
  digitalWrite(3, LOW);   // turn the LED off by making the voltage LOW
  delay(500);   
}
