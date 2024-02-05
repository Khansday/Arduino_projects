
void setup() {
  // put your setup code here, to run once:
  // Set PB3 as an output
  DDRB |= (1 << DDB3);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Toggle the LED state
  PORTB ^= (1 << PB3);

  // Wait for a second
  delay(1000);

}
