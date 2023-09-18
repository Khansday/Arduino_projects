int switch_pin = 2;   //switch on pin 2 

int switch_state;     // variable to see the switch position

void setup() {
  pinMode(switch_pin, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop() {
  switch_state = digitalRead(switch_pin);
  Serial.println(!switch_state);
}

