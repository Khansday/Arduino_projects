int IN3 = 5;                //direction  pin 
int IN4  = 6;               //direction  pin 
int ENB = 3;               //speed PWM pin

void setup() {
  pinMode(IN1, OUTPUT);     //set direction pin to send signal
  pinMode(IN2, OUTPUT);     //set direction pin to send signal
  pinMode(ENA, OUTPUT);     //set speed PWM pin to send signal

  digitalWrite(IN1, HIGH);  //changing HIGH,LOW will make the motor spin or stop 
  digitalWrite(IN2, LOW); 
  analogWrite(ENA, 100);    //speed ranges from 0 to 255
}

void loop() {
  //leave the loop function empty
}
