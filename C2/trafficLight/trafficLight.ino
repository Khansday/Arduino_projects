int redLight = 10;
int yellowLight = 9;
int greenLight = 8;
int motor1 = 7; 
int motor2 = 6;
int motorSpeed = 5;
int buzzer = 11;


void setup() {
  //put all pins chosen as OUTPUT
  pinMode(redLight, OUTPUT);
  pinMode(yellowLight, OUTPUT);
  pinMode(greenLight, OUTPUT);
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motorSpeed, OUTPUT);
  pinMode(buzzer, OUTPUT);
  // Make sure the pins are off at start
  digitalWrite(redLight, LOW);
  digitalWrite(yellowLight, LOW);
  digitalWrite(greenLight, LOW);
  digitalWrite(motor1, LOW);
  digitalWrite(motor2, LOW);
  digitalWrite(motorSpeed, LOW);
  digitalWrite(buzzer, LOW);
}

void loop() {
  digitalWrite(redLight, HIGH);     //red light ON
  digitalWrite(motor1, LOW);        //motor stops
  digitalWrite(motor2, LOW);
  digitalWrite(motorSpeed, LOW);
  delay(1000);                      //wait 1 sec

  digitalWrite(redLight, LOW);      //red light OFF
  digitalWrite(yellowLight, HIGH);  //yellow light ON
  digitalWrite(motor1, HIGH);       // motor spin fwd
  digitalWrite(motor2, LOW);
  analogWrite(motorSpeed, 127);     // motor half speed
  delay(2000);                      //wait 2 sec

  digitalWrite(yellowLight, LOW);   //yellow light OFF
  digitalWrite(greenLight, HIGH);   //green light ON
  digitalWrite(motor1, HIGH);       // motor spin fwd
  digitalWrite(motor2, LOW);
  analogWrite(motorSpeed, 255);     //motor full speed
  delay(1000);                      //wait 1 sec

  tone(buzzer, 4000, 1000);     //rude driver honking for 1 sec
  noTone(buzzer);                   //disconnect buzzer
  delay(4000);                      //wait 4 sec
  digitalWrite(greenLight, LOW);    //green light OFF
}
