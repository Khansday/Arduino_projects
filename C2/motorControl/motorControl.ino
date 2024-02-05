int IN1 = 5;                //direction  pin 
int IN2  = 6;               //direction  pin 
int ENA = 3;               //speed PWM pin

int IN3 = 5;                //direction  pin 
int IN4  = 6;               //direction  pin 
int ENB = 3;               //speed PWM pin

void setup() {
  pinMode(IN1, OUTPUT);     //set direction pin to send signal
  pinMode(IN2, OUTPUT);     //set direction pin to send signal
  pinMode(ENA, OUTPUT);     //set speed PWM pin to send signal
  pinMode(IN3, OUTPUT);     //set direction pin to send signal
  pinMode(IN4, OUTPUT);     //set direction pin to send signal
  pinMode(ENB, OUTPUT);     //set speed PWM pin to send signal
}

void loop() {
  digitalWrite(IN1, HIGH);  //changing HIGH,LOW will make the motor spin or stop 
  digitalWrite(IN2, LOW); 
  analogWrite(ENA, 100);    //speed ranges from 0 to 255
  digitalWrite(IN3, HIGH);  //changing HIGH,LOW will make the motor spin or stop 
  digitalWrite(IN4, LOW); 
  analogWrite(ENB, 100);    //speed ranges from 0 to 255
  delay(5000);
  digitalWrite(IN1, LOW);  //changing HIGH,LOW will make the motor spin or stop 
  digitalWrite(IN2, HIGH); 
  analogWrite(ENA, 100);    //speed ranges from 0 to 255
  digitalWrite(IN3, LOW);  //changing HIGH,LOW will make the motor spin or stop 
  digitalWrite(IN4, HIGH); 
  analogWrite(ENB, 200);    //speed ranges from 0 to 255
  delay(200);
    
}

