#define M1_d 7
#define M1_dr 8
#define M1_speed 6
#define M2_d 9
#define M2_dr 10
#define M2_speed 11
#define min_speed 35


int pwm = 0;
int dir;
void setSpeed(){
  analogWrite(M1_speed , pwm);  //set motor speed 
  analogWrite(M2_speed , pwm);  //set motor speed 
}

void Fwd(){
  digitalWrite(M1_d , HIGH);
  digitalWrite(M1_dr , LOW);

  digitalWrite(M2_d , HIGH);
  digitalWrite(M2_dr , LOW);
}

void Bwd(){
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , HIGH);

  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , HIGH);
}



void setup()
{
//set motor driver pin as output
pinMode(M1_d , OUTPUT);
pinMode(M1_dr , OUTPUT);
pinMode(M1_speed , OUTPUT);
pinMode(M2_d , OUTPUT);
pinMode(M2_dr , OUTPUT);
pinMode(M2_speed , OUTPUT);
pinMode(5 , OUTPUT);

// make surre the motors are OFF

digitalWrite(M1_d , LOW);
digitalWrite(M1_dr , LOW);
digitalWrite(M1_speed , LOW);
digitalWrite(M2_d , LOW);
digitalWrite(M2_dr , LOW);
digitalWrite(M2_speed , LOW);
digitalWrite(5 , 0);

Serial.begin(115200);
Fwd();
}



void loop(){
  Serial.println("Insert direction: 1)fwd  2)bckwd");
  while (Serial.available()) Serial.read();
  while (Serial.available() == 0) {
  }
  dir = Serial.parseInt();
  if (dir == 1){
    Fwd();
  } 
  else {
    Bwd();
  }
  delay(1000);
  Serial.println("Insert speed: ");
  //delay(2000);
  while (Serial.available()) Serial.read();
  while (Serial.available() == 0) {
  }
  pwm = Serial.parseInt();
  Serial.println(pwm);
  setSpeed();
//delay(1000);
}

