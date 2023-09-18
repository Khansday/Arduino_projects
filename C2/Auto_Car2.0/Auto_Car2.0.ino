 #include <Servo.h>

Servo robotNeck;  // create servo object to control a servo

#define M1_d 4
#define M1_dr 5
#define M1_speed 3
#define M2_d 11
#define M2_dr 10
#define M2_speed 9
#define STBY 6

int motorSpeed =130; 

int robotNeck_pin = 13; 

int distanceSensor = A0; 
int distanceValue;
int maxDistance = 200;
int LED_red = 7;
int LED_white = 8;
int buzzer = 12;
int Pwr = 0; 

int standby = 0;

void goForward(){
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , HIGH);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , HIGH);
  analogWrite(M2_speed, motorSpeed);
  digitalWrite(LED_white , LOW);
  digitalWrite(LED_red , LOW); 
}

void goBackwards(){
  digitalWrite(M1_d , HIGH);
  digitalWrite(M1_dr , LOW);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , HIGH);
  digitalWrite(M2_dr , LOW);
  analogWrite(M2_speed, motorSpeed);
  digitalWrite(LED_white , LOW);
  digitalWrite(LED_red , LOW);  
}

void turnLeft(){
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , LOW);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , HIGH);
  digitalWrite(M2_dr , LOW);
  analogWrite(M2_speed, motorSpeed);
  digitalWrite(LED_red , HIGH);
  digitalWrite(LED_white , HIGH);
} 

void turnRight(){
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , HIGH);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , LOW);
  analogWrite(M2_speed, motorSpeed);
  digitalWrite(LED_white , HIGH);
  digitalWrite(LED_red , HIGH); 
}

void stop(){
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , LOW);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , LOW);
  analogWrite(M2_speed, motorSpeed);
  digitalWrite(LED_white , LOW);
  digitalWrite(LED_red , LOW); 
}

void makeTurn(){
  int distanceLeft;
  int distanceRight;

  digitalWrite(LED_white , HIGH);
  goBackwards();
  delay(500);
  digitalWrite(LED_white , LOW);
  stop();
  
  robotNeck.write(145);
  delay(500);
  distanceLeft =  analogRead(distanceSensor);
  delay(500);
  robotNeck.write(35);
  delay(500);
  distanceRight =  analogRead(distanceSensor);
  delay(500);
  
  robotNeck.write(90);

  if (distanceLeft >= distanceRight){
    turnLeft();
    delay(1000);
  }
  else{
    turnRight();
    delay(1000);
  }
  
  goForward();

}

void safeMode(){
  digitalWrite(LED_red , LOW);
  noTone(buzzer);
}

void alertMode(){
  digitalWrite(LED_red , HIGH);
  tone(buzzer , 2000);
}



void setup() {
  // put your setup code here, to run once:
pinMode(M1_d , OUTPUT);
pinMode(M1_dr , OUTPUT);
pinMode(M1_speed , OUTPUT);
pinMode(M2_d , OUTPUT);
pinMode(M2_dr , OUTPUT);
pinMode(M2_speed , OUTPUT);
pinMode(LED_red , OUTPUT);
pinMode(LED_white , OUTPUT);
pinMode(buzzer , OUTPUT);
pinMode(STBY , OUTPUT);

pinMode(Pwr , INPUT_PULLUP);

digitalWrite(M1_d , LOW);
digitalWrite(M1_dr , LOW);
digitalWrite(M1_speed , LOW);
digitalWrite(M2_d , LOW);
digitalWrite(M2_dr , LOW);
digitalWrite(M2_speed , LOW);
digitalWrite(LED_red , LOW);
digitalWrite(LED_white , LOW);
digitalWrite(buzzer , LOW);
digitalWrite(STBY , HIGH);

  stop();
  delay(1000);

  robotNeck.attach(robotNeck_pin);
  robotNeck.write(90);
  Serial.begin(115200);
}

void loop() {
  standby = digitalRead(Pwr);
  while (standby)
  {
  safeMode();
  distanceValue = analogRead(distanceSensor);
  Serial.println(distanceValue);

  if (distanceValue >= maxDistance){

    stop();
    //alertMode();
    delay(500);
    makeTurn();
    
  }
  else{

    goForward();
  }
  standby = digitalRead(Pwr);
  }
  stop();

}



