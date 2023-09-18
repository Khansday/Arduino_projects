 #include <Servo.h>

Servo robotNeck;  // create servo object to control a servo

#define M1_d 1
#define M1_dr 2
#define M1_speed 5
#define M2_d 3
#define M2_dr 4
#define M2_speed 6

int motorSpeed =130; 

int robotNeck_pin = 12; 

int distanceSensor = A0; 
int distanceValue;
int maxDistance = 200;
int LED_red = 7;
int LED_white = 9;
int buzzer = 8; 

void goForward(){
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , HIGH);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , HIGH);
  analogWrite(M2_speed, motorSpeed);
}

void goBackwards(){
  digitalWrite(M1_d , HIGH);
  digitalWrite(M1_dr , LOW);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , HIGH);
  digitalWrite(M2_dr , LOW);
  analogWrite(M2_speed, motorSpeed); 
}

void turnLeft(){
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , LOW);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , HIGH);
  digitalWrite(M2_dr , LOW);
  analogWrite(M2_speed, motorSpeed);
} 

void turnRight(){
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , HIGH);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , LOW);
  analogWrite(M2_speed, motorSpeed); 
}

void stop(){
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , LOW);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , LOW);
  analogWrite(M2_speed, motorSpeed);
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

digitalWrite(M1_d , LOW);
digitalWrite(M1_dr , LOW);
digitalWrite(M1_speed , LOW);
digitalWrite(M2_d , LOW);
digitalWrite(M2_dr , LOW);
digitalWrite(M2_speed , LOW);
digitalWrite(LED_red , LOW);
digitalWrite(LED_white , LOW);
digitalWrite(buzzer , LOW);

  stop();
  delay(1000);

  robotNeck.attach(robotNeck_pin);
  robotNeck.write(90);
  //Serial.begin(9600);
}

void loop() {
  safeMode();
  distanceValue = analogRead(distanceSensor);
  ////Serial.println(distanceValue);

  if (distanceValue >= maxDistance){

    stop();
    alertMode();
    delay(500);
    makeTurn();
    
  }
  else{

    goForward();
  }

}



