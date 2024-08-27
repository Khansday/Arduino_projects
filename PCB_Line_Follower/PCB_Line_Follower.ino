

#define M1_d 5
#define M1_dr 4
#define M1_speed 3
#define M2_d 7
#define M2_dr 6
#define M2_speed 11

int motorSpeed =180;
const int sensorThreshold = 300; //Adjust this threshold to detect the line ( 0 - 1023 ); 

#define leftSensorPin A0     // Analog pin for left sensor
#define rightSensorPin A1     // Analog pin for right sensor
int leftSensorValue;
int rightSensorValue;


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
  digitalWrite(M1_d , HIGH);
  digitalWrite(M1_dr , LOW);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d ,LOW);
  digitalWrite(M2_dr , HIGH);
  analogWrite(M2_speed, motorSpeed);
} 

void turnRight(){
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , HIGH);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , HIGH);
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



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(M1_d , OUTPUT);
  pinMode(M1_dr , OUTPUT);
  pinMode(M1_speed , OUTPUT);
  pinMode(M2_d , OUTPUT);
  pinMode(M2_dr , OUTPUT);
  pinMode(M2_speed , OUTPUT);

  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , LOW);
  digitalWrite(M1_speed , LOW);
  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , LOW);
  digitalWrite(M2_speed , LOW);

}

void loop() {
  leftSensorValue = analogRead(leftSensorPin);
  rightSensorValue = analogRead(rightSensorPin);
  Serial.print("A0 ");
  Serial.print(leftSensorValue);
  Serial.print(" A1 ");
  Serial.print(rightSensorValue);
  Serial.println();

  // Line follower logic
  if (leftSensorValue > sensorThreshold && rightSensorValue > sensorThreshold) {
    // Both sensors are on the line
    goForward();
  } else if (leftSensorValue > sensorThreshold && rightSensorValue < sensorThreshold) {
    // Left sensor is on the line only
    turnLeft();
  } else if (rightSensorValue > sensorThreshold && leftSensorValue < sensorThreshold) {
    // Right sensor is on the line only
    turnRight(); 
  } else {
    // Both sensors are off the line
    stop();
  }
}


