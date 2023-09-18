int motorSpeedFwd = 100;    //speed of the motors when going straight ( 0 - 255 )
int motorSpeedTurn = 100;    //speed of the motors when turning ( 0 - 255 )
//float turnFactor = 0.0;     //percentage of speed of inner wheel compared to the outer wheel (0 - 1)
const int sensorThreshold = 300; //Adjust this threshold to detect the line ( 0 - 1023 )


// Pin Definitions
#define leftSensorPin A1     // Analog pin for left sensor
#define rightSensorPin A2     // Analog pin for right sensor
int leftSensorValue;
int rightSensorValue;

#define M1_d 5
#define M1_dr 4
#define M1_speed 3
#define M2_d 10
#define M2_dr 9
#define M2_speed 11
#define STBY 6
#define Pwr 12
int pwr_state;

void setup() {
  // Initialize motor pins as output
  pinMode(M1_d , OUTPUT);
  pinMode(M1_dr , OUTPUT);
  pinMode(M1_speed , OUTPUT);
  pinMode(M2_d , OUTPUT);
  pinMode(M2_dr , OUTPUT);
  pinMode(M2_speed , OUTPUT);
  pinMode(Pwr , INPUT_PULLUP);

  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , LOW);
  digitalWrite(M1_speed , LOW);
  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , LOW);
  digitalWrite(M2_speed , LOW);

  // Initialize serial communication for debugging
  Serial.begin(115200);
}

void loop() {
  stop();
  pwr_state = digitalRead(Pwr);
  while (!pwr_state){
    leftSensorValue = analogRead(leftSensorPin);
    rightSensorValue = analogRead(rightSensorPin);

    // Debugging: Print sensor and potentiometer values
    Serial.print("Left Sensor: ");
    Serial.print(leftSensorValue);
    Serial.print("\tRight Sensor: ");
    Serial.println(rightSensorValue);

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
  pwr_state = digitalRead(Pwr);
  }
}

// Helper functions to control the motors with adjustable speed
void goForward(){
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , HIGH);
  analogWrite(M1_speed, motorSpeedFwd);
  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , HIGH);
  analogWrite(M2_speed, motorSpeedFwd);
}

void goBackwards(){
  digitalWrite(M1_d , HIGH);
  digitalWrite(M1_dr , LOW);
  analogWrite(M1_speed, motorSpeedFwd);
  digitalWrite(M2_d , HIGH);
  digitalWrite(M2_dr , LOW);
  analogWrite(M2_speed, motorSpeedFwd); 
}

void turnLeft(){
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , HIGH);
  analogWrite(M1_speed, motorSpeedTurn*turnFactor);
  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , HIGH);
  analogWrite(M2_speed, motorSpeedTurn);
} 

void turnRight(){
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , HIGH);
  analogWrite(M1_speed, motorSpeedTurn);
  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , HIGH);
  analogWrite(M2_speed, motorSpeedTurn*turnFactor); 
}

void stop(){
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , LOW);
  analogWrite(M1_speed, motorSpeedFwd);
  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , LOW);
  analogWrite(M2_speed, motorSpeedFwd);
}
