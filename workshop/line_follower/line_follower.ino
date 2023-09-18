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
#define potentiometerPin A0 // Analog pin for speed
int motorSpeed = 0;

#define LED_red 7
#define LED_white 8  
#define Pwr 12
int pwr_state;


// Constants
const int sensorThreshold = 90; // Adjust this threshold based on sensor readings

void setup() {
  // Initialize motor pins as output
pinMode(M1_d , OUTPUT);
pinMode(M1_dr , OUTPUT);
pinMode(M1_speed , OUTPUT);
pinMode(M2_d , OUTPUT);
pinMode(M2_dr , OUTPUT);
pinMode(M2_speed , OUTPUT);
pinMode(LED_red , OUTPUT);
pinMode(LED_white , OUTPUT);
pinMode(Pwr , INPUT_PULLUP);

digitalWrite(M1_d , LOW);
digitalWrite(M1_dr , LOW);
digitalWrite(M1_speed , LOW);
digitalWrite(M2_d , LOW);
digitalWrite(M2_dr , LOW);
digitalWrite(M2_speed , LOW);
digitalWrite(LED_red , LOW);
digitalWrite(LED_white , LOW);


  // Initialize potentiometer pin as input
  pinMode(potentiometerPin, INPUT);

  // Initialize serial communication for debugging
  Serial.begin(115200);
}

void loop() {
  stop();
  pwr_state = digitalRead(Pwr);
  while (!pwr_state){
    leftSensorValue = analogRead(leftSensorPin);
    rightSensorValue = analogRead(rightSensorPin);
    motorSpeed = analogRead(potentiometerPin);

    // Debugging: Print sensor and potentiometer values
    /*Serial.print("Left Sensor: ");
    Serial.print(leftSensorValue);
    Serial.print("\tRight Sensor: ");
    Serial.print(rightSensorValue);
    Serial.print("\tPotentiometer: ");
    Serial.println(motorSpeed);*/

    int speed = map(motorSpeed, 0, 1023, 0, 255); // Map potentiometer value to motor speed

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
