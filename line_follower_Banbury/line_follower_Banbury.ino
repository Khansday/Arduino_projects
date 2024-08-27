// Define sensor and motor driver pins
int sensorLeft = A0;
int sensorRight = A1;

int ENA = 11;
int IN1 = 10;
int IN2 = 9;
int IN3 = 8;
int IN4 = 7;
int ENB = 6;

// Threshold for sensor reading to determine line detection
int threshold = 300;

// Motor speeds
int motorSpeed = 200;  //forward speed
int turnFactor = 0.5;  //turning speed

void setup() {
  // Initialize motor control pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Initialize serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Read sensor values
  int leftSensorValue = analogRead(sensorLeft);
  int rightSensorValue = analogRead(sensorRight);

  // Debugging: Print sensor values to the serial monitor
  Serial.print("Left Sensor: ");
  Serial.print(leftSensorValue);
  Serial.print("  Right Sensor: ");
  Serial.println(rightSensorValue);

  // Determine the robot's movement based on sensor values
  if (leftSensorValue > threshold && rightSensorValue > threshold) {
    // Both sensors detect the line: Move forward
    moveForward();
  } else if (leftSensorValue > threshold && rightSensorValue < threshold) {
    // Left sensor detects the line, right sensor does not: Turn right
    turnRight();
  } else if (leftSensorValue < threshold && rightSensorValue > threshold) {
    // Right sensor detects the line, left sensor does not: Turn left
    turnLeft();
  } else {
    // No sensor detects the line: Stop
    stop();
  }

  // Small delay to allow for sensor reading stability
  delay(50);
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, motorSpeed);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, motorSpeed);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, motorSpeed); // Reduce speed for smoother turn
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, motorSpeed * turnFactor);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, motorSpeed * turnFactor);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, motorSpeed); // Reduce speed for smoother turn
}

void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}





