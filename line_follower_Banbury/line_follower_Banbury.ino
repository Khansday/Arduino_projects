// Define sensor and motor driver pins
int sensorLeft = A0;
int sensorRight = A1;

int ENA = 10;
int IN1 = 9;
int IN2 = 8;
int IN3 = 7;
int IN4 = 6;
int ENB = 5;

// Threshold for sensor reading to determine line detection
int threshold = 300;

// Motor speeds
int motorSpeed = 150;  //forward speed
int turnFactor = 0.0;  //turning speed

void setup() {
  // Initialize motor control pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Initialize serial communication for debugging
  Serial.begin(115200);
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
    sharpturnRight();
  } else if (leftSensorValue < threshold && rightSensorValue > threshold) {
    // Right sensor detects the line, left sensor does not: Turn left
    sharpturnLeft();
  } else {
    // No sensor detects the line: Stop
    stop();
  }

  // Small delay to allow for sensor reading stability
  //delay(50);
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

void sharpturnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, motorSpeed); // Reduce speed for smoother turn
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 255 - motorSpeed * turnFactor);
}

void sharpturnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 255 - motorSpeed * turnFactor);
  
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





