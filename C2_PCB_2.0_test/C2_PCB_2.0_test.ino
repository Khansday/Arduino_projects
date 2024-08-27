// Define motor control pins
const int pwmA = 3;
const int ina1 = 4;
const int ina2 = 5;
const int inb1 = 6;
const int inb2 = 7;
const int pwmB = 11;

int speed_value = 120;

// Define LED pin
const int ledPin = 8;

// Define IR sensor pin
const int irSensor = A0;

// Threshold for obstacle detection
const int threshold = 350;

void setup() {
  // Set motor control pins as output
  pinMode(pwmA, OUTPUT);
  pinMode(ina1, OUTPUT);
  pinMode(ina2, OUTPUT);
  pinMode(inb1, OUTPUT);
  pinMode(inb2, OUTPUT);
  pinMode(pwmB, OUTPUT);

  // Set LED pin as output
  pinMode(ledPin, OUTPUT);

  // Set IR sensor pin as input
  pinMode(irSensor, INPUT);

  // Initialize serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Read the IR sensor value
  int irValue = analogRead(irSensor);

  // Debugging
  Serial.println(irValue);

  // Check for obstacle
  if (irValue > threshold) {
    // Obstacle detected
    stopMoving();
    delay(500); // Short delay before turning

    // Turn right
    turnRight();
    delay(500); // Adjust the delay based on your turning speed and angle

 } else {
    // No obstacle, keep moving forward
    moveForward();
  }
}

// Function to move forward
void moveForward() {
  digitalWrite(ina1, HIGH);
  digitalWrite(ina2, LOW);
  digitalWrite(inb1, HIGH);
  digitalWrite(inb2, LOW);
  analogWrite(pwmA, speed_value); // Full speed
  analogWrite(pwmB, speed_value); // Full speed
  digitalWrite(ledPin, HIGH); // Turn on LED
}

// Function to stop
void stopMoving() {
  digitalWrite(ina1, LOW);
  digitalWrite(ina2, LOW);
  digitalWrite(inb1, LOW);
  digitalWrite(inb2, LOW);
  analogWrite(pwmA, 0);
  analogWrite(pwmB, 0);
  digitalWrite(ledPin, LOW); // Turn off LED
}

// Function to turn right
void turnRight() {
  digitalWrite(ina1, HIGH);
  digitalWrite(ina2, LOW);
  digitalWrite(inb1, LOW);
  digitalWrite(inb2, HIGH);
  analogWrite(pwmA, speed_value); // Full speed
  analogWrite(pwmB, speed_value); // Full speed
  digitalWrite(ledPin, HIGH); // Turn on LED
}
