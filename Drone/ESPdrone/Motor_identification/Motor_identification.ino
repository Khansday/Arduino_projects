// Define motor pins
const int motor1Pin = 5; // GPIO pin for motor 1 
const int motor2Pin = 6; // GPIO pin for motor 2 
const int motor3Pin = 3; // GPIO pin for motor 3
const int motor4Pin = 4; // GPIO pin for motor 4

void setup() {
  // Initialize serial communication
  Serial.begin(921600);

  // Set motor pins as output
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  pinMode(motor3Pin, OUTPUT);
  pinMode(motor4Pin, OUTPUT);
}

void loop() {
  Serial.println("M1 ON");
  analogWrite(motor1Pin, 15);
  delay(1000);
  analogWrite(motor1Pin, 0);
  Serial.println("M1 OFF");
  delay(2000);

    Serial.println("M2 ON");
  analogWrite(motor2Pin, 15);
  delay(1000);
  analogWrite(motor2Pin, 0);
  Serial.println("M2 OFF");
  delay(2000);

    Serial.println("M3 ON");
  analogWrite(motor3Pin, 15);
  delay(1000);
  analogWrite(motor3Pin, 0);
  Serial.println("M3 OFF");
  delay(2000);

    Serial.println("M4 ON");
  analogWrite(motor4Pin, 15);
  delay(1000);
  analogWrite(motor4Pin, 0);
  Serial.println("M4 OFF");
  delay(2000);

}