#include <Arduino.h>

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
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    // Read the incoming byte
    int pwmValue = Serial.parseInt();
    //clean residual data
    while(Serial.available()) Serial.read();
    // Ensure the PWM value is within the valid range
    pwmValue = constrain(pwmValue, 0, 255);

    // Set the PWM value for all motors
    analogWrite(motor1Pin, pwmValue);
    analogWrite(motor2Pin, pwmValue);
    analogWrite(motor3Pin, pwmValue);
    analogWrite(motor4Pin, pwmValue);

    // Print the PWM value to the Serial Monitor
    Serial.print("PWM Value set to: ");
    Serial.println(pwmValue);
  }

  delay(100); // Small delay to allow processing
}