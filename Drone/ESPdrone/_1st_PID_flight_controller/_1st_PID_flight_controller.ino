#include <Wire.h>

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050

struct PID {
  float Kp ;
  float Ki ;
  float Kd ;
  float integral;
  float previous_error;
  unsigned long lastTime;
};

PID rollPID = {1.0, 0.0, 0.0, 0.0, 0.0, 0};
PID pitchPID = {1.0, 0.0, 0.0, 0.0, 0.0, 0};
PID yawPID = {1.0, 0.0, 0.0, 0.0, 0.0, 0};

// Define motor pins
const int motor1Pin = 5; // GPIO pin for motor 1
const int motor2Pin = 6; // GPIO pin for motor 2
const int motor3Pin = 3; // GPIO pin for motor 3
const int motor4Pin = 4; // GPIO pin for motor 4

float roll, pitch, yaw; // Current angles

void setup() {
  Serial.begin(921600);
  Wire.begin(11, 10); // Initialize I2C with SDA on GPIO11 and SCL on GPIO10
  Wire.setClock(1000000); // Set I2C clock speed to 1 MHz

  // Initialize MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Set to 0 to wake up the MPU-6050
  Wire.endTransmission(true);

  // Set accelerometer sensitivity to ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0);    // Set sensitivity to ±2g
  Wire.endTransmission(true);

  // Set gyroscope sensitivity to ±250 degrees/second
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0);    // Set sensitivity to ±250 degrees/second
  Wire.endTransmission(true);

  // Initialize motor pins as output
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  pinMode(motor3Pin, OUTPUT);
  pinMode(motor4Pin, OUTPUT);
}

void loop() {
  // Read the MPU6050 data
  int16_t ax, ay, az, gx, gy, gz;
  readMPU6050Data(ax, ay, az, gx, gy, gz);

  // Convert raw values to usable data
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  float gyroX = gx / 131.0;
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;

  // Calculate roll and pitch from the accelerometer data
  float accelRoll = atan2(accelY, accelZ) * 180 / PI;
  float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;

  // Integrate gyroscope data to get angles
  roll += gyroX * 0.001;
  pitch += gyroY * 0.001;
  yaw += gyroZ * 0.001;

  // Apply complementary filter
  roll = 0.98 * roll + 0.02 * accelRoll;
  pitch = 0.98 * pitch + 0.02 * accelPitch;

  // Calculate PID for each axis
  float rollOutput = calculatePID(rollPID, 0, roll);
  float pitchOutput = calculatePID(pitchPID, 0, pitch);
  float yawOutput = calculatePID(yawPID, 0, yaw);

  // Apply motor adjustments based on PID output
  setMotorSpeeds(rollOutput, pitchOutput, yawOutput);
 
  // Print the results
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("\tPitch: ");
  Serial.print(pitch);
  Serial.print("\tYaw: ");
  Serial.print(yaw);
  Serial.println();
}

float calculatePID(PID &pid, float setpoint, float current) {
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - pid.lastTime) / 1000.0;
  pid.lastTime = currentTime;

  float error = setpoint - current;
  pid.integral += error * elapsedTime;
  float derivative = (error - pid.previous_error) / elapsedTime;
  pid.previous_error = error;

  return pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
}

void setMotorSpeeds(float rollOutput, float pitchOutput, float yawOutput) {
  int motor1Speed = constrain(127 + rollOutput + pitchOutput - yawOutput, 0, 255);
  int motor2Speed = constrain(127 - rollOutput + pitchOutput + yawOutput, 0, 255);
  int motor3Speed = constrain(127 - rollOutput - pitchOutput - yawOutput, 0, 255);
  int motor4Speed = constrain(127 + rollOutput - pitchOutput + yawOutput, 0, 255);

  analogWrite(motor1Pin, motor1Speed);
  analogWrite(motor2Pin, motor2Speed);
  analogWrite(motor3Pin, motor3Speed);
  analogWrite(motor4Pin, motor4Speed);
}

void readMPU6050Data(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Starting register for accelerometer readings
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // Request 14 bytes

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Ignore temperature readings
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}