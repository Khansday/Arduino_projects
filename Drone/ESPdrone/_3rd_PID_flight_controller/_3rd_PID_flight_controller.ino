#include <Wire.h>
#include <Arduino.h>

class PID {
public:
  PID(float kp, float ki, float kd, float dt) : kp(kp), ki(ki), kd(kd), dt(dt), integral(0), prevError(0) {}

  float calculate(float setpoint, float measured) {
    float error = setpoint - measured;
    integral += error * dt;
    float derivative = (error - prevError) / dt;
    prevError = error;
    return kp * error + ki * integral + kd * derivative;
  }

private:
  float kp;
  float ki;
  float kd;
  float dt;
  float integral;
  float prevError;
};

// Define motor pins
const int FRpin = 5; // GPIO pin for motor 1
const int BRpin = 6; // GPIO pin for motor 2
const int BLpin = 3; // GPIO pin for motor 3
const int FLpin = 4; // GPIO pin for motor 4

// PID parameters
float kp = 0.8;
float ki = 0.0;
float kd = 0.0;
float dt = 0.001; // 1 kHz

// Add these variables for offsets
int16_t ax_offset = -340 , ay_offset = -130 , az_offset = -1140;
int16_t gx_offset = -199, gy_offset = 3, gz_offset = 15;


// Define setpoints for roll and pitch
float setpointRoll = 0.0;
float setpointPitch = 0.0;
int setpointMotorPWM = 90;

// Create PID controllers for roll and pitch
PID pidRoll(kp, ki, kd, dt);
PID pidPitch(kp, ki, kd, dt);

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
const int CONFIG = 0x1A;  //address of teh low pass filter register
const float alpha = 0.98;  // Complementary filter constant
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

// Unbalance threshold
const float unbalanceThreshold = 35.0; // degrees

unsigned long lastTime = 0;
unsigned long currentTime = 0;

void setup() {
  Serial.begin(921600);
  Wire.begin(11, 10); // Initialize I2C with SDA on GPIO11 and SCL on GPIO10
  Wire.setClock(1000000); //clock speed at 1MHz

  // Set motor pins as output
  pinMode(FRpin, OUTPUT);
  pinMode(BRpin, OUTPUT);
  pinMode(BLpin, OUTPUT);
  pinMode(FLpin, OUTPUT);

  setupMPU6050();

  // Initial delay to allow the sensor to stabilize
  delay(100);
}

void loop() {
  currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0; // Convert dt to seconds

  if (dt >= 0.001) { // Ensure the loop runs at approximately 1 kHz (1ms per loop)
    lastTime = currentTime;

    // Read raw values from MPU6050
    int16_t ax, ay, az, gx, gy, gz;
    readMPU6050Data(ax, ay, az, gx, gy, gz);
   
    // Convert raw values to accelerometer (g) and gyroscope (degrees/second)
    //negative signs to match teh orientation of the drone
    float accelX = -ax / 16384.0;
    float accelY = -ay / 16384.0;
    float accelZ = az / 16384.0;
    float gyroX = -gx / 131.0;
    float gyroY = -gy / 131.0;
    float gyroZ = gz / 131.0;
   
    // Calculate roll and pitch from the accelerometer data
    float accelPitch = atan2(accelY, accelZ) * 180 / PI;
    float accelRoll = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;
   
    // Integrate gyroscope data to get angles
    roll += gyroY * dt;
    pitch += gyroX * dt;
    yaw += gyroZ * dt;
   
    // Apply complementary filter
    roll = alpha * roll + (1.0 - alpha) * accelRoll;
    pitch = alpha * pitch + (1.0 - alpha) * accelPitch;
    // Yaw is only from the gyroscope since accelerometer doesn't give yaw angle

    // Check for extreme unbalance
    if (abs(roll) > unbalanceThreshold || abs(pitch) > unbalanceThreshold) {
      // Turn off motors if too unbalanced
      analogWrite(FRpin, 0);
      analogWrite(BRpin, 0);
      analogWrite(BLpin, 0);
      analogWrite(FLpin, 0);
      Serial.println("Drone is too unbalanced! Motors turned off.");
      while (true);
    }

    // PID calculations
    float pidOutputRoll = pidRoll.calculate(setpointRoll, roll);
    float pidOutputPitch = pidPitch.calculate(setpointPitch, pitch);
   
    // Calculate motor PWM values
    int FRPWM = constrain(setpointMotorPWM - pidOutputRoll + pidOutputPitch, 0, 255);
    int BRPWM = constrain(setpointMotorPWM - pidOutputRoll - pidOutputPitch, 0, 255);
    int BLPWM = constrain(setpointMotorPWM + pidOutputRoll - pidOutputPitch, 0, 255);
    int FLPWM = constrain(setpointMotorPWM + pidOutputRoll + pidOutputPitch, 0, 255);
   
    // Set the PWM values for the motors
    analogWrite(FRpin, FRPWM);
    analogWrite(BRpin, BRPWM);
    analogWrite(BLpin, BLPWM);
    analogWrite(FLpin, FLPWM);
   
    // Print the results less frequently
    static int printCounter = 0;
    printCounter++;
    if (printCounter >= 20) { // Print every 10 loops (~100ms)
      Serial.print("M1: ");
      Serial.print(FRPWM);
      Serial.print("\tM2: ");
      Serial.print(BRPWM);
      Serial.print("\tM3: ");
      Serial.print(BLPWM);
      Serial.print("\tM4: ");
      Serial.print(FLPWM);
      Serial.print("\tR: ");
      Serial.print(roll);
      Serial.print("\tP: ");
      Serial.print(pitch);
      Serial.print("\tY: ");
      Serial.print(yaw);
      Serial.print("\tF: ");
      Serial.print(1.0 / dt);
      Serial.println(" Hz");
      printCounter = 0;
    }
  }
}

void setupMPU6050(){
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

  // Configure the DLPF 
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(CONFIG);
  Wire.write(0x04); // DLPF_CFG = 6 (5 Hz); DLPF_CFG = 4 (21 Hz)
  Wire.endTransmission(true);
}

void readMPU6050Data(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Starting register for accelerometer readings
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // Request 14 bytes

  ax = Wire.read() << 8 | Wire.read() - ax_offset; 
  ay = Wire.read() << 8 | Wire.read() - ay_offset;
  az = Wire.read() << 8 | Wire.read() - az_offset;
  Wire.read(); Wire.read(); // Ignore temperature readings
  gx = Wire.read() << 8 | Wire.read() - gx_offset;
  gy = Wire.read() << 8 | Wire.read() - gy_offset;
  gz = Wire.read() << 8 | Wire.read() - gz_offset;
}
