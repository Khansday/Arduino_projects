/*
 THIS CODE READS THE MPU6050 AT THE MAX SPEED SUPPORTED 
 PRINTS THE ROLL YAW AND PITCH AS FAST AS POSSIBLE
 THE LOOP FREQUENCY IS LIMITED AT 1 KHZ 
*/

#include <Wire.h>

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
const float alpha = 0.98;  // Complementary filter constant

float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

unsigned long lastTime = 0;
unsigned long currentTime = 0;
float dt = 0.001; // Initial dt value for the first loop

// Define motor pins
const int FRpin = 5; // GPIO pin for motor 1 (Front Right)
const int BRpin = 6; // GPIO pin for motor 2 (Back Right)
const int BLpin = 3; // GPIO pin for motor 3 (Back Left)
const int FLpin = 4; // GPIO pin for motor 4 (Front Left)

// PWM value for motors when tilted
const int motorPWM = 15;

void setup() {
  Serial.begin(921600);
  Wire.begin(11, 10); // Initialize I2C with SDA on GPIO11 and SCL on GPIO10
  Wire.setClock(1000000); // Clock speed at 1MHz

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

  // Initial delay to allow the sensor to stabilize
  delay(100);

  // Initialize motor pins
  pinMode(FRpin, OUTPUT);
  pinMode(BRpin, OUTPUT);
  pinMode(BLpin, OUTPUT);
  pinMode(FLpin, OUTPUT);
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
   
    // Print the results
    Serial.print("R: ");
    Serial.print(roll);
    Serial.print("\tP: ");
    Serial.print(pitch);
    Serial.print("\tY: ");
    Serial.print(yaw);
    Serial.print("\tF: ");
    Serial.print(1.0 / dt);
    Serial.println(" Hz");

    // Control motors based on roll and pitch
    controlMotors(roll, pitch);
  }
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

void controlMotors(float roll, float pitch) {
  // Reset motor PWM values
  int pwmFR = 0;
  int pwmBR = 0;
  int pwmBL = 0;
  int pwmFL = 0;

  // Adjust motor speeds based on tilt
  if (pitch > 0 ) { // Tilted back
    if (roll> 0) pwmBR = motorPWM;  //tilted right
    else pwmBL = motorPWM;  // tilted left
  }
  else{          //tilted forward
    if (roll> 0) pwmFR = motorPWM;  //tilted right
    else pwmFL = motorPWM;  // tilted left
  }
  // Set motor PWM values
  analogWrite(FRpin, pwmFR);
  analogWrite(BRpin, pwmBR);
  analogWrite(BLpin, pwmBL);
  analogWrite(FLpin, pwmFL);
}