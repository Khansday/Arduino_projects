/*
 THIS CODE READS THE MPU6050 AT THE MAX SPEED SUPPORTED 
 PRINTS THE ROLL YAW AND PITCH AS FAST AS POSSIBLE
 THE LOOP FRFEQUENCY IS LIMITED AT 1 KHZ 
*/

// Define motor pins
const int FRpin = 5; // GPIO pin for motor 1
const int BRpin = 6; // GPIO pin for motor 2
const int BLpin = 3; // GPIO pin for motor 3
const int FLpin = 4; // GPIO pin for motor 4

#include <Wire.h>

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
const float alpha = 0.98;  // Complementary filter constant

float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

unsigned long lastTime = 0;
unsigned long currentTime = 0;
float dt = 0.001; // Initial dt value for the first loop


  // Calculate motor PWM values
int speed = 50;
int i;   

void setup() {
  Serial.begin(921600);
  Wire.begin(11, 10); // Initialize I2C with SDA on GPIO11 and SCL on GPIO10
  Wire.setClock(1000000); //clock speed at 1MHz

      // Set motor pins as output
  pinMode(FRpin, OUTPUT);
  pinMode(BRpin, OUTPUT);
  pinMode(BLpin, OUTPUT);
  pinMode(FLpin, OUTPUT);

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

    // Set the PWM values for the motors
    analogWrite(FRpin, speed);
    analogWrite(BRpin, speed);
    analogWrite(BLpin, speed);
    analogWrite(FLpin, speed);

  // Initial delay to allow the sensor to stabilize
  delay(100);
}

void loop() {
  currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0; // Convert dt to seconds
  // Set the PWM values for the motors
    analogWrite(FRpin, speed);
    analogWrite(BRpin, speed);
    analogWrite(BLpin, speed);
    analogWrite(FLpin, speed);
  if (dt >= 0.01) { // Ensure the loop runs at approximately 1 kHz (1ms per loop)
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

    float accelRoll = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;
   
    // Integrate gyroscope data to get angles
    roll += gyroY * dt;


   
    // Apply complementary filter
    roll = alpha * roll + (1.0 - alpha) * accelRoll;

    // Yaw is only from the gyroscope since accelerometer doesn't give yaw angle
   
    // Print the results
    Serial.println(roll);
    //Serial.println(speed);

    if (i >5){
      if (speed >100) speed = 0;
      else speed++;
      i = 0;
    }
    else i++;
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