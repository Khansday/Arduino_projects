#include <Wire.h>
#include <math.h>
#include <Arduino.h>
#include "driver/ledc.h"

// Define motor pins
const int FRpin = 5; // GPIO pin for motor 1 (Front-Right)
const int BRpin = 6; // GPIO pin for motor 2 (Back-Right)
const int BLpin = 3; // GPIO pin for motor 3 (Back-Left)
const int FLpin = 4; // GPIO pin for motor 4 (Front-Left)

#define LED_BLE_PIN 7
#define LED_RED_PIN 8
#define LED_GRN_PIN 9

// Define PWM parameters
const int pwmFreq = 5000;  // 5 kHz PWM frequency
const ledc_timer_bit_t pwmResolution = LEDC_TIMER_13_BIT; // 13-bit resolution (0-8191)
const ledc_timer_t pwmTimer = LEDC_TIMER_0; // Timer 0
const ledc_mode_t pwmSpeedMode = LEDC_LOW_SPEED_MODE;

// MPU6050 variables and addresses
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
const int CONFIG = 0x1A;  // Address of the low pass filter register
float alpha = 0.98;  // Initial complementary filter constant

// Offsets for calibration
int16_t ax_offset = -340, ay_offset = -130, az_offset = -1140;
int16_t gx_offset = -199, gy_offset = 3, gz_offset = 15;

float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

unsigned long lastTime = 0;
unsigned long currentTime = 0;
float dt = 0.001; // Initial dt value for the first loop

double linear_acc_threshold = 0.2; // Threshold to decide when to use or discard accelerometer data

// PID constants for roll and pitch
float Kp_roll = 30.5, Ki_roll = 0.0, Kd_roll = 0.0;
float Kp_pitch = 30.5, Ki_pitch = 0.0, Kd_pitch = 0.0;

float rollSetpoint = 0.0;  // Desired roll angle
float pitchSetpoint = 0.0; // Desired pitch angle

float rollErrorSum = 0.0, lastRollError = 0.0;
float pitchErrorSum = 0.0, lastPitchError = 0.0;

// Tilt threshold (degrees)
const float tiltThreshold = 30.0; // Example: stop motors if roll or pitch exceeds 30 degrees
int i=0, cyclesPrint = 10;  //debug printing variable and cycles

void setup() {
  Serial.begin(921600); // highest speed possible on my setup
  Wire.begin(11, 10); // Initialize I2C with SDA on GPIO11 and SCL on GPIO10
  Wire.setClock(1000000); // Clock speed at 1MHz

  pinMode(LED_GRN_PIN, OUTPUT);
  pinMode(LED_BLE_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);

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

  // Configure the DLPF to 5 Hz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(CONFIG);
  Wire.write(0x06); // DLPF_CFG = 6 (5 Hz)
  Wire.endTransmission(true);

  // Initial delay to allow the sensor to stabilize
  delay(100);

  // Configure the LEDC timer and channels for motor control
  configurePWM();
}

void configurePWM() {
  // Configure the LEDC timer
  ledc_timer_config_t ledc_timer = {
    .speed_mode       = pwmSpeedMode,
    .duty_resolution  = pwmResolution,
    .timer_num        = pwmTimer,
    .freq_hz          = pwmFreq,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  // Configure LEDC channels
  configureMotorChannel(FRpin, LEDC_CHANNEL_0);
  configureMotorChannel(BRpin, LEDC_CHANNEL_1);
  configureMotorChannel(BLpin, LEDC_CHANNEL_2);
  configureMotorChannel(FLpin, LEDC_CHANNEL_3);
}

void configureMotorChannel(int motorPin, ledc_channel_t channel) {
  ledc_channel_config_t ledc_channel = {
    .gpio_num       = motorPin,
    .speed_mode     = pwmSpeedMode,
    .channel        = channel,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = pwmTimer,
    .duty           = 0,
    .hpoint         = 0
  };
  ledc_channel_config(&ledc_channel);
}

void loop() {
  currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0; // Convert dt to seconds

  if (dt >= 0.001) { // Ensure the loop runs at approximately 1 kHz (1ms per loop)
    lastTime = currentTime;

    // Read sensor data and calculate orientation
    int16_t ax, ay, az, gx, gy, gz;
    readMPU6050Data(ax, ay, az, gx, gy, gz);
    calculateOrientation(ax, ay, az, gx, gy, gz, dt);

    // Check if the drone is tilted beyond the threshold
    if (isTiltedBeyondThreshold()) {
      stopMotors(); // Stop all motors if tilted beyond the threshold
      return; // Exit loop to prevent further execution
    }


    // Compute PID for roll
    float rollError = rollSetpoint - roll;
    rollErrorSum += rollError * dt;
    float rollRate = (rollError - lastRollError) / dt;
    float rollCorrection = Kp_roll * rollError + Ki_roll * rollErrorSum + Kd_roll * rollRate;
    lastRollError = rollError;

    // Compute PID for pitch
    float pitchError = pitchSetpoint - pitch;
    pitchErrorSum += pitchError * dt;
    float pitchRate = (pitchError - lastPitchError) / dt;
    float pitchCorrection = Kp_pitch * pitchError + Ki_pitch * pitchErrorSum + Kd_pitch * pitchRate;
    lastPitchError = pitchError;

    // Adjust motor speeds based on PID output
    adjustMotorSpeeds(rollCorrection, pitchCorrection);
    i++;
    // Print the results
    if (i = cyclesPrint){
      debugPrint();
      i = 0;
    }
    
    
  }
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

void calculateOrientation(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float dt) {
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

  // Calculate the magnitude of the acceleration vector
  double accelMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  // Integrate gyroscope data to get angles
  roll += gyroY * dt + gyroX * sin(roll * PI / 180) * tan(pitch * PI / 180) * dt + gyroZ * cos(roll * PI / 180) * tan(pitch * PI / 180) * dt;
  pitch += gyroX * cos(roll * PI / 180) * dt - gyroZ * sin(roll * PI / 180) * dt;
  yaw += (gyroX * sin(roll * PI / 180) / cos(pitch * PI / 180) + gyroZ * cos(roll * PI / 180) / cos(pitch * PI / 180)) * dt;

  // Adjust the complementary filter coefficient based on the acceleration magnitude
  if (fabs(1.0 - accelMagnitude) < linear_acc_threshold) {
    //numbers are multipled to fit match map func requirements & increase resolution
    alpha = map(fabs(1.0 - accelMagnitude)*1000, 0,linear_acc_threshold*1000, 900,980);
    alpha = alpha/1000; //bring back original number format
    roll = alpha * roll + (1.0 - alpha) * accelRoll;
    pitch = alpha * pitch + (1.0 - alpha) * accelPitch;
    //Visualise on the drone the accelerometer data is valid
      digitalWrite(LED_RED_PIN, LOW);
      digitalWrite(LED_GRN_PIN, HIGH);
  }
  else{
    //Visualise on the drone the accelerometer data is discarded
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GRN_PIN, LOW);
  }
}

bool isTiltedBeyondThreshold() {
  return (fabs(roll) > tiltThreshold || fabs(pitch) > tiltThreshold);
}

void stopMotors() {
  ledc_set_duty(pwmSpeedMode, LEDC_CHANNEL_0, 0);
  ledc_update_duty(pwmSpeedMode, LEDC_CHANNEL_0);

  ledc_set_duty(pwmSpeedMode, LEDC_CHANNEL_1, 0);
  ledc_update_duty(pwmSpeedMode, LEDC_CHANNEL_1);

  ledc_set_duty(pwmSpeedMode, LEDC_CHANNEL_2, 0);
  ledc_update_duty(pwmSpeedMode, LEDC_CHANNEL_2);

  ledc_set_duty(pwmSpeedMode, LEDC_CHANNEL_3, 0);
  ledc_update_duty(pwmSpeedMode, LEDC_CHANNEL_3);

  Serial.println("Motors stopped due to excessive tilt!");
}

void adjustMotorSpeeds(float rollCorrection, float pitchCorrection) {
  // Base speed for the motors (can be adjusted based on throttle input)
  int baseSpeed = 4000; // Example value, adjust based on your requirements
  //int minPWM = 0;
  //int maxPWM = 8191;

  int minPWM = baseSpeed * 0.4;
  int maxPWM = baseSpeed * 1.5;

  minPWM = constrain(minPWM, 0, 8191);
  maxPWM = constrain(maxPWM, 0, 8191);

  // Adjust motor speeds based on roll and pitch corrections
  int FRPWM = round(constrain(baseSpeed - rollCorrection + pitchCorrection, minPWM, maxPWM)); // Front-Right
  int BRPWM = round(constrain(baseSpeed - rollCorrection - pitchCorrection, minPWM, maxPWM)); // Back-Right
  int BLPWM = round(constrain(baseSpeed + rollCorrection - pitchCorrection, minPWM, maxPWM)); // Back-Left
  int FLPWM = round(constrain(baseSpeed + rollCorrection + pitchCorrection, minPWM, maxPWM)); // Front-Left

  // Set motor speeds
  ledc_set_duty(pwmSpeedMode, LEDC_CHANNEL_0, FRPWM);
  ledc_update_duty(pwmSpeedMode, LEDC_CHANNEL_0);

  ledc_set_duty(pwmSpeedMode, LEDC_CHANNEL_1, BRPWM);
  ledc_update_duty(pwmSpeedMode, LEDC_CHANNEL_1);

  ledc_set_duty(pwmSpeedMode, LEDC_CHANNEL_2, BLPWM);
  ledc_update_duty(pwmSpeedMode, LEDC_CHANNEL_2);

  ledc_set_duty(pwmSpeedMode, LEDC_CHANNEL_3, FLPWM);
  ledc_update_duty(pwmSpeedMode, LEDC_CHANNEL_3);
}

void debugPrint(){
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
}
