/*
 THIS CODE READS THE MPU6050 AT THE MAX SPEED SUPPORTED 
 PRINTS THE ROLL YAW AND PITCH AS FAST AS POSSIBLE
 THE LOOP FREQUENCY IS LIMITED AT 1 KHZ 
*/

#include <Wire.h>
#include <math.h>
#include <Arduino.h>
#include "driver/ledc.h"

// Define motor pins
const int FRpin = 5; // GPIO pin for motor 1 (Front-Right)
const int BRpin = 6; // GPIO pin for motor 2 (Back-Right)
const int BLpin = 3; // GPIO pin for motor 3 (Back-Left)
const int FLpin = 4; // GPIO pin for motor 4 (Front-Left)

// Define PWM parameters
const int pwmFreq = 5000;  // 5 kHz PWM frequency
const ledc_timer_bit_t pwmResolution = LEDC_TIMER_13_BIT; // 13-bit resolution (0-8191)
const ledc_timer_t pwmTimer = LEDC_TIMER_0; // Timer 0
const ledc_mode_t pwmSpeedMode = LEDC_LOW_SPEED_MODE;

#define LED_BLE_PIN 7
#define LED_RED_PIN 8
#define LED_GRN_PIN 9

const int MPU_ADDRESS = 0x68; // I2C address of the MPU-6050
const int CONFIG_REGISTER = 0x1A;  // Address of the low pass filter register
float complementary_filter_alpha = 0.98f;  // Initial complementary filter constant

// Offset variables
int16_t accel_x_offset = 0, accel_y_offset = 0, accel_z_offset = 0;
int16_t gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;

float roll_angle = 0.0f;
float pitch_angle = 0.0f;
float yaw_angle = 0.0f;

unsigned long previous_time = 0;
const unsigned long loop_frequency = 2000;  //in microseconds
float current_frequency;

float accel_threshold = 0.2f; //variable to decide when to use or discard accelerometer data

// PID constants for roll and pitch
float Kp_roll = 19.5, Ki_roll = 0.0, Kd_roll = 0.0;
float Kp_pitch = 19.5, Ki_pitch = 0.0, Kd_pitch = 0.0;

float rollSetpoint = 0.0;  // Desired roll angle
float pitchSetpoint = 0.0; // Desired pitch angle

float rollErrorSum = 0.0, lastRollError = 0.0;
float pitchErrorSum = 0.0, lastPitchError = 0.0;

// Tilt threshold (degrees)
const float tiltThreshold = 30.0; // Example: stop motors if roll or pitch exceeds 30 degrees

int debug_counter = 0, debug_cycles = 20;  //debug printing variable and cycles

void setup() {
  pinMode(LED_GRN_PIN, OUTPUT);
  pinMode(LED_BLE_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);

  Serial.begin(250000); // highest speed possible on my setup, higher than this I get errors
  Wire.begin(11, 10); // Initialize I2C with SDA on GPIO11 and SCL on GPIO10
  Wire.setClock(200000); // Clock speed at 200 KHz 

  // Initialize MPU6050
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Set to 0 to wake up the MPU-6050
  Wire.endTransmission(true);

  // Set accelerometer sensitivity to ±2g
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0);    // Set sensitivity to ±2g
  Wire.endTransmission(true);

  // Set gyroscope sensitivity to ±250 degrees/second
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0);    // Set sensitivity to ±250 degrees/second
  Wire.endTransmission(true);

  // Configure the DLPF to 5 Hz
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(CONFIG_REGISTER);
  Wire.write(0x06); // DLPF_CFG = 6 (5 Hz)
  Wire.endTransmission(true);

  // Initial delay to allow the sensor to stabilize
  delay(2000);

  // Calibrate sensor at each startup
  digitalWrite(LED_BLE_PIN, HIGH);
  calibrateSensors();
  digitalWrite(LED_BLE_PIN, LOW);

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

  unsigned long current_time = micros();
  unsigned long elapsed_time = current_time - previous_time;

  if (elapsed_time >= loop_frequency) { // Ensure the loop runs at approximately 1 kHz (1ms per loop)
    previous_time = current_time;
    double delta_time = double(elapsed_time) / 1000000.0f;
    current_frequency = 1 / delta_time; // just use for debugging
    
    // Read raw values from MPU6050
    int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;

    if (readMPU6050Data(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)) {
    
      // Convert raw values to accelerometer (g) and gyroscope (degrees/second)
      // Negative signs to match the orientation of the drone
      float accel_x_g = -accel_x / 16384.0;
      float accel_y_g = -accel_y / 16384.0;
      float accel_z_g = accel_z / 16384.0;
      float gyro_x_dps = -gyro_x / 131.0;
      float gyro_y_dps = -gyro_y / 131.0;
      float gyro_z_dps = gyro_z / 131.0;
    
      // Calculate roll and pitch from the accelerometer data
      float accel_pitch_angle = atan2(accel_y_g, accel_z_g) * 180 / PI;
      float accel_roll_angle = atan2(-accel_x_g, sqrt(accel_y_g * accel_y_g + accel_z_g * accel_z_g)) * 180 / PI;

      // Calculate the magnitude of the acceleration vector
      float accel_magnitude = sqrt(accel_x_g * accel_x_g + accel_y_g * accel_y_g + accel_z_g * accel_z_g);
      
      // Integrate gyroscope data to get angles, accounting for cross-coupling effects without small angle approximation
      roll_angle += gyro_y_dps * delta_time + gyro_x_dps * sin(roll_angle * PI / 180) * tan(pitch_angle * PI / 180) * delta_time + gyro_z_dps * cos(roll_angle * PI / 180) * tan(pitch_angle * PI / 180) * delta_time;
      pitch_angle += gyro_x_dps * cos(roll_angle * PI / 180) * delta_time - gyro_z_dps * sin(roll_angle * PI / 180) * delta_time;
      yaw_angle += (gyro_x_dps * sin(roll_angle * PI / 180) / cos(pitch_angle * PI / 180) + gyro_z_dps * cos(roll_angle * PI / 180) / cos(pitch_angle * PI / 180)) * delta_time;
      
      // Adjust the complementary filter coefficient based on the acceleration magnitude
      if (fabs(1.0 - accel_magnitude) < accel_threshold) {
        // If the acceleration magnitude does NOT deviate significantly from 1g
        // calculate dynamic filter coefficient
        // numbers are multiplied to fit match map func requirements & increase resolution
        complementary_filter_alpha = map(fabs(1.0 - accel_magnitude) * 1000, 0, accel_threshold * 1000, 900, 980);
        complementary_filter_alpha = complementary_filter_alpha / 1000; // bring back original number format
        
        // Apply complementary filter
        roll_angle = complementary_filter_alpha * roll_angle + (1.0 - complementary_filter_alpha) * accel_roll_angle;
        pitch_angle = complementary_filter_alpha * pitch_angle + (1.0 - complementary_filter_alpha) * accel_pitch_angle;
        
        // Yaw is only from the gyroscope since accelerometer doesn't give yaw angle
        // Visualize on the drone the accelerometer data is valid
        digitalWrite(LED_RED_PIN, LOW);
        digitalWrite(LED_GRN_PIN, HIGH);

      } else {
        // Visualize on the drone the accelerometer data is discarded
        digitalWrite(LED_RED_PIN, HIGH);
        digitalWrite(LED_GRN_PIN, LOW);
      }
          // Print the results
      if (++debug_counter == debug_cycles) {
        attitudePrint();
        //sensorPrint(accel_x_g, accel_y_g, accel_z_g, gyro_x_dps, gyro_y_dps, gyro_z_dps);
        debug_counter = 0;
      }
    }

    // Check if the drone is tilted beyond the threshold
    if (isTiltedBeyondThreshold()) {
      stopMotors(); // Stop all motors if tilted beyond the threshold
      return; // Exit loop to prevent further execution
    }

    // Compute PID for roll
    float rollError = rollSetpoint - roll_angle;
    rollErrorSum += rollError * delta_time;
    float rollRate = (rollError - lastRollError) / delta_time;
    float rollCorrection = Kp_roll * rollError + Ki_roll * rollErrorSum + Kd_roll * rollRate;
    lastRollError = rollError;

    // Compute PID for pitch
    float pitchError = pitchSetpoint - pitch_angle;
    pitchErrorSum += pitchError * delta_time;
    float pitchRate = (pitchError - lastPitchError) / delta_time;
    float pitchCorrection = Kp_pitch * pitchError + Ki_pitch * pitchErrorSum + Kd_pitch * pitchRate;
    lastPitchError = pitchError;

    // Adjust motor speeds based on PID output
    adjustMotorSpeeds(rollCorrection, pitchCorrection);
    
  }
}

bool readMPU6050Data(int16_t &accel_x_out, int16_t &accel_y_out, int16_t &accel_z_out, int16_t &gyro_x_out, int16_t &gyro_y_out, int16_t &gyro_z_out) {
  bool success_flag = false;

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B); // Starting register for accelerometer readings
  int transmission_error = Wire.endTransmission(false);
  if (transmission_error == 0) {
    int bytes_received = Wire.requestFrom(MPU_ADDRESS, 14, true); // Request 14 bytes
    if (bytes_received == 14) { // received the amount of data that was requested?
      accel_x_out = Wire.read() << 8 | Wire.read(); 
      accel_y_out = Wire.read() << 8 | Wire.read();
      accel_z_out = Wire.read() << 8 | Wire.read();
      Wire.read(); Wire.read(); // Ignore temperature readings
      gyro_x_out = Wire.read() << 8 | Wire.read() ;
      gyro_y_out = Wire.read() << 8 | Wire.read() ;
      gyro_z_out = Wire.read() << 8 | Wire.read() ;

      accel_x_out -= accel_x_offset; 
      accel_y_out -= accel_y_offset;
      accel_z_out -= accel_z_offset;
      gyro_x_out -= gyro_x_offset;
      gyro_y_out -= gyro_y_offset;
      gyro_z_out -= gyro_z_offset;
      success_flag = true;
    }
  }
  return success_flag;
}

void calibrateSensors() {
  int32_t accel_x_sum = 0, accel_y_sum = 0, accel_z_sum = 0;
  int32_t gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
  const int num_samples = 1000;

  for (int sample_index = 0; sample_index < num_samples; sample_index++) {
    int16_t accel_x_sample, accel_y_sample, accel_z_sample, gyro_x_sample, gyro_y_sample, gyro_z_sample;
    readMPU6050Data(accel_x_sample, accel_y_sample, accel_z_sample, gyro_x_sample, gyro_y_sample, gyro_z_sample);
    accel_x_sum += accel_x_sample;
    accel_y_sum += accel_y_sample;
    accel_z_sum += accel_z_sample;
    gyro_x_sum += gyro_x_sample;
    gyro_y_sum += gyro_y_sample;
    gyro_z_sum += gyro_z_sample;
    delay(3); // Small delay between readings
    digitalWrite(LED_GRN_PIN, digitalRead(LED_GRN_PIN) == HIGH ? LOW : HIGH);
  }

  // Calculate and store the offsets
  accel_x_offset = accel_x_sum / num_samples;
  accel_y_offset = accel_y_sum / num_samples;
  accel_z_offset = (accel_z_sum / num_samples) - 16384; // Adjust for gravity
  gyro_x_offset = gyro_x_sum / num_samples;
  gyro_y_offset = gyro_y_sum / num_samples;
  gyro_z_offset = gyro_z_sum / num_samples;

  Serial.print("ax: ");
  Serial.print(accel_x_offset);
  Serial.print(" ay: ");
  Serial.print(accel_y_offset);
  Serial.print(" az: ");
  Serial.print(accel_z_offset);
  Serial.print(" gx: ");
  Serial.print(gyro_x_offset);
  Serial.print(" gy: ");
  Serial.print(gyro_y_offset);
  Serial.print(" gz: ");
  Serial.println(gyro_z_offset);
  delay(2000);
}

bool isTiltedBeyondThreshold() {
  return (fabs(roll_angle) > tiltThreshold || fabs(pitch_angle) > tiltThreshold);
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
  int baseSpeed = 6500; // Example value, adjust based on your requirements
  //int minPWM = 0;
  //int maxPWM = 8191;

  int minPWM = baseSpeed * 0.4;
  int maxPWM = baseSpeed * 1.3;

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


void attitudePrint() {
  // Print the results
  Serial.print("R: ");
  Serial.print(roll_angle);
  Serial.print("\tP: ");
  Serial.print(pitch_angle);
  Serial.print("\tY: ");
  Serial.print(yaw_angle);
  Serial.print("\tF: ");
  Serial.print(current_frequency);
  Serial.println(" Hz");
}

void sensorPrint(float accel_x_print, float accel_y_print, float accel_z_print, float gyro_x_print, float gyro_y_print, float gyro_z_print) {
  Serial.print("aX: ");
  Serial.print(accel_x_print, 2);  // Print accel_x_print with 4 decimal places

  Serial.print(" aY: ");
  Serial.print(accel_y_print, 2);  // Print accel_y_print with 4 decimal places

  Serial.print(" aZ: ");
  Serial.print(accel_z_print, 2);  // Print accel_z_print with 4 decimal places

  Serial.print(" gX: ");
  Serial.print(gyro_x_print, 2);   // Print gyro_x_print with 4 decimal places

  Serial.print(" gY: ");
  Serial.print(gyro_y_print, 2);   // Print gyro_y_print with 4 decimal places

  Serial.print(" gZ: ");
  Serial.println(gyro_z_print, 2);   // Print gyro_z_print with 4 decimal places
}
