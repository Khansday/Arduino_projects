/*
 THIS CODE READS THE MPU6050 AT THE MAX SPEED SUPPORTED 
 PRINTS THE ROLL YAW AND PITCH AS FAST AS POSSIBLE
 THE LOOP FREQUENCY IS LIMITED 
*/

#include <Wire.h>
#include <math.h>

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

double roll_rate = 0.0;
double pitch_rate = 0.0;
double yaw_rate = 0.0;

float old_roll{};
float old_pitch{};
float old_yaw{};

float roll_target_error {};
float pitch_target_error {};
float target_Kp = 0.50f;

unsigned long previous_time = 0;
const unsigned long loop_frequency = 2000;  //in microseconds
float current_frequency;

float accel_threshold = 0.2f; //variable to decide when to use or discard accelerometer data

int debug_cycles = 10, debug_counter = debug_cycles;  //debug printing variable and cycles

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

      calculateRate();

      float roll_target = squareRootControl(roll_angle);
      float pitch_target = squareRootControl(pitch_angle);

      roll_target_error = roll_target - roll_rate;
      pitch_target_error = pitch_target - pitch_rate;

      // Print the results
      if (--debug_counter == 0) {
        //attitudePrint();
        //attitudeRatePrint();
        //targetPrint(roll_target, pitch_target);
        targetErrorPrint();
        //ensorPrint(accel_x_g, accel_y_g, accel_z_g, gyro_x_dps, gyro_y_dps, gyro_z_dps);
        debug_counter = debug_cycles;
      }
    }
  }
}

void calculateRate(){
      roll_rate = double(roll_angle - old_roll) ;
      pitch_rate = double(pitch_angle - old_pitch) ;
      yaw_rate = double(yaw_angle - old_yaw) ;

      old_roll = roll_angle;
      old_pitch = pitch_angle;
      old_yaw = yaw_angle;
}

float squareRootControl(float attitude_axis){
  int signOfX = (attitude_axis > 0) - (attitude_axis < 0);
  double target = target_Kp * (sqrt(fabs(attitude_axis))) * signOfX;
  return target;
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

void attitudeRatePrint() {
  // Print the results
  Serial.print("R: ");
  Serial.print(roll_rate);
  Serial.print("\tP: ");
  Serial.print(pitch_rate);
  Serial.print("\tY: ");
  Serial.print(yaw_rate);
  Serial.print("\tF: ");
  Serial.print(current_frequency);
  Serial.println(" Hz");
}

void targetPrint(float roll_target_print, float pitch_target_print){
  Serial.print("R: ");
  Serial.print(roll_target_print);
  Serial.print("\tP: ");
  Serial.print(pitch_target_print);
  Serial.print("\tF: ");
  Serial.print(current_frequency);
  Serial.println(" Hz"); 
}

void targetErrorPrint(){
  Serial.print("R: ");
  Serial.print(roll_target_error);
  Serial.print("\tP: ");
  Serial.print(pitch_target_error);
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
