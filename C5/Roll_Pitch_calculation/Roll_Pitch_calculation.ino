#include <Wire.h>

#define LED_BLE_PIN 7
#define LED_RED_PIN 8
#define LED_GRN_PIN 9
#define SDA_PIN 11
#define SCL_PIN 10

const int MPU_ADDRESS = 0x68; // I2C address of the MPU-6050
const int CONFIG_REGISTER = 0x1A;  // Address of the low pass filter register

int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
float accel_x_g, accel_y_g, accel_z_g, gyro_x_dps, gyro_y_dps, gyro_z_dps;

// Offset variables
int16_t accel_x_offset = 0, accel_y_offset = 0, accel_z_offset = 0;
int16_t gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;


float accel_pitch_angle, accel_roll_angle, accel_magnitude;
float accel_threshold = 0.2; //variable to decide when to use or discard accelerometer data

float gyro_pitch_angle, gyro_roll_angle, gyro_yaw_angle;

float roll_angle = 0.0f;
float pitch_angle = 0.0f;
float yaw_angle = 0.0f;


unsigned long previous_time = 0;
const unsigned long loop_frequency = 2000;  //in microseconds
float current_frequency;
double delta_time;

int debug_cycles = 100, debug_counter = debug_cycles;  //debug printing variable and cycles

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
  delay(20);

  //Starts calibrartion
  delay(2000);
  digitalWrite(LED_BLE_PIN, HIGH);
  calibrateSensors();
  digitalWrite(LED_BLE_PIN, LOW);

}

void loop() {
  bool success_flag = false;
  unsigned long current_time = micros();
  unsigned long elapsed_time = current_time - previous_time;

  if (elapsed_time >= loop_frequency) { // Ensure the loop runs at approximately  
    previous_time = current_time;
    delta_time = double(elapsed_time) / 1000000.0f;
    current_frequency = 1 / delta_time; // just use for debugging

    readMPU6050Data();
    
    scaleMPU6050Data();

    AccelerometerDataProcessing();
  
    GyroDataProcessing();

    magnitudeLEDs();



    if (--debug_counter == 0) {

      //sensorPrint();
      //scaeledValuePrint();
      //accDataPrint();
      gyroDataPrint(); 
      debug_counter = debug_cycles;
    }
    
  }
}

void calibrateSensors() {
  int32_t accel_x_sum = 0, accel_y_sum = 0, accel_z_sum = 0;
  int32_t gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
  const int num_samples = 1000;

  for (int sample_index = 0; sample_index < num_samples; sample_index++) {
    int16_t accel_x_sample, accel_y_sample, accel_z_sample, gyro_x_sample, gyro_y_sample, gyro_z_sample;
    readMPU6050Data();
    accel_x_sum += accel_x;
    accel_y_sum += accel_y;
    accel_z_sum += accel_z;
    gyro_x_sum += gyro_x;
    gyro_y_sum += gyro_y;
    gyro_z_sum += gyro_z;
    delay(1); // Small delay between readings
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

void readMPU6050Data() {
  bool success_flag = false;

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B); // Starting register for accelerometer readings
  int transmission_error = Wire.endTransmission(false);
  if (transmission_error == 0) {
    int bytes_received = Wire.requestFrom(MPU_ADDRESS, 14, true); // Request 14 bytes
    if (bytes_received == 14) { // received the amount of data that was requested?
      accel_x = Wire.read() << 8 | Wire.read(); 
      accel_y = Wire.read() << 8 | Wire.read();
      accel_z = Wire.read() << 8 | Wire.read();
      Wire.read(); Wire.read(); // Ignore temperature readings
      gyro_x = Wire.read() << 8 | Wire.read() ;
      gyro_y = Wire.read() << 8 | Wire.read() ;
      gyro_z = Wire.read() << 8 | Wire.read() ;

      accel_x -= accel_x_offset; 
      accel_y -= accel_y_offset;
      accel_z -= accel_z_offset;
      gyro_x -= gyro_x_offset;
      gyro_y -= gyro_y_offset;
      gyro_z -= gyro_z_offset;

      success_flag = true;
    }
  }
  //return success_flag;
}

void scaleMPU6050Data(){
      // Convert raw values to accelerometer (g) and gyroscope (degrees/second)
      // Negative signs to match the orientation of the drone
      accel_x_g = -accel_x / 16384.0;
      accel_y_g = -accel_y / 16384.0;
      accel_z_g = accel_z / 16384.0;
      gyro_x_dps = -gyro_x / 131.0;
      gyro_y_dps = -gyro_y / 131.0;
      gyro_z_dps = gyro_z / 131.0;
}

void AccelerometerDataProcessing(){
  // Calculate roll and pitch from the accelerometer data
  accel_pitch_angle = atan2(accel_y_g, accel_z_g) * 180 / PI;
  accel_roll_angle = atan2(-accel_x_g, sqrt(accel_y_g * accel_y_g + accel_z_g * accel_z_g)) * 180 / PI;
    
  // Calculate the magnitude of the acceleration vector
  accel_magnitude = sqrt(accel_x_g * accel_x_g + accel_y_g * accel_y_g + accel_z_g * accel_z_g);
}

void GyroDataProcessing(){
  // Integrate gyroscope data to get angles, accounting for cross-coupling effects without small angle approximation
  gyro_roll_angle += gyro_y_dps * delta_time + gyro_x_dps * sin(gyro_roll_angle * PI / 180) * tan(gyro_pitch_angle * PI / 180) * delta_time + gyro_z_dps * cos(gyro_roll_angle * PI / 180) * tan(gyro_pitch_angle * PI / 180) * delta_time;
  gyro_pitch_angle += gyro_x_dps * cos(gyro_roll_angle * PI / 180) * delta_time - gyro_z_dps * sin(gyro_roll_angle * PI / 180) * delta_time;
  gyro_yaw_angle += (gyro_x_dps * sin(gyro_roll_angle * PI / 180) / cos(gyro_pitch_angle * PI / 180) + gyro_z_dps * cos(gyro_roll_angle * PI / 180) / cos(gyro_pitch_angle * PI / 180)) * delta_time;
} 

void magnitudeLEDs(){
  // Adjust the complementary filter coefficient based on the acceleration magnitude
  if (fabs(1.0 - accel_magnitude) < accel_threshold) {
    
    // Visualize on the drone the accelerometer data is valid
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GRN_PIN, HIGH);

  } else {
    // Visualize on the drone the accelerometer data is discarded
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GRN_PIN, LOW);
  }
}

void sensorPrint() {
  Serial.print("aX: ");
  Serial.print(accel_x);  // Print accel_x_print with 4 decimal places

  Serial.print(" aY: ");
  Serial.print(accel_y);  // Print accel_y_print with 4 decimal places

  Serial.print(" aZ: ");
  Serial.print(accel_z);  // Print accel_z_print with 4 decimal places

  Serial.print(" gX: ");
  Serial.print(gyro_x);   // Print gyro_x_print with 4 decimal places

  Serial.print(" gY: ");
  Serial.print(gyro_y);   // Print gyro_y_print with 4 decimal places

  Serial.print(" gZ: ");
  Serial.print(gyro_z);   // Print gyro_z_print with 4 decimal places

  Serial.print(" F: ");
  Serial.println(current_frequency, 2);   // Print gyro_z_print with 4 decimal places
}

void scaeledValuePrint() {
  Serial.print("aXg: ");
  Serial.print(accel_x_g);  // Print accel_x_print with 4 decimal places

  Serial.print(" aYg: ");
  Serial.print(accel_y_g);  // Print accel_y_print with 4 decimal places

  Serial.print(" aZg: ");
  Serial.print(accel_z_g);  // Print accel_z_print with 4 decimal places

  Serial.print(" gXdps: ");
  Serial.print(gyro_x_dps);   // Print gyro_x_print with 4 decimal places

  Serial.print(" gYdps: ");
  Serial.print(gyro_y_dps);   // Print gyro_y_print with 4 decimal places

  Serial.print(" gZdps: ");
  Serial.print(gyro_z_dps);   // Print gyro_z_print with 4 decimal places

  Serial.print(" F: ");
  Serial.println(current_frequency, 2);   // Print gyro_z_print with 4 decimal places
}

void accDataPrint() {
  Serial.print("Acc Roll: ");
  Serial.print(accel_roll_angle);   

  Serial.print(" Acc Pitch: ");
  Serial.print(accel_pitch_angle); 

  Serial.print(" Acc Magnitude: ");
  Serial.print(accel_magnitude);   

  Serial.print(" F: ");
  Serial.println(current_frequency, 2);   // Print gyro_z_print with 4 decimal places
}

void gyroDataPrint() {
  Serial.print("Gyro Roll: ");
  Serial.print(gyro_roll_angle);   

  Serial.print(" Gyro Pitch: ");
  Serial.print(gyro_pitch_angle); 

  Serial.print(" Gyro Yaw: ");
  Serial.print(gyro_yaw_angle);   

  Serial.print(" F: ");
  Serial.println(current_frequency, 2);   // Print gyro_z_print with 4 decimal places
}

void complemetaryFilter(){
  
}

