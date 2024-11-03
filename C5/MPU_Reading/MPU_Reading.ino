
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

float roll_angle = 0.0f;
float pitch_angle = 0.0f;
float yaw_angle = 0.0f;


unsigned long previous_time = 0;
const unsigned long loop_frequency = 2000;  //in microseconds
float current_frequency;

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
  delay(2000);


}

void loop() {
  bool success_flag = false;
  unsigned long current_time = micros();
  unsigned long elapsed_time = current_time - previous_time;

  if (elapsed_time >= loop_frequency) { // Ensure the loop runs at approximately  
    previous_time = current_time;
    double delta_time = double(elapsed_time) / 1000000.0f;
    current_frequency = 1 / delta_time; // just use for debugging

    readMPU6050Data();
    
    scaleMPU6050Data();

    // Calculate roll and pitch from the accelerometer data
    float accel_pitch_angle = atan2(accel_y_g, accel_z_g) * 180 / PI;
    float accel_roll_angle = atan2(-accel_x_g, sqrt(accel_y_g * accel_y_g + accel_z_g * accel_z_g)) * 180 / PI;
    


    if (--debug_counter == 0) {

      //sensorPrint();
      scaeledValuePrint();
      debug_counter = debug_cycles;
    }
    
  }
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




// Calculate roll and pitch from the accelerometer data
      float accel_pitch_angle = atan2(accel_y_g, accel_z_g) * 180 / PI;
      float accel_roll_angle = atan2(-accel_x_g, sqrt(accel_y_g * accel_y_g + accel_z_g * accel_z_g)) * 180 / PI;
