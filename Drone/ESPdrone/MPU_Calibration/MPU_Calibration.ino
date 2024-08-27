/*
 THIS CODE READS THE MPU6050 AT THE MAX SPEED SUPPORTED 
 PRINTS THE ROLL YAW AND PITCH AS FAST AS POSSIBLE
 THE LOOP FREQUENCY IS LIMITED AT 1 KHZ 
*/

#include <Wire.h>
#include <math.h>

#define LED_BLE_PIN 7
#define LED_RED_PIN 8
#define LED_GRN_PIN 9

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
const int CONFIG = 0x1A;  // Address of the low pass filter register
float alpha = 0.98;  // Initial complementary filter constant

// Offset variables
int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;

float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

unsigned long lastTime = 0;
unsigned long currentTime = 0;
float dt = 0.001; // Initial dt value for the first loop


void setup() {
  pinMode(LED_GRN_PIN, OUTPUT);
  pinMode(LED_BLE_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);

  Serial.begin(921600); // Highest speed possible on my setup, higher than this I get errors
  Wire.begin(11, 10); // Initialize I2C with SDA on GPIO11 and SCL on GPIO10
  Wire.setClock(1000000); // Clock speed at 1MHz (highest available)

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
  Serial.println();
  delay(3000);
  for (int i=0; i <10; i++){
    // Perform sensor calibration
    calibrateSensors();
  }
  

  // Initial delay to allow the sensor to stabilize
  delay(100);
}

void loop() {
  
}

void readMPU6050Data(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Starting register for accelerometer readings
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // Request 14 bytes

  ax = Wire.read() << 8 | Wire.read() ; 
  ay = Wire.read() << 8 | Wire.read() ;
  az = Wire.read() << 8 | Wire.read() ;
  Wire.read(); Wire.read(); // Ignore temperature readings
  gx = Wire.read() << 8 | Wire.read() ;
  gy = Wire.read() << 8 | Wire.read() ;
  gz = Wire.read() << 8 | Wire.read() ;
}


void calibrateSensors() {
  int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
  int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
  const int num_samples = 1000;

  for (int i = 0; i < num_samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readMPU6050Data(ax, ay, az, gx, gy, gz);
    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    delay(3); // Small delay between readings
  }

  // Calculate and store the offsets
  ax_offset = ax_sum / num_samples;
  ay_offset = ay_sum / num_samples;
  az_offset = (az_sum / num_samples) - 16384; // Adjust for gravity
  gx_offset = gx_sum / num_samples;
  gy_offset = gy_sum / num_samples;
  gz_offset = gz_sum / num_samples;

  Serial.print("ax: ");
  Serial.print(ax_offset);
  Serial.print(" ay: ");
  Serial.print(ay_offset);
  Serial.print(" az: ");
  Serial.print(az_offset);
  Serial.print(" gx: ");
  Serial.print(gx_offset);
  Serial.print(" gy: ");
  Serial.print(gy_offset);
  Serial.print(" gz: ");
  Serial.println(gz_offset);
  delay(500);
}
