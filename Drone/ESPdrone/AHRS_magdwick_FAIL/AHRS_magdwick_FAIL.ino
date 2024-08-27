/*
 THIS CODE READS THE MPU6050 AT THE MAX SPEED SUPPORTED 
 PRINTS THE ROLL YAW AND PITCH AS FAST AS POSSIBLE
 THE LOOP FREQUENCY IS LIMITED AT 1 KHZ 
*/

#include <Wire.h>
#include "SensorFusion.h" //SF
SF fusion;


const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
const int CONFIG = 0x1A;  // Address of the low pass filter register

// Add these variables for offsets
int16_t ax_offset = -263, ay_offset = -95, az_offset = -1157;
int16_t gx_offset = -226, gy_offset = 58, gz_offset = 179;

float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

unsigned long lastTime = 0;
unsigned long currentTime = 0;
double dt = 0.002; // Initial dt value for the first loop
int loopFreq = 2000;  //in microseconds

int i=0, cyclesPrint = 10;  //debug printing variable and cycles

void setup() {
  Serial.begin(250000); // highest speed possible on my setup, higher than this I get errors
  Wire.begin(11, 10); // Initialize I2C with SDA on GPIO11 and SCL on GPIO10
  Wire.setClock(200000); // Clock speed at 1MHz (highest available)

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
}

void loop() {
  currentTime = micros();
  dt = (currentTime - lastTime); // Convert dt to seconds

  if (dt >= loopFreq) { // Ensure the loop runs at approximately 1 kHz (1ms per loop)
    lastTime = currentTime;
    dt /= 1000000;

    // Read raw values from MPU6050
    int16_t ax, ay, az, gx, gy, gz;
    readMPU6050Data(ax, ay, az, gx, gy, gz);
   
    // Convert raw values to accelerometer (g) and gyroscope (degrees/second)
    // Negative signs to match the orientation of the drone
    float accelX = -ax / 16384.0;
    float accelY = -ay / 16384.0;
    float accelZ = az / 16384.0;
    float gyroX = -gx / 131.0 *DEG_TO_RAD;
    float gyroY = -gy / 131.0 *DEG_TO_RAD;
    float gyroZ = gz / 131.0 *DEG_TO_RAD;
   
    dt = fusion.deltatUpdate(); //this have to be done before calling the fusion update
    //fusion.MahonyUpdate(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, dt);
    fusion.MadgwickUpdate(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, dt);  //mahony is suggested if there isn't the mag and the mcu is slow

  pitch = fusion.getPitch();
  roll = fusion.getRoll();    //you could also use getRollRadians() ecc
  yaw = fusion.getYaw();

    i++;
    // Print the results
    if (i = cyclesPrint){
      debugPrint();
      // Serial.print("accelX: ");
      // Serial.print(accelX, 2);  // Print accelX with 4 decimal places

      // Serial.print(" accelY: ");
      // Serial.print(accelY, 2);  // Print accelY with 4 decimal places

      // Serial.print(" accelZ: ");
      // Serial.print(accelZ, 2);  // Print accelZ with 4 decimal places

      // Serial.print(" gyroX: ");
      // Serial.print(gyroX, 2);   // Print gyroX with 4 decimal places

      // Serial.print(" gyroY: ");
      // Serial.print(gyroY,2);   // Print gyroY with 4 decimal places

      // Serial.print(" gyroZ: ");
      // Serial.println(gyroZ, 2);   // Print gyroZ with 4 decimal places
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

