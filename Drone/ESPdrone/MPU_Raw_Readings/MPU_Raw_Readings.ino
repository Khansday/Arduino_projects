#include <Wire.h>

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050

void setup() {
  Serial.begin(115200);
  Wire.begin(11, 10); // Initialize I2C with SDA on GPIO11 and SCL on GPIO10

  // Initialize MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Set to 0 to wake up the MPU-6050
  byte error = Wire.endTransmission();
 
  // Check if the MPU6050 is connected properly
  if (error == 0) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.print("MPU6050 connection failed with error: ");
    Serial.println(error);
    while (1); // Halt the program if connection fails
  }

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
}

void loop() {
  // Read raw values from MPU6050
  int16_t ax, ay, az, gx, gy, gz;
  if (readMPU6050Data(ax, ay, az, gx, gy, gz)) {
    // Print raw values
    Serial.print("AX: "); Serial.print(ax);
    Serial.print(" AY: "); Serial.print(ay);
    Serial.print(" AZ: "); Serial.print(az);
    Serial.print(" GX: "); Serial.print(gx);
    Serial.print(" GY: "); Serial.print(gy);
    Serial.print(" GZ: "); Serial.println(gz);
  } else {
    Serial.println("Failed to read from MPU6050");
  }
 
  // Wait for the next loop
  delay(100);
}

bool readMPU6050Data(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Starting register for accelerometer readings
  byte error = Wire.endTransmission(false);
  if (error != 0) {
    return false; // Return false if communication failed
  }

  Wire.requestFrom(MPU_ADDR, 14, true); // Request 14 bytes
  if (Wire.available() == 14) {
    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // Ignore temperature readings
    gx = Wire.read() << 8 | Wire.read();
    gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();
    return true; // Return true if data read successfully
  } else {
    return false; // Return false if data not available
  }
}