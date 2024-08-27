
// These must be defined before including TinyEKF.h
#define EKF_N 3  // Roll, Pitch, Yaw
#define EKF_M 6  // AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ

#include <Wire.h>
#include <math.h>
#include <tinyekf.h>


// Define process noise covariance matrix Q
static const float Q[EKF_N*EKF_N] = {
    0.01, 0,    0,
    0,    0.01, 0,
    0,    0,    0.01
};

// Define measurement noise covariance matrix R
static const float R[EKF_M*EKF_M] = {
    0.1, 0,   0,   0,   0,   0,
    0,   0.1, 0,   0,   0,   0,
    0,   0,   0.1, 0,   0,   0,
    0,   0,   0,   0.1, 0,   0,
    0,   0,   0,   0,   0.1, 0,
    0,   0,   0,   0,   0,   0.1
};

// Define state-transition matrix F (identity matrix)
static const float F[EKF_N*EKF_N] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
};

// Define measurement-function Jacobian matrix H
static const float H[EKF_M*EKF_N] = {
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
};

// Instantiate EKF structure
static ekf_t ekf;

#define LED_BLE_PIN 7
#define LED_RED_PIN 8
#define LED_GRN_PIN 9

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
double dt = 0.001; // Initial dt value for the first loop
int loopFreq = 1000;  //in microseconds

float linear_acc_threshold = 0.2; //variable to decide when to use or discard accelerometer data

int i = 0, cyclesPrint = 10;  //debug printing variable and cycles

void setup() {
    pinMode(LED_GRN_PIN, OUTPUT);
    pinMode(LED_BLE_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);

    Serial.begin(921600); // highest speed possible on my setup, higher than this I get errors
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

    // Use identity matrix as initial covariance matrix
    const float Pdiag[EKF_N] = {1, 1, 1};
    ekf_initialize(&ekf, Pdiag);

    // Initial delay to allow the sensor to stabilize
    delay(100);
}

void loop() {
  currentTime = micros();
  dt = (currentTime - lastTime);

  if (dt >= loopFreq) {
      lastTime = currentTime;
      dt /= 1000000;

      // Read raw values from MPU6050
      int16_t ax, ay, az, gx, gy, gz;
      readMPU6050Data(ax, ay, az, gx, gy, gz);

      // Convert raw values to accelerometer (g) and gyroscope (degrees/second)
      float accelX = ax / 16384.0;
      float accelY = ay / 16384.0;
      float accelZ = az / 16384.0;
      float gyroX = gx / 131.0;
      float gyroY = gy / 131.0;
      float gyroZ = gz / 131.0;

      // Process model: integrate gyro data
      const float fx[EKF_N] = {
          ekf.x[0] + gyroX * dt,  // Roll
          ekf.x[1] + gyroY * dt,  // Pitch
          ekf.x[2] + gyroZ * dt   // Yaw
      };

      // Run the prediction step of the EKF
      ekf_predict(&ekf, fx, F, Q);

const float hx[EKF_M] = {
    accelX,                             // Expected accel X based on roll
    accelY,                             // Expected accel Y based on pitch
    accelZ,                             // Expected accel Z should remain around 1g if stationary
    ekf.x[0],                           // Expected gyro X (roll rate)
    ekf.x[1],                           // Expected gyro Y (pitch rate)
    ekf.x[2]                            // Expected gyro Z (yaw rate)
};


      // Observation vector z
      const float z[EKF_M] = {accelX, accelY, accelZ, gyroX, gyroY, gyroZ};

      // Run the update step
      if (ekf_update(&ekf, z, hx, H, R)) {
          roll = ekf.x[0];
          pitch = ekf.x[1];
          yaw = ekf.x[2];
      }

      // Output or debug information
      attitudePrint();
      //sensorPrint(accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
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

void attitudePrint() {
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

void sensorPrint(float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ) {
    Serial.print("accelX: ");
    Serial.print(accelX, 2);  // Print accelX with 2 decimal places

    Serial.print(" accelY: ");
    Serial.print(accelY, 2);  // Print accelY with 2 decimal places

    Serial.print(" accelZ: ");
    Serial.print(accelZ, 2);  // Print accelZ with 2 decimal places

    Serial.print(" gyroX: ");
    Serial.print(gyroX, 2);   // Print gyroX with 2 decimal places

    Serial.print(" gyroY: ");
    Serial.print(gyroY, 2);   // Print gyroY with 2 decimal places

    Serial.print(" gyroZ: ");
    Serial.println(gyroZ, 2); // Print gyroZ with 2 decimal places
}
