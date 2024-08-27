#include <Wire.h>
#include <math.h>

#define LED_BLE_PIN 7
#define LED_RED_PIN 8
#define LED_GRN_PIN 9

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
const int CONFIG = 0x1A;  // Address of the low pass filter register

// // Add these variables for offsets
// int16_t ax_offset = -263, ay_offset = -95, az_offset = -1157;
// int16_t gx_offset = -226, gy_offset = 58, gz_offset = 179;

// Offset variables
int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;


float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

unsigned long lastTime = 0;
unsigned long currentTime = 0;
double dt = 0.002; // Initial dt value for the first loop
int loopFreq = 2000;  //in microseconds

float linear_acc_threshold = 0.2; //variable to decide when to use or discard accelerometer data

int i=0, cyclesPrint = 10;  //debug printing variable and cycles

// Kalman filter variables
float Q_angle = 0.05f;
float Q_bias = 0.05f;
float R_measure = 0.8f; 

float angleRoll = 0.0f; // Reset the roll angle
float biasRoll = 0.0f; // Reset roll bias
float P_roll[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}}; // Error covariance matrix for roll

float anglePitch = 0.0f; // Reset the pitch angle
float biasPitch = 0.0f; // Reset pitch bias
float P_pitch[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}}; // Error covariance matrix for pitch

void setup() {
  pinMode(LED_GRN_PIN, OUTPUT);
  pinMode(LED_BLE_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);

  Serial.begin(250000); // highest speed possible on my setup, higher than this I get errors
  Wire.begin(11, 10); // Initialize I2C with SDA on GPIO11 and SCL on GPIO10
  Wire.setClock(200000); // Clock speed at 200 KHz 


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
  delay(2000);

  //calibrate sensor at each startup
  digitalWrite(LED_BLE_PIN, HIGH);
  calibrateSensors();
  digitalWrite(LED_BLE_PIN, LOW);

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
    float gyroX = -gx / 131.0;
    float gyroY = -gy / 131.0;
    float gyroZ = gz / 131.0;
   
    // Calculate roll and pitch from the accelerometer data
    float accelPitch = atan2(accelY, accelZ) * 180 / PI;
    float accelRoll = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;

    // Use the Kalman filter for roll and pitch
    roll = getKalmanAngle(accelRoll, gyroY, dt, angleRoll, biasRoll, P_roll);
    pitch = getKalmanAngle(accelPitch, gyroX, dt, anglePitch, biasPitch, P_pitch);
    
    // Yaw is still calculated using the gyroscope only, as accelerometer doesn't give yaw information
    yaw += (gyroX * sin(roll * PI / 180) / cos(pitch * PI / 180) + gyroZ * cos(roll * PI / 180) / cos(pitch * PI / 180)) * dt;
    
    i++;
    // Print the results
    if (i == cyclesPrint){
      attitudePrint();
      //sensorPrint(accelX,accelY,accelZ,gyroX,gyroY,gyroZ);
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
    digitalWrite(LED_GRN_PIN, digitalRead(LED_GRN_PIN) == HIGH ? LOW : HIGH);

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
  delay(2000);
}

float getKalmanAngle(float newAngle, float newRate, float dt, float &angle, float &bias, float P[2][2]) {
    // Predict
    float rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Update
    float S = P[0][0] + R_measure;
    float K[2]; // Kalman gain
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}

void attitudePrint(){
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

void sensorPrint(float accelX,float accelY,float accelZ,float gyroX,float gyroY,float gyroZ){
  Serial.print("accelX: ");
  Serial.print(accelX, 2);  // Print accelX with 4 decimal places

  Serial.print(" accelY: ");
  Serial.print(accelY, 2);  // Print accelY with 4 decimal places

  Serial.print(" accelZ: ");
  Serial.print(accelZ, 2);  // Print accelZ with 4 decimal places

  Serial.print(" gyroX: ");
  Serial.print(gyroX, 2);   // Print gyroX with 4 decimal places

  Serial.print(" gyroY: ");
  Serial.print(gyroY,2);   // Print gyroY with 4 decimal places

  Serial.print(" gyroZ: ");
  Serial.println(gyroZ, 2);   // Print gyroZ with 4 decimal places
}
