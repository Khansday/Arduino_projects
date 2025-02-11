// (c) Michael Schoeffler 2017, http://www.mschoeffler.de

#include "Wire.h" // This library allows you to communicate with I2C devices.

int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

float accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
float gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int temperature; // variables for temperature data 
 
// 250 deg/s --> 131.0, 500 deg/s --> 65.5, 1000 deg/s --> 32.8, 2000 deg/s --> 16.4
float scaleFactorGyro = 65.5;
// 2g --> 16384 , 4g --> 8192 , 8g --> 4096, 16g --> 2048
float scaleFactorAccel = 8192;

long timer;
float freq;

float aZ, aX, gY;

float accRoll, roll;
float tau = 0.98; 

float gyro_y_cal = -66.3;


void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)

  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)

  setup_mpu_6050_registers();
  //calibrate_sensor(); only do it once

}
void loop() {
  freq = 1/((micros() - timer) * 1e-6);
  timer = micros();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable AND PROCESSED
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  gyro_y -= gyro_y_cal;   //add this after finding offset

  aZ = accelerometer_z / scaleFactorAccel;
  aX = accelerometer_x / scaleFactorAccel;
  gY = gyro_y / scaleFactorGyro;

  accRoll = atan2(aX, aZ) * RAD_TO_DEG;
  roll =  (tau)*(roll - gY* (1/freq)) + (1-tau)*(accRoll);
  // print out data
 /* Serial.print("aX = "); Serial.print((accelerometer_x ));  //16384.0
  Serial.print(" | aY = "); Serial.print((accelerometer_y));
  Serial.print(" | aZ = "); Serial.print((accelerometer_z));
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
  Serial.print(" | gX = "); Serial.print((gyro_x ));  // 131.0
  Serial.print(" | gY = "); Serial.print((gyro_y ));
  Serial.print(" | gZ = "); Serial.print((gyro_z ));
  Serial.print(" | freq = "); Serial.print((freq ));
  Serial.print(" | Accroll = "); Serial.print((accRoll ));*/
  Serial.print(" | roll = "); Serial.print((roll ));
  Serial.print(" | freq = "); Serial.print((freq ));

  Serial.println();

  while (micros() - timer <= 4000);

}

void setup_mpu_6050_registers() {
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  // Configure the accelerometer
  // Wire.write(0x__);
  // Wire.write; 2g --> 0x00, 4g --> 0x08, 8g --> 0x10, 16g --> 0x18
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission();
  // Configure the gyro
  // Wire.write(0x__);
  // 250 deg/s --> 0x00, 500 deg/s --> 0x08, 1000 deg/s --> 0x10, 2000 deg/s --> 0x18
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}


void calibrate_sensor(){
  int readings_num = 1000;  //1000 samples
  // Calibration
  Serial.println("Calibrating sensor, place on level surface and do not move.");
  // Take  readings for each coordinate and then find average offset
  for (int i = 0; i < readings_num; i ++){
    if(i % 200 == 0)Serial.print(".");  //print . every 200 cycles

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
    Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
    Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers

    // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable AND PROCESSED
    accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
    accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
    accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
    temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
    gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
    gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
    gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

    gyro_y_cal += gyro_y;
    delay(1);
  }
  // Average the values
  gyro_y_cal /= readings_num;
  Serial.println("y offset:  ");
  Serial.println(gyro_y_cal);
  delay(50000);
}
