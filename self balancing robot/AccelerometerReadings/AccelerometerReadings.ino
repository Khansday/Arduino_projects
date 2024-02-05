// (c) Michael Schoeffler 2017, http://www.mschoeffler.de

#include "Wire.h" // This library allows you to communicate with I2C devices.

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data 

int16_t Oaccelerometer_x, Oaccelerometer_y, Oaccelerometer_z; // variables for accelerometer raw data
int16_t Ogyro_x, Ogyro_y, Ogyro_z; // variables for gyro raw data

long int ErrAx = 0, ErrAy = 0, ErrAz = 0, ErrGy= 0, ErrGx= 0, ErrGz= 0;

char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)

  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  calculateError();
  Serial.println(ErrAx);
  Serial.println(ErrAy);
  Serial.println(ErrAz);
  Serial.println(ErrGx);
  Serial.println(ErrGy);
  Serial.println(ErrGz);
  delay(5000);
}
void loop() {
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

  //Apply filters
  accelerometer_x = accelerometer_x * 0.2 + Oaccelerometer_x * 0.8;
  accelerometer_y = accelerometer_y * 0.2 + Oaccelerometer_y * 0.8;
  accelerometer_z = accelerometer_z * 0.2 + Oaccelerometer_z * 0.8;

  gyro_x = gyro_x * 0.2 + Ogyro_x * 0.8;
  gyro_y = gyro_y * 0.2 + Ogyro_y * 0.8;
  gyro_z = gyro_z * 0.2 + Ogyro_z * 0.8;

  //update old values for next iteration
  Oaccelerometer_x = accelerometer_x;
  Oaccelerometer_y = accelerometer_y;
  Oaccelerometer_z = accelerometer_z;
  Ogyro_x = gyro_x;
  Ogyro_y = gyro_y;
  Ogyro_z = gyro_z;


  // print out data
  Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x - ErrAx));  //16384.0
  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y - ErrAy));
  Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z - ErrAz));
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
  Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x - ErrGx));  // 131.0
  Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y - ErrGy ));
  Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z - ErrGz ));
  Serial.println();
  
  // delay
  delay(33);
}


void calculateError(){
  int readings = 2000;
  for (int i = 0; i<readings; i++ )
  {
    Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  //temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  ErrAx = ErrAx + accelerometer_x, 
  ErrAy = ErrAy + accelerometer_y, 
  ErrAz = ErrAz + accelerometer_z, 
  ErrGy= ErrGy + gyro_y, 
  ErrGx= ErrGx + gyro_x, 
  ErrGz= ErrGz + gyro_z;
  }
  ErrAx = ErrAx /readings, 
  ErrAy = ErrAy /readings, 
  ErrAz = ErrAz /readings, 
  ErrGy= ErrGy /readings, 
  ErrGx= ErrGx /readings, 
  ErrGz= ErrGz /readings;
}