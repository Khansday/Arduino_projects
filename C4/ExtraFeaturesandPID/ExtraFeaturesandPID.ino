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

float gyro_y_cal = 195;

//pins for the motor driver
int M1_d = 8;
int M1_dr = 7;
int M1_speed = 6;
int M2_d = 10;
int M2_dr = 9;
int M2_speed = 11;
int max_speed = 180; 
int min_speed = 140;

int pwm;                //value of the speed of the motors
float err, Kp = 15.00,  Kd = 0, Ki = 0;// error of the PID system and P constant

float set_point_offset = 1.5;  //allowing set point calibration
int BALANCE_LED = 5;    //LED that lights up when robot is balanced




void setup() {
  Serial.begin(115200);
  Wire.begin();

  setup_mpu_6050_registers();
  //calibrate_sensor(); //only do it once

  //set motor driver pin as output
  pinMode(M1_d , OUTPUT);
  pinMode(M1_dr , OUTPUT);
  pinMode(M1_speed , OUTPUT);
  pinMode(M2_d , OUTPUT);
  pinMode(M2_dr , OUTPUT);
  pinMode(M2_speed , OUTPUT);

  pinMode(BALANCE_LED, OUTPUT);

}
void loop() {
  freq = 1/((micros() - timer) * 1e-6);
  timer = micros();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with accelerometer register 
  Wire.endTransmission(false); //connection is kept active.
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

  // Complementary filter
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
  Serial.print(" | roll = "); Serial.print((roll - set_point_offset ));
  Serial.println();

  balance_LED();  //new function to control the movement of the robot
  
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

void balance_LED(){
    err = roll - set_point_offset;
    if (err > -0.3 && err < 0.3){ //if almost balanced with 0.3 degrees of tollerance
    digitalWrite(BALANCE_LED ,0);  //turn the LED ON
    stop();
  }
  
  else if((roll - set_point_offset) > 30 || (roll - set_point_offset) < -30){ //if tipped over
    stop();
  }

  else{
    digitalWrite(BALANCE_LED ,1);   //turn the LED OFF
    // moved here because I dont wanna update motors if no need
   move_robot();      //update the pwm and direction
  }
}

void move_robot(){
  
  pwm = err * Kp ;  //value with PID loop
  pwm = abs(pwm);
  pwm = constrain(pwm, min_speed, max_speed);

  if (err > 0) {
    digitalWrite(M1_d , HIGH);
    digitalWrite(M1_dr , LOW);
    digitalWrite(M2_d , HIGH);
    digitalWrite(M2_dr , LOW);
    analogWrite(M1_speed , pwm );  
    analogWrite(M2_speed , pwm);
  }
  else {
    digitalWrite(M1_d , LOW);
    digitalWrite(M1_dr , HIGH);
    digitalWrite(M2_d , LOW);
    digitalWrite(M2_dr , HIGH);
    analogWrite(M1_speed , pwm );  
    analogWrite(M2_speed , pwm);
  }
}

void stop(){
  digitalWrite(M1_d , LOW);
    digitalWrite(M1_dr , LOW);
    digitalWrite(M2_d , LOW);
    digitalWrite(M2_dr , LOW);
}