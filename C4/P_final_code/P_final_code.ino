// This library allows you to communicate with I2C devices.
#include "Wire.h" 

// I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int MPU_ADDR = 0x68; 

// variables for accelerometer raw data
float accelerometer_x, accelerometer_y, accelerometer_z; 
// variables for gyro raw data
float gyro_x, gyro_y, gyro_z; 
// variable for temperature data 
int temperature; 

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
float err, Kp = 15.00;// error of the PID system and P constant

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
  // starting with accelerometer register 
  Wire.write(0x3B); 
  //connection is kept active
  Wire.endTransmission(false);
  // request a total of 7*2=14 registers 
  Wire.requestFrom(MPU_ADDR, 7*2, true); 
  
  accelerometer_x = Wire.read()<<8 | Wire.read(); 
  accelerometer_y = Wire.read()<<8 | Wire.read(); 
  accelerometer_z = Wire.read()<<8 | Wire.read(); 
  temperature = Wire.read()<<8 | Wire.read();
  gyro_x = Wire.read()<<8 | Wire.read(); 
  gyro_y = Wire.read()<<8 | Wire.read();
  gyro_z = Wire.read()<<8 | Wire.read(); 

  //add this after finding offset
  gyro_y -= gyro_y_cal;   

  aZ = accelerometer_z / scaleFactorAccel;
  aX = accelerometer_x / scaleFactorAccel;
  gY = gyro_y / scaleFactorGyro;

  // Complementary filter
  accRoll = atan2(aX, aZ) * RAD_TO_DEG;
  roll =  (tau)*(roll - gY* (1/freq)) + (1-tau)*(accRoll);

  Serial.print("roll = "); 
  Serial.print((roll - set_point_offset ));
  Serial.println();
  //new function to control the movement of the robot
  balance_LED();
  //wait at least 4 ms have passed  
  while (micros() - timer <= 4000);
}

void setup_mpu_6050_registers() {
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  // Configure the accelerometer
  // Wire.write; 2g --> 0x00, 4g --> 0x08, 8g --> 0x10, 16g --> 0x18
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission();
  // Configure the gyro
  // 250 deg/s --> 0x00, 500 deg/s --> 0x08, 1000 deg/s --> 0x10, 2000 deg/s --> 0x18
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}

void balance_LED(){
    err = roll - set_point_offset;
    //if almost balanced with 0.3 degrees of tollerance
    if (err > -0.3 && err < 0.3){ 
    digitalWrite(BALANCE_LED ,0);
    stop();
  }
  
  else if((roll - set_point_offset) > 30 || (roll - set_point_offset) < -30){
    stop();
  }

  else{
    digitalWrite(BALANCE_LED ,1);
   move_robot(); //update the pwm and direction
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