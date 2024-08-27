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

  pinMode(M1_d , OUTPUT);
  pinMode(M1_dr , OUTPUT);
  pinMode(M1_speed , OUTPUT);
  pinMode(M2_d , OUTPUT);
  pinMode(M2_dr , OUTPUT);
  pinMode(M2_speed , OUTPUT);

  pinMode(BALANCE_LED, OUTPUT);
  pwm = 50;
  forward();
}
void loop() {
  forward();
  delay(1000);
  backward();
  delay(1000);
}

void forward(){
    digitalWrite(M1_d , HIGH);
    digitalWrite(M1_dr , LOW);
    digitalWrite(M2_d , HIGH);
    digitalWrite(M2_dr , LOW);
    analogWrite(M1_speed , pwm );  
    analogWrite(M2_speed , pwm);
  }
  void backward() {
    digitalWrite(M1_d , LOW);
    digitalWrite(M1_dr , HIGH);
    digitalWrite(M2_d , LOW);
    digitalWrite(M2_dr , HIGH);
    analogWrite(M1_speed , pwm );  
    analogWrite(M2_speed , pwm);
  }


void stop(){
  digitalWrite(M1_d , LOW);
    digitalWrite(M1_dr , LOW);
    digitalWrite(M2_d , LOW);
    digitalWrite(M2_dr , LOW);
}