//Include I2C library and declare variables
#include <Wire.h>
//#include <MPU6050.h>

long loopTimer, loopTimer2;
int temperature;
double accelPitch;
double accelRoll;
long acc_x, acc_y, acc_z;
double accel_x, accel_y, accel_z;
double gyroRoll, gyroPitch, gyroYaw;
int gyro_x, gyro_y, gyro_z;
//long gyro_x_cal, gyro_y_cal, gyro_z_cal, acc_x_cal, acc_y_cal, acc_z_cal;  //dont need this after hard coding the offset
long gyro_x_cal = 181, gyro_y_cal = -655, gyro_z_cal = -39 , acc_x_cal, acc_y_cal, acc_z_cal;
double rotation_x, rotation_y, rotation_z;
double freq, dt;
double tau = 0.98;
double roll = 0;
double pitch = 0;

// 250 deg/s --> 131.0, 500 deg/s --> 65.5, 1000 deg/s --> 32.8, 2000 deg/s --> 16.4
long scaleFactorGyro = 65.5;

// 2g --> 16384 , 4g --> 8192 , 8g --> 4096, 16g --> 2048
long scaleFactorAccel = 8192;


//pins for the motor driver

#define M1_d 2
#define M1_dr 3
#define M1_speed 9
#define M2_d 4
#define M2_dr 5
#define M2_speed 10
#define min_speed 35


//Bunch of variables
int pwm;
float err, Kp = 14.0, Kd = 0, Ki = 0; //this sets the sensitivity of the motors in regards to tilt. Tune it with trial and error
bool fwd = false;  //decide to drive motor fwd or bkwd
float errd;  //variable to store derivative of error
float fltr = 0.1; //use this to smooth out rough data
double Ox=0;  //store previous reading for next cycle
float integral = 0; //store the cumulative error
float set_point_offset;  //allowing set point calibration

#define P_pot A0           //pot to set the Kp value 0 to 30
#define D_pot A1           //pot to set the Kd value 0 to 20
#define set_point_offset_pot A2   // use this pot to change the centre set point
#define I_pot A3           //pot to set the Ki value 0 to 5

#define BALANCE_LED 12    //LED that lights up when robot is balanced

void setup() {
  // Start
  Wire.begin();
  Serial.begin(115200);

  // Setup the registers of the MPU-6050 and start up
  setup_mpu_6050_registers();

  //calibrate_sensor();   //dont need this anymore

  // Display headers
  /*Serial.print("\nNote 1: Yaw is not filtered and will drift!\n");
  Serial.print("\nNote 2: Make sure sampling frequency is ~250 Hz\n");
  Serial.print("Sampling Frequency (Hz)\t\t");
  Serial.print("Roll (deg)\t\t");
  Serial.print("Pitch (deg)\t\t");
  Serial.print("Yaw (deg)\t\t\n");*/

  //set motor driver pin as output
  pinMode(M1_d , OUTPUT);
  pinMode(M1_dr , OUTPUT);
  pinMode(M1_speed , OUTPUT);
  pinMode(M2_d , OUTPUT);
  pinMode(M2_dr , OUTPUT);
  pinMode(M2_speed , OUTPUT);
  pinMode(P_pot , INPUT);
  pinMode(D_pot , INPUT);
  pinMode(I_pot , INPUT);
  pinMode(set_point_offset_pot , INPUT);

  // make surre the motors are OFF when powered up
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , LOW);
  digitalWrite(M1_speed , LOW);
  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , LOW);
  digitalWrite(M2_speed , LOW);

  //set teh LED as output
  pinMode(BALANCE_LED, OUTPUT); 

  // Reset the loop timer
  loopTimer = micros();
  loopTimer2 = micros();
}


void loop() {
  freq = 1/((micros() - loopTimer2) * 1e-6);
  loopTimer2 = micros();
  dt = 1/freq;

  // Read the raw acc data from MPU-6050
  read_mpu_6050_data();

  // Subtract the offset calibration value
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  // Convert to instantaneous degrees per second
  rotation_x = (double)gyro_x / (double)scaleFactorGyro;
  rotation_y = (double)gyro_y / (double)scaleFactorGyro;
  rotation_z = (double)gyro_z / (double)scaleFactorGyro;

  // Convert to g force
  accel_x = (double)acc_x / (double)scaleFactorAccel;
  accel_y = (double)acc_y / (double)scaleFactorAccel;
  accel_z = (double)acc_z / (double)scaleFactorAccel;

  // Complementary filter
  accelPitch = atan2(accel_y, accel_z) * RAD_TO_DEG;
  accelRoll = atan2(accel_x, accel_z) * RAD_TO_DEG;

  pitch = (tau)*(pitch + rotation_x*dt) + (1-tau)*(accelPitch);
  roll =  (tau)*(roll - rotation_y*dt) + (1-tau)*(accelRoll);

  /*gyroPitch += rotation_x*dt;
  gyroRoll -= rotation_y*dt;
  gyroYaw += rotation_z*dt;*/

  // Visualize just the roll
  // Serial.print(roll); Serial.print(",");
  // Serial.print(gyroRoll); Serial.print(",");
  // Serial.println(accelRoll);

  // Visualize just the pitch
  // Serial.print(pitch); Serial.print(",");
  // Serial.print(gyroPitch); Serial.print(",");
  // Serial.println(accelPitch);

  // Data out serial monitor
   Serial.print(freq,0);   Serial.print(",");
   //Serial.println(roll - set_point_offset,1);   //Serial.print(", \t");
    Serial.print(" Kp = ");    Serial.println(Kp);
  //  Serial.print(" Ki = ");    Serial.print(Ki);
  //  Serial.print(" Kd = ");    Serial.println(Kd);     

  
  set_PD_constants();   //reading pots to calibrate PD values
  calibrate_set_point_offset(); //read pot to adjust centre point
  balance_LED();         //light up an LED if robot is balanced
  // Wait until the loopTimer reaches 4000us (250Hz) before next loop
  while (micros() - loopTimer <= 4000);
  loopTimer = micros();

}

void calibrate_set_point_offset(){
  set_point_offset = analogRead(set_point_offset_pot);
  set_point_offset = (float(set_point_offset / 1023) * 10) - 5 ; //the value goes from +- 5 degrees

}

void set_PD_constants(){
  Kp = analogRead(P_pot); //read Kp pot
  Kd = analogRead(D_pot); //read Kd pot
  Ki = analogRead(I_pot); //read Ki pot
  Kp = ( float(Kp / 1023) * 5) + 14;  //map the values
  Kd = float(Kd / 1023) * 20;
  Ki = float(Ki / 1023) * 5;
}

void balance_LED(){
  if ((roll - set_point_offset) > -0.3 && (roll - set_point_offset) < 0.3){ //if almost balanced with 0.3 degrees of tollerance
    digitalWrite(BALANCE_LED ,1);
    stop();
  }
  else{
    digitalWrite(BALANCE_LED ,0);
    // moved here because I dont wanna update motors if no need
    balance_robot();      //update the pwm and direction
  }
}

void read_mpu_6050_data() {
  // Subroutine for reading the raw data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);

  // Read data --> Temperature falls between acc and gyro registers
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() <<8 | Wire.read();
  gyro_x = Wire.read()<<8 | Wire.read();
  gyro_y = Wire.read()<<8 | Wire.read();
  gyro_z = Wire.read()<<8 | Wire.read();
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
  for (int cal_int = 0; cal_int < readings_num; cal_int ++){
    if(cal_int % 200 == 0)Serial.print(".");  //print . every 200 cycles
    read_mpu_6050_data();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;

    acc_x_cal += acc_x;
    acc_y_cal += acc_y;
    acc_z_cal += acc_z;
    delay(3);
  }
  // Average the values
  gyro_x_cal /= readings_num;
  gyro_y_cal /= readings_num;
  gyro_z_cal /= readings_num;

  acc_x_cal /= readings_num;
  acc_y_cal /= readings_num;
  acc_z_cal /= readings_num;  
}


void balance_robot(){
  err = float(roll) - set_point_offset;   //error from setpoint
  errd = err - Ox;                        //error for Kd
  Ox = err;                               //preparing for next cycle
  integral = integral + (Ki * err) ;
  integral = constrain(integral, -5, 5);

  if (err > 0){   //if tilted fw/bkw
    fwd = true;
  }
  else{
    fwd = false;
  }
  err = abs(err); //dont want to feed a negative pwm value
  //Serial.println(err);  //debug line
  //Serial.println(set_point_offset);  //debug line
  //set motor speed
  pwm = err * Kp + errd * Kd + integral;  //value with PD loop
  pwm = constrain(pwm, min_speed , 248);  //make sure value fits input parameter

  analogWrite(M1_speed , pwm );  //set motor speed add a bit because this motor is weaker
  analogWrite(M2_speed , pwm);  //

  if (fwd){   //if tilted fw/bkw
    digitalWrite(M1_d , HIGH);
    digitalWrite(M1_dr , LOW);
    digitalWrite(M2_d , HIGH);
    digitalWrite(M2_dr , LOW);
  }
  else{
    digitalWrite(M1_d , LOW);
    digitalWrite(M1_dr , HIGH);
    digitalWrite(M2_d , LOW);
    digitalWrite(M2_dr , HIGH);
  }
}

void stop(){
  digitalWrite(M1_d , LOW);
    digitalWrite(M1_dr , LOW);
    digitalWrite(M2_d , LOW);
    digitalWrite(M2_dr , LOW);
}
