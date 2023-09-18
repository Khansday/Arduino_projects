//Include I2C library and declare variables
#include <Wire.h>

long duration, loopTimer, loopTimer2;
int temperature;
double accelRoll;
long acc_x, acc_y, acc_z;
double accel_x, accel_y, accel_z;
double gyroRoll;
int gyro_x, gyro_y, gyro_z;
//long gyro_x_cal, gyro_y_cal, gyro_z_cal, acc_x_cal, acc_y_cal, acc_z_cal;  //dont need this after hard coding the offset
long gyro_x_cal = 18, gyro_y_cal = 5, gyro_z_cal = -32 , acc_x_cal, acc_y_cal, acc_z_cal;
double rotation_x, rotation_y, rotation_z;
double freq, dt;
double tau = 0.98;
double roll = 0;

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
#define min_speed 40
#define max_speed 120


//Bunch of variables
int pwm;
float err, Kp = 40.0, Kd = 20, Ki = 5; //this sets the sensitivity of the motors in regards to tilt. Tune it with trial and error
float errd;  //variable to store derivative of error
double Ox=0;  //store previous reading for next cycle
float integral = 0; //store the cumulative error
float set_point_offset =-6.7;  //allowing set point calibration
int distance0, distance1; 
float movement_offset = 1.2; //variable to move th erobot by changing the roll angle

#define IR_0 A0           //pot to set the Kp value 0 to 30
#define IR_1 A1           //pot to set the Kd value 0 to 20

#define BALANCE_LED 12    //LED that lights up when robot is balanced

#define ECHO 7
#define TRIGGER 8

void setup() {
  // Start
  Wire.begin();
  Serial.begin(115200);

  // Setup the registers of the MPU-6050 and start up
  setup_mpu_6050_registers();

  //set motor driver pin as output
  pinMode(M1_d , OUTPUT);
  pinMode(M1_dr , OUTPUT);
  pinMode(M1_speed , OUTPUT);
  pinMode(M2_d , OUTPUT);
  pinMode(M2_dr , OUTPUT);
  pinMode(M2_speed , OUTPUT);
  pinMode(IR_0 , INPUT);
  pinMode(IR_1 , INPUT);
  pinMode(ECHO, INPUT); // Sets the trigPin as an Output
  pinMode(TRIGGER, OUTPUT); // Sets the echoPin as an Input

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
  //rotation_x = (double)gyro_x / (double)scaleFactorGyro;
  rotation_y = (double)gyro_y / (double)scaleFactorGyro;
  //rotation_z = (double)gyro_z / (double)scaleFactorGyro;

  // Convert to g force
  accel_x = (double)acc_x / (double)scaleFactorAccel;
  //accel_y = (double)acc_y / (double)scaleFactorAccel;
  accel_z = (double)acc_z / (double)scaleFactorAccel;

  // Complementary filter
  accelRoll = atan2(accel_x, accel_z) * RAD_TO_DEG;
  roll =  (tau)*(roll - rotation_y*dt) + (1-tau)*(accelRoll);
  // Data out serial monitor
  // Serial.print(freq,0);   Serial.print(",");
  // Serial.println(roll - set_point_offset,1);   //Serial.print(", \t");
    //Serial.print(" Kp = ");    Serial.println(Kp);
  //  Serial.print(" Ki = ");    Serial.print(Ki);
  //  Serial.print(" Kd = ");    Serial.println(Kd);   
  Serial.println(distance0);  

  
  //set_PID_constants();   //reading pots to calibrate PD values
  //calibrate_set_point_offset(); //read pot to adjust centre point
  //read_IR();
  balance_LED();         //light up an LED if robot is balanced
  
  // Wait until the loopTimer reaches 4000us (250Hz) before next loop
  while (micros() - loopTimer <= 4000);
  loopTimer = micros();

}

void read_ultra(){  //not doing it becuz is slow
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);
  duration = pulseIn(ECHO, HIGH,1000);
  distance0 = duration * 0.034 / 2;
}

void read_IR(){
  distance0 = analogRead(IR_0);
  distance1= analogRead(IR_1);
  if (distance0 < 400 && distance1 >550){
    roll -=   movement_offset;
  }
  else if (distance1 < 400 && distance0 >550){
    roll +=  movement_offset;   
  }
}

/*void calibrate_set_point_offset(){
  set_point_offset = analogRead(set_point_offset_pot);
  set_point_offset = (float(set_point_offset / 1023) * 10) - 5 ; //the value goes from +- 5 degrees
}

void set_PID_constants(){
  Kp = analogRead(P_pot); //read Kp pot
  Kd = analogRead(D_pot); //read Kd pot
  Ki = analogRead(I_pot); //read Ki pot
  Kp = (float(Kp / 1023) * 10) + 15;  //map the values
  Kd = float(Kd / 1023) * 20;
  Ki = float(Ki / 1023) * 5;
}*/

void balance_LED(){
  if ((roll - set_point_offset) > 89.7 && (roll - set_point_offset) < 90.3){ //if almost balanced with 0.3 degrees of tollerance
    digitalWrite(BALANCE_LED ,1);
    stop();
  }
  else if((roll - set_point_offset) > 105 || (roll - set_point_offset) < 75){ //if tipped over
    stop();
    integral = 0;
  }
  else{
    digitalWrite(BALANCE_LED ,0);
    // moved here because I dont wanna update motors if no need
    balance_robot();      //update the pwm and direction
  }
}

void read_mpu_6050_data() {
  // Subroutine for reading the raw data
  Wire.beginTransmission(0x68);  //add of mpu
  Wire.write(0x3B);               //ask to read at 3b
  Wire.endTransmission();       // finishing the setup
  Wire.requestFrom(0x68, 14);   //ask for 14 bytes at x68 addrs

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
  Wire.write(0x6B); //power mode regiister
  Wire.write(0x00); //awake state
  Wire.endTransmission();
  // Configure the accelerometer
  // Wire.write(0x__);
  // Wire.write; 2g --> 0x00, 4g --> 0x08, 8g --> 0x10, 16g --> 0x18
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); //acc configs
  Wire.write(0x08);
  Wire.endTransmission();
  // Configure the gyro
  // Wire.write(0x__);
  // 250 deg/s --> 0x00, 500 deg/s --> 0x08, 1000 deg/s --> 0x10, 2000 deg/s --> 0x18
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); //gyro configs
  Wire.write(0x08);
  Wire.endTransmission();
}

void balance_robot(){
  err = 90 - float((roll) - set_point_offset);   //error from setpoint
  distance0 = analogRead(IR_0);
  distance1= analogRead(IR_1);
  if (distance0 < 400 && distance1 >550){
    err -=  movement_offset;
  }
  else if (distance1 < 400 && distance0 >550){
    err +=  movement_offset;   
  }
   errd = err - Ox;                        //error for Kd
   Ox = err;                               //preparing for next cycle
   integral = integral + (Ki * err) ;
   integral = constrain(integral, -3, 3);
  pwm = err * Kp + errd * Kd + integral;  //value with PD loop
  //Serial.println(err);

  if (err > 0){   //if tilted fw/bkw
    pwm = constrain(pwm, min_speed , max_speed);  //make sure value fits input parameter
    digitalWrite(M1_d , HIGH);
    digitalWrite(M1_dr , LOW);
    digitalWrite(M2_d , HIGH);
    digitalWrite(M2_dr , LOW);
    analogWrite(M1_speed , pwm );  //set motor speed add a bit because this motor is weaker
    analogWrite(M2_speed , pwm);  

  }
  else{
    pwm = constrain(pwm,-max_speed, -min_speed  );  //make sure value fits input parameter
    digitalWrite(M1_d , LOW);
    digitalWrite(M1_dr , HIGH);
    digitalWrite(M2_d , LOW);
    digitalWrite(M2_dr , HIGH);
    analogWrite(M1_speed , -pwm );  //set motor speed add a bit because this motor is weaker
    analogWrite(M2_speed , -pwm);  
  }


}

void stop(){
  digitalWrite(M1_d , LOW);
    digitalWrite(M1_dr , LOW);
    digitalWrite(M2_d , LOW);
    digitalWrite(M2_dr , LOW);
}
