#define DECODE_NEC
#include <Arduino.h>
#include <IRremote.hpp>

#define IR_RECEIVE_PIN      2 // To be compatible with interrupt example, pin 2 is chosen here.

/*
 * Helper macro for getting a macro definition as string
 */
#if !defined(STR_HELPER)
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#endif


#include <Wire.h>
long loopTimer, loopTimer2;
int temperature;
double accelRoll;
long acc_x, acc_y, acc_z;
double accel_x, accel_y, accel_z;
double gyroRoll;
int gyro_x, gyro_y, gyro_z;
//long gyro_x_cal, gyro_y_cal, gyro_z_cal, acc_x_cal, acc_y_cal, acc_z_cal;  //dont need this after hard coding the offset
long gyro_x_cal = 181, gyro_y_cal = -655, gyro_z_cal = -39 , acc_x_cal, acc_y_cal, acc_z_cal;
double rotation_x, rotation_y, rotation_z;
double freq, dt;
double tau = 0.98;
double roll = 0;

// 250 deg/s --> 131.0, 500 deg/s --> 65.5, 1000 deg/s --> 32.8, 2000 deg/s --> 16.4
long scaleFactorGyro = 65.5;

// 2g --> 16384 , 4g --> 8192 , 8g --> 4096, 16g --> 2048
long scaleFactorAccel = 8192;
//pins for the motor driver

#define M1_d 7
#define M1_dr 8
#define M1_speed 9
#define M2_d 5
#define M2_dr 6
#define M2_speed 10
#define min_speed 35

#define P_pot A0           //pot to set the Kp value 0 to 30
#define D_pot A1           //pot to set the Kd value 0 to 20
#define set_point_offset_pot A2   // use this pot to change the centre set point
#define I_pot A3           //pot to set the Ki value 0 to 5

#define BALANCE_LED 12    //LED that lights up when robot is balanced
//Bunch of variables
int pwm;
float err, Kp = 22.50, Kd = 5, Ki = 5; //this sets the sensitivity of the motors in regards to tilt. Tune it with trial and error
float errd;  //variable to store derivative of error
double Ox=0;  //store previous reading for next cycle
float integral = 0; //store the cumulative error
float set_point_offset = -2.0;  //allowing set point calibration



int mode;




void setup(){
  Wire.begin();

  setup_mpu_6050_registers();

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

  //set teh LED as output
  pinMode(BALANCE_LED, OUTPUT); 

  Serial.begin(115200);
  // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  // Just to know which program is running on my Arduino
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));
  Serial.print(F("Ready to receive IR signals of protocols: "));
  printActiveIRProtocols(&Serial);
  Serial.println(F("at pin " STR(IR_RECEIVE_PIN)));
  loopTimer = micros();
  loopTimer2 = micros();
}


void loop(){
freq = 1/((micros() - loopTimer2) * 1e-6);
  loopTimer2 = micros();
  dt = 1/freq;

  // Read the raw acc data from MPU-6050
  read_mpu_6050_data();

  // Subtract the offset calibration value
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  rotation_y = (double)gyro_y / (double)scaleFactorGyro;

  // Convert to g force
  accel_x = (double)acc_x / (double)scaleFactorAccel;
  //accel_y = (double)acc_y / (double)scaleFactorAccel;
  accel_z = (double)acc_z / (double)scaleFactorAccel;

  // Complementary filter
  accelRoll = atan2(accel_x, accel_z) * RAD_TO_DEG;
  roll =  (tau)*(roll - rotation_y*dt) + (1-tau)*(accelRoll);
  balance_LED();  
/*
  // Data out serial monitor
  // Serial.println(freq,0);  // Serial.print(",");
  // Serial.println(roll - set_point_offset,1);   //Serial.print(", \t");
   // Serial.print(" Kp = ");    Serial.println(Kp);
  //  Serial.print(" Ki = ");    Serial.print(Ki);
  //  Serial.print(" Kd = ");    Serial.println(Kd);     
*/
  if (IrReceiver.decode()) {
      mode = IrReceiver.decodedIRData.command;
      IrReceiver.resume(); // Enable receiving of the next value
    }

  switch(mode){
    case(0x18):   //arrow up
      set_point_offset = -2.0 + 0.5;  //allowing set point calibration

    break;
    case(0x52):   //arrow down
      float set_point_offset = -2.0 - 0.5;  //allowing set point calibration
    break;
    case(0x8):   //arrow left
      set_point_offset = -2.0;  //allowing set point calibration
    break;
    case(0x5A):   //arrow right
      set_point_offset = -2.0;  //allowing set point calibration
    break;
  }
  // Wait until the loopTimer reaches 4000us (250Hz) before next loop
  while (micros() - loopTimer <= 4000){
    
   
  }
  loopTimer = micros();

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

void balance_LED(){
  if ((roll - set_point_offset) > -0.3 && (roll - set_point_offset) < 0.3){ //if almost balanced with 0.3 degrees of tollerance
    digitalWrite(BALANCE_LED ,1);
    stop();
  }
  
  else if((roll - set_point_offset) > 30 || (roll - set_point_offset) < -30){ //if tipped over
    stop();
    integral = 0;
  }
  else{
    digitalWrite(BALANCE_LED ,0);
    // moved here because I dont wanna update motors if no need
    balance_robot();      //update the pwm and direction
  }
}



void balance_robot(){
  err = float(roll) - set_point_offset;   //error from setpoint
  errd = err - Ox;                        //error for Kd
  Ox = err;                               //preparing for next cycle
  integral = integral + (Ki * err) ;
  integral = constrain(integral, -2, 2);
  pwm = err * Kp + errd * Kd + integral;  //value with PD loop
  

  if (pwm > 0){   //if tilted fw/bkw
    pwm = constrain(pwm, min_speed , 255);  //make sure value fits input parameter
    digitalWrite(M1_d , HIGH);
    digitalWrite(M1_dr , LOW);
    digitalWrite(M2_d , HIGH);
    digitalWrite(M2_dr , LOW);
    analogWrite(M1_speed , pwm );  //set motor speed add a bit because this motor is weaker
    analogWrite(M2_speed , pwm);  

  }
  else{
    pwm = constrain(pwm,-255, -min_speed  );  //make sure value fits input parameter
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

