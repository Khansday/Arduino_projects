#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


Adafruit_MPU6050 mpu;

//Bunch of variables
int pwm;
float err, Kp = 89, Kd = 0; //this sets the sensitivity of the motors in regards to tilt. Tune it with trial and error
bool fwd = false;  //decide to drive motor fwd or bkwd
float errd;  //variable to store derivative of error
float fltr; //use this to smooth out rough data
double Ox=0;  //store previous reading for next cycle

//pins for the motor driver

#define M1_d 2
#define M1_dr 3
#define M1_speed 9
#define M2_d 4
#define M2_dr 5
#define M2_speed 10

 
void setup(){
  Serial.begin(115200);

	// Try to initialize!
	if (!mpu.begin()) {
		Serial.println("Failed to find MPU6050 chip");
		while (1) {
		  delay(10);
		}
	}

	// set accelerometer range to +-8G
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

	// set gyro range to +- 500 deg/s
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);

	// set filter bandwidth to 21 Hz
	mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

	delay(100);


//set motor driver pin as output
pinMode(M1_d , OUTPUT);
pinMode(M1_dr , OUTPUT);
pinMode(M1_speed , OUTPUT);
pinMode(M2_d , OUTPUT);
pinMode(M2_dr , OUTPUT);
pinMode(M2_speed , OUTPUT);

// make surre the motors are OFF

digitalWrite(M1_d , LOW);
digitalWrite(M1_dr , LOW);
digitalWrite(M1_speed , LOW);
digitalWrite(M2_d , LOW);
digitalWrite(M2_dr , LOW);
digitalWrite(M2_speed , LOW);

delay(2000);
}
void loop(){

	/* Get new sensor events with the readings */
	sensors_event_t a, g, temp;
	mpu.getEvent(&a, &g, &temp);

  //a.acceleration.x  = (1-filtr) * a.acceleration.x + filtr * Ox;  //filterring

  errd = a.acceleration.x - Ox;   //difference from previous reading 
  Ox = a.acceleration.x ; //get ready for next cycle

//Serial.println(a.acceleration.x);  //debug line

//Calculate how many degrees are we far from the centre
err = float(a.acceleration.x); 

if (err > 0){   //if tilted fw/bkw
  //set direction
  fwd = true;

}
else{
  //set direction
  fwd = false;

}
err = abs(err); //dont want to feed a negative pwm value
//Serial.println(err);  //debug line

//set motor speed
pwm = err * Kp + errd * Kd;  //value with PD loop
pwm = constrain(pwm, 0 , 255);  //make sure value fits input parameter

analogWrite(M1_speed , pwm);  //set motor speed 
analogWrite(M2_speed , pwm);  //set motor speed 

if (fwd){   //if tilted fw/bkw
  //set direction
  digitalWrite(M1_d , HIGH);
  digitalWrite(M1_dr , LOW);

  digitalWrite(M2_d , HIGH);
  digitalWrite(M2_dr , LOW);
}
else{
  //set direction
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , HIGH);

  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , HIGH);
}
//Serial.println(pwm); //debug line

//delay(10);
}

