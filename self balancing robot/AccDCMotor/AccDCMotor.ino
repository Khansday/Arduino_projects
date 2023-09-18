#include<Wire.h>
#include <Servo.h>

Servo myservo;  // create servo object to control a servo

 
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;
 
double x, Ox,Ex=0;
double y, Oy,Ey=0;
double z, Oz,Ez=0;
int err, pwm;

int motorAngle;

#define M1p 5
#define M1n 6

 
void setup(){
Wire.begin();
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
Serial.begin(19200);
//calculate_IMU_error();

myservo.attach(9);  // attaches the servo on pin 9 to the servo object

pinMode(M1p , OUTPUT);
pinMode(M1n , OUTPUT);

digitalWrite(M1p , LOW);
digitalWrite(M1n , LOW);

delay(2000);
}
void loop(){
Wire.beginTransmission(MPU_addr);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true);
AcX=Wire.read()<<8|Wire.read();
AcY=Wire.read()<<8|Wire.read();
AcZ=Wire.read()<<8|Wire.read();
int xAng = map(AcX,minVal,maxVal,-90,90);
int yAng = map(AcY,minVal,maxVal,-90,90);
int zAng = map(AcZ,minVal,maxVal,-90,90);
 
x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

x = 0.2 * x + 0.8 * Ox;
y = 0.2 * y + 0.8 * Oy;
z = 0.2 * z + 0.8 * Oz;

Ox = x;
Oy = y;
Oz = z;

err = (y - 180) *2; 
//set motor speed
pwm = map(abs(err), 0, 180, 0 ,255);
pwm = constrain(pwm, 0 , 255);
if (err > 0){   //if tilted fw/bkw
  //set direction
  analogWrite(M1p , pwm);
  digitalWrite(M1n , LOW);
}
else{
  //set direction
  digitalWrite(M1p , LOW);
  analogWrite(M1n , pwm);
}
Serial.println(pwm);
/*
motorAngle = map(y , 0 , 360 , 0 , 180);
myservo.write(motorAngle);*/


/*x = x - Ex;
y = y -Ey;
z = z - Ez;*/
 
/*Serial.print("AngleX= ");
Serial.println(x);
 
Serial.print("AngleY= ");
Serial.println(y);
 
Serial.print("AngleZ= ");
Serial.println(z);
Serial.println("-----------------------------------------");*/
delay(50);
}

/*
void calculate_IMU_error(){
  int readings = 100;
  for (int i=0; i<readings; i++){
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);
    AcX=Wire.read()<<8|Wire.read();
    AcY=Wire.read()<<8|Wire.read();
    AcZ=Wire.read()<<8|Wire.read();
    int xAng = map(AcX,minVal,maxVal,-90,90);
    int yAng = map(AcY,minVal,maxVal,-90,90);
    int zAng = map(AcZ,minVal,maxVal,-90,90);
    
    x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
    y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
    z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

    Ex = Ex + x;
    Ez = Ez + z;
    Ey = Ey + y;

  }
  Ex = Ex / readings;
  Ez = Ez / readings;
  Ey = Ey / readings;

  Ex = 0.00f - Ex;
  Ey = 0.00f - Ey;
  Ez = 90.0f - Ez;

  Serial.println(Ex);
  Serial.println(Ey);
  Serial.println(Ez);


}*/