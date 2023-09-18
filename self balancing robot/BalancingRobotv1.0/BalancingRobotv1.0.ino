#include<Wire.h>

const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;
 
double x, Ox,Ex=0;
double y, Oy,Ey=0;
double z, Oz,Ez=0;

int err, pwm;
float Kp = 1.5; //this sets the sensitivity of the motors in regards to tilt. Tune it with trial and error
bool fwd = false;

#define M1p 5
#define M1n 6
#define M2p 10
#define M2n 11

 
void setup(){
Wire.begin();
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
Serial.begin(19200);


pinMode(M1p , OUTPUT);
pinMode(M1n , OUTPUT);
pinMode(M2p , OUTPUT);
pinMode(M2n , OUTPUT);

digitalWrite(M1p , LOW);
digitalWrite(M1n , LOW);
digitalWrite(M2p , LOW);
digitalWrite(M2n , LOW);

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

//filtering the data to avoid sudden movement and instability
x = 0.2 * x + 0.8 * Ox;
y = 0.2 * y + 0.8 * Oy;
z = 0.2 * z + 0.8 * Oz;
//setting up the filter for the net reading cycle
Ox = x;
Oy = y;
Oz = z;
Serial.println(y);
//Calculate how many degrees are we far fomr the centre
err = (180 - y); 
if (err > 0){   //if tilted fw/bkw
  //set direction
  fwd = true;
  //bkwd = false;
}
else{
  //set direction
  fwd = false;
  //bkwd = true;
}
err = 180 - abs(err); 
//Serial.println(err);
//set motor speed
pwm = err * Kp;
pwm = constrain(pwm, 0 , 255);
if (fwd){   //if tilted fw/bkw
  //set direction
  analogWrite(M1p , pwm);
  digitalWrite(M1n , LOW);
  analogWrite(M2p , pwm);
  digitalWrite(M2n , LOW);
}
else{
  //set direction
  digitalWrite(M1p , LOW);
  analogWrite(M1n , pwm);
  digitalWrite(M2p , LOW);
  analogWrite(M2n , pwm);
}
//Serial.println(pwm);

delay(10);
}

