#include <math.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000); // highest speed possible on my setup, higher than this I get errors
  float a = -0.02;
  Serial.println();
  float Kp = 1.00;

  for(float i = 0; i<=5; i+= 0.01){
    int signOfX = (i > 0) - (i < 0);
    double r = Kp * (sqrt(fabs(i))) * signOfX;
    Serial.print(i);
    Serial.print(" ");
    Serial.println(r);
  }
 

}

void loop() {
  // put your main code here, to run repeatedly:

}
