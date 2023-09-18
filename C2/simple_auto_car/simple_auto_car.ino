int M1_d = 1;            //pin definitions for the motor driver
int M1_dr = 2;
int M1_speed = 5;
int M2_d = 3;
int M2_dr = 4;
int M2_speed = 6;
int motorSpeed = 130;    //storing the speed of the robot

int distanceSensor = A0; //distance sensor pin 
int distanceValue;       //variable to store the distance value
int maxDistance = 200;   //threshold distance for the obstacle

void goForward(){                   //function to make the robot go forward
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , HIGH);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , HIGH);
  analogWrite(M2_speed, motorSpeed);
}

void turnLeft(){                    //function to make the robot turn left
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , LOW);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , HIGH);
  digitalWrite(M2_dr , LOW);
  analogWrite(M2_speed, motorSpeed);
} 

void setup() {
pinMode(M1_d , OUTPUT);           //setting pins connected to components
pinMode(M1_dr , OUTPUT);
pinMode(M1_speed , OUTPUT);
pinMode(M2_d , OUTPUT);
pinMode(M2_dr , OUTPUT);
pinMode(M2_speed , OUTPUT);
pinMode(distanceSensor, INPUT);
}

void loop() {
  distanceValue = analogRead(distanceSensor);  //read the sensor 
  if (distanceValue >= maxDistance){           //check if the obstacle is too close
    turnLeft();                                //turn left
    delay(500);                                //duration for how long it turns
  }
  else{                                        //otherwise
    goForward();                               //go forward
  }
}



