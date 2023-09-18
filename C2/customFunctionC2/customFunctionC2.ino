int M1_p = 1;  //pins to control motor 1
int M1_n = 2;
int M1_s = 3;
int M2_p = 4;  //pins to control motor 2
int M2_n = 5;
int M2_s = 6;

void go_Forward(){
  Serial.println("Car forward");  //show this message on the screen
  digitalWrite(M1_p , HIGH);      //motor 1 fwd
  digitalWrite(M1_n , LOW);
  analogWrite(M1_s , 80);     

  digitalWrite(M2_p , HIGH);      //mnotor 2 fwd
  digitalWrite(M2_n , LOW);
  analogWrite(M2_s , 80); 
}

void setup() {
  pinMode(M1_p , OUTPUT);  //setting the pins going to the motor driver as output
  pinMode(M1_n , OUTPUT);
  pinMode(M1_s , OUTPUT);
  pinMode(M2_p , OUTPUT);
  pinMode(M2_n , OUTPUT);
  pinMode(M2_s , OUTPUT);

  Serial.begin(9600);     //start the serial monitor

  go_Forward();           //call function to move forward
}

void loop() {
         
}    




  void go_Backward(){
  Serial.println("Car backward");
  digitalWrite(M1_p , LOW); //motor1 bkwd
  digitalWrite(M1_n , HIGH);
  analogWrite(M1_s , 80);  //low speed

  digitalWrite(M2_p , LOW); //motor 2 bkwd
  digitalWrite(M2_n , HIGH);
  analogWrite(M2_s , 80); //low speed
}

void go_Left(){
  Serial.println("Car left");
  digitalWrite(M1_p , LOW); //motor1 stop
  digitalWrite(M1_n , LOW);
  analogWrite(M1_s , 80);  //low speed

  digitalWrite(M2_p , HIGH); //mnotor 2 fwd
  digitalWrite(M2_n , LOW);
  analogWrite(M2_s , 80); //low speed
}

void go_Right(){
  Serial.println("Car right");
  digitalWrite(M1_p , HIGH); //motor1 fwd
  digitalWrite(M1_n , LOW);
  analogWrite(M1_s , 80);  //low speed

  digitalWrite(M2_p , LOW); //mnotor 2 stop
  digitalWrite(M2_n , LOW);
  analogWrite(M2_s , 80); //low speed
}

void stop(){
  Serial.println("Car stop");
  digitalWrite(M1_p , LOW); //motor1 fwd
  digitalWrite(M1_n , LOW);
  analogWrite(M1_s , 80);  //low speed

  digitalWrite(M2_p , LOW); //mnotor 2 stop
  digitalWrite(M2_n , LOW);
  analogWrite(M2_s , 80); //low speed
}   
