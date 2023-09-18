int finish_line = 15;  //    PUT FINISH LINE DISTANCE HERE


//pins for the motor driver

#define M1_d 1
#define M1_dr 2
#define M1_speed 3
#define M2_d 4
#define M2_dr 5
#define M2_speed 6

// TCS230 or TCS3200 pins wiring to Arduino
#define S0 9
#define S1 10
#define S2 11
#define S3 12
#define sensorOut 13
// Stores frequency read by the photodiodes
int redFrequency = 0;
int freq_treshold = 100;  //change based on colour readings

// pins for pot and button
#define speed_pot A0
#define start_button 0
int motorSpeed;
bool button_state;

//pins for ultrasound sensor
#define trigPin  8
#define echoPin  7
long duration;
int distance;



void setup() {
  // put your setup code here, to run once:
  //set motor driver pin as output
  pinMode(M1_d , OUTPUT);
  pinMode(M1_dr , OUTPUT);
  pinMode(M1_speed , OUTPUT);
  pinMode(M2_d , OUTPUT);
  pinMode(M2_dr , OUTPUT);
  pinMode(M2_speed , OUTPUT);

  // Setting the pins for colour sensor
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  // Setting the sensorOut as an input
  pinMode(sensorOut, INPUT);
  // Setting frequency scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
  // Setting RED (R) filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);

  // Setting ultrasound pins 
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  //settting pot and button
  pinMode(speed_pot, INPUT); 
  pinMode(start_button, INPUT_PULLUP); 

  // Begins serial communication 
  //Serial.begin(115200); //comment if you want code to work

  //delay(5000);   //wait 5 second when powered
  set_speed();
  wait_start();
}

void loop() {
  // put your main code here, to run repeatedly:
  read_red_colour();
  get_distance();
  control_motor();
  delay(100);
  /*goForward();
  delay(1000);
  goBackwards();
  delay(1000);*/

}

void set_speed(){
  motorSpeed = analogRead(speed_pot);   //read the pot
  motorSpeed = map(motorSpeed, 0 , 1023 , 0 , 255); // set the motor speed
  //Serial.print("speed: ");
  //Serial.println(motorSpeed);
}

void wait_start(){
  //Serial.print("Get ready!");
  button_state = digitalRead(start_button);
  while (!button_state) button_state = digitalRead(start_button); //release button to start after initial press
  //Serial.print("Race started!");
}

void control_motor(){
  if (distance > finish_line && redFrequency > freq_treshold){
    goForward();
  }
  else if (distance <= finish_line || redFrequency < freq_treshold){
    stop();
  }

}


void get_distance(){
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH, 50000);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  //Serial.print("Distance: ");
  //Serial.println(distance);
}

void read_red_colour(){  
  // Reading the output frequency
  redFrequency = pulseIn(sensorOut, LOW);
  
  // Printing the RED (R) value
  //Serial.print("R = ");
  //Serial.println(redFrequency);
}



void goForward(){
  //Serial.println("Forward ");
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , HIGH);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , HIGH);
  analogWrite(M2_speed, motorSpeed);
}

void goBackwards(){
  //Serial.println("Backwards ");
  digitalWrite(M1_d , HIGH);
  digitalWrite(M1_dr , LOW);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , HIGH);
  digitalWrite(M2_dr , LOW);
  analogWrite(M2_speed, motorSpeed); 
}
/*
void turnLeft(){
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , LOW);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , HIGH);
  digitalWrite(M2_dr , LOW);
  analogWrite(M2_speed, motorSpeed);
} 

void turnRight(){
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , HIGH);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , LOW);
  analogWrite(M2_speed, motorSpeed); 
}    */

void stop(){
  //Serial.println("STOP ");
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , LOW);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , LOW);
  analogWrite(M2_speed, motorSpeed);
}  
