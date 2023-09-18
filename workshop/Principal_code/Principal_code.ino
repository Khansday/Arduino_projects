#define pingPin 7 // Trigger Pin of Ultrasonic Sensor
#define echoPin 6 // Echo Pin of Ultrasonic Sensor
long duration, cm;
float threshold_distance = 15; //CHANGEABLE

#define M1_d 2 // pin A
#define M1_dr 3 // pin B
#define M1_speed 9 // PWM pin
int motorSpeed = 85;  //turning speed of principal CHANGEABLE

#define Pwr 0
int pwr_state;

#define buzzer 8

long loopTimer;
int turning_duration = 5000; //CHANGEABLE how often it switches direction
int clockwise = 1;

void setup() {
  // Serial.begin(115200); // Starting Serial Terminal
  // Serial.println("program started");
  // Initialize motor pins as output
  pinMode(M1_d , OUTPUT);
  pinMode(M1_dr , OUTPUT);
  pinMode(M1_speed , OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(Pwr , INPUT_PULLUP);
  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);
  //init the principal movement 
  digitalWrite(M1_d, clockwise);
  digitalWrite(M1_dr, !clockwise);
  analogWrite( M1_speed,motorSpeed);
  noTone(buzzer);
  loopTimer = millis();
}

void loop() {
  stop();
  pwr_state = digitalRead(Pwr);
  while (!pwr_state){
    if (millis() - loopTimer > turning_duration){
    //Serial.println("switching direction");
    switchDirection();
    loopTimer = millis();
  }
    readDistance();
    if (cm < threshold_distance){
      alertMode();
      
    }
    else{
      safeMode();

    }
    pwr_state = digitalRead(Pwr);
    // Serial.print(cm);
    // Serial.print(" cm");
    // Serial.println();
     delay(100);
  }
}

void readDistance(){
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  cm = microsecondsToCentimeters(duration);
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}

void alertMode(){
  tone(buzzer , 1500, 2000);
}

void safeMode(){
  noTone(buzzer);
}

void switchDirection(){
  clockwise = 1 - clockwise;
  digitalWrite(M1_d, clockwise);
  digitalWrite(M1_dr, !clockwise);
}

void stop(){
  digitalWrite(M1_d, LOW);
  digitalWrite(M1_dr, LOW);
}