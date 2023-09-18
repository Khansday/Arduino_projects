
int LED_TIMER = 2000;
int DETECTION_RANGE = 9;




int LED1 = 10;
int buzzer = 3;

int trigPin = 9;    // TRIG pin
int echoPin = 8;    // ECHO pin

float duration_us, distance_cm;

int tick = 0;
int temp = 0;
void setup() {
  // put your setup code here, to run once:
    // begin serial port
  Serial.begin (9600);

  // configure the trigger pin to output mode
  pinMode(trigPin, OUTPUT);
  // configure the echo pin to input mode
  pinMode(echoPin, INPUT);
  tick = millis();
  pinMode(LED1, OUTPUT);
  pinMode(buzzer, OUTPUT);

}

void loop() {
  temp = millis();
  // put your main code here, to run repeatedly:
  if (temp > tick +LED_TIMER){
    digitalWrite(LED1,!digitalRead(LED1));
    tick = millis();
  }

   // generate 10-microsecond pulse to TRIG pin
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // measure duration of pulse from ECHO pin
  duration_us = pulseIn(echoPin, HIGH);

  // calculate the distance
  distance_cm = 0.017 * duration_us;

//   print the value to Serial Monitor
  Serial.print("distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");
  if (distance_cm < DETECTION_RANGE){
    tone(buzzer, 3000); 
  }
  else{
    noTone(buzzer);
  }

}
