int eyes = A0;        //analog pin of the sensor
int distance;         //variable to store the sensor's reading
int LED = 5;          //pin for the detection LED
int threshold = 300;  //storing the detection value

void setup() {
  pinMode(eyes, INPUT);
  pinMode(LED, INPUT);
}
void loop() {
  distance = analogRead(eyes);          //read the sensor value and store it
  if (distance > detection_threshold){  //if the obstacle is too close
    digitalWrite(LED, HIGH);            //switch ON the LED
  }
  else{                                 //otherwise
    digitalWrite(LED, LOW);             //switch OFF the LED
  }
  delay(100);
}

