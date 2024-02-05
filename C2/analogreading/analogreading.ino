int eyes = A0;        //analog pin of the sensor
int distance;         //variable to store the sensor's reading


void setup() {
  pinMode(eyes, INPUT);
  Serial.begin(115200);
}
void loop() {
  distance = analogRead(eyes);          //read the sensor value and store it
  Serial.println(distance);
  delay(100);
}

