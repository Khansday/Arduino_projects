int line_sensor = A0;
int line_value;
int detecting_threshold = 100;

void setup() {
  // put your setup code here, to run once:
  pinMode(line_sensor, INPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  line_value = analogRead(line_sensor);
  if (line_value < detecting_threshold){
  Serial.println("WHERE AM I ;(");
  }
  else {
  Serial.println("I AM ON THE LINE :P");
  }
  delay(100);
}


