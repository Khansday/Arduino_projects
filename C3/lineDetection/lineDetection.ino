int line_sensor = A0;
int line_value;
int detecting_threshold = 100;

void setup() {
  pinMode(line_sensor, INPUT);
  Serial.begin(115200);
}

void loop() {
  line_value = analogRead(line_sensor);  //READ THE SIGNAL WIRE
  if (line_value < detecting_threshold){ //COMPARE THE VALUE READ
  Serial.println("NOT ON THE LINE");
  }
  else {
  Serial.println("ON THE LINE :P");
  }
  delay(100);
}


