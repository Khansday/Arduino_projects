int buzzer = 8; 

void setup() {
  // put your setup code here, to run once:
  pinMode(buzzer, OUTPUT);
  tone(buzzer,32 - 65500, 2000);  // 500 -- 10 000
  digitalWrite(LED , HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:

}
