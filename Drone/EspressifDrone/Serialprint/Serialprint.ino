void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for Serial to initialize
  Serial.println("Serial communication is working!");
}

void loop() {
  Serial.println("Hi from ESP32-S2!");
  delay(1000);
}