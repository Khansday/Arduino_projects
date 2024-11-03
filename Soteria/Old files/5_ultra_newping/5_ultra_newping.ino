#define NUM_SENSORS 5

const int triggerPin = 2; // Shared trigger pin for all sensors
const int UltraPins[NUM_SENSORS] = {1, 2, 3, 4, 5}; // Echo pins for each sensor
const int maxDistance = 58.21 * 100; // Maximum distance in centimeters before timeout

unsigned long echoTime;
bool sensorReady = false;
int currentSensor = 0;
int distance = 0;
unsigned long previousMillis = 0;
const long interval = 50 ; // Interval to read one sensor

void setup() {
  Serial.begin(250000);

}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    readNextSensor();
    previousMillis = currentMillis;
  }
}

void readNextSensor() {
  if (currentSensor < NUM_SENSORS) {
    readSensor(UltraPins[currentSensor]); // Pass sensor index as identifier
    currentSensor++;
  } else {
    currentSensor = 0; // Reset to read the first sensor again
  }
}

void readSensor(int echoPin) {
  // Trigger the sensor
  pinMode(echoPin, OUTPUT);
  digitalWrite(echoPin, LOW);
  delayMicroseconds(2);
  digitalWrite(echoPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(echoPin, LOW);

  // Switch pin to INPUT to read echo
  pinMode(echoPin, INPUT);
  unsigned long echoTime = pulseIn(echoPin, HIGH, maxDistance); // Maximum imeout
  int distance = echoTime * 0.034 /2;

  Serial.print("Sensor ");
  Serial.print(currentSensor +1);
  Serial.print(" Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}
