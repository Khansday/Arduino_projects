#define NUM_SENSORS 5

const int triggerPin = 2; // Shared trigger pin for all sensors
const int echoPins[NUM_SENSORS] = {3, 4, 5, 6, 7}; // Echo pins for each sensor
const int maxDistance = 200; // Maximum distance in centimeters

unsigned long echoTime;
bool sensorReady = false;
int currentSensor = 0;
int distance = 0;
unsigned long previousMillis = 0;
const long interval = 1000 ; // Interval to read one sensor

void setup() {
  Serial.begin(9600);
  pinMode(triggerPin, OUTPUT);
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(echoPins[i], INPUT);
  }
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
    readSensor(echoPins[currentSensor], currentSensor + 1); // Pass sensor index as identifier
    currentSensor++;
  } else {
    currentSensor = 0; // Reset to read the first sensor again
  }
}

void readSensor(int echoPin, int sensorNumber) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  echoTime = pulseIn(echoPin, HIGH, 100000);  //change the last parameter to change timeout limit
  distance = echoTime / 29 / 2;

  Serial.print("Sensor ");
  Serial.print(sensorNumber);
  Serial.print(" Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}
