#define NUM_SENSORS 5

const int triggerPin = 2; // Shared trigger pin for all sensors
const int echoPins[NUM_SENSORS] = {3, 4, 5, 6, 7}; // Echo pins for each sensor
const int maxDistance = 200; // Maximum distance in centimeters

unsigned long startTime;
unsigned long echoTime;
bool sensorReady = false;
int distance = 0;

void setup() {
  Serial.begin(115200);
  pinMode(triggerPin, OUTPUT);
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(echoPins[i], INPUT);
  }
}

void loop() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    readSensor(echoPins[i], i + 1); // Pass sensor index as identifier
    delay(400);
  }
  
}

void readSensor(int echoPin, int sensorNumber) {

    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    echoTime = pulseIn(echoPin, HIGH);
    distance = echoTime / 29 / 2;




  Serial.print("Sensor ");
  Serial.print(sensorNumber);
  Serial.print(" Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}
