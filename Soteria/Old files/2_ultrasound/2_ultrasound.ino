const int triggerPin = 2;
const int echoPin = 3;
const int echoPin2 = 4;

const int maxDistance = 200;

unsigned long startTime;
unsigned long echoTime;
bool sensorReady = false;
int distance = 0, distance1 = 0;

void setup() {
  Serial.begin(9600);
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(echoPin2, INPUT);

}

void loop() {

    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    echoTime = pulseIn(echoPin, HIGH);
    distance = echoTime / 29 / 2;

    delay(50);

    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    echoTime = pulseIn(echoPin2, HIGH);
    distance1 = echoTime / 29 / 2;




    delay(50);

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm");

  Serial.print("   Distance1: ");
  Serial.print(distance1);
  Serial.println(" cm");
}
