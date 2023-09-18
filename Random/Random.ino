#define ECHO 7
#define TRIGGER 8
int distance0;
long duration; 

void setup()
{
  pinMode(ECHO, INPUT); // Sets the trigPin as an Output
  pinMode(TRIGGER, OUTPUT); // Sets the echoPin as an Input

Serial.begin(115200);

}



void loop(){
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);
  duration = pulseIn(ECHO, HIGH,100000);
  distance0 = duration * 0.034 / 2;
  Serial.println(distance0);  
}

