
int IR_0 = A0;   //eyes on pin A0 
int IR_1 = A1;   //eyes on pin A0 
int distance1,distance0;

void setup() {
  // put your setup code here, to run once:
  //pinMode( Eyes , INPUT);
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
    distance1= analogRead(IR_1);
    distance0= analogRead(IR_0);
  if (distance0 < 400 && distance1 >550){
    Serial.println("S1 detected");
  }
  else if (distance1 < 400 && distance0 >550){
    Serial.println("S0 detected"); 
  }
  else {
    Serial.println("NONE detected"); 
  }
  delay(200);
}

