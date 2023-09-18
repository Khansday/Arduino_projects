void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  //initialise serial monitor
  delay(30);    //giving time to complete the initialistaion
  bool a = true;
  Serial.println(a); //print the bool, displays 1
  a = false;
  Serial.println(a); //print the bool, display 0 
  float num = 10.5;

  
  Serial.println(num,4); //print the float,num of decimals


  for (int i = 10; i<5; i=i+10){
    Serial.println(i);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
