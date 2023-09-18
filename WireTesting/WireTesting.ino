// Input pins:<br />
int rside         = A0; // right side sensor input
int rfront        = A1; //front left line sensor input
int lfront        = A2; //front right left sensor input
int rf= 0 , lf = 0 , ls =0 , rs=0;
int lside         = A3; // left side sensor input 
int fourwayswitch = A6; // input from function switch
int Receive    = 0;  // Receive pin
int Transmit    = 1;  // Transmit pin
int m1encoder1    = 2;  // motor 1 encoder 1 input interrupt pin
int m1encoder2    = 4;  // motor 1 encoder 2 input
int m2encoder1    = 3;  // motor 2 encoder 1 input interrupt pin
int m2encoder2    = 5;  // motor 2 encoder 2 input
// Output pins:
int sensorLED1    = 6;  // 1st diagnostic LED on sensor board
int lmotorDIR     = 7;  // Left motor dirn input 1
int rmotorDIR     = 8;  // Right motor dirn input 3
int lmotorPWM     = 9;  // Left motor PWN pin<br />
int rmotorPWM     = 10; // Right motor PWN pin<br />
int sensorLED2    = 11; // 2nd diagnostic LED on sensor board<br />
int trigger       = 12; // trigger for sensor LEDs<br />
int LED13         = 13; // ext LED Red<br />
//line follower parameters
int treshold; // contains the value pf the treshold to sense the line
int maxspeed= 255; //max speed for PWM 0-255
int minspeed= 50; //speed of inner wheel at curve
bool onStart = false; //to store start finish line

int temp=maxspeed; //variable to store speed value
void setup() {
  // put your setup code here, to run once:
  pinMode(sensorLED1, OUTPUT);
  pinMode(lmotorDIR, OUTPUT);
  pinMode(rmotorDIR, OUTPUT);
  pinMode(lmotorPWM, OUTPUT);
  pinMode(rmotorPWM, OUTPUT);
  pinMode(sensorLED2, OUTPUT);
  pinMode(trigger, OUTPUT);
  pinMode(LED13, OUTPUT);
  pinMode(m1encoder1, INPUT);
  pinMode(m1encoder2, INPUT);
  pinMode(m2encoder1, INPUT);
  pinMode(m2encoder2, INPUT);
  Serial.begin(9600);          // set up serial monitor comms on USB
  digitalWrite(trigger, 0);
  digitalWrite(sensorLED1, 1); 
  digitalWrite(sensorLED2, 1);
  // increase speed of ADC conversions to 28us each
  // by changing the clock prescaler from 128 to 16
  bitClear(ADCSRA, ADPS0);
  bitClear(ADCSRA, ADPS1);
  bitSet(ADCSRA, ADPS2);
  //setting the treshold levels
  lf= analogRead(lfront); //the left one
  digitalWrite(trigger, 1);
  delay(5);
  treshold = (lf + analogRead(lfront))/2;
  
}
void STOP()
{
  digitalWrite(lmotorPWM, LOW); //max speeed
  digitalWrite(rmotorPWM, LOW);//maxspeed
}
void AHEAD()
{
  analogWrite(lmotorPWM, maxspeed); //max speeed
  analogWrite(rmotorPWM, maxspeed);//maxspeed
}
void LEFT()
{
  digitalWrite(sensorLED1, 1);
  analogWrite(lmotorPWM, maxspeed); //max speeed
  analogWrite(rmotorPWM, minspeed);//slowly keep going ahead
}
void RIGHT()
{
  digitalWrite(sensorLED2, 1);
  analogWrite(lmotorPWM, minspeed);//slowly keep going ahead
  analogWrite(rmotorPWM, maxspeed);//maxspeed
}

void loop(void) 
{
rf= analogRead(A0);
Serial.print(rf); // left one
Serial.print("\n"); //new line
}
