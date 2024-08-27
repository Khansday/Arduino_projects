#define DECODE_NEC
#include <Arduino.h>
#include "PinDefinitionsAndMore.h" // Define macros for input and output pin etc.
#include <IRremote.hpp>

#include <U8g2lib.h>
#include <Wire.h>

U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0);
int pixelColour = 1;

#include <Servo.h>
Servo robotNeck;  // create servo object to control a servo

#define M1_d 5
#define M1_dr 4
#define M1_speed 3
#define M2_d 7
#define M2_dr 6
#define M2_speed 11

int motorSpeed =180;
const int sensorThreshold = 300; //Adjust this threshold to detect the line ( 0 - 1023 ); 

#define leftSensorPin A1     // Analog pin for left sensor
#define rightSensorPin A2     // Analog pin for right sensor
int leftSensorValue;
int rightSensorValue;

int robotNeck_pin = 9; 

int distanceSensor = A0; 
int distanceValue;
int maxDistance = 200;
int LED_red = 7;
int LED_white = 8;
int buzzer = 12;
int standby = 0;

int mode;
bool RCflag = false;
bool autoFlag = false;
bool lineFlag = false;
char *modestr = "START";

void goForward(){
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , HIGH);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , HIGH);
  analogWrite(M2_speed, motorSpeed);
}

void goBackwards(){
  digitalWrite(M1_d , HIGH);
  digitalWrite(M1_dr , LOW);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , HIGH);
  digitalWrite(M2_dr , LOW);
  analogWrite(M2_speed, motorSpeed); 
}

void turnLeft(){
  digitalWrite(M1_d , HIGH);
  digitalWrite(M1_dr , LOW);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d ,LOW);
  digitalWrite(M2_dr , HIGH);
  analogWrite(M2_speed, motorSpeed);
} 

void turnRight(){
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , HIGH);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , HIGH);
  digitalWrite(M2_dr , LOW);
  analogWrite(M2_speed, motorSpeed);
}

void stop(){
  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , LOW);
  analogWrite(M1_speed, motorSpeed);
  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , LOW);
  analogWrite(M2_speed, motorSpeed);
}

void makeTurn(){
  int distanceLeft;
  int distanceRight;
  goBackwards();
  delay(500);
  stop();
  robotNeck.write(145);
  delay(500);
  distanceLeft =  analogRead(distanceSensor);
  delay(500);
  robotNeck.write(35);
  delay(500);
  distanceRight =  analogRead(distanceSensor);
  delay(500);
  robotNeck.write(90);
  if (distanceLeft >= distanceRight){
    turnLeft();
    delay(1000);
  }
  else{
    turnRight();
    delay(1000);
  }
  goForward();
}

void drawBox(){
  u8g2.firstPage();  
  do {
    u8g2.drawBox(0,0,128,64); //box in the middle of the screen
  } while( u8g2.nextPage() );
}

void writeMode(){
  u8g2.firstPage();  
  do {
    // graphic commands to redraw the complete screen should be placed here  
  u8g2.setFont(u8g_font_unifont);
  //u8g2.setFont(u8g_font_osb21);
  u8g2.drawStr( 0, 22, modestr);
  } while( u8g2.nextPage() );
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(M1_d , OUTPUT);
  pinMode(M1_dr , OUTPUT);
  pinMode(M1_speed , OUTPUT);
  pinMode(M2_d , OUTPUT);
  pinMode(M2_dr , OUTPUT);
  pinMode(M2_speed , OUTPUT);
  pinMode(LED_red , OUTPUT);
  pinMode(LED_white , OUTPUT);
  pinMode(buzzer , OUTPUT);

  digitalWrite(M1_d , LOW);
  digitalWrite(M1_dr , LOW);
  digitalWrite(M1_speed , LOW);
  digitalWrite(M2_d , LOW);
  digitalWrite(M2_dr , LOW);
  digitalWrite(M2_speed , LOW);
  digitalWrite(LED_red , LOW);
  digitalWrite(LED_white , LOW);
  digitalWrite(buzzer , LOW);

  //drawBox();
  u8g2.begin();  


  robotNeck.attach(robotNeck_pin);
  robotNeck.write(90);
  
  // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  // Just to know which program is running on my Arduino
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));
  Serial.print(F("Ready to receive IR signals of protocols: "));
  printActiveIRProtocols(&Serial);
  Serial.println(F("at pin " STR(IR_RECEIVE_PIN)));
}

void loop() {
  if (IrReceiver.decode()) {
    // Print a short summary of received data
    IrReceiver.printIRResultShort(&Serial);
    IrReceiver.printIRSendUsage(&Serial);
    if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
      Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
      // We have an unknown protocol here, print more info
      IrReceiver.printIRResultRawFormatted(&Serial, true);
    }
    Serial.println();
    IrReceiver.resume(); // Enable receiving of the next value
    mode = IrReceiver.decodedIRData.command;
  
    switch (mode){
      case(0x45):   //Num 1
        Serial.println(F("Autonomous mode"));
        modestr = "Autonomous mode";
        RCflag = false;
        autoFlag = true;
        lineFlag = false;  
      break;
      case(0x46):   //Num 2
        Serial.println(F("Remote mode"));
        modestr = "Remote mode";
        RCflag = true;
        autoFlag = false;
        lineFlag = false;
      break;
      case(0x1C):   //ok button
        Serial.println(F("Stop"));
        modestr = "Stop";
        RCflag = false;
        autoFlag = false;
        lineFlag = false;
        stop();
      break;
      case(0x47):   //button 3
        pixelColour = 1 - pixelColour;
        u8g2.setColorIndex(pixelColour);
        drawBox();
      break;
      case(0x44):   //button 4
      modestr = "Line follower";
        RCflag = false;
        autoFlag = false;
        lineFlag = true;
      break;
      default:
        Serial.println(F("Not recognised"));
      break;
    }
    writeMode();
  }
  if (RCflag) RCmode();
  if (autoFlag) beAutonomous();
  if (lineFlag) followLine();
}

void beAutonomous(){
  distanceValue = analogRead(distanceSensor);
  Serial.println(distanceValue);
  if (distanceValue >= maxDistance){
    stop();
    delay(500);
    robotNeck.attach(robotNeck_pin);
    makeTurn(); 
    robotNeck.detach();
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  }
  else{
    goForward();
  }
}

void RCmode(){
  distanceValue = analogRead(distanceSensor);
  if (distanceValue >= maxDistance){
    stop();
    delay(500);
    goBackwards();
    delay(1000);
    stop();
  }
  else{
    switch(mode){
      case(0x18):   //arrow up
        goForward();
        Serial.println(F("go forward"));
      break;
      case(0x52):   //arrow down
        goBackwards();
        Serial.println(F("go bacwards"));
      break;
      case(0x8):   //arrow left
        turnLeft();
        Serial.println(F("go left"));
      break;
      case(0x5A):   //arrow right
        turnRight();
        Serial.println(F("go right"));
      break;
    }
  }
}

void followLine(){
  leftSensorValue = analogRead(leftSensorPin);
  rightSensorValue = analogRead(rightSensorPin);
  distanceValue = analogRead(distanceSensor);
  if (distanceValue >= maxDistance){
    stop();
  }
  // Line follower logic
  else if (leftSensorValue > sensorThreshold && rightSensorValue > sensorThreshold) {
    // Both sensors are on the line
    goForward();
  } else if (leftSensorValue > sensorThreshold && rightSensorValue < sensorThreshold) {
    // Left sensor is on the line only
    turnLeft();
  } else if (rightSensorValue > sensorThreshold && leftSensorValue < sensorThreshold) {
    // Right sensor is on the line only
    turnRight(); 
  } else {
    // Both sensors are off the line
    stop();
  }
}
