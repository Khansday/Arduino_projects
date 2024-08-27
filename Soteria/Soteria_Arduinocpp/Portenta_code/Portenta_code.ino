/* 
 * This sketch shows how an arduino board can act as a host for nicla. 
 * An host board can configure the sensors of nicla and then read their values.
 * The host board should be connected to nicla through the eslov connector.
 * 
 * In this example, the temperature sensor is enabled and its
 * values are periodically read and then printed to the serial channel
 * 
 * NOTE: if Nicla is used as a Shield on top of a MKR board,
 * please use BHY2Host.begin(false, NICLA_AS_SHIELD)
*/

#include "Arduino.h"
#include "Arduino_BHY2Host.h"
#include "mbed_power_mgmt.h"
#include "portenta_info.h"
#include "mbed.h"



const int numSensors = 5;  // Number of ultrasonic sensors

// Pin configurations
int triggerPins[numSensors] = {D6, D6, D6, D6, D6};  // Common trigger pin
int echoPins[numSensors] = {D2, D3, D4, D1, D5};    // Echo pins for each sensor
unsigned long durations[numSensors];
const float speedOfSound = 0.0343;  // Speed of sound in air in cm/microsecond

Sensor temp(SENSOR_ID_TEMP_WU);
Sensor humidity(SENSOR_ID_HUM);
Sensor pressure(SENSOR_ID_BARO);
Sensor gas(SENSOR_ID_GAS);
SensorXYZ gyroscope(SENSOR_ID_GYRO);
SensorXYZ accelerometer(SENSOR_ID_ACC);
SensorQuaternion quaternion(SENSOR_ID_RV);
SensorBSEC bsec(SENSOR_ID_BSEC);
int yled = LEDR;
int gled = LEDG;
int buzz = D0;
int heart_in = A3;
int heart_lo_n = A4;
int heart_lo_p = D5;
//int ext_temp = A2;

int drate = 100;

void setup()
{
 

  Serial.begin(460800);
  //while(!Serial);
  pinMode(buzz,OUTPUT);
  pinMode(yled,OUTPUT);
  pinMode(gled,OUTPUT);
  pinMode(heart_in,INPUT);
  pinMode(heart_lo_n,INPUT);
  pinMode(heart_lo_p,INPUT);
  //pinMode(ext_temp,INPUT);
 
  digitalWrite(yled,LOW);
  delay(50);
  for (int i = 0;i<10;i++){
      digitalWrite(yled,HIGH);
      delay(50);
      digitalWrite(yled,LOW);
      delay(50);
  }

  digitalWrite(gled,LOW);
  digitalWrite(gled,HIGH);

  digitalWrite(buzz,HIGH);
  delay(1000);
  digitalWrite(buzz,LOW);




  BHY2Host.begin();
  temp.begin(2000,0);
  humidity.begin(2000,0);
  pressure.begin();
  gyroscope.begin();
  accelerometer.begin();
  bsec.begin();
  gas.begin();

  delay(10);
  digitalWrite(gled,LOW);
  delay(3000);

  for (int i = 0; i < numSensors; i++) {
    pinMode(triggerPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT_PULLDOWN);
  }
 
}

void loop()
{

  static auto printTime = millis();
  BHY2Host.update();
  delay(5);

    if (millis() - printTime >= drate) {
      Serial.println("");
      Serial.println("");
      //Serial.println("Temperature: " + String(temp.value()-5.4) + "°C");
      //Serial.println("Humidity: " + String(humidity.value()) + "%");
      Serial.println("Air Quality: " + String(bsec.iaq()));
      Serial.println("Atmospheric Pressure: " + pressure.toString() + " hPa");
      Serial.println(String("Gyroscope values: ") + gyroscope.toString());
      Serial.println(String("Accelerometer values: ") + accelerometer.toString());
      //Serial.println(analogRead(ext_temp));
      printTime = millis();

    }


    for (int i = 0; i < numSensors; i++) {
      // Trigger ultrasonic sensor
      digitalWrite(triggerPins[i], LOW);
      delayMicroseconds(10);
      digitalWrite(triggerPins[i], HIGH);
      delayMicroseconds(10);
      digitalWrite(triggerPins[i], LOW);

      // Measure duration
      durations[i] = pulseIn(echoPins[i], HIGH, 40000UL);
      
      // Calculate distance in centimeters
      float distance_cm = (durations[i] * speedOfSound) / 2.0;
      
      // Print distance to serial monitor
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.print(" - Distance: ");
      Serial.print(distance_cm);
      Serial.println(" cm");
        if (distance_cm < 25) {       

      digitalWrite(buzz,HIGH);
      } 
      else { 

      digitalWrite(buzz,LOW);
      }
      delay(100);
      
    }

}
