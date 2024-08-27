#include "Arduino.h"
#include "Arduino_BHY2.h"
#include "Nicla_System.h"

Sensor temp(SENSOR_ID_TEMP);
Sensor humidity(SENSOR_ID_HUM);
Sensor pressure(SENSOR_ID_BARO);
Sensor gas(SENSOR_ID_GAS);
SensorXYZ gyroscope(SENSOR_ID_GYRO);
SensorXYZ accelerometer(SENSOR_ID_ACC);
SensorQuaternion quaternion(SENSOR_ID_RV);
SensorBSEC bsec(SENSOR_ID_BSEC);

#define DEBUG false

void setup() {
  nicla::begin();
  nicla::leds.begin();
  Serial.begin(230400);
#if DEBUG
  BHY2.debug(Serial);
#endif
  BHY2.begin();
  temp.begin();
}

void loop() {
  BHY2.update(100);

}
