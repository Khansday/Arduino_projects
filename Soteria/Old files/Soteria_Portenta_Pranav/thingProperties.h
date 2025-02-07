// Code generated by Arduino IoT Cloud, DO NOT EDIT.

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>

const char SSID[]     = SECRET_SSID;    // Network SSID (name)
const char PASS[]     = SECRET_OPTIONAL_PASS;    // Network password (use for WPA, or use as key for WEP)

void onAirQualityChange();
void onHmdtChange();
void onPrssChange();
void onTmpChange();
void onAxChange();
void onAyChange();
void onAzChange();
void onDistance1Change();
void onDistance2Change();
void onDistance3Change();
void onDistance4Change();
void onDistance5Change();
void onGxChange();
void onGyChange();
void onGzChange();
void onPulseBeatChange();
void onLocationChange();

float air_Quality;
float hmdt;
float prss;
float tmp;
int ax;
int ay;
int az;
int distance1;
int distance2;
int distance3;
int distance4;
int distance5;
int gx;
int gy;
int gz;
int pulse_Beat;
CloudLocation location;

void initProperties(){

  ArduinoCloud.addProperty(air_Quality, READWRITE, ON_CHANGE, onAirQualityChange);
  ArduinoCloud.addProperty(hmdt, READWRITE, ON_CHANGE, onHmdtChange);
  ArduinoCloud.addProperty(prss, READWRITE, ON_CHANGE, onPrssChange);
  ArduinoCloud.addProperty(tmp, READWRITE, ON_CHANGE, onTmpChange);
  ArduinoCloud.addProperty(ax, READWRITE, ON_CHANGE, onAxChange);
  ArduinoCloud.addProperty(ay, READWRITE, ON_CHANGE, onAyChange);
  ArduinoCloud.addProperty(az, READWRITE, ON_CHANGE, onAzChange);
  ArduinoCloud.addProperty(distance1, READWRITE, ON_CHANGE, onDistance1Change);
  ArduinoCloud.addProperty(distance2, READWRITE, ON_CHANGE, onDistance2Change);
  ArduinoCloud.addProperty(distance3, READWRITE, ON_CHANGE, onDistance3Change);
  ArduinoCloud.addProperty(distance4, READWRITE, ON_CHANGE, onDistance4Change);
  ArduinoCloud.addProperty(distance5, READWRITE, ON_CHANGE, onDistance5Change);
  ArduinoCloud.addProperty(gx, READWRITE, ON_CHANGE, onGxChange);
  ArduinoCloud.addProperty(gy, READWRITE, ON_CHANGE, onGyChange);
  ArduinoCloud.addProperty(gz, READWRITE, ON_CHANGE, onGzChange);
  ArduinoCloud.addProperty(pulse_Beat, READWRITE, ON_CHANGE, onPulseBeatChange);
  ArduinoCloud.addProperty(location, READWRITE, ON_CHANGE, onLocationChange);

}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
