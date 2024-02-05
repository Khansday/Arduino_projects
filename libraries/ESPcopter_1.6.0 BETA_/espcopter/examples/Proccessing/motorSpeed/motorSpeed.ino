#include <ESP8266WiFi.h>
const char WiFiAPPSK[] = "";
WiFiServer server(80);

#define blueLed 16
#define redLed 2
#define greenLed 0

int red = 0, green = 0, blue = 0;
int pinPWM[4] = {14, 15, 12, 13}; //Define PWM pin
int throttle = 0; 
String strial;
String inString = ""; 
int currentColor = 0;
void setup() {
  Serial.begin(9600);
 analogWriteFreq(20000); 
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(blueLed, OUTPUT);
  
  pinMode(pinPWM[0], OUTPUT);
  pinMode(pinPWM[1], OUTPUT);
  pinMode(pinPWM[2], OUTPUT);
  pinMode(pinPWM[3], OUTPUT);
  
 setupWiFi();

}

void loop() {
WiFiClient client = server.available();
 Serial.println("waiting... ");

  Serial.println(client);
  if (client ) {
    while (client.connected()) {
      while(1){
      if (client.available()) {
   
        int inChar;
        inChar = client.read();

  if (isDigit(inChar)) {
    // convert the incoming byte to a char 
    // and add it to the string:
    inString += (char)inChar; 
  }

   if (inChar == '*') {
    // do something different for each value of currentColor:
    switch (currentColor) {
      case 0:    // 0 = red
        red = inString.toInt();
        // clear the string for new input:
        inString = "";
        break;
      case 1:    // 1 = green:
        green = inString.toInt();
        // clear the string for new input:
        inString = "";
        break;
    }
    currentColor++;
  }
  // if you get a newline, you know you've got
  // the last color, i.e. blue:
  if (inChar == '?') {
    throttle = inString.toInt();
    // clear the string for new input:
    inString = "";
    // reset the color counter:
    currentColor = 0;
  }
    }

  
     analogWrite(pinPWM[0], throttle);
     analogWrite(pinPWM[1], throttle);
     analogWrite(pinPWM[2], throttle);
     analogWrite(pinPWM[3], throttle);

   
    }

   // Serial.println("Client disconnected.");
   // client.stop();
 } 

 }
}


void setupWiFi(){
  WiFi.mode(WIFI_AP);

  // Do a little work to get a unique-ish name. Append the
  // last two bytes of the MAC (HEX'd) to "Thing-":
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();
  String AP_NameString = "ESPCopter";

  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);

  for (int i=0; i<AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);

   WiFi.softAP(AP_NameChar, WiFiAPPSK);

   server.begin();
}