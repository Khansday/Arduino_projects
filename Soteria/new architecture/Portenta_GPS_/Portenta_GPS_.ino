#include "Arduino.h"
#include "portenta_info.h"
#include "mbed.h"

// Define latitude and longitude as global variables
float latitude = 0.0;
float longitude = 0.0;

void setup() {
  Serial.begin(250000);  // Debugging output
  Serial.println("Program started");
  Serial1.begin(9600);   // Assuming GPS module communicates at 9600 baud rate
}

void loop() {
  while (Serial1.available() > 0) {   // Check for GPS data
    //Serial.println("Data received");
    String nmeaData = Serial1.readStringUntil('\n');

    // Check if the data is a GNGGA or GPGGA sentence
    if (nmeaData.startsWith("$GNGGA") || nmeaData.startsWith("$GPGGA")) {
      // Serial.println("Data verified");
      // parseGGA(nmeaData);
      // printLatLon();
      Serial.println(nmeaData);
    }
  }
}

void parseGGA(String sentence) {  // GPS parsing function
  int commaIndex;
  int startIndex = 0;
  String fields[15];  // GGA has up to 15 fields

  // Split the NMEA sentence into fields
  for (int i = 0; i < 15; i++) {
    commaIndex = sentence.indexOf(',', startIndex);
    if (commaIndex == -1) {
      fields[i] = sentence.substring(startIndex);
      break;
    }
    fields[i] = sentence.substring(startIndex, commaIndex);
    startIndex = commaIndex + 1;
  }

  // Check if it's a GGA sentence and has enough fields for latitude and longitude
  if ((fields[0] == "$GNGGA" || fields[0] == "$GPGGA") && fields[6].toInt() > 0) {
    // Only proceed if fix quality is greater than 0 (valid fix)
    if (fields[2].length() >= 4 && fields[4].length() >= 5) {
      // Extract latitude
      latitude = fields[2].substring(0, 2).toDouble() + fields[2].substring(2).toDouble() / 60.0;
      if (fields[3] == "S") {
        latitude = -latitude;
      }

      // Extract longitude
      longitude = fields[4].substring(0, 3).toDouble() + fields[4].substring(3).toDouble() / 60.0;
      if (fields[5] == "W") {
        longitude = -longitude;
      }
    }
  } else {
    // Handle no fix case
    Serial.println("No GPS fix");
  }
}


void printLatLon() {
  Serial.print("Lat: ");
  Serial.print(latitude, 6);  // Printing with precision
  Serial.print(" Lon: ");
  Serial.println(longitude, 6);  // Printing with precision
}
