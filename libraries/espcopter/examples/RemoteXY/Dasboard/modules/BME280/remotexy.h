/*
   -- New project --
   
   This source code of graphical user interface 
   has been generated automatically by RemoteXY editor.
   To compile this code using RemoteXY library 2.4.3 or later version 
   download by link http://remotexy.com/en/library/
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/                   
     - for ANDROID 4.7.12 or later version;
     - for iOS 1.4.7 or later version;
    
   This source code is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.    
*/

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__ESP8266WIFI_LIB_POINT
#include <ESP8266WiFi.h>

#include <RemoteXY.h>

// RemoteXY connection settings 
#define REMOTEXY_WIFI_SSID "RemoteXY"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377


// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
  { 255,0,0,12,0,104,0,11,13,0,
  129,0,14,4,75,4,17,84,101,109,
  112,101,114,97,116,117,114,101,32,72,
  117,109,105,100,105,116,121,32,80,114,
  101,115,115,117,114,101,32,83,101,110,
  115,111,114,0,68,49,0,21,34,27,
  8,36,84,101,109,112,101,114,97,116,
  117,114,101,32,0,68,49,34,21,33,
  27,8,134,72,117,109,105,100,105,116,
  121,32,0,68,49,67,21,33,27,8,
  204,80,114,101,115,115,117,114,101,32,
  0 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // output variables
  float onlineGraph_1;
  float onlineGraph_2;
  float onlineGraph_3;

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

