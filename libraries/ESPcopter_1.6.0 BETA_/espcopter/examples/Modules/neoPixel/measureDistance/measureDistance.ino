#define STANDALONE  // standard mode 

#include "espcopter.h" // library


void setup() {
  mainSetup(); // main flying setup
  
}

void loop() {
   mainLoop ();  // main flying loop
   
   int ledCount =  map(getMeasureAltitude(),0,1200,0,11); // change mm to led number
  
   for(int i =0; i < ledCount; i++){ // turn on led up to current altitude 
   ESPsetPixel (i,125,0,0);
   }

   for(int i =11; ledCount < i; i--){// turn off led up to current altitude 
   ESPsetPixel (i,0,0,0);
   }

   ESPpixelShow(); // commit led change
   
   
 }