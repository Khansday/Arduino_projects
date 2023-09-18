
#include "U8glib.h"

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);

uint8_t x_box = 10;
uint8_t y_box = 10;
int multiplier;

void setup() {
  // put your setup code here, to run once:
//if (! u8g.begin(void)){
//while(1);
//}
//Serial.begin(9600);
}


void draw(void) {
  // graphic commands to redraw the complete screen should be placed here  
  //u8g.setFont(u8g_font_unifont);
  //u8g.setFont(u8g_font_osb21);
  //u8g.drawStr( 0, 22, "Hello World!");
  u8g.drawBox(x_box,y_box,10,10); //box in the middle of the screen
  u8g.drawFrame(0,0,128,64); //frame all around the screen
  
//  u8g.enableCursor();
//  u8g.setCursorPos(15, 15);
//  u8g.setCursorStyle(0x20);
//  u8g.enableCursor();
}

void loop() {
  // put your main code here, to run repeatedly:
  multiplier = analogRead(0);
  //Serial.println(multiplier);
  u8g.firstPage();  
  do {
    draw();
  } while( u8g.nextPage() );

  x_box = map(multiplier,0 , 1023 , 0, 118);
  //x_box = multiplier;
  y_box = map(multiplier,0 , 1023 , 0, 54);
  //y_box = multiplier;
  // Serial.println(x_box);
  // Serial.println(y_box);
//  x_box = 118;
//  y_box = 54;
//  if (x_box >50){ x_box=10;}
//  if (y_box >50){ y_box=10;}
}
