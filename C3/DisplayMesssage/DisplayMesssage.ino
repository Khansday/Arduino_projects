#include <U8g2lib.h>

U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

char *a = "CIAO";
void setup(void) {
  u8g2.begin();
}

void loop(void) {
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 10);
    u8g2.print(a);
  } while ( u8g2.nextPage() );
  delay(1000);
}

