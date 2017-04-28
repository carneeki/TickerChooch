#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

extern "C"
{
  #include <stdlib.h>
  #include <avr/pgmspace.h>
  #include <avr/io.h>
}

#include "TinkerChooch.h"
#include "drawLCD.h"
#include "font.h"
#include "LCD4884.h"

void drawSpeed(uint16_t sp, bool fwd)
{
  char buf[20]       = ""; // buffer for text to go to LCD
  strcpy(buf, "SPEED: ");

  strcat(buf, (const char*) ((fwd)? 0xaf : 0xae) );
  strcat(buf, " ");

  for(uint16_t i = 0; i< (sp+1)/204; i++)
    strcat(buf, (const char*) 0xdb);

  lcd.writeLine(0, buf);
}

void drawDistance(uint16_t distance)
{
  char buf[20]       = ""; // buffer for text to go to LCD
  strcpy(buf, " DIST:          ");

  // 3 digits long
  if(distance < 1000 && distance >= 100)
    itoa(distance, &buf[7], 10); // FIXME: Len would be appalled

  // 2 digits long
  if(distance <  100 && distance >=  10)
    itoa(distance, &buf[8], 10); // FIXME and Pong

  // 1 digit long
  if(distance <   10)
    itoa(distance, &buf[9], 10); // FIXME and Mitchell

  strcat(buf, " mm  ");

  lcd.writeLine(1, buf);
}

void drawPause(uint16_t pauseTime)
{
  char buf[20]       = ""; // buffer for text to go to LCD
  strcpy(buf, "PAUSE:          ");

  if(pauseTime < 1000 && pauseTime > 100)
    itoa(pauseTime, &buf[7], 10);

  if(pauseTime <  100 && pauseTime >  10)
    itoa(pauseTime, &buf[8], 10);

  if(pauseTime <   10)
    itoa(pauseTime, &buf[9], 10);

  strcat(buf, " s");

  lcd.writeLine(2, buf);
}

void drawBoxTop()
{
  char buf[20]       = ""; // buffer for text to go to LCD
  strcpy(buf, "\xC9\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xBB");
  lcd.writeLine(3, buf);
}

void drawBoxBottom()
{
  char buf[20]       = ""; // buffer for text to go to LCD
  strcpy(buf, "\xC8\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xCD\xBC");
  lcd.writeLine(4, buf);
}

void drawChooching()
{
  char buf[20]       = ""; // buffer for text to go to LCD
  strcpy(buf, "\xBA  CHOOCHING!  \xBA");
  lcd.writeLine(5, buf);
}

void drawStart()
{
  char buf[20]       = ""; // buffer for text to go to LCD
  strcpy(buf, "\xBA PRESS  START \xBA");
  lcd.writeLine(4, buf);
}

void drawPauses(uint16_t remainingTime)
{
  char buf[20]       = ""; // buffer for text to go to LCD
  strcpy(buf, "\xBA PRESS  START \xBA");
  strcpy(buf, "\xBA PAUSED:       ");

  if(remainingTime < 1000 && remainingTime > 100)
    itoa(remainingTime, &buf[7], 10);

  if(remainingTime <  100 && remainingTime >  10)
    itoa(remainingTime, &buf[8], 10);

  if(remainingTime <   10)
    itoa(remainingTime, &buf[9], 10);


  strcat(buf, " s\xBA");

  lcd.writeLine(4, buf);
}

void drawInit1()
{
  for(unsigned int i=0; i< sizeof(bm0); i++)
    lcd.writeByte(true, pgm_read_byte_near(bm0[i]));
  Serial.println("Logo 0");
}

void drawInit2()
{
  for(unsigned int i=0; i< sizeof(bm0); i++)
    lcd.writeByte(true, pgm_read_byte_near(bm1[i]));
  Serial.println("Logo 1");
}

void drawPanic()
{
  char buf[20] = "";
  drawBoxTop();

  strcpy(buf, "\xBA              \xBA");
  lcd.writeLine(1, buf);

  strcpy(buf, "\xBA    PANIC!    \xBA");
  lcd.writeLine(2, buf);

  strcpy(buf,"\xBA      :(      \xBA");
  lcd.writeLine(3, buf);

  strcpy(buf, "\xBA              \xBA");
  lcd.writeLine(4, buf);

  drawBoxBottom();
}
