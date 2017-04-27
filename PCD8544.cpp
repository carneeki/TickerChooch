/*
 * PCD8544 based on http://playground.arduino.cc/Code/PCD8544
 * but with some of the extended ASCII code.
 *
 * I am too lazy to add the remaining 128 (or so) characters myself
 * right now, so I am only going to add the ones necessary, and will do more
 * in due course.
 *
 * I realise this is not the most efficient use of memory, but it's there, so
 * let's just use and abuse it.
 */

// NOTE: Probably best to not change anything here
#include <stdint.h>
#include <Arduino.h>
#include "PCD8544.h"
#include "font.h"

void LcdCharacter(char character)
{
  LcdWrite(LCD_D, 0x00);

  uint8_t buf[5]=""; // buffer the character to send
  memcpy(buf, ASCII[character -0x20], sizeof(buf));
  for(int i=0; i<5; i++)
    LcdWrite(LCD_D, buf[i]);

  LcdWrite(LCD_D, 0x00);
}

void LcdClear()
{
  for (int index = 0; index < LCD_X * LCD_Y / 8; index++)
    LcdWrite(LCD_D, 0x00);
}

void LcdInitialise()
{
  pinMode(PIN_SCE, OUTPUT);
  pinMode(PIN_RESET, OUTPUT);
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_DC, OUTPUT);
  pinMode(PIN_SDIN, OUTPUT);
  pinMode(PIN_SCLK, OUTPUT);
  digitalWrite(PIN_RESET, LOW);
  digitalWrite(PIN_RESET, HIGH);
  digitalWrite(PIN_CS, LOW);
  LcdWrite(LCD_C, 0x21 );  // LCD Extended Commands.
  LcdWrite(LCD_C, 0xB1 );  // Set LCD Vop (Contrast).
  LcdWrite(LCD_C, 0x04 );  // Set Temp coefficent. //0x04
  LcdWrite(LCD_C, 0x14 );  // LCD bias mode 1:48. //0x13
  LcdWrite(LCD_C, 0x20 );  // LCD Basic Commands
  LcdWrite(LCD_C, 0x0C );  // LCD in normal mode.
}

void LcdString(char *characters)
{
  while (*characters)
  {
    LcdCharacter(*characters++);
  }
}

void LcdWrite(byte dc, byte data)
{
  digitalWrite(PIN_DC, dc);
  digitalWrite(PIN_SCE, LOW);
  shiftOut(PIN_SDIN, PIN_SCLK, MSBFIRST, data);
  digitalWrite(PIN_SCE, HIGH);
}
