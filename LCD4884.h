/*
Modified by Lauren
version 0.3

Any suggestions are welcome.
E-mail: Lauran.pan@gmail.com

Editor     : Lauren from DFRobot
Date       : 06.01.2012

* Have the back light under control.
* Update the library and sketch to compatible with IDE V1.0 and earlier

*/

#ifndef LCD4884_h
#define LCD4884_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// SPI Interface --- (using Arduino Digital Pin 2,3,4,5,6)
#define SPI_SCK 2     //Serial Clock(Master Output)
#define SPI_MOSI 3   //Master Output,Slave Input
#define LCD_DC  4   //Data/Command(command active low)
#define SPI_CS  5  //Chip Select,Slave Transmit Enable(active low,Master Output)
#define LCD_RST 6 //One Reset button
#define LCD_BL  7//Backlit control (Arduino DIO Pin 7)


//display mode -- normal / highlight
#define MENU_NORMAL 0
#define MENU_HIGHLIGHT 1
#define OFF 0
#define ON 1


class LCD4884
{
  public:
    LCD4884();
    void init(void);
    void backlight(bool);
    void writeByte(uint8_t, bool);
    void LCD_draw_bmp_pixel(uint8_t, uint8_t, unsigned char map[], uint8_t, uint8_t);
    void LCD_write_string(uint8_t, uint8_t, char s[]);
    void LCD_write_char(uint8_t);
    void LCD_set_XY(uint8_t, uint8_t);
    void clear(void);
    void writeLine(uint8_t, char s[]);
};
extern LCD4884 lcd;

#endif
