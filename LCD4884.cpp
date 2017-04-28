/*
Modified by Adam <carneeki@carneeki.net>
Version ???

Modified by Lauren
version 0.3

Any suggestions are welcome.
E-mail: Lauran.pan@gmail.com

Editor     : Lauren from DFRobot
Date       : 06.01.2012

* Have the back light under control.
* Update the library and sketch to compatible with IDE V1.0 and earlier

*/

#include "LCD4884.h"
#include "font.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "WConstants.h"
#endif


extern "C"
{
#include <avr/pgmspace.h>
#include <avr/io.h>
}


LCD4884::LCD4884()
{};

LCD4884 lcd = LCD4884();

void LCD4884::backlight(bool bl)
{
  digitalWrite(LCD_BL, (bl)?HIGH:LOW);
}

void LCD4884::init(void)
{
	for(int i = 2; i < 8; i++)	// pin2--pin7
	{
		pinMode(i,OUTPUT);
		digitalWrite(i,LOW);
	}

	digitalWrite(LCD_RST,LOW);
	delayMicroseconds(1);
	digitalWrite(LCD_RST,HIGH);

	digitalWrite(SPI_CS,LOW);  //Chip Select,Slave Transmit Enable(active low,Master Output)
	delayMicroseconds(1);
	digitalWrite(SPI_CS,HIGH);
	delayMicroseconds(1);
	digitalWrite(LCD_BL,HIGH);

	// data_type=0, all are command bytes
	writeByte(0x21, 0); //Function Set:0 0 1 0 0 PD V H=0010 0001;PD=0,V=0,H=1;
	writeByte(0xc0, 0); //Set Vop:1 Vop6 Vop5 Vop4 Vop3 Vop2 Vop1 Vop0=1100 0000
	writeByte(0x06, 0); //Set Temperature Coefficient:0 0 0 0 0 1 Tc1 Tc0=0000 0110;Tc1=1,Tc0=0(Vlcd temperature coefficient 2)
  writeByte(0x13, 0); //Set Bias System (BSx):0 0 0 1 0 BS2 BS1 BS0=0001 0011;BS2=0, BS1=1, BS0=1==>N=4,MUX RATE=1:48

  writeByte(0x20, 0); //Function Set:0 0 1 0 0 PD V H=0010 0000;PD=0,V=0,H=0;
	clear();
	writeByte(0x0c, 0); //Display Control: 0 0 0 0 1 D 0 E=0000 1100 ;D=1,E=0:normal mode

	digitalWrite(SPI_CS,LOW);
}

/**
 * uint8_t c character to send
 * bool dc data = true, command = false
 */
void LCD4884::writeByte(uint8_t dat, bool dc)
{
  unsigned int i;
	digitalWrite(SPI_CS,LOW); //Chip Enable:Active LOW

  digitalWrite(LCD_DC, (dc)?HIGH:LOW);

	for(i=0;i<8;i++)
	{
    digitalWrite(SPI_MOSI, (dat & 0x80)? HIGH:LOW);

		digitalWrite(SPI_SCK,LOW);
		dat = dat << 1;
		digitalWrite(SPI_SCK,HIGH);
	}
	digitalWrite(SPI_CS,HIGH);
}

void LCD4884::LCD_draw_bmp_pixel(uint8_t X,uint8_t Y,unsigned char *map,
                  unsigned char Pix_x,unsigned char Pix_y)
{
    unsigned int i,n;
    unsigned char row;

    if (Pix_y%8==0) row=Pix_y/8;//row from 1 to 6;Pix_y from R0 to R47
      else
        row=Pix_y/8+1;//Quotient+1

    for (n=0;n<row;n++)
    {
		LCD_set_XY(X,Y);
        for(i=0; i<Pix_x; i++)
          {
            writeByte(map[i+n*Pix_x], 1);// D/C=1:write data to display RAM
          }
        Y++;
      }
}

/**
 * write a line on line number
 * with characters in char s[]
 */
void LCD4884::writeLine(uint8_t line, char s[])
{
  LCD_set_XY(0, 8*line);
  for(uint8_t i = 0; i <= sizeof(s); i++)
    LCD_write_char(s[i]);
}

void LCD4884::LCD_write_char(uint8_t c)
{
  c -= 0x20;

  for(uint8_t i = 0; i<5; i++)
    writeByte( pgm_read_byte(ASCII[c][i]), true );
}

/******************************************************************/
void LCD4884::LCD_set_XY(uint8_t X, uint8_t Y)
{
  writeByte(0x40 | Y, 0);		// column
  writeByte(0x80 | X, 0);   // row
}


void LCD4884::clear(void)
{
  unsigned int i;

  writeByte(0x0c, 0);
  writeByte(0x80, 0);

  for (i=0; i<504; i++)//6*84
    writeByte(0, 1);
}
