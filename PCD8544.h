#define PIN_SCE   7   // backlight
#define PIN_RESET 6   // reset pin (active low)
#define PIN_CS    5   // chip select (active low)
#define PIN_DC    4   // data / command (command = low)
#define PIN_SDIN  3   // serial data pin
#define PIN_SCLK  2   // serial clock pin

#define LCD_C     LOW
#define LCD_D     HIGH

#define LCD_X     84
#define LCD_Y     48

void LcdCharacter(char character);
void LcdClear();
void LcdInitialise();
void LcdString(char *characters);
void LcdWrite(byte dc, byte data);
