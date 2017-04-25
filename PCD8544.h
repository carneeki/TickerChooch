#define PIN_SCE   7
#define PIN_RESET 6
#define PIN_DC    5
#define PIN_SDIN  4
#define PIN_SCLK  3

#define LCD_C     LOW
#define LCD_D     HIGH

#define LCD_X     84
#define LCD_Y     48

void LcdCharacter(char character);
void LcdClear();
void LcdInitialise();
void LcdString(char *characters);
void LcdWrite(byte dc, byte data);
