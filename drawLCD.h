#ifndef DRAWLCD_H
#define DRAWLCD_H
void drawSpeed(uint16_t sp, bool fwd);
void drawDistance(uint16_t distance);
void drawPause(uint16_t pauseTime);
void drawBoxTop();
void drawBoxBottom();
void drawChooching();
void drawStart();
void drawPauses(uint16_t remainingTime);
void drawInit1();
void drawInit2();
void drawPanic();
#endif
