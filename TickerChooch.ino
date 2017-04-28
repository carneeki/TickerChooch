/*******************************************************************************
 * TickerChooch                                                                *
 *                                                                             *
 * Makes a stepper motor drive along a track for a preprogrammed distance with *
 * pauses for given times. Speed is set in configuration. Details are          *
 * displayed on an LCD panel.                                                  *
 *                                                                             *
 * By Adam Carmichael <carneeki@carneeki.net>                                  *
 * For Emma's Spare Room Machine Shop <missemmajade@gmail.com>                 *
 *                                                                             *
 ******************************************************************************/
#include <Stepper.h>
#include <limits.h>
#include <stdlib.h>
#include "font.h"
#include "PCD8544.h"
#include "drawLCD.h"

// TODO: Change all these values to suit

// pins 2,3,4,5,6,7 used for LCD screen
#define STEP_AP          8   // Stepper pin A+
#define STEP_AN          9   // Stepper pin A-
#define STEP_BP         10   // Stepper pin B+
#define STEP_BN         11   // Stepper pin B-
#define STEPS_PER_REV 2048   // Number of steps to make one complete revolution of
                             // the motor shaft

#define SPEED         1000   // Speed in mm per minute
#define DIST_PER_REV    62.5 // Distance travelled for 1 rev of stepper shaft

#define SW_LIMIT        18   // Limit switch pin is pulled HIGH when at limit
                             // also - use pin 2 or 3 so there's an interrupt
#define SW_DIR_FWD      12   // Forwards direction is pulled HIGH for forward
#define SW_DIR_REV      13   // Reverse direction is pulled HIGH for reverse
#define SW_START        19   // Start / stop push button

#define KNOB_DIST       A0   // Wiper for distance knob
#define KNOB_PAUSE      A1   // Wiper for pause knob
#define KNOB_SPEED      A2   // Wiper for speed knob
#define DIST_MAX      1500   // Maximum distance (mm)
#define DIST_INCR       20   // # of increments for distance
#define PAUSE_MAX       90   // Maximum pause time (seconds)
#define PAUSE_INCR       1   // # of increments for pause time

#define STATE_INIT       1   // initialisation state - not much really happens
#define STATE_DATA       2   // data entry and ready state
#define STATE_RUN        3   // run state - carriage moves
#define STATE_PAUSE      4   // pause state between movements

#define UPDATE_TIME     50   // number of millis() between screen updates

// motor object
Stepper motor(STEPS_PER_REV, STEP_AP, STEP_AN, STEP_BP, STEP_BN);

// data entered vars
int distance       = 0;  // configured distance to travel (in mm)
int pauseTime      = 0;  // configured pause time (in seconds)
int speed          = 0;  // configured speed (in mm / minute)
bool run           = false; // direction switch is not neutral
bool fwd           = true;  // fwd = true, rev = false

// chooch() cycle vars
int remainingSteps = 0;  // steps to complete in this chooch() cycle
int remainingDist  = 0;  // remaining distance in this chooch() cycle

// pause() cycle vars
int remainingTime  = 0;  //  remaining seconds in this pause() cycle

// state machine vars
int currentState   = 0;  // current state for state machine
int nextState      = 0;  // state to jump to

// working and scratch values
#ifndef SCRATCH
#define SCRATCH
char buf[17]       = ""; // buffer for text to go to LCD
unsigned long prev = 0;  // previous time the loop() ran
bool doPaint       = false; // update the LCD screen?
#endif

/**
 * Calculate the number of steps required to travel a given distance
 */
int calcSteps(int mm)
{
  return (mm / DIST_PER_REV) * STEPS_PER_REV;
}

/**
 * Get the distance from the distance knob, but round it
 */
int getDistance()
{
  int roundedDist = analogRead(KNOB_DIST) / (1023/DIST_INCR);
  return map( roundedDist, 0, DIST_INCR, 0, DIST_MAX);
}

/**
 * Get the pause time from the pause time knob, but round it
 */
int getPauseTime()
{
  int roundedPause = analogRead(KNOB_PAUSE) / (1023/PAUSE_INCR);
  return map( roundedPause, 0, PAUSE_INCR, 0, PAUSE_MAX);
}

/**
 * get the state of the direction switch, if reverse or fwds is high, then run
 * should be high. if both are low, then run should be false
 */
bool getRun()
{
  if( digitalRead(SW_DIR_FWD) || digitalRead(SW_DIR_REV))
    run = true;
  else
    run = false;

  return run;
}

bool getDir()
{
  if(digitalRead(SW_DIR_FWD))
    fwd = true;

  if(digitalRead(SW_DIR_REV))
    fwd = false;

  return fwd;
}

/**
 * Wrapper for Stepper.step(int) because it is blocking and won't
 * allow use of limit switches (well maybe an ISR could be used?)
 */
void chooch()
{
  if( remainingSteps >= 1 && !digitalRead(SW_LIMIT) )
  {
    motor.step(10);
    remainingSteps -= 10;
  }

  // prevent remainingSteps from being less than 0
  if(remainingSteps < 0)
    remainingSteps = 0;

  // if limit switch is hit, update state machine to go to [STATE_DATA]
  if(digitalRead(SW_LIMIT))
    nextState = STATE_DATA;
  else
    nextState = STATE_PAUSE;
}

/**
 * Wrapper for delay(int). Also permit limit switches to break the cycle.
 */
void pause()
{
  if( remainingTime >= 1 && !digitalRead(SW_LIMIT) )
  {
    delay(1000);
    remainingTime--;
  }

  // prevent remainingTime from being less than 0
  if(remainingTime < 0)
    remainingTime = 0;

  // if limit switch is hit, update state machine to go to [read data]
  if(digitalRead(SW_LIMIT))
    nextState = STATE_DATA;
  else
    nextState = STATE_RUN;
}
void setup()
{
  // attach the limit switch to an interrupt so we can pause
  // when the carriage reaches the end of the rail
  pinMode(SW_LIMIT, INPUT);
  attachInterrupt(digitalPinToInterrupt(SW_LIMIT), eStop, RISING);

  // attach the start/pause switch to an interrupt so we can pause / start
  pinMode(SW_START, INPUT);
  attachInterrupt(digitalPinToInterrupt(SW_PAUSE), startPause, RISING);

  // set up other switch inputs
  pinMode(SW_DIR_FWD, INPUT);
  pinMode(SW_DIR_REV, INPUT);

  LcdInitialise();
  LcdClear();
}

void loop()
{
  // should I paint the screen?
  if(millis() - prev > UPDATE_TIME )
  {
    doPaint = true;
    prev = millis();
  }
  else
    doPaint = false;

  /* State machine:
   *  [STATE_INIT] -> [STATE_DATA] -> [STATE_RUN] <-> [STATE_PAUSE]
   *
   * If a limit switch is hit in [STATE_RUN] or [STATE_PAUSE], then next state
   * is set to [STATE_DATA].
   */
  switch(nextState)
  {
    if(doPaint)
      LcdClear();

    case STATE_INIT:
      nextState = STATE_DATA;

      if(doPaint)
      {
        drawInit1();
        delay(5000);

        drawInit2();
        delay(2000);
      }
    break;

    case STATE_DATA:
      distance = getDistance();
      pauseTime = getPauseTime();
      remainingSteps = calcSteps(distance);

      if(doPaint)
      {

        drawSpeed();
        drawPause();
        drawDistance();
        drawBoxTop();

        drawStart();

        drawBoxBottom();
      }

      if(digitalRead(SW_START))
        nextState = STATE_RUN;
    break;

    case STATE_RUN:
      if(doPaint)
      {
        drawSpeed();
        drawPause();
        drawDistance();
        drawBoxTop();

        drawChooching();

        drawBoxBottom();
      }

      chooch(); // chooch is blocking
    break;

    case STATE_PAUSE:
      if(doPaint)
      {
        drawSpeed();
        drawPause();
        drawDistance();
        drawBoxTop();

        drawPauses();

        drawBoxBottom();
      }

      pause(); // pause is blocking (contains delay())
    break;

    default:
      // we shouldn't ever get to this point. enter AvE mode.
      // panic. call for help.
      drawPanic();
      delay(INT_MAX);
    break;
  }
}

void eStop()
{
  nextState = STATE_DATA;
}

void startPause()
{
  // if currently running, go to data entry, else, we probably want to
  // go to run state
  nextState = (nextState == STATE_RUN)? STATE_DATA : STATE_RUN;
}


void drawSpeed()
{
  strcpy(buf, "SPEED: ");

  strcat(buf, (const char*) ((fwd)? 0xaf : 0xae) );
  strcat(buf, " ");

  for(int i = 0; i< (analogRead(KNOB_SPEED)+1)/204; i++)
    strcat(buf, (const char*) 0xdb);

  LcdString(buf);
}

void drawDistance()
{
  strcpy(buf, " DIST:    ");

  // 3 digits long
  if(distance < 1000 && distance >= 100)
    itoa(distance, &buf[7], 10); // FIXME: Len would be appalled

  // 2 digits long
  if(distance <  100 && distance >=  10)
    itoa(distance, &buf[8], 10); // FIXME and Pong

  // 1 digit long
  if(distance <   10 && distance >=   0)
    itoa(distance, &buf[9], 10); // FIXME and Mitchell

  strcat(buf, " mm  ");

  LcdString(buf);
}

void drawPause()
{
  String strOut = "PAUSE: ";
  char buf[] ="";

  if(pauseTime < 1000)
    strOut += " ";

  if(pauseTime <  100)
    strOut += " ";

  if(pauseTime <   10)
    strOut += " ";

  strOut += pauseTime + " s   ";

  strOut.toCharArray(buf,sizeof(strOut));

  LcdString(buf);
}

void drawBoxTop()
{
  char buf[] = "╔══════════════╗";
  LcdString(buf);
}
void drawBoxBottom()
{
  char buf[] = "╚══════════════╝";
  LcdString(buf);
}

void drawChooching()
{
  char buf[] = "║  CHOOCHING!  ║";
  LcdString(buf);
}

void drawStart()
{
  char buf[] = "║ PRESS  START ║";
  LcdString(buf);
}

void drawPauses()
{
  String strOut = "║ PAUSED: ";
  char buf[] ="";

  if(remainingTime < 100)
    strOut += " ";

  if(remainingTime <  10)
    strOut += " ";

  strOut += remainingTime + " s║";

  strOut.toCharArray(buf,sizeof(strOut));

  LcdString(buf);
}

void drawInit1()
{
  for(unsigned int i=0; i< sizeof(bm0); i++)
    LcdWrite(LCD_D, pgm_read_byte_near(bm1[i]));
}

void drawInit2()
{
  for(unsigned int i=0; i< sizeof(bm0); i++)
    LcdWrite(LCD_D, pgm_read_byte_near(bm0[i]));
}

void drawPanic()
{
  char buf[] ="";
  drawBoxTop();

  strcpy(buf, "║              ║");
  LcdString(buf);

  strcpy(buf, "║    PANIC!    ║");
  LcdString(buf);

  strcpy(buf,"║      :(      ║");
  LcdString(buf);

  strcpy(buf,"║              ║");
  LcdString(buf);

  drawBoxBottom();
}
