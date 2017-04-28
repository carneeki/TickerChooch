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
#include "TinkerChooch.h"
#include "font.h"
#include "LCD4884.h"
#include "drawLCD.h"

// motor object
Stepper motor(STEPS_PER_REV, STEP_AP, STEP_AN, STEP_BP, STEP_BN);

// data entered vars
uint16_t distance       = 0;  // configured distance to travel (in mm)
uint16_t pauseTime      = 0;  // configured pause time (in seconds)
uint16_t speed          = 0;  // configured speed (in mm / minute)
bool run           = false; // direction switch is not neutral
bool fwd           = true;  // fwd = true, rev = false

// chooch() cycle vars
uint16_t remainingSteps = 0;  // steps to complete in this chooch() cycle
uint16_t remainingDist  = 0;  // remaining distance in this chooch() cycle

// pause() cycle vars
uint16_t remainingTime  = 0;  //  remaining seconds in this pause() cycle

// state machine vars
uint8_t nextState      = STATE_INIT;  // state to jump to

// working and scratch values
#ifndef SCRATCH
#define SCRATCH
unsigned long prev = 0;  // previous time the loop() ran
bool doPaint       = false; // update the LCD screen?
#endif

/**
 * Calculate the number of steps required to travel a given distance
 */
uint16_t calcSteps(uint16_t mm)
{
  return (mm / DIST_PER_REV) * STEPS_PER_REV;
}

/**
 * Get the distance from the distance knob, but round it
 */
uint16_t getDistance()
{
  int roundedDist = analogRead(KNOB_DIST) / (1023/DIST_INCR);
  return map( roundedDist, 0, DIST_INCR, 0, DIST_MAX);
}

/**
 * Get the pause time from the pause time knob, but round it
 */
uint16_t getPauseTime()
{
  int roundedPause = analogRead(KNOB_PAUSE) / (1023/PAUSE_INCR);
  return map( roundedPause, 0, PAUSE_INCR, 0, PAUSE_MAX);
}

uint16_t getSpeed()
{
  uint16_t roundedSpeed = analogRead(KNOB_SPEED)/(1023/SPEED_INCR);
  return map( roundedSpeed, 0, SPEED_INCR, 0, SPEED_MAX);
}

/**
 * get the state of the direction switch, if reverse or fwds is high, then run
 * should be high. if both are low, then run should be false
 */
bool getRun()
{
  run = ( digitalRead(SW_DIR_FWD) || digitalRead(SW_DIR_REV))?
    true : false;

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

  // if limit switch is hit, update state machine to go to [read data]
  if(digitalRead(SW_LIMIT))
    nextState = STATE_DATA;
  else
    nextState = STATE_RUN;
}
void setup()
{
  // TODO: remove me after debugging
  Serial.begin(115200);
  Serial.println("Rebooty party");

  // attach the limit switch to an interrupt so we can pause
  // when the carriage reaches the end of the rail
  pinMode(SW_LIMIT, INPUT);
  attachInterrupt(digitalPinToInterrupt(SW_LIMIT), eStop, RISING);

  // attach the start/pause switch to an interrupt so we can pause / start
  pinMode(SW_START, INPUT);
  attachInterrupt(digitalPinToInterrupt(SW_START), startPause, RISING);

  // set up other switch inputs
  pinMode(SW_DIR_FWD, INPUT);
  pinMode(SW_DIR_REV, INPUT);

  lcd.init();
  lcd.clear();
}

void loop()
{
  // should I paint the screen?
  if(millis() - prev > UPDATE_TIME )
  {
    doPaint = true;
    prev = millis();

    Serial.print("nextState: ");
    Serial.println(nextState);
  }
  else
    doPaint = false;

  /* State machine:
   *  [STATE_INIT] -> [STATE_DATA] -> [STATE_RUN] <-> [STATE_PAUSE]
   *
   * If a limit switch is hit in [STATE_RUN] or [STATE_PAUSE], then next state
   * is set to [STATE_DATA].
   */
  if(nextState == STATE_INIT)
  {
    nextState = STATE_DATA;

    if(doPaint)
    {
      lcd.clear();
      drawInit1();
      delay(5000);
      drawInit2();
      delay(2000);
    }
  } else if(nextState == STATE_DATA)
  {
    if(doPaint)
    {
      getSpeed();
      remainingSteps = calcSteps(distance);

      lcd.clear();

      drawSpeed(fwd, analogRead(KNOB_SPEED));
      drawPause(getPauseTime());
      drawDistance(getDistance());
      drawBoxTop();

      drawStart();

      drawBoxBottom();
    }
  } else if(nextState == STATE_RUN)
  {
    if(doPaint)
    {
      lcd.clear();

      drawSpeed(fwd, speed);;
      drawPause(getPauseTime());
      drawDistance(getDistance());
      drawBoxTop();

      drawChooching();

      drawBoxBottom();
    }

    chooch(); // chooch is blocking
  } else if(nextState == STATE_PAUSE)
  {
    if(doPaint)
    {
      lcd.clear();

      drawSpeed(fwd, analogRead(KNOB_SPEED));;
      drawPause(pauseTime);
      drawDistance(distance);
      drawBoxTop();

      drawPauses(remainingTime);

      drawBoxBottom();
    }

    pause(); // pause is blocking (contains delay())
  }
  else
  {
    // we shouldn't ever get to this point. enter AvE mode.
    // panic. call for help.
    lcd.clear();
    drawPanic();
    delay(INT_MAX);
  }
}

void eStop()
{
  nextState = STATE_DATA;
  Serial.println("estop()");
}

void startPause()
{
  if(nextState == STATE_DATA)
    nextState = STATE_PAUSE;

  if(nextState == STATE_RUN)
    nextState = STATE_DATA;

  Serial.println("startPause()");
}
