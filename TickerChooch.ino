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
#include <limits.h>
#include <stdlib.h>
#include <LiquidCrystal.h>
#include "TickerChooch.h"
#include "X113647Stepper.h"

// data entered vars
uint16_t k_distance       = 0;  // distance knob value
uint16_t k_pauseTime      = 0;  // pause time knob value
uint16_t k_speed          = 0;  // speed knob value

uint16_t distance         = 0;  // distance per chooch() cycle (mm)
uint16_t pauseTime        = 0;  // pause time per chooch() (seconds)
uint16_t speed            = 0;  // speed (mm / minute)

bool run                  = false; // direction switch is not neutral
bool fwd                  = true;  // fwd = true, rev = false

// chooch() cycle vars
uint16_t remainingSteps   = 0;  // steps to complete in this chooch() cycle
uint16_t remainingDist    = 0;  // remaining distance in this chooch() cycle
uint16_t remainingTime    = 0;  // remaining seconds in this pause() cycle

// state machine vars
uint8_t nextState         = STATE_INIT;  // state to jump to

// working and scratch values
unsigned long prev        = 0;  // previous time the loop() ran
bool doPaint              = false; // update the LCD screen?

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
X113647Stepper motor(STEPS_PER_REV, STEP_AP, STEP_AN, STEP_BP, STEP_BN);

void setup()
{
  lcd.begin(16, 2);
  lcd.print("Rebooty party");

  // TODO: remove me after debugging
  Serial.begin(115200);
  Serial.println("Rebooty party");

  delay(2000);

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

  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
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
    if(doPaint)
    {
      lcd.clear();

      lcd.print("Please work!");
      delay(2000);
      lcd.print("Did it work?");
      delay(2000);
    }
    nextState = STATE_DATA;
  } else if(nextState == STATE_DATA)
  {
    if(doPaint)
    {
      lcd.clear();
      lcd.print("STATE_DATA");

      getSpeed();
      remainingSteps = calcSteps(distance);

      //drawSpeed(fwd, analogRead(KNOB_SPEED));
      //drawPause(getPauseTime());
      //drawDistance(getDistance());
      //drawBoxTop();

      //drawStart();

      //drawBoxBottom();
    }
  } else if(nextState == STATE_RUN)
  {
    if(doPaint)
    {
      lcd.clear();
      //drawSpeed(fwd, speed);;
      //drawPause(getPauseTime());
      //drawDistance(getDistance());
      //drawBoxTop();

      //drawChooching();

      //drawBoxBottom();
    }

    chooch(calcSteps(distance)); // chooch is blocking
  } else if(nextState == STATE_PAUSE)
  {
    if(doPaint)
    {
      lcd.clear();
      //drawSpeed(fwd, analogRead(KNOB_SPEED));;
      //drawPause(pauseTime);
      //drawDistance(distance);
      //drawBoxTop();

      //drawPauses(remainingTime);

      //drawBoxBottom();
    }

    pause(); // pause is blocking (contains delay())
  }
  else
  {
    // we shouldn't ever get to this point. enter AvE mode.
    // panic. call for help.
    lcd.clear();
    //drawPanic();
    delay(INT_MAX);
  }
}

/**
 * Calculate the number of steps required to travel a given distance
 */
uint16_t calcSteps(uint16_t mm)
{
  return (mm / DIST_PER_REV) * STEPS_PER_REV;
}

/**
 * calculate the RPMs for X113647Stepper.setSpeed();
 * Include direction component
 */
int16_t calcRpm(uint16_t sp)
{
  return (sp / DIST_PER_REV) * (fwd?1:-1);
}

/**
 * Get the distance from the distance knob, but round it
 */
uint16_t getDistance()
{
  k_distance = analogRead(KNOB_DIST);

  distance = k_distance / (1023/DIST_INCR); // round to nearest increment
  distance = map( distance, 0, DIST_INCR, 0, DIST_MAX);

  return distance;
}

/**
 * Get the pause time from the pause time knob, but round it
 */
uint16_t getPauseTime()
{
  k_pauseTime = analogRead(KNOB_PAUSE);

  pauseTime = k_pauseTime / (1023/PAUSE_INCR); // round to nearest increment
  pauseTime = map( pauseTime, 0, PAUSE_INCR, 0, PAUSE_MAX);

  return pauseTime;
}

uint16_t getSpeed()
{
  k_speed = analogRead(KNOB_SPEED);
  speed = k_speed/(1023/SPEED_INCR); // round to nearest increment
  speed = map( speed, 0, SPEED_INCR, 0, SPEED_MAX);
  return speed;
}

/**
 * read direction switch and set run + direction vars
 */
void getDir()
{
  /*
   * Truth table:
   *
   *   Inputs    Outputs
   *  FWD  REV   run   fwd
   *    0    0     0     0
   *    0    1     1     0
   *    1    0     1     1
   *    1    1     0     0
   *
   *  run = FWD xor  REV;
   *  fwd = FWD and !rev;
   */
  run = digitalRead(SW_DIR_FWD) ^  digitalRead(SW_DIR_REV);
  fwd = digitalRead(SW_DIR_FWD) & !digitalRead(SW_DIR_REV);
}

/**
 * Wrapper for X113647Stepper.step()
 */
void chooch(uint16_t steps)
{
  motor.setSpeed(calcRpm(speed));
  motor.step(steps);

  nextState = STATE_PAUSE;
}

/**
 * Wrapper for delay(int)
 */
void pause()
{
  if( remainingTime >= 1 )
  {
    delay(1000);
    remainingTime--;
  }

  nextState = STATE_RUN;
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
