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

// TODO: Change all these values to suit

// pins 3,4,5,6,7 used for LCD screen
#define STEP_AP 8          // Stepper pin A+
#define STEP_AN 9          // Stepper pin A-
#define STEP_BP 10         // Stepper pin B+
#define STEP_BN 11         // Stepper pin B-
#define STEPS_PER_REV 2048 // Number of steps to make one complete revolution of
                           // the motor shaft

#define SPEED 1000         // Speed in mm per minute
#define DIST_PER_REV 62.5  // Distance travelled for 1 rev of stepper shaft

#define SW_LIMIT   11      // Limit switch pin is pulled HIGH when at limit
#define SW_DIR_FWD 12      // Forwards direction is pulled HIGH for forward
#define SW_DIR_REV 13      // Reverse direction is pulled HIGH for reverse
#define SW_START    1      // Start / stop push button

#define KNOB_DIST  A0      // Wiper for distance knob
#define KNOB_PAUSE A1      // Wiper for pause knob
#define DIST_MAX  1500     // Maximum distance (mm)
#define DIST_INCR   20     // # of increments for distance
#define PAUSE_MAX   90     // Maximum pause time (seconds)
#define PAUSE_INCR   1     // # of increments for pause time

#define STATE_INIT  1
#define STATE_DATA  2
#define STATE_RUN   3
#define STATE_PAUSE 4

// motor object
Stepper motor(STEPS, STEP_AP, STEP_AN, STEP_BP, STEP_BN);

// data entered vars
int distance     = 0;   // configured distance to travel (in mm)
int pauseTime    = 0;   // configured pause time (in seconds)

// chooch() cycle vars
int remainingSteps = 0; // steps to complete in this chooch() cycle
int remainingDist  = 0; // remaining distance in this chooch() cycle

// pause() cycle vars
int remainingTime = 0;  //  remaining seconds in this pause() cycle

// state machine vars
int currentState = 0;   // current state for state machine
int nextState    = 0;   // state to jump to

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
 * Prepare a screen update
 */
void screen()
{

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

void loop()
{
  /* State machine:
   *  [STATE_INIT] -> [STATE_DATA] -> [STATE_RUN] <-> [STATE_PAUSE]
   *
   * If a limit switch is hit in [STATE_RUN] or [STATE_PAUSE], then next state
   * is set to [STATE_DATA].
   */
  switch(nextState)
  {
    case STATE_INIT:
      nextState = STATE_DATA;
    break;

    case STATE_DATA:
      distance = getDistance();
      pauseTime = getPauseTime();
      remainingSteps = calcSteps(distance);
    break;

    case STATE_RUN:
      chooch(); // chooch is blocking
    break;

    case STATE_PAUSE:
      pause(); // pause is blocking (contains delay())
    break;

    default:
      // we shouldn't ever get to this point. enter AvE mode.
      // panic. call for help.
    break;
  }

  screen();
}
