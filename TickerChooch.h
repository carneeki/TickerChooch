#ifndef TICKERCHOOCH_H
#define TICKERCHOOCH_H

// TODO: Change all these values to suit

// NOTE: pins 4, 5, 6, 7, 8, 9, A0 reserved for LCD screen

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

#define KNOB_DIST       A1   // Wiper for distance knob
#define KNOB_PAUSE      A2   // Wiper for pause knob
#define KNOB_SPEED      A3   // Wiper for speed knob
#define SPEED_MAX     1023   // Maximum speed (as analogRead())
#define SPEED_INCR      20   // # of increments for speed
#define DIST_MAX      1500   // Maximum distance (mm)
#define DIST_INCR       20   // # of increments for distance
#define PAUSE_MAX       90   // Maximum pause time (seconds)
#define PAUSE_INCR       1   // # of increments for pause time

#define STATE_INIT       1   // initialisation state - not much really happens
#define STATE_DATA       2   // data entry and ready state
#define STATE_RUN        3   // run state - carriage moves
#define STATE_PAUSE      4   // pause state between movements

#define UPDATE_TIME     50   // number of millis() between screen updates

uint16_t calcSteps(uint16_t);
uint16_t getDistance();
uint16_t getPauseTime();
uint16_t getSpeed();
void getDir();
void chooch();
void pause();
void eStop();
void startPause();

#endif
