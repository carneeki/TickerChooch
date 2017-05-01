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
int16_t speed             = 0;  // speed (mm / minute)

volatile bool run                  = false; // direction switch is not neutral
volatile bool fwd                  = true;  // fwd = true, rev = false

// chooch() cycle vars
uint16_t remainingSteps   = 0;  // steps to complete in this chooch() cycle
uint16_t remainingDist    = 0;  // remaining distance in this chooch() cycle
volatile uint16_t reStartTime      = 0;  // time (in millis) pause ends
volatile uint16_t debounce         = 0;  // millis since button press
char isrBuf[255] = "";

// state machine vars
volatile uint8_t nextState         = STATE_INIT;  // state to jump to

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

  // TODO: I am here for testing
  pinMode(23, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(43, OUTPUT);
  pinMode(53, OUTPUT);
  digitalWrite(23, LOW);
  digitalWrite(33, LOW);
  digitalWrite(43, LOW);
  digitalWrite(53, HIGH);

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
  // show any messages from ISRs
  if(strcmp(isrBuf,"")!=0)
  {
    Serial.println(isrBuf);
    memcpy(isrBuf,"",255);
  }

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
  if(nextState == STATE_INIT)
  {
    if(doPaint)
    {
      paintInit();
      delay(2000);
    }

    nextState = STATE_DATA;

  } // end STATE_INIT
  else if(nextState == STATE_DATA)
  {
    getDir();
    getSpeed();
    getPauseTime();
    getDistance();
    remainingSteps = calcSteps(distance);

    if(doPaint)
      paintData();

  } // end STATE_DATA
  else if(nextState == STATE_RUN)
  {
    if(doPaint)
      paintRun();

    if(remainingSteps - STEPS_PER_CHOOCH > 0 )
      chooch(STEPS_PER_CHOOCH);
    else
    {
      chooch(remainingSteps);
      reStartTime = millis() + (1000*pauseTime);
      nextState = STATE_PAUSE;
    }
  } // end STATE_RUN
  else if(nextState == STATE_PAUSE)
  {
    if(millis() >= reStartTime)
    {
      nextState = STATE_RUN;
      Serial.println("setting state to STATE_RUN");
      return;
    }
    if(doPaint)
      paintPause();
  } // end STATE_PAUSE
  else
  {
    // we shouldn't ever get to this point. enter AvE mode.
    // panic. call for help.
    paintPanic();
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
  uint16_t tmp = 0; // temporary var

  k_distance = analogRead(KNOB_DIST);

  tmp = k_distance / (1023/DIST_INCR); // round to nearest increment
  distance = map( tmp, 0, DIST_INCR, 0, DIST_MAX);

  return distance;
}

/**
 * Get the pause time from the pause time knob, but round it
 */
uint16_t getPauseTime()
{
  uint16_t tmp = 0; // temporary var
  k_pauseTime = analogRead(KNOB_PAUSE);

  tmp = k_pauseTime / (1023/PAUSE_INCR); // round to nearest increment
  pauseTime = map( tmp, 0, PAUSE_INCR, 0, PAUSE_MAX);

  // TODO: remove me after debugging
  pauseTime = 10;

  return pauseTime;
}

int16_t getSpeed()
{
  int16_t tmp = 0; // temporary var for speed

  k_speed = analogRead(KNOB_SPEED);
  tmp = k_speed/(1023/SPEED_INCR); // round to nearest increment
  speed = map( tmp, 0, SPEED_INCR, 0, SPEED_MAX) * (fwd?1:-1);

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
}

void eStop()
{
  // ignore bouncey switches
  if(debounce >= millis() - DEBOUNCE_TIME)
    return;

  debounce = millis();

  nextState = STATE_DATA;
  memcpy(isrBuf,"estop(): Entering ready state...",255);
}

void startPause()
{
  // ignore bouncey switches
  if(debounce >= millis() - DEBOUNCE_TIME)
    return;

  noInterrupts();
  debounce = millis();

  if(nextState == STATE_DATA)
  {
    nextState = STATE_PAUSE;
    reStartTime = millis() + (1000 * pauseTime);
    memcpy(isrBuf,"startPause(): Entering pause...",255);
  }
  else if(nextState == STATE_RUN)
  {
    memcpy(isrBuf,"startPause(): Entering ready state...",255);
    nextState = STATE_DATA;
  }
  interrupts();
}

void paintInit()
{
  char buf[17] = "";
  strcpy(buf, "Initialising...");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(buf);
  Serial.println(buf);
}

void paintData()
{
  char buf[17] = "";
  strcpy(buf, "D:");
  strcat(buf, pad(distance));
  strcat(buf, "  P:");
  strcat(buf, pad(pauseTime));
  lcd.setCursor(0,0);
  lcd.print(buf);
  Serial.println(buf);

  strcpy(buf, "S:");
  strcat(buf, pad(speed));
  strcat(buf, "      ");
  strcat(buf, "RDY");
  lcd.setCursor(0,1);
  lcd.print(buf);
  Serial.println(buf);
}

void paintRun()
{
  char buf[17] = "";
  strcpy(buf, "D:");
  strcat(buf, pad(distance));
  strcat(buf, "  P:");
  strcat(buf, pad(pauseTime));
  lcd.setCursor(0,0);
  lcd.print(buf);
  Serial.println(buf);

  strcpy(buf, "S:");
  strcat(buf, pad(speed));
  strcat(buf, "      ");
  strcat(buf, "RUN");
  lcd.setCursor(0,1);
  lcd.print(buf);
  Serial.println(buf);
}

void paintPause()
{
  char buf[17] = "";
  strcpy(buf, "D:");
  strcat(buf, pad(distance));
  strcat(buf, "  P:");
  strcat(buf, pad( (int) (reStartTime-millis())/1000 ));
  lcd.setCursor(0,0);
  lcd.print(buf);
  Serial.println(buf);

  strcpy(buf, "S:");
  strcat(buf, pad(speed));
  strcat(buf, "      ");
  strcat(buf, "PAU");
  lcd.setCursor(0,1);
  lcd.print(buf);
  Serial.println(buf);
}

void paintPanic()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("     PANIC!     ");
  lcd.setCursor(0,1);
  lcd.print("       :(       ");
}

char* pad(uint16_t data)
{
  char buf[8] = "";
  char tmp[8] = "";
  if(data < 10000)  // 4 digit numbers
    strcat(buf, " ");
  if(data <  1000)  // 3 digit numbers
    strcat(buf, " ");
  if(data <   100)  // 2 digit numbers
    strcat(buf, " ");
  if(data <    10)  // 1 digit numbers
    strcat(buf, " ");

  return strcat(buf, itoa(data, tmp, 10));
}
