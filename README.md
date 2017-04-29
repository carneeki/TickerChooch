# TickerChooch

Makes a stepper motor drive along a track for a preprogrammed distance with
pauses for given times. Speed is set in configuration. Details are displayed on
an LCD panel.

By Adam Carmichael <carneeki@carneeki.net>
For Emma's Spare Room Machine Shop <missemmajade@gmail.com>

## State Machine
There is some state machine logic in the loop() code. The states are:
* STATE_INIT  - machine is booting up
* STATE_DATA  - machine is accepting data and ready to go
* STATE_RUN   - machine is currently moving
* STATE_PAUSE - machine is paused between movements

While in STATE_RUN or STATE_PAUSE, if a limit switch is pressed, the machine
returns to STATE_DATA and is ready for data entry.

## What the LCD shows for each state:

### Accepting data:
```
/----------------\
|D: xxxx  P: xxxx|
|S: xxxx      RDY|
\----------------/
```
Speed will have a `-` symbol to indicate direction is reversed if the direction
switch is set to reverse.


## Running:
```
/----------------\
|D: xxxx   P: xxx|
|S: xxxx      RUN|
\----------------/
```

## Paused
`P: ` shows remaining time
```
/----------------\
|D: xxxx   P: xxx|
|S: xxxx      PAU|
\----------------/

 D:    0   P:   0
 S:    0      RDY
 D:    0   P:     0
 S:    0      RDY
 D:    0   P:     0

```

## Panic
When the state machine enters an unknown state
```
/----------------\
|     PANIC!     |
|       :(       |
\----------------/
```
