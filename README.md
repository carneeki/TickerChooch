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
### Boot up
You'll just have to wait and see!

### Accepting data:
```
SPEED: »████████
 DIST: 1000 mm
PAUSE:   60 s
╔══════════════╗
║ PRESS  START ║
╚══════════════╝
```
Machine is accepting data. The '»' character indicates the carriage will move
to the right at a speed indicated by the '█' characters. Movement to the left
is indicated by a '«' character.

Upon pushing start, the machine will enter the paused state to give time to
clear the carriage.

### During movements
```
SPEED: »████████
 DIST: 1000 mm
PAUSE:   60 s
╔══════════════╗
║  CHOOCHING!  ║
╚══════════════╝
```
Future revisions may include a counter to show how much travel is remaining
each cycle.

### When paused
```
SPEED: »████████
  DIST: 1000 mm
 PAUSE:   60 s
╔══════════════╗
║ PAUSED:  03 s║
╚══════════════╝
```
Note that the paused timer counts down the seconds until movement again.


### If the machine enters an unknown state.
```
╔══════════════╗
║              ║
║    PANIC!    ║
║      :(      ║
║              ║
╚══════════════╝
```
