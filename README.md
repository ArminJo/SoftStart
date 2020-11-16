# SoftStart / TRIACRamp library

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Build Status](https://github.com/ArminJo/SoftStart/workflows/LibraryBuild/badge.svg)](https://github.com/ArminJo/SoftStart/actions)
[![Hit Counter](https://hitcounter.pythonanywhere.com/count/tag.svg?url=https%3A%2F%2Fgithub.com%2FArminJo%2FSoftStart)](https://github.com/brentvollebregt/hit-counter)

Generates TRIAC control pulse for soft start of motors used in circular saw, angle grinder and other DIY tools.

### Implemented and tested for AVR-ATtiny85

### Schematics of a embedded motor soft-start
![Eagle schematics](https://github.com/ArminJo/SoftStart/blob/master/extras/Softstart.png)<br/>
The eagle file is [here](https://github.com/ArminJo/SoftStart/blob/master/extras).
Instead of 330 ohm at PB0 you can use a series of 120 ohm with a 220 ohm with 10 nF parallel.

### After calling startRamp():
- Wait for 8 zero crossings of zero crossing interrupt to synchronize timing and check for right frequency (50 or 60 Hz).
- Output TRIAC pulse and decrease pulse delay from `START_PHASE_SHIFT_DEGREES` to `MINIMUM_PHASE_SHIFT_COUNT` (0 degree) at every voltage zero crossing.
- After delay gets zero, output TRIAC pulse pulse at zero crossing of AC line. The multi pulse (3*350 micro seconds) will cover small delays of current zero crossing.

# Settings for the SoftStart example
If `LOAD_ON_OFF_DETECTION` is defined, the program does not start with ramp at boot up time, but waits for interrupt at LoadDetectionInput (pin 6). This is useful, if you want to build an plug in soft start adapter. It only starts working when the attached device is switched on, e.g. a load is detected. 

FUSE VALUES for embedded version, which requires fast start, since soft start must begin as soon as power is on.
- Low=0X52 Int RC Osc. 8 MHz divided by 8 (default). 14 Clk + 4 ms startup (for fast startup).
- High=0XDC BrowOut at VCC=4.3 volt
- Extended=0XFF (default)

FUSE VALUES for `LOAD_ON_OFF_DETECTION` defined, which means that CPU power is always on.
You may use the default values or enable additional Brown-out detection eg. at 4.3 volt.
- Low=0X62 (default) Int RC Osc. 8 MHz divided by 8. 14 Clk + 64 ms startup.
- High=0XDC BrowOut at VCC=4.3 volt
- Extended=0XFF (default)

# Modifying library properties
To change internal values like the layout of the multi pulse, you have to modify the values in TRIACRamp.h.<br/>

To access the Arduino library files from a sketch, you have to first use `Sketch/Show Sketch Folder (Ctrl+K)` in the Arduino IDE.<br/>
Then navigate to the parallel `libraries` folder and select the library you want to access.<br/>
The library files itself are located in the `src` sub-directory.<br/>
If you did not yet store the example as your own sketch, then with Ctrl+K you are instantly in the right library folder.
## Consider to use [Sloeber](http://eclipse.baeyens.it/stable.php?OS=Windows) as IDE
If you are using Sloeber as your IDE, you can easily define global symbols at *Properties/Arduino/CompileOptions*.<br/>
![Sloeber settings](https://github.com/ArminJo/ServoEasing/blob/master/pictures/SloeberDefineSymbols.png)

## Calibration mode
Calibration mode outputs actual phase counter forever (at 115200 Baud (@1MHZ) at pin 6 / PB1) in order to adjust the 50% duty cycle trimmer.
- Output format: `<counterForPositiveHalfWave>|<counterForNegativeHalfWave>\n`

