# SoftStart / TRIACRamp library

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Build Status](https://github.com/ArminJo/SoftStart/workflows/LibraryBuild/badge.svg)](https://github.com/ArminJo/SoftStart/actions)
![Hit Counter](https://visitor-badge.laobi.icu/badge?page_id=ArminJo_SoftStart)

Generates TRIAC control pulse for soft start of motors used in circular saw, angle grinder and other DIY tools.

### Implemented and tested for AVR-ATtiny85 @ 1MHz

# Schematics of an embedded motor soft-start
![Eagle schematics](https://github.com/ArminJo/SoftStart/blob/master/extras/SoftstartEmbedded.png)<br/>
# Schematics of a plug in module for motor soft-start
![Eagle schematics](https://github.com/ArminJo/SoftStart/blob/master/extras/SoftstartPlugIn.png)<br/>
The eagle files are [here](https://github.com/ArminJo/SoftStart/blob/master/extras).

- Instead of 330 &ohm; at PB0 you can use a series of 120 &ohm; with a 220 &ohm; with 10 nF parallel.
- The current sense circuit is only required for the plug in soft start adapter mode (`LOAD_ON_OFF_DETECTION` is enabled) to detect power disconnect. In this case the current is limited to 2 A (~ 450 W) if you using 1N4004 diodes. For more power you may use 1N5004 instead and/or use 2 diodes in parallel.

# How it works (after calling startRamp())
- Wait for 8 zero crossings of zero crossing interrupt to synchronize timing and check for right frequency (50 or 60 Hz).
- Output TRIAC pulse and decrease pulse delay from `START_PHASE_SHIFT_DEGREES` to 0 degree at every voltage zero crossing.
    The decrease amount is specified by the **ramp speed trimmer**, i.e. a voltage at pin 2.
- After delay gets zero, output TRIAC pulse pulse at zero crossing of AC line. The multi pulse (3 * 250 microseconds) will cover small delays of current zero crossing.

# Calibration mode
Calibration mode is entered, when the ADC value from the **ramp speed trimmer** is less than **4**.<br/>
This mode outputs the timer counter value forever (at 115200 Baud (@1MHZ) at pin 6 / PB1) in order to adjust the **50% duty cycle trimmer**.
Both values must be the same.
The output format is: `<counterForPositiveHalfWave>|<counterForNegativeHalfWave>\n`<br/>
You have to use an isolation transformer to safely read this value.<br/>
I have not yet tested it, but using **two 1 M&ohm; resistors instead of the ramp speed trimmer should work too**.

# Fuse values
FUSE VALUES for **embedded version**, which requires **fast start**, since soft start must begin as soon as power is on.
- Low=0X52 Int RC Osc. 8 MHz divided by 8 (default). 14 Clk + 4 ms startup (for fast startup).
- High=0XDC BrowOut at VCC=4.3 volt
- Extended=0XFF (default)

FUSE VALUES for **plug in soft start adapter**, i.e. `LOAD_ON_OFF_DETECTION` is defined, which means that CPU power is always on.
You may use the default values or enable additional Brown-out detection eg. at 4.3 volt.
- Low=0X62 (default) Int RC Osc. 8 MHz divided by 8. 14 Clk + 64 ms startup.
- High=0XDC BrowOut at VCC=4.3 volt
- Extended=0XFF (default)

# Compile options / macros for this software
To customize the software to different requirements, there are some compile options / macros available.<br/>
Modify it by commenting them out or in, or change the values if applicable. Or define the macro with the -D compiler option for global compile (the latter is not possible with the Arduino IDE, so consider using [Sloeber](https://eclipse.baeyens.it).<br/>

| Name | Default value | File | Description |
|-|-|-|-|
| `LOAD_ON_OFF_DETECTION` | enabled | SoftStart.cpp | If enabled, the program does not start with ramp at boot up time, but waits for interrupt at LoadDetectionInput (pin 6). This is useful, if you want to build an plug in soft start adapter. It only starts working when the attached device is switched on, e.g. a load is detected. |
| `START_PHASE_SHIFT_DEGREES` | 160 | TRIACRamp.h | Initial delay of TRIAC trigger impulse. Values from 0 - 180 degrees, but the extremes make no sense. |
| `TRIAC_PULSE_WIDTH_MICROS` | 250 | TRIACRamp.h | Length of trigger pulse - 100 us is too small for my circuit. |
| `TRIAC_PULSE_NUMBERS` | 250 | TRIACRamp.h | Amount of multiple trigger pulses if delay is less than total time of multiple pulses. This avoids flickering at small loads. |
| `TRIAC_PULSE_BREAK_MICROS` | 400 | TRIACRamp.h | Length of break between (multiple) trigger pulses. |

# Modifying compile options
### Changing include (*.h) files with Arduino IDE
First, use *Sketch > Show Sketch Folder (Ctrl+K)*.<br/>
If you have not yet saved the example as your own sketch, then you are instantly in the right library folder.<br/>
Otherwise you have to navigate to the parallel `libraries` folder and select the library you want to access.<br/>
In both cases the library source and include files are located in the libraries `src` directory.<br/>
The modification must be renewed for each new library version!

### Modifying compile options with Sloeber IDE
If you are using [Sloeber](https://eclipse.baeyens.it) as your IDE, you can easily define global symbols with *Properties > Arduino > CompileOptions*.<br/>
![Sloeber settings](https://github.com/ArminJo/ServoEasing/blob/master/pictures/SloeberDefineSymbols.png)

# Pictures
| | |
|-|-|
| ![ATtiny85 board](https://github.com/ArminJo/SoftStart/blob/master/extras/ATtiny85Board.jpg) | ![Triac and heat sink](https://github.com/ArminJo/SoftStart/blob/master/extras/Triac+HeatSink.jpg) |
| ATtiny85 board | Triac and heat sink |
| ![Placement](https://github.com/ArminJo/SoftStart/blob/master/extras/Placement1.jpg) | ![Placement](https://github.com/ArminJo/SoftStart/blob/master/extras/Placement2.jpg) |
| Placement with 100 nF and selfmade coil | Placement with 200 nF |
| ![Before](https://github.com/ArminJo/SoftStart/blob/master/extras/Before.jpg) | ![After](https://github.com/ArminJo/SoftStart/blob/master/extras/After.jpg) |
| Before | After with 100 nF (It runs for 6 years now) |

