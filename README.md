# SoftStart / TRIACRamp library
Generates TRIAC control pulse for soft start of motors used in circular saw, angle grinder and other DIY tools.

### Implemented and tested for AVR-ATtiny85

## Download
The actual version can be downloaded directly from GitHub [here](https://github.com/ArminJo/SoftStart/blob/master/TRIACRamp.zip?raw=true)

### After calling startRamp():
- Wait for 8 zero crossings of zero crossing interrupt to synchronize timing and check for right frequency (50 or 60 Hz).
- Output TRIAC pulse and decrease pulse delay from `START_PHASE_SHIFT_DEGREES` to 0 degree at every voltage zero crossing.
- After delay gets zero, output TRIAC pulse pulse at zero crossing of AC line. The multi pulse (3*350 micro seconds) will cover small delays of current zero crossing.

To change internal values like the layout of the multi pulse, you have to modify the values in TRIACRamp.h. You will find the file in the Arduino IDE under "Sketch/Show Sketch Folder" (or Ctrl+K) and then in the TRIACRamp/src directory.

Calibration mode outputs actual phase counter forever (at 115200 Baud (@1MHZ) at pin 6 / PB1) in order to adjust the 50% duty cycle trimmer.
- Format: `<counterForPositiveHalfWave>|<counterForNegativeHalfWave>\n`

