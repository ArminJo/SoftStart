/*
 * SoftStart.cpp
 *
 * Generates Triac control pulses for SoftStart of series motors.
 *
 *  Copyright (C) 2015  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of SoftStart https://github.com/ArminJo/SoftStart.
 *
 *  SoftStart is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 * No delay() or millis() or micros() are used, leaving both timers for TRIAC signal generation.
 *
 * Calibration mode is only available if INFO is defined (default).
 * It is entered by setting ramp speed to 0 (slow).
 * It outputs actual phase counter forever (at 115200 Baud (@1MHZ) at pin 6 / PB1) in order to adjust the 50% duty cycle trimmer.
 * Calibration mode can be ended only by reset.
 *
 * Voltage trigger interrupt is only enabled if load is attached.
 *
 * FUSE VALUES for LOAD_ON_OFF_DETECTION which means that CPU power is always on.
 * You may use the default values or enable additional Brown-out detection eg. at 4.3 volt.
 * Low=0X62 (default) Int RC Osc. 8 MHz divided by 8. 14 Clk + 64 ms startup.
 * High=0XDC BrowOut at VCC=4.3 volt
 * Extended=0XFF (default)
 *
 * FUSE VALUES for embedded version, which requires fast start, since soft start must begin as soon as power is on -> 14 Clk + Enable BOD
 * Low=0X52 Int RC Osc. 8 MHz divided by 8 (default). 14 Clk + 4 ms startup (for fast startup).
 * High=0XDC BrowOut at VCC=4.3 volt
 * Extended=0XFF (default)
 *
 */

#include "TRIACRamp.h"

#include "ATtinyUtils.h"
#include "ADCUtils.h"
#include "digitalWriteFast.h"

#ifdef INFO
#define TX_PIN PB1 // must be defined before this include
#include "ATtinySerialOut.cpp.h"
#endif

#include <avr/interrupt.h>
#include <math.h>   // for pow and log10f

//#define LOAD_ON_OFF_DETECTION // Do not start with ramp at boot up time, but wait for interrupt at LoadDetectionInput pin 6.

#define VERSION_EXAMPLE "2.0"

//
// ATMEL ATTINY85
//
//                                        +-\/-+
//                  RESET ADC0 (D5) PB5  1|    |8  Vcc
// USB+   Ramp Duration - ADC3 (D3) PB3  2|    |7  PB2 (D2) INT0/ADC1 - Zero voltage Crossing Sense
// USB-   Current Sense - ADC2 (D4) PB4  3|    |6  PB1 (D1) MISO/DO/AIN1/OC0B/OC1A/PCINT1 - TX Debug output - (Digispark) LED
//                                  GND  4|    |5  PB0 (D0) OC0A - TRIAC Control
//                                        +----+

#ifdef LOAD_ON_OFF_DETECTION
#ifdef INFO
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__)
#error Code size of this example is too large to fit in an ATtiny 25 or 45. deactivate #define LOAD_ON_OFF_DETECTION or deactivate #define INFO in TRIACRamp.h to shrink the code for an ATtiny45.
#endif
#endif
/*
 * Attaching and detaching of load is detected by measuring current (PIN 2 / PB3) at middle of each half wave.
 */
#define ZeroCurrentDetectionADCChannel 2
#endif

// PIN 2 /PB3 is used to read ramp duration
#define RampDurationADCChannel 3
/*
 * RAMP timing
 */
void readRampDelay(); // Value is read from PIN 3 and converted to value between 10 and 1000
// RampControl.DelayDecrement of 0x3E8 gives a 0.3 second ramp (34 zero crossings) from delay start value of 132 to 0
// RampControl.DelayDecrement of 0x100 gives a 1.3 second ramp (132 zero crossings) from delay start value of 132 to 0
// RampControl.DelayDecrement of 0x40 gives a 5 second ramp (528 zero crossings) from delay start value of 132 to 0
// RampControl.DelayDecrement of 0x10 gives a 20 second ramp from delay start value of 132 to 0
// RampControl.DelayDecrement of 0x01 gives a 5 minutes ramp from delay start value of 132 to 0

struct RampControlStruct RampControl;

#ifdef LOAD_ON_OFF_DETECTION
struct ControlStruct {
    bool isLoadAttached;
    int8_t NoLoadFoundCount; // Counter to determine if load is attached or detached

// do it only once per mains period
    bool LoadAttachedCurrentSampleTaken;

#  ifdef INFO
    uint16_t ZeroCurrentADCReferenceValueAtLastPrint; // to enable printing of only changed values - not used yet
#  endif
// ADC input is biased by VCC/2. Used for check of zero current detection / load detaching detection.
    uint16_t ZeroCurrentADCReferenceValue;

} SoftStartControl;

#define ALLOWED_DELTA_ZERO_CURRENT_LSB 6

#define PERIODS_THRESHOLD_FOR_LOAD_DETACHED 25 // periods without a load until off
#define PERIODS_THRESHOLD_FOR_LOAD_ATTACHED 4 // periods with a load until on
void setLoadDetached(void);
void checkForLoadAttached(void);
/**
 * Forward Declarations
 */
void setLoadAttached(void);
#endif // LOAD_ON_OFF_DETECTION

/***********************************************************************************
 * Code starts here
 ***********************************************************************************/
void setup(void) {

#ifdef INFO
    initTXPin();
    useCliSeiForStrings(true);
#endif

    initRampControl();

    // disable digital input buffer for ADC inputs to save power
    DIDR0 = (1 << ADC3D) | (1 << ADC2D);

#ifdef INFO
    // Print before debug output is switched to input
    // 35 characters which takes 3,3 millis at 115200 baud
    writeString(F("START " __FILE__ "\nVersion " VERSION_EXAMPLE " from " __DATE__ "\n"));
#endif

#ifdef INFO
    /*
     * Read value from external trimmer now in order to check for calibration mode (value < TEST_MODE_MAX_ADC_VALUE)
     */
    if (readADCChannelWithOversample(RampDurationADCChannel, 4) < TEST_MODE_MAX_ADC_VALUE) {
        writeString(F("Activate calibration mode\n"));
        RampControl.CalibrationModeActive = true;
    }
#endif

#ifdef LOAD_ON_OFF_DETECTION
    // here we are right after power on, so wait for external inputs to settle
    delay4CyclesInlineExact(0xFFFF);
    /*
     * Read reference value for zero current
     */
    uint16_t tZeroCurrentADCReferenceValue = readADCChannelWithOversample(ZeroCurrentDetectionADCChannel, 4);
    SoftStartControl.ZeroCurrentADCReferenceValue = tZeroCurrentADCReferenceValue;
#  ifdef INFO
    SoftStartControl.ZeroCurrentADCReferenceValueAtLastPrint = tZeroCurrentADCReferenceValue;
    writeString(F("ZeroCurrentReferenceValue="));
    writeUnsignedInt(tZeroCurrentADCReferenceValue);
    write1Start8Data1StopNoParity('\n');
    if (RampControl.CalibrationModeActive) {
        startRamp(); // this force calibration output
    } else {
        setLoadDetached();
    }
#  else
    setLoadDetached();
#  endif

#else // LOAD_ON_OFF_DETECTION
    readRampDelay();
    startRamp();
#endif // LOAD_ON_OFF_DETECTION

    interrupts(); // Enable interrupts
}

/* Actions for LOAD_ON_OFF_DETECTION
 * Take 1 sample at middle of current period to determine if load is still attached
 *
 * for INFO
 * Print ramp output for data set in ISR
 *
 * ALWAYS
 * Check and handle timer overflow == missing zero crossing interrupt
 */
void loop(void) {

#ifdef LOAD_ON_OFF_DETECTION
    uint8_t tCurrentCount = TCNT0;
    /*
     * Take 1 sample at middle of current period to determine if load is still attached
     */
    if (tCurrentCount < (TIMER_COUNT_AT_ZERO_CROSSING / 2)) {
        SoftStartControl.LoadAttachedCurrentSampleTaken = false;
    } else if (!SoftStartControl.LoadAttachedCurrentSampleTaken) {
        // Do it only once per mains period
        SoftStartControl.LoadAttachedCurrentSampleTaken = true;
        checkForLoadAttached();
    }
#endif

    checkAndHandleCounterOverflowForLoop();

#if defined(INFO)
    printRampInfo();
#endif

}

/*
 * Read ramp duration. Use x^2 function to get better resolution at low input values.
 * convert to 0 -> 5 second ramp
 */
void readRampDelay() {
    float tValue = readADCChannelWithOversample(RampDurationADCChannel, 4);
    tValue = tValue / (1024 / 1.2); // gives range 0,00469 to 1.8 since input values < 4 leads to test mode
    tValue = tValue + 1.8; // gives range 1.80469 to 3
    uint32_t tRampMillis = pow(17, tValue); // gives 163,78 to 4913
    setRampDurationMillis(tRampMillis);
#ifdef INFO
    writeString(F("Ramp="));
    writeLong(tRampMillis);
    writeString(F(" ms\n"));
//    writeString(" ms\ncomputed DelayDecrementPerHalfWaveShift16=");
//    writeLong(RampControl.DelayDecrementPerHalfWaveShift16);
//    write1Start8Data1StopNoParity('\n');
#endif
}

#ifdef LOAD_ON_OFF_DETECTION
/*
 * Load attached, stop triggering TRIAC and reset state
 * Enable voltage zero crossing interrupt
 */
void setLoadAttached(void) {
    SoftStartControl.isLoadAttached = true;

#ifdef INFO
    writeString(F("Start\n"));
#endif
    // get maybe changed value for next turn
    readRampDelay();
    startRamp();
}

/*
 * Load detached, stop triggering TRIAC and reset state
 */
void setLoadDetached(void) {
    SoftStartControl.isLoadAttached = false;
    stopRamp();

// enable pcint and disable timer interrupts
// First write last message
#if defined(INFO)
    writeString(F("Switch OFF and wait for load attached\n"));
#endif
}

/*
 * We are called at the middle of the current period.
 * Check if current flows / load is attached or not.
 */
void checkForLoadAttached(void) {
    uint16_t tCurrentDetectionADCValue = readADCChannelWithOversample(ZeroCurrentDetectionADCChannel, 2);
    uint8_t tInPositiveHalfWave = PINB & (1 << ZeroVoltageDetectionInput);

    /*
     * Check if current is zero
     */
    uint16_t tZeroCurrentADCReferenceValue = SoftStartControl.ZeroCurrentADCReferenceValue;
    bool tZeroCurrentDetected = false;
    if (tInPositiveHalfWave) {
        // Positive phase
        if (tCurrentDetectionADCValue < tZeroCurrentADCReferenceValue) {
            /*
             * Current is below zero current on positive phase -> adjust zero current value
             */
            SoftStartControl.ZeroCurrentADCReferenceValue--;
            tZeroCurrentDetected = true;
        } else if (tCurrentDetectionADCValue < (tZeroCurrentADCReferenceValue + ALLOWED_DELTA_ZERO_CURRENT_LSB)) {
            tZeroCurrentDetected = true;
        }
    } else {
        // negative phase
        if (tCurrentDetectionADCValue > tZeroCurrentADCReferenceValue) {
            /*
             * Current is above zero current on positive phase -> adjust zero current value
             */
            SoftStartControl.ZeroCurrentADCReferenceValue++;
            tZeroCurrentDetected = true;
        } else if (tCurrentDetectionADCValue > (tZeroCurrentADCReferenceValue - ALLOWED_DELTA_ZERO_CURRENT_LSB)) {
            tZeroCurrentDetected = true;
        }
    }

    if (tZeroCurrentDetected) {
        if (SoftStartControl.isLoadAttached) {
            /*
             * when load was attached but now is detached, increment counter until threshold.
             */
            SoftStartControl.NoLoadFoundCount++;
#ifdef INFO
            writeString("L");
            write1Start8Data1StopNoParity(SoftStartControl.NoLoadFoundCount);
            write1Start8Data1StopNoParity('\n');
#endif
            // * 2 since we count each half wave
            if (SoftStartControl.NoLoadFoundCount >= (PERIODS_THRESHOLD_FOR_LOAD_DETACHED * 2)) {
                setLoadDetached();
            }
        }
    } else {
        /*
         * Load current detected, decrement counter until 0
         */
        if (SoftStartControl.NoLoadFoundCount > 0) {
            SoftStartControl.NoLoadFoundCount--;
        }
        /*
         * threshold for load attached reached.
         */
        if (!SoftStartControl.isLoadAttached
                && SoftStartControl.NoLoadFoundCount
                        <= ((PERIODS_THRESHOLD_FOR_LOAD_DETACHED - PERIODS_THRESHOLD_FOR_LOAD_ATTACHED) * 2)) {
            setLoadAttached();
        }
    }
}

#endif // LOAD_ON_OFF_DETECTION
