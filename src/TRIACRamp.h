/*
 * TRIACRamp.h
 *
 *  Copyright (C) 2018  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *  License: GPL v3 (http://www.gnu.org/licenses/gpl.html)
 *
 */
/*
 * Definitions for Attiny85
 */

#ifndef TRIACRAMP_H_
#define TRIACRAMP_H_

#include <stdint.h>

//#define DEBUG // for debug output
#define INFO  // for info output - delay, load detaching values
//#define ERROR   // print error output - count overflow/missing zero crossing trigger

// Propagate debug level
#ifdef TRACE
#define DEBUG
#endif
#ifdef DEBUG
#define INFO
#endif
#ifdef INFO
#define WARN
#endif
#ifdef WARN
#define ERROR
#endif

// if used for 60 instead of 50 Hz mains uncomment the next line
//#define MAINS_HAVE_60_HZ

//
// ATMEL ATTINY85
//
//                                        +-\/-+
//                  RESET ADC0 (D5) PB5  1|    |8  Vcc
//      PCINT3/CLKI/!OC1B/ADC3 (D3) PB3  2|    |7  PB2 (D2) INT0 - Zero Voltage Crossing Sense
// (Zero) Current Sense - ADC2 (D4) PB4  3|    |6  PB1 (D1) MISO/DO/AIN1/OC0B/OC1A/PCINT1 - TX Debug output / LED out / Load on/off detect
//                                  GND  4|    |5  PB0 (D0) OC0A - TRIAC Control
//                                        +----+
//

/*
 * PIN and ADC channel definitions
 */
#define TRIACControlOutput PB0          // active LOW
#define ZeroVoltageDetectionInput PB2   // generates interrupts at both edges

#define LED_PIN PB1
#ifdef TX_PIN
#if (LED_PIN != TX_PIN)
#error "LED pin must be equal TX pin."
#endif
#endif
// outcomment if no led for indicating ramp is used
// #define RAMP_INDICATOR_LED LED_PIN

/*
 * Timer definitions
 */
#define TIMER0_COUNTER_TOP 0xFF
#define TIMER0_FAST_PWM ((1 << WGM01) | (1 << WGM00))

#if (F_CPU != 1000000) &&  (F_CPU != 8000000)
#error "F_CPU value must be 1000000 or 8000000."
#endif

#if (F_CPU == 1000000)
#define TIMER0_CLOCK_CYCLE_MICROS 64
#endif

#if (F_CPU == 8000000)
#define TIMER0_CLOCK_CYCLE_MICROS 128
#endif

#ifdef MAINS_HAVE_60_HZ
// 0x82. 130 * 64 us gives 8320 microseconds
#define TOTAL_PHASE_SHIFT_COUNT (8333 / TIMER0_CLOCK_CYCLE_MICROS)
#define HALF_WAVES_PER_SECOND 120
#else
// 0x9C. 156 * 64 us gives 9984 microseconds
// 0x4E. 78 * 128 us gives 9984 microseconds
#define TOTAL_PHASE_SHIFT_COUNT (10000 / TIMER0_CLOCK_CYCLE_MICROS)
#define HALF_WAVES_PER_SECOND 100
#endif

#define TIMER_COUNT_AT_ZERO_CROSSING (TOTAL_PHASE_SHIFT_COUNT - 1) // -1 since timer starts with 0xFF

// for plausibility check of voltage zero crossing detection. 6 => ~2 Hz/380 usec
#define ALLOWED_DELTA_PHASE_SHIFT_COUNT 6

/*
 * GENERAL definitions
 */
// for synchronizing and measurement of mains period after power up - must be an even number
#define SYNCHRONIZING_CYCLES (2*4)

/*
 * Trigger impulse timing
 */
#define TIMER1_CLOCK_DIVIDER TIMER1_CLOCK_DIVIDER_FOR_8_MICROS // gives max 2ms trigger pulse
#define TRIAC_PULSE_TIMER_CLOCK_CYCLE_MICROS 8

/*
 *  Length of trigger pulse - 100 us is too small for my circuit
 */
#define TRIAC_PULSE_WIDTH_MICROS 250

/*
 * Amount of multiple trigger pulses if delay is less than total time of multiple pulses
 * This avoids flickering at small loads
 */
#define TRIAC_PULSE_NUMBERS 3

/*
 *  Length of break between multiple trigger pulses
 */
#define TRIAC_PULSE_BREAK_MICROS 400

// Initial delay of TRIAC trigger impulse.
// Values from 0 - 180 degrees, but the extremes makes no sense
#define START_PHASE_SHIFT_DEGREES 160

#define START_PHASE_SHIFT_MARGIN_COUNT ((TOTAL_PHASE_SHIFT_COUNT * (180 - START_PHASE_SHIFT_DEGREES)) / 180)
#if (START_PHASE_SHIFT_DEGREES < 45)
#error "START_PHASE_SHIFT_DEGREES below 45 degree makes no sense."
#endif
#define START_PHASE_SHIFT_MARGIN_MICROS (((1000000 / HALF_WAVES_PER_SECOND) * (180 - START_PHASE_SHIFT_DEGREES)) / 180) // only used for plausi below
#if  (TRIAC_PULSE_WIDTH_MICROS > (START_PHASE_SHIFT_MARGIN_MICROS + 20))
#error "START_PHASE_SHIFT_DEGREES is too high. The TRIAC pulse may reach the next half wave"
#endif

// Timer0 count at end of ramp - Can be adjusted for testing
#define MINIMUM_PHASE_SHIFT_COUNT 0
//#define MINIMUM_PHASE_SHIFT_COUNT 23 // for testing

/*
 * Test mode to adjust external trimmer to 50% duty cycle
 */
#define TEST_MODE_MAX_ADC_VALUE 4
extern bool isTestMode;  // output actual counter forever in order to adjust the 50% duty cycle trimmer

/*
 * Sometimes it helps the compiler if you use this union
 */
union Mylong {
    struct {
        uint8_t LowByte;
        uint8_t MidLowByte;
        uint8_t MidHighByte;
        uint8_t HighByte;
    } byte;
    struct {
        uint16_t LowWord;
        uint16_t HighWord;
    } word;
    uint32_t ULong;
    int32_t Long;
};

/*
 * States of the state machine
 */
#define STATE_STOP 0
#define STATE_OUTPUT_COUNTER 1 // output actual counter forever in order to adjust the 50% duty cycle trimmer
#define STATE_WAIT_FOR_SETTLING 2
#define STATE_RAMP_UP 3
#define STATE_RAMP_DOWN 4 // for future use
#define STATE_FULL_POWER 5

struct RampControlStruct {
    volatile uint8_t SoftStartState;
    bool CalibrationModeActive;

    /*
     * Timer
     */
    uint8_t NextOCRA; // Next Timer value to be set - if equals 0xFF then no delay is assumed => STATE_FULL_POWER
    /*
     * Since the counter has a resolution of 64 us (and you can see this brightness steps) we use this to get a finer resolution.
     */
    uint8_t NextMicrosecondsDelayForTriggerPulse;
    uint8_t MicrosecondsDelayForTriggerPulse;

    uint8_t TRIACPulseCount; //counts multiple TRIAC pulses
#ifdef INFO
    volatile uint8_t ActualTimerCountAtTriggerPulse;
#endif
    /*
     * Ramp computation
     * Delay has 3 Byte resolution.
     * The MidHighByte Byte is directly used for Timer OCRA.
     * The MidLowByte is used for microseconds delay.
     * The LowByte is used to increase the resolution to enable long ramps.
     */
    Mylong TimerCountForTriggerDelayShift16; // only used for RAMP
    uint32_t DelayDecrement; // amount of delay to be subtracted from RampControl.TimerCountForTriggerDelayShift16 each half wave.

    /*
     * The measured count of Timer0 at zero crossing of one half wave.
     * Computed by settling phase.
     */
    uint8_t MainsHalfWaveTimerCount; // Used twice. Once at startup and once at beginning of full power state
    volatile uint16_t MainsHalfWaveTimerCountAccumulated;
    uint8_t HalfWaveCounterIntern; // counts half waves to determine end of settling phase

    /*
     * derived timer used by loop()
     */
    volatile uint16_t HalfWaveCounterForExternalTiming;
    // Signal to loop() that interrupt has occurred and new half wave begins
    volatile bool EnableHalfWaveActionAtFullPower;
    // Signal to loop() to write data
    volatile bool DoWriteRampData;
};
extern RampControlStruct RampControl;

void initRampControl();
void startRamp();
void stopRamp();
void setCalibrationMode();
void checkAndHandleCounterOverflowForLoop();
void printRampInfo();
void StartTriacPulse();

#ifdef __cplusplus
extern "C" {
#endif
// must be provided by Arduino library or own code
void delayMicroseconds(unsigned int us);
#ifdef __cplusplus
} // extern "C"
#endif

#endif /* TRIACRAMP_H_ */
