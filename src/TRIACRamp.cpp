/*
 * TRIACRamp.cpp
 *
 *
 * Generates Triac control pulses for SoftStart of series motors.
 *
 *  Copyright (C) 2018  Armin Joachimsmeyer
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
 *
 * On ramp start:
 * - TRIAC_CONTROL_STATE_WAIT_FOR_SETTLING -> - Wait for 8 zero crossings of zero crossing interrupt to synchronize timing and check for right frequency (50 or 60 Hz).
 * - STATE_RAMP -> Output TRIAC pulse and decrease pulse delay from `START_PHASE_SHIFT_DEGREES` to MINIMUM_PHASE_SHIFT_COUNT (0 degree) at every voltage zero crossing.
 * - TRIAC_CONTROL_STATE_FULL_POWER -> Output TRIAC pulse pulse at zero crossing of AC line. The multi pulse (3*350 micro seconds) will cover small delays of current zero crossing.
 *
 * Calibration mode outputs actual phase counter forever (at 115200 Baud (@1 MHZ) at pin 6 / PB1) in order to adjust the 50% duty cycle trimmer.
 * Format: <counterForPositiveHalfWave>|<counterForNegativeHalfWave>\n
 *
 * INTERNALS:
 * Current voltage is biased by VCC/2 by two resistors in order to be able to measure also negative current.
 * Mains triggers the interrupt pin on every edge.
 * 50 Hz gives 10 milliseconds cycle of mains and 156 * 64 microsecond clock cycles (9984 microseconds)
 * 60 Hz gives 8.33 milliseconds cycle of mains and 130 * 64 microsecond clock cycles (8320 microseconds)
 *
 * Timer0 runs in Fast PWM Mode starting at FF every 50/60 Hz half cycle and counting up. It is never stopped, only interrupts are disabled.
 * - At startup the counter is used to determine length of 50/60 Hz half cycle which compensates for internal oscillator drift.
 * - During ramp the timer is set to generate Interrupts after zero crossing, which turns on the TRIAC.
 * Timer1 is used for generating the width of TRIAC pulse. TRIAC is turned off after a delay implemented by CounterOverflow.
 * Timer1 also implements the breaks and the pulses for multiple TRIAC pulses by CounterOverflow.
 *
 * Zero current detection is done by ADC reading the voltage at the current shunt -> ZeroVoltageDetectionInput.
 *
 * Serial Output:
 * N1<ActualTimer0Count> - Noise -> timer at zero crossing interrupt not in expected range.
 * N2<ActualTimer0Count> - Noise during ramp -> timer at zero crossing interrupt not in expected (narrower) range.
 * MP<MainsHalfWaveTimer0Count> - MainsPeriod -> measured timer count for a half wave, printed at end of synchronizing phase
 * R<NextTimerDelay><NextTRIACDelayMicros><ActualTimer0CountAtTRIACPulse> - Ramp info
 * OF<ActualTimer0Count> OverFlow -> missing (or noisy) zero crossing interrupt.
 */

#include <Arduino.h>
#include "TRIACRamp.h"
#include "ATtinyUtils.h"

#include <avr/io.h>
#include "digitalWriteFast.h"

#include "ATtinySerialOut.h"

void initRampControl() {
    // Pin is active LOW, so set them to HIGH initially
    digitalWriteFast(TRIACControlOutput, HIGH);
    pinModeFast(TRIACControlOutput, OUTPUT);

    // Enable zero crossing interrupt at PB2
    GIMSK = (1 << INT0); // Enable INT0 (50/60 HZ input) + disable PinChange interrupt
    MCUCR = (1 << ISC00); // interrupt on any edge for INT0 to get 100/120 Hz interrupts

    /*
     * Setup Timer0 for ramp delay and measurement of mains period.
     * This timer runs forever
     */
// Fast PWM mode
    TCCR0A = TIMER0_FAST_PWM;
#if (F_CPU == 1000000)
    //Start Timer 0 1MHz/64 -> 64 us clock -> gives 156 counts for 10 ms
    TCCR0B = TIMER0_CLOCK_DIVIDER_FOR_64_MICROS;
#endif
#if (F_CPU == 8000000)
    //Start Timer 0 8MHz/10254 -> 128 us clock -> gives 78 counts for 10 ms
    TCCR0B = TIMER0_CLOCK_DIVIDER_FOR_128_MICROS;
#endif
    TCNT0 = 0;

    RampControl.NextOCRA = OCRA_VALUE_FOR_NO_POWER; // to avoid triggering the TRIAC - 0xFF means full power mode with no minimal phase count and should not be used here
    RampControl.SoftStartState = TRIAC_CONTROL_STATE_STOP;
    RampControl.DoWriteRampData = false;
    RampControl.CalibrationModeActive = false;
    // TODO use EEPROM for this value
    RampControl.MainsHalfWaveTimerCount = TIMER_COUNT_AT_ZERO_CROSSING; // initialize value
}

/*
 * Stop triggering TRIAC and reset ramp state
 * Enable voltage zero crossing interrupt for ramp generation.
 */
void startRamp(void) {
    RampControl.HalfWaveCounterIntern = 0;
    RampControl.MainsHalfWaveTimerCountAccumulated = 0;
    RampControl.NextOCRA = RampControl.MainsHalfWaveTimerCount - START_PHASE_SHIFT_MARGIN_COUNT;
#ifdef INFO
    RampControl.ActualTimerCountAtTriggerPulse = 0;
#endif

    GIFR = (1 << INTF0) | (1 << PCIF); // reset all interrupt flags
    TIMSK = 0; // Start with all timer interrupts disabled

#ifdef RAMP_INDICATOR_LED
    // switch LED on
    digitalWriteFast(LED_PIN, 0);
#endif
    RampControl.SoftStartState = TRIAC_CONTROL_STATE_WAIT_FOR_SETTLING;

}

void stopRamp() {
    RampControl.SoftStartState = TRIAC_CONTROL_STATE_STOP;
    RampControl.NextOCRA = OCRA_VALUE_FOR_NO_POWER; // Otherwise it sticks at full power.
    TIMSK = 0; // All Timer interrupts disabled
    TIFR = (1 << OCF0A) | (1 << TOV1); // Clear Timer0 output compare match int  + Timer1 overflow int

    // set TRIAC pin to inactive
    digitalWriteFast(TRIACControlOutput, 1);

#ifdef RAMP_INDICATOR_LED
    // switch LED off
    digitalWriteFast(LED_PIN, 1);
#endif
}

void switchToFullPower() {
    RampControl.SoftStartState = TRIAC_CONTROL_STATE_FULL_POWER;
    RampControl.MicrosecondsDelayForTriggerPulse = 0;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverflow"
// truncation is intended!
    RampControl.NextOCRA = TIMER0_COUNTER_TOP + MINIMUM_PHASE_SHIFT_COUNT;
#pragma GCC diagnostic pop
}

void setCalibrationMode() {
    RampControl.CalibrationModeActive = true;
}

void setRampDurationSeconds(uint16_t aRampDurationSeconds) {
    uint32_t tValue = ((uint32_t)TIMER_COUNT_AT_ZERO_CROSSING << 16) / HALF_WAVES_PER_SECOND; // 0x00018CCC
    if (aRampDurationSeconds == 0) {
        RampControl.DelayDecrement = tValue * 4; // Quarter of a second
    } else {
        RampControl.DelayDecrement = tValue / aRampDurationSeconds;
    }
}

void setRampDurationMillis(uint32_t aRampDurationMillis) {
    // instead of *1000 use *100 and /10 in divisor to avoid 32 bit overflow
    uint32_t tValue = (((uint32_t)TIMER_COUNT_AT_ZERO_CROSSING << 16) * 100) / (HALF_WAVES_PER_SECOND / 10); // 0x060E0000
    if (aRampDurationMillis == 0) {
        RampControl.DelayDecrement = ((uint32_t)TIMER_COUNT_AT_ZERO_CROSSING << 16) / 4; // 4 mains periods to full power
    } else {
        RampControl.DelayDecrement = tValue / aRampDurationMillis;
    }
}

/*
 * Voltage zero crossing detection by external interrupt pin.
 * The actual delay of the TRIAC pulse for this half wave was computed in the interrupt of the half wave before!
 *
 * Counter is set to FF in order to reload RampControl.NextOCRA immediately at next clock.
 * TRIAC_CONTROL_STATE_STOP -> Just count interrupts
 * TRIAC_CONTROL_STATE_WAIT_FOR_SETTLING -> Timer0 is used to measure the period between the line interrupts.
 *                            Compute mains period clock cycles and prepare first TRIAC pulse.
 * STATE_RAMP -> Delay is implemented by Timer0 counting up until RampControl.NextOCRA is reached.
 *               This in turn generates an interrupt which switches TRIAC on. Timer1 is used for generating the TRIAC pulse(s).
 *               RampControl.NextOCRA is decreased every voltage zero crossing.
 * TRIAC_CONTROL_STATE_FULL_POWER -> to keep it simple just change nothing, use old values already set.
 */

ISR(INT0_vect) {
    uint8_t tActualCount = TCNT0;

    /*
     * reset prescaler in order to avoid jitter
     */
    GTCCR = (1 << PSR0);

// reset counter to top in order to reload ocra at next clock
    OCR0A = RampControl.NextOCRA;
    TCNT0 = TIMER0_COUNTER_TOP;

    /*
     * Now begin state machine
     */
    if (RampControl.NextOCRA == TIMER_VALUE_FOR_FULL_POWER) {
        // TRIAC_CONTROL_STATE_FULL_POWER with MINIMUM_PHASE_SHIFT_COUNT == 0 here.
        // Here no additional delay, no timer counting from 0xFF to 0x00,
        // so there is a step in delay from last ramp delay to full power of (measured) 160 microseconds
        StartTriacPulse();
        RampControl.HalfWaveCounterForExternalTiming++;
        RampControl.EnableHalfWaveActionAtFullPower = true;
        return;
    }
    RampControl.HalfWaveCounterForExternalTiming++;

    if (RampControl.SoftStartState == TRIAC_CONTROL_STATE_STOP) {
        // do nothing;
        return;
    }

    if (RampControl.CalibrationModeActive) {

        /*
         * TEST / CALIBRATION here: output actual counter forever in order to adjust the 50% duty cycle trimmer
         */
        if (digitalReadFast(ZeroVoltageDetectionInput)) {
            write1Start8Data1StopNoParity(tActualCount);
            write1Start8Data1StopNoParity('|');
        } else {
            write1Start8Data1StopNoParity(tActualCount);
            write1Start8Data1StopNoParity('\n');
        }
        return;
    }

    /*
     * Detect noise -> just return
     */
    if (tActualCount < (RampControl.MainsHalfWaveTimerCount - ALLOWED_DELTA_PHASE_SHIFT_COUNT)
            || tActualCount > (RampControl.MainsHalfWaveTimerCount + ALLOWED_DELTA_PHASE_SHIFT_COUNT)) {
#ifdef ERROR
        write1Start8Data1StopNoParity('N');
        write1Start8Data1StopNoParity('1');
        write1Start8Data1StopNoParity(tActualCount);
        write1Start8Data1StopNoParity('\n');
#endif
        return;
    }

    if (RampControl.SoftStartState == TRIAC_CONTROL_STATE_WAIT_FOR_SETTLING) {
        /*******************************************************************
         * TRIAC_CONTROL_STATE_WAIT_FOR_SETTLING -> measure period and get delay at last
         *******************************************************************/
        if (RampControl.HalfWaveCounterIntern == 0) {
            /*
             * First cycle. tActualCount may be invalid so just increment counter - see below
             */

        } else if (RampControl.HalfWaveCounterIntern <= SYNCHRONIZING_CYCLES) {
            /*
             * sum count for one mains phase
             */
            RampControl.MainsHalfWaveTimerCountAccumulated += tActualCount;
#ifdef DEBUG
            write1Start8Data1StopNoParity('A');
            write1Start8Data1StopNoParity(tActualCount);
#endif
        } else {
            /*
             * Last Cycle. -> compute mains period clock cycles and prepare first TRIAC pulse
             */
            RampControl.MainsHalfWaveTimerCount = (RampControl.MainsHalfWaveTimerCountAccumulated + (SYNCHRONIZING_CYCLES / 2))
                    / SYNCHRONIZING_CYCLES;

            RampControl.TimerCountForTriggerDelayShift16.word.LowWord = 0;
            RampControl.TimerCountForTriggerDelayShift16.word.HighWord = RampControl.MainsHalfWaveTimerCount
                    - START_PHASE_SHIFT_MARGIN_COUNT;
            RampControl.NextOCRA = RampControl.MainsHalfWaveTimerCount - START_PHASE_SHIFT_MARGIN_COUNT;
            RampControl.NextMicrosecondsDelayForTriggerPulse = 0;

            TIFR = (1 << OCF0A) | (1 << TOV1);    // Otherwise an interrupt is generated directly
            TCCR1 = 0; // stop timer 1 - is required here even if no one starts it before
            TIMSK = (1 << OCIE0A) | (1 << TOIE1); // Timer0 output compare match int enabled + Timer1 overflow int enabled

            RampControl.SoftStartState = TRIAC_CONTROL_STATE_RAMP_UP;
#ifdef INFO
            write1Start8Data1StopNoParity('M');
            write1Start8Data1StopNoParity('P');
            write1Start8Data1StopNoParity(RampControl.MainsHalfWaveTimerCount);
            write1Start8Data1StopNoParity('\n');
#endif
        }
        RampControl.HalfWaveCounterIntern++;

    } else if (RampControl.SoftStartState == TRIAC_CONTROL_STATE_RAMP_UP) {
        /*
         * Use narrower plausibility values here for the (10 millis) mains cycle. If not met, just do nothing and wait for next transition,
         * main loop will handle counter overflow.
         */
        if (tActualCount < (RampControl.MainsHalfWaveTimerCount - (ALLOWED_DELTA_PHASE_SHIFT_COUNT / 2))
                || tActualCount > (RampControl.MainsHalfWaveTimerCount + (ALLOWED_DELTA_PHASE_SHIFT_COUNT / 2))) {
            /*
             * Noise here. Actual count not in the expected range
             * Printing of noise may lead to delaying the next interrupt and therefore missing the right count value
             */
#ifdef ERROR
            write1Start8Data1StopNoParity('N');
            write1Start8Data1StopNoParity('2');
            write1Start8Data1StopNoParity(tActualCount);
            write1Start8Data1StopNoParity('\n');
#endif
        } else {
            /*
             * TRIAC_CONTROL_STATE_RAMP_UP  && No noise
             */
            if ((RampControl.TimerCountForTriggerDelayShift16.word.HighWord) > MINIMUM_PHASE_SHIFT_COUNT) {
                /*
                 *  Normal ramp -> compute trigger delay value for next mains phase / interrupt - handle underflow
                 *  decrement duration, set timer (compute current zero crossing count) and check for end
                 *
                 */
                RampControl.MicrosecondsDelayForTriggerPulse = RampControl.NextMicrosecondsDelayForTriggerPulse;
                if (RampControl.TimerCountForTriggerDelayShift16.ULong >= RampControl.DelayDecrement) {
                    RampControl.TimerCountForTriggerDelayShift16.ULong -= RampControl.DelayDecrement;
                } else {
                    // underflow here
                    RampControl.TimerCountForTriggerDelayShift16.ULong = 0;
                }

                /*
                 * Because of double buffering OCRA will be active at next period
                 */
                RampControl.NextOCRA = RampControl.TimerCountForTriggerDelayShift16.byte.MidHighByte;
                // Results in value from 0 to TIMER0_CLOCK_CYCLE_MICROS
                RampControl.NextMicrosecondsDelayForTriggerPulse = RampControl.TimerCountForTriggerDelayShift16.byte.MidLowByte
                        / (256 / TIMER0_CLOCK_CYCLE_MICROS);

#ifdef INFO
                /*
                 * Delegate printing to main loop, otherwise it may delay TRIAC trigger
                 */
                RampControl.DoWriteRampData = true;
#endif
            } else {
                /*
                 * End of ramp -> switch to full power
                 */
                switchToFullPower();
#ifdef RAMP_INDICATOR_LED
                // switch LED off
                digitalWriteFast(LED_PIN, 1);
#endif
            }
        }
    } else /*if (sZeroCrossingState == TRIAC_CONTROL_STATE_FULL_POWER)*/{
        /*
         * TRIAC_CONTROL_STATE_FULL_POWER here -> to keep it simple, just change nothing, use values already set at end of ramp.
         */
    }
}

/*
 * Timer0 Compare Interrupt
 * Now the ramp-timer delay is reached => add additional delay for better resolution and then start the TRIAC pulse
 */
ISR(TIMER0_COMPA_vect) {
// additional delay for finer ramp/delay resolution
    delayMicroseconds(RampControl.MicrosecondsDelayForTriggerPulse);
    StartTriacPulse();
}

/*
 * Output TRIAC pulse
 * Start Timer1
 */
void StartTriacPulse(void) {
// set TRIAC pin to active
    digitalWriteFast(TRIACControlOutput, 0);
#ifdef INFO
    RampControl.ActualTimerCountAtTriggerPulse = TCNT0;
#endif
    RampControl.TRIACPulseCount = 1;

// start timer1 to end pulse
    GTCCR = (1 << PSR1); // reset prescaler
    TCNT1 = 0x100 - (TRIAC_PULSE_WIDTH_MICROS / TRIAC_PULSE_TIMER_CLOCK_CYCLE_MICROS);
    TCCR1 = TIMER1_CLOCK_DIVIDER; // normal mode -> gives maximum TRIAC pulse width of 1 Milliseconds for timer clock 4 us and count to 256.
}

/*
 * Switch off Triac output
 */
#define TRIAC_MULTIPLE_PULSE_TIME_MICROS ((TRIAC_PULSE_WIDTH_MICROS * TRIAC_PULSE_NUMBERS) + (TRIAC_PULSE_BREAK_MICROS*(TRIAC_PULSE_NUMBERS-1)))
ISR(TIMER1_OVF_vect) {
    TCCR1 = 0; // stop timer 1

    if (RampControl.SoftStartState == TRIAC_CONTROL_STATE_STOP) {
        /*
         * Other Timer1 usage (e.g.Tone scan mode) here, just return after timer stopped. Next slope will start timer again.
         */
        return;
    }

    bool tTriacLevel = digitalReadFast(TRIACControlOutput);
    if (!tTriacLevel) {
        /*
         * deactivate TRIAC output
         */
        digitalWriteFast(TRIACControlOutput, 1);
        uint8_t tTCNT0 = TCNT0;
        if (RampControl.TRIACPulseCount < TRIAC_PULSE_NUMBERS
                && (tTCNT0 == TIMER_VALUE_FOR_FULL_POWER || tTCNT0 < (TRIAC_MULTIPLE_PULSE_TIME_MICROS / TIMER0_CLOCK_CYCLE_MICROS))) {
            /*
             *  output multiple TRIAC pulse since remaining delay is less than total time of multiple pulses
             */
            // setup timer for break timing
            GTCCR = (1 << PSR1); // reset prescaler
            TCNT1 = 0x100 - (TRIAC_PULSE_BREAK_MICROS / TRIAC_PULSE_TIMER_CLOCK_CYCLE_MICROS);
            TCCR1 = TIMER1_CLOCK_DIVIDER; // normal mode -> gives maximum TRIAC pulse width of 1 Milliseconds for timer clock 4 us and count to 256.
        }
    } else {
        /*
         * start next TRIAC pulse after the break
         */
        digitalWriteFast(TRIACControlOutput, 0);
        // start timer1 to end pulse
        GTCCR = (1 << PSR1); // reset prescaler
        TCNT1 = 0x100 - (TRIAC_PULSE_WIDTH_MICROS / TRIAC_PULSE_TIMER_CLOCK_CYCLE_MICROS);
        TCCR1 = TIMER1_CLOCK_DIVIDER; // normal mode -> gives maximum TRIAC pulse width of 1 Milliseconds for timer clock 4 us and count to 256.
        RampControl.TRIACPulseCount++;
    }
}

/*
 * Write output for STATE_RAMP here in order to keep the timing of ISR
 */
void printRampInfo() {
#ifdef INFO
    if (RampControl.DoWriteRampData) {
        write1Start8Data1StopNoParityWithCliSei('R');
        // output actual trigger delay
        write1Start8Data1StopNoParityWithCliSei(RampControl.TimerCountForTriggerDelayShift16.byte.MidHighByte);
        write1Start8Data1StopNoParityWithCliSei(RampControl.TimerCountForTriggerDelayShift16.byte.MidLowByte);
        write1Start8Data1StopNoParityWithCliSei(RampControl.ActualTimerCountAtTriggerPulse);
        write1Start8Data1StopNoParityWithCliSei('\n');
        RampControl.DoWriteRampData = false;
    }
#endif
}

void checkAndHandleCounterOverflowForLoop() {
    if (!RampControl.CalibrationModeActive) {
        /*
         * Check for counter overflow if load attached, but not in test/calibration mode
         */
        uint8_t tActualTimerCount = TCNT0;
        if (tActualTimerCount
                >= RampControl.MainsHalfWaveTimerCount
                        + (ALLOWED_DELTA_PHASE_SHIFT_COUNT * 2)&& tActualTimerCount < TIMER_VALUE_FOR_FULL_POWER) {
            // assume missing trigger -> setup counter for next period
            TCNT0 = tActualTimerCount - RampControl.MainsHalfWaveTimerCount;
#ifdef ERROR
            if (RampControl.SoftStartState != TRIAC_CONTROL_STATE_STOP) {
                write1Start8Data1StopNoParityWithCliSei('O');
                write1Start8Data1StopNoParityWithCliSei('F');
                write1Start8Data1StopNoParityWithCliSei(tActualTimerCount);
                write1Start8Data1StopNoParityWithCliSei('\n');
            }
#endif
        }
    }
}
