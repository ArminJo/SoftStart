/*
 * TinyUtils.cpp
 *
 *  Created on: 05.03.2018
 *      Author: Armin
 */
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny87__) || defined(__AVR_ATtiny167__)

#include "ATtinyUtils.h"

#include <avr/boot.h>  // needed for boot_signature_byte_get()
#include <avr/power.h> // needed for clock_prescale_set()
#include <avr/io.h>
#include <avr/interrupt.h>  // for sei() + cli()

// since we have not included Arduino.h
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

/*
 * Use port pin number (PB0-PB5) not case or other pin number
 */
inline void toggleFastPortB(uint8_t aOutputPinNumber) {
    PINB = (1 << aOutputPinNumber);
}

// for constant values we can also use: digitalWriteFast()
inline void digitalWriteFastPortB(uint8_t aOutputPinNumber, bool aValue) {
    (aValue ? PORTB |= (1 << aOutputPinNumber) : PORTB &= ~(1 << aOutputPinNumber));
}

// for constant values we can also use: digitalReadFast()
inline bool digitalReadFastPortB(uint8_t aInputPinNumber) {
    return (PINB & (1 << aInputPinNumber));
}

// not for INPUT_PULLUP - can be done by setting to input and adding digitalWriteFastPortB(aOutputPinNumber,1);
inline void pinModeFastPortB(uint8_t aOutputPinNumber, uint8_t aMode) {
    (aMode ? DDRB |= (1 << aOutputPinNumber) /* OUTPUT */: DDRB &= ~(1 << aOutputPinNumber));
}

#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)

/*
 * Like tone(), but use OCR1B (PB4) + !OCR1B (PB3)
 */
void PWMtone(uint8_t aPin, unsigned int aFrequency, unsigned long aDurationMillis) {
    tone(aPin, aFrequency / 2, aDurationMillis); // specify half frequency -> PWM doubles it
    TCCR1 = TCCR1 & 0x0F; // reset mode and disconnect OC1A pins, keep only prescaler
    GTCCR = (1 << PWM1B) | (1 << COM1B0); // Switch to PWM Mode with OCR1B (PB4) + !OCR1B (PB3) outputs enabled
    OCR1B = OCR1C / 2; // set PWM to 50%
}

/*
 * initialize outputs and use PWM Mode
 * if aUseOutputB == false output frequency at Pin6/5 - PB1/PB0 - OCR1A/!OCR1A
 * else at Pin3/2 - PB4/PB3 - OCR1B/!OCR1B
 */
void toneWithTimer1PWM(uint16_t aFrequency, bool aUseOutputB) {
    uint8_t tPrescaler = 0x01;
    uint16_t tOCR = F_CPU / aFrequency;
    while (tOCR > 0x100 && tPrescaler < 0x0F) {
        tPrescaler++;
        tOCR >>= 1;
    }

    tOCR--; // The frequency of the PWM will be Timer Clock 1 Frequency divided by (OCR1C value + 1).

    if (aUseOutputB) {
        pinModeFastPortB(PB3, OUTPUT);
        pinModeFastPortB(PB4, OUTPUT);
        GTCCR = (1 << PWM1B) | (1 << COM1B0); // PWM Mode with OCR1B (PB4) + !OCR1B (PB3) outputs enabled
        TCCR1 = tPrescaler;
        OCR1B = tOCR / 2; // 50% PWM
    } else {
        pinModeFastPortB(PB1, OUTPUT);
        pinModeFastPortB(PB0, OUTPUT);
        GTCCR = 0;
        TCCR1 = (1 << PWM1A) | (1 << COM1A0) | tPrescaler; // PWM Mode with OCR1A (PB1) + !OCR1A (PB0) outputs enabled
        OCR1A = tOCR / 2; // 50% PWM
    }
    OCR1C = tOCR; // Frequency
}
#endif

/*
 * Code to change Digispark Bootloader clock settings to get the right CPU frequency
 * and to reset Digispark OCCAL tweak.
 * Call it if you want to use the standard ATtiny library, BUT do not call it, if you need Digispark USB functions available for 16 MHz.
 */
void changeDigisparkClock() {
    uint8_t tLowFuse = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    if ((tLowFuse & (~FUSE_CKSEL3 | ~FUSE_CKSEL2 | ~FUSE_CKSEL1 | ~FUSE_CKSEL0 )) == 0x01) { // cannot use ~FUSE_CKSEL0 on right side :-(
#elif defined(__AVR_ATtiny87__) || defined(__AVR_ATtiny167__)
        if ((tLowFuse & (~FUSE_CKSEL3 | ~FUSE_CKSEL2 | ~FUSE_CKSEL1 )) == 0x0E) { // cannot use ~FUSE_CKSEL1 on right side :-(
#endif
        /*
         * Here we have High Frequency PLL Clock ( 16 MHz)
         */
#if (F_CPU == 1000000)
        // Divide 16 MHz Pll clock by 16 for Digispark Boards to get the requested 1 MHz
        clock_prescale_set(clock_div_16);
//        CLKPR = (1 << CLKPCE);  // unlock function
//        CLKPR = (1 << CLKPS2); // 0x04 -> %16
#endif
#if (F_CPU == 8000000)
        // Divide 16 MHz Pll clock by 2 for Digispark Boards to get the requested 8 MHz
        clock_prescale_set(clock_div_2);
//        CLKPR = (1 << CLKPCE);  // unlock function
//        CLKPR = (1 << CLKPS0); // 0x01 -> %2
#endif
    }

    /*
     * Code to reset Digispark OCCAL tweak
     */
#define  SIGRD  5 // needed for boot_signature_byte_get()
    uint8_t tStoredOSCCAL = boot_signature_byte_get(1);
    if (OSCCAL != tStoredOSCCAL) {
#ifdef DEBUG
        uint8_t tOSCCAL = OSCCAL;
        writeString(F("Changed OSCCAL from "));
        writeUnsignedByteHex(tOSCCAL);
        writeString(F(" to "));
        writeUnsignedByteHex(tStoredOSCCAL);
        write1Start8Data1StopNoParity('\n');
#endif
        // retrieve the factory-stored oscillator calibration bytes to revert the digispark OSCCAL tweak
        OSCCAL = tStoredOSCCAL;
    }
}

#endif //  defined (__AVR_ATtiny85__)
