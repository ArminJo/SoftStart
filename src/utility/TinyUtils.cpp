/*
 * TinyUtils.cpp
 *
 *  Created on: 05.03.2018
 *      Author: Armin
 */

#include "TinyUtils.h"
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
    (aMode ? DDRB |= (1 << aOutputPinNumber) : DDRB &= ~(1 << aOutputPinNumber));
}

#if defined(GTCCR)
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
