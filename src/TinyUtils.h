/*
 * TinyUtils.h
 *
 *  Created on: 05.03.2018
 *      Author: Armin
 */

//
// ATMEL ATTINY85
//
//                                         +-\/-+
//        PCINT5/!RESET/ADC0/dW (D5) PB5  1|    |8  Vcc
// PCINT3/XTAL1/CLKI/!OC1B/ADC3 (D3) PB3  2|    |7  PB2 (D2) SCK/USCK/SCL/ADC1/T0/INT0/PCINT2
//  PCINT4/XTAL2/CLKO/OC1B/ADC2 (D4) PB4  3|    |6  PB1 (D1) MISO/DO/AIN1/OC0B/OC1A/PCINT1 / TX Debug output
//                                   GND  4|    |5  PB0 (D0) MOSI/DI/SDA/AIN0/OC0A/!OC1A/AREF/PCINT0
//                                         +----+

#ifndef TINYUTILS_H_
#define TINYUTILS_H_

#include <Arduino.h>
#include <avr/io.h>

#if (F_CPU == 1000000)
#define TIMER0_CLOCK_DIVIDER_FOR_64_MICROS ((1 << CS01) | (1 << CS00))

#define TIMER1_CLOCK_DIVIDER_FOR_8_MICROS (1 << CS12)
#define TIMER1_CLOCK_DIVIDER_FOR_4_MICROS ((1 << CS11) | (1 << CS10))
#define TIMER1_CLOCK_DIVIDER_FOR_2_MICROS (1 << CS11)
#define TIMER1_CLOCK_DIVIDER_FOR_1_MICRO (1 << CS10)
#endif

#if (F_CPU == 8000000)
#define TIMER0_CLOCK_DIVIDER_FOR_128_MICROS ((1 << CS02) | (1 << CS00))

#define TIMER1_CLOCK_DIVIDER_FOR_8_MICROS ((1 << CS12) | (1 << CS11)| (1 << CS10))
#define TIMER1_CLOCK_DIVIDER_FOR_4_MICROS ((1 << CS12) | (1 << CS11))
#define TIMER1_CLOCK_DIVIDER_FOR_2_MICROS ((1 << CS12) | (1 << CS10))
#define TIMER1_CLOCK_DIVIDER_FOR_1_MICRO (1 << CS12)
#endif

/*
 * Only suitable for constant values
 * Loading of value adds 2 extra cycles (check .lss file for exact timing)
 *
 * Only multiple of 4 cycles are possible. Last loop is only 3 cycles.
 * 1 -> 3(+2) cycles
 * 2 -> 7(+2) cycles
 * 3 -> 11(+2) cycles
 * 4 -> 15(+2) cycles
 * 5 -> 19(+2) cycles
 * 6 -> 23(+2) cycles
 */
inline void delay4CyclesInlineExact(uint16_t a4Microseconds) {
    // the following loop takes 4 cycles (4 microseconds  at 1MHz) per iteration
    __asm__ __volatile__ (
            "1: sbiw %0,1" "\n\t"    // 2 cycles
            "brne .-4" : "=w" (a4Microseconds) : "0" (a4Microseconds)// 2 cycles
    );
}

void toneWithTimer1PWM(uint16_t aFrequency, bool aUseOutputB = false);

#endif /* TINYUTILS_H_ */
