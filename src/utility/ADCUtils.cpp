/*
 * ADCUtils.cpp
 *
 *  Created on: 23.02.2018
 *      Author: Armin
 */

#include "ADCUtils.h"

// Union to speed up the combination of low and high bytes to a word
// it is not optimal since the compiler still generates 2 unnecessary moves
// but using  -- value = (high << 8) | low -- gives 5 unnecessary instructions
union Myword {
    struct {
        uint8_t LowByte;
        uint8_t HighByte;
    } byte;
    uint16_t UWord;
    int16_t Word;
    uint8_t * BytePointer;
};

uint16_t readADCChannel(uint8_t aChannelNumber) {
    Myword tUValue;
    ADMUX = aChannelNumber | DEFAULT; // DEFAULT = VCC

//  ADCSRB = 0; // free running mode  - is default
    ADCSRA = ((1 << ADEN) | (1 << ADSC) | (1 << ADIF) | ADC_PRESCALE);

// wait for free running conversion to finish
    loop_until_bit_is_set(ADCSRA, ADIF);

// Get value
    tUValue.byte.LowByte = ADCL;
    tUValue.byte.HighByte = ADCH;
    return tUValue.UWord;
}

uint16_t readADCChannelWithReference(uint8_t aChannelNumber, uint8_t aVoltageReference) {
    Myword tUValue;
    ADMUX = aChannelNumber | aVoltageReference; // DEFAULT = VCC

// ADCSRB = 0; // free running mode if ADATE is 1 - is default
    // ADSC-StartConversion ADIF-Reset Interrupt Flag
    ADCSRA = ((1 << ADEN) | (1 << ADSC) | (1 << ADIF) | ADC_PRESCALE);

// wait for single conversion to finish
    loop_until_bit_is_set(ADCSRA, ADIF);

// Get value
    tUValue.byte.LowByte = ADCL;
    tUValue.byte.HighByte = ADCH;
    return tUValue.UWord;
}

uint16_t readADCChannelWithOversample(uint8_t aChannelNumber, uint8_t aOversampleExponent) {
    return readADCChannelWithReferenceOversample(aChannelNumber, DEFAULT, aOversampleExponent);
}

uint16_t readADCChannelWithReferenceOversample(uint8_t aChannelNumber, uint8_t aVoltageReference, uint8_t aOversampleExponent) {
    Myword tUValue;
    uint16_t tSumValue = 0;
    ADMUX = aChannelNumber | aVoltageReference;

// ADCSRB = 0; // free running mode if ADATE is 1 - is default
    // ADSC-StartConversion ADATE-AutoTriggerEnable ADIF-Reset Interrupt Flag
    ADCSRA = ((1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIF) | ADC_PRESCALE);

    for (uint8_t i = 0; i < (1 << aOversampleExponent); i++) {
// wait for free running conversion to finish
        loop_until_bit_is_set(ADCSRA, ADIF);

// Get value
        tUValue.byte.LowByte = ADCL;
        tUValue.byte.HighByte = ADCH;
// without "| (1 << ADSC)" it does not work - undocumented feature???
        ADCSRA |= (1 << ADIF) | (1 << ADSC); // clear bit to recognize next conversion has finished
        tSumValue += tUValue.Word;
    }
    ADCSRA &= ~(1 << ADATE); // Disable auto-triggering (free running mode)
    return (tSumValue >> aOversampleExponent);
}

float getVCCVoltage(void) {
    // use AVCC with external capacitor at AREF pin as reference
    float tVCC = readADCChannelWithReferenceOversample(ADC_1_1_VOLT_CHANNEL_MUX, DEFAULT, 2);
    return ((1024 * 1.1) / tVCC);
}

float getTemperature(void) {
    // use internal 1.1 Volt as reference
    float tTemp = (readADCChannelWithReferenceOversample(ADC_TEMPERATURE_CHANNEL_MUX, INTERNAL, 2) - 317);
    return (tTemp / 1.22);
}

