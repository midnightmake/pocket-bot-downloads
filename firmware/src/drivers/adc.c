#include "drivers/adc.h"

#include <avr/io.h>

void adcInit()
{
	ADCSRA |= (1 << ADEN);			// enable
	ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);
	// [voltage reference is AREF by default]
}

void adcSetChannel(uint8_t channel)
{
	// wait until any conversion is finished (it should be)
	while(ADCSRA & (1 << ADSC));

	// clear channel selection
	ADMUX &= 0xf0;

	// if channel is valid, select it; yes, we have 9 channels, not 8
	if(channel >= 0 && channel <= 8)
	{
		ADMUX |= channel;
	}
}

uint16_t adcRead()
{
	uint16_t result = 0;

	// start a conversion and wait for it to finish
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1 << ADSC));

	// store result
	result = ADC;

	// and we're done
	return result;
}

uint16_t adcReadMultiple(uint8_t samples)
{
	uint32_t accum;
	uint8_t i;

	// take multiple samples
	accum = 0;
	for(i = 0; i < samples; ++ i)
	{
		accum += adcRead();
	}

	// take average
	return accum / (uint32_t)samples;
}
