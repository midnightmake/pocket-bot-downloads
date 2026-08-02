#include "drivers/button.h"

#include <stdint.h>

#include <avr/io.h>

// - - - defines - - - //

// button input pin
#define BUTTON_PORT PORTB
#define BUTTON_INPUT PINB
#define BUTTON_DIR DDRB
#define BUTTON_PIN 0

// - - - implementations - - - //

void buttonInit()
{
	// set our pin as an input with internal pull-up enabled
	BUTTON_DIR &= ~(1 << BUTTON_PIN);
	BUTTON_PORT |= (1 << BUTTON_PIN);
}

uint8_t buttonDown()
{
	return !(BUTTON_INPUT & (1 << BUTTON_PIN));
}
