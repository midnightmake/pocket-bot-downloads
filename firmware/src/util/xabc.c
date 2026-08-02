#include "util/xabc.h"
#include "util/store.h"

#include <avr/io.h>
#include <avr/eeprom.h>

#include <stdint.h>

static uint8_t x, a, b, c;

void xabcInit()
{
	// load a value from EEPROM to seed things with
	a = eeprom_read_byte(STORE_XABC_SEED_ADDR);

	// increment and write it back so the next time we power cycle we'll have something new
	a ++;
	eeprom_write_byte(STORE_XABC_SEED_ADDR, a);
}

void xabcReseed(uint8_t value)
{
	a = value;
}

// [adapted from: https://eternityforest.com/doku/doku.php?id=tech:the_xabc_random_number_generator]
// return a pseudorandom number; repeats the sequence every power cycle
uint8_t xabcGet()
{
	x ++;						// x is incremented every round and is not affected by any other variable
	a = (a ^ c ^ x);			// note the mix of addition and XOR
	b = (b + a);				// and the use of very few instructions
	c = ((c + (b >> 1)) ^ a);	// the right shift is to ensure that high-order bits from b can affect
	return c;					// low order bits of other variables
}
