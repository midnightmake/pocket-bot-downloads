#pragma once

#include <avr/io.h>

#include <stdint.h>

// optional; uses EEPROM values seeded every power cycle to seed the PRNG
void xabcInit();

// reseed with a given value
void xabcReseed(uint8_t value);

// returns the next PRNG in the sequence
uint8_t xabcGet();
