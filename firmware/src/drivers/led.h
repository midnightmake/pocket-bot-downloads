#pragma once

#include <stdint.h>

// this is the number of LEDs that are chained together and is dictated by the bot hardware itself
#define NUM_LEDS 5

// must be called before any LED functions are called
void ledInit();

// sets the colour of the given LED; valid indices are from 0 to (NUM_LEDS - 1); note that you must
// call ledRefresh() in order to actually make the new colours show
void ledSet(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);

// gets the colour of the given LED; valid indices are from 0 to (NUM_LEDS - 1); note that colours
// could be out of date if we're retrieving colour values that were not ledRefresh()'d
void ledGet(uint8_t index, uint8_t *red, uint8_t *green, uint8_t *blue);

// easy method to turn off all LEDs; still requires a call to ledRefresh()
void ledClear();

// use this to show the colour(s) you set with ledSet()
void ledRefresh();
