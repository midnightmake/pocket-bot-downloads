#pragma once

#include <stdint.h>

// this is the number of line sensors we have and is dictated by the bot hardware itself
#define NUM_LINE_SENSORS 3

// must be called before any other line sensor functions are called
void lineSensorInit();

// return the brightness value perceived by the given line sensors
void lineSensorRead(uint16_t *sensors);
