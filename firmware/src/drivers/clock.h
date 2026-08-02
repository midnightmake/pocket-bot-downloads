#pragma once

#include <avr/io.h>

// must be called before any clock functions are called
void clockInit();

// return the current time in milliseconds
uint32_t clockMillis();

// return the current time in microseconds
uint32_t clockMicros();

// disable or enable the clock so other functions can use the timer
void clockPause();
void clockResume();
