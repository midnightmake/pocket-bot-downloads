#pragma once

#include <stdint.h>

// must be called before any button functions are called
void buttonInit();

// use to determine if a button is pressed down; be mindful that debouncing may be necessary
uint8_t buttonDown();
