#pragma once

#include <stdint.h>

// must be called before using any of the other battery functions
void batteryInit();

// perform a read on the battery voltage and store it internally
void batteryRead();

// return the result of the last call to batteryRead(); result is undefined is batteryRead() was not called
uint16_t batteryGetMillivolts();
