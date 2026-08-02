#pragma once

#include <stdint.h>

// - - - prototypes - - - //

void adcInit();
void adcSetChannel(uint8_t channel);
uint16_t adcRead();
uint16_t adcReadMultiple(uint8_t samples);
