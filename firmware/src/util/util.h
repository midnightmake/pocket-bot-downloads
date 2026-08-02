#pragma once

#include <stdint.h>

// thanks StackOverflow! https://stackoverflow.com/a/68676196
#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))
#define clamp(x, lower, upper) (min((upper), max((x), (lower))))

// flash a "ready" LED and block until user presses and releases the button
void utilWaitForButtonPress(uint8_t index);

// show a "warning" LED if the battery voltage is getting low, and shut down the bot if it goes below mininum safe levels
void utilHandleLowPower();

// given an 8-bit hue position, calculate RGB values for it
void utilHue2RGB(uint8_t hue, uint8_t *red, uint8_t *green, uint8_t *blue);
