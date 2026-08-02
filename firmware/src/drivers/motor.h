#pragma once

#include <stdint.h>

// must be called before any other motors functions are called
void motorInit();

// set to 1 if you want to use hard braking when motor speeds are set to zero; set to 0 if you want to coast to a stop instead;
// default value is 1
void motorUseHardBrake(uint8_t useHardBrake);

// set left motor speed; valid range is -100 to 100;
// default value is 0
void motorLeft(int8_t speed);

// set right motor speed; valid range is -100 to 100
// default value is 0
void motorRight(int8_t speed);

// if your motors are a little sloppy and don't run at the same speeds, you can use these to tweak your motor scaling so that they do;
// note that you should provide values less than 1.0, not greater, since increasing motor speeds past maximum makes no sense
void motorScaleLeft(float scale);
void motorScaleRight(float scale);
