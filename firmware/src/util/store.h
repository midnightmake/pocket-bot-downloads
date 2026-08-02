#pragma once

// the purpose of this file is to define addresses for EEPROM reads/writes in a single place so that drivers
// or other functions don't step on each others' toes

// XABC pseudo-random number generator seed
#define STORE_XABC_SEED_ADDR 0x0

// storage for line calibration values; note we need two bytes for each of those values
// [note]: these are arranged in such a way that minimums and maximums can be written
//         in a single write each
#define STORE_MIN_READING_SENSOR_1_ADDR 0x10
#define STORE_MIN_READING_SENSOR_2_ADDR 0x12
#define STORE_MIN_READING_SENSOR_3_ADDR 0x14
#define STORE_MAX_READING_SENSOR_1_ADDR 0x16
#define STORE_MAX_READING_SENSOR_2_ADDR 0x18
#define STORE_MAX_READING_SENSOR_3_ADDR 0x1a

// where our unique robot ID is stored
#define STORE_ID_ADDR 0x20
