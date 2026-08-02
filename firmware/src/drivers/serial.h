#pragma once

#include <stdint.h>

// perform any initialization required of the serial comms
void serialInit();

// configure baud rates
void serialSetBaudRate(uint32_t baudRate);

// formatted serial output
void serialTXRaw(uint8_t *data, uint16_t len);
void serialTX(const char *format, ...);

// serial RX
uint8_t serialRXAvailable();
uint8_t serialRX();
