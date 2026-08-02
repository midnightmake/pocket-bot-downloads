#!/bin/bash
avrdude -B 4kHz -c avrispv2 -P usb -p m328p -U lfuse:w:0xce:m -U hfuse:w:0xd1:m -U efuse:w:0xff:m -vvvv
avrdude -B 1MHz -p m328p -P usb -c avrispv2 -U flash:w:Release/pocket-bot-firmware.hex -vvvv
