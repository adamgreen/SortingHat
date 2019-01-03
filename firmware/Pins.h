/*  Copyright (C) 2019  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
// Defines the pins used for connecting to external hardware.
#ifndef PINS_H_
#define PINS_H_

// Pin to use as output for 1-bit DAC (PWM) sound.
#define SOUND_PIN           1

// Pin to use as output for the mouth servo.
#define MOUTH_SERVO_PIN     2

// Pin to use as input for the random selection switch.
#define RANDOM_SWITCH_PIN   3

// Pins used for interfacing to a SD Card via SPI.
#define SDC_CS_PIN          24
#define SDC_MOSI_PIN        25
#define SDC_MISO_PIN        28
#define SDC_SCK_PIN         29

#endif // PINS_H_
