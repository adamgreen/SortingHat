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
#define SOUND_PIN           15

// Pin to use as output for the mouth servo.
#define MOUTH_SERVO_PIN     6

// Pin to use as input for the random selection switch.
#define RANDOM_SWITCH_PIN   8

// Pins used for interfacing to a SD Card via SPI.
#define SDC_CS_PIN          2
#define SDC_MOSI_PIN        23
#define SDC_MISO_PIN        0
#define SDC_SCK_PIN         30

// Pins used for interfacing to UART with no flow control.
#define UART_RX_PIN         3
#define UART_TX_PIN         5


// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_50_PPM}

#endif // PINS_H_
