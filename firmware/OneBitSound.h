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
// Module to create analog audio output on a single pin via a 1-bit DAC approach.
// This code uses TIMER1, 1 GPIOTE channel, and 2 PPI channels.
#ifndef ONE_BIT_SOUND_H_
#define ONE_BIT_SOUND_H_

#include <stdint.h>
#include <stddef.h>
#include <app_scheduler.h>


// Size of sound buffer in bytes. 512 is SD sector size.
#define SOUND_BUFFER_SIZE 512

// Number of sound buffers to use. Should be atleast 2 so that 1 can be output to the speaker while the other is being
// read in from the SD card via SPI.
#define SOUND_BUFFER_COUNT 2

// An array of these sound buffers is used to communicate audio samples between the timer ISR and main code.
typedef struct SoundBuffer
{
    // Current index in data array.
    // If >= 512 then this buffer is empty.
    volatile uint32_t    index;
    volatile uint8_t     data[SOUND_BUFFER_SIZE];
} SoundBuffer;

extern SoundBuffer g_soundBuffers[SOUND_BUFFER_COUNT];

void       emptySoundBuffers();
ret_code_t soundInit(uint32_t pin, app_sched_event_handler_t pSoundBufferFillFunc);

#endif // ONE_BIT_SOUND_H_
