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
// Module for initializing the FAT file system on a SD card (interfaced via SPI) so that .wav sound files can be opened
// and read.
#ifndef WAFE_FILE_H_
#define WAFE_FILE_H_

#include <ff.h>

void    fatfsInit();
FRESULT openWavFile(const char* pFilename, FIL* pFile);

#endif // WAFE_FILE_H_
