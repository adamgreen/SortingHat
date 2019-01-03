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
#include "WaveFile.h"
#include "Pins.h"
#include <stdint.h>
#include <diskio_blkdev.h>
#include <nrf_block_dev_sdc.h>



// Structures used to represent the data/chunks in a .wav file.
typedef struct WaveHeader
{
    union
    {
        uint8_t  chunkIdBytes[4];
        uint32_t chunkId;
    };
    uint32_t chunkSize;
    union
    {
        uint8_t formatBytes[4];
        uint32_t format;
    };
} WaveHeader;

typedef struct WaveSubchunkHeader
{
    union
    {
        uint8_t  subchunkIdBytes[4];
        uint32_t subchunkId;
    };
    uint32_t subchunkSize;
} WaveSubchunkHeader;

typedef struct WaveFormatChunk
{
    uint16_t audioFormat; // 1 = PCM
    uint16_t numChannels;
    uint32_t sampleRate;
    uint32_t byteRate;
    uint16_t blockAlign;
    uint16_t bitsPerSample;
} WaveFormatChunk;



static FATFS g_fatFileSystem;

// This block device is used to read/write from SD cards.
NRF_BLOCK_DEV_SDC_DEFINE(
        g_blockDeviceSDC,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);



// Forward Function Declarations
static FRESULT findMono22050Hz8bitDataChunk(const char* pFilename, FIL* pFile);
static FRESULT processFormatChunk(const char* pFilename, FIL* pFile, uint32_t size);



void fatfsInit()
{
    FRESULT result;

    // Initialize FATFS disk I/O interface by providing the block device.
    static diskio_blkdev_t drives[] =
    {
        DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(g_blockDeviceSDC, block_dev), NULL)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    DSTATUS diskState = STA_NOINIT;
    for (uint32_t retries = 3; retries && diskState; --retries)
    {
        diskState = disk_initialize(0);
    }
    if (diskState)
    {
        printf("error: SD card initialization failed.\n");
        return;
    }

    result = f_mount(&g_fatFileSystem, "", 1);
    if (result)
    {
        printf("error: SD card mount failed.\n");
        return;
    }
}

FRESULT openWavFile(const char* pFilename, FIL* pFile)
{
    FRESULT result = FR_OK;

    result = f_open(pFile, pFilename, FA_READ | FA_OPEN_EXISTING);
    if (result != FR_OK)
    {
        printf("Unable to open %s\n", pFilename);
        return result;
    }

    WaveHeader waveHeader;
    UINT       bytesRead;
    result = f_read(pFile, &waveHeader, sizeof(waveHeader), (UINT *) &bytesRead);
    if (result != FR_OK || bytesRead != sizeof(waveHeader))
    {
        printf("Failed to read wave header for %s\n.", pFilename);
        result = result == FR_OK ? FR_INT_ERR : result;
        goto Error;
    }
    if (0 != memcmp(waveHeader.chunkIdBytes, "RIFF", 4) ||
        0 != memcmp(waveHeader.formatBytes, "WAVE", 4))
    {
        printf("Failed to find valid wave file header in %s\n", pFilename);
        goto Error;
    }

    result = findMono22050Hz8bitDataChunk(pFilename, pFile);
    if (result != FR_OK)
    {
        goto Error;
    }

    result = FR_OK;
Error:
    if (result != FR_OK)
    {
        f_close(pFile);
    }
    return result;
}

static FRESULT findMono22050Hz8bitDataChunk(const char* pFilename, FIL* pFile)
{
    bool validFormat = false;
    FRESULT result = FR_OK;
    UINT bytesRead = 0;
    WaveSubchunkHeader subchunkHeader;

    while (1)
    {
        result = f_read(pFile, &subchunkHeader, sizeof(subchunkHeader), &bytesRead);
        if (result != FR_OK || bytesRead != sizeof(subchunkHeader))
        {
            goto Error;
        }
        if (0 == memcmp(subchunkHeader.subchunkIdBytes, "fmt ", 4))
        {
            result = processFormatChunk(pFilename, pFile, subchunkHeader.subchunkSize);
            if (result != FR_OK)
            {
                goto Error;
            }
            validFormat = true;
        }
        else if (0 == memcmp(subchunkHeader.subchunkIdBytes, "data", 4))
        {
            break;
        }
        else
        {
            result = f_lseek(pFile, f_tell(pFile) + subchunkHeader.subchunkSize);
            if (result != FR_OK)
            {
                goto Error;
            }
        }
    }

    result = FR_OK;
Error:
    if (!validFormat)
    {
        printf("error: Didn't find valid WAV file format in %s\n", pFilename);
        result = FR_INT_ERR;
    }
    return result;
}

static FRESULT processFormatChunk(const char* pFilename, FIL* pFile, uint32_t size)
{
    FRESULT result = FR_OK;
    UINT bytesRead = 0;
    WaveFormatChunk format;

    if (size != sizeof(format))
    {
        printf("error: PCM format of %s should be %u bytes long but was %lu bytes\n", pFilename, sizeof(format), size);
        return FR_INT_ERR;
    }
    result = f_read(pFile, &format, sizeof(format), &bytesRead);
    if (result != FR_OK || bytesRead != sizeof(format))
    {
        return result != FR_OK ? result : FR_INT_ERR;
    }
    if (format.audioFormat != 1)
    {
        printf("error: Format of %s was %u, not 1 for PCM", pFilename, format.audioFormat);
        return FR_INT_ERR;
    }
    if (format.numChannels != 1)
    {
        printf("error: Unexpected channel count of %u in %s. Expected mono.", format.numChannels, pFilename);
        return FR_INT_ERR;
    }
    if (format.sampleRate != 22050)
    {
        printf("error: Unexpected sample rate of %lu in %s. Expected 22.05kHz.", format.sampleRate, pFilename);
        return FR_INT_ERR;
    }
    if (format.bitsPerSample != 8)
    {
        printf("error: Unexpected bits per sample of %u in %s. Expected 8-bits.", format.bitsPerSample, pFilename);
        return FR_INT_ERR;
    }

    return FR_OK;
}
