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
#include "OneBitSound.h"
#include <app_util_platform.h>
#include <nrf_drv_gpiote.h>
#include <nrf_drv_ppi.h>
#include <nrf_drv_timer.h>



// Number of 16MHz clock cycles to make one 22.05kHz period.
#define SOUND_PWM_PERIOD (363*2)

// Number of 16MHz clock cycles in TIMER1_IRQHandler() overhead.
#define SOUND_PWM_ISR_OVERHEAD 107

// Maximum PWM duty cycle.
#define SOUND_PWM_MAX_PERIOD (SOUND_PWM_PERIOD - SOUND_PWM_ISR_OVERHEAD)



static app_sched_event_handler_t    g_pSoundBufferFillFunc;
static const    nrf_drv_timer_t     g_soundTimer = NRF_DRV_TIMER_INSTANCE(1);
static          nrf_ppi_channel_t   g_soundToggleLoPpiChannel;
static          nrf_ppi_channel_t   g_soundToggleHiPpiChannel;

SoundBuffer                         g_soundBuffers[SOUND_BUFFER_COUNT];
static          size_t              g_soundBufferRead = 0;

static          uint32_t            g_soundNextSample;



// Forward Function Declarations.
static void     timer1Handler(nrf_timer_event_t eventType, void* pvContext);
static uint32_t calcTimeForDutyCycle(uint32_t  duty);



void       emptySoundBuffers()
{
    for (size_t i = 0 ; i < ARRAY_SIZE(g_soundBuffers) ; i++)
    {
        g_soundBuffers[i].index = sizeof(g_soundBuffers[0].data);
    }
}

ret_code_t soundInit(uint32_t pin, app_sched_event_handler_t pSoundBufferFillFunc)
{
    g_pSoundBufferFillFunc = pSoundBufferFillFunc;

    // Make sure that the required PPI and GPIOTE peripherals are initialized.
    uint32_t err_code = nrf_drv_ppi_init();
    if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_MODULE_ALREADY_INITIALIZED))
    {
        return NRF_ERROR_NO_MEM;
    }

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        if (err_code != NRF_SUCCESS)
        {
            return NRF_ERROR_INTERNAL;
        }
    }

    // Using 1 GPIOTE event to toggle the 1-bit DAC pin high and low at the required intervals.
    nrf_drv_gpiote_out_config_t toggleConfig = GPIOTE_CONFIG_OUT_TASK_TOGGLE(0);
    err_code = nrf_drv_gpiote_out_init((nrf_drv_gpiote_pin_t)pin, &toggleConfig);
    if (err_code != NRF_SUCCESS)
    {
        return NRF_ERROR_NO_MEM;
    }

    // Use 2 PPI channels.
    // One to toggle the pin low at the end of a PWM cycle and another to toggle it high to result in the desired
    // duty cycle time.
    if (nrf_drv_ppi_channel_alloc(&g_soundToggleLoPpiChannel) != NRF_SUCCESS)
    {
        return NRF_ERROR_NO_MEM;
    }
    if (nrf_drv_ppi_channel_alloc(&g_soundToggleHiPpiChannel) != NRF_SUCCESS)
    {
        return NRF_ERROR_NO_MEM;
    }

    uint32_t gpioteTask = nrf_drv_gpiote_out_task_addr_get(pin);
    uint32_t compare0Event = nrf_drv_timer_compare_event_address_get(&g_soundTimer, NRF_TIMER_CC_CHANNEL0);
    uint32_t compare1Event = nrf_drv_timer_compare_event_address_get(&g_soundTimer, NRF_TIMER_CC_CHANNEL1);
    nrf_drv_ppi_channel_assign(g_soundToggleLoPpiChannel, compare0Event, gpioteTask);
    nrf_drv_ppi_channel_assign(g_soundToggleHiPpiChannel, compare1Event, gpioteTask);

    // Initialize timer.
    nrf_drv_timer_config_t timerConfig  = {
        .frequency          = NRF_TIMER_FREQ_16MHz,
        .mode               = NRF_TIMER_MODE_TIMER,
        .bit_width          = NRF_TIMER_BIT_WIDTH_16,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGHEST,
        .p_context          = NULL
    };
    err_code = nrf_drv_timer_init(&g_soundTimer, &timerConfig, timer1Handler);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    nrf_drv_timer_clear(&g_soundTimer);
    nrf_drv_timer_extended_compare(&g_soundTimer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   SOUND_PWM_MAX_PERIOD,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK | NRF_TIMER_SHORT_COMPARE0_STOP_MASK,
                                   true);
    nrf_drv_timer_compare(&g_soundTimer,
                          NRF_TIMER_CC_CHANNEL1,
                          calcTimeForDutyCycle(0),
                          false);
    g_soundNextSample = calcTimeForDutyCycle(0);

    nrf_drv_gpiote_out_task_enable((nrf_drv_gpiote_pin_t)pin);
    nrf_drv_ppi_channel_enable(g_soundToggleLoPpiChannel);
    nrf_drv_ppi_channel_enable(g_soundToggleHiPpiChannel);
    nrf_drv_timer_clear(&g_soundTimer);
    nrf_drv_timer_enable(&g_soundTimer);

    return NRF_SUCCESS;
}

static uint32_t calcTimeForDutyCycle(uint32_t  duty)
{
    return (SOUND_PWM_MAX_PERIOD-1) - ((duty * (SOUND_PWM_MAX_PERIOD-1)) / 255);
}

void TIMER1_IRQHandler(void)
{
    nrf_timer_cc_write(g_soundTimer.p_reg, NRF_TIMER_CC_CHANNEL1, g_soundNextSample);
    nrf_timer_event_clear(g_soundTimer.p_reg, NRF_TIMER_EVENT_COMPARE0);
    nrf_timer_task_trigger(g_soundTimer.p_reg, NRF_TIMER_TASK_START);

    SoundBuffer* pBuffer = &g_soundBuffers[g_soundBufferRead];
    if (pBuffer->index == SOUND_BUFFER_SIZE)
    {
        g_soundNextSample = calcTimeForDutyCycle(0x80);
        return;
    }
    g_soundNextSample = calcTimeForDutyCycle(pBuffer->data[pBuffer->index++]);
    if (pBuffer->index == SOUND_BUFFER_SIZE)
    {
        g_soundBufferRead = (g_soundBufferRead + 1) % ARRAY_SIZE(g_soundBuffers);
        app_sched_event_put(NULL, 0, g_pSoundBufferFillFunc);
    }
}

// This function is never actually called because I have my own implementation of TIMER1_IRQHandler()) above
// but a valid pointer needs to be passed into nrf_drv_timer_init() anyway.
static void timer1Handler(nrf_timer_event_t eventType, void* pvContext)
{
}
