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
/* A Bluetooth Low Energy (BLE) enabled Harry Potter Sorting Hat

   This project replaces the original electronics in the Harry Potter Sorting Hat from Barnes and Noble,
   https://www.barnesandnoble.com/w/home-gift-harry-potter-sorting-hat/30008992, with a Nordic nRF5 BLE capable device.

   Pins.h contains the definitions for which pins are connected to which external hardware.

   The user can connect to the device with a BLE capable phone using the Nordic UART service and send 'g', 'r', 'h', or
   's' to select the house and then a series of random clips from the movies will play, ending with an announcement of
   the selected house.

   The sound clips are stored on SD card as .wav files and are played back by pulsing a pin as a 1-bit DAC at 22.05kHz.
   The mouth is connected to a hobby servo instead of the original open-loop DC motor and the position of this servo
   is based on the volume in upcoming audio buffers.

   The sound clips and the code flow to randomly select between them originates from
   https://github.com/gowenrw/arduino_sorting_hat created by @gowenrw, Richard Gowen.

   This sample is heavily influenced by Nordic's BLE UART Service (ble_app_uart) SDK sample.
*/
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <app_button.h>
#include <app_pwm.h>
#include <app_timer.h>
#include <app_uart.h>
#include <ble_advdata.h>
#include <ble_advertising.h>
#include <ble_conn_params.h>
#include <ble_hci.h>
#include <ble_nus.h>
#include <bsp.h>
#include <bsp_btn_ble.h>
#include <nordic_common.h>
#include <nrf.h>
#include <nrf_drv_gpiote.h>
#include <nrf_drv_rng.h>
#include <softdevice_handler.h>
#include "WaveFile.h"
#include "OneBitSound.h"
#include "Pins.h"



// The service database for this device can't be changed at runtime. Must be non-zero for DFU.
#define IS_SRVC_CHANGED_CHARACT_PRESENT 0

#if (NRF_SD_BLE_API_VERSION == 3)
// MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event.
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT
#endif

// Reply when unsupported features are requested.
#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2

// This application doesn't need to act as a central device.
#define CENTRAL_LINK_COUNT              0
// This application is a peripheral and only hosts 1 peripheral link back to a central device.
#define PERIPHERAL_LINK_COUNT           1

#define DEVICE_NAME                     "SortingHat"

// UUID type for the Nordic UART Service (vendor specific).
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN

// The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms).
#define APP_ADV_INTERVAL                64
// The advertising timeout (in units of seconds).
#define APP_ADV_TIMEOUT_IN_SECONDS      180

// Value of the RTC1 PRESCALER register.
#define APP_TIMER_PRESCALER             0
// Size of timer operation queues.
#define APP_TIMER_OP_QUEUE_SIZE         4

// Timings for how often peripheral and central should communicate over the link. The shorter the interval, the more
// data that can be sent over the link but uses more CPU and power resources.
// Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units.
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)
// Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units.
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)
// The number of the connection interval events that the peripheral can ignore before response is required.
#define SLAVE_LATENCY                   0

// Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units.
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)
// Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds).
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)
// Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds).
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)
// Number of attempts before giving up the connection parameter negotiation.
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

#define UART_TX_BUF_SIZE                256
#define UART_RX_BUF_SIZE                256



static ble_nus_t                    g_nordicUartService;
static uint16_t                     g_currBleConnection = BLE_CONN_HANDLE_INVALID;

// UUIDS returned in advertising scan response.
static ble_uuid_t                   g_advertiseUuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};

// Where the main thread should write the next entry in OneBitSound's g_soundBuffers array.
static          size_t              g_soundBufferWrite = 0;

// The number of buffers that have been filled with descending values to ramp down the voltage to the amplifier input.
static          uint32_t            g_soundFinalBuffers = 0;

// The list of wave files to play.
static          const char*         g_wavesToPlay[6];
static          const char**        g_ppCurrentWave = &g_wavesToPlay[5];

// The .wav file handle currently being read for playback.
static          FIL                 g_wavFile;

// PWM instance for servo control uses TIMER2.
APP_PWM_INSTANCE(PWM1, 2);

// The number of clock ticks for mouth fully open and closed position of servo.
static          int32_t             g_closedMouthClockTicks;
static          int32_t             g_openMouthClickTicks;
static          int32_t             g_mouthClockTickRange;



// Forward Function Declarations.
static void pullSoundAndServoOutputPinsLow();
static void initUart(void);
static void uartEventHandler(app_uart_evt_t* pEvent);
static void playHouseAnnouncement(uint8_t houseSelection);
static void soundBufferRead(void* pEventData, uint16_t eventSize);
static void  fillFinalSoundBuffers();
static int32_t calculateMaxVolume(uint8_t* pData, size_t length);
static void enterLowPowerModeUntilNextScheduledEvent();
static void waitForBleHouseChoice();
static void initButtonsAndLeds();
static void enterDeepSleep(void);
static void bspEventHandler(bsp_event_t event);
static void randomSelectionSwitchPressedHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static void initBleStack(void);
static void bleEventHandler(ble_evt_t* pBleEvent);
static void handleBleEventsForApplication(ble_evt_t* pBleEvent);
static void initGapParams(void);
static void initBleUartService(void);
static void nordicUartServiceHandler(ble_nus_t* pNordicUartService, uint8_t* pData, uint16_t length);
static void initBleAdvertising(void);
static void bleAdvertisingEventHandler(ble_adv_evt_t bleAdvertisingEvent);
static void initConnectionParameters(void);
static void connectionParameterEventHandler(ble_conn_params_evt_t* pEvent);
static void connectionParameterErrorHandler(uint32_t errorCode);
static void enterLowPowerModeUntilNextEvent(void);



int main(void)
{
    pullSoundAndServoOutputPinsLow();

    initUart();
    printf("\nHarry Potter Sorting Hat Hack!\n");

    // If the user selected a house via a BLE connection, the selection will have been placed in GPREGRET and then the
    // device reset.
    uint8_t houseSelection = NRF_POWER->GPREGRET;
    NRF_POWER->GPREGRET = 0;
    if (houseSelection != 0)
    {
        playHouseAnnouncement(houseSelection);
    }
    else
    {
        waitForBleHouseChoice();
    }

    return -1;
}

static void pullSoundAndServoOutputPinsLow()
{
    nrf_gpio_cfg_output(SOUND_PIN);
    nrf_gpio_pin_clear(SOUND_PIN);
    nrf_gpio_cfg_output(MOUTH_SERVO_PIN);
    nrf_gpio_pin_clear(MOUTH_SERVO_PIN);
}

static void initUart(void)
{
    uint32_t                     errorCode;
    const app_uart_comm_params_t commParams =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&commParams,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uartEventHandler,
                       APP_IRQ_PRIORITY_LOWEST,
                       errorCode);
    APP_ERROR_CHECK(errorCode);
}

static void uartEventHandler(app_uart_evt_t * pEvent)
{
    uint8_t     dummyByte;

    switch (pEvent->evt_type)
    {
        case APP_UART_DATA_READY:
            // Just eat and discard any received UART bytes at this time.
            app_uart_get(&dummyByte);
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(pEvent->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(pEvent->data.error_code);
            break;

        default:
            break;
    }
}

// The sound clips and the code flow to randomly select between them originates from
// https://github.com/gowenrw/arduino_sorting_hat created by Richard Gowen (@alt_bier)
static void playHouseAnnouncement(uint8_t houseSelection)
{
    uint32_t errorCode;

    printf("Playing house announcement...\n");

    APP_SCHED_INIT(0, 2);

    // Initialize the PWM instance used for the servo control of the hat's mouth.
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(1000000/50, MOUTH_SERVO_PIN);
    pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;

    // Initialize and enable PWM for the servo.
    errorCode = app_pwm_init(&PWM1, &pwm1_cfg, NULL);
    APP_ERROR_CHECK(errorCode);
    app_pwm_enable(&PWM1);

    // Determine the number of clock ticks for mouth fully closed and mouth fully opened.
    float ticksPerMillisecond = (float)app_pwm_cycle_ticks_get(&PWM1) / 20.0f;
    g_closedMouthClockTicks = ticksPerMillisecond * 1.75f;
    g_mouthClockTickRange = ticksPerMillisecond * -0.4f;
    g_openMouthClickTicks = g_closedMouthClockTicks + g_mouthClockTickRange;
    // Amplify the mouth movement a bit.
    g_mouthClockTickRange *= 1.5;

    fatfsInit();

    const char* pHouseClip;
    switch (houseSelection)
    {
    case 'g':
        pHouseClip = "h-gryff1.wav";
        break;
    case 'r':
        pHouseClip = "h-raven1.wav";
        break;
    case 'h':
        pHouseClip = "h-huffl1.wav";
        break;
    case 's':
        pHouseClip = "h-slyth1.wav";
        break;
    default:
        return;
    }

    // Setup Audio File Name Arrays
    // Array of Stalling Audio Clips
    static const char* stallingClips[] =
    {
        "s-ahrigh.wav", // "Ahh... Right then."
        "s-diffic.wav", // "Hmm... Difficult, very difficult."
        "s-itsall.wav", // "You could be great, you know.  It's all here, in your head." -- break into two clips put great to attributes
        "s-ohyes1.wav", // "Oh, Yes."
        "s-righto.wav", // "Hm, right. Ok."
        "s-wheret.wav"  // "But where to put you?"
    };
    // Array of Attribute Audio Clips
    static const char* attributeClips[] =
    {
        "t-courag.wav", // "Plenty of courage, I see."
        "t-notaba.wav", // "Not a bad mind a..." -- needs work, either clip or expand or replace with great clip above
        "t-readym.wav", // "Ahhh yes. A ready mind."
        "t-talent.wav"  // "There's talent."
    };
    // Array of Audio Clips to play just before announcing house.
    static const char* penultimateClips[] =
    {
        "k-better.wav", // "Better be..."
        "k-iknow1.wav", // "I Know!"
        "k-iknowj.wav", // "I know just what to do with you."
        "k-youbel.wav"  // "You belong in..."
    };

    // Using the hardware random number generator since it exists and hard to get unique random selections across
    // CPU resets otherwise.
    uint8_t randNumbers[4];
    uint8_t availableBytes = 0;
    errorCode = nrf_drv_rng_init(NULL);
    APP_ERROR_CHECK(errorCode);
    do
    {
        nrf_drv_rng_bytes_available(&availableBytes);
    } while (availableBytes < sizeof(randNumbers));
    errorCode = nrf_drv_rng_rand(randNumbers, sizeof(randNumbers));
    APP_ERROR_CHECK(errorCode);

    // Determine random audio clip numbers to be played.
    int stallClip1 = randNumbers[0] % ARRAY_SIZE(stallingClips);
    int stallClip2 = (stallClip1 + 1 + (randNumbers[1] % (ARRAY_SIZE(stallingClips)-1))) % ARRAY_SIZE(stallingClips);
    int attributeClip = randNumbers[2] % ARRAY_SIZE(attributeClips);
    int penultimateClip = randNumbers[3] % ARRAY_SIZE(penultimateClips);

    // UNDONE: Remove in final version.
    if (stallClip1 == stallClip2)
    {
        // If my math above is correct then stall clip #1 should never be the same as stall clip #2.
        { __asm volatile ("bkpt #0"); }
    }

    // Queue them up for playback.
    g_wavesToPlay[0] = stallingClips[stallClip1];
    g_wavesToPlay[1] = attributeClips[attributeClip];
    g_wavesToPlay[2] = stallingClips[stallClip2];
    g_wavesToPlay[3] = penultimateClips[penultimateClip];
    g_wavesToPlay[4] = pHouseClip;
    g_wavesToPlay[5] = NULL;
    g_ppCurrentWave = &g_wavesToPlay[0];

    // Call soundBufferRead() synchronously to pre-fill the sound buffers with samples from the .wav files before
    // calling soundInit() to start the audio playback process.
    emptySoundBuffers();
    soundBufferRead(NULL, 0);

    // Start the timer interrupt used for audio playback.
    errorCode = soundInit(SOUND_PIN, soundBufferRead);
    APP_ERROR_CHECK(errorCode);

    for (;;)
    {
        enterLowPowerModeUntilNextScheduledEvent();
    }
}

// Fills in sound buffers with samples from the list of SD card based .wav audio files.
static void soundBufferRead(void * pEventData, uint16_t eventSize)
{
    if (*g_ppCurrentWave == NULL)
    {
        fillFinalSoundBuffers();
        return;
    }
    if (g_wavFile.obj.fs != 0 && f_eof(&g_wavFile))
    {
        f_close(&g_wavFile);
        g_ppCurrentWave++;
    }

    // Using a while to skip over any files that fail to open.
    while (*g_ppCurrentWave && g_wavFile.obj.fs == 0)
    {
        FRESULT result = openWavFile(*g_ppCurrentWave, &g_wavFile);
        if (result != FR_OK)
        {
            f_close(&g_wavFile);
            g_ppCurrentWave++;
        }
    }
    if (g_wavFile.obj.fs == 0)
    {
        fillFinalSoundBuffers();
        return;
    }

    // Fill in as many of the buffer as possible.
    for (size_t i = 0 ; i < ARRAY_SIZE(g_soundBuffers) ; i++)
    {
        SoundBuffer* pBuffer = &g_soundBuffers[g_soundBufferWrite];
        if (pBuffer->index < SOUND_BUFFER_SIZE)
        {
            // This buffer isn't empty.
            break;
        }

        UINT bytesRead;
        FRESULT result = f_read(&g_wavFile, (void*)pBuffer->data, sizeof(pBuffer->data), &bytesRead);
        if (result != FR_OK)
        {
            fillFinalSoundBuffers();
            return;
        }
        if (bytesRead < sizeof(pBuffer->data))
        {
            memset((void*)&pBuffer->data[bytesRead], 0x80, sizeof(pBuffer->data) - bytesRead);
        }
        pBuffer->index = 0;
        g_soundBufferWrite = (g_soundBufferWrite + 1) % ARRAY_SIZE(g_soundBuffers);

        // Set mouth position based on maximum volume of samples in this buffer.
        // Setting the position early when reading the samples should be ok since it gives servo time to move mouth
        // into position.
        uint16_t servoPos = g_closedMouthClockTicks +
                            (calculateMaxVolume((uint8_t*)pBuffer->data, sizeof(pBuffer->data)) *
                             g_mouthClockTickRange / 128);
        if (servoPos < g_openMouthClickTicks)
        {
            // Don't let the mouth attempt to open too far.
            servoPos = g_openMouthClickTicks;
        }
        app_pwm_channel_duty_ticks_set(&PWM1, 0, servoPos);
    }
}

// The purpose of this function is really two-fold:
// * Fill in the last 1/2 second or so of audio with decreasing values to slowly ramp down the analog audio output
//   to 0 volts.
// * After ramping down the voltage, the CPU will be reset to go back into BLE mode so that the user can pick
//   another house for the next iteration.
static void fillFinalSoundBuffers()
{
    if (g_soundFinalBuffers > 22)
    {
        // Can reset the device once a full set of ramping down final buffers has been played
        // to relax the speaker coil gently.
        NVIC_SystemReset();
        return;
    }
    else
    {
        // Ramp the analog sound output down from midpoint to 0V.
        SoundBuffer* pBuffer = &g_soundBuffers[g_soundBufferWrite];
        uint8_t value = 0x80 - 6 * g_soundFinalBuffers;
        if (value > 0x80)
        {
            value = 0;
        }
        memset((void*)pBuffer->data, value, sizeof(pBuffer->data));
        pBuffer->index = 0;
        g_soundBufferWrite = (g_soundBufferWrite + 1) % ARRAY_SIZE(g_soundBuffers);
    }
    g_soundFinalBuffers++;
    app_pwm_channel_duty_ticks_set(&PWM1, 0, g_closedMouthClockTicks);
}

static int32_t calculateMaxVolume(uint8_t* pData, size_t length)
{
    int32_t max = 0;
    while (length-- > 0)
    {
        uint8_t data = *pData++;

        int32_t diff;
        if (data > 0x80)
        {
            diff = data - 0x80;
        }
        else
        {
            diff = 0x80 - data;
        }

        if (diff > max)
        {
            max = diff;
        }
    }
    return max;
}

static void enterLowPowerModeUntilNextScheduledEvent()
{
    __WFE();
    app_sched_execute();
}

static void waitForBleHouseChoice()
{
    printf("Waiting for house selection over BLE connection...\n");

    // Initialize.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    initButtonsAndLeds();
    initBleStack();
    initGapParams();
    initBleUartService();
    initBleAdvertising();
    initConnectionParameters();

    uint32_t errorCode = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(errorCode);

    for (;;)
    {
        enterLowPowerModeUntilNextEvent();
    }
}

static void initButtonsAndLeds()
{

    uint32_t errorCode = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                  APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                  bspEventHandler);
    APP_ERROR_CHECK(errorCode);

    bsp_event_t startupEvent;
    errorCode = bsp_btn_ble_init(NULL, &startupEvent);
    APP_ERROR_CHECK(errorCode);

    // Configure to interrupt if the switch for random house selection is pressed.
    nrf_drv_gpiote_in_config_t config =
    {
        .is_watcher = false,
        .hi_accuracy = false,
        .pull = NRF_GPIO_PIN_PULLUP,
        .sense = NRF_GPIOTE_POLARITY_HITOLO
    };
    errorCode = nrf_drv_gpiote_in_init(RANDOM_SWITCH_PIN, &config, randomSelectionSwitchPressedHandler);
    APP_ERROR_CHECK(errorCode);
    nrf_drv_gpiote_in_event_enable(RANDOM_SWITCH_PIN, true);
}

static void bspEventHandler(bsp_event_t event)
{
    uint32_t errorCode;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            enterDeepSleep();
            break;

        case BSP_EVENT_DISCONNECT:
            errorCode = sd_ble_gap_disconnect(g_currBleConnection, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (errorCode != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(errorCode);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (g_currBleConnection == BLE_CONN_HANDLE_INVALID)
            {
                errorCode = ble_advertising_restart_without_whitelist();
                if (errorCode != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(errorCode);
                }
            }
            break;

        default:
            break;
    }
}

static void enterDeepSleep(void)
{
    uint32_t errorCode = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(errorCode);

    // Prepare wakeup buttons.
    errorCode = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(errorCode);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    errorCode = sd_power_system_off();
    APP_ERROR_CHECK(errorCode);
}

static void randomSelectionSwitchPressedHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    // Using the hardware random number generator to pick a house.
    uint8_t randNumber;
    uint8_t availableBytes = 0;
    uint32_t errorCode = nrf_drv_rng_init(NULL);
    APP_ERROR_CHECK(errorCode);
    do
    {
        nrf_drv_rng_bytes_available(&availableBytes);
    } while (availableBytes < sizeof(randNumber));
    errorCode = nrf_drv_rng_rand(&randNumber, sizeof(randNumber));
    APP_ERROR_CHECK(errorCode);

    // Randomly select the house for the announcement.
    static uint8_t houses[4] = { 'g', 'r', 'h', 's' };
    uint8_t house = houses[randNumber % ARRAY_SIZE(houses)];

    // Remember the selected house before resetting the CPU.
    NRF_POWER->GPREGRET = house;
    NVIC_SystemReset();
}

static void initBleStack(void)
{
    uint32_t errorCode;

    nrf_clock_lf_cfg_t clockConfig = NRF_CLOCK_LFCLKSRC;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clockConfig, NULL);

    ble_enable_params_t defaultBleParams;
    errorCode = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &defaultBleParams);
    APP_ERROR_CHECK(errorCode);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    defaultBleParams.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    errorCode = softdevice_enable(&defaultBleParams);
    APP_ERROR_CHECK(errorCode);

    // Subscribe for BLE events.
    errorCode = softdevice_ble_evt_handler_set(bleEventHandler);
    APP_ERROR_CHECK(errorCode);
}

static void bleEventHandler(ble_evt_t * pBleEvent)
{
    ble_conn_params_on_ble_evt(pBleEvent);
    ble_nus_on_ble_evt(&g_nordicUartService, pBleEvent);
    handleBleEventsForApplication(pBleEvent);
    ble_advertising_on_ble_evt(pBleEvent);
    bsp_btn_ble_on_ble_evt(pBleEvent);
}

static void handleBleEventsForApplication(ble_evt_t * pBleEvent)
{
    uint32_t errorCode;

    switch (pBleEvent->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            errorCode = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(errorCode);
            g_currBleConnection = pBleEvent->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            errorCode = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(errorCode);
            g_currBleConnection = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // This application doesn't support secure pairing.
            errorCode = sd_ble_gap_sec_params_reply(g_currBleConnection, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(errorCode);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            errorCode = sd_ble_gatts_sys_attr_set(g_currBleConnection, NULL, 0, 0);
            APP_ERROR_CHECK(errorCode);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            errorCode = sd_ble_gap_disconnect(pBleEvent->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(errorCode);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            errorCode = sd_ble_gap_disconnect(pBleEvent->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(errorCode);
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            errorCode = sd_ble_user_mem_reply(pBleEvent->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(errorCode);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = pBleEvent->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    errorCode = sd_ble_gatts_rw_authorize_reply(pBleEvent->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(errorCode);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            errorCode = sd_ble_gatts_exchange_mtu_reply(pBleEvent->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(errorCode);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}

static void initGapParams(void)
{
    uint32_t                errorCode;
    ble_gap_conn_params_t   gapPreferredConnectionParams;
    ble_gap_conn_sec_mode_t securityMode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&securityMode);

    errorCode = sd_ble_gap_device_name_set(&securityMode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(errorCode);

    memset(&gapPreferredConnectionParams, 0, sizeof(gapPreferredConnectionParams));

    gapPreferredConnectionParams.min_conn_interval = MIN_CONN_INTERVAL;
    gapPreferredConnectionParams.max_conn_interval = MAX_CONN_INTERVAL;
    gapPreferredConnectionParams.slave_latency     = SLAVE_LATENCY;
    gapPreferredConnectionParams.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    errorCode = sd_ble_gap_ppcp_set(&gapPreferredConnectionParams);
    APP_ERROR_CHECK(errorCode);
}

static void initBleUartService(void)
{
    uint32_t       errorCode;
    ble_nus_init_t nordicUartServiceParams;

    memset(&nordicUartServiceParams, 0, sizeof(nordicUartServiceParams));

    nordicUartServiceParams.data_handler = nordicUartServiceHandler;

    errorCode = ble_nus_init(&g_nordicUartService, &nordicUartServiceParams);
    APP_ERROR_CHECK(errorCode);
}

static void nordicUartServiceHandler(ble_nus_t * pNordicUartService, uint8_t * pData, uint16_t length)
{
    if (length < 1)
    {
        return;
    }

    uint8_t house = tolower(pData[0]);
    if (house == 'g' || house == 'r' || house == 'h' || house == 's')
    {
        // Remember the selected house before resetting the CPU.
        NRF_POWER->GPREGRET = house;
        NVIC_SystemReset();
    }
}

static void initBleAdvertising(void)
{
    uint32_t               errorCode;
    ble_advdata_t          advertisingData;
    ble_advdata_t          scanResponse;
    ble_adv_modes_config_t options;

    // Data to be sent with each advertising cycle.
    memset(&advertisingData, 0, sizeof(advertisingData));
    advertisingData.name_type          = BLE_ADVDATA_FULL_NAME;
    advertisingData.include_appearance = false;
    advertisingData.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    // Data to be sent back to central device if it sends a scan request.
    memset(&scanResponse, 0, sizeof(scanResponse));
    scanResponse.uuids_complete.uuid_cnt = sizeof(g_advertiseUuids) / sizeof(g_advertiseUuids[0]);
    scanResponse.uuids_complete.p_uuids  = g_advertiseUuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    errorCode = ble_advertising_init(&advertisingData, &scanResponse, &options, bleAdvertisingEventHandler, NULL);
    APP_ERROR_CHECK(errorCode);
}

static void bleAdvertisingEventHandler(ble_adv_evt_t bleAdvertisingEvent)
{
    uint32_t errorCode;

    switch (bleAdvertisingEvent)
    {
        case BLE_ADV_EVT_FAST:
            errorCode = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(errorCode);
            break;
        case BLE_ADV_EVT_IDLE:
            enterDeepSleep();
            break;
        default:
            break;
    }
}

static void initConnectionParameters(void)
{
    uint32_t               errorCode;
    ble_conn_params_init_t initParams;

    memset(&initParams, 0, sizeof(initParams));

    initParams.p_conn_params                  = NULL;
    initParams.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    initParams.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    initParams.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    initParams.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    initParams.disconnect_on_fail             = false;
    initParams.evt_handler                    = connectionParameterEventHandler;
    initParams.error_handler                  = connectionParameterErrorHandler;

    errorCode = ble_conn_params_init(&initParams);
    APP_ERROR_CHECK(errorCode);
}

static void connectionParameterEventHandler(ble_conn_params_evt_t * pEvent)
{
    uint32_t errorCode;

    if (pEvent->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        errorCode = sd_ble_gap_disconnect(g_currBleConnection, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(errorCode);
    }
}

static void connectionParameterErrorHandler(uint32_t errorCode)
{
    APP_ERROR_HANDLER(errorCode);
}

static void enterLowPowerModeUntilNextEvent(void)
{
    uint32_t errorCode = sd_app_evt_wait();
    APP_ERROR_CHECK(errorCode);
}




// Break into debugger if any errors/asserts are detected at runtime.
// This handler is called when an application error is encountered in BLE stack or application code.
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    { __asm volatile ("bkpt #0"); }
}
