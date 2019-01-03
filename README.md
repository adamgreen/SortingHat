# BLE Harry Potter Sorting Hat
This project replaces the original electronics in a [Harry Potter Sorting Hat](https://www.barnesandnoble.com/w/home-gift-harry-potter-sorting-hat/30008992) with a Nordic nRF5 BLE capable device.


## Legal Notes
* This Harry Potter Sorting Hat upgrade is created by a Harry Potter fan for other such fans. It is not sponsored or endorsed by Warner Bros. Entertainment or J.K. Rowling.
* Materials from the Harry Potter series of films are the intellectual property of Warner Bros. Entertainment.


## Project Overview
The user can connect to the device with a BLE capable phone using the Nordic UART service and send **g**, **r**, **h**, or **s** to select the house and then a series of random clips from the movies will play, ending with an announcement of the selected house.

The sound clips are stored on SD card as .wav files and are played back by pulsing a pin as a 1-bit DAC at 22.05kHz. The mouth is connected to a hobby servo instead of the original open-loop DC motor and the position of this servo is based on the volume in upcoming audio buffers.

## Credits
The sound clips and the code flow to randomly select between them originates from the https://github.com/gowenrw/arduino_sorting_hat repository created by [@gowenrw](https://github.com/gowenrw).

This sample is heavily influenced by Nordic's BLE UART Service (ble_app_uart) SDK sample.


## Firmware Build
I have only tested the build on macOS but it might work on other Posix operating systems too.

**Note:** Building the firmware and deploying it to the nRF51 requires the **nrfjprog** tool. The archive for your OS can be downloaded from [here](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF5-Command-Line-Tools). You will also need a SEGGER J-Link JTAG/SWD Debugger. If your projects are of a non-commercial nature, you can use this [low cost module available from Adafruit](https://www.adafruit.com/product/3571).

* `make sdk` will download and install the required Nordic SDK. This should only need to be done once.
* `make audio` will generate the .wav files to be placed in the root folder of the SD card.
* `make flash_softdevice` will install the required Nordic SoftDevice on the nRF51422 microcontroller using the J-Link debugger. This will typically only need to be done once for each microcontroller.
* `make all` will build the hat firmware.
* `make flash` will build the hat firmware and deploy it using the J-Link debugger.
