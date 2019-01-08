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


# Bill of Materials
Description | Quantity | Part Number(s)
------------|----------|---------------
[LM386MMX-1 Low Voltage Audio Amplifier](https://www.digikey.com/products/en?keywords=LM386MMX-1/NOPBCT-ND) | 1 | IC1
[MIC5205 Linear Regulator - 3.3V 150mA SOT23-5](https://www.digikey.com/products/en?keywords=576-1259-1-ND) | 1 | U1
[microSD Socket](https://www.sparkfun.com/products/127) | 1 | JP1
[Dynastream N5150M5CD - nRF51422 Module](https://www.digikey.com/products/en?keywords=N5150M5CD) | 1 | U2
[0.01uF Capacitor - 0805](https://www.adafruit.com/product/441) | 1 | C5
[0.1uF Capacitor - 0805](https://www.adafruit.com/product/441) | 4 | C3, C4, C8, C10
[10uF Capacitor (16V) - Tantalum](https://www.digikey.com/products/en?keywords=478-8235-1-ND) | 4 | C1, C2, C6, C7
[220uF Capacitor (10V) - Tantalum](https://www.digikey.com/products/en?keywords=478-6612-1-ND) | 1 | C9
[10kΩ Resistor - 0805](https://www.adafruit.com/product/441) | 1 | R1
[10Ω Resistor - 0805](https://www.adafruit.com/product/441) | 1 | R6
[1kΩ Resistor - 0805](https://www.adafruit.com/product/441) | 3 | R2, R3, R7
[4.7kΩ Resistor - 0805](https://www.adafruit.com/product/441) | 1 | R4
[10kΩ 11-TurnTrimmer Potentiometer](https://www.digikey.com/products/en?keywords=3223W-1-103ECT-ND) | 1 | R5
[Sliding Switch](https://www.digikey.com/products/en?keywords=EG2481-ND) | 1 | S1


# Schematic
<a href="https://github.com/adamgreen/SortingHat/blob/master/hardware/Schematic.pdf"><img src="https://raw.githubusercontent.com/adamgreen/SortingHat/master/hardware/Schematic.png" alt="Schematic" /></a>


# OSHPark Renderings of PCB
<img src="https://raw.githubusercontent.com/adamgreen/SortingHat/master/images/20190107-PCB_Top.png" alt="OSHPark rendering of PCB Top" width="640" height="842" />
<img src="https://raw.githubusercontent.com/adamgreen/SortingHat/master/images/20190107-PCB_Bottom.png" alt="OSHPark rendering of PCB Bottom" width="640" height="842" />
