# esp32-serprog
a minimal flashrom/serprog SPI programmer implementation for esp32 family, inspired by [pico-serprog](https://github.com/stacksmashing/pico-serprog) and [stm32-verprog](https://github.com/dword1511/stm32-vserprog)


portable across at least esp32* mcus, although tested (&implemented) only for ESP32S3 since i have only them


for MCUs without serial/JTAG peripheral, UART or USB-CDC should be used


since i have only ESP32S3 i decided to target built-in serial/JTAG and it kind of works but there are issues most likely with the serial peripheral itself:

* using flashrom MINGW build on windows i haven't encountered any issues except i had to flush the bootloader junk before using the flashrom (e.g. open&close COM port using putty / idf.py monitor)
* using same version flashrom ubuntu WSL2 build and usbipd to forward the usb device there is no bootloader junk issue, but i always encounter data loss at random point when reading 4MByte flash (takes tens of seconds), so flashrom just hangs waiting for the data - found [a similar issue on the esp32 forum](https://www.esp32.com/viewtopic.php?f=13&t=32209) and some other ongoing development regarding serial buffers
* haven't tested on real linux or macOS

