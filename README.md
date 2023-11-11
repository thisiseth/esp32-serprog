# esp32-serprog
a minimal flashrom/serprog SPI programmer implementation for esp32 family, inspired by [pico-serprog](https://github.com/stacksmashing/pico-serprog) and [stm32-verprog](https://github.com/dword1511/stm32-vserprog)

portable across at least esp32* mcus, although tested only on ESP32S3 since i have only them

## Configuration

assuming you already have esp-idf environment set up, target board selected etc:

1) run `idf.py menuconfig` and set `ESP System settings -> Channel for console output` to the serial interface you are going to use. 
  this will initialize the corresponding peripheral and bind **stdin** and **stdout**
2) at the beginning of `main.c` check what are the predefined SPI pins for your target MCU / set your own (if configurable).
3) below the SPI config check and modify chosen serial interface configuration (e.g. baud rate if using HW UART)

serial interface to use options are (MCU dependent):

### USB Serial/JTAG

since i have only ESP32S3 i decided to target built-in serial/JTAG first, and it kind of works but there are issues most likely with the serial peripheral itself:

found [a similar issue on the esp32 forum](https://www.esp32.com/viewtopic.php?f=13&t=32209) and some other [ongoing development](https://github.com/espressif/esp-idf/pull/12291) regarding serial buffers

* using flashrom MINGW build on windows: it works reliably for some speed combinations e.g. 1M baud rate 4M SPI rate, except i had to flush the bootloader junk before using the flashrom (e.g. open&close COM port using putty / idf.py monitor)
* using same version flashrom ubuntu WSL2 build and usbipd to forward the usb device: there is no bootloader junk issue, but i always encounter data loss at random point when reading 4MByte flash (takes tens of seconds), so flashrom just hangs waiting for the data
* haven't tested on real linux or macOS

### HW UART

after the serial/JTAG failure i decided to implement the real UART support, since i'm using a devkit board with CH343 USB-UART converter wired to UART0:

* works perfectly as expected
* bottlenecked by UART baud rate
* available on all ESP32 family MCUs
* a board with USB-UART converter (or an external one) is required though
* i used baud rate of 921600 bps for testing

### USB-CDC

not implemented - haven't touched this one yet - enabling USB-CDC on esp32s3 requires burning an eFuse to disconnect the USB serial/JTAG PHY permanently

## Build and flash

run `idf.py flash`

## Use with flashrom

**WARNING**: ESP32 family IO operates at 3.3V, **DO NOT** connect 1.8V flash chips directly, use a levelshifter in that case. 
even the cheapest BSS138-based one will do the job although the SPI speed will be limited to ~200-400k. a proper levelshifter IC like TXS0108E should work fine at all reasonable SPI speeds

*i have successfully reflashed my Steam Deck 1.8V 16MByte BIOS chip using the cheapest aliexpress MOSFET levelshifter and 1.8V LDO at spispeed=250K, although it took like 20 minutes*

SPI speed selection is up to you, depends on the flash chip, wiring etc. - ESP32 should support up to 80MHz in theory

UART baud rate matters only when using HW UART, at least serial/JTAG will always transmit as fast as USB allows

(linux/wsl2)

identify a programmer&chip (no action)
```
flashrom -p serprog:dev=/dev/ttyACM0:921600,spispeed=8M
```

read to file
```
flashrom -p serprog:dev=/dev/ttyACM0:921600,spispeed=8M -c <chip> -r flash.bin
```

erase chip
```
flashrom -p serprog:dev=/dev/ttyACM0:921600,spispeed=8M -c <chip> --erase
```

write from file
```
flashrom -p serprog:dev=/dev/ttyACM0:921600,spispeed=8M -c <chip> -w flash.bin
```

## License

since i have borrowed snippets from the [pico-serprog](https://github.com/stacksmashing/pico-serprog) by [stacksmashing](https://github.com/stacksmashing), the least common denominator is GPL 3.0
