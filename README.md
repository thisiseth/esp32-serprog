# esp32-serprog
a minimal flashrom/serprog SPI programmer implementation for esp32 family, inspired by [pico-serprog](https://github.com/stacksmashing/pico-serprog) and [stm32-vserprog](https://github.com/dword1511/stm32-vserprog)

## Features

Use your ESP32 board as a [flashrom](https://flashrom.org/) compatible SPI flash programmer

* Should be compatible with all ESP32 family MCUs
* Uses ESP32 HW SPI peripheral capable of up to 80 MHz clock rate
* Multiple connection options: USB Serial/JTAG, HW UART, TCP (Wi-Fi)<sup>1</sup>

<sup>1</sup> - flashrom serprog TCP capability is limited to non-windows OSes and requires flashrom version 1.4.0 or later 

## Configuration

assuming you already have esp-idf environment set up, target board selected etc:

1) run `idf.py menuconfig` and set `ESP System settings -> Channel for console output` to the serial interface you are going to use. 
  this will initialize the corresponding peripheral and bind **stdin** and **stdout**
2) at the beginning of `main.c` check what are the predefined SPI pins for your target MCU / set your own (if configurable).
3) below the SPI config check and modify chosen serial interface configuration (e.g. baud rate if using HW UART)

serial interface to use options are (MCU dependent):

### USB Serial/JTAG

older versions of esp-idf had issues with USB Serial/JTAG reliability, 
but as of esp-idf 5.4.0 everything appears to work as intended

this is the default option for USB-capable chips (ESP32-S3, ESP32-C3, ESP32-C6 etc.)

### HW UART

if USB Serial/JTAG is not supported by your MCU or works unreliably,
HW UART can be used:

* works perfectly as expected
* bottlenecked by UART baud rate
* available on all ESP32 family MCUs
* a board with USB-UART converter (or an external one) is required though
* i used baud rates of 921600 & 4_000_000 bps for testing

### USB-CDC

in theory tinyUSB can be used instead of built-in USB Serial/JTAG peripheral, 
but it is supported only by a subset of USB Serial/JTAG-capable chips (e.g. ESP32-S3 but not ESP32-C3/C6)
and since USB Serial/JTAG no longer misbehaves, i don't see any benefit in implementing USB-CDC

### TCP over Wi-Fi

*serprog TCP support is available only in non-windows builds e.g. linux, wsl2 etc.*

***warning**: flashrom serprog TCP support is working reliably only on flashrom version >= 1.4.0*

to enable TCP over Wi-Fi, before building run `idf.py menuconfig` and go to `esp32-serprog`,
select `Wi-Fi enabled` and set up your (existing) Wi-Fi credentials

ESP32 will connect to your Wi-Fi network and start listening for flashrom connections at (default setting) TCP port `8888`. 
IP address to connect to can be obtained either from default console output or by resolving the hostname (default setting) `esp32-serprog` / looking at your router client table

flashrom usage is the same as with UART, but instead of `dev=/dev/ttyACM0` you pass your ip:port e.g. `ip=192.168.0.2:8888`:

```
flashrom -p serprog:ip=192.168.1.120:8888,spispeed=20M -c W25Q32BV/W25Q32CV/W25Q32DV --verbose -w ../8266.bin
```

## Build and flash

run `idf.py flash`

## Use with flashrom

**WARNING**: ESP32 family IO operates at 3.3V, **DO NOT** connect 1.8V flash chips directly, use a levelshifter in that case. 
even the cheapest BSS138-based one will do the job although the SPI speed will be limited to ~200-400k. a proper levelshifter IC like TXS0108E should work fine at all reasonable SPI speeds

*i have successfully reflashed my Steam Deck 1.8V 16MByte BIOS chip using the cheapest aliexpress MOSFET levelshifter and 1.8V LDO at spispeed=250K, although it took like 20 minutes*

SPI speed selection is up to you, depends on the flash chip, wiring etc. - ESP32 should support up to 80MHz in theory. If no spispeed parameter is provided default value of 8M is used

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
