# Pico MPPSE

This is a FTDI MPSSE compatible JTAG implementation for the Raspberry
Pi Pico's RP2040 MCU. It allows to use the Raspberry Pi Pico as a
replacement for the FTDI MPSSE device family and use the Pico e.g.
as a flash adapter for FPGA boards.

## Pin usage

| FTXXX  | JTAG | DIR | RP2040 |
|--------|------|-----|--------|
| D0     | TCK  |  O  |  18    |
| D1     | TDI  |  O  |  16    |
| D2     | TDO  |  I  |  17    |
| D3     | TMS  |  O  |  19    |

## Current state

The device implements a very basic MPSSE barely sufficient to be used
for JTAG. It has successfully been used to detect various chips in a
JTAG border scan and to program the Efinix Trion T20 FPGA using
[openFPGAloader](https://github.com/trabucayre/openFPGALoader).

## Installation

Build this using the [Pico-SDK](https://github.com/raspberrypi/pico-sdk) and
load it onto a Raspberry Pi Pico.

This uses UART0 on GPIO0+1 for debug output at 115200 bit/s.

## Usage

The device should show up via USB as an FT2232:

```
$ lsusb
...
Bus 001 Device 003: ID 0403:6010 Future Technology Devices International, Ltd FT2232C/D/H Dual UART/FIFO IC
...
```

The openFPGAloader should be able to open and use it:

```
$ openFPGAloader --detect
empty
No cable or board specified: using direct ft2232 interface
Jtag frequency : requested 6.00MHz   -> real 6.00MHz  
index 0:
        idcode 0x210a79
        manufacturer efinix
        family Trion
        model  T8QFP144/T13/T20
        irlength 4
```

## Related projects

  - [Pico-DirtyJtag](https://github.com/phdussud/pico-dirtyJtag)
  
