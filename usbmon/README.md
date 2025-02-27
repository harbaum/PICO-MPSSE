# USBMon mpsse parse

This is a python script that reads usbmon traces written under Linux during a USB communication
with a FTDI MPSSE device like the FT232H, FT2232 or FT4243.

The trace is generated for example for USB bus #1:

```
$ sudo modprobe usbmon
$ sudo cat /sys/kernel/debug/usb/usbmon/1u > trace.txt
```

Alternally and especially if all traffic needs to be captured the
[usbmon command line utility](https://github.com/radupotop/usbmon)
 should be used instead. The following will make sure the up
 to 4096 bytes are being captured per USB transfer:

```
$ sudo modprobe usbmon
$ sudo usbmon -fu -i 1 -s 4096 > trace.txt
```

As long as this trace is being written software like [openFPGAloader](https://github.com/trabucayre/openFPGALoader) or proprietary
FPGA programming tools may be used:

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

As a result the USB traffic generated is included with the trace. If e.g. the
ft2232 was installed as

```
$ lsusb
...
Bus 001 Device 034: ID 0403:6010 Future Technology Devices International, Ltd FT2232C/D/H Dual UART/FIFO IC
...
```

it shows up as bus 1 and devices ce 34 in the trace.

## Example

An example of a FTDI232H on USB Bus 1, Device 17 being used to detect
a Efinix Trion T20 FPGA using openfpgaloader [is being included](ft232h_openfpgaloader_detect_t20_on_1_017.txt).

It can be parsed like so:

```
$ ./usbmon_mpsse_parse.py "ft232h_openfpgaloader_detect_t20_on_1_017.txt" 1:17
USBMON MPSSE parser
USB Vendor request 0x0 Reset -> Reset SIO
USB Vendor request 0x3 Set baud rate -> divisor=0x20104e2 9600.0
USB Vendor request 0x3 Set baud rate -> divisor=0x201c068 243.62514211466623
USB Vendor request 0x0 Reset -> Reset SIO
USB Vendor request 0xb Set bitmode -> mode=RESET mask=0x0
USB Vendor request 0x0 Reset -> Purge RX
USB Vendor request 0x0 Reset -> Purge TX
USB Vendor request 0x9 Set latency timer -> latency=5
USB Vendor request 0xb Set bitmode -> mode=MPSSE mask=0xfb
USB BULK IN 32 60 
USB BULK OUT 8a 86 04 00 
MPSSE: CMD 0x8a -> Disable div by 5 (60MHz master clock)
MPSSE: CMD 0x86 -> Set TCK/SK Divisor to 4 = 1.2/6.0 Mhz
USB BULK IN 32 60 
USB Vendor request 0x0 Reset -> Purge RX
USB Vendor request 0x0 Reset -> Purge TX
USB BULK OUT 80 08 0b 82 08 0b 
MPSSE: CMD 0x80 -> Set Data Bits low byte value 0x8 direction IIIIOIOO
MPSSE: CMD 0x82 -> Set Data Bits high byte value 0x8 direction IIIIOIOO
USB BULK OUT 4b 05 ff 4b 03 82 39 03 00 ff ff ff ff 87 
MPSSE: CMD 0x4b -> SHIFT W-VE,BIT,R+VE,LSB,W-TMS LEN=6 (0+6)
> TMS ff 
MPSSE: CMD 0x4b -> SHIFT W-VE,BIT,R+VE,LSB,W-TMS LEN=4 (0+4)
> TMS 82 
JTAG: TDI 1 TMS 0 -> RUN TEST IDLE
JTAG: TDI 1 TMS 1 -> SELECT DR SCAN
JTAG: TDI 1 TMS 0 -> CAPTURE DR
JTAG: TDI 1 TMS 0 -> SHIFT DR
MPSSE: CMD 0x39 -> SHIFT W-VE,BYTE,R+VE,LSB,W-TDI,R-TDO LEN=32 (4+0)
> TDI ff ff ff ff 
MPSSE: CMD 0x87 -> Flush
USB BULK IN 32 60 79 0a 21 00 
< TDO 79 0a 21 00 
USB BULK OUT 39 03 00 ff ff ff ff 87 
MPSSE: CMD 0x39 -> SHIFT W-VE,BYTE,R+VE,LSB,W-TDI,R-TDO LEN=32 (4+0)
> TDI ff ff ff ff 
MPSSE: CMD 0x87 -> Flush
USB BULK IN 32 60 ff ff ff ff 
< TDO ff ff ff ff 
USB BULK OUT 4b 04 bf 
MPSSE: CMD 0x4b -> SHIFT W-VE,BIT,R+VE,LSB,W-TMS LEN=5 (0+5)
> TMS bf 
JTAG: TDI 1 TMS 1 -> EXIT1 DR
JTAG: TDI 1 TMS 1 -> UPDATE DR
JTAG: DR loaded with 65 (8+1) bits
JTAG: DR data out 79 0a 21 00 ff ff ff ff
JTAG: IDCODE 0x210a79
JTAG: Manufacturer: efinix
JTAG: Family: Trion
JTAG: Model: T8QFP144/T13/T20
JTAG: IR length: 4
JTAG: TDI 1 TMS 1 -> SELECT DR SCAN
JTAG: TDI 1 TMS 1 -> SELECT IR SCAN
JTAG: TDI 1 TMS 1 -> TEST LOGIC RESET
USB BULK OUT 80 ff 00 82 ff 00 84 39 04 00 aa 55 00 ff aa 85 87 
MPSSE: CMD 0x80 -> Set Data Bits low byte value 0xff direction IIIIIIII
MPSSE: CMD 0x82 -> Set Data Bits high byte value 0xff direction IIIIIIII
MPSSE: CMD 0x84 -> Connect loopback
MPSSE: CMD 0x39 -> SHIFT W-VE,BYTE,R+VE,LSB,W-TDI,R-TDO LEN=40 (5+0)
> TDI aa 55 00 ff aa 
MPSSE: CMD 0x85 -> Disconnect loopback
MPSSE: CMD 0x87 -> Flush
USB BULK IN 32 60 aa 55 00 ff aa 
< TDO aa 55 00 ff aa 
USB Vendor request 0xb Set bitmode -> mode=RESET mask=0x0
USB Vendor request 0x0 Reset -> Reset SIO
USB Vendor request 0x0 Reset -> Purge TX
USB Vendor request 0x0 Reset -> Purge RX
```
