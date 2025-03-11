# Pico MPSSE

The goal of this project is to allow the Raspberry Pi Pico to act
as a MPSSE compatible JTAG programmer for DIY FPGA projects.

It consists of two parts:

  - [usbmon_mpsse_parser](usbmon) is a python script allowing to analyze the USB traffic as generated during the use of a MPSSE capable device
  - [pico_mpsse](pico_mpsse) is a firmware for the Raspberry Pi Pico allowing it to act like a MPSSE device
