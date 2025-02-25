#ifndef CONFIG_H
#define CONFIG_H

// TMS pin needs to be the TDI pin + 1

// first JTAG
#define JTAG1_PIN_TCK  2   // D0
#define JTAG1_PIN_TDO  3   // D2
#define JTAG1_PIN_TMS  4   // D3
#define JTAG1_PIN_TDI  5   // D1

#define JTAG1_PIN_D4   6
#define JTAG1_PIN_D5   7
#define JTAG1_PIN_D6   8
#define JTAG1_PIN_D7   9

// second JTAG
#define JTAG2_PIN_TCK 19
#define JTAG2_PIN_TDO 18
#define JTAG2_PIN_TMS 17
#define JTAG2_PIN_TDI 16

#define JTAG2_PIN_D4  20
#define JTAG2_PIN_D5  21
#define JTAG2_PIN_D6  22
#define JTAG2_PIN_D7  26

#endif // CONFIG_H
