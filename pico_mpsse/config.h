#ifndef CONFIG_H
#define CONFIG_H

// TMS pin needs to be the TDI pin + 1

// first JTAG
#define JTAG1_PIN_TDI  2   // AD1
#define JTAG1_PIN_TMS  3   // AD3
#define JTAG1_PIN_TDO  4   // AD2
#define JTAG1_PIN_TCK  5   // AD0

#define JTAG1_PIN_D4   6   // AD4
#define JTAG1_PIN_D5   7   // AD5
#define JTAG1_PIN_D6   8   // AD6
#define JTAG1_PIN_D7   9   // AD7

// second JTAG
#define JTAG2_PIN_TCK 19   // BD1
#define JTAG2_PIN_TDO 18   // BD3
#define JTAG2_PIN_TMS 17   // BD2
#define JTAG2_PIN_TDI 16   // BD0

#define JTAG2_PIN_D4  20   // BD4
#define JTAG2_PIN_D5  21   // BD5
#define JTAG2_PIN_D6  22   // BD6
#define JTAG2_PIN_D7  26   // BD7

#endif // CONFIG_H
