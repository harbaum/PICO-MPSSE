
#ifndef _PIO_JTAG_H
#define _PIO_JTAG_H

#include "hardware/pio.h"
#include "hardware/dma.h"

typedef struct pio_jtag_inst {
  PIO pio;
  uint sm;
  uint pin_tck, pin_tdi, pin_tdo, pin_tms;
  uint pins_upper[4];
  
  int tx_dma_chan;
  int rx_dma_chan;
  dma_channel_config tx_c;
  dma_channel_config rx_c;
  bool write_pending;
} pio_jtag_inst_t;

//interface to the PIO engine itself
void init_jtag(pio_jtag_inst_t* jtag, uint freq);
void jtag_set_clk_freq(pio_jtag_inst_t *jtag, uint freq_khz);
void pio_jtag_blocking(pio_jtag_inst_t *jtag, const uint8_t *src, uint8_t *dst, size_t len);
void pio_jtag_write_read_blocking(pio_jtag_inst_t *jtag, const uint8_t *bsrc, uint8_t *bdst, size_t len);

// convenience and support functions
void jtag_enable(pio_jtag_inst_t* jtag, bool enable); 
void jtag_write_tms(pio_jtag_inst_t* jtag, uint tdi, const uint8_t *src, uint8_t *dst, size_t len);		    
void jtag_write_tdi_read_tdo(pio_jtag_inst_t* jtag, const uint8_t *src, uint8_t *dst, size_t len);
void jtag_write_tdi(pio_jtag_inst_t* jtag, const uint8_t *src, size_t len);
void jtag_read_tdo(pio_jtag_inst_t* jtag, uint8_t *dst, size_t len);

#endif
