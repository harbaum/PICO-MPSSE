#include <hardware/clocks.h>
#include "hardware/dma.h"
#include "pio_jtag.h"
#include "jtag.pio.h"

#include <stdio.h>
#include "config.h"

void dma_init(pio_jtag_inst_t* jtag) {
  // Configure a channel to write a buffer to PIO
  // SM0's TX FIFO, paced by the data request signal from that peripheral.
  jtag->tx_dma_chan = dma_claim_unused_channel(true);
  jtag->tx_c = dma_channel_get_default_config(jtag->tx_dma_chan);
  channel_config_set_transfer_data_size(&jtag->tx_c, DMA_SIZE_8);
  channel_config_set_read_increment(&jtag->tx_c, true);
  channel_config_set_dreq(&jtag->tx_c, (jtag->pio==pio0)?DREQ_PIO0_TX0:DREQ_PIO1_TX0);
  dma_channel_configure(
			jtag->tx_dma_chan,
			&jtag->tx_c,
			&jtag->pio->txf[0], // Write address (only need to set this once)
			NULL,               // Don't provide a read address yet
			0,                  // Don't provide the count yet
			false               // Don't start yet
			);
  // Configure a channel to read a buffer from PIO
  // SM0's RX FIFO, paced by the data request signal from that peripheral.
  jtag->rx_dma_chan = dma_claim_unused_channel(true);
  jtag->rx_c = dma_channel_get_default_config(jtag->rx_dma_chan);
  channel_config_set_transfer_data_size(&jtag->rx_c, DMA_SIZE_8);
  channel_config_set_write_increment(&jtag->rx_c, false);
  channel_config_set_read_increment(&jtag->rx_c, false);
  channel_config_set_dreq(&jtag->rx_c, (jtag->pio==pio0)?DREQ_PIO0_RX0:DREQ_PIO1_RX0);
  dma_channel_configure(
			jtag->rx_dma_chan,
			&jtag->rx_c,
			NULL,               // Dont provide a write address yet
			&jtag->pio->rxf[0], // Read address (only need to set this once)
			0,                  // Don't provide the count yet
			false               // Don't start yet
			);
}

void __time_critical_func(pio_jtag_blocking)(pio_jtag_inst_t *jtag, const uint8_t *bsrc, uint8_t *bdst, size_t len) {
  size_t byte_length = (len+7)/8;  // bytes needed for len bits
  size_t last_shift = ((byte_length << 3) - len);
  size_t tx_remain = (len+3)/4, rx_remain = last_shift ? byte_length : byte_length+1;
  // uint8_t* rx_last_byte_p = &bdst[byte_length-1];
  io_rw_8 *txfifo = (io_rw_8 *) &jtag->pio->txf[jtag->sm];
  static uint8_t x; // scratch local to receive data if no receive buffer is provided
  //kick off the process by sending the len to the tx pipeline

  // if(bdst && last_shift) printf("#%d ----> LAST SHIFT %d\n", (jtag->pio==pio0)?0:1, last_shift);
  
  if(jtag->write_pending) {
    while (dma_channel_is_busy(jtag->rx_dma_chan))
      tight_loop_contents();
    
    jtag->write_pending = false;
  }

  *(io_rw_32*)txfifo = len-1;
  
  channel_config_set_write_increment(&jtag->rx_c, bdst?true:false);
  channel_config_set_read_increment(&jtag->tx_c, true);
  dma_channel_set_config(jtag->rx_dma_chan, &jtag->rx_c, false);
  dma_channel_set_config(jtag->tx_dma_chan, &jtag->tx_c, false);
  
  dma_channel_transfer_to_buffer_now(jtag->rx_dma_chan, (void*)(bdst?bdst:&x), rx_remain);
  dma_channel_transfer_from_buffer_now(jtag->tx_dma_chan, (void*)bsrc, tx_remain);
  
  if(bdst || 1)  {
    while (dma_channel_is_busy(jtag->rx_dma_chan)) tight_loop_contents();

    // enabling this actually makes efinity programmer to not detect the FPGA
    //    if (last_shift)
    //      *rx_last_byte_p = *rx_last_byte_p << last_shift;
  } else
    jtag->write_pending = true;
  
  // stop the compiler hoisting a non volatile buffer access above the DMA completion.
  __compiler_memory_barrier();
}

static uint16_t jtag_get_clk_divider(int freq_khz) {
  uint clk_sys_freq_khz_4 = clock_get_hz(clk_sys) / (4*1000);
  uint16_t divider = clk_sys_freq_khz_4 / freq_khz;
  if (divider < 2) divider =  2; //max reliable freq 
  
  printf("Clk %d kHz, divider for %d kHz = %d\n", 4*clk_sys_freq_khz_4, freq_khz, divider);
  printf("  -> real freq = %d kHz\n", clk_sys_freq_khz_4 / divider);
  return divider;
}

void jtag_set_clk_freq(pio_jtag_inst_t *jtag, uint freq_khz) {
  pio_sm_set_clkdiv_int_frac(jtag->pio, jtag->sm, jtag_get_clk_divider(freq_khz), 0);
}

// tdi bits "exanded" for interleaved tdi/tms tranmission
static uint16_t interleave_table[256];

// TODO: include this into the interleave calculation 
uint8_t bit_reverse_u8(uint8_t byte) {
  byte = ((byte & 0x55) << 1) | ((byte & 0xaa) >> 1);
  byte = ((byte & 0x33) << 2) | ((byte & 0xcc) >> 2);
  byte = ((byte & 0x0f) << 4) | ((byte & 0xf0) >> 4);

  return byte;
}

void jtag_enable(pio_jtag_inst_t* jtag, bool enable) {
  if(enable) {
    printf("JTAG #%d ENABLE\n", (jtag->pio==pio0)?0:1);
    int gpio_func_pio = (jtag->pio == pio0)?GPIO_FUNC_PIO0:GPIO_FUNC_PIO1;
    gpio_set_function(jtag->pin_tdi, gpio_func_pio);
    gpio_set_function(jtag->pin_tms, gpio_func_pio);
    gpio_set_function(jtag->pin_tck, gpio_func_pio);
  } else {    
    printf("JTAG #%d DISABLE\n", (jtag->pio==pio0)?0:1);
    // revert to bitbang and make all pins inputs to make them tristate
    gpio_set_function(jtag->pin_tdi, GPIO_FUNC_SIO); gpio_set_dir(jtag->pin_tdi, GPIO_IN);
    gpio_set_function(jtag->pin_tms, GPIO_FUNC_SIO); gpio_set_dir(jtag->pin_tms, GPIO_IN);
    gpio_set_function(jtag->pin_tck, GPIO_FUNC_SIO); gpio_set_dir(jtag->pin_tck, GPIO_IN); 
  }
}
    
void init_jtag(pio_jtag_inst_t* jtag, uint freq) {
  uint16_t clkdiv = jtag_get_clk_divider(freq);   // clkdiv = 31;  // around 1 MHz @ 125MHz clk_sys
  pio_jtag_init(jtag->pio, jtag->sm, clkdiv, jtag->pin_tck, jtag->pin_tdi, jtag->pin_tms, jtag->pin_tdo);
  dma_init(jtag);    
  
  // Setup the interleave table. This does three things:
  // - it expands with every second bit being 0
  // - it swaps high and low byte in the 16 bit target
  // - it reversed the bit order
  for(uint16_t j=0;j<256;j++) {
    uint8_t i = bit_reverse_u8(j);
    interleave_table[j] =
	((i&0x01)<< 8) | ((i&0x02)<< 9) | ((i&0x04)<<10) | ((i&0x08)<<11) |
	((i&0x10)>> 4) | ((i&0x20)>> 3) | ((i&0x40)>> 2) | ((i&0x80)>> 1);
  }
}

// TODO: integrate this into the transmit function
void jtag_write_tms(pio_jtag_inst_t* jtag, uint tdi, const uint8_t *src, uint8_t *dst, size_t len) {
  uint16_t wlen = (len+7)/8;
  uint16_t tx_buffer[wlen];  // bytes are expanded to words for interleaved two-bit-transmission
  uint16_t tdi_mask = tdi?0x5555:0x0000;
  
  for(int i;i<wlen;i++)
    tx_buffer[i] = (interleave_table[src[i]]<<1) | tdi_mask;

  pio_jtag_blocking(jtag, (const uint8_t *)tx_buffer, dst, len);  

  // bit reverse result
  if(dst)
    for(int i=0;i<wlen;i++)
      dst[i] = bit_reverse_u8(dst[i]);  
}

void jtag_write_tdi_read_tdo(pio_jtag_inst_t* jtag, const uint8_t *src, uint8_t *dst, size_t len) {
  uint16_t wlen = (len+7)/8;
  uint16_t tx_buffer[wlen];  // bytes are expanded to words for interleaved two-bit-transmission

  // printf("LEN %d %d\n", len, wlen);
  
  // drive TMS constant 0 when writing tdi
  for(int i=0;i<wlen;i++)
    tx_buffer[i] = src?interleave_table[src[i]]:0x5555;

  pio_jtag_blocking(jtag, (const uint8_t *)tx_buffer, dst, len);

  // bit reverse result
  if(dst)
    for(int i=0;i<wlen;i++)
      dst[i] = bit_reverse_u8(dst[i]);  
}

void jtag_write_tdi(pio_jtag_inst_t* jtag, const uint8_t *src, size_t len) {
  jtag_write_tdi_read_tdo(jtag, src, NULL, len);
}

void jtag_read_tdo(pio_jtag_inst_t* jtag, uint8_t *dst, size_t len) {
  jtag_write_tdi_read_tdo(jtag, NULL, dst, len);
}

