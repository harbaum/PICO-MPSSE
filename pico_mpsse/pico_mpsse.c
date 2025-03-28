/*
 * pico_mpsse.c
 *
 * MPSSE implementation for the Raspberry Pi Pico / RP2040
 * Intended to be used with openfpgaloader to load FPGAs
 */

/*
  TODO:
  - Handle read-only requests exceeding the reply buffer size  
 */

#include <stdio.h>
#include "pico/stdlib.h"// Pico
#include <pico/unique_id.h>
#include <string.h>// For memcpy
#include <ctype.h>

#include "usb_common.h"           // Include descriptor struct definitions
#include "hardware/regs/usb.h"    // USB register definitions from pico-sdk
#include "hardware/structs/usb.h" // USB hardware struct definitions from pico-sdk
#include "hardware/irq.h"         // For interrupt enable and numbers
#include "hardware/resets.h"      // For resetting the USB controller
#include "pico_mpsse.h"           // Device descriptors

#include "hardware/clocks.h"      // To adjust system clock to allow for 6Mhz
#include "pio_jtag.h"
#include "config.h"

#if 0
#define DEBUG_BULK0
#define DEBUG_BULK1
#define DEBUG_SHIFT
#define DEBUG_REPLY
#define DEBUG_GPIO
#define DEBUG_MPSSE
#define DEBUG_TRUNCATION
#define DEBUG_FLOWCONTROL
#endif

// this is the dummy status reply sent in all replies
#define REPLY_STATUS   "\x32\x60"

/* ---------------------------------------------------------------- */
/* ----------------------------- USB ------------------------------ */
/* ---------------------------------------------------------------- */

#define usb_hw_set ((usb_hw_t *)hw_set_alias_untyped(usb_hw))
#define usb_hw_clear ((usb_hw_t *)hw_clear_alias_untyped(usb_hw))

// Function prototypes for our device specific endpoint handlers defined
// later on
void ep0_in_handler(uint8_t *buf, uint16_t len);
void ep0_out_handler(uint8_t *buf, uint16_t len);
void ep1_in_handler(uint8_t *buf, uint16_t len);
void ep2_out_handler(uint8_t *buf, uint16_t len);
void ep3_in_handler(uint8_t *buf, uint16_t len);
void ep4_out_handler(uint8_t *buf, uint16_t len);

// Global device address
static bool should_set_address = false;
static uint8_t dev_addr = 0;
static volatile bool configured = false;

// Global data buffer for EP0
static uint8_t ep0_buf[64];

// USB_DPRAM_SIZE is 4096u which is sifficient for 4 512 bit endpoints

#ifndef JTAG1_PIN_D4
#define JTAG1_PIN_D4 -1
#endif

#ifndef JTAG1_PIN_D5
#define JTAG1_PIN_D5 -1
#endif

#ifndef JTAG1_PIN_D6
#define JTAG1_PIN_D6 -1
#endif

#ifndef JTAG1_PIN_D7
#define JTAG1_PIN_D7 -1
#endif

#ifndef JTAG2_PIN_D4
#define JTAG2_PIN_D4 -1
#endif

#ifndef JTAG2_PIN_D5
#define JTAG2_PIN_D5 -1
#endif

#ifndef JTAG2_PIN_D6
#define JTAG2_PIN_D6 -1
#endif

#ifndef JTAG2_PIN_D7
#define JTAG2_PIN_D7 -1
#endif

static void port_gpio_set_dir(struct jtag *jtag, uint8_t dir) {
  // check if any direction bits have changed at all
  if(dir == jtag->pio.gpio_dir) return;

  // enable the PIO if the direction of the lower four port pins matches the JTAG/SPI use case
  pio_jtag_enable(&jtag->pio, (dir & 0x0f) == 0x0b);

  // set lower pins direction if not in pio mode
  if(!jtag->pio.pio_enabled) {
    uint8_t low_pins[] = { jtag->pio.pin_tck, jtag->pio.pin_tdi, jtag->pio.pin_tdo, jtag->pio.pin_tms };
    for(int i=0;i<4;i++) {
      gpio_set_dir(low_pins[i], (dir&(1<<i))?GPIO_OUT:GPIO_IN);
#ifdef DEBUG_GPIO
      printf("#%d GPIO%d DIR = %s\n", i, low_pins[i], (dir&(1<<i))?"output":"input");
#endif
    }
  }

  // handle upper bits
  for(int i=0;i<4;i++) {
    if(jtag->pio.pins_upper[i]) {
      gpio_set_dir(jtag->pio.pins_upper[i], (dir&(1<<(i+4)))?GPIO_OUT:GPIO_IN);
#ifdef DEBUG_GPIO
      printf("#%d GPIO%d DIR = %s\n", i+4, jtag->pio.pins_upper[i], (dir&(1<<(i+4)))?"output":"input");
#endif	  
    }
  }    
  jtag->pio.gpio_dir = dir;
}

static void port_gpio_set(struct jtag *jtag, uint8_t value) {
  // PIO is only used in JTAG/SPI compatible mode with direction of D0-D3 being 0x0b
  if(jtag->pio.pio_enabled) {      
    // this command also sets a certain state to the lower output pins
    uint8_t ostate = 0;
    if(value & (1<<0)) ostate |= PIO_JTAG_BIT_TCK;
    if(value & (1<<1)) ostate |= PIO_JTAG_BIT_TDI;
    // bit 2 is an input in JTAG/SPI mode and cannot be set this way
    if(value & (1<<3)) ostate |= PIO_JTAG_BIT_TMS;
#ifdef DEBUG_GPIO
    printf("PIO GPIO = %d\n", ostate);
#endif
    pio_set_outputs(&jtag->pio, ostate);
  } else {
    // pio not active, set lower GPIO directly
    uint8_t low_pins[] = { jtag->pio.pin_tck, jtag->pio.pin_tdi, jtag->pio.pin_tdo, jtag->pio.pin_tms };
    for(int i=0;i<4;i++) {
      if(jtag->pio.gpio_dir & (1<<i)) {
#ifdef DEBUG_GPIO
	printf("#%d GPIO%d = %d\n", i, low_pins[i], (value<<i)?1:0);
#endif
	gpio_put(low_pins[i], (value<<i)?1:0);
      }
    }
  }
    
  // handle upper gpio if present
  for(int i=0;i<4;i++) {
    if(jtag->pio.pins_upper[i]) {
      // set value id pin is configured as output
      if(jtag->pio.gpio_dir & (1<<(i+4))) {
	gpio_put(jtag->pio.pins_upper[i], (value&(1<<(i+4)))?1:0);
#ifdef DEBUG_GPIO
	printf("#%d GPIO%d = %d\n", i+4, jtag->pio.pins_upper[i], (value&(1<<(i+4)))?1:0);	    
#endif
      }
    }
#ifdef DEBUG_GPIO
    else printf("\n");
#endif
  }
}

static uint8_t port_gpio_get(struct jtag *jtag) {
  uint8_t low_pins[] = { jtag->pio.pin_tck, jtag->pio.pin_tdi, jtag->pio.pin_tdo, jtag->pio.pin_tms };
  uint8_t retval = 0;

  // read the state of the lower four bits which are otherwise controlled by the
  // PIO engine
  for(int i=0;i<4;i++)
    if(gpio_get(low_pins[i])) retval |= (1<<i);
  
  // check state of upper four bits
  for(int i=0;i<4;i++)
    if(jtag->pio.pins_upper[i] != -1)
      if(gpio_get(jtag->pio.pins_upper[i])) retval |= (1<<(4+i));

  return retval;
}

// Struct defining the device configuration
static struct usb_device_configuration dev_config = {
        .device_descriptor = &device_descriptor,
#ifdef USB_HS
        .device_qualifier_descriptor = &device_qualifier_descriptor,
#endif
	.ports[0].jtag.pio.pio = pio0,
	.ports[0].jtag.pio.sm = 0,
	.ports[0].jtag.pio.pin_tck = JTAG1_PIN_TCK,
	.ports[0].jtag.pio.pin_tdi = JTAG1_PIN_TDI,
	.ports[0].jtag.pio.pin_tdo = JTAG1_PIN_TDO,
	.ports[0].jtag.pio.pin_tms = JTAG1_PIN_TMS,
	.ports[0].jtag.pio.pins_upper = { JTAG1_PIN_D4, JTAG1_PIN_D5, JTAG1_PIN_D6, JTAG1_PIN_D7 },	  
	.ports[0].jtag.pio.write_pending = false,
	.ports[0].jtag.pending_writes = 0,
	.ports[0].jtag.eps = { EP1_IN_ADDR, EP2_OUT_ADDR },
	.ports[0].jtag.reply_len = 0,
	.ports[0].jtag.tx_pending = false,
	.ports[0].jtag.rx_disabled = false,
	.ports[0].jtag.cmd_buf.len = 0,
	.ports[0].interface_descriptor = &interface_descriptor_p0,
	.ports[0].endpoints = { {
	    .descriptor = &ep1_in,
	    .handler = &ep1_in_handler,
	    .endpoint_control = &usb_dpram->ep_ctrl[0].in,
	    .buffer_control = &usb_dpram->ep_buf_ctrl[1].in,
	    .data_buffer = &usb_dpram->epx_data[0 * MAX_PKT_SIZE],
	  }, {
	    .descriptor = &ep2_out,
	    .handler = &ep2_out_handler,
	    .endpoint_control = &usb_dpram->ep_ctrl[1].out,
	    .buffer_control = &usb_dpram->ep_buf_ctrl[2].out,
	    .data_buffer = &usb_dpram->epx_data[1 * MAX_PKT_SIZE],
	  } },
	
	.ports[1].jtag.pio.pio = pio1,
	.ports[1].jtag.pio.sm = 0,
	.ports[1].jtag.pio.pin_tck = JTAG2_PIN_TCK,
	.ports[1].jtag.pio.pin_tdi = JTAG2_PIN_TDI,
	.ports[1].jtag.pio.pin_tdo = JTAG2_PIN_TDO,
	.ports[1].jtag.pio.pin_tms = JTAG2_PIN_TMS,
	.ports[1].jtag.pio.pins_upper = { JTAG2_PIN_D4, JTAG2_PIN_D5, JTAG2_PIN_D6, JTAG2_PIN_D7 },	  
	.ports[1].jtag.pio.write_pending = false,
	.ports[1].jtag.pending_writes = 0,
	.ports[1].jtag.eps = { EP3_IN_ADDR, EP4_OUT_ADDR },
	.ports[1].jtag.reply_len = 0,
	.ports[1].jtag.tx_pending = false,
	.ports[1].jtag.rx_disabled = false,
	.ports[1].jtag.cmd_buf.len = 0,
	.ports[1].interface_descriptor = &interface_descriptor_p1,
	.ports[1].endpoints = { {
	    .descriptor = &ep3_in,
	    .handler = &ep3_in_handler,
	    .endpoint_control = &usb_dpram->ep_ctrl[2].in,
	    .buffer_control = &usb_dpram->ep_buf_ctrl[3].in,
	    .data_buffer = &usb_dpram->epx_data[2 * MAX_PKT_SIZE],
	  }, {
	    .descriptor = &ep4_out,
	    .handler = &ep4_out_handler,
	    .endpoint_control = &usb_dpram->ep_ctrl[3].out,
	    .buffer_control = &usb_dpram->ep_buf_ctrl[4].out,
	    .data_buffer = &usb_dpram->epx_data[3 * MAX_PKT_SIZE],
	  } },
	
        .config_descriptor = &config_descriptor,
        .lang_descriptor = lang_descriptor,
        .descriptor_strings = descriptor_strings,
        .endpoints = { {
	    .descriptor = &ep0_out,
	    .handler = &ep0_out_handler,
	    .endpoint_control = NULL, // NA for EP0
	    .buffer_control = &usb_dpram->ep_buf_ctrl[0].out,
	    // EP0 in and out share a data buffer
	    .data_buffer = &usb_dpram->ep0_buf_a[0],
	  }, {
	    .descriptor = &ep0_in,
	    .handler = &ep0_in_handler,
	    .endpoint_control = NULL, // NA for EP0,
	    .buffer_control = &usb_dpram->ep_buf_ctrl[0].in,
	    // EP0 in and out share a data buffer
	    .data_buffer = &usb_dpram->ep0_buf_a[0],
	  }
        }
};

/**
 * @brief Given an endpoint address, return the usb_endpoint_configuration of that endpoint. Returns NULL
 * if an endpoint of that address is not found.
 *
 * @param addr
 * @return struct usb_endpoint_configuration*
 */
struct usb_endpoint_configuration *usb_get_endpoint_configuration(uint8_t addr) {
    // search the two control endpoints and both of the two ports endpoints
    for (int i = 0; i < 2; i++) {
      if ( dev_config.endpoints[i].descriptor->bEndpointAddress == addr)
	return &dev_config.endpoints[i];
      for (uint p = 0; p < 2; p++) {
	if ( dev_config.ports[p].endpoints[i].descriptor->bEndpointAddress == addr)
	  return &dev_config.ports[p].endpoints[i];
      }
    }

    return NULL;
}

/**
 * @brief Given a C string, fill the EP0 data buf with a USB string descriptor for that string.
 *
 * @param C string you would like to send to the USB host
 * @return the length of the string descriptor in EP0 buf
 */
uint8_t usb_prepare_string_descriptor(const unsigned char *str) {
    // 2 for bLength + bDescriptorType + strlen * 2 because string is unicode. i.e. other byte will be 0
    static const uint8_t bDescriptorType = 0x03;
    uint8_t bLength;
    volatile uint8_t *buf = &ep0_buf[0];
      
    if(str) {
      // ordinary string taken from flash
      bLength = 2 + (strlen((const char *)str) * 2);
      *buf++ = bLength;
      *buf++ = bDescriptorType;

      uint8_t c;
      do {
        c = *str++;
        *buf++ = c;
        *buf++ = 0;
      } while (c != '\0');
    } else {
      // serial number
      pico_get_unique_board_id_string(&ep0_buf[0]+2, 64);
      bLength = strlen(&ep0_buf[0]+2);
      
      for(int i=bLength;i>=0;i--) {
	buf[2+2*i]   = buf[2+i];
	buf[2+2*i+1] = 0x00;
      }

      bLength = 2 + 2 * bLength;
      buf[0] = bLength;
      buf[1] = bDescriptorType;     
    }
      
    return bLength;
}

/**
 * @brief Take a buffer pointer located in the USB RAM and return as an offset of the RAM.
 *
 * @param buf
 * @return uint32_t
 */
static inline uint32_t usb_buffer_offset(volatile uint8_t *buf) {
    return (uint32_t) buf ^ (uint32_t) usb_dpram;
}

/**
 * @brief Set up the endpoint control register for an endpoint (if applicable. Not valid for EP0).
 *
 * @param ep
 */
void usb_setup_endpoint(const struct usb_endpoint_configuration *ep) {
    printf("Set up endpoint 0x%x with buffer address 0x%p\n", ep->descriptor->bEndpointAddress, ep->data_buffer);

    // EP0 doesn't have one so return if that is the case
    if (!ep->endpoint_control) {
        return;
    }

    // Get the data buffer as an offset of the USB controller's DPRAM
    uint32_t dpram_offset = usb_buffer_offset(ep->data_buffer);
    uint32_t reg = EP_CTRL_ENABLE_BITS
                   | EP_CTRL_INTERRUPT_PER_BUFFER
                   | (ep->descriptor->bmAttributes << EP_CTRL_BUFFER_TYPE_LSB)
                   | dpram_offset;
    *ep->endpoint_control = reg;
}

/**
 * @brief Set up the endpoint control register for each endpoint.
 *
 */
void usb_setup_endpoints() {
    for (int i = 0; i < 2; i++) {
      usb_setup_endpoint(&dev_config.endpoints[i]);
      for (uint p = 0; p < 2; p++) 
	usb_setup_endpoint(&dev_config.ports[p].endpoints[i]);
    }
}

/**
 * @brief Set up the USB controller in device mode, clearing any previous state.
 *
 */
void usb_device_init() {
    // Reset usb controller
    reset_unreset_block_num_wait_blocking(RESET_USBCTRL);

    // Clear any previous state in dpram just in case
    memset(usb_dpram, 0, sizeof(*usb_dpram)); // <1>

    // Enable USB interrupt at processor
    irq_set_enabled(USBCTRL_IRQ, true);

    // Mux the controller to the onboard usb phy
    usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;

    // Force VBUS detect so the device thinks it is plugged into a host
    usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;

    // Enable the USB controller in device mode.
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;

    // Enable an interrupt per EP0 transaction
    usb_hw->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS; // <2>

    // Enable interrupts for when a buffer is done, when the bus is reset,
    // and when a setup packet is received
    usb_hw->inte = USB_INTS_BUFF_STATUS_BITS |
                   USB_INTS_BUS_RESET_BITS |
                   USB_INTS_SETUP_REQ_BITS;

    // Set up endpoints (endpoint control registers)
    // described by device configuration
    usb_setup_endpoints();

    // Present full speed device by enabling pull up on DP
    usb_hw_set->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
}

/**
 * @brief Given an endpoint configuration, returns true if the endpoint
 * is transmitting data to the host (i.e. is an IN endpoint)
 *
 * @param ep, the endpoint configuration
 * @return true
 * @return false
 */
static inline bool ep_is_tx(struct usb_endpoint_configuration *ep) {
    return ep->descriptor->bEndpointAddress & USB_DIR_IN;
}

/**
 * @brief Starts a transfer on a given endpoint.
 *
 * @param ep, the endpoint configuration.
 * @param buf, the data buffer to send. Only applicable if the endpoint is TX
 * @param len, the length of the data in buf (this example limits max len to one packet - 64 bytes)
 */
void usb_start_transfer(struct usb_endpoint_configuration *ep, uint8_t *buf, uint16_t len) {
    // We are asserting that the length is <= 64 bytes for simplicity of the example.
    // For multi packet transfers see the tinyusb port.
    assert(len <= 64);

    // printf("Start transfer of len %d on ep addr 0x%x\n", len, ep->descriptor->bEndpointAddress);

    // Prepare buffer control register value
    uint32_t val = len | USB_BUF_CTRL_AVAIL;

    if (ep_is_tx(ep)) {
        // Need to copy the data from the user buffer to the usb memory
        memcpy((void *) ep->data_buffer, (void *) buf, len);
        // Mark as full
        val |= USB_BUF_CTRL_FULL;
    }

    // Set pid and flip for next transfer
    val |= ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
    ep->next_pid ^= 1u;

    *ep->buffer_control = val;
}

/**
 * @brief Send device descriptor to host
 *
 */
void usb_handle_device_descriptor(volatile struct usb_setup_packet *pkt) {
    const struct usb_device_descriptor *d = dev_config.device_descriptor;
    // EP0 in
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_IN_ADDR);
    // Always respond with pid 1
    ep->next_pid = 1;
    usb_start_transfer(ep, (uint8_t *) d, MIN(sizeof(struct usb_device_descriptor), pkt->wLength));
}

#ifdef USB_HS
void usb_handle_device_qualifier_descriptor(volatile struct usb_setup_packet *pkt) {
    const struct usb_device_qualifier_descriptor *d = dev_config.device_qualifier_descriptor;
    // EP0 in
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_IN_ADDR);
    // Always respond with pid 1
    ep->next_pid = 1;
    usb_start_transfer(ep, (uint8_t *) d, MIN(sizeof(struct usb_device_qualifier_descriptor), pkt->wLength));
}
#endif

/**
 * @brief Send the configuration descriptor (and potentially the configuration and endpoint descriptors) to the host.
 *
 * @param pkt, the setup packet received from the host.
 */
void usb_handle_config_descriptor(volatile struct usb_setup_packet *pkt) {
    uint8_t *buf = &ep0_buf[0];

    // First request will want just the config descriptor
    const struct usb_configuration_descriptor *d = dev_config.config_descriptor;
    memcpy((void *) buf, d, sizeof(struct usb_configuration_descriptor));
    buf += sizeof(struct usb_configuration_descriptor);

    // If we more than just the config descriptor copy it all
    //    if (pkt->wLength >= d->wTotalLength) {
      for (uint p = 0; p < 2; p++) {
	// send both ports interface desciptors with both endpoints each
        memcpy((void *) buf, dev_config.ports[p].interface_descriptor, sizeof(struct usb_interface_descriptor));
        buf += sizeof(struct usb_interface_descriptor);
	memcpy((void *) buf, dev_config.ports[p].endpoints[0].descriptor, sizeof(struct usb_endpoint_descriptor));
	buf += sizeof(struct usb_endpoint_descriptor);
	memcpy((void *) buf, dev_config.ports[p].endpoints[1].descriptor, sizeof(struct usb_endpoint_descriptor));
	buf += sizeof(struct usb_endpoint_descriptor);
      }
      //    }

    // Send data
    // Get len by working out end of buffer subtract start of buffer
    uint32_t len = (uint32_t) buf - (uint32_t) &ep0_buf[0];
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), &ep0_buf[0], MIN(len, pkt->wLength));
}

/**
 * @brief Handle a BUS RESET from the host by setting the device address back to 0.
 *
 */
void usb_bus_reset(void) {
    // Set address back to 0
    dev_addr = 0;
    should_set_address = false;
    usb_hw->dev_addr_ctrl = 0;
    configured = false;
}

/**
 * @brief Send the requested string descriptor to the host.
 *
 * @param pkt, the setup packet from the host.
 */
void usb_handle_string_descriptor(volatile struct usb_setup_packet *pkt) {
    uint8_t i = pkt->wValue & 0xff;
    uint8_t len = 0;

    if (i == 0) {
        len = 4;
        memcpy(&ep0_buf[0], dev_config.lang_descriptor, len);
    } else {
        // Prepare fills in ep0_buf
        len = usb_prepare_string_descriptor((i<=2)?dev_config.descriptor_strings[i - 1]:NULL);
    }

    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), &ep0_buf[0], MIN(len, pkt->wLength));
}

/**
 * @brief Handles a SET_ADDR request from the host. The actual setting of the device address in
 * hardware is done in ep0_in_handler. This is because we have to acknowledge the request first
 * as a device with address zero.
 *
 * @param pkt, the setup packet from the host.
 */
void usb_set_device_address(volatile struct usb_setup_packet *pkt) {
    // Set address is a bit of a strange case because we have to send a 0 length status packet first with
    // address 0
    dev_addr = (pkt->wValue & 0xff);
    printf("Set address %d\r\n", dev_addr);
    // Will set address in the callback phase
    should_set_address = true;
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
}

/**
 * @brief Handles a SET_CONFIGRUATION request from the host. Assumes one configuration so simply
 * sends a zero length status packet back to the host.
 *
 * @param pkt, the setup packet from the host.
 */
void usb_set_device_configuration(__unused volatile struct usb_setup_packet *pkt) {
    // Only one configuration so just acknowledge the request
    printf("Device Enumerated\r\n");
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
    configured = true;
}

/* handle USB status request */
void usb_handle_status_request(volatile struct usb_setup_packet *pkt) {
  static uint8_t status[2] = { 0,0 };
  struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_IN_ADDR);
  ep->next_pid = 1;
  usb_start_transfer(ep, status, MIN(2, pkt->wLength));
}


/**
 * @brief Respond to a setup packet from the host.
 *
 */
static void check_for_outgoing_data(struct jtag *jtag);

/* this is a fake eeprom. It's not permanent and changes from PC */
/* are lost after power cycle */
static uint16_t eeprom_dummy_data[256] = { 
  0x0801, 0x0403, 0x6010, 0x0500, 0x00c0, 0x0000, 0x0200, 0x1296,
  0x16a8, 0x14be, 0x0046, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0312, 0x004e, 0x006f, 0x0074, 0x0020,
  0x0046, 0x0054, 0x0044, 0x0049, 0x0316, 0x0050, 0x0069, 0x0063,
  0x006f, 0x0020, 0x004d, 0x0050, 0x0053, 0x0053, 0x0045, 0x0314,
  0x0030, 0x0031, 0x0032, 0x0033, 0x0034, 0x0035, 0x0036, 0x0037,
  0x0038, 0x0302, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x6aa4
};

void usb_handle_setup_packet(void) {
    volatile struct usb_setup_packet *pkt = (volatile struct usb_setup_packet *) &usb_dpram->setup_packet;

    // Reset PID to 1 for EP0 IN
    usb_get_endpoint_configuration(EP0_IN_ADDR)->next_pid = 1u;

    if (pkt->bmRequestType == USB_DIR_OUT) {
        if (pkt->bRequest == USB_REQUEST_SET_ADDRESS) {
            usb_set_device_address(pkt);
        } else if (pkt->bRequest == USB_REQUEST_SET_CONFIGURATION) {
            usb_set_device_configuration(pkt);
        } else {
	    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
            printf("Other OUT request (0x%x)\r\n", pkt->bRequest);
        }
    } else if (pkt->bmRequestType == USB_DIR_IN) {
        if (pkt->bRequest == USB_REQUEST_GET_STATUS) {
	  usb_handle_status_request(pkt);
	  
        } else if (pkt->bRequest == USB_REQUEST_GET_DESCRIPTOR) {
            switch (pkt->wValue >> 8) {
                case USB_DT_DEVICE:
                    usb_handle_device_descriptor(pkt);
                    printf("GET DEVICE DESCRIPTOR 0x%04x/0x%02x %d\r\n", pkt->wIndex, pkt->wValue&0xff, pkt->wLength);
                    break;

                case USB_DT_CONFIG:
                    usb_handle_config_descriptor(pkt);
                    printf("GET CONFIG DESCRIPTOR 0x%04x/0x%02x %d\r\n", pkt->wIndex, pkt->wValue&0xff, pkt->wLength);
                    break;

                case USB_DT_STRING:
                    usb_handle_string_descriptor(pkt);
                    printf("GET STRING DESCRIPTOR 0x%04x/0x%02x %d\r\n", pkt->wIndex, pkt->wValue&0xff, pkt->wLength);
                    break;

#ifdef USB_HS
	        case USB_DT_DEVICE_QUALIFIER:
		    usb_handle_device_qualifier_descriptor(pkt);
		    printf("GET DEVICE QUALIFIER DESCRIPTOR\r\n");
		    break;
#endif
		    
                default:
		    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
                    printf("Unhandled GET_DESCRIPTOR type 0x%x\r\n", pkt->wValue >> 8);
            }
        } else {
            printf("Other IN request (0x%x)\r\n", pkt->bRequest);
        }
    }
    
    else if (pkt->bmRequestType == USB_VENDOR_OUT) {
      switch(pkt->bRequest) {
      case 0x00:
	printf("RESET, #%d=%d\n", pkt->wIndex, pkt->wValue);

	// TODO: check if this actually resets everything and returns to idle/all tristate
	
	break;

      case 0x01:
	printf("SET MODEM CONTROL, #%d=%d\n", pkt->wIndex, pkt->wValue);
	break;
	
      case 0x02:
	printf("SET FLOW CONTROL, #%d=%d\n", pkt->wIndex, pkt->wValue);
	break;
	
      case 0x03:
	printf("SET BAUD RATE, #%d=%d\n", pkt->wIndex, pkt->wValue);
	break;

      case 0x04:
	printf("SET DATA, #%d=%d\n", pkt->wIndex, pkt->wValue);
	break;

      case 0x05:
	printf("POLL MODEM STATUS, #%d=0x%02x\n", pkt->wIndex, pkt->wValue);
	break;

      case 0x06:
	printf("SET EVENT CHARACTER, #%d=0x%02x\n", pkt->wIndex, pkt->wValue);
	break;

      case 0x07:
	printf("SET ERROR CHARACTER, #%d=0x%02x\n", pkt->wIndex, pkt->wValue);
	break;

      case 0x09:
	printf("SET LATENCY TIMER, #%d=%d\n", pkt->wIndex, pkt->wValue);
	break;

      case 0x0b:
	printf("SET BITMODE, #%d=0x%02x\n", pkt->wIndex, pkt->wValue);
	if(pkt->wIndex >= 1 && pkt->wIndex <= 2) {
	  struct jtag *jtag = &dev_config.ports[pkt->wIndex-1].jtag;
	  
	  jtag->mode = pkt->wValue>>8;
	  port_gpio_set_dir(jtag, pkt->wValue & 0xff);
	}
	break;

      case 0x91:
	printf("WRITE EEPROM, #%d=%04x\n", pkt->wIndex, pkt->wValue);
	if(pkt->wIndex < 256) eeprom_dummy_data[pkt->wIndex] = pkt->wValue;
	break;

      case 0x92:
	printf("ERASE EEPROM, #%d=0x%04x\n", pkt->wIndex, pkt->wValue);
	// zero out our entire fake eeprom ...
	memset(eeprom_dummy_data, 0, sizeof(eeprom_dummy_data));
	break;

      default:
	printf("Unsupported vendor out request 0x%02x, #%d=%d\n",
	       pkt->bRequest, pkt->wIndex, pkt->wValue);
	break;	
      }
      usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
    }
    
    else if (pkt->bmRequestType == USB_VENDOR_IN) {
      switch(pkt->bRequest) {
      case 0x05:
	printf("POLL MODEM STATUS\n");
	usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), REPLY_STATUS, 2);
	break;
	
      case 0x90: // read eeprom
	if(pkt->wIndex < 256) {
	  printf("READ EEPROM, #%d=$%04x\n", pkt->wIndex, eeprom_dummy_data[pkt->wIndex]);
	  usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), (uint8_t*)&(eeprom_dummy_data[pkt->wIndex]), 2);
	} else {
	  printf("READ EEPROM, #%d (out of range)\n", pkt->wIndex);
	  usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
	}
	break;
	
      default:
	printf("Unsupported vendor in request 0x%02x, #%d=%d\n", pkt->bRequest, pkt->wIndex, pkt->wValue);
	usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
	break;
      }
    } else {
      static const char *types_str[] = { "Standard", "Class", "Vendor", "Reserved" };
      static const char *recipient_str[] = { "Device", "Interface", "Endpoint", "Other" };      
      printf("Unknown USB request type 0x%02x: %s %s %s\n", pkt->bmRequestType,
	     types_str[(pkt->bmRequestType>>5)&0x03],
	     ((pkt->bmRequestType&0x1f)<4)?recipient_str[pkt->bmRequestType&0x1f]:"Reserved",
	     (pkt->bmRequestType&0x80)?"out":"in"
	     );
      usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
    }
}

/**
 * @brief Notify an endpoint that a transfer has completed.
 *
 * @param ep, the endpoint to notify.
 */
static void usb_handle_ep_buff_done(struct usb_endpoint_configuration *ep) {
    uint32_t buffer_control = *ep->buffer_control;
    // Get the transfer length for this endpoint
    uint16_t len = buffer_control & USB_BUF_CTRL_LEN_MASK;

    // Call that endpoints buffer done handler
    ep->handler((uint8_t *) ep->data_buffer, len);
}

/**
 * @brief Find the endpoint configuration for a specified endpoint number and
 * direction and notify it that a transfer has completed.
 *
 * @param ep_num
 * @param in
 */
static void usb_handle_buff_done(uint ep_num, bool in) {
  uint8_t ep_addr = ep_num | (in ? USB_DIR_IN : 0);
  // printf("EP %d (in = %d) done\n", ep_num, in);

  for (uint i = 0; i < 2; i++) {
    // check the two control endpoints
    if (dev_config.endpoints[i].descriptor->bEndpointAddress == ep_addr) {
      usb_handle_ep_buff_done(&dev_config.endpoints[i]);
      return;
    }
    // and both endpoints of both ports
    for (uint p = 0; p < 2; p++) {
      if (dev_config.ports[p].endpoints[i].descriptor->bEndpointAddress == ep_addr) {
	usb_handle_ep_buff_done(&dev_config.ports[p].endpoints[i]);
	return;
      }
    }
  }
}

/**
 * @brief Handle a "buffer status" irq. This means that one or more
 * buffers have been sent / received. Notify each endpoint where this
 * is the case.
 */
static void usb_handle_buff_status() {
    uint32_t buffers = usb_hw->buf_status;
    uint32_t remaining_buffers = buffers;

    uint bit = 1u;
    for (uint i = 0; remaining_buffers && i < USB_NUM_ENDPOINTS * 2; i++) {
        if (remaining_buffers & bit) {
            // clear this in advance
            usb_hw_clear->buf_status = bit;
            // IN transfer for even i, OUT transfer for odd i
            usb_handle_buff_done(i >> 1u, !(i & 1u));
            remaining_buffers &= ~bit;
        }
        bit <<= 1u;
    }
}

/**
 * @brief USB interrupt handler
 *
 */
#ifdef __cplusplus
extern "C" {
#endif
/// \tag::isr_setup_packet[]
void isr_usbctrl(void) {
    // USB interrupt handler
    uint32_t status = usb_hw->ints;
    uint32_t handled = 0;

    // Setup packet received
    if (status & USB_INTS_SETUP_REQ_BITS) {
        handled |= USB_INTS_SETUP_REQ_BITS;
        usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
        usb_handle_setup_packet();
    }
/// \end::isr_setup_packet[]

    // Buffer status, one or more buffers have completed
    if (status & USB_INTS_BUFF_STATUS_BITS) {
        handled |= USB_INTS_BUFF_STATUS_BITS;
        usb_handle_buff_status();
    }

    // Bus is reset
    if (status & USB_INTS_BUS_RESET_BITS) {
        printf("BUS RESET\n");
        handled |= USB_INTS_BUS_RESET_BITS;
        usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;
        usb_bus_reset();
    }

    if (status ^ handled) {
        panic("Unhandled IRQ 0x%x\n", (uint) (status ^ handled));
    }
}
#ifdef __cplusplus
}
#endif

/**
 * @brief EP0 in transfer complete. Either finish the SET_ADDRESS process, or receive a zero
 * length status packet from the host.
 *
 * @param buf the data that was sent
 * @param len the length that was sent
 */
void ep0_in_handler(__unused uint8_t *buf, __unused uint16_t len) {
    if (should_set_address) {
        // Set actual device address in hardware
        usb_hw->dev_addr_ctrl = dev_addr;
        should_set_address = false;
    } else {
        // Receive a zero length status packet from the host on EP0 OUT
        struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_OUT_ADDR);
        usb_start_transfer(ep, NULL, 0);
    }

    // printf("EP0 IN\n");  
}

void hexdump(void *data, int size) {
  int i, b2c;
  int n=0;
  char *ptr = (char*)data;

  if(!size) return;

  while(size>0) {
    printf("%04x: ", n);

    b2c = (size>16)?16:size;
    for(i=0;i<b2c;i++)      printf("%02x ", 0xff&ptr[i]);
    printf("  ");
    for(i=0;i<(16-b2c);i++) printf("   ");
    for(i=0;i<b2c;i++)      printf("%c", isprint(ptr[i])?ptr[i]:'.');
    printf("\n");
    ptr  += b2c;
    size -= b2c;
    n    += b2c;
  }
}

void ep0_out_handler(__unused uint8_t *buf, __unused uint16_t len) {
  // printf("EP0 OUT\n");  
}

static bool check_reply_buffer(struct jtag *jtag) {
  // the reply buffer has a limited size. Once it runs
  // too full we need to stop accepting incoming requests
  // as the buffer may otherwise overflow. Since we cannot
  // know how much data the next request will ask to be
  // returned we stop the receiver once we have less than
  // 64 bytes in the reply buffer left.

  // reply_len does not include the two header bytes
#ifdef DEBUG_FLOWCONTROL
  printf("Reply buffer usage is %d of %d\n", jtag->reply_len+2, REPLY_BUFFER_SIZE);
#endif
  
  if(jtag->reply_len+2 > REPLY_BUFFER_SIZE-64) {
    // the buffer should actually never overflow
    if(jtag->reply_len+2 > REPLY_BUFFER_SIZE)
      printf(">>>>>>>>>>>>>> REPLY BUFFER DID OVERFLOW!!! <<<<<<<<<<<<<<<<<\n");
#ifdef DEBUG_FLOWCONTROL
    else
      printf("FLOW: reply buffer may overflow\n");
#endif
      
#ifdef DEBUG_FLOWCONTROL
    // don't allow any more replies
    printf("FLOW: stopping receiver\n");
#endif
    return false;
  }
  return true;
}

// parse a non-shifting MPSSE command
static uint16_t mpsse_cmd_parse(struct jtag *jtag) {
  uint8_t cmd = jtag->cmd_buf.data.cmd.code;
  
  switch(cmd) {
  case 0x80:
  case 0x82:
    uint8_t value = jtag->cmd_buf.data.cmd.b[0];
    uint8_t dir = jtag->cmd_buf.data.cmd.b[1];
    
#ifdef DEBUG_MPSSE
    printf("MPSSE: Set data bits %s value 0x%02x, dir 0x%02x=", (cmd&2)?"high":"low", value, dir);
    for(int i=0;i<8;i++) printf("%c", (dir&(0x80>>i))?'O':'I');
    printf("\n");
#endif
    
    /* we currently only support the lower bits */
    if(!(cmd&2)) {
      // second payload byte is direction. Lowest bits 0xb is JTAG (and SPI) mapping
      port_gpio_set_dir(jtag, dir);
      port_gpio_set(jtag, value);
    }
    break;
                
  case 0x81:
  case 0x83:
    uint8_t reply = 0;

    // only low byte supported
    if(!(cmd&2)) reply = port_gpio_get(jtag);
      
#ifdef DEBUG_MPSSE
    printf("MPSSE: Get data bits %s: 0x%02x\n", (cmd&2)?"high":"low", reply);
#endif
    jtag->reply_buffer[jtag->reply_len+2] = reply;
    jtag->reply_len += 1;    
    break;
    
  case 0x84:
#ifdef DEBUG_MPSSE
    printf("MPSSE: Connect loopback\n");
#endif
    break;

  case 0x85:
#ifdef DEBUG_MPSSE
    printf("MPSSE: Disconnect loopback\n");
#endif
    break;

  case 0x86: {
    // send input state in a reply byte
    int divisor = jtag->cmd_buf.data.cmd.w;
    int rate = 12000000 / ((1+divisor) * 2);
#ifdef DEBUG_MPSSE
    printf("MPSSE: Set PIO %d TCK/SK Divisor to %d = %d Mhz\n", PIO_INDEX(&jtag->pio), divisor, rate/1000000);
#endif
    pio_jtag_set_clk_freq(&jtag->pio, rate/1000);
  } break;

  case 0x87:
#ifdef DEBUG_MPSSE
    printf("MPSSE: Flush\n");
#endif
    break;

  case 0x8a:
#ifdef DEBUG_MPSSE
    printf("MPSSE: Disable div by 5 (60MHz master clock)\n");
#endif
    break;
        
  case 0x8b:
#ifdef DEBUG_MPSSE
    printf("MPSSE: Enable div by 5 (12MHz master clock)\n");
#endif
    break;
  }
  return 0;
}

static uint16_t mpsse_shift_bits(struct jtag *jtag) {
  uint8_t cmd = jtag->cmd_buf.data.cmd.code;

  // calculate data shift length
  uint16_t shift_len = jtag->cmd_buf.data.cmd.b[0]+1; // shift length was given in bits-1

  // request to write something? TDI or TMS?
  uint8_t data;
  if(cmd & 0x50) data = jtag->cmd_buf.data.cmd.b[1];
  
#ifdef DEBUG_SHIFT
  printf("MPSSE: shift %d bits\n", shift_len);
  hexdump(&data, (shift_len+7)/8);
#endif
  
  if(cmd & 0x40) {
    // printf("JTAG TMS BIT WRITE %d ", shift_len); hexdump(buf, 1);
    pio_jtag_write_tms(&jtag->pio, (cmd&8)?1:0, (data&0x80)?1:0, &data,
		       (cmd & 0x20)?(jtag->reply_buffer + jtag->reply_len + 2):NULL, shift_len);
    if(cmd & 0x20) jtag->reply_len += (shift_len+7)/8;
  } else {
    // printf("JTAG TDI BIT WRITE %d ", shift_len); hexdump(buf, 1);
    pio_jtag_write_tdi_read_tdo(&jtag->pio, (cmd&8)?1:0, (cmd & 0x10)?&data:NULL,
				(cmd & 0x20)?(jtag->reply_buffer + jtag->reply_len + 2):NULL, shift_len);
    if(cmd & 0x20) jtag->reply_len += (shift_len+7)/8;
  }
  
  return 0;  // no additional bytes used
} 

static uint16_t mpsse_shift_bytes(struct jtag *jtag, uint8_t *buf, uint16_t len) {
  uint8_t cmd = jtag->cmd_buf.data.cmd.code;

  // length is given in bytes and command length is variable
  uint16_t shift_len = jtag->cmd_buf.data.cmd.w + 1;

#ifdef DEBUG_SHIFT
  printf("MPSSE: shift %d bytes (%d avail)\n", shift_len, len);
  if(shift_len > len) hexdump(buf, len);
  else                hexdump(buf, shift_len);
#endif

  // it may happen that we are supposed to shift out more bits than we have payload
  if((cmd & 0x10) && (shift_len > len)) {
#ifdef DEBUG_TRUNCATION
    printf("Trunc write %d to %d\n", shift_len, len);
#endif

    pio_jtag_write_tdi_read_tdo(&jtag->pio, (cmd&8)?1:0, buf,
	    (cmd & 0x20)?(jtag->reply_buffer + jtag->reply_len + 2):NULL,len*8);
    if(cmd & 0x20) jtag->reply_len += len;
    jtag->pending_writes = shift_len-len;
    jtag->pending_write_cmd = cmd;

    return len;  // all data consumed that was there
  }

  // there is either no data to be sent or the data present is sufficient for
  // the full transfer
  pio_jtag_write_tdi_read_tdo(&jtag->pio, (cmd&8)?1:0, (cmd & 0x10)?buf:NULL,
			      (cmd & 0x20)?(jtag->reply_buffer + jtag->reply_len + 2):NULL,
			      shift_len*8);
  if(cmd & 0x20) jtag->reply_len += shift_len;
  
  // W-TDI set? There was payload used
  return (cmd & 0x10)?shift_len:0;
}

static uint16_t mpsse_shift_parse(struct jtag *jtag, uint8_t *buf, uint16_t len) {
  // get command byte
  uint8_t cmd = jtag->cmd_buf.data.cmd.code;

  /* mpsse command bits  
     0: W-VE  1: BIT  2: R-VE  3: LSB  4: W-TDI  5: R-TDO  6: W-TMS  7: 0 */

  // check if it's an allowed command as not all possible command bit patterns are
  // actually valid. 
  if((cmd != 0x10) && (cmd != 0x11) && (cmd != 0x12) && (cmd != 0x13) &&
     (cmd != 0x18) && (cmd != 0x19) && (cmd != 0x1a) && (cmd != 0x1b) &&
     (cmd != 0x20) && (cmd != 0x22) && (cmd != 0x24) && (cmd != 0x26) &&
     (cmd != 0x28) && (cmd != 0x2a) && (cmd != 0x2c) && (cmd != 0x2e) &&
     (cmd != 0x31) && (cmd != 0x33) && (cmd != 0x34) && (cmd != 0x36) &&
     (cmd != 0x39) && (cmd != 0x3b) && (cmd != 0x3c) && (cmd != 0x3e) &&
     // TMS commands
     (cmd != 0x4a) && (cmd != 0x4b) && (cmd != 0x6a) && (cmd != 0x6b) &&
     (cmd != 0x6e) && (cmd != 0x6f)) {

    // reply with 0xfa / bad command
    jtag->reply_buffer[2+jtag->reply_len++] = 0xfa;    
    return 0;
  }
  
  if(cmd & 2) return mpsse_shift_bits(jtag);
  else        return mpsse_shift_bytes(jtag, buf, len);
}
  
static uint16_t mpsse_parse_cmd(struct jtag *jtag, uint8_t *buf, uint16_t len) {
  if(jtag->cmd_buf.data.cmd.code & 0x80)
    return mpsse_cmd_parse(jtag);
    
  return mpsse_shift_parse(jtag, buf, len);
}

static void check_for_outgoing_data(struct jtag *jtag) {
  if(jtag->tx_pending) return;
  
  // check if there's now data in the reply buffer and request to return it
  if(jtag->reply_len) {
#ifdef DEBUG_REPLY
    printf("REPLY: %d\n", jtag->reply_len);
    hexdump(jtag->reply_buffer+2, jtag->reply_len);
#endif
    
    // data is always stored from byte 2 on in the reply buffer, so that the
    // status can be placed in front
    memcpy(jtag->reply_buffer, REPLY_STATUS, 2);

    // as a full speed device we can return max 62 bytes per USB transfer
    if(jtag->reply_len >= 62) {
      usb_start_transfer(usb_get_endpoint_configuration(jtag->eps[0]), jtag->reply_buffer, 64);
      // shift data down
      memmove(jtag->reply_buffer+2, jtag->reply_buffer+64, REPLY_BUFFER_SIZE-64);
      jtag->reply_len -= 62;
    } else {    
      // Send all data back to host
      usb_start_transfer(usb_get_endpoint_configuration(jtag->eps[0]), jtag->reply_buffer, jtag->reply_len+2);
      jtag->reply_len = 0;
    }
    jtag->tx_pending = true;

    if(jtag->rx_disabled && check_reply_buffer(jtag)) {
#ifdef DEBUG_FLOWCONTROL
      printf("FLOW: re-enable receiver\n");
#endif
      jtag->rx_disabled = false;
      usb_start_transfer(usb_get_endpoint_configuration(jtag->eps[1]), NULL, 64);      
    }
  } else {
#ifdef DEBUG_REPLY
    printf("REPLY: no pending data\n");
#endif
    
    usb_start_transfer(usb_get_endpoint_configuration(jtag->eps[0]), REPLY_STATUS, 2);    
    jtag->tx_pending = true;
  }
}

// get command size incl. length bytes or the like. But without the
// payload of the byte stream command
static uint8_t mpsse_cmd_size(uint8_t cmd) {
  if(cmd & 0x80) {
    // generic MPSSE command

    // command and one more byte
    if((cmd == 0x90) ||   // CPUMode read short address
       (cmd == 0x8e))     // clock for n bits with no data transfer
      return 2;	
	
    // command and two more bytes
    if((cmd == 0x80) || (cmd == 0x82) || // set data bits
       (cmd == 0x86) || // set divisor
       (cmd == 0x91) || // CPUMode read extended address
       (cmd == 0x92) || // CPUMode write short address

       (cmd == 0x8f) || // clock for n*8 bits with no data transfer
       (cmd == 0x9c) || // clock for n*8 bits with no data transfer until gpio high
       (cmd == 0x9d) || // clock for n*8 bits with no data transfer until gpio low
       (cmd == 0x9e))   // set IO to only drive on a '0' and tristate on '1'
      return 3;

    // command and three more bytes
    if(cmd == 0x93)     // CPUMode write extended address
      return 4;

    // otherwise no additional bytes needed, just the command itself
    return 1;
  }

  // shift commands
  if(cmd & 0x02) {
    // bit commands have an additional  byte length and a byte payload ...
    if(cmd & 0x50)
      return 3;

    // ... unless they are read-only, then they have only the byte length
    return 2;
  }
  
  // byte shift commands always need an additional two byte length
  return 3;
}

static void mpsse_parse(struct jtag *jtag, uint8_t *buf, uint16_t len) {
  if(jtag->mode == 1) {
    printf("BITBANG\n");

    port_gpio_set(jtag, buf[0]);
    // return input state
    jtag->reply_buffer[jtag->reply_len + 2] = port_gpio_get(jtag);
    jtag->reply_len += 1;
  }
  
  // check if this port is in mpsse mode at all
  else if(jtag->mode == 2) {
    // check if there are remaining bytes to shift from previous request
    if(jtag->pending_writes) {
      uint16_t bytes2shift = (len < jtag->pending_writes)?len:jtag->pending_writes;
#ifdef DEBUG_TRUNCATION
      printf("--> add %d of %d\n", bytes2shift, jtag->pending_writes);
#endif
      
      pio_jtag_write_tdi_read_tdo(&jtag->pio, (jtag->pending_write_cmd&8)?1:0, buf,
			      (jtag->pending_write_cmd & 0x20)?(jtag->reply_buffer + jtag->reply_len + 2):NULL,
			      (uint32_t)bytes2shift*8);
      if(jtag->pending_write_cmd & 0x20) jtag->reply_len += bytes2shift;
      check_reply_buffer(jtag);  // here for debugging only
      
      jtag->pending_writes -= bytes2shift;
      
      len -= bytes2shift;
      buf += bytes2shift;
      if(!len) return;
    }

    // process the entire payload
    while(len) {
      // add byte to command buffer. We don't need to check for buffer overflows since the
      // following check against mpsse_cmd_size() will always trigger before
      jtag->cmd_buf.data.bytes[jtag->cmd_buf.len++] = *buf++;
      len--;

      // check if there's a full command in buffer
      if(jtag->cmd_buf.len && mpsse_cmd_size(jtag->cmd_buf.data.cmd.code) <= jtag->cmd_buf.len) {
	int x = mpsse_parse_cmd(jtag, buf, len);
	buf += x;
	len -= x;
	
	// flush the command buffer
	jtag->cmd_buf.len = 0;
      }
    }
  } else {
    printf("Ignoring DATA in default mode\n");
    hexdump(buf, len);
  }
}

// pending outgoing data has been sent to host
void ep1_in_handler(__unused uint8_t *buf, uint16_t len) {
  gpio_put(PICO_DEFAULT_LED_PIN, 1);
  
  // EP1 handles incoming data for port 0
  struct jtag *jtag = &dev_config.ports[0].jtag;
  
#ifdef DEBUG_BULK0
  printf("EP1 >>>>>>>>>> Sent %d bytes to host\n", len);
  hexdump(buf, len);
#endif

  jtag->tx_pending = false;
  check_for_outgoing_data(jtag);

  gpio_put(PICO_DEFAULT_LED_PIN, 0);
}

void ep2_out_handler(uint8_t *buf, uint16_t len) {
  gpio_put(PICO_DEFAULT_LED_PIN, 1);
  
  // EP2 handles outgoing data for port 0
  struct jtag *jtag = &dev_config.ports[0].jtag;

#ifdef DEBUG_BULK0
  printf("EP2 >>>>>>>>>>>> RX <<<<<<<<<<<<<\n");
  hexdump(buf, len);
#endif
  
  mpsse_parse(jtag, buf, len);
  
  // re-enable receiver
  if(!check_reply_buffer(jtag))
    jtag->rx_disabled = true;
  else
    usb_start_transfer(usb_get_endpoint_configuration(EP2_OUT_ADDR), NULL, 64);

  gpio_put(PICO_DEFAULT_LED_PIN, 0);
}

void ep3_in_handler(__unused uint8_t *buf, uint16_t len) {
  gpio_put(PICO_DEFAULT_LED_PIN, 1);
  
  // EP3 handles incoming data for port 1
  struct jtag *jtag = &dev_config.ports[1].jtag;

#ifdef DEBUG_BULK1
  printf("EP3 >>>>>>>>>> Sent %d bytes to host\n", len);
  hexdump(buf, len);
#endif
  
  jtag->tx_pending = false;
  check_for_outgoing_data(jtag);

  gpio_put(PICO_DEFAULT_LED_PIN, 0);
}

void ep4_out_handler(uint8_t *buf, uint16_t len) {
  gpio_put(PICO_DEFAULT_LED_PIN, 1);

  // EP4 handles outgoing data for port 1
  struct jtag *jtag = &dev_config.ports[1].jtag;

#ifdef DEBUG_BULK1
  printf("EP4 >>>>>>>>>>>> RX <<<<<<<<<<<<<\n");
  hexdump(buf, len);
#endif
  
  mpsse_parse(jtag, buf, len);
  
  // re-enable receiver
  if(!check_reply_buffer(jtag))
    jtag->rx_disabled = true;
  else
    usb_start_transfer(usb_get_endpoint_configuration(EP4_OUT_ADDR), NULL, 64);

  gpio_put(PICO_DEFAULT_LED_PIN, 0);
}

void jtag_init(pio_jtag_inst_t* jtag_pio) {
  printf(">> Initializing PIO JTAG #%d <<\n", PIO_INDEX(jtag_pio));
  
  pio_jtag_init(jtag_pio, 1000);      // initially go with 1 Mhz TCK clock
  pio_jtag_enable(jtag_pio, false);   // start with jtag disabled and the JTAG pins switched to input

  // handle the upper four GPIO pins of the port if present
  for(int i=0;i<4;i++) {
    if(jtag_pio->pins_upper[i] != -1) {
      printf("GPIO %d = pin %d\n", i, jtag_pio->pins_upper[i]);

      // by default the pins are inputs
      gpio_init(jtag_pio->pins_upper[i]);
      gpio_set_dir(jtag_pio->pins_upper[i], GPIO_IN);      
    }
  }
}

int main(void) {
  // lowering the system clock from 125MHz to 120Mhz allows to
  // run the JTAG at exactly 6MHz
  set_sys_clock_khz(120000, true);

  stdio_init_all();
  printf("<<<<<<<<<<<<<<<<< Pico MPSSE >>>>>>>>>>>>>>>>>>\n");

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  gpio_put(PICO_DEFAULT_LED_PIN, 1);

  jtag_init(&dev_config.ports[0].jtag.pio);
  jtag_init(&dev_config.ports[1].jtag.pio);

  usb_device_init();
  
  // Wait until configured
  while(!configured) tight_loop_contents();
  
  // get ready to tx to host
  check_for_outgoing_data(&dev_config.ports[0].jtag);
  check_for_outgoing_data(&dev_config.ports[1].jtag);
  
  // Get ready to rx from host
  usb_start_transfer(usb_get_endpoint_configuration(EP2_OUT_ADDR), NULL, 64);
  usb_start_transfer(usb_get_endpoint_configuration(EP4_OUT_ADDR), NULL, 64);

  // led off
  gpio_put(PICO_DEFAULT_LED_PIN, 0);
  
  // Everything is interrupt driven so just loop here
  while(1) tight_loop_contents();
}
