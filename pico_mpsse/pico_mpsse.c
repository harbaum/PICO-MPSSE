/*
 * pico_mpsse.c
 *
 * FTDI MPSSE implementation for the Raspberry Pi Pico / RP2040
 * Intended to be used with openfpgaloader to load FPGAs
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

// #define DEBUG_BULK0
// #define DEBUG_BULK1
// #define DEBUG_SHIFT
// #define DEBUG_GPIO

#define FTDI_REPLY_STATUS   "\x32\x60"

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

// Struct defining the device configuration
static struct usb_device_configuration dev_config = {
        .device_descriptor = &device_descriptor,
        .device_qualifier_descriptor = &device_qualifier_descriptor,

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
	.ports[0].interface_descriptor = &interface_descriptor_p0,
	.ports[0].endpoints = { {
	    .descriptor = &ep1_in,
	    .handler = &ep1_in_handler,
	    .endpoint_control = &usb_dpram->ep_ctrl[0].in,
	    .buffer_control = &usb_dpram->ep_buf_ctrl[1].in,
	    .data_buffer = &usb_dpram->epx_data[0 * 512],
	  }, {
	    .descriptor = &ep2_out,
	    .handler = &ep2_out_handler,
	    .endpoint_control = &usb_dpram->ep_ctrl[1].out,
	    .buffer_control = &usb_dpram->ep_buf_ctrl[2].out,
	    .data_buffer = &usb_dpram->epx_data[1 * 512],
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
	.ports[1].interface_descriptor = &interface_descriptor_p1,
	.ports[1].endpoints = { {
	    .descriptor = &ep3_in,
	    .handler = &ep3_in_handler,
	    .endpoint_control = &usb_dpram->ep_ctrl[2].in,
	    .buffer_control = &usb_dpram->ep_buf_ctrl[3].in,
	    .data_buffer = &usb_dpram->epx_data[2 * 512],
	  }, {
	    .descriptor = &ep4_out,
	    .handler = &ep4_out_handler,
	    .endpoint_control = &usb_dpram->ep_ctrl[3].out,
	    .buffer_control = &usb_dpram->ep_buf_ctrl[4].out,
	    .data_buffer = &usb_dpram->epx_data[3 * 512],
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

void usb_handle_device_qualifier_descriptor(volatile struct usb_setup_packet *pkt) {
    const struct usb_device_qualifier_descriptor *d = dev_config.device_qualifier_descriptor;
    // EP0 in
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_IN_ADDR);
    // Always respond with pid 1
    ep->next_pid = 1;
    usb_start_transfer(ep, (uint8_t *) d, MIN(sizeof(struct usb_device_qualifier_descriptor), pkt->wLength));
}

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
    if (pkt->wLength >= d->wTotalLength) {
      for (uint p = 0; p < 2; p++) {
	// send both ports interface desciptors with both endpoints each
        memcpy((void *) buf, dev_config.ports[p].interface_descriptor, sizeof(struct usb_interface_descriptor));
        buf += sizeof(struct usb_interface_descriptor);
	memcpy((void *) buf, dev_config.ports[p].endpoints[0].descriptor, sizeof(struct usb_endpoint_descriptor));
	buf += sizeof(struct usb_endpoint_descriptor);
	memcpy((void *) buf, dev_config.ports[p].endpoints[1].descriptor, sizeof(struct usb_endpoint_descriptor));
	buf += sizeof(struct usb_endpoint_descriptor);
      }
    }

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

void usb_handle_setup_packet(void) {
    volatile struct usb_setup_packet *pkt = (volatile struct usb_setup_packet *) &usb_dpram->setup_packet;
    uint8_t req_direction = pkt->bmRequestType;
    uint8_t req = pkt->bRequest;

    // Reset PID to 1 for EP0 IN
    usb_get_endpoint_configuration(EP0_IN_ADDR)->next_pid = 1u;

    if (req_direction == USB_DIR_OUT) {
        if (req == USB_REQUEST_SET_ADDRESS) {
            usb_set_device_address(pkt);
        } else if (req == USB_REQUEST_SET_CONFIGURATION) {
            usb_set_device_configuration(pkt);
        } else {
	    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
            printf("Other OUT request (0x%x)\r\n", pkt->bRequest);
        }
    } else if (req_direction == USB_DIR_IN) {
        if (req == USB_REQUEST_GET_STATUS) {
	  usb_handle_status_request(pkt);
	  
        } else if (req == USB_REQUEST_GET_DESCRIPTOR) {
            switch (pkt->wValue >> 8) {
                case USB_DT_DEVICE:
                    usb_handle_device_descriptor(pkt);
                    printf("GET DEVICE DESCRIPTOR\r\n");
                    break;

                case USB_DT_CONFIG:
                    usb_handle_config_descriptor(pkt);
                    printf("GET CONFIG DESCRIPTOR\r\n");
                    break;

                case USB_DT_STRING:
                    usb_handle_string_descriptor(pkt);
                    printf("GET STRING DESCRIPTOR\r\n");
                    break;

	        case USB_DT_DEVICE_QUALIFIER:
		    usb_handle_device_qualifier_descriptor(pkt);
		    printf("GET DEVICE QUALIFIER DESCRIPTOR\r\n");
		    break;

                default:
		    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
                    printf("Unhandled GET_DESCRIPTOR type 0x%x\r\n", pkt->wValue >> 8);
            }
        } else {
            printf("Other IN request (0x%x)\r\n", pkt->bRequest);
        }
    }
    else if (req_direction == 0x40) {

      switch(pkt->bRequest) {
      case 0x00:
	printf("FTDI RESET, #%d=%d\n", pkt->wIndex, pkt->wValue);
	break;

      case 0x03:
	printf("FTDI SET BAUD RATE, #%d=%d\n", pkt->wIndex, pkt->wValue);
	break;

      case 0x05:
	printf("FTDI POLL MODEM STATUS, #%d=0x%02x\n", pkt->wIndex, pkt->wValue);
	break;

      case 0x09:
	printf("FTDI SET LATENCY TIMER, #%d=%d\n", pkt->wIndex, pkt->wValue);
	break;

      case 0x0b:
	printf("FTDI SET BITMODE, #%d=0x%02x\n", pkt->wIndex, pkt->wValue);
	if(pkt->wIndex >= 1 && pkt->wIndex <= 2) {
	  struct jtag *jtag = &dev_config.ports[pkt->wIndex-1].jtag;
	  
	  jtag->mode = pkt->wValue>>8;

	  // in bitbang mode setup the direction of the upper four bits
	  jtag->gpio_dir = (pkt->wValue>>4)&0x0f;
	    
	  for(int i=0;i<4;i++) {
	    if(jtag->pio.pins_upper[i]) {
#ifdef DEBUG_GPIO
	      printf("  GPIO %d:", jtag->pio.pins_upper[i]);
#endif	  
	      gpio_put(jtag->pio.pins_upper[i], 0);  // TODO: Check why this is needed. Otherwise FPGA won't boot
	      gpio_set_dir(jtag->pio.pins_upper[i], (jtag->gpio_dir&(1<<i))?GPIO_OUT:GPIO_IN);
#ifdef DEBUG_GPIO
	      printf(" %s\n", (jtag->gpio_dir&(1<<i))?"output":"input");
#endif
	    }
	  }	  
	}
	break;

      default:
	printf("Unsupported vendor request %02x, #%d=%d\n", pkt->bRequest, pkt->wIndex, pkt->wValue);
	break;	
      }
      usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
    }
    else if (req_direction == 0xc0) {
      printf("VENDOR IN %02x\n", pkt->bRequest);
    }
    else
      printf("Uknown request type %02x\n", pkt->bmRequestType);

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

static uint16_t mpsse_cmd_parse(struct jtag *jtag, uint8_t *buf, uint16_t len) {

  switch(buf[0]) {
  case 0x80:
  case 0x82:
    if(len < 3) return 0;  // needs at least 3 bytes
    printf("MPSSE: Set data bits %s value %02x, dir %02x\n", (buf[0]&2)?"high":"low", buf[1], buf[2]);
    
    /* we currently only support the lower bits */
    if(!(buf[0]&2)) {
      // second payload byte is direction. Lowest bits 0x0b is JTAG mapping
      jtag_enable(&jtag->pio, (buf[2] & 0x0f) == 0x0b);

      // handle upper gpio if present
      for(int i=0;i<4;i++) {
	if(jtag->pio.pins_upper[i]) {
#ifdef DEBUG_GPIO
	  printf("  GPIO %d:", jtag->pio.pins_upper[i]);
#endif	  
	  gpio_put(jtag->pio.pins_upper[i], 0);  // TODO: Check why this is needed. Otherwise FPGA won't boot
	  gpio_set_dir(jtag->pio.pins_upper[i], (buf[2]&(1<<(i+4)))?GPIO_OUT:GPIO_IN);
#ifdef DEBUG_GPIO
	  printf(" %s", (buf[2]&(1<<(i+4)))?"output":"input");
#endif	  

	  // set value id pin is configured as output
	  if(buf[2]&(1<<(i+4))) {
	    gpio_put(jtag->pio.pins_upper[i], (buf[1]&(1<<(i+4)))?1:0);
#ifdef DEBUG_GPIO
	    printf(" = %d\n", (buf[1]&(1<<(i+4)))?1:0);	    
#endif	  
	  }
#ifdef DEBUG_GPIO
	  else printf("\n");
#endif	  
	}
      }
    }
    return 3;
                
  case 0x84:
    printf("MPSSE: Connect loopback\n");
    return 1;

  case 0x85:
    printf("MPSSE: Disconnect loopback\n");
    return 1;

  case 0x86: {
    if(len < 3) return 0;  // needs at least 3 bytes    
    int divisor = buf[1] + 256*buf[2];
    int rate = 12000000 / ((1+divisor) * 2);
    printf("MPSSE: Set TCK/SK Divisor to %d = %d Mhz\n", divisor, rate/1000000);
    jtag_set_clk_freq(&jtag->pio, rate/1000);    
    return 3;
  }

  case 0x87:
    // printf("MPSSE: Flush\n");
    return 1;

  case 0x8a:
    printf("MPSSE: Disable div by 5 (60MHz master clock)\n");
    return 1;
        
  case 0x8b:
    printf("MPSSE: Enable div by 5 (12MHz master clock)\n");
    return 1;

  }
  return 1;
}

// According to the AN 108, only six TMS opcodes actually exist.
// These are 0x4a, 0x4b, 0x6a, 0x6b, 0x6e and 0x6f
// 0x4a = 0b01001010
// 0x4b = 0b01001011
// 0x6a = 0b01101010
// 0x6b = 0b01101011
// 0x6e = 0b01101110
// 0x6f = 0b01101111

// BIT is always set, LSB is always set and W-TDI is never set 
// -> no TMS byte write exists

static uint16_t mpsse_shift_parse(struct jtag *jtag, uint8_t *buf, uint16_t len) {
  // calculate data shift length
  uint16_t cmd_len = 0;
  uint16_t shift_len = 0;
  uint8_t cmd = buf[0];

  // printf("CMD "); hexdump(buf, len);
  
  /* mpsse command bits  
     0: W-VE
     1: BIT
     2: R-VE
     3: LSB
     4: W-TDI
     5: R-TDO
     6: W-TMS
     7: 0
  */

  if(cmd & 2) {
    // length is in bits, total command length is 3
    if(len < 2) return 0;    // needs at last cmd,len
    shift_len = buf[1] + 1;  // shift length was given in bits-1
    // write TDI or TMS?
    if(cmd & 0x50) {
      // write bits has one additional payload byte    
      if(len < 3) return 0;
      cmd_len = 3;
      buf += 2;
    } else {
      // read-only bits
      cmd_len = 2;
      buf += 1;
    }

#ifdef DEBUG_SHIFT
    printf("MPSSE: shift %d bits\n", shift_len);
#endif

    // new PIO code
    if(cmd & 0x40) {
      // printf("JTAG TMS BIT WRITE %d ", shift_len); hexdump(buf, 1);
      jtag_write_tms(&jtag->pio, (buf[0]&0x80)?1:0, buf,
		     (cmd & 0x20)?(jtag->reply_buffer + jtag->reply_len + 2):NULL, shift_len);
      if(cmd & 0x20) jtag->reply_len += (shift_len+7)/8;

    } else {
      // printf("JTAG TDI BIT WRITE %d ", shift_len); hexdump(buf, 1);
      jtag_write_tdi_read_tdo(&jtag->pio, (cmd & 0x10)?buf:NULL,
			      (cmd & 0x20)?(jtag->reply_buffer + jtag->reply_len + 2):NULL, shift_len);
      if(cmd & 0x20) jtag->reply_len += (shift_len+7)/8;
    }
    
    return cmd_len;    
  }

  // length is given in bytes and command length is variable
  if(len < 3) return 0;                // needs at least cmd,len16
  shift_len = buf[1] + 256*buf[2] + 1; // shift length was given in bytes-1

  // W-TDI set?
  if(cmd & 0x10) cmd_len = 3+shift_len;
  else           cmd_len = 3;
  
#ifdef DEBUG_SHIFT
  printf("MPSSE: shift %d bytes (%d avail)\n", shift_len, len);
#endif

  // it may happen that we are supposed to shift out more bits than we have payload
  if((cmd & 0x10) && (3+shift_len > len)) {
    // printf("---------------------> truncating write %d to %d\n", shift_len, len-3);

    jtag_write_tdi_read_tdo(&jtag->pio, buf+3,
	    (cmd & 0x20)?(jtag->reply_buffer + jtag->reply_len + 2):NULL,(len-3)*8);
    if(cmd & 0x20) jtag->reply_len += (len-3);
    jtag->pending_writes = shift_len-(len-3);
    jtag->pending_write_cmd = cmd;
    
    return len;   // all consumed that was there so far
  }

  jtag_write_tdi_read_tdo(&jtag->pio, (cmd & 0x10)?(buf+3):NULL,
			  (cmd & 0x20)?(jtag->reply_buffer + jtag->reply_len + 2):NULL, shift_len*8);
  if(cmd & 0x20) jtag->reply_len += shift_len;
  return cmd_len;
}
  
static uint16_t mpsse_parse(struct jtag *jtag, uint8_t *buf, uint16_t len) {
  if(!len) return 0;  // nothing parsed as there was nothing to parse

  if(buf[0] & 0x80)
    return mpsse_cmd_parse(jtag, buf, len);

  return mpsse_shift_parse(jtag, buf, len);
}

static void check_for_outgoing_data(struct jtag *jtag) {
  if(jtag->tx_pending) return;
  
  // check if there's now data in the reply buffer and request to return it
  if(jtag->reply_len) {
    //printf("REPLY: %d\n", jtag->reply_len);
    //hexdump(jtag->reply_buffer+2, jtag->reply_len);
    
    // data is always stored from byte 2 on in the reply buffer, so that the
    // ftdi status can be placed in front
    memcpy(jtag->reply_buffer, FTDI_REPLY_STATUS, 2);

    // as a full speed device we can return max 62 bytes per USB transfer
    if(jtag->reply_len > 62) {
      usb_start_transfer(usb_get_endpoint_configuration(jtag->eps[0]), jtag->reply_buffer, 64);
      // shift data down
      memmove(jtag->reply_buffer, jtag->reply_buffer+62, 194);  // reply buffer is 256 bytes
      jtag->reply_len -= 62;
    } else {    
      // Send all data back to host
      usb_start_transfer(usb_get_endpoint_configuration(jtag->eps[0]), jtag->reply_buffer, jtag->reply_len+2);
      jtag->reply_len = 0;
    }
    jtag->tx_pending = true;
  } else {
    // printf("REPLY: no pending data\n");
    
    usb_start_transfer(usb_get_endpoint_configuration(jtag->eps[0]), FTDI_REPLY_STATUS, 2);    
    jtag->tx_pending = true;
  }
}

// pending outgoing data has been sent to host
void ep1_in_handler(__unused uint8_t *buf, uint16_t len) {
  // EP1 handles incoming data for port 0
  struct jtag *jtag = &dev_config.ports[0].jtag;
  
#ifdef DEBUG_BULK0
  printf("EP1 >>>>>>>>>> Sent %d bytes to host\n", len);
  hexdump(buf, len);
#endif

  jtag->tx_pending = false;
  check_for_outgoing_data(jtag);
}

static void mpsse_parse_all(struct jtag *jtag, uint8_t *buf, uint16_t len) {
  if(jtag->mode == 1) {
    printf("BITBANG\n");

    // handle upper four bits
    for(int i=0;i<4;i++) {
      if((jtag->pio.pins_upper[i] != -1) && jtag->gpio_dir & (1<<i)) {	
	printf("drive %d to %d\n", 4+i, buf[0] & (1<<(i+4))?1:0);
	gpio_put(jtag->pio.pins_upper[i], (buf[0]&(1<<(i+4)))?1:0);
      }
    }

    // TODO: return input state
  }
  
  // check if this port is in mpsse mode at all
  if(jtag->mode == 2) {
    // check if there are remaining bytes to shift from previous request
    if(jtag->pending_writes) {
      uint16_t bytes2shift = (len < jtag->pending_writes)?len:jtag->pending_writes;
      // printf("add %d\n", bytes2shift);    
      
      jtag_write_tdi_read_tdo(&jtag->pio, buf,
			      (jtag->pending_write_cmd & 0x20)?(jtag->reply_buffer + jtag->reply_len + 2):NULL,
			      (uint32_t)bytes2shift*8);
      if(jtag->pending_write_cmd & 0x20) jtag->reply_len += bytes2shift;
      
      jtag->pending_writes -= bytes2shift;
      
      len -= bytes2shift;
      buf += bytes2shift;
      if(!len) return;
    }
    
    int x = mpsse_parse(jtag, buf, len);
    while(x && len) {
      buf += x;
      len -= x;
      x = mpsse_parse(jtag, buf, len);
    }
  }

  //  if(len) printf("---------------- leftover %d ----------------\n", len);
}

void ep2_out_handler(uint8_t *buf, uint16_t len) {
  // EP2 handles outgoing data for port 0
  struct jtag *jtag = &dev_config.ports[0].jtag;

#ifdef DEBUG_BULK0
  printf("EP2 >>>>>>>>>>>> RX <<<<<<<<<<<<<\n");
  hexdump(buf, len);
#endif
  
  mpsse_parse_all(jtag, buf, len);
  
  // re-enable receiver
  usb_start_transfer(usb_get_endpoint_configuration(EP2_OUT_ADDR), NULL, 64);
}

void ep3_in_handler(__unused uint8_t *buf, uint16_t len) {
  // EP3 handles incoming data for port 1
  struct jtag *jtag = &dev_config.ports[1].jtag;

#ifdef DEBUG_BULK1
  printf("EP3 >>>>>>>>>> Sent %d bytes to host\n", len);
  hexdump(buf, len);
#endif
  
  jtag->tx_pending = false;
  check_for_outgoing_data(jtag);
}

void ep4_out_handler(uint8_t *buf, uint16_t len) {
  // EP4 handles outgoing data for port 1
  struct jtag *jtag = &dev_config.ports[1].jtag;

#ifdef DEBUG_BULK1
  printf("EP4 >>>>>>>>>>>> RX <<<<<<<<<<<<<\n");
  hexdump(buf, len);
#endif
  
  mpsse_parse_all(jtag, buf, len);
  
  // re-enable receiver
  usb_start_transfer(usb_get_endpoint_configuration(EP4_OUT_ADDR), NULL, 64);
}

void pio_jtag_init(pio_jtag_inst_t* jtag_pio) {
  printf(">> Initializing PIO JTAG #%d <<\n", (jtag_pio->pio == pio0)?1:2);
  
  init_jtag(jtag_pio, 1000);      // initially go with 1 Mhz TCK clock
  jtag_enable(jtag_pio, false);   // start with jtag disabled and the JTAG pins switched to input

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
    
  pio_jtag_init(&dev_config.ports[0].jtag.pio);
  pio_jtag_init(&dev_config.ports[1].jtag.pio);
    
  usb_device_init();
  
  // Wait until configured
  while(!configured) tight_loop_contents();
  
  // get ready to tx to host
  check_for_outgoing_data(&dev_config.ports[0].jtag);
  check_for_outgoing_data(&dev_config.ports[1].jtag);
  
  // Get ready to rx from host
  usb_start_transfer(usb_get_endpoint_configuration(EP2_OUT_ADDR), NULL, 64);
  usb_start_transfer(usb_get_endpoint_configuration(EP4_OUT_ADDR), NULL, 64);
  
  // Everything is interrupt driven so just loop here
  while(1) tight_loop_contents();
}
