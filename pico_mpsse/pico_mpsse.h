/*
 *
 */

#ifndef PICO_MPSSE_H_
#define PICO_MPSSE_H_

#include "usb_common.h"
#include "pio_jtag.h"

#define MAX_PKT_SIZE  64 // should actually be 512, but this is no highspeed device ...

typedef void (*usb_ep_handler)(uint8_t *buf, uint16_t len);

// Struct in which we keep the endpoint configuration
struct usb_endpoint_configuration {
    const struct usb_endpoint_descriptor *descriptor;
    usb_ep_handler handler;

    // Pointers to endpoint + buffer control registers
    // in the USB controller DPSRAM
    volatile uint32_t *endpoint_control;
    volatile uint32_t *buffer_control;
    volatile uint8_t *data_buffer;

    // Toggle after each packet (unless replying to a SETUP)
    uint8_t next_pid;
};

#define MODE_MPSSE  2

struct jtag {
  bool tx_pending;
  uint8_t reply_len;
  uint8_t reply_buffer[1024];
  uint8_t eps[2];
  uint16_t pending_writes;
  uint8_t pending_write_cmd;
  uint8_t mode;  // 2=MPSSE
  uint8_t gpio_dir;
  pio_jtag_inst_t pio;
};

// Struct in which we keep the device configuration
struct usb_device_configuration {
  const struct usb_device_descriptor *device_descriptor;
  
  // device has two ports each 
  struct {
    const struct usb_interface_descriptor *interface_descriptor;
    struct usb_endpoint_configuration endpoints[2];

    struct jtag jtag;
  } ports[2];
  
  const struct usb_configuration_descriptor *config_descriptor;
  const struct usb_device_qualifier_descriptor *device_qualifier_descriptor;
  const unsigned char *lang_descriptor;
  const unsigned char **descriptor_strings;
  struct usb_endpoint_configuration endpoints[2];
};

#define EP0_IN_ADDR  (USB_DIR_IN  | 0)
#define EP0_OUT_ADDR (USB_DIR_OUT | 0)
#define EP1_IN_ADDR  (USB_DIR_IN  | 1)
#define EP2_OUT_ADDR (USB_DIR_OUT | 2)
#define EP3_IN_ADDR  (USB_DIR_IN  | 3)
#define EP4_OUT_ADDR (USB_DIR_OUT | 4)

// EP0 IN and OUT
static const struct usb_endpoint_descriptor ep0_out = {
        .bLength          = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = EP0_OUT_ADDR, // EP number 0, OUT from host (rx to device)
        .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
        .wMaxPacketSize   = 64,
        .bInterval        = 0
};

static const struct usb_endpoint_descriptor ep0_in = {
        .bLength          = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = EP0_IN_ADDR, // EP number 0, OUT from host (rx to device)
        .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
        .wMaxPacketSize   = 64,
        .bInterval        = 0
};

// Descriptors
static const struct usb_device_descriptor device_descriptor = {
        .bLength         = sizeof(struct usb_device_descriptor),
        .bDescriptorType = USB_DT_DEVICE,
        .bcdUSB          = 0x0200, // USB 2.0 device
        .bDeviceClass    = 0,      // Specified in interface descriptor
        .bDeviceSubClass = 0,      // No subclass
        .bDeviceProtocol = 0,      // No protocol
        .bMaxPacketSize0 = 64,     // Max packet size for ep0
        .idVendor        = 0x0403, // Vendor id
        .idProduct       = 0x6010, // Product ID
        .bcdDevice       = 0x0500, // Device revision number
        .iManufacturer   = 1,      // Manufacturer string index
        .iProduct        = 2,      // Product string index
        .iSerialNumber   = 3,      // Serial number
        .bNumConfigurations = 1    // One configuration
};

static const struct usb_device_qualifier_descriptor device_qualifier_descriptor = {
        .bLength         = sizeof(struct usb_device_qualifier_descriptor),
        .bDescriptorType = USB_DT_DEVICE_QUALIFIER,
        .bcdUSB          = 0x0200, // USB 2.0 device
        .bDeviceClass    = 0,      // Specified in interface descriptor
        .bDeviceSubClass = 0,      // No subclass
        .bDeviceProtocol = 0,      // No protocol
        .bMaxPacketSize0 = 64,     // Max packet size for ep0
        .bNumConfigurations = 1,   // One configuration
	.bRESERVED       = 0
};

static const struct usb_interface_descriptor interface_descriptor_p0 = {
        .bLength            = sizeof(struct usb_interface_descriptor),
        .bDescriptorType    = USB_DT_INTERFACE,
        .bInterfaceNumber   = 0,
        .bAlternateSetting  = 0,
        .bNumEndpoints      = 2,    // Interface has 2 endpoints
        .bInterfaceClass    = 0xff, // Vendor specific endpoint
        .bInterfaceSubClass = 0xff,
        .bInterfaceProtocol = 0xff,
        .iInterface         = 2
};

static const struct usb_endpoint_descriptor ep1_in = {
        .bLength          = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = EP1_IN_ADDR,
        .bmAttributes     = USB_TRANSFER_TYPE_BULK,
        .wMaxPacketSize   = MAX_PKT_SIZE,
        .bInterval        = 0
};

static const struct usb_endpoint_descriptor ep2_out = {
        .bLength          = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = EP2_OUT_ADDR,
        .bmAttributes     = USB_TRANSFER_TYPE_BULK,
        .wMaxPacketSize   = MAX_PKT_SIZE,
        .bInterval        = 0
};

static const struct usb_interface_descriptor interface_descriptor_p1 = {
        .bLength            = sizeof(struct usb_interface_descriptor),
        .bDescriptorType    = USB_DT_INTERFACE,
        .bInterfaceNumber   = 1,
        .bAlternateSetting  = 0,
        .bNumEndpoints      = 2,    // Interface has 2 endpoints
        .bInterfaceClass    = 0xff, // Vendor specific endpoint
        .bInterfaceSubClass = 0xff,
        .bInterfaceProtocol = 0xff,
        .iInterface         = 2
};

static const struct usb_endpoint_descriptor ep3_in = {
        .bLength          = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = EP3_IN_ADDR,
        .bmAttributes     = USB_TRANSFER_TYPE_BULK,
        .wMaxPacketSize   = MAX_PKT_SIZE,
        .bInterval        = 0
};

static const struct usb_endpoint_descriptor ep4_out = {
        .bLength          = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = EP4_OUT_ADDR,
        .bmAttributes     = USB_TRANSFER_TYPE_BULK,
        .wMaxPacketSize   = MAX_PKT_SIZE,
        .bInterval        = 0
};

static const struct usb_configuration_descriptor config_descriptor = {
        .bLength         = sizeof(struct usb_configuration_descriptor),
        .bDescriptorType = USB_DT_CONFIG,
        .wTotalLength    = (sizeof(config_descriptor) +
                            sizeof(interface_descriptor_p0) +
                            sizeof(ep1_in) + sizeof(ep2_out) +
                            sizeof(interface_descriptor_p1) +
                            sizeof(ep3_in) + sizeof(ep4_out)),
        .bNumInterfaces  = 2,
        .bConfigurationValue = 1, // Configuration 1
        .iConfiguration = 0,      // No string
        .bmAttributes = 0x80,     // attributes: bus powered
        .bMaxPower = 0x32         // 100ma
};

static const unsigned char lang_descriptor[] = {
        4,         // bLength
        0x03,      // bDescriptorType == String Descriptor
        0x09, 0x04 // language id = us english
};

static const unsigned char *descriptor_strings[] = {
        (unsigned char *) "Not FTDI",        // Vendor
        (unsigned char *) "Pico MPSSE"       // Product
};

#endif
