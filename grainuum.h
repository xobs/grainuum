/****************************************************************************
 * Grainuum Software USB Stack                                              *
 *                                                                          *
 * MIT License:                                                             *
 * Copyright (c) 2016 Sean Cross                                            *
 *                                                                          *
 * Permission is hereby granted, free of charge, to any person obtaining a  *
 * copy of this software and associated documentation files (the            *
 * "Software"), to deal in the Software without restriction, including      *
 * without limitation the rights to use, copy, modify, merge, publish,      *
 * distribute, distribute with modifications, sublicense, and/or sell       *
 * copies of the Software, and to permit persons to whom the Software is    *
 * furnished to do so, subject to the following conditions:                 *
 *                                                                          *
 * The above copyright notice and this permission notice shall be included  *
 * in all copies or substantial portions of the Software.                   *
 *                                                                          *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS  *
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF               *
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.   *
 * IN NO EVENT SHALL THE ABOVE COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,   *
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR    *
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR    *
 * THE USE OR OTHER DEALINGS IN THE SOFTWARE.                               *
 *                                                                          *
 * Except as contained in this notice, the name(s) of the above copyright   *
 * holders shall not be used in advertising or otherwise to promote the     *
 * sale, use or other dealings in this Software without prior written       *
 * authorization.                                                           *
 ****************************************************************************/

#ifndef _GRAINUUM_H
#define _GRAINUUM_H

#include <stdint.h>

/**
 * @brief   Extra fields for GrainuumState struct.
 * @note    You probably can ignore this.
 */
#ifndef GRAINUUM_STATE_EXTRA
#define GRAINUUM_STATE_EXTRA
#endif /* GRAINUUM_STATE_EXTRA */

/**
 * @brief   Extra fields for GrainuumUSB struct.
 * @note    Use this to store context and thread information.
 */
#ifndef GRAINUUM_EXTRA
#define GRAINUUM_EXTRA
#endif /* GRAINUUM_EXTRA */

#define SET_ADDRESS 5
#define SET_CONFIGURATION 9

enum usb_pids {
  USB_PID_RESERVED = 0xf0,
  USB_PID_OUT = 0xe1,
  USB_PID_ACK = 0xd2,
  USB_PID_DATA0 = 0xc3,
  USB_PID_PING = 0xb4,
  USB_PID_SOF = 0xa5,
  USB_PID_NYET = 0x96,
  USB_PID_DATA2 = 0x87,
  USB_PID_SPLIT = 0x78,
  USB_PID_IN = 0x69,
  USB_PID_NAK = 0x5a,
  USB_PID_DATA1 = 0x4b,
  USB_PID_ERR = 0x3c,
  USB_PID_SETUP = 0x2d,
  USB_PID_STALL = 0x1e,
  USB_PID_MDATA = 0x0f,
};

struct GrainuumUSB;
struct GrainuumState;
struct GrainuumConfig;

/* Function callbacks */

/* Each of these functions are called by the USB system to get a buffer.
 * On return, *data will point to the buffer, and the number of bytes
 * in the buffer will be returned.
 *
 * If the data does not exist, return 0.
 */
typedef int (*get_usb_descriptor_t)(struct GrainuumUSB *usb,
                                    const void *pkt,
                                    const void **data);
typedef void (*usb_set_config_num_t)(struct GrainuumUSB *usb,
                                     int configNum);

/*
 * Called when doing an OUT xfer (data to device) to get a buffer for
 * the specified endpoint.
 * It is up to the user to ensure the buffer is large enough.
 */
typedef void * (*usb_get_buffer_t)(struct GrainuumUSB *usb,
                                   uint8_t epnum,
                                   int32_t *size);

/*
 * When data is received (i.e. OUT EP), this function will be called.
 */
typedef int (*usb_data_in_t)(struct GrainuumUSB *usb,
                             uint8_t epnum,
                             uint32_t bytes,
                             const void *data);

/**
 * @brief   Called immediately after @p grainuumSendData() has queued data.
 * @note    This function can be used to e.g. sleep a thread.
 * @param[in] usb     pointer to the @p GrainuumUSB object
 * @param[in] epnum   endpoint number of the transfer
 * @param[in] data    pointer to the data being written
 * @param[in] size   number of bytes being written
 * @api
 */
typedef void (*usb_data_out_start_t)(struct GrainuumUSB *usb,
                                     int epnum,
                                     const void *data,
                                     int size);

/**
 * @brief   Called once all data has been sent.
 * @note    This function can be used to e.g. wake up a thread.
 * @param[out] usb     pointer to the @p GrainuumUSB object
 * @param[out] result  whether the transfer was successful (0), or had an error.
 * @api
 */
typedef int (*usb_data_out_finish_t)(struct GrainuumUSB *usb,
                                     int result);

/* Structure of a USB packet on the wire, plus size field */
struct usb_packet {
  union {
    struct {
      uint8_t pid;
      uint8_t data[10]; /* Including CRC */
    } __attribute((packed, aligned(4)));
    uint8_t raw_data[11];
  } __attribute((packed, aligned(4)));
  uint8_t size; /* Not including pid (so may be 0) */
  /* Checksum omitted */
} __attribute__((packed, aligned(4)));

/* USB Descriptors */

#define DT_DEVICE 0x01
#define DT_CONFIGURATION 0x02
#define DT_STRING 0x03
#define DT_INTERFACE 0x04
#define DT_ENDPOINT 0x05
#define DT_DEVICE_QUALIFIER 0x06
#define DT_OTHER_SPEED_CONFIGURATION 0x07
#define DT_INTERFACE_POWER 0x08

#define DT_HID 0x21
#define DT_HID_REPORT 0x22
#define DT_PID 0x23

struct usb_setup_packet {
  uint8_t bmRequestType;
  uint8_t bRequest;
  union {
    uint16_t wValue;
    struct {
      uint8_t wValueL;
      uint8_t wValueH;
    };
  };
  uint16_t wIndex;
  uint16_t wLength;
} __attribute__((packed, aligned(4)));

struct usb_device_descriptor {
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint16_t bcdUSB;
  uint8_t  bDeviceClass;
  uint8_t  bDeviceSubClass;
  uint8_t  bDeviceProtocol;
  uint8_t  bMaxPacketSize0;
  uint16_t idVendor;
  uint16_t idProduct;
  uint16_t bcdDevice;
  uint8_t  iManufacturer;
  uint8_t  iProduct;
  uint8_t  iSerialNumber;
  uint8_t  bNumConfigurations;
} __attribute__((packed, aligned(4)));

struct usb_configuration_descriptor {
  uint8_t  bLength;             /* Size of this descriptor, in bytes (9) */
  uint8_t  bDescriptorType;     /* DT_CONFIGURATION (2) */
  uint16_t wTotalLength;        /* Total length of this, plus sizeof(data) */
  uint8_t  bNumInterfaces;      /* Number of interfaces supported by config */
  uint8_t  bConfigurationValue; /* Value used by Set Configuration */
  uint8_t  iConfiguration;      /* index of string descriptor for config */
  uint8_t  bmAttributes;        /* Bitmap of attributes.  D7 must be 1. */
  uint8_t  bMaxPower;           /* Maximum power, in units of 2mA */
  uint8_t  data[];              /* Remaining descriptors */
} __attribute__((packed, aligned(4)));

struct usb_string_descriptor {
  uint8_t bLength;          /* sizeof(usb_string_descriptor) + sizeof(data) */
  uint8_t bDescriptorType;  /* DT_STRING (3) */
  uint8_t data[];           /* UTF-16LE string data or lang data(for string 0 */
} __attribute__((packed, aligned(4)));

struct usb_interface_descriptor {
  uint8_t bLength;            /* sizeof(usb_interface_descriptor) (9) */
  uint8_t bDescriptorType;    /* DT_INTERFACE (4) */
  uint8_t bInterfaceNumber;   /* Which interface this describes.  Usually 0. */
  uint8_t bAlternateSetting;  /* ??? */
  uint8_t bNumEndpoints;      /* Number of endpoints, minus 1 */
  uint8_t bInterfaceClass;    /* Class code */
  uint8_t bInterfaceSubclass; /* Class sub-code */
  uint8_t bInterfaceProtocol; /* Protocol code, assigned by USB */
  uint8_t iInterface;         /* Index of string for this interface */
} __attribute__((packed, aligned(4)));

struct usb_endpoint_descriptor {
  uint8_t  bLength;           /* sizeof(usb_endpoint_descriptor) (7) */
  uint8_t  bDescriptorType;   /* DT_ENDPOINT (5) */
  uint8_t  bEndpointAddress;  /* High bit 1:IN, 0:OUT. Lower 4-bits are EP# */
  uint8_t  bmAttributes;      /* 0=control, 2=bulk, 3=interrupt */
  uint16_t wMaxPacketSize;    /* Max packet size for this EP */
  uint8_t  bInterval;         /* Polling rate (in 1ms units) */
} __attribute__((packed, aligned(4)));

struct usb_hid_descriptor {
  uint8_t  bLength;           /* sizeof(usb_hid_descriptor) (9) */
  uint8_t  bDescriptorType;   /* DT_HID (0x21) */
  uint16_t bcdHID;            /* HID class version number, in BCD */
  uint8_t  bCountryCode;      /* Target country (usually 0) */
  uint8_t  bNumDescriptors;   /* Number of HID class descriptors (usually 1) */
  uint8_t  bReportDescriptorType;   /* Report descriptor type (usually 0x22) */
  uint16_t wReportDescriptorLength; /* Length of the HID/PID report descriptor */
} __attribute__((packed, aligned(4)));

#define GRAINUUM_BUFFER_ELEMENT_SIZE 12 /* 1 PID, 8 data, 2 CRC16, 1 size */
/* grainuum_buffer is aligned such that its first byte is on a word boundary.
 * This is because the first byte of every packet is a PID, which is
 * immediately discarded.  This leaves the remainder of the packet
 * word-aligned.
 */

#define GRAINUUM_BUFFER(name, sz)                           \
struct {                                                    \
  uint8_t  head;                                            \
  uint8_t  tail;                                            \
  uint8_t  padding;                                         \
  uint8_t  buffer[(sz) * GRAINUUM_BUFFER_ELEMENT_SIZE];     \
} name; uint8_t * name ## _head_ptr;
#define GRAINUUM_BUFFER_INIT(name)                          \
  do {                                                      \
    (name).head = 0;                                        \
    (name).tail = 0;                                        \
    name ## _head_ptr = (name).buffer;                      \
  } while(0)
#define GRAINUUM_BUFFER_ADVANCE(name)                       \
  do {                                                      \
    (name).head += GRAINUUM_BUFFER_ELEMENT_SIZE;            \
    if ((name).head >= sizeof((name).buffer))               \
      (name).head = 0;                                      \
    name ## _head_ptr = ((name).buffer + (name).head);      \
  } while(0)
#define GRAINUUM_BUFFER_TOP(name)                           \
  (&((name).buffer[(name).tail]))
#define GRAINUUM_BUFFER_REMOVE(name)                        \
  do {                                                      \
    (name).tail += GRAINUUM_BUFFER_ELEMENT_SIZE;            \
  if ((name).tail >= sizeof((name).buffer))                 \
    (name).tail = 0;                                        \
  } while(0)
#define GRAINUUM_BUFFER_IS_EMPTY(name)                      \
  ((name).head == (name).tail)
#define GRAINUUM_BUFFER_ENTRY(name)                         \
  name ## _head_ptr

/* Grainuum Structs */

struct GrainuumConfig {
  get_usb_descriptor_t      getDescriptor;
  usb_set_config_num_t      setConfigNum;
  usb_get_buffer_t          getReceiveBuffer;
  usb_data_in_t             receiveData;
  usb_data_out_start_t      sendDataStarted;
  usb_data_out_finish_t     sendDataFinished;
  void                     *data;
  struct GrainuumUSB       *usb;
} __attribute__((packed, aligned(4)));

struct GrainuumState {
  struct GrainuumUSB *usb;

  uint8_t data_in[8];

  const void *data_out;     /* Pointer to the data that's being sent */
  int32_t data_out_left;    /* How much data has yet to be sent */
  int32_t data_out_max;     /* The maximum number of bytes to send */
  int32_t data_out_epnum;   /* Which endpoint the data is for */

  struct usb_packet packet; /* Currently-queued packet */
  int packet_queued;        /* Whether a packet is queued */

  uint32_t tok_pos;         /* Position within the current token */
  void *tok_buf;            /* Buffer storing current token's data */
  uint8_t tok_epnum;        /* Last token's endpoint */

  uint8_t data_buffer;      /* Whether we're sending DATA0 or DATA1 */
  uint8_t packet_type;      /* PACKET_SETUP, PACKET_IN, or PACKET_OUT */

  uint8_t address;          /* Our configured address */

  GRAINUUM_STATE_EXTRA
} __attribute__((packed, aligned(4)));

struct GrainuumUSB {

  struct GrainuumConfig *cfg; /* Callbacks */
  int initialized;

  /* USB D- pin specification */
  uint32_t usbdnIAddr;
  uint32_t usbdnSAddr;
  uint32_t usbdnCAddr;
  uint32_t usbdnDAddr;
  uint32_t usbdnShift;

  /* USB D+ pin specification */
  uint32_t usbdpIAddr;
  uint32_t usbdpSAddr;
  uint32_t usbdpCAddr;
  uint32_t usbdpDAddr;
  uint32_t usbdpShift;

  uint32_t usbdnMask;
  uint32_t usbdpMask;

  uint32_t queued_size;
  uint32_t queued_epnum;
  const void *queued_data;

  struct GrainuumState state;     /* Associated state */

  GRAINUUM_EXTRA
} __attribute__((packed, aligned(4)));

#ifdef __cplusplus
extern "C" {
#endif

static inline void grainuumWritel(uint32_t value, uint32_t addr)
{
  *((volatile uint32_t *)addr) = value;
}

static inline uint32_t grainuumReadl(uint32_t addr)
{
  return *(volatile uint32_t *)addr;
}

/*===========================================================================*/
/* Weak hook functions.                                                      */
/*===========================================================================*/

/**
 * @brief   Called just before the USB device is plugged in.
 * @param[in] usb     pointer to the @p GrainuumUSB object
 * @api
 */
void grainuumConnectPre(struct GrainuumUSB *usb);

/**
 * @brief   Called just after the USB device is plugged in.
 * @param[in] usb     pointer to the @p GrainuumUSB object
 * @api
 */
void grainuumConnectPost(struct GrainuumUSB *usb);

/**
 * @brief   Called just before the USB device is unplugged.
 * @param[in] usb     pointer to the @p GrainuumUSB object
 * @api
 */
void grainuumDisconnectPre(struct GrainuumUSB *usb);

/**
 * @brief   Called just after the USB device is unplugged.
 * @param[in] usb     pointer to the @p GrainuumUSB object
 * @api
 */
void grainuumDisconnectPost(struct GrainuumUSB *usb);

/**
 * @brief   Called just before the USB device is first initialized.
 * @param[in] usb     pointer to the @p GrainuumUSB object
 * @api
 */
void grainuumInitPre(struct GrainuumUSB *usb);

/**
 * @brief   Called just before the USB device is first initialized.
 * @param[in] usb     pointer to the @p GrainuumUSB object
 * @api
 */
void grainuumInitPost(struct GrainuumUSB *usb);

/**
 * @brief   Called immediately after a packet has been received.
 * @note    This is called from an interrupt context.  Data will
 *          be stored in the buffer that was passed to @p grainuumCaptureI()
 * @param[in] usb     pointer to the @p GrainuumUSB object
 * @iclass
 */
void grainuumReceivePacket(struct GrainuumUSB *usb);

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/**
 * @brief   Returns nonzero if Grainuum has been initialized.
 * @param[in] usb     pointer to the @p GrainuumUSB object.
 * @return            nonzero if @p GrainuumUSB is initialized.
 * @retval 0          Object is not initilized.
 * @api
 */
int grainuumInitialized(struct GrainuumUSB *usb);

/**
 * @brief   Queues some data to be sent to the host.
 * @note    After the first 8 bytes, @p data must remain valid
 *          until the transfer has completed.  This generally
 *          means you can send const data stored in the text
 *          section, or small 8-byte packets.
 * @param[in] usb     pointer to the @p GrainuumUSB object.
 * @param[in] epnum   endpoint number of the transfer.
 * @param[in] data    pointer to the data being written.
 * @param[in] size    number of bytes being written.
 * @return            0 if the transfer completed successfully.
 * @retval 0          Transfer completed successfully.
 * @api
 */
int grainuumSendData(struct GrainuumUSB *usb, int epnum, const void *data, int size);

/**
 * @brief   Process one received packet through the Grainuum state machine.
 * @note    This feeds USB packets into the state machine.  It should not
 *          be called as part of an interrupt.
 * @param[in] usb     pointer to the @p GrainuumUSB object.
 * @param[in] packet  The USB packet that was most recently received, with byte 12 holding the size.
 * @api
 */
void grainuumProcess(struct GrainuumUSB *usb,
                     const uint8_t packet[12]);

/**
 * @brief   Initialize the Grainuum USB system.
 * @note    This is meant to run as part of an interrupt.  Pass
 *          the storage buffer in as @p samples.  The number
 *          of bytes that were read will be stored in the last
 *          byte of the array.  For best performance, make
 *          sure that @p sample is on byte 3 of a 4-byte boundary,
 *          so that samples[1] is on a word boundary. The @p GrainuumUSB
 *          object will start out disconnected.
 * @param[in] usb   Pointer to the @p GrainuumUSB object to initialize.
 * @param[in] link  Pointer to the @p GrainuumConfig object to use.
 * @api
 */
void grainuumInit(struct GrainuumUSB *usb, struct GrainuumConfig *link);

/**
 * @brief   Capture a USB packet from the wire.
 * @note    This is meant to run as part of an interrupt.  Pass
 *          the storage buffer in as @p samples.  The number
 *          of bytes that were read will be stored in the last
 *          byte of the array.  For best performance, make
 *          sure that @p sample is on byte 3 of a 4-byte boundary,
 *          so that samples[1] is on a  word boundary.
 * @param[in] usb     pointer to the @p GrainuumUSB object.
 * @param[in] packet  Buffer to store the read samples.
 * @api
 */
void grainuumCaptureI(struct GrainuumUSB *usb, uint8_t packet[12]);

/**
 * @brief   Internal function. Queues 8 bytes to be sent by the phy.
 * @note    This is an internal function, and is not meant to be called.
 *          It is meant to queue properly-formatted USB packets complete
 *          with CRC-16 (if required).
 * @param[in] usb     pointer to the @p GrainuumUSB object.
 * @param[in] epnum   The endpoint number to queue data for.
 * @param[in] buffer  The data to queue.
 * @param[in] size    The number of bytes that are queued.
 * @notapi
 */
void grainuumWriteQueue(struct GrainuumUSB *usb, int epnum,
                        const void *buffer, int size);

/**
 * @brief   Simulates plugging the device into USB.
 * @note    All USB Connect hooks will be called.
 *          The default USB state is "disconnected",
 *          so @p grainuumConnect() must be called
 *          to start communications.
 * @param[in] usb     pointer to the @p GrainuumUSB object.
 * @api
 */
void grainuumConnect(struct GrainuumUSB *usb);

/**
 * @brief   Simulates unplugging the device from USB.
 * @note    All USB Disconnect hooks will be called.
 * @param[in] usb     pointer to the @p GrainuumUSB object.
 * @api
 */
void grainuumDisconnect(struct GrainuumUSB *usb);

/**
 * @brief   Reads one packet from the wire.
 * @note    This must be called from an interrupt context with
 *          interrupts disabled.
 * @param[in] usb       Pointer to the @p GrainuumUSB object.
 * @param[out] samples  Buffer where the samples will be stored.
 * @return              The number of bytes read, or negative on error
 * @retval -1           Timeout while reading.
 * @retval -2           Read too many bits.
 * @retval -3           Unable to find sync end.
 * @retval -4           Probably a keepalive packet.
 * @notapi
 */
int usbPhyReadI(const struct GrainuumUSB *usb, uint8_t samples[11]);

/**
 * @brief   Writes one packet from the wire.
 * @note    This must be called from an interrupt context with
 *          interrupts disabled.
 * @param[in] usb       Pointer to the @p GrainuumUSB object.
 * @param[in] samples   Buffer where the samples will be stored.
 * @param[in] size      Number of bytes to write.
 * @notapi
 */
void usbPhyWriteI(const struct GrainuumUSB *usb, const void *buffer, uint32_t size);

#ifdef __cplusplus
};
#endif

#endif /* _GRAINUUM_H */
