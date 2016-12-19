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
 #include "grainuum.h"

#ifndef NULL
#define NULL ((void *)0)
#endif

void *memcpy(void *dest, const void *src, unsigned int n);

enum usb_mac_packet_type {
  packet_type_none,
  packet_type_setup,
  packet_type_setup_in,
  packet_type_setup_out,
  packet_type_in,
  packet_type_out,
};

__attribute__((weak))
void grainuumSendWait(struct GrainuumUSB *usb, int epnum,
                      const void *data, int size)
{
  (void)usb;
  (void)epnum;
  (void)data;
  (void)size;
}

static uint16_t crc16_add(uint16_t crc, uint8_t c, uint16_t poly)
{
  uint8_t  i;

  for (i = 0; i < 8; i++) {
    if ((crc ^ c) & 1)
      crc = (crc >> 1) ^ poly;
    else
      crc >>= 1;
    c >>= 1;
  }
  return crc;
}

static uint16_t crc16(const uint8_t *data, uint32_t size,
                      uint16_t init, uint32_t poly)
{

  while (size--)
    init = crc16_add(init, *data++, poly);

  return init;
}

static void grainuum_mac_clear_tx(struct GrainuumMAC *mac, int result)
{
  struct GrainuumUSB *usb = mac->usb;

  /* If a thread is blocking, wake it up with a failure */
  if (usb->cfg->sendDataFinished)
    usb->cfg->sendDataFinished(usb, result);
  mac->data_out_left = 0;
  mac->data_out_max = 0;
  mac->data_out = NULL;
  mac->packet_queued = 0;
}

static void grainuum_mac_process_tx(struct GrainuumMAC *mac)
{

  uint16_t crc;
  struct GrainuumUSB *usb = mac->usb;

  /* Don't allow us to re-prepare data */
  if (mac->packet_queued) {
    return;
  }
  mac->packet_queued = 1;

  /* If there's no data to send, then don't send any */
  if (!mac->data_out) {
    mac->packet_queued = 0;
    return;
  }

  /* If we've sent all of our data, then there's nothing else to send */
  if ((mac->data_out_left < 0) || (mac->data_out_max < 0)) {
    grainuum_mac_clear_tx(mac, 0);
    return;
  }

  /* Pick the correct PID, DATA0 or DATA1 */
  if (mac->data_buffer & (1 << mac->tok_epnum))
    mac->packet.pid = USB_PID_DATA1;
  else
    mac->packet.pid = USB_PID_DATA0;

  /* If there's no data, prepare a special NULL packet */
  if ((mac->data_out_left == 0) || (mac->data_out_max == 0)) {

    /* The special-null thing only happens for EP0 */
    if (mac->data_out_epnum != 0) {
      grainuum_mac_clear_tx(mac, 0);
      return;
    }
    mac->packet.data[0] = 0;  /* CRC16 for empty packets is 0 */
    mac->packet.data[1] = 0;
    mac->packet.size = 2;
    grainuumWriteQueue(usb, mac->data_out_epnum,
                         &mac->packet, mac->packet.size + 1);
    return;
  }

  /* Keep the packet size to 8 bytes max */
  if (mac->data_out_left > 8)
    mac->packet.size = 8;
  else
    mac->packet.size = mac->data_out_left;

  /* Limit the amount of data transferred to data_out_max */
  if (mac->packet.size > mac->data_out_max)
    mac->packet.size = mac->data_out_max;

  /* Copy over data bytes */
  memcpy(mac->packet.data, mac->data_out, mac->packet.size);

  /* Calculate and copy the crc16 */
  crc = ~crc16(mac->packet.data, mac->packet.size, 0xffff, 0xa001);
  mac->packet.data[mac->packet.size++] = crc;
  mac->packet.data[mac->packet.size++] = crc >> 8;

  /* Prepare the packet, including the PID at the end */
  grainuumWriteQueue(usb, mac->data_out_epnum,
                       &mac->packet, mac->packet.size + 1);
}

/* Called when a packet is ACKed.
 * Updates the outgoing packet buffer.
 */
static void usbMacTransferSuccess(struct GrainuumMAC *mac)
{

  /* Reduce the amount of data left.
   * If the packet is divisible by 8, this will cause one more call
   * to this function with mac->data_out_left == 0.  This will send
   * a NULL packet, which indicates end-of-transfer.
   */
  mac->data_out_left -= 8;
  mac->data_out_max -= 8;
  mac->data_out += 8;

  if ((mac->data_out_left < 0) || (mac->data_out_max < 0)) {
    grainuum_mac_clear_tx(mac, 0);

    /* End of a MAC setup packet */
    if (mac->packet_type == packet_type_setup_out)
      mac->packet_type = packet_type_none;
    if (mac->packet_type == packet_type_setup_in)
      mac->packet_type = packet_type_none;
    if (mac->packet_type == packet_type_out)
      mac->packet_type = packet_type_none;
  }

  mac->packet_queued = 0;
}

/* Send data down the wire, interrupting any existing
 * data that may be queued.
 */
static int grainuum_mac_send_data(struct GrainuumMAC *mac,
                             int epnum,
                             const void *data,
                             int size,
                             int max)
{

  /* De-queue any data that may already be queued. */
  grainuum_mac_clear_tx(mac, 1);

  mac->data_out_epnum = epnum;
  mac->data_out_left = size;
  mac->data_out_max = max;
  mac->data_out = data;

  return 0;
}

int grainuumSendData(struct GrainuumUSB *usb, int epnum,
                     const void *data, int size)
{

  struct GrainuumMAC *mac = &usb->mac;
  int ret;

  if (mac->data_out || !mac->address || mac->packet_queued) {
    return -11; /* EAGAIN */
  }

  ret = grainuum_mac_send_data(mac, epnum, data, size, size);
  if (ret)
    return ret;

  grainuum_mac_process_tx(mac);

  if (usb->cfg->sendDataStarted)
    usb->cfg->sendDataStarted(usb, epnum, data, size);

  return 0;
}

static int grainuum_mac_process_setup(struct GrainuumMAC *mac, const uint8_t packet[10])
{

  const struct usb_setup_packet *setup;
  const void *response = (void *)-1;
  uint32_t response_len = 0;
  struct GrainuumUSB *usb = mac->usb;
  struct GrainuumConfig *cfg = usb->cfg;

  setup = (const struct usb_setup_packet *)packet;

  if ((setup->bmRequestType == 0x00) && (setup->bRequest == SET_ADDRESS)) {
    mac->address = setup->wValue;
  }
  else if ((setup->bmRequestType == 0x00) && (setup->bRequest == SET_CONFIGURATION)) {
    if (cfg->setConfigNum)
      cfg->setConfigNum(usb, setup->wValue);
  }
  else {
    response_len = cfg->getDescriptor(usb, setup, &response);
  }
  grainuum_mac_send_data(mac, mac->tok_epnum, response, response_len, setup->wLength);

  return 0;
}

static void grainuum_mac_parse_data(struct GrainuumMAC *mac,
                               const uint8_t packet[10],
                               uint32_t size)
{
  (void)size;
  struct GrainuumUSB *usb = mac->usb;

  switch (mac->packet_type) {

  case packet_type_setup:
    grainuum_mac_process_setup(mac, packet);
    grainuum_mac_process_tx(mac);
    mac->packet_type = packet_type_none;
    break;

  case packet_type_out:
    // XXX HACK: An OUT packet gets generated (on Windows at least) when
    // terminating a SETUP sequence.  This seems odd.
    if (mac->tok_epnum == 0)
      break;
    // Copy over the packet, minus the CRC16
    memcpy(mac->tok_buf + mac->tok_pos, packet, size - 2);
    mac->tok_pos += (size - 2);
    if (!usb->cfg->receiveData(usb, mac->tok_epnum, size - 2, packet))
      mac->packet_type = packet_type_none;
    break;

  case packet_type_in:
  case packet_type_none:
  default:
    break;
  }
}

static inline void grainuum_mac_parse_token(struct GrainuumMAC *mac,
                                       const uint8_t packet[2])
{

  mac->tok_epnum = (((const uint16_t *)packet)[0] >> 7) & 0xf;
  /*mac->tok_addr  = (((const uint16_t *)packet)[0] >> 11) & 0x1f; // Field unused in this code*/
}

void grainuumProcess(struct GrainuumUSB *usb,
                     const uint8_t packet[12])
{

  uint32_t size = packet[11];
  struct GrainuumMAC *mac = &usb->mac;
  switch(packet[0]) {
  case USB_PID_SETUP:
    mac->packet_type = packet_type_setup;
    grainuum_mac_clear_tx(mac, 1);
    grainuum_mac_parse_token(mac, packet + 1);
    break;

  case USB_PID_DATA0:
    mac->data_buffer |= (1 << mac->tok_epnum);
    grainuum_mac_parse_data(mac, packet + 1, size - 1);
    break;

  case USB_PID_DATA1:
    mac->data_buffer &= ~(1 << mac->tok_epnum);
    grainuum_mac_parse_data(mac, packet + 1, size - 1);
    break;

  case USB_PID_OUT:
    grainuum_mac_parse_token(mac, packet + 1);
    mac->packet_type = packet_type_out;
    mac->tok_pos = 0;
    mac->tok_buf = usb->cfg->getReceiveBuffer(usb, mac->tok_epnum, NULL);
  break;

  case USB_PID_ACK:
    mac->data_buffer ^= (1 << mac->tok_epnum);
    usbMacTransferSuccess(mac);
    if (mac->data_out) {
      grainuum_mac_process_tx(mac);
    }
    else {
      grainuum_mac_clear_tx(mac, 0);
    }
    break;

  default:
    break;
  }
}