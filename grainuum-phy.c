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

__attribute__((weak))
void grainuumConnectPre(struct GrainuumUSB *usb)
{
  (void)usb;
}
__attribute__((weak))
void grainuumConnectPost(struct GrainuumUSB *usb)
{
  (void)usb;
}

__attribute__((weak))
void grainuumDisconnectPre(struct GrainuumUSB *usb)
{
  (void)usb;
}
__attribute__((weak))
void grainuumDisconnectPost(struct GrainuumUSB *usb)
{
  (void)usb;
}

__attribute__((weak))
void grainuumReceivePacket(struct GrainuumUSB *usb)
{
  (void)usb;
}

__attribute__((weak))
void grainuumInitPre(struct GrainuumUSB *usb)
{
  (void)usb;
}

__attribute__((weak))
void grainuumInitPost(struct GrainuumUSB *usb)
{
  (void)usb;
}

/* --- */

__attribute__((section(".ramtext")))
void grainuum_receive_packet(struct GrainuumUSB *usb) {
  grainuumReceivePacket(usb);
}

__attribute__((section(".ramtext")))
void grainuumCaptureI(struct GrainuumUSB *usb, uint8_t *samples)
{
  int ret;
  const uint8_t nak_pkt[] = {USB_PID_NAK};
  const uint8_t ack_pkt[] = {USB_PID_ACK};

  ret = usbPhyReadI(usb, samples);
  if (ret <= 0) {
    if (ret != -1)
      usbPhyWriteI(usb, nak_pkt, sizeof(nak_pkt));
    return;
  }

  /* Save the byte counter for later inspection */
  samples[11] = ret;

  switch (samples[0]) {
  case USB_PID_IN:
    /* Make sure we have queued data, and that it's for this particular EP */
    if ((!usb->queued_size)
    || (((((const uint16_t *)(samples+1))[0] >> 7) & 0xf) != usb->queued_epnum))
    {
      usbPhyWriteI(usb, nak_pkt, sizeof(nak_pkt));
      break;
    }

    usbPhyWriteI(usb, usb->queued_data, usb->queued_size);
    break;

  case USB_PID_SETUP:
    grainuum_receive_packet(usb);
    break;

  case USB_PID_OUT:
    grainuum_receive_packet(usb);
    break;

  case USB_PID_ACK:
    /* Allow the next byte to be sent */
    usb->queued_size = 0;
    grainuum_receive_packet(usb);
    break;

  case USB_PID_DATA0:
  case USB_PID_DATA1:
    usbPhyWriteI(usb, ack_pkt, sizeof(ack_pkt));
    grainuum_receive_packet(usb);
    break;

  default:
    usbPhyWriteI(usb, nak_pkt, sizeof(nak_pkt));
    break;
  }

  return;
}

int grainuumInitialized(struct GrainuumUSB *usb)
{
  if (!usb)
    return 0;

  return usb->initialized;
}

void grainuumWriteQueue(struct GrainuumUSB *usb, int epnum,
                        const void *buffer, int size)
{
  usb->queued_data = buffer;
  usb->queued_epnum = epnum;
  usb->queued_size = size;
}

void grainuumInit(struct GrainuumUSB *usb,
                  struct GrainuumConfig *cfg) {

  if (usb->initialized)
    return;

  grainuumInitPre(usb);

  usb->cfg = cfg;
  usb->state.usb = usb;
  cfg->usb = usb;

  usb->initialized = 1;

  grainuumInitPost(usb);
}

void grainuumDisconnect(struct GrainuumUSB *usb) {

  grainuumDisconnectPre(usb);

  /* Set both lines to 0 (clear both D+ and D-) to simulate unplug. */
  grainuumWritel(usb->usbdpMask, usb->usbdpCAddr);
  grainuumWritel(usb->usbdnMask, usb->usbdnCAddr);

  /* Set both lines to output */
  grainuumWritel(grainuumReadl(usb->usbdpDAddr) | usb->usbdpMask, usb->usbdpDAddr);
  grainuumWritel(grainuumReadl(usb->usbdnDAddr) | usb->usbdnMask, usb->usbdnDAddr);

  grainuumDisconnectPost(usb);
}

void grainuumConnect(struct GrainuumUSB *usb) {

  grainuumConnectPre(usb);

  /* Set both lines to input */
  grainuumWritel(grainuumReadl(usb->usbdpDAddr) & ~usb->usbdpMask, usb->usbdpDAddr);
  grainuumWritel(grainuumReadl(usb->usbdnDAddr) & ~usb->usbdnMask, usb->usbdnDAddr);

  grainuumConnectPost(usb);
}
