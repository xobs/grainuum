Grainuum USB
============

A software implementation of a USB stack for small CPUs.

Grainuum is designed to run on Cortex-M0+ CPUs running at 48 MHz with
single-cycle IO.  In practice, this means it runs well on a wide variety
of Kinetis chips.


Usage
=======

To start with, create a GrainuumUSB object that defines your device's pin layout.
Specify the offsets for setting and clearing pins, sampling pins, changing the
pin direction, and the offsets of the two pins in the various banks.

The structure is defined as such:

    static struct USBPHY {
      // USB D- line descriptor 
      uint32_t dpIAddr; // GPIO "sample-whole-bank" address
      uint32_t dpSAddr; // GPIO "set-pin-level" address
      uint32_t dpCAddr; // GPIO "clear-pin-level" address
      uint32_t dpDAddr; // GPIO "pin-direction" address, where 1 = output
      uint32_t dpShift; // Shift of GPIO pin in S/C/D/I addresses
      
      // USB D+ line descriptor, as above
      uint32_t dnIAddr;
      uint32_t dnSAddr;
      uint32_t dnCAddr;
      uint32_t dnDAddr;
      uint32_t dnShift;
      
      // USB masks
      uint32_t dpMask;  // Mask of GPIO pin in S/C/D/I addresses
      uint32_t dnMask;
      ...
    };


For example, if D+ was on pin PTA4 and D- was on PTB0, you might specify the
following layout:

    static struct GrainuumUSB myUSB = {
      /* PTB0 */
      .usbdnIAddr = 0xf8000050, /* FGPIOB_PDIR */
      .usbdnSAddr = 0xf8000044, /* FGPIOB_PSOR */
      .usbdnCAddr = 0xf8000048, /* FGPIOB_PCOR */
      .usbdnDAddr = 0xf8000054, /* FGPIOB_PDDR */
      .usbdnMask  = (1 << 0),
      .usbdnShift = 0,
      
      /* PTA4 */
      .usbdpIAddr = 0xf8000010, /* FGPIOA_PDIR */
      .usbdpSAddr = 0xf8000004, /* FGPIOA_PSOR */
      .usbdpCAddr = 0xf8000008, /* FGPIOA_PCOR */
      .usbdpDAddr = 0xf8000014, /* FGPIOA_PDDR */
      .usbdpMask  = (1 << 4),
      .usbdpShift = 4,
    };

You should also set up a GrainuumConfig device to handle USB communication:

    struct GrainuumConfig {
        /* Called by GrainuumUSB to send descriptors to the host */
      get_usb_descriptor_t      getDescriptor;

        /* Called by GrainuumUSB when the host sets the configuration number */
      usb_set_config_num_t      setConfigNum;

        /* Called by GrainuumUSB to get space to store incoming data */
      usb_get_buffer_t          getReceiveBuffer;

        /* Called by GrainuumUSB when data has been received from the host */
      usb_data_in_t             receiveData;
      
        /* Called by GrainuumUSB after sendData() has queued data, but before it is sent */
      usb_data_out_start_t      sendDataStarted;

        /* Called by GrainuumUSB after sendData() has sent the data to the host */
      usb_data_out_finish_t     sendDataFinished;
    } __attribute__((packed, aligned(4)));

The most important function to fill in is getDescriptor(), which will
allow the USB system to respond to requests from the host.  Most other
entries are optional.

Register these two objects with GrainuumUSB:

  void grainuumInit(struct GrainuumUSB *usb, struct GrainuumConfig *cfg);

This will initialize the PHY and put it in "Disconnected" mode.  To connect, call grainuumConnect();

  void grainuumConnect(struct GrainuumUSB *usb);

Now you can hook your interrupt handler.  When an ISR hits, call grainuumCaptureI()
with a buffer big enough to hold one USB packet:

  void grainuumCaptureI(struct GrainuumUSB *usb, uint8_t packet[12]);

Then, sometime later once the interrupt is finished, pass the same buffer to grainuumProcess():

  void grainuumProcess(struct GrainuumUSB *usb,
                       const uint8_t packet[12]);

*The packet that is passed to grainuumProcess() and grainuumCaptureI() MUST be aligned
such that packet[1] is word-aligned.  One way to do this might be to define packet[16]
as being aligned, and pass &packet[3] to these functions.  Or you can use Granuum Buffers,
which are described below.*

To send data to the host, use grainuumSendData():

  int grainuumSendData(struct GrainuumUSB *usb, int epnum, const void *data, int size);


Grainuum Buffers
----------------

The USB PHY uses a ring buffer to log all incoming data as it enters
the device.  This data has special alignment requirements.  You can use
Grainuum Buffers to manage this data.

Grainuum Buffers are a set of macros that wrap all of the alignment
magic.

Declare a Grainuum Buffer using the GRAINUUM_BUFFER macro, specifying the
number of complete packets to buffer.  To declare a buffer named
*usb_buffer* with four elements, write:

    GRAINUUM_BUFFER(usb_buffer, 4);

In your program code, you must initialize the buffer before you use it:

    GRAINUUM_BUFFER_INIT(usb_buffer);

To check if the buffer is empty, use is_empty:

    if (!GRAINUUM_BUFFER_IS_EMPTY(usb_buffer)) {
        ... work on the buffer ...
    }

You'll generally want to get a pointer to the top of the buffer,
and advance it only if the data is filled.  To get a pointer
to the top of the buffer (and pass it to grainuumCaptureI()), type:

    grainuumCaptureI(usb, GRAINUUM_BUFFER_ENTRY(usb_buffer));

If the buffer is filled, advance the buffer with advance():

    GRAINUUM_BUFFER_ADVANCE(usb_buffer);

To get the oldest item in the queue, use top():

    uint8_t *usb_pkt = GRAINUUM_BUFFER_TOP(usb_buffer);

When you're done with the packet and want to advance tne end
of the buffer (i.e. remove the oldest item), use remove():

    GRAINUUM_BUFFER_REMOVE(usb_buffer);

Callbacks and Hooks
-------------------

Most of the normal configuration is done through the GrainuumConfig structure.