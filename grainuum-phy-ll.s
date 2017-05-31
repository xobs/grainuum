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

#ifndef GRAINUUM_SECTION
#define GRAINUUM_SECTION .ramtext
#endif 
.section GRAINUUM_SECTION /* Can also run out of .section .text */

.cpu    cortex-m0plus
.fpu    softvfp

#if 0
  /***************************************************************************
   * USB PHY low-level code
   *
   * Exports the following functions:
   *
   *    int usbPhyReadI(struct GrainuumUSB *usb, uint8_t buffer[11])
   *    void usbPhyWriteI(struct GrainuumUSB *usb, const uint8_t buffer[11], uint32_t count)
   *
   * Interrupts are disabled during this code, since it is time-critical.
   * Note that as a Kinetis "feature", jumps of more than 48 bytes can
   * cause random amounts of jitter.  Make sure you don't do that.
   *
   * Both functions take the following struct as their first parameter:
   *
   *  static struct USBPHY {
   *    // USB D- line descriptor 
   *    uint32_t dpIAddr; // GPIO "sample-whole-bank" address
   *    uint32_t dpSAddr; // GPIO "set-pin-level" address
   *    uint32_t dpCAddr; // GPIO "clear-pin-level" address
   *    uint32_t dpDAddr; // GPIO "pin-direction" address, where 1 = output
   *    uint32_t dpShift; // Shift of GPIO pin in S/C/D/I addresses
   *
   *    // USB D+ line descriptor, as above
   *    uint32_t dnIAddr;
   *    uint32_t dnSAddr;
   *    uint32_t dnCAddr;
   *    uint32_t dnDAddr;
   *    uint32_t dnShift;
   *
   *    // USB masks
   *    uint32_t dpMask;  // Mask of GPIO pin in S/C/D/I addresses
   *    uint32_t dnMask;
   *
   *    // Optional extra data follows
   * };
   */
#endif

/* [r|w]usbphy offsets */
.equ dpIAddr,0x08
.equ dpSAddr,0x0c
.equ dpCAddr,0x10
.equ dpDAddr,0x14
.equ dpShift,0x18

.equ dnIAddr,0x1c
.equ dnSAddr,0x20
.equ dnCAddr,0x24
.equ dnDAddr,0x28
.equ dnShift,0x2c

.equ dpMask,0x30
.equ dnMask,0x34

  /*
   *
   * Each USB bit takes about 666 nS, which at a 48 MHz clock rate gives us
   * 32 cycles to read each bit.  This code runs on a Cortex M0+, where each
   * instruction is one cycle, except taken-branches are three cycles.
   *
   * A USB frame begins with the pattern KJKJ...KK.  A USB frame ends with
   * a double-SE0 state.  USB low-speed packets have an 8-bit sync period, and
   * a maximum of 11 bytes.  Thus, a USB low-speed packet looks like this:
   *
   *     KJKJKJKK | data | 00
   *
   * Our usbReadData() code will start by positioning ourselves in the
   * middle of a pulse.  We do that by samping the line, then waiting for
   * it to change, then waiting some number of cycles.
   *
   * Once we're positioned, we then start looking for the KK end-of-sync
   * indicator.  An interrupt takes 16 clock cycles, and it probably took
   * at least 32 clock cycles to get here, meaning we've already lost the
   * first KJ.  This is fine, as we just need to look for "KK" to indicate
   * the end of the sync period.
   *
   * Since a USB low-speed packet is at most 11 bytes, we can store this in
   * three 32-bit registers.  We chain three registers together in a shift-
   * chain by self-adding-with-carry on each of the three registers in
   * sequence to move the top bit from one into the bottom bit of the next.
   *
   * Add a 1 to the low register if the state is the same as the previous
   * state, and add a 0 to the low register if the state has changed.
   *
   * As a special case, when we get six consecutive bits in a row (i.e. six
   * ones or six zeroes), the host will "stuff" one bit and flip the state,
   * meaning we should ignore that 1-bit.  If this is the case, the shift
   * should not be processed.
   *
   * Continue reading bits in until we get a double-SE0.  Actually, any
   * SE0 should be considered end-of-frame.
   *
   */

rretval  .req r0  /* Return value (at the end) */

rusbphy  .req r0  /* Pointer to the USBPHY struct (stored on stack) */
rscratch .req r0  /* General-purpose scratch register (after premable) */
routptr  .req r1  /* Outgoing sample buffer */

rone     .req r2  /* The value 1 */
rreg     .req r3  /* Register to sample pin */
rval     .req r4  /* Currently-sampled value */
rmash    .req r5  /* Mask/shift to isolate required pin */

rsample  .req r6  /* Most recent byte */
rcounter .req r7  /* Number of bytes sampled */

rlastval .req r8  /* What value was the last pin? */
runstuff .req r9  /* Log of the last six bits, for unstuffing */

rdpshift .req r10 /* Shift amount for D+ */
rdpiaddr .req r11 /* Pointer to D+ sample address */
rdniaddr .req r12 /* Pointer to D- sample address */

/* Read Stack:
 * 0: dnMask
 * 4: rusbphy (currently unused)
 */
.equ rsDnMask,0x00
.equ rsUsbPhy,0x04

.func usbPhyReadI
.global usbPhyReadI

usb_phy_read__se0:
  add sp, #12
  pop {r2-r6}
  mov r8, r2
  mov r9, r3
  mov r10, r4
  mov r11, r5
  mov r12, r6

  mov rretval, #0
  sub rretval, #4                   // Return -4
  pop {r2-r7,pc}

/*int */usbPhyReadI/*(struct GrainuumUSB *usb, uint8_t samples[11])*/:
  push {r2-r7,lr}
  mov r2, r8
  mov r3, r9
  mov r4, r10
  mov r5, r11
  mov r6, r12
  push {r2-r6}                      // Save high registers
  sub sp, #12

  ldr rreg, [rusbphy, #dnIAddr]     // Grab the address for the data input reg.
  ldr rmash, [rusbphy, #dnMask]     // Grab the mask for the bit.

  str rusbphy, [sp, #rsUsbPhy]      // Save the USBPHY on the stack.
  str rmash, [sp, #rsDnMask]        // Save it on the stack for later.

  ldr rsample, [rusbphy, #dpIAddr]  // Also grab D+ address
  ldr rcounter, [rusbphy, #dpMask]  // And D+ mask

  /* Wait for the line to flip */
  ldr rval, [rreg]                  // Sample D-, to watch for it flipping
  ldr rone, [rsample]               // Sample D+ at the same time
  and rval, rmash                   // Mask off the interesting bit
  mov rlastval, rval                // Save the bit for use in looking for sync

  /* Check to see if it's SE0, in which case this is a keepalive pkt */
  and rone, rcounter                // Mask off the interesting bit
  add rone, rval                    // Combine D+ and D-.
  beq usb_phy_read__se0             // Exit if SE0 condition (both are 0).

  /* Clear out the register shift-chain */
  mov rsample, #0                   // Reset the sample byte value.
  mov rcounter, #0                  // Reset the "bits" counter.

  mov rone, #1                      // Actually load the value '1' into the reg.

  mov rval, #0b11
  mov runstuff, rval                // Load 0b11 into unstuff reg, as the header
                                    // ends with the pattern KK, which starts
                                    // a run of two.

  // The loop is 4 cycles on a failure.  One
  // pulse is 32 cycles.  Therefore, loop up
  // to 8 times before giving up.
.rept 8
  ldr rval, [rreg]                  // Sample USBDP
  and rval, rmash                   // Mask off the interesting bit
  cmp rval, rlastval                // Wait for it to change
  bne usb_phy_read__sync_wait       // When it changes, go wait for sync pulse
.endr
  b usb_phy_read__timeout           // It never changed, so return "timeout".

usb_phy_read__sync_wait:
  // Move us away from the start of the pulse, to avoid transition errors.
  bl usb_phy__wait_5_cycles

  // Wait for the end-of-header sync pulse, which is when the value
  // repeats itself.  This is the "KK" in the KJKJKJKK training sequence.
.rept 6
  ldr rval, [rreg]                    // Sample USBDP
  and rval, rmash                     // Mask off the interesting bit
  cmp rlastval, rval
  beq usb_phy_read__start_reading_usb
  mov rlastval, rval
  bl usb_phy__wait_27_cycles
.endr
  b usb_phy_read__sync_timeout

  /* We're synced to the middle of a pulse, and the clock sync / start-of-
   * -frame has been found.  Real packet data follows.
   */
usb_phy_read__start_reading_usb:
  // ?

  /* Adjust rlastval so that it's in the correct position -- we skip doing
     this above since we're only interested in the value changing, not in
     what the value is.  However, we're now interested in what the value
     is, so we now deal with shifts instead of masks, and always mask by
     #1.
   */
  mov rval, rlastval
  ldr rmash, [rusbphy, #dpShift]
  ror rval, rmash
  and rval, rone
  mov rlastval, rval
  // 6

  /* We have plenty of extra cycles here, because the first bit is a K,
   * and we simply need to wait for it to finish.
   */
  ldr rreg, [rusbphy, #dpIAddr]       // Cache the address of the D+ input bank
  mov rdpiaddr, rreg                  // to save one cycle.
  ldr rreg, [rusbphy, #dnIAddr]       // Cache the address of the D- input bank
  mov rdniaddr, rreg                  // to save another cycle.
  ldr rreg, [rusbphy, #dpShift]       // Cache the D+ shift, too.
  mov rdpshift, rreg
  // 9

  nop
  nop
  nop
  nop
  nop
  nop
  nop

usb_phy_read__get_usb_bit:
  mov rval, rdpiaddr                  // Get the address of the D+ input bank.
  mov rreg, rdniaddr                  // Get the address of the D- input bank.
  ldr rval, [rval]                    // Actually sample D+
  ldr rreg, [rreg]                    // Also sample D-

  mov rmash, rdpshift                 // Get the shift of the D+ bit
  ror rval, rmash                     // Rotate the value down to bit 1.
  and rval, rone                      // Mask off everything else.
  // 7

  /* xor this bit with the last bit and invert it, in order to get
   * the logical value */
  mov rmash, rlastval                 // Load up the previous bit
  mov rlastval, rval                  // Save the current bit for the next loop.
  eor rmash, rval                     // Check to see if the state has changed.
  mvn rmash, rmash                    // Invert, as 1 ^ 1 or 0 ^ 0 should be 1.
  and rmash, rone                     // Mask off everything but the last bit.

  orr rsample, rmash                  // Append the value to the sample.
  ror rsample, rone                   // Move it to the top of the buffer.

  add runstuff, runstuff              // Shift the "unstuff" value up by 1.
  add runstuff, rmash                 // "or" in the one-bit rval to the bottom.

  // 9

  // [16 total]

  /* If we've completed a byte, the rcounter will mask to 0.
   * If the byte is done, advance.
   * If the byte continues, check for SE0.
   */
  add rcounter, rone                  // Increment the total-bit counter.
  mov rscratch, #7                    // Prepare to mask by 0x7
  tst rcounter, rscratch              // Perform the mask
  beq usb_phy_read__advance_byte      // If the result is 0, advance the byte.
  // 4 (or 5, if branch taken)

  // The result is NOT 0, so see if it's an SE0.
usb_phy_read__check_se0:
  // Check for SE0
  ldr rscratch, [sp, #rsDnMask]
  and rreg, rscratch
  add rreg, rval                      // An end-of-frame is indicated by two
                                      // frames of SE0.  If this is the case,
                                      // then the result of adding these
                                      // together will result in 0.
  bne usb_phy_read__check_unstuff
  b usb_phy_read__exit                // Exit if so.
  // 6 (if not SE0)

usb_phy_read__advance_byte:
  lsr rsample, #24                    // Rotate the sample down to the byte list
  strb rsample, [routptr]             // Save the value to the out buffer.
  add routptr, rone                   // Advance the out pointer.
  mov rsample, #0                     // Reset sample accumulator.
  // 5

  // Figure out if we need to unstuff, or if we can just continue to the
  // next bit.
  // Six consecutive USB states will be followed by a dummy
  // state flip.  Ignore this.
usb_phy_read__check_unstuff:
  mov rreg, runstuff
  mov rval, #0b111111                 // Unstuff mask
  and rreg, rval
  cmp rreg, rval

  /* Loop again */
  bne usb_phy_read__get_usb_bit
  // 2 (if branch taken, 1 if unstuffing needs to happen)

usb_phy_read__unstuff:
  nop                                 // We get here when the current bit has
                                      // one more clock cycle left.  Add a nop
                                      // just to make the cycle-counting easier.

  /* --- New "loop" starts here --- */
  // NOTE that we don't increment "rcounter" here.
  mov runstuff, rone                  // We're skipping over one bit, which
                                      // results in a new run of one.
  add runstuff, rone
  // 2

  /* Invert the last value, since a false 0 was added to the stream */
  mov rreg, rlastval                  // Read the current last val into a lo reg.
  mvn rreg, rreg                      // Negate the value.
  and rreg, rreg, rone                // Mask it with 0b1
  mov rlastval, rreg                  // Save it back into a lo reg.
  // 4

  cmp rcounter, #88                   // Sanity check: see if we've read more
                                      // than 88 bytes (11 bits), because we've
                                      // got cycles to spare here.
  bgt usb_phy_read__exit              // Exit if so
  // 2

  bl usb_phy__wait_22_cycles

  b usb_phy_read__get_usb_bit
  // 2

usb_phy_read__exit:
  // a minimum of 10 cycles have elapsed since we got here

  /* Count the number of bytes read (rcounter / 8) */
  mov rretval, rcounter
  asr rretval, #3                 // Return number of bytes (not bits) read.
  cmp rretval, #11                // Error out if we read more than 11 bytes.
  bgt usb_phy_read__overflow_exit
  // 4

usb_phy_read__return:
  add sp, #12                     // Restore stack.
  pop {r2-r6}
  mov r8, r2
  mov r9, r3
  mov r10, r4
  mov r11, r5
  mov r12, r6
  // 13

  pop {r2-r7,pc}
  // 8

  /* Read too many bits, return -2 */
usb_phy_read__overflow_exit:
  mov rretval, #0
  sub rretval, #2
  b usb_phy_read__return

  /* Unable to find pulse end, return -3 */
usb_phy_read__sync_timeout:
  mov rretval, #0
  sub rretval, #3
  b usb_phy_read__return

  /* Timeout while reading, return -1 */
usb_phy_read__timeout:
  mov rretval, #0
  sub rretval, #1
  b usb_phy_read__return

.endfunc
.type usbPhyReadI, %function
.size usbPhyReadI, .-usbPhyReadI



usb_phy__wait_32_cycles: nop
usb_phy__wait_31_cycles: nop
usb_phy__wait_30_cycles: nop
usb_phy__wait_29_cycles: nop
usb_phy__wait_28_cycles: nop
usb_phy__wait_27_cycles: nop
usb_phy__wait_26_cycles: nop
usb_phy__wait_25_cycles: nop
usb_phy__wait_24_cycles: nop
usb_phy__wait_23_cycles: nop
usb_phy__wait_22_cycles: nop
usb_phy__wait_21_cycles: nop
usb_phy__wait_20_cycles: nop
usb_phy__wait_19_cycles: nop
usb_phy__wait_18_cycles: nop
usb_phy__wait_17_cycles: nop
usb_phy__wait_16_cycles: nop
usb_phy__wait_15_cycles: nop
usb_phy__wait_14_cycles: nop
usb_phy__wait_13_cycles: nop
usb_phy__wait_12_cycles: nop
usb_phy__wait_11_cycles: nop
usb_phy__wait_10_cycles: nop
usb_phy__wait_9_cycles:  nop
usb_phy__wait_8_cycles:  nop
usb_phy__wait_7_cycles:  nop
usb_phy__wait_6_cycles:  nop
usb_phy__wait_5_cycles:  mov pc, lr





/*
 * usbPhyWriteI
 * Register (arguments):
 *   r0: USBPHY
 *   r1: pointer to buffer data
 *   r2: number of bytes to write
 */
wusbphy   .req r0   /* Pointer to USBPHY value */

wlastsym  .req r1   /* The last symbol (0 = j, 1 = k) */
wpkt      .req r2   /* Current byte */
wleft     .req r3   /* Number of bits left before we need to reload wpkt */

/* These are used when writing values out.  May be repurposed. */
wpaddr    .req r4   /* Write "address" for D+ line, during normal operation */
wnaddr    .req r5   /* Write "address" for D- line, during normal operation */
wtmp1     .req r4
wtmp2     .req r5

/* These must remain unchanged during the whole operation. */
wpmask    .req r6   /* Write mask for D+ line */
wnmask    .req r7   /* Write mask for D- line */

wbytes    .req r8   /* Pointer to bytes (from r2 arg) */
wend      .req r9   /* End of the wbytes array (from r1+r2 arg) */
wdpsetreg .req r10
wdpclrreg .req r11
wdnclrreg .req r12
wstuff    .req r0   /* The last six bits, used for bit stuffing (reuses wusbphy) */

/* Stack data:
   0: D- set (wDnSAddr)
   4: wusbphy
 */

/* Indexes off of internal data */
.equ wDnSAddr,0x00      /* D- Set Addr */
.equ wDnCAddr,0x04      /* D- Clear Addr */
.equ wPkt1,0x08
.equ wPkt1Num,0x0c
.equ wPkt2,0x10
.equ wPkt2Num,0x14
.equ wPkt3,0x18
.equ wPkt3Num,0x1c
.equ wFirstPkt,0x20
.equ wSpSave,0x24

.thumb
.align  2
.thumb_func
.global usbPhyWriteI
.func usbPhyWriteI
/*void */usbPhyWriteI/*(struct GrainuumUSB *usb, const uint8_t buffer[11], uint32_t count)*/:
  push {r3-r7,lr}
  mov r3, r8
  mov r4, r9
  mov r5, r10
  mov r6, r11
  push {r3-r6}                      // Save other arguments.

  /* Allocate and populate the stack */
  sub sp, #8                        // Use 8 bytes of stack

  /* Cache D+ set and clear registers early on */
  ldr wtmp1, [wusbphy, #dpSAddr]    // Registers are faster than RAM, and we
  mov wdpsetreg, wtmp1              // only have two free registers, so pre-
  ldr wtmp1, [wusbphy, #dpCAddr]    // cache the D+ addresses to save one
  mov wdpclrreg, wtmp1              // clock cycle.
  ldr wtmp1, [wusbphy, #dnCAddr]    // Cache D- clr as well.
  mov wdnclrreg, wtmp1

  /* Load D+ and D- masks, used for direction and value setting */
  ldr wpmask, [wusbphy, #dpMask]    // USB D+ mask
  ldr wnmask, [wusbphy, #dnMask]    // USB D- mask

  /* Pre-set the lines to J-state to prevent glitching */
#if 0
  mov wpaddr, wdpsetreg             // D+ set
  ldr wnaddr, [wusbphy, #dnCAddr]   // D- clr
#else
  mov wpaddr, wdpclrreg             // D+ clr
  ldr wnaddr, [wusbphy, #dnSAddr]   // D- set
#endif
  str wpmask, [wpaddr]              // Write D+ value
  str wnmask, [wnaddr]              // Write D- value

  /* Set D+ line to OUTPUT */
  ldr wtmp1, [wusbphy, #dpDAddr]    // Get the direction address
  ldr wtmp2, [wtmp1]                // Get the direction value
  orr wtmp2, wtmp2, wpmask          // Set the direciton mask
  str wtmp2, [wtmp1]                // Set the direction for D+

  /* Set D- line to OUTPUT */
  ldr wtmp1, [wusbphy, #dnDAddr]    // Get the direction address
  ldr wtmp2, [wtmp1]                // Get the direction value
  orr wtmp2, wtmp2, wnmask          // Set the direciton mask
  str wtmp2, [wtmp1]                // Set the direction for D-

  /* Set K state.  This indicates the start of the packet. */
  mov wpaddr, wdpclrreg             // D+ clr
  ldr wnaddr, [wusbphy, #dnSAddr]   // D- set
  str wpmask, [wpaddr]              // Write D+ value
  str wnmask, [wnaddr]              // Write D- value

  /* Now that the packet has started, we have 30 cycles to complete setup. */

  ldr wtmp1, [wusbphy, #dnSAddr]    // Cache D- Set addr
  str wtmp1, [sp, #0]               // Save it on the stack
  // 4

  /* Save passed-in values on the stack, before we scribble over them. */
  str wusbphy, [sp, #4]             // Save wusbphy,
  mov wbytes, r1                    // byte pointer,
  add r2, r2, r1                    // calculate the end of the array
  mov wend, r2                      // and save it too.
  // 5

  /* Load the next byte into wpkt */
  mov wtmp1, wbytes                 // Load the byte pointer
  ldrb wpkt, [wtmp1]                // Get the actual byte
  add wtmp1, #1                     // Increase the byte pointer
  mov wbytes, wtmp1                 // Save the byte pointer back into wbytes
  // 5

  mvn wpkt, wpkt                    // Invert the value to make the tests work
  mov wleft, #8                     // Start over with 8 bytes.
  // 2

usb_phy_write__get_first_packet:
  mov wlastsym, #1                  // Last symbols were "KK" from the header,
  mov wstuff, #0b111100             // so load a run of 2 into the stuff value.
  // 2

  // usb start-of-frame header //
  /*bl usb_phy_write__state_k // K state entered above already */
  bl usb_phy__wait_6_cycles
  bl usb_phy_write__state_j
  bl usb_phy_write__state_k
  bl usb_phy_write__state_j
  bl usb_phy_write__state_k
  bl usb_phy_write__state_j
  bl usb_phy_write__state_k
  bl usb_phy__wait_26_cycles        // Hold k state for one more cycle.  Take
                                    // up the slack that would normally
                                    // follow this.
  // end of header //

usb_phy_write__top:

  mov wtmp1, #0                     // Clear wthisbit, so we can add later.
  lsr wpkt, #1                      // Shift the bottom bit into the carry bit.
  adc wtmp1, wtmp1                  // Pull the new bit out from the carry bit.

  add wstuff, wstuff, wstuff        // Shift up the stuff bit, to allow for
  add wstuff, wstuff, wtmp1         // adding the new bit in.

  add wlastsym, wlastsym, wtmp1     // Add the new bit to the last symbol.
                                    // Since we're only looking at the last
                                    // bit, this becomes an XOR.
  mov wtmp1, #0b1                   // Examine the last bit, to check for a
  tst wlastsym, wtmp1               // state transition or not.
  // 8

  /* Write the desired state out (each branch is balanced) */
  bne usb_phy_write__j
usb_phy_write__k:
  mov wpaddr, wdpsetreg             // D+ set
  mov wnaddr, wdnclrreg
  b usb_phy_write__out

usb_phy_write__j:
  mov wpaddr, wdpclrreg             // D+ clr
  ldr wnaddr, [sp, #0]              // D- set

usb_phy_write__out:
  str wpmask, [wpaddr]
  str wnmask, [wnaddr]
  // 7 (either branch taken)

  // 15 cycles total

  sub wleft, wleft, #1              // See how many bits we have left to write.
  bne usb_phy_write__continue_byte  // If nonzero, write another bit.
  // 2

  /* We just finished writing a byte.  Load the next byte, or exit. */
usb_phy_write__finished_byte:
  mov wtmp1, wbytes                 // Move byte array into a lo reg
  cmp wtmp1, wend                   // See if we've reached the end.
  beq usb_phy_write__send_eof       // Exit if it's now 0.
  ldrb wpkt, [wtmp1]                // Read the next byte into wpkt.
  add wtmp1, #1                     // Advance byte array by one.
  mov wbytes, wtmp1                 // ...or store byte addr back in the hi reg.
  // 7

usb_phy_write__calculate_next_pkt:
  mvn wpkt, wpkt                    // Invert it to make the math work.
  mov wleft, #8                     // Reset "bits left" to 8.
  // 2

  nop
  // 1

  /* If we just wrote "111111", then stuff one bit */
usb_phy_write__stuff_bit_maybe:
  mov wtmp2, #0b111111              // Compare it with the wstuff value
  and wtmp2, wstuff                 // AND the two together.  If they're the
  beq usb_phy_write__stuff_bit      // same, then stuff one bit.
  // 3

usb_phy_write__done_stuffing_bit:
  b usb_phy_write__top
  // 2

  /* We're still writing this byte, so there's nothing to do. */
usb_phy_write__continue_byte:
  bl usb_phy__wait_7_cycles
  b usb_phy_write__stuff_bit_maybe
  // 2

usb_phy_write__stuff_bit:
  /* When we get here, we are already into the packet. */
// Need 18 cycles until packet is written
  bl usb_phy__wait_6_cycles
  mov wstuff, #0b111111             // Clear out the bit-stuff rcounter
  // 2

  add wlastsym, wlastsym, #1        // Invert the last symbol.

  mov wtmp1, #0b1                   // See if we need to send j or k
  tst wlastsym, wtmp1
  // 3

  /* Write the desired state out (each branch is balanced) */
  bne usb_phy_write__stuff_j
usb_phy_write_stuff_k:
  mov wpaddr, wdpsetreg             // D+ set
  mov wnaddr, wdnclrreg             // D- clr
  b usb_phy_write__stuff_out

usb_phy_write__stuff_j:
  mov wpaddr, wdpclrreg             // D+ clr
  ldr wnaddr, [sp, #0]              // D- set

usb_phy_write__stuff_out:
  str wpmask, [wpaddr]
  str wnmask, [wnaddr]
  // 7 (either branch taken)

  bl usb_phy__wait_13_cycles
  b usb_phy_write__done_stuffing_bit

usb_phy_write__eof_stuff_bit:
  bl usb_phy__wait_12_cycles
  mov wstuff, #0b111111             // Clear out the bit-stuff rcounter
  // 2

  add wlastsym, wlastsym, #1        // Invert the last symbol.

  mov wtmp1, #0b1                   // See if we need to send j or k
  tst wlastsym, wtmp1
  // 3

  /* Write the desired state out (each branch is balanced) */
  bne usb_phy_write__eof_stuff_j
usb_phy_write__eof_stuff_k:
  mov wpaddr, wdpsetreg             // D+ set
  mov wnaddr, wdnclrreg             // D- clr
  b usb_phy_write__eof_stuff_out

usb_phy_write__eof_stuff_j:
  mov wpaddr, wdpclrreg             // D+ clr
  ldr wnaddr, [sp, #0]              // D- set

usb_phy_write__eof_stuff_out:
  str wpmask, [wpaddr]
  str wnmask, [wnaddr]
  bl usb_phy__wait_27_cycles
  b usb_phy_write__send_se0
  // 7 (either branch taken)

usb_phy_write__send_eof:
  mov wtmp2, #0b111111              // Compare it with the wstuff value
  and wtmp2, wstuff                 // AND the two together.  If they're the
  beq usb_phy_write__eof_stuff_bit  // same, then stuff one bit.

  bl usb_phy__wait_16_cycles
usb_phy_write__send_se0:
  bl usb_phy_write__state_se0
  bl usb_phy_write__state_se0

  /* Set J-state, as required by the spec */
#if 1
  bl usb_phy__wait_6_cycles
  mov wpaddr, wdpsetreg             // D+ set
  mov wnaddr, wdnclrreg             // D- clr
  str wpmask, [wpaddr]
  str wnmask, [wnaddr]
#else
#warning "Not setting J state (fix this before committing)"
#endif
  /* Cheat a bit on the end-of-packet time, since the following
   * instructions take roughly 10 cycles before the lines reset.
   */
  bl usb_phy__wait_28_cycles

  // --- Done Transmitting --- //

  // Restore sp, since we're done with it
  ldr wusbphy, [sp, #4]             // Restore wusbphy
  add sp, #8                        // Restore stack pointer.

  /* Now, set both lines back to INPUT */

  /* Set D+ line to INPUT */
  ldr wtmp1, [wusbphy, #dpDAddr]    // Get the direction address
  ldr wtmp2, [wtmp1]                // Get the direction value
  bic wtmp2, wtmp2, wpmask          // Clear the direciton mask
  str wtmp2, [wtmp1]                // Set the direction for D+

  /* Set D- line to INPUT */
  ldr wtmp1, [wusbphy, #dnDAddr]    // Get the direction address
  ldr wtmp2, [wtmp1]                // Get the direction value
  bic wtmp2, wtmp2, wnmask          // Clear the direciton mask
  str wtmp2, [wtmp1]                // Set the direction for D-

  pop {r3-r6}                       // Restore registers
  mov r11, r6
  mov r10, r5
  mov r9, r4
  mov r8, r3
  pop {r3-r7,pc}                    // Restore and return to caller.

  // Useful functions
usb_phy_write__state_se0:
  mov wpaddr, wdpclrreg             // D+ clr
  mov wnaddr, wdnclrreg             // D- clr
  b usb_phy_write__out_func
usb_phy_write__state_j:
  mov wpaddr, wdpsetreg             // D+ set
  mov wnaddr, wdnclrreg             // D- clr
  b usb_phy_write__out_func
usb_phy_write__state_k:
  mov wpaddr, wdpclrreg             // D+ clr
  ldr wnaddr, [sp, #0]              // D- set
  nop
usb_phy_write__out_func:
  str wpmask, [wpaddr]
  str wnmask, [wnaddr]
  b usb_phy__wait_25_cycles

.endfunc
.type usbPhyWriteI, %function
.size usbPhyWriteI, .-usbPhyWriteI
