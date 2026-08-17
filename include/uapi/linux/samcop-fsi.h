/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * UAPI for the SAMCOP FSI fingerprint sensor raw-acquisition driver
 * (HP iPAQ h54xx/h55xx, Atmel AT77C101B "FingerChip").
 *
 * read() on /dev/fsi returns raw native-endian 32-bit words, whole words
 * only; a count below 4 fails with EINVAL. The driver acquires frames of
 * SAMCOP_FSI_FRAME_WORDS words and only ever queues complete frames, so
 * the stream is frame-contiguous and the first word read after open() is
 * the first word of a frame. A blocking read of up to 8 frames
 * (8 * SAMCOP_FSI_FRAME_WORDS words) waits until that many words are
 * queued and returns them all, so a read of exactly one frame returns one
 * frame; a larger read returns 8 frames. A non-blocking read returns the
 * whole words queued, or fails with EAGAIN. After 10 consecutive frames
 * fail, acquisition stops: the heater is switched off, poll() reports
 * EPOLLERR and read() fails with EIO once the queued words are consumed.
 * Close and reopen the device to restart.
 *
 * This is a raw-acquisition ABI only. Presence detection, swipe
 * assembly and minutiae matching are userspace's job (for instance NBIS
 * mindtct/bozorth3), never the kernel's. libfprint has no generic
 * backend for a raw chardev transport like this one (its UDEV-matched
 * backends are SPIDEV and HIDRAW specifically); integrating with it
 * would need a bespoke libfprint driver written against this ABI.
 */
#ifndef _UAPI_LINUX_SAMCOP_FSI_H
#define _UAPI_LINUX_SAMCOP_FSI_H

#include <linux/ioctl.h>
#include <linux/types.h>

/*
 * One frame: word 0 is a header, words 1..280 are the sensor's 280
 * columns of eight 4-bit pixels each. In the header, bytes 0 and 1 are
 * sync bytes: each satisfies (b & 0xef) == 0xe0, so the header word
 * satisfies (w & 0xefef) == 0xe0e0. The vendor software uses the low
 * nibble of byte 2 as a table index (probably a temperature code); the
 * meaning of byte 3 is unknown. This layout comes from the vendor
 * driver's handling of the data, not from a datasheet. The hardware
 * delivers the header last; the driver moves it to the front, and
 * drops any frame whose header fails the sync test or that the
 * hardware flagged, so every delivered header passes it.
 */
#define SAMCOP_FSI_FRAME_WORDS		281

#define SAMCOP_FSI_IOC_MAGIC		'F'

/*
 * Pixel-clock prescaler: 8-bit divider, default 5, kept across opens. Values above 255
 * are rejected. A new value is written to the hardware the next time the
 * driver restarts the frame sequencer, before the next frame.
 */
#define SAMCOP_FSI_IOC_SET_PRESCALER	_IOW(SAMCOP_FSI_IOC_MAGIC, 0, __u32)
#define SAMCOP_FSI_IOC_GET_PRESCALER	_IOR(SAMCOP_FSI_IOC_MAGIC, 1, __u32)

/*
 * Integration count (the DMC register): 12-bit, default 0, kept across opens. Named
 * "integration count" rather than the DMC abbreviation's literal "dummy
 * clock" -- it is the sensor's pixel-reset-to-readout interval, not a
 * clock. Values above 4095 are rejected. Applied like the prescaler.
 */
#define SAMCOP_FSI_IOC_SET_INTEGRATION	_IOW(SAMCOP_FSI_IOC_MAGIC, 2, __u32)
#define SAMCOP_FSI_IOC_GET_INTEGRATION	_IOR(SAMCOP_FSI_IOC_MAGIC, 3, __u32)

/*
 * Temperature-stabilisation ramp control. AT77C101B is a pyroelectric
 * (thermal-differential) sensor with a TPP/TPE heating pair -- with no
 * ridge/valley temperature differential there is no signal at all.
 *
 * START arms TEMP_PWR_ON and a safety cutoff timer of (150 * arg)
 * jiffies -- a real closed-loop safety bound on a physical heating
 * element, not a policy knob. INC extends that cutoff by (150 * arg)
 * more jiffies without restarting the ramp. STOP clears TEMP_PWR_ON and
 * cancels the timer unconditionally; it takes no argument. The heater is
 * also switched off on close() and when acquisition stops on errors.
 */
#define SAMCOP_FSI_IOC_TEMP_START	_IOW(SAMCOP_FSI_IOC_MAGIC, 4, __u32)
#define SAMCOP_FSI_IOC_TEMP_INC		_IOW(SAMCOP_FSI_IOC_MAGIC, 5, __u32)
#define SAMCOP_FSI_IOC_TEMP_STOP	_IO(SAMCOP_FSI_IOC_MAGIC, 6)

/*
 * When the driver's queue of 8 frames is full, acquisition pauses rather
 * than dropping data, so no frame is ever split or lost from the middle
 * of the stream. This ioctl returns nonzero if acquisition paused at
 * least once since open() or the last GET_OVERRUN call -- that is, there
 * is a time gap between two delivered frames -- and clears the flag.
 */
#define SAMCOP_FSI_IOC_GET_OVERRUN	_IOR(SAMCOP_FSI_IOC_MAGIC, 7, __u32)

#endif /* _UAPI_LINUX_SAMCOP_FSI_H */
