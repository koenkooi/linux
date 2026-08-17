// SPDX-License-Identifier: GPL-2.0-only
/*
 * Raw-acquisition driver for the SAMCOP FSI fingerprint sensor cell
 * (HP iPAQ h54xx/h55xx, Atmel AT77C101B "FingerChip" thermal swipe
 * sensor, 500 dpi, 8x280 pixels). The h51xx has no sensor; the cell is
 * still probed on every variant, because nothing in the devicetree
 * distinguishes them and the register window probes cleanly either way.
 *
 * Only raw 32-bit FIFO words are moved from the hardware through a ring
 * buffer to read(). Presence detection, frame assembly and matching
 * belong in userspace.
 *
 * Register map:
 *
 *   0x00 Control       clock/reset mode, TEMP_PWR_ON, IRQ-source enables,
 *                       read-mode select
 *   0x04 Prescaler      PCLK divider, 8-bit, reset default 19
 *   0x08 DMC            integration count, 12-bit, reset default 1124
 *                       (the sensor's pixel-reset-to-readout interval)
 *   0x0c FIFO_Control   FIFO reset + trigger-level select
 *   0x10 STA            bit0 ACQ, bits9:1 word count, bit10
 *                       DCOL, bit11 INVL, bit12 LFRM
 *   0x14 DAT            FIFO data port, one read pops one word
 *
 * Clock: the "samcop-fsi" clkdev alias the MFD core registers on the FCD
 * gate (CPM_ClockControl bit 4). It is enabled per open and release, not
 * for the driver's lifetime. No power-gate GPIO for the sensor is known;
 * GPIO61 is the RS232 FORCEON pin and must not be used for it. How the
 * sensor is powered beyond the clock gate and pin mux is unverified.
 *
 * Pin muxing: GPD0-7 (FCD_DATA0..7) and GPE0-2 (FCD_RESET/FCD_PCLK/
 * FCD_TPE) are muxed to their alternate function by the MFD core's
 * samcop_reset_fcd(), called from samcop_fsi_hw_open(). They are not
 * GPIO consumers. ACKN and OE have no GPD/GPE bit and are presumed
 * handled inside the FSI sequencer.
 *
 * Acquisition is polled, as in the vendor WinCE driver, which keeps every
 * status interrupt enable clear. Enabling the DCOL/INVL/LFRM sources in
 * IRQ read mode stormed the FSI interrupt on an h5550 as soon as /dev/fsi
 * was read, until the SAMCOP storm backstop masked it. The cell's
 * interrupt resource is therefore not requested and the source stays
 * masked in the SAMCOP interrupt controller. The DMA read mode is not
 * used.
 *
 * DAT is read as one 32-bit word per access, as the vendor driver does; a
 * frame is 281 such words (a 4-byte header plus 280 columns of eight
 * 4-bit pixels).
 *
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/mfd/samcop.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include <uapi/linux/samcop-fsi.h>

/* Register offsets, relative to the SAMCOP_FSI_BASE cell resource --
 * decode lives here, not in include/linux/mfd/samcop.h, matching every
 * other SAMCOP sub-device's split (samcop-adc.c, leds-samcop.c,
 * samcop-sdi.c all keep their own register maps out of the shared MFD
 * header the same way).
 */
#define SAMCOP_FSI_CONTROL		0x00
#define SAMCOP_FSI_PRESCALER		0x04
#define SAMCOP_FSI_DMC			0x08	/* integration count */
#define SAMCOP_FSI_FIFO_CONTROL	0x0c
#define SAMCOP_FSI_STA			0x10
#define SAMCOP_FSI_DAT			0x14

#define SAMCOP_FSI_CONTROL_MODE_MASK		GENMASK(1, 0)
#define SAMCOP_FSI_CONTROL_MODE_FRAME_STOP	0x0
#define SAMCOP_FSI_CONTROL_MODE_FRAME_START	0x1
#define SAMCOP_FSI_CONTROL_MODE_NAP		0x2
#define SAMCOP_FSI_CONTROL_MODE_FRAME_RECEIVE	0x3
#define SAMCOP_FSI_CONTROL_TEMP_PWR_ON		BIT(2)
#define SAMCOP_FSI_CONTROL_CLK_OUT_EN		BIT(3)
#define SAMCOP_FSI_CONTROL_DCOL_IE		BIT(4)
#define SAMCOP_FSI_CONTROL_INVL_IE		BIT(5)
#define SAMCOP_FSI_CONTROL_LFRM_IE		BIT(6)
/* Read-mode select: 0 POLL (reset default), 1 IRQ, 2 DMA. */
#define SAMCOP_FSI_CONTROL_READ_MODE_MASK	GENMASK(8, 7)
#define SAMCOP_FSI_CONTROL_IRQ_BITS		(SAMCOP_FSI_CONTROL_DCOL_IE | \
						 SAMCOP_FSI_CONTROL_INVL_IE | \
						 SAMCOP_FSI_CONTROL_LFRM_IE | \
						 SAMCOP_FSI_CONTROL_READ_MODE_MASK)

#define SAMCOP_FSI_STA_ACQ			BIT(0)
/*
 * Named a FIFO word count, but on an h5550 it was read well above the
 * 281-word frame, so it is not a plain FIFO level: the driver never
 * reads more words than the rest of the current frame.
 */
#define SAMCOP_FSI_STA_FIFO_CNT_MASK		GENMASK(9, 1)
#define SAMCOP_FSI_STA_DCOL			BIT(10)
#define SAMCOP_FSI_STA_INVL			BIT(11)
#define SAMCOP_FSI_STA_LFRM			BIT(12)
/*
 * The vendor driver only ever writes this constant to STA, never the
 * value read back from it.
 */
#define SAMCOP_FSI_STA_ACK			(SAMCOP_FSI_STA_DCOL | \
						 SAMCOP_FSI_STA_INVL | \
						 SAMCOP_FSI_STA_LFRM)

/*
 * FIFO_Control: a FIFO reset plus a trigger-level select, bit positions
 * unknown. The vendor driver writes 7 on init, twice on every resync and
 * on shutdown; the register reads back 0.
 */
#define SAMCOP_FSI_FIFO_CONTROL_INIT		0x7

/* The values the vendor driver programs; it never changes them. */
#define SAMCOP_FSI_PRESCALER_DEFAULT		5
#define SAMCOP_FSI_PRESCALER_MAX		0xff	/* 8-bit field */
#define SAMCOP_FSI_DMC_DEFAULT			0
#define SAMCOP_FSI_DMC_MAX			0xfff	/* 12-bit field */

/*
 * The vendor driver treats 101 consecutive equal STA counts as "the
 * sequencer has stopped" and has no time bound; this one gives up after
 * SAMCOP_FSI_STABLE_TIMEOUT_US and carries on, as a resync follows anyway.
 */
#define SAMCOP_FSI_STABLE_READS			101
#define SAMCOP_FSI_STABLE_TIMEOUT_US		10000

/* Consecutive failed frames before acquisition stops with -EIO. */
#define SAMCOP_FSI_MAX_FAILS			10

/*
 * Ring buffer depth in 32-bit words, not bytes: ring_head and ring_tail
 * are word indices into a u32 array of exactly SAMCOP_FSI_RING_WORDS
 * elements.
 */
#define SAMCOP_FSI_RING_FRAMES		8U
#define SAMCOP_FSI_RING_WORDS		(SAMCOP_FSI_FRAME_WORDS * SAMCOP_FSI_RING_FRAMES)

/* Bit in samcop_fsi.flags. */
#define SAMCOP_FSI_PARAMS_DIRTY		0

/*
 * Busy-polled, because with HZ=100 and no high-resolution timers any
 * sleep would cap the frame rate far below the sensor's.
 */
static unsigned int timeout_us = 2000;
module_param(timeout_us, uint, 0644);
MODULE_PARM_DESC(timeout_us,
		 "Bound on each busy-wait for sensor data in microseconds, 1..100000 (default 2000)");

/* Sync test the vendor driver applies to header bytes 0 and 1. */
#define SAMCOP_FSI_SYNC_MASK		0xefef
#define SAMCOP_FSI_SYNC_VALUE		0xe0e0

struct samcop_fsi {
	struct device *dev;
	struct device *samcop_dev;	/* SAMCOP core device, for samcop_reset_fcd() */
	void __iomem *regs;
	struct clk *clk;
	struct miscdevice miscdev;

	/* Serialises Control read-modify-writes from the acquisition
	 * thread, the ioctls and the heater safety timer.
	 */
	spinlock_t reg_lock;

	/* Written to the hardware only while the sequencer is stopped. */
	u32 prescaler;
	u32 integration;
	unsigned long flags;

	/* Single-open exclusivity, see samcop_fsi_open(). */
	struct mutex open_lock;
	bool open;

	struct task_struct *thread;
	u32 frame[SAMCOP_FSI_FRAME_WORDS];	/* acquisition thread only */

	/*
	 * Ring of raw 32-bit words. Only whole frames are added, so the
	 * stream stays frame-aligned however read() splits it.
	 */
	struct mutex read_lock;
	u32 *ring;
	unsigned int ring_head;
	unsigned int ring_tail;
	unsigned int ring_count;
	spinlock_t ring_lock;
	wait_queue_head_t ring_wait;
	wait_queue_head_t space_wait;
	bool ring_overrun;
	int error;

	/* Temperature-ramp safety cutoff -- see uapi/linux/samcop-fsi.h's
	 * header comment on the (150 * arg) jiffies formula.
	 */
	struct timer_list temp_timer;
};

/*
 * Every Control write goes through here and also clears the status
 * interrupt enables and the read mode, so no path can turn the interrupt
 * source back on.
 */
static void samcop_fsi_control_rmw(struct samcop_fsi *fsi, u32 mask, u32 bits)
{
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&fsi->reg_lock, flags);
	val = readl_relaxed(fsi->regs + SAMCOP_FSI_CONTROL);
	/* Clear exactly the mask bits: (val & ~mask) | bits. */
	val = (val & ~(mask | SAMCOP_FSI_CONTROL_IRQ_BITS)) | bits;
	writel_relaxed(val, fsi->regs + SAMCOP_FSI_CONTROL);
	spin_unlock_irqrestore(&fsi->reg_lock, flags);
}

static void samcop_fsi_set_mode(struct samcop_fsi *fsi, u32 mode)
{
	samcop_fsi_control_rmw(fsi, SAMCOP_FSI_CONTROL_MODE_MASK, mode);
}

static inline u32 samcop_fsi_sta(struct samcop_fsi *fsi)
{
	return readl_relaxed(fsi->regs + SAMCOP_FSI_STA);
}

static inline unsigned int samcop_fsi_count(u32 sta)
{
	return FIELD_GET(SAMCOP_FSI_STA_FIFO_CNT_MASK, sta);
}

/*
 * The vendor driver bounds these waits by 100 STA reads; a time bound
 * keeps them independent of the bus speed.
 */
static int samcop_fsi_wait_data(struct samcop_fsi *fsi, u32 *sta)
{
	unsigned int tmo = clamp(READ_ONCE(timeout_us), 1U, 100000U);
	u32 val;
	int ret;

	ret = readl_relaxed_poll_timeout(fsi->regs + SAMCOP_FSI_STA, val,
					 samcop_fsi_count(val), 0, tmo);
	*sta = val;
	return ret;
}

static bool samcop_fsi_wait_stable(struct samcop_fsi *fsi)
{
	ktime_t end = ktime_add_us(ktime_get(), SAMCOP_FSI_STABLE_TIMEOUT_US);
	unsigned int last = samcop_fsi_count(samcop_fsi_sta(fsi));
	unsigned int same = 0, n;

	while (same < SAMCOP_FSI_STABLE_READS) {
		n = samcop_fsi_count(samcop_fsi_sta(fsi));
		if (n == last) {
			same++;
		} else {
			same = 0;
			last = n;
		}
		if (ktime_after(ktime_get(), end))
			return false;
	}

	return true;
}

/* Restart the frame sequencer; the vendor driver's resync sequence. */
static int samcop_fsi_resync(struct samcop_fsi *fsi)
{
	u32 sta;
	int ret;

	samcop_fsi_set_mode(fsi, SAMCOP_FSI_CONTROL_MODE_FRAME_STOP);
	if (!samcop_fsi_wait_stable(fsi))
		dev_dbg_ratelimited(fsi->dev, "STA count did not settle\n");

	if (test_and_clear_bit(SAMCOP_FSI_PARAMS_DIRTY, &fsi->flags)) {
		writel_relaxed(READ_ONCE(fsi->prescaler),
			       fsi->regs + SAMCOP_FSI_PRESCALER);
		writel_relaxed(READ_ONCE(fsi->integration),
			       fsi->regs + SAMCOP_FSI_DMC);
	}

	writel_relaxed(SAMCOP_FSI_STA_ACK, fsi->regs + SAMCOP_FSI_STA);
	writel_relaxed(SAMCOP_FSI_FIFO_CONTROL_INIT,
		       fsi->regs + SAMCOP_FSI_FIFO_CONTROL);
	writel_relaxed(SAMCOP_FSI_FIFO_CONTROL_INIT,
		       fsi->regs + SAMCOP_FSI_FIFO_CONTROL);

	samcop_fsi_set_mode(fsi, SAMCOP_FSI_CONTROL_MODE_FRAME_START);
	ret = samcop_fsi_wait_data(fsi, &sta);
	samcop_fsi_set_mode(fsi, SAMCOP_FSI_CONTROL_MODE_FRAME_STOP);

	return ret;
}

/*
 * Read one frame into fsi->frame, the vendor driver's per-frame
 * sequence: resync when the sequencer state looks stale, read the frame
 * in chunks of whatever STA reports, then switch to FRAME_RECEIVE and back
 * to FRAME_STOP, which prepares the next frame.
 */
static int samcop_fsi_acquire_one(struct samcop_fsi *fsi, bool force)
{
	unsigned int total = 0, n, i;
	int ret, fret;
	u32 sta;

	sta = samcop_fsi_sta(fsi);
	n = samcop_fsi_count(sta);
	if (force || test_bit(SAMCOP_FSI_PARAMS_DIRTY, &fsi->flags) ||
	    (sta & (SAMCOP_FSI_STA_DCOL | SAMCOP_FSI_STA_INVL)) ||
	    !n || n > SAMCOP_FSI_FRAME_WORDS) {
		ret = samcop_fsi_resync(fsi);
		if (ret)
			return ret;
	}

	ret = 0;
	while (total < SAMCOP_FSI_FRAME_WORDS) {
		ret = samcop_fsi_wait_data(fsi, &sta);
		if (ret)
			break;

		n = samcop_fsi_count(sta);
		if (n > SAMCOP_FSI_FRAME_WORDS - total) {
			ret = -EIO;
			break;
		}
		for (i = 0; i < n; i++)
			fsi->frame[total++] = readl_relaxed(fsi->regs + SAMCOP_FSI_DAT);

		sta = samcop_fsi_sta(fsi);
		if (sta & (SAMCOP_FSI_STA_DCOL | SAMCOP_FSI_STA_INVL)) {
			ret = -EIO;
			break;
		}
		cond_resched();
	}

	writel_relaxed(SAMCOP_FSI_STA_ACK, fsi->regs + SAMCOP_FSI_STA);
	samcop_fsi_set_mode(fsi, SAMCOP_FSI_CONTROL_MODE_FRAME_RECEIVE);
	fret = samcop_fsi_wait_data(fsi, &sta);
	samcop_fsi_set_mode(fsi, SAMCOP_FSI_CONTROL_MODE_FRAME_STOP);

	return ret ?: fret;
}

/*
 * The vendor driver tests the sync bytes in the last word read and then
 * moves that word to the front, so frames are delivered header first.
 */
static int samcop_fsi_check_frame(struct samcop_fsi *fsi)
{
	u32 hdr = fsi->frame[SAMCOP_FSI_FRAME_WORDS - 1];

	memmove(&fsi->frame[1], &fsi->frame[0],
		(SAMCOP_FSI_FRAME_WORDS - 1) * sizeof(u32));
	fsi->frame[0] = hdr;

	return (hdr & SAMCOP_FSI_SYNC_MASK) == SAMCOP_FSI_SYNC_VALUE ? 0 : -EIO;
}

static bool samcop_fsi_ring_space(struct samcop_fsi *fsi)
{
	bool space;

	spin_lock(&fsi->ring_lock);
	space = SAMCOP_FSI_RING_WORDS - fsi->ring_count >= SAMCOP_FSI_FRAME_WORDS;
	spin_unlock(&fsi->ring_lock);

	return space;
}

static void samcop_fsi_ring_commit(struct samcop_fsi *fsi)
{
	unsigned int first;

	spin_lock(&fsi->ring_lock);
	first = min_t(unsigned int, SAMCOP_FSI_FRAME_WORDS,
		      SAMCOP_FSI_RING_WORDS - fsi->ring_head);
	memcpy(fsi->ring + fsi->ring_head, fsi->frame, first * sizeof(u32));
	memcpy(fsi->ring, fsi->frame + first,
	       (SAMCOP_FSI_FRAME_WORDS - first) * sizeof(u32));
	fsi->ring_head = (fsi->ring_head + SAMCOP_FSI_FRAME_WORDS) %
			 SAMCOP_FSI_RING_WORDS;
	fsi->ring_count += SAMCOP_FSI_FRAME_WORDS;
	spin_unlock(&fsi->ring_lock);

	wake_up_interruptible(&fsi->ring_wait);
}

static void samcop_fsi_temp_stop(struct samcop_fsi *fsi);

static void samcop_fsi_stop_on_error(struct samcop_fsi *fsi, int err)
{
	samcop_fsi_temp_stop(fsi);
	samcop_fsi_set_mode(fsi, SAMCOP_FSI_CONTROL_MODE_FRAME_STOP);

	spin_lock(&fsi->ring_lock);
	fsi->error = err;
	spin_unlock(&fsi->ring_lock);
	wake_up_interruptible(&fsi->ring_wait);

	dev_err(fsi->dev, "%u consecutive frames failed, acquisition stopped\n",
		SAMCOP_FSI_MAX_FAILS);
}

static int samcop_fsi_thread(void *data)
{
	struct samcop_fsi *fsi = data;
	unsigned int fails = 0;
	bool force = true;
	int ret;

	while (!kthread_should_stop()) {
		/*
		 * Pause instead of dropping data when the reader falls
		 * behind; the next frame's entry check resyncs if the
		 * sequencer state went stale meanwhile.
		 */
		if (!samcop_fsi_ring_space(fsi)) {
			spin_lock(&fsi->ring_lock);
			fsi->ring_overrun = true;
			spin_unlock(&fsi->ring_lock);
			wait_event_interruptible(fsi->space_wait,
						 samcop_fsi_ring_space(fsi) ||
						 kthread_should_stop());
			continue;
		}

		ret = samcop_fsi_acquire_one(fsi, force);
		if (!ret)
			ret = samcop_fsi_check_frame(fsi);
		if (!ret) {
			samcop_fsi_ring_commit(fsi);
			fails = 0;
			force = false;
		} else {
			force = true;
			if (++fails >= SAMCOP_FSI_MAX_FAILS) {
				samcop_fsi_stop_on_error(fsi, -EIO);
				break;
			}
		}
		cond_resched();
	}

	/* kthread_stop() from release() expects the thread to still exist. */
	for (;;) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (kthread_should_stop())
			break;
		schedule();
	}
	__set_current_state(TASK_RUNNING);

	return 0;
}

static void samcop_fsi_temp_timeout(struct timer_list *t)
{
	struct samcop_fsi *fsi = timer_container_of(fsi, t, temp_timer);

	/* Safety cutoff: unconditionally clear TEMP_PWR_ON. This is closed-
	 * loop control of a physical heating element (the sensor's TPP/TPE
	 * pair) -- see uapi/linux/samcop-fsi.h's header comment -- not a
	 * policy default, so it fires whether or not userspace ever calls
	 * FSI_IOC_TEMP_STOP.
	 */
	samcop_fsi_control_rmw(fsi, SAMCOP_FSI_CONTROL_TEMP_PWR_ON, 0);
}

/* Longest heater on-time userspace can ask for, in 150 jiffy steps. */
#define SAMCOP_FSI_TEMP_MAX_STEPS	((600 * HZ) / 150)

static void samcop_fsi_temp_start(struct samcop_fsi *fsi, u32 arg)
{
	arg = min_t(u32, arg, SAMCOP_FSI_TEMP_MAX_STEPS);
	samcop_fsi_control_rmw(fsi, SAMCOP_FSI_CONTROL_TEMP_PWR_ON,
			       SAMCOP_FSI_CONTROL_TEMP_PWR_ON);
	mod_timer(&fsi->temp_timer, jiffies + 150UL * arg);
}

static void samcop_fsi_temp_inc(struct samcop_fsi *fsi, u32 arg)
{
	arg = min_t(u32, arg, SAMCOP_FSI_TEMP_MAX_STEPS);
	/* Extend the cutoff without touching TEMP_PWR_ON or restarting the
	 * ramp itself -- mod_timer() on an already-armed timer just moves
	 * its expiry.
	 */
	mod_timer(&fsi->temp_timer, jiffies + 150UL * arg);
}

static void samcop_fsi_temp_stop(struct samcop_fsi *fsi)
{
	timer_delete_sync(&fsi->temp_timer);
	samcop_fsi_control_rmw(fsi, SAMCOP_FSI_CONTROL_TEMP_PWR_ON, 0);
}

/*
 * Leave the block stopped, without heater, and in NAP, the vendor
 * driver's shutdown state. Called with the clock enabled; at probe this
 * also clears whatever a previous kernel or the bootloader left behind.
 */
static void samcop_fsi_quiesce(struct samcop_fsi *fsi)
{
	samcop_fsi_control_rmw(fsi, SAMCOP_FSI_CONTROL_MODE_MASK |
				    SAMCOP_FSI_CONTROL_TEMP_PWR_ON,
			       SAMCOP_FSI_CONTROL_MODE_FRAME_STOP);
	writel_relaxed(SAMCOP_FSI_STA_ACK, fsi->regs + SAMCOP_FSI_STA);
	writel_relaxed(SAMCOP_FSI_FIFO_CONTROL_INIT,
		       fsi->regs + SAMCOP_FSI_FIFO_CONTROL);
	samcop_fsi_set_mode(fsi, SAMCOP_FSI_CONTROL_MODE_NAP);
}

static int samcop_fsi_hw_open(struct samcop_fsi *fsi)
{
	struct task_struct *thread;
	int ret;

	ret = clk_prepare_enable(fsi->clk);
	if (ret)
		return ret;

	/* Route GPD/GPE to the sensor before touching any FSI register --
	 * see this file's header comment on pin muxing.
	 */
	ret = samcop_reset_fcd(fsi->samcop_dev);
	if (ret)
		goto err_clk;

	/* The vendor driver's initialisation order. */
	clear_bit(SAMCOP_FSI_PARAMS_DIRTY, &fsi->flags);
	writel_relaxed(fsi->prescaler, fsi->regs + SAMCOP_FSI_PRESCALER);
	writel_relaxed(fsi->integration, fsi->regs + SAMCOP_FSI_DMC);
	writel_relaxed(SAMCOP_FSI_FIFO_CONTROL_INIT,
		       fsi->regs + SAMCOP_FSI_FIFO_CONTROL);
	samcop_fsi_control_rmw(fsi, ~0U, SAMCOP_FSI_CONTROL_CLK_OUT_EN);
	samcop_fsi_set_mode(fsi, SAMCOP_FSI_CONTROL_MODE_NAP);
	samcop_fsi_control_rmw(fsi, SAMCOP_FSI_CONTROL_MODE_MASK |
				    SAMCOP_FSI_CONTROL_TEMP_PWR_ON,
			       SAMCOP_FSI_CONTROL_MODE_FRAME_STOP);

	spin_lock(&fsi->ring_lock);
	fsi->ring_head = 0;
	fsi->ring_tail = 0;
	fsi->ring_count = 0;
	fsi->ring_overrun = false;
	fsi->error = 0;
	spin_unlock(&fsi->ring_lock);

	thread = kthread_run(samcop_fsi_thread, fsi, "samcop-fsi");
	if (IS_ERR(thread)) {
		ret = PTR_ERR(thread);
		samcop_fsi_quiesce(fsi);
		goto err_clk;
	}
	fsi->thread = thread;

	return 0;

err_clk:
	clk_disable_unprepare(fsi->clk);
	return ret;
}

static void samcop_fsi_hw_close(struct samcop_fsi *fsi)
{
	kthread_stop(fsi->thread);
	fsi->thread = NULL;
	/*
	 * The heater timer does a Control read-modify-write, so it must be
	 * gone before the clock is gated, and the heater must not outlive
	 * the open file.
	 */
	samcop_fsi_temp_stop(fsi);
	samcop_fsi_quiesce(fsi);
	clk_disable_unprepare(fsi->clk);
}

static int samcop_fsi_open(struct inode *inode, struct file *file)
{
	struct samcop_fsi *fsi = container_of(file->private_data, struct samcop_fsi, miscdev);
	int ret;

	/* Single open, checked and set atomically under open_lock. */
	mutex_lock(&fsi->open_lock);
	if (fsi->open) {
		mutex_unlock(&fsi->open_lock);
		return -EBUSY;
	}
	fsi->open = true;
	mutex_unlock(&fsi->open_lock);

	ret = samcop_fsi_hw_open(fsi);
	if (ret) {
		mutex_lock(&fsi->open_lock);
		fsi->open = false;
		mutex_unlock(&fsi->open_lock);
		return ret;
	}

	file->private_data = fsi;

	return 0;
}

static int samcop_fsi_release(struct inode *inode, struct file *file)
{
	struct samcop_fsi *fsi = file->private_data;

	samcop_fsi_hw_close(fsi);

	mutex_lock(&fsi->open_lock);
	fsi->open = false;
	mutex_unlock(&fsi->open_lock);

	return 0;
}

static bool samcop_fsi_readable(struct samcop_fsi *fsi, unsigned int words)
{
	bool ready;

	spin_lock(&fsi->ring_lock);
	ready = fsi->ring_count >= words || fsi->error;
	spin_unlock(&fsi->ring_lock);

	return ready;
}

static ssize_t samcop_fsi_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct samcop_fsi *fsi = file->private_data;
	unsigned int want, avail, tail, first, n;
	ssize_t ret;
	int err;

	if (count < sizeof(u32))
		return -EINVAL;

	/*
	 * A blocking read waits for everything it asked for, up to the
	 * ring size, so a frame-sized read returns exactly one frame.
	 */
	want = min_t(size_t, count / sizeof(u32), SAMCOP_FSI_RING_WORDS);

	if (mutex_lock_interruptible(&fsi->read_lock))
		return -ERESTARTSYS;

	if (!(file->f_flags & O_NONBLOCK)) {
		ret = wait_event_interruptible(fsi->ring_wait,
					       samcop_fsi_readable(fsi, want));
		if (ret)
			goto out;
	}

	spin_lock(&fsi->ring_lock);
	avail = fsi->ring_count;
	tail = fsi->ring_tail;
	err = fsi->error;
	spin_unlock(&fsi->ring_lock);

	if (!avail) {
		ret = err ?: -EAGAIN;
		goto out;
	}

	/* The thread only writes outside [tail, tail + avail). */
	n = min(want, avail);
	first = min(n, SAMCOP_FSI_RING_WORDS - tail);
	if (copy_to_user(buf, fsi->ring + tail, first * sizeof(u32)) ||
	    copy_to_user(buf + first * sizeof(u32), fsi->ring,
			 (n - first) * sizeof(u32))) {
		ret = -EFAULT;
		goto out;
	}

	spin_lock(&fsi->ring_lock);
	fsi->ring_tail = (tail + n) % SAMCOP_FSI_RING_WORDS;
	fsi->ring_count -= n;
	spin_unlock(&fsi->ring_lock);
	wake_up_interruptible(&fsi->space_wait);

	ret = n * sizeof(u32);
out:
	mutex_unlock(&fsi->read_lock);
	return ret;
}

static __poll_t samcop_fsi_poll(struct file *file, poll_table *wait)
{
	struct samcop_fsi *fsi = file->private_data;
	__poll_t mask = 0;

	poll_wait(file, &fsi->ring_wait, wait);

	spin_lock(&fsi->ring_lock);
	if (fsi->ring_count)
		mask |= EPOLLIN | EPOLLRDNORM;
	if (fsi->error)
		mask |= EPOLLERR;
	spin_unlock(&fsi->ring_lock);

	return mask;
}

static long samcop_fsi_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct samcop_fsi *fsi = file->private_data;
	void __user *argp = (void __user *)arg;
	u32 val;

	switch (cmd) {
	case SAMCOP_FSI_IOC_SET_PRESCALER:
		if (copy_from_user(&val, argp, sizeof(val)))
			return -EFAULT;
		if (val > SAMCOP_FSI_PRESCALER_MAX)
			return -EINVAL;
		WRITE_ONCE(fsi->prescaler, val);
		set_bit(SAMCOP_FSI_PARAMS_DIRTY, &fsi->flags);
		return 0;

	case SAMCOP_FSI_IOC_GET_PRESCALER:
		val = READ_ONCE(fsi->prescaler);
		return copy_to_user(argp, &val, sizeof(val)) ? -EFAULT : 0;

	case SAMCOP_FSI_IOC_SET_INTEGRATION:
		if (copy_from_user(&val, argp, sizeof(val)))
			return -EFAULT;
		if (val > SAMCOP_FSI_DMC_MAX)
			return -EINVAL;
		WRITE_ONCE(fsi->integration, val);
		set_bit(SAMCOP_FSI_PARAMS_DIRTY, &fsi->flags);
		return 0;

	case SAMCOP_FSI_IOC_GET_INTEGRATION:
		val = READ_ONCE(fsi->integration);
		return copy_to_user(argp, &val, sizeof(val)) ? -EFAULT : 0;

	case SAMCOP_FSI_IOC_TEMP_START:
		if (copy_from_user(&val, argp, sizeof(val)))
			return -EFAULT;
		samcop_fsi_temp_start(fsi, val);
		return 0;

	case SAMCOP_FSI_IOC_TEMP_INC:
		if (copy_from_user(&val, argp, sizeof(val)))
			return -EFAULT;
		samcop_fsi_temp_inc(fsi, val);
		return 0;

	case SAMCOP_FSI_IOC_TEMP_STOP:
		samcop_fsi_temp_stop(fsi);
		return 0;

	case SAMCOP_FSI_IOC_GET_OVERRUN:
		spin_lock(&fsi->ring_lock);
		val = fsi->ring_overrun;
		fsi->ring_overrun = false;
		spin_unlock(&fsi->ring_lock);

		return copy_to_user(argp, &val, sizeof(val)) ? -EFAULT : 0;

	default:
		return -ENOTTY;
	}
}

static const struct file_operations samcop_fsi_fops = {
	.owner		= THIS_MODULE,
	.open		= samcop_fsi_open,
	.release	= samcop_fsi_release,
	.read		= samcop_fsi_read,
	.poll		= samcop_fsi_poll,
	.unlocked_ioctl	= samcop_fsi_ioctl,
	.llseek		= noop_llseek,
};

static int samcop_fsi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct samcop_fsi *fsi;
	struct resource *mem;
	int ret;

	fsi = devm_kzalloc(dev, sizeof(*fsi), GFP_KERNEL);
	if (!fsi)
		return -ENOMEM;

	fsi->dev = dev;
	/*
	 * The mfd core sets each cell's platform_device.dev.parent to the
	 * SAMCOP core device that registered it -- samcop_reset_fcd() takes
	 * exactly that device (dev_get_drvdata() against it recovers the
	 * core's private struct samcop *).
	 */
	fsi->samcop_dev = dev->parent;

	/* Look the register window up by type, not by resource index. */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -EINVAL;

	/*
	 * devm_ioremap(), not devm_ioremap_resource(): the latter claims the
	 * region exclusively and would fail with -EBUSY, because the SAMCOP
	 * core already holds the whole MMIO window.
	 */
	fsi->regs = devm_ioremap(dev, mem->start, resource_size(mem));
	if (!fsi->regs)
		return -ENOMEM;

	/* Not devm_clk_get_enabled() -- gated per-open/-release instead,
	 * see this file's header comment on power sequencing.
	 */
	fsi->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(fsi->clk))
		return dev_err_probe(dev, PTR_ERR(fsi->clk), "failed to get clock\n");

	spin_lock_init(&fsi->reg_lock);

	ret = clk_prepare_enable(fsi->clk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable clock\n");
	samcop_fsi_quiesce(fsi);
	clk_disable_unprepare(fsi->clk);

	fsi->prescaler = SAMCOP_FSI_PRESCALER_DEFAULT;
	fsi->integration = SAMCOP_FSI_DMC_DEFAULT;

	fsi->ring = devm_kcalloc(dev, SAMCOP_FSI_RING_WORDS, sizeof(*fsi->ring), GFP_KERNEL);
	if (!fsi->ring)
		return -ENOMEM;

	spin_lock_init(&fsi->ring_lock);
	init_waitqueue_head(&fsi->ring_wait);
	init_waitqueue_head(&fsi->space_wait);
	mutex_init(&fsi->open_lock);
	mutex_init(&fsi->read_lock);
	timer_setup(&fsi->temp_timer, samcop_fsi_temp_timeout, 0);

	fsi->miscdev.minor = MISC_DYNAMIC_MINOR;
	fsi->miscdev.name = "fsi";
	fsi->miscdev.fops = &samcop_fsi_fops;
	fsi->miscdev.parent = dev;

	ret = misc_register(&fsi->miscdev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register misc device\n");

	platform_set_drvdata(pdev, fsi);

	dev_info(dev, "SAMCOP FSI fingerprint sensor cell ready (h54xx/h55xx only)\n");

	return 0;
}

static void samcop_fsi_remove(struct platform_device *pdev)
{
	struct samcop_fsi *fsi = platform_get_drvdata(pdev);

	misc_deregister(&fsi->miscdev);
	timer_shutdown_sync(&fsi->temp_timer);

	/*
	 * misc_deregister() waits out any in-flight file operations, so no
	 * open() can still be racing in here -- but an already-open fd's
	 * hardware state (clock, acquisition thread, timer) is only ever torn
	 * down by release(), and there is nothing left to synchronously
	 * force-close from a platform_driver's .remove(). This is the same
	 * shape every other devm-based SAMCOP cell leaves to its device
	 * model teardown ordering; a real bind/unbind race here would need
	 * a per-open refcount on the platform_device itself, which nothing
	 * in this MFD's other cells implements either.
	 */
}

static struct platform_driver samcop_fsi_driver = {
	.probe	= samcop_fsi_probe,
	.remove	= samcop_fsi_remove,
	.driver = {
		.name	= "samcop-fsi",
		/*
		 * The capture thread polls the block through devm resources and
		 * misc_deregister() does not wait for open files, so unbinding
		 * under an open /dev/fsi would free them in use.
		 */
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(samcop_fsi_driver);

MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("SAMCOP FSI fingerprint sensor raw-acquisition driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:samcop-fsi");
