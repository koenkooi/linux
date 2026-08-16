// SPDX-License-Identifier: GPL-2.0-only
/*
 * MMC host driver for the SDI (SD/MMC) block on the Samsung HAMCOP
 * companion ASIC (HP iPAQ h2200/h2210/h2212/h2215) -- the real, physical
 * SD card slot on this board. The PXA25x SoC's own MMC controller
 * (pxamci) is never used here: h2200's slot hangs entirely
 * off HAMCOP, and CONFIG_MMC_PXA staying unset is correct, not an
 * oversight.
 *
 * PIO only. HAMCOP's SDI block can also drive its own DMA channel into
 * on-chip SRAM (see the 2007 driver's dmabounce-into-SRAM scheme), but
 * that scheme is unportable as-is (dmabounce is gone from mainline) and
 * the SRAM it targeted is now owned outright by arch/arm/mach-pxa/
 * h2200_pm.c's WinCE-bootloader-preservation dance (see hamcop.c's top
 * comment) -- a real internal-DMA path needs a SRAM-claim API neither of
 * those drivers expose yet. That is out of scope for this pass.
 *
 * Card-detect and slot-power are h2200 board lines (PXA GPIO8 and GPIO61
 * respectively), not HAMCOP registers. Card detect is the generic "cd"
 * gpiod slot line (mmc/slot-gpio.h), falling back to MMC_CAP_NEEDS_POLL
 * when no board supplies one. Slot power is the optional "vmmc"
 * regulator.
 *
 * The SD bus pull-ups (HAMCOP's own, in the shared SPUCR register) are
 * switched with the rail; see include/linux/mfd/hamcop.h.
 *
 * Register map: this is a from-scratch rewrite, not a port of
 * the old out-of-tree samcop_sdi.c -- that driver targets the
 * generic 32-bit-register "SAMCOP" SDI variant (shared with several other
 * HTC/Simtec companion chips), while HAMCOP's own SDI block halves every
 * register to 16 bits (2-byte stride) and, in a few places, reshuffles
 * bitfields across genuinely different registers rather than just
 * halving an existing one. None of that split is guesswork: the MFD core
 * of the old out-of-tree port carries a meta-register shim
 * (hamcop_sdi_read_register()/hamcop_sdi_write_register()) whose whole
 * job is translating SAMCOP's 32-bit register set onto HAMCOP's real
 * ones for the *same* samcop_sdi.c driver -- i.e. it's the closest thing
 * this chip has to a datasheet, written by whoever had HP's real HAMCOP
 * documentation in hand. Every register/bitfield split below is read
 * straight out of that shim, not inferred:
 *
 *  - SDICON, SDIPRE, SDICMDCON, SDICMDSTAT, SDIDSTA, SDIFSTA: passed
 *    through unchanged -- these never used more than 16 bits of SAMCOP's
 *    32-bit registers to begin with, so HAMCOP's bitfield layout for them
 *    is bit-identical to SAMCOP's (see samcop-sdi.h in the same tree).
 *  - SDICMDARG, SDIRSP0-3: split into an H/L register pair, H = bits
 *    31:16, L = bits 15:0.
 *  - SDITIMER: same H/L split (TIMER1 = bits 31:16, TIMER2 = bits 15:0).
 *  - SDIDCON: SAMCOP packed an 11-bit block count into bits [10:0] and
 *    every control flag from bit 12 up. HAMCOP moved the block count out
 *    to its own SDIBNUM register and left SDIDCON holding just the
 *    control flags, shifted down by 12 to close the gap (SAMCOP bit N ->
 *    HAMCOP_SDIDCON bit N-12).
 *  - SDIBNUM: SAMCOP's bits [11:0] (masked to 0xfff), now a register of
 *    its own instead of living inside SDIDCON.
 *  - SDIIMSK1/SDIIMSK2: SAMCOP's 18-bit-wide mask doesn't fit one 16-bit
 *    register. The shim splits it at bit 5, not bit 16: SDIIMSK2 holds
 *    SAMCOP bits [4:0] unchanged (the RX/TXFIFO threshold bits), and
 *    SDIIMSK1 holds SAMCOP bits [17:5] shifted down by 5.
 *  - SDIDCNT1/SDIDCNT2: SAMCOP's status-only remaining-count register
 *    ([23:12]=blocks, [11:0]=bytes) split the same way as SDIDCON/SDIBNUM
 *    (DCNT1 = blocks remaining, DCNT2 = bytes remaining). Not decoded by
 *    this driver -- nothing here needs it bit-exact, only for a debug
 *    dump, and that isn't included in this first pass.
 *  - SDIDATA: a single 16-bit FIFO port, read/written twice per SAMCOP
 *    32-bit "word" rather than once -- i.e. genuinely a 2-byte access
 *    granularity, not a wider register split into halves. This is the
 *    piece that actually drives this driver's PIO loop shape.
 *
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/mfd/hamcop.h>
#include <linux/minmax.h>
#include <linux/mmc/host.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/scatterlist.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

/* ---------------------------------------------------------------------
 * Register offsets, relative to this cell's own mem resource (i.e.
 * already HAMCOP_SDI_BASE-relative -- see include/linux/mfd/hamcop.h and
 * drivers/mfd/hamcop.c's hamcop_sdi_resources[]). Recovered from
 * the old out-of-tree hamcop-sdi.h.
 * ---------------------------------------------------------------------
 */
#define HAMCOP_SDI_CON		0x00
#define HAMCOP_SDI_PRE		0x02
#define HAMCOP_SDI_CMDARGH	0x04
#define HAMCOP_SDI_CMDARGL	0x06
#define HAMCOP_SDI_CMDCON	0x08
#define HAMCOP_SDI_CMDSTAT	0x0a
#define HAMCOP_SDI_RSP0H	0x0c
#define HAMCOP_SDI_RSP0L	0x0e
#define HAMCOP_SDI_RSP1H	0x10
#define HAMCOP_SDI_RSP1L	0x12
#define HAMCOP_SDI_RSP2H	0x14
#define HAMCOP_SDI_RSP2L	0x16
#define HAMCOP_SDI_RSP3H	0x18
#define HAMCOP_SDI_RSP3L	0x1a
#define HAMCOP_SDI_TIMER1	0x1c
#define HAMCOP_SDI_TIMER2	0x1e
#define HAMCOP_SDI_BSIZE	0x20
#define HAMCOP_SDI_BNUM		0x22
#define HAMCOP_SDI_DCON		0x24
#define HAMCOP_SDI_DCNT1	0x26
#define HAMCOP_SDI_DCNT2	0x28
#define HAMCOP_SDI_DSTA		0x2a
#define HAMCOP_SDI_FSTA		0x2c
#define HAMCOP_SDI_IMSK1	0x30
#define HAMCOP_SDI_IMSK2	0x32
#define HAMCOP_SDI_DATA		0x60

#define HAMCOP_SDI_BNUM_MASK	0x0fff

/* Real hardware ceiling on the TIMER1:TIMER2 pair -- only 21 bits are
 * implemented (the old out-of-tree HAMCOP_SDI_TIMER_MAX), not the full
 * 32 bits H:L addressing could reach. Use it rather than a hand-picked
 * "big enough" value so we never write into whatever bits 21-31 actually
 * are.
 */
#define HAMCOP_SDI_TIMER_MAX	0x1fffff

/* SDICON: bit-identical to SAMCOP/S3C2410 for FIFORESET; bit 0 is
 * genuinely different -- HAMCOP repurposes it as an SD-vs-MMC protocol
 * mode select (confirmed by hamcop-sdi.h) rather than SAMCOP/S3C2410's
 * SDCLK on/off toggle. This driver only ever runs the h2200 SD slot, so
 * it is written once and never touched again -- see hamcop_sdi_set_ios().
 */
#define HAMCOP_SDICON_CLOCKTYPE_SD	(0 << 0)
#define HAMCOP_SDICON_CLOCKTYPE_MMC	BIT(0)
#define HAMCOP_SDICON_FIFORESET	BIT(1)

/* SDICMDCON / SDICMDSTAT / SDIDSTA / SDIFSTA: unchanged from SAMCOP. */
#define HAMCOP_SDICMDCON_LONGRSP	BIT(10)
#define HAMCOP_SDICMDCON_WAITRSP	BIT(9)
#define HAMCOP_SDICMDCON_CMDSTART	BIT(8)
#define HAMCOP_SDICMDCON_SENDERHOST	BIT(6)
#define HAMCOP_SDICMDCON_INDEX		0x3f

#define HAMCOP_SDICMDSTAT_CRCFAIL	BIT(12)
#define HAMCOP_SDICMDSTAT_CMDSENT	BIT(11)
#define HAMCOP_SDICMDSTAT_CMDTIMEOUT	BIT(10)
#define HAMCOP_SDICMDSTAT_RSPFIN	BIT(9)

#define HAMCOP_SDIDSTA_FIFOFAIL	BIT(8)
#define HAMCOP_SDIDSTA_CRCFAIL		BIT(7)
#define HAMCOP_SDIDSTA_RXCRCFAIL	BIT(6)
#define HAMCOP_SDIDSTA_DATATIMEOUT	BIT(5)
#define HAMCOP_SDIDSTA_XFERFINISH	BIT(4)
#define HAMCOP_SDIDSTA_TXDATAON	BIT(1)
#define HAMCOP_SDIDSTA_RXDATAON	BIT(0)

#define HAMCOP_SDIFSTA_TFDET		BIT(13)
#define HAMCOP_SDIFSTA_RFDET		BIT(12)
/*
 * COUNTMASK is the width of the occupancy field, not the capacity. Seven
 * bits is what an occupancy of 0..64 requires, and a FIFO deeper than 64
 * bytes was ruled out on hardware: pushing a whole transfer into it in one
 * burst never raised the observed occupancy above 64. Only the write path
 * consumes the depth, through hamcop_sdi_fifo_free(), so overestimating it
 * hands out space that does not exist and overruns the FIFO on every
 * write while reads keep working.
 */
#define HAMCOP_SDIFSTA_COUNTMASK	0x7f
#define HAMCOP_SDI_FIFO_DEPTH		64

/*
 * SDIDCON: SAMCOP's control flags, shifted down by 12 (SAMCOP moved its
 * BLKNUM field out to SDIBNUM -- see the file header comment). DATMODE
 * is a 2-bit field at bits [1:0]; the two encodings this driver needs are
 * the only ones defined.
 */
#define HAMCOP_SDIDCON_XFER_RXSTART	(2 << 0)
#define HAMCOP_SDIDCON_XFER_TXSTART	(3 << 0)
#define HAMCOP_SDIDCON_STOP		BIT(2)
#define HAMCOP_SDIDCON_WIDEBUS		BIT(4)
#define HAMCOP_SDIDCON_BLOCKMODE	BIT(5)
#define HAMCOP_SDIDCON_RXAFTERCMD	BIT(7)
#define HAMCOP_SDIDCON_TXAFTERRESP	BIT(8)

/*
 * SDIIMSK1/SDIIMSK2: SAMCOP's 18-bit mask split at bit 5 (not bit 16) --
 * SDIIMSK2 holds SAMCOP bits [4:0] verbatim, SDIIMSK1 holds SAMCOP bits
 * [17:5] shifted down by 5. Only the bits this driver actually sets are
 * named.
 */
#define HAMCOP_SDIIMSK1_DATAFINISH	BIT(2)
#define HAMCOP_SDIIMSK1_DATATIMEOUT	BIT(3)
#define HAMCOP_SDIIMSK1_DATACRC	BIT(4)
#define HAMCOP_SDIIMSK1_CRCSTATUS	BIT(5)
#define HAMCOP_SDIIMSK1_FIFOFAIL	BIT(6)
#define HAMCOP_SDIIMSK1_RESPONSEND	BIT(9)
#define HAMCOP_SDIIMSK1_CMDTIMEOUT	BIT(10)
#define HAMCOP_SDIIMSK1_CMDSENT	BIT(11)
#define HAMCOP_SDIIMSK1_RESPONSECRC	BIT(12)

#define HAMCOP_SDIIMSK2_RXFIFOHALF	BIT(0)
#define HAMCOP_SDIIMSK2_RXFIFOLAST	BIT(2)
#define HAMCOP_SDIIMSK2_TXFIFOHALF	BIT(4)

/* SD protocol minimum: 74 SDCLK cycles before the card is ready for its
 * first command. The old out-of-tree hamcop_sdi_card_power() inserts this
 * exact wait after every prescaler change; ported here for the same
 * reason (SAMCOP_CARD_INIT_CYCLES in samcop-sdi.h names the same 74).
 */
#define HAMCOP_SDI_INIT_CYCLES		74

/*
 * Bound, in SDCLK cycles, on waiting for SDIDCON.STOP or SDICON.FRST to read
 * back clear. Neither has a documented duration, but a write to the block
 * while one is pending can be lost: the SAMCOP twin of this block was seen
 * losing the next DCON write, with every direction and mode bit gone, when
 * STOP was still set, and only at slow dividers. The bound assumes the worst
 * a force-stop could plausibly wait out, one 512 byte block plus CRC on a
 * 1-bit bus, so a bit that is genuinely stuck is reported rather than waited
 * on for ever.
 */
#define HAMCOP_SDI_CTRL_SETTLE_CYCLES	((512 + 2) * 8 + HAMCOP_SDI_INIT_CYCLES)

/* The same bound in microseconds before a rate has been published. */
#define HAMCOP_SDI_CTRL_SETTLE_US_NOCLK	1000

/*
 * Bounds on the software backstop timer (host->req_timer). The MMC core's
 * own data->timeout_ns is the starting point, but it is derived from the
 * card's CSD and says nothing about the host block wedging, so it is
 * floored at a second (below that a merely slow card would be aborted) and
 * capped at twenty (above that a wedge is indistinguishable from the hang the
 * timer exists to break). The vendor bootloader waits about 15 seconds on every
 * command, so the cap leaves headroom above that.
 */
#define HAMCOP_SDI_REQ_TIMEOUT_MIN_MS	1000
#define HAMCOP_SDI_REQ_TIMEOUT_MAX_MS	20000

/*
 * The MMC core waits power_delay_ms (10 ms by default) after switching the
 * slot on before it sends the first command. The HP Photosmart SDIO camera is
 * only sometimes ready by then: on the h2200 about 4 of 10 scans after a slot
 * power cycle got no answer to CMD5 at all. Which delay helps is not known,
 * so it is a parameter that can be swept from the command line; 0 keeps the
 * core default. Takes effect at the next probe.
 */
static unsigned int hamcop_sdi_power_delay_ms;
module_param_named(power_delay_ms, hamcop_sdi_power_delay_ms, uint, 0644);
MODULE_PARM_DESC(power_delay_ms,
		 "Milliseconds to wait after slot power-up before the first command (0 = core default, 10 ms)");

/*
 * The same camera is missed by most first scans after a cold power-up (3 of 3
 * boots on the h2200), while a slot power cycle from userspace finds it in
 * about half of the attempts or more. When the first scan ends with the
 * card-detect line saying a card is present but no card bound, give the slot
 * a warm-up blip (rail on, then off) and scan again, up to scan_retries
 * times. The blip timings are parameters because the values that work best
 * are not known yet.
 */
static unsigned int hamcop_sdi_scan_retries = 3;
module_param_named(scan_retries, hamcop_sdi_scan_retries, uint, 0644);
MODULE_PARM_DESC(scan_retries,
		 "Rescans with a slot warm-up blip when a card is present but none was found (0 = off, default 3)");
static unsigned int hamcop_sdi_blip_on_ms = 1000;
module_param_named(blip_on_ms, hamcop_sdi_blip_on_ms, uint, 0644);
MODULE_PARM_DESC(blip_on_ms, "Milliseconds the slot rail stays on in a rescan's warm-up blip");
static unsigned int hamcop_sdi_blip_off_ms = 300;
module_param_named(blip_off_ms, hamcop_sdi_blip_off_ms, uint, 0644);
MODULE_PARM_DESC(blip_off_ms, "Milliseconds the slot rail stays off after the warm-up blip");

enum hamcop_sdi_waitfor {
	COMPLETION_NONE,
	COMPLETION_FINALIZE,
	COMPLETION_CMDSENT,
	COMPLETION_RSPFIN,
	COMPLETION_XFERFINISH,
	COMPLETION_XFERFINISH_RSPFIN,
};

enum hamcop_sdi_pio_dir {
	XFER_NONE,
	XFER_READ,
	XFER_WRITE,
};

struct hamcop_sdi_host {
	struct device		*dev;
	struct mmc_host		*mmc;
	struct delayed_work	scan_retry_work;
	unsigned int		scan_retries_done;
	bool			blip_next;
	void __iomem		*base;
	int			irq;
	struct clk		*clk;

	/*
	 * Guards every register access below CMDCON write time and all host
	 * state shared between hamcop_sdi_irq(), the tasklet and the submit
	 * path. hamcop_sdi_request() takes it and hamcop_sdi_send_request()
	 * runs entirely underneath it, so send_request() must never be called
	 * without it held;
	 * hamcop_sdi_finalize_request()'s stop-command recursion reaches it
	 * from the tasklet, which already holds it.
	 */
	spinlock_t		lock;

	struct mmc_request	*mrq;
	bool			cmd_is_stop;
	enum hamcop_sdi_waitfor	complete_what;

	enum hamcop_sdi_pio_dir	pio_active;
	unsigned int		pio_sgptr;
	unsigned int		pio_bytes;
	unsigned int		pio_count;
	u8			*pio_ptr;

	unsigned int		bus_width;
	unsigned long		clk_rate;
	unsigned long		real_rate;
	u8			prescaler;

	/*
	 * Shadow copies of SDIIMSK1/SDIIMSK2: reads cost as much as writes
	 * behind HAMCOP's static-memory window, and the PIO path updates the
	 * mask on every FIFO refill.
	 *
	 * A shadow is not self-synchronising: enable_imsk() and
	 * disable_imsk() are read-modify-writes of these fields, and the
	 * handlers clear them. Updating them is only safe under host->lock,
	 * which therefore covers the submit path as well as the handlers and
	 * the tasklet.
	 */
	u16			imsk1;
	u16			imsk2;

	/*
	 * Backstop for a data phase that never signals completion at all.
	 * Every other error this driver reports arrives as an interrupt, so
	 * a block that simply stops producing them would hang the MMC core's
	 * request wait forever with nothing in the log.
	 * Armed per data-carrying command in hamcop_sdi_send_command(),
	 * disarmed in hamcop_sdi_finalize_request().
	 */
	struct timer_list	req_timer;

	struct tasklet_struct	pio_tasklet;
};

static inline u16 hamcop_sdi_read(struct hamcop_sdi_host *host, u32 reg)
{
	return readw_relaxed(host->base + reg);
}

static inline void hamcop_sdi_write(struct hamcop_sdi_host *host, u32 reg, u16 val)
{
	writew_relaxed(val, host->base + reg);
}

/* These three update the host->imsk1/imsk2 shadows, so they need host->lock. */
static void hamcop_sdi_enable_imsk(struct hamcop_sdi_host *host, u16 imsk1, u16 imsk2)
{
	lockdep_assert_held(&host->lock);

	if (imsk1) {
		host->imsk1 |= imsk1;
		hamcop_sdi_write(host, HAMCOP_SDI_IMSK1, host->imsk1);
	}

	if (imsk2) {
		host->imsk2 |= imsk2;
		hamcop_sdi_write(host, HAMCOP_SDI_IMSK2, host->imsk2);
	}
}

/*
 * Drop selected mask bits without disturbing the rest. TXFIFOHALF and the
 * other FSTA thresholds are live occupancy levels, not latched status, so
 * the ISR cannot ack them -- masking is the only way to stop one asserting
 * once its consumer has gone away.
 */
static void hamcop_sdi_disable_imsk(struct hamcop_sdi_host *host, u16 imsk1, u16 imsk2)
{
	lockdep_assert_held(&host->lock);

	if (imsk1) {
		host->imsk1 &= ~imsk1;
		hamcop_sdi_write(host, HAMCOP_SDI_IMSK1, host->imsk1);
	}

	if (imsk2) {
		host->imsk2 &= ~imsk2;
		hamcop_sdi_write(host, HAMCOP_SDI_IMSK2, host->imsk2);
	}
}

static void hamcop_sdi_clear_imsk(struct hamcop_sdi_host *host)
{
	lockdep_assert_held(&host->lock);

	host->imsk1 = 0;
	host->imsk2 = 0;
	hamcop_sdi_write(host, HAMCOP_SDI_IMSK1, 0);
	hamcop_sdi_write(host, HAMCOP_SDI_IMSK2, 0);
}

static inline u16 hamcop_sdi_fifo_count(struct hamcop_sdi_host *host)
{
	return hamcop_sdi_read(host, HAMCOP_SDI_FSTA) & HAMCOP_SDIFSTA_COUNTMASK;
}

/*
 * Free space in the TX FIFO. The occupancy count reaches
 * HAMCOP_SDI_FIFO_DEPTH on hardware, so the subtraction has to saturate:
 * as u16 arithmetic it would wrap to 65535, and the write loop would push
 * a whole transfer into the FIFO in one burst and get a FIFO overrun.
 * Zero free space ends the fill loop and arms TXFIFOHALF so that the
 * interrupt handler resumes as the FIFO drains.
 */
static inline u16 hamcop_sdi_fifo_free(struct hamcop_sdi_host *host)
{
	u16 count = hamcop_sdi_fifo_count(host);

	if (count >= HAMCOP_SDI_FIFO_DEPTH - 1)
		return 0;

	return HAMCOP_SDI_FIFO_DEPTH - 1 - count;
}

/*
 * Wait @cycles SDCLK periods at the currently programmed prescaler.
 *
 * Every wait in this driver is denominated in SDCLK cycles rather than in
 * a wall-clock number picked by hand, because SDCLK is what clocks the
 * blocks being waited on and its rate moves by a factor of 64 between card
 * identification and full speed. Callers must run after the first
 * .set_ios() (i.e. after hamcop_sdi_set_clock() has published a non-zero
 * host->real_rate); the probe-time reset below does not wait,
 * having nothing in flight to wait for.
 */
static void hamcop_sdi_settle(struct hamcop_sdi_host *host, unsigned int cycles)
{
	udelay(DIV_ROUND_UP(cycles * 1000000UL, host->real_rate));
}

/*
 * Poll a self-clearing control bit (SDIDCON.STOP, SDICON.FRST) until it
 * reads back clear, bounded by HAMCOP_SDI_CTRL_SETTLE_CYCLES. The next write
 * to the block must not land while one is still pending. Normally this exits
 * on the first read.
 */
static void hamcop_sdi_wait_clear(struct hamcop_sdi_host *host, u32 reg,
				  u16 bit, const char *what)
{
	unsigned int bound_us;
	u16 val;

	bound_us = host->real_rate ?
		DIV_ROUND_UP_ULL((u64)HAMCOP_SDI_CTRL_SETTLE_CYCLES * USEC_PER_SEC,
				 host->real_rate) :
		HAMCOP_SDI_CTRL_SETTLE_US_NOCLK;

	if (read_poll_timeout_atomic(hamcop_sdi_read, val, !(val & bit), 1,
				     bound_us, false, host, reg))
		dev_err_ratelimited(host->dev,
				    "%s still set %u us after it was written (reg %#04x = %#06x, SDCLK %lu Hz)\n",
				    what, bound_us, reg, val, host->real_rate);
}

/*
 * Park the data engine the way a request that finished normally leaves it:
 * block count and DCON zeroed, nothing forced. This is all the vendor
 * bootloader does between requests, and none of the vendor drivers force-stop
 * a transfer that completed.
 */
static void hamcop_sdi_idle_data(struct hamcop_sdi_host *host)
{
	hamcop_sdi_write(host, HAMCOP_SDI_BNUM, 0);
	hamcop_sdi_write(host, HAMCOP_SDI_DCON, 0);
}

/*
 * Put the data engine back in its idle state: force-stop, then disarm. Only
 * for paths where a transfer may still be running: errors, timeouts,
 * recovery, probe and teardown. A request that completed normally goes
 * through hamcop_sdi_idle_data() instead.
 *
 * HAMCOP_SDIDCON bit 2 is SAMCOP's SDIDCON bit 14 (this driver's flags are
 * SAMCOP's shifted down by 12, see the file header). On the Samsung SDI
 * blocks this IP descends from that bit is StopDat on S3C2410 but DatStart
 * on S3C2440. The probe-time writable-bit readback (write 0xfffc to DCON,
 * read back 0x03f8) matches the S3C2410 control-bit layout bit for bit and
 * nothing else, so HAMCOP is the S3C2410 variant and writing bit 2 stops
 * the data engine rather than arming it.
 *
 * The stop is not instantaneous and a DCON write issued while it is still
 * pending is dropped, so wait for the bit to clear before the idle writes
 * and before whatever arms the next transfer.
 */
static void hamcop_sdi_stop_data(struct hamcop_sdi_host *host)
{
	hamcop_sdi_write(host, HAMCOP_SDI_DCON, HAMCOP_SDIDCON_STOP);
	hamcop_sdi_wait_clear(host, HAMCOP_SDI_DCON, HAMCOP_SDIDCON_STOP,
			      "SDIDCON.STOP");
	hamcop_sdi_idle_data(host);
}

/*
 * FIFORESET alone. The datasheet only says the bit clears itself; wait for it
 * anyway so the next write never races it.
 */
static void hamcop_sdi_fifo_reset(struct hamcop_sdi_host *host)
{
	hamcop_sdi_write(host, HAMCOP_SDI_CON,
			 hamcop_sdi_read(host, HAMCOP_SDI_CON) |
			  HAMCOP_SDICON_FIFORESET);
	hamcop_sdi_wait_clear(host, HAMCOP_SDI_CON, HAMCOP_SDICON_FIFORESET,
			      "SDICON.FRST");
}

/*
 * The above plus FIFORESET, for the paths that need the block genuinely
 * reset rather than merely disarmed: probe (which inherits whatever the
 * previous owner of this chip left behind), the stuck-data-line recovery in
 * hamcop_sdi_setup_data(), the request timeout, and finalizing a request
 * that failed.
 *
 * FIFORESET is the only reset this chip is known to have. It is what
 * stands in for the S3C2440-only SDRESET that s3cmci pairs with its own
 * force-stop in exactly this recovery loop (s3cmci_reset(), called from
 * s3cmci_setup_data()) -- a bit HAMCOP's recovered register documentation
 * does not describe, so it is not written blind here. FIFORESET itself is
 * known-safe on this part: the old out-of-tree hamcop_sdi_card_power() sets
 * it on every power-on.
 */
static void hamcop_sdi_reset_data(struct hamcop_sdi_host *host)
{
	hamcop_sdi_stop_data(host);
	hamcop_sdi_fifo_reset(host);
}

/* ---------------------------------------------------------------------
 * PIO
 * ---------------------------------------------------------------------
 */

static int hamcop_sdi_get_data_buffer(struct hamcop_sdi_host *host,
				      unsigned int *bytes, u8 **pointer)
{
	struct scatterlist *sg;

	if (host->pio_active == XFER_NONE)
		return -EINVAL;

	if (!host->mrq || !host->mrq->data)
		return -EINVAL;

	if (host->pio_sgptr >= host->mrq->data->sg_len)
		return -EBUSY;

	sg = &host->mrq->data->sg[host->pio_sgptr];
	*bytes = sg->length;
	*pointer = sg_virt(sg);
	host->pio_sgptr++;

	return 0;
}

static void hamcop_sdi_pio_read(struct hamcop_sdi_host *host)
{
	void __iomem *from = host->base + HAMCOP_SDI_DATA;
	unsigned int fifo;

	while ((fifo = hamcop_sdi_fifo_count(host))) {
		if (!host->pio_bytes &&
		    hamcop_sdi_get_data_buffer(host, &host->pio_bytes, &host->pio_ptr)) {
			host->pio_active = XFER_NONE;
			host->complete_what = COMPLETION_FINALIZE;
			return;
		}

		if (fifo > host->pio_bytes)
			fifo = host->pio_bytes;

		host->pio_bytes -= fifo;
		host->pio_count += fifo;

		while (fifo >= 2) {
			put_unaligned(readw_relaxed(from), (u16 *)host->pio_ptr);
			host->pio_ptr += 2;
			fifo -= 2;
		}
		if (fifo)
			*host->pio_ptr++ = (u8)readw_relaxed(from);
	}

	if (!host->pio_bytes &&
	    hamcop_sdi_get_data_buffer(host, &host->pio_bytes, &host->pio_ptr)) {
		host->pio_active = XFER_NONE;
		host->complete_what = COMPLETION_FINALIZE;
		return;
	}

	hamcop_sdi_enable_imsk(host, 0,
			       HAMCOP_SDIIMSK2_RXFIFOHALF | HAMCOP_SDIIMSK2_RXFIFOLAST);
}

static void hamcop_sdi_pio_write(struct hamcop_sdi_host *host)
{
	void __iomem *to = host->base + HAMCOP_SDI_DATA;
	unsigned int fifo;

	while ((fifo = hamcop_sdi_fifo_free(host)) > 1) {
		if (!host->pio_bytes &&
		    hamcop_sdi_get_data_buffer(host, &host->pio_bytes, &host->pio_ptr)) {
			/*
			 * Nothing left to feed the FIFO. TXFIFOHALF is a live
			 * occupancy level, so leaving it enabled here means it
			 * re-asserts as the card drains the residue, with no
			 * refill to answer it and no way for the ISR to ack it
			 * -- the SDI holds the demux line down until XFERFINISH
			 * and hamcop_irq_handler() burns its loop budget. Drop
			 * just this threshold; IMSK1's DATAFINISH, FIFOFAIL,
			 * DATACRC and DATATIMEOUT must stay enabled because the
			 * write is not finished, and XFERFINISH is exactly the
			 * completion this leg is now waiting for.
			 */
			hamcop_sdi_disable_imsk(host, 0, HAMCOP_SDIIMSK2_TXFIFOHALF);
			host->pio_active = XFER_NONE;
			return;
		}

		if (fifo > host->pio_bytes)
			fifo = host->pio_bytes;
		else
			fifo &= ~1U;

		host->pio_bytes -= fifo;
		host->pio_count += fifo;

		while (fifo >= 2) {
			writew_relaxed(get_unaligned((u16 *)host->pio_ptr), to);
			host->pio_ptr += 2;
			fifo -= 2;
		}

		/*
		 * Odd tail. Only the "fifo = host->pio_bytes" leg above can
		 * leave one byte here, and only for a single-block transfer
		 * with an odd block size -- which setup_data() explicitly
		 * permits (it rejects odd blksz only when blocks > 1). Those
		 * bytes were already charged to pio_bytes/pio_count, so
		 * dropping them silently desynchronises the driver's idea of
		 * the transfer from the block counter and strands the
		 * request. Pad into the low half of one FIFO write rather
		 * than reading a second byte off the end of the buffer, which
		 * is what the s3cmci idiom ("fifo = (fifo + 3) >> 2") would
		 * do here.
		 */
		if (fifo)
			writew_relaxed(*host->pio_ptr++, to);
	}

	hamcop_sdi_enable_imsk(host, 0, HAMCOP_SDIIMSK2_TXFIFOHALF);
}

static void hamcop_sdi_finalize_request(struct hamcop_sdi_host *host);
static void hamcop_sdi_send_request(struct mmc_host *mmc);

static void hamcop_sdi_tasklet(struct tasklet_struct *t)
{
	struct hamcop_sdi_host *host = from_tasklet(host, t, pio_tasklet);
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);

	if (host->complete_what == COMPLETION_FINALIZE) {
		hamcop_sdi_clear_imsk(host);

		if (host->pio_active != XFER_NONE) {
			dev_err(host->dev,
				"unfinished %s -- pio_count:%u pio_bytes:%u\n",
				host->pio_active == XFER_READ ? "read" : "write",
				host->pio_count, host->pio_bytes);
			if (host->mrq->data)
				host->mrq->data->error = -EIO;
			host->pio_active = XFER_NONE;
		}

		hamcop_sdi_finalize_request(host);
	}

	spin_unlock_irqrestore(&host->lock, flags);
}

/*
 * Defined further down the file, next to the interrupt handler it was
 * factored out of.
 */
static void hamcop_sdi_process_status(struct hamcop_sdi_host *host);

/*
 * host->req_timer's callback: a data phase produced no completion
 * interrupt at all inside its budget.
 *
 * Before declaring a timeout, ask the question the interrupt would have
 * answered: has the transfer actually finished? The completion logic of the
 * interrupt handler is run directly, reading the SDI block's own status
 * registers rather than relying on the interrupt controller.
 * DSTA.XFERFINISH remains the completion criterion, only polled instead of
 * signalled.
 *
 * If that finds no completion, the abort path reuses the interrupt handler's
 * own: put the block back in a known state, record the error on the
 * in-flight command, then hand the request to the tasklet
 * through COMPLETION_FINALIZE, exactly what hamcop_sdi_irq()'s close_transfer
 * leg does. It takes only host->lock and touches no mask bit beyond
 * hamcop_sdi_clear_imsk(), so it adds no new lock interaction with the ISR.
 */
static void hamcop_sdi_req_timeout(struct timer_list *t)
{
	struct hamcop_sdi_host *host = timer_container_of(host, t, req_timer);
	struct mmc_command *cmd;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);

	/*
	 * A finalize that reached the lock first has already cleared
	 * host->mrq. timer_delete() cannot cancel a callback that is
	 * already spinning here, so this check, not the delete, is what
	 * makes that race harmless.
	 */
	if (!host->mrq || host->complete_what == COMPLETION_NONE ||
	    host->complete_what == COMPLETION_FINALIZE)
		goto out;

	cmd = host->cmd_is_stop ? host->mrq->stop : host->mrq->cmd;

	/*
	 * Poll fallback. Run the completion logic a real XFERFINISH would
	 * have delivered.
	 */
	hamcop_sdi_process_status(host);

	if (host->complete_what == COMPLETION_FINALIZE) {
		dev_dbg(host->dev,
			"CMD%u completed by polling, the completion interrupt was not delivered\n",
			cmd->opcode);
		goto out;
	}

	dev_err(host->dev,
		"request timed out: CMD%u, pio_count:%u pio_bytes:%u DSTA %#06x FSTA %#06x DCON %#06x\n",
		cmd->opcode, host->pio_count, host->pio_bytes,
		hamcop_sdi_read(host, HAMCOP_SDI_DSTA),
		hamcop_sdi_read(host, HAMCOP_SDI_FSTA),
		hamcop_sdi_read(host, HAMCOP_SDI_DCON));
	hamcop_sdi_reset_data(host);

	/*
	 * Cleared before handing over so the tasklet's "unfinished
	 * transfer" leg does not overwrite this error with its own -EIO;
	 * the counts it would have reported are in the line above.
	 */
	host->pio_active = XFER_NONE;

	cmd->error = -ETIMEDOUT;
	if (cmd->data)
		cmd->data->error = -ETIMEDOUT;

	host->complete_what = COMPLETION_FINALIZE;
	hamcop_sdi_clear_imsk(host);
	tasklet_schedule(&host->pio_tasklet);

out:
	spin_unlock_irqrestore(&host->lock, flags);
}

/* ---------------------------------------------------------------------
 * Command/response state machine
 * ---------------------------------------------------------------------
 */

/*
 * The command/data-status processing of hamcop_sdi_irq(), factored out so
 * host->req_timer's callback can run the identical logic when the interrupt
 * never arrives. The SDI block's status registers are read directly, so if
 * the transfer finished CMDSTAT/DSTA/FSTA already say so.
 *
 * Caller holds host->lock.
 */
static void hamcop_sdi_process_status(struct hamcop_sdi_host *host)
{
	struct mmc_command *cmd;
	u16 csta, dsta, fsta;
	u16 cclear = 0, dclear = 0;

	csta = hamcop_sdi_read(host, HAMCOP_SDI_CMDSTAT);
	dsta = hamcop_sdi_read(host, HAMCOP_SDI_DSTA);
	fsta = hamcop_sdi_read(host, HAMCOP_SDI_FSTA);

	if (host->complete_what == COMPLETION_NONE ||
	    host->complete_what == COMPLETION_FINALIZE) {
		hamcop_sdi_clear_imsk(host);
		goto ack_and_out;
	}

	if (!host->mrq) {
		hamcop_sdi_clear_imsk(host);
		goto ack_and_out;
	}

	cmd = host->cmd_is_stop ? host->mrq->stop : host->mrq->cmd;
	if (!cmd) {
		hamcop_sdi_clear_imsk(host);
		goto ack_and_out;
	}

	if (csta & HAMCOP_SDICMDSTAT_CMDTIMEOUT) {
		cmd->error = -ETIMEDOUT;
		goto fail_transfer;
	}

	if (csta & HAMCOP_SDICMDSTAT_CMDSENT) {
		if (host->complete_what == COMPLETION_CMDSENT)
			goto close_transfer;
		cclear |= HAMCOP_SDICMDSTAT_CMDSENT;
	}

	if (csta & HAMCOP_SDICMDSTAT_CRCFAIL) {
		/*
		 * Same fixup s3cmci/samcop_sdi carry: this SDI IP generation
		 * misreports CRC failure for long (136-bit) responses --
		 * CMD2/CMD9/CMD10 -- so don't fail the command over it.
		 */
		if ((cmd->flags & MMC_RSP_CRC) && !(cmd->flags & MMC_RSP_136)) {
			cmd->error = -EILSEQ;
			goto fail_transfer;
		}
		cclear |= HAMCOP_SDICMDSTAT_CRCFAIL;
	}

	if (csta & HAMCOP_SDICMDSTAT_RSPFIN) {
		if (host->complete_what == COMPLETION_RSPFIN)
			goto close_transfer;
		if (host->complete_what == COMPLETION_XFERFINISH_RSPFIN)
			host->complete_what = COMPLETION_XFERFINISH;
		cclear |= HAMCOP_SDICMDSTAT_RSPFIN;
	}

	if (!cmd->data)
		goto clear_status;

	if (dsta & HAMCOP_SDIDSTA_FIFOFAIL) {
		cmd->data->error = -EILSEQ;
		goto fail_transfer;
	}

	if (dsta & (HAMCOP_SDIDSTA_RXCRCFAIL | HAMCOP_SDIDSTA_CRCFAIL)) {
		cmd->data->error = -EILSEQ;
		goto fail_transfer;
	}

	if (dsta & HAMCOP_SDIDSTA_DATATIMEOUT) {
		cmd->data->error = -ETIMEDOUT;
		goto fail_transfer;
	}

	/*
	 * Refill the FIFO from the handler rather than a tasklet: the
	 * half-FIFO interrupts only move 32 bytes at a time, so the round
	 * trip through softirq dispatch dominated the transfer time.
	 *
	 * This has to sit below the fault checks, which end at
	 * fail_transfer and park pio_active at XFER_NONE so that no more
	 * data is moved for a failed transfer, and above the XFERFINISH
	 * check, so a completed transfer is not topped up.
	 */
	if (host->pio_active == XFER_WRITE && (fsta & HAMCOP_SDIFSTA_TFDET))
		hamcop_sdi_pio_write(host);
	else if (host->pio_active == XFER_READ && (fsta & HAMCOP_SDIFSTA_RFDET))
		hamcop_sdi_pio_read(host);

	/*
	 * Finalizing calls back into the MMC core, so it stays in the
	 * tasklet.
	 */
	if (host->complete_what == COMPLETION_FINALIZE) {
		hamcop_sdi_clear_imsk(host);
		tasklet_schedule(&host->pio_tasklet);
		goto clear_status;
	}

	if (dsta & HAMCOP_SDIDSTA_XFERFINISH) {
		if (host->complete_what == COMPLETION_XFERFINISH)
			goto close_transfer;
		if (host->complete_what == COMPLETION_XFERFINISH_RSPFIN)
			host->complete_what = COMPLETION_RSPFIN;
		dclear |= HAMCOP_SDIDSTA_XFERFINISH;
	}

clear_status:
	hamcop_sdi_write(host, HAMCOP_SDI_CMDSTAT, cclear);
	hamcop_sdi_write(host, HAMCOP_SDI_DSTA, dclear);
	return;

fail_transfer:
	host->pio_active = XFER_NONE;

close_transfer:
	host->complete_what = COMPLETION_FINALIZE;
	hamcop_sdi_clear_imsk(host);
	tasklet_schedule(&host->pio_tasklet);

ack_and_out:
	/*
	 * Ack everything this entry observed.
	 *
	 * The paths that reach here, the early exits for an interrupt with
	 * nothing to do and the close_transfer abort and finish leg, must
	 * not return with CMDSTAT and DSTA untouched. Zeroing IMSK1 and
	 * IMSK2 does not drop the line on this IP family: the SAMCOP twin
	 * of this driver had the same gap and saw the same status value on
	 * some 110 consecutive handler entries, until the MFD demux gave up
	 * and masked the source. The HAMCOP demux cannot ack it either:
	 * every hwirq is handle_edge_irq() and hamcop_irq_ack() clears the
	 * pending latch of the interrupt controller, not the status
	 * registers of the SDI block.
	 *
	 * Acking wholesale is safe on exactly these paths, as none of them
	 * leaves a state machine transition owed to a bit read in this
	 * entry. The clear_status path above acks only the bits it
	 * consumed, because there the request is still live.
	 *
	 * FSTA is not written: its TFDET and RFDET bits are
	 * live occupancy detectors, not latched events.
	 */
	hamcop_sdi_write(host, HAMCOP_SDI_CMDSTAT, csta);
	hamcop_sdi_write(host, HAMCOP_SDI_DSTA, dsta);
}

static irqreturn_t hamcop_sdi_irq(int irq, void *dev_id)
{
	struct hamcop_sdi_host *host = dev_id;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	hamcop_sdi_process_status(host);
	spin_unlock_irqrestore(&host->lock, flags);

	return IRQ_HANDLED;
}

static void hamcop_sdi_finalize_request(struct hamcop_sdi_host *host)
{
	struct mmc_request *mrq = host->mrq;
	struct mmc_command *cmd;

	if (host->complete_what != COMPLETION_FINALIZE || !mrq)
		return;

	/*
	 * Disarm the backstop. timer_delete(), not timer_delete_sync():
	 * this runs from the tasklet with host->lock held, and the callback
	 * takes that same lock, so a synchronous delete would deadlock
	 * against a callback already spinning on it. The callback's own
	 * host->mrq check makes the non-synchronous delete safe -- see
	 * hamcop_sdi_req_timeout().
	 */
	timer_delete(&host->req_timer);

	cmd = host->cmd_is_stop ? mrq->stop : mrq->cmd;

	cmd->resp[0] = (hamcop_sdi_read(host, HAMCOP_SDI_RSP0H) << 16) |
			hamcop_sdi_read(host, HAMCOP_SDI_RSP0L);
	cmd->resp[1] = (hamcop_sdi_read(host, HAMCOP_SDI_RSP1H) << 16) |
			hamcop_sdi_read(host, HAMCOP_SDI_RSP1L);
	cmd->resp[2] = (hamcop_sdi_read(host, HAMCOP_SDI_RSP2H) << 16) |
			hamcop_sdi_read(host, HAMCOP_SDI_RSP2L);
	cmd->resp[3] = (hamcop_sdi_read(host, HAMCOP_SDI_RSP3H) << 16) |
			hamcop_sdi_read(host, HAMCOP_SDI_RSP3L);

	hamcop_sdi_write(host, HAMCOP_SDI_CMDARGH, 0);
	hamcop_sdi_write(host, HAMCOP_SDI_CMDARGL, 0);

	if (cmd->data && cmd->error)
		cmd->data->error = cmd->error;

	/*
	 * Force-stop and FIFO-reset only a request that failed, where the data
	 * engine may still be running; a completed one is parked the way the
	 * vendor bootloader parks it. A still-pending force-stop is what can
	 * swallow the DCON write of the next request.
	 */
	if (cmd->error || (cmd->data && cmd->data->error))
		hamcop_sdi_reset_data(host);
	else
		hamcop_sdi_idle_data(host);

	hamcop_sdi_write(host, HAMCOP_SDI_CMDCON, 0);
	hamcop_sdi_clear_imsk(host);

	if (cmd->data && cmd->data->stop && !host->cmd_is_stop) {
		host->cmd_is_stop = true;
		hamcop_sdi_send_request(host->mmc);
		return;
	}

	if (!mrq->data)
		goto request_done;

	if (mrq->data->error == 0)
		mrq->data->bytes_xfered = mrq->data->blocks * mrq->data->blksz;
	else
		mrq->data->bytes_xfered = 0;

request_done:
	host->complete_what = COMPLETION_NONE;
	host->mrq = NULL;
	mmc_request_done(host->mmc, mrq);
}

static void hamcop_sdi_send_command(struct hamcop_sdi_host *host, struct mmc_command *cmd)
{
	unsigned long ms;
	u16 ccon;

	hamcop_sdi_enable_imsk(host,
			       HAMCOP_SDIIMSK1_CRCSTATUS | HAMCOP_SDIIMSK1_CMDTIMEOUT |
				HAMCOP_SDIIMSK1_RESPONSEND | HAMCOP_SDIIMSK1_CMDSENT |
				HAMCOP_SDIIMSK1_RESPONSECRC, 0);

	if (cmd->data)
		host->complete_what = COMPLETION_XFERFINISH_RSPFIN;
	else if (cmd->flags & MMC_RSP_PRESENT)
		host->complete_what = COMPLETION_RSPFIN;
	else
		host->complete_what = COMPLETION_CMDSENT;

	hamcop_sdi_write(host, HAMCOP_SDI_CMDARGH, cmd->arg >> 16);
	hamcop_sdi_write(host, HAMCOP_SDI_CMDARGL, cmd->arg & 0xffff);

	ccon = cmd->opcode & HAMCOP_SDICMDCON_INDEX;
	ccon |= HAMCOP_SDICMDCON_SENDERHOST | HAMCOP_SDICMDCON_CMDSTART;
	if (cmd->flags & MMC_RSP_PRESENT)
		ccon |= HAMCOP_SDICMDCON_WAITRSP;
	if (cmd->flags & MMC_RSP_136)
		ccon |= HAMCOP_SDICMDCON_LONGRSP;

	hamcop_sdi_write(host, HAMCOP_SDI_CMDCON, ccon);

	/*
	 * Arm the backstop for every command, not only for ones with a data
	 * phase. The hardware command timeout raises an interrupt through the
	 * same chained demux as everything else, so its completion can go
	 * missing just as well, and a stop or status command without a timer
	 * would then hang the request forever. See host->req_timer.
	 */
	ms = HAMCOP_SDI_REQ_TIMEOUT_MIN_MS;
	if (cmd->data)
		ms = clamp_t(unsigned long, cmd->data->timeout_ns / NSEC_PER_MSEC,
			     HAMCOP_SDI_REQ_TIMEOUT_MIN_MS,
			     HAMCOP_SDI_REQ_TIMEOUT_MAX_MS);

	mod_timer(&host->req_timer, jiffies + msecs_to_jiffies(ms));
}

static int hamcop_sdi_setup_data(struct hamcop_sdi_host *host, struct mmc_data *data)
{
	u16 dcon, dsta;
	int retries = 3;

	/* The FIFO port moves 2 bytes/access -- a multi-block transfer
	 * whose block size isn't even can't be walked cleanly.
	 */
	if ((data->blksz & 1) && data->blocks > 1) {
		dev_warn(host->dev, "odd block size %u unsupported for multi-block transfers\n",
			 data->blksz);
		return -EINVAL;
	}

	while ((dsta = hamcop_sdi_read(host, HAMCOP_SDI_DSTA)) &
	       (HAMCOP_SDIDSTA_TXDATAON | HAMCOP_SDIDSTA_RXDATAON)) {
		if (retries-- == 0) {
			/* Dump what is actually stuck rather than just
			 * saying that something is -- this is the one place
			 * the driver gets to see the data engine wedged, and
			 * s3cmci_setup_data() spends its own last retry on a
			 * register dump for the same reason.
			 */
			dev_err(host->dev,
				"data line stuck busy, refusing new transfer (DSTA %#06x DCON %#06x BNUM %#06x FSTA %#06x CON %#06x PRE %#06x)\n",
				dsta,
				hamcop_sdi_read(host, HAMCOP_SDI_DCON),
				hamcop_sdi_read(host, HAMCOP_SDI_BNUM),
				hamcop_sdi_read(host, HAMCOP_SDI_FSTA),
				hamcop_sdi_read(host, HAMCOP_SDI_CON),
				hamcop_sdi_read(host, HAMCOP_SDI_PRE));
			return -EBUSY;
		}

		hamcop_sdi_reset_data(host);

		/*
		 * Then give the engine real SDCLK edges to act on before
		 * looking again. Without a wait here this loop re-read DSTA a
		 * couple of hundred nanoseconds after the write -- all three
		 * iterations of it fit inside a single SDCLK period at the
		 * 400kHz the core runs card identification at (2.5us) -- so it
		 * could only ever observe the same pre-stop value three times
		 * over and report a stuck data line that was merely mid-stop.
		 *
		 * HAMCOP_SDI_INIT_CYCLES is reused as the settle length for
		 * the same reason hamcop_sdi_set_clock() uses it: it is the
		 * one figure this project has for how long this block needs
		 * after a control-register write, and here it is a ceiling on
		 * a loop that exits on its next read in the normal case, not
		 * a fixed cost.
		 */
		hamcop_sdi_settle(host, HAMCOP_SDI_INIT_CYCLES);
	}

	dcon = HAMCOP_SDIDCON_BLOCKMODE;
	if (host->bus_width == MMC_BUS_WIDTH_4)
		dcon |= HAMCOP_SDIDCON_WIDEBUS;

	if (data->flags & MMC_DATA_WRITE)
		dcon |= HAMCOP_SDIDCON_TXAFTERRESP | HAMCOP_SDIDCON_XFER_TXSTART;
	if (data->flags & MMC_DATA_READ)
		dcon |= HAMCOP_SDIDCON_RXAFTERCMD | HAMCOP_SDIDCON_XFER_RXSTART;

	/*
	 * Reset the FIFO before every data phase, not just on the recovery
	 * path above. Residue from the previous transfer (an aborted read's
	 * tail, a write the card never drained) would otherwise prefix this
	 * one's data, and there is no other point where the block is put
	 * back into a known state on a successful request.
	 *
	 * FIFORESET only, without a force-stop: the previous request was
	 * either parked by hamcop_sdi_idle_data() after completing or reset in
	 * full after failing, so there is no transfer to stop here.
	 */
	hamcop_sdi_fifo_reset(host);

	/* Timer, block size and count, then DCON, in the datasheet's order. */
	hamcop_sdi_write(host, HAMCOP_SDI_TIMER1, HAMCOP_SDI_TIMER_MAX >> 16);
	hamcop_sdi_write(host, HAMCOP_SDI_TIMER2, HAMCOP_SDI_TIMER_MAX & 0xffff);
	hamcop_sdi_write(host, HAMCOP_SDI_BSIZE, data->blksz);
	hamcop_sdi_write(host, HAMCOP_SDI_BNUM, data->blocks & HAMCOP_SDI_BNUM_MASK);
	hamcop_sdi_write(host, HAMCOP_SDI_DCON, dcon);

	hamcop_sdi_enable_imsk(host,
			       HAMCOP_SDIIMSK1_FIFOFAIL | HAMCOP_SDIIMSK1_DATACRC |
				HAMCOP_SDIIMSK1_DATATIMEOUT | HAMCOP_SDIIMSK1_DATAFINISH, 0);

	return 0;
}

static void hamcop_sdi_prepare_pio(struct hamcop_sdi_host *host, struct mmc_data *data)
{
	host->pio_sgptr = 0;
	host->pio_bytes = 0;
	host->pio_count = 0;

	if (data->flags & MMC_DATA_WRITE) {
		host->pio_active = XFER_WRITE;
		hamcop_sdi_pio_write(host);
	} else {
		host->pio_active = XFER_READ;
		hamcop_sdi_enable_imsk(host, 0,
				       HAMCOP_SDIIMSK2_RXFIFOHALF |
					HAMCOP_SDIIMSK2_RXFIFOLAST);
	}
}

/*
 * Runs with host->lock held, from hamcop_sdi_request() or from
 * hamcop_sdi_finalize_request()'s stop-command recursion (which is itself
 * called from the tasklet, already holding it), so it must never take the
 * lock itself. Nothing below here sleeps: the only delay on the path is
 * hamcop_sdi_settle()'s udelay().
 */
static void hamcop_sdi_send_request(struct mmc_host *mmc)
{
	struct hamcop_sdi_host *host = mmc_priv(mmc);
	struct mmc_request *mrq = host->mrq;
	struct mmc_command *cmd = host->cmd_is_stop ? mrq->stop : mrq->cmd;

	lockdep_assert_held(&host->lock);

	hamcop_sdi_write(host, HAMCOP_SDI_CMDSTAT, 0xffff);
	hamcop_sdi_write(host, HAMCOP_SDI_DSTA, 0xffff);
	hamcop_sdi_write(host, HAMCOP_SDI_FSTA, 0xffff);

	if (cmd->data) {
		int ret = hamcop_sdi_setup_data(host, cmd->data);

		if (ret) {
			cmd->error = ret;
			cmd->data->error = ret;
			/* Nothing has been armed yet, so there is no
			 * completion to wait for and no interrupt source to
			 * leave enabled -- and, as on the -ENOMEDIUM path in
			 * hamcop_sdi_request(), the core owns (and may free)
			 * mrq again the moment mmc_request_done() returns, so
			 * the host must stop pointing at it. Only
			 * hamcop_sdi_finalize_request() used to do that, and
			 * this path never reaches it.
			 */
			host->mrq = NULL;
			host->complete_what = COMPLETION_NONE;
			hamcop_sdi_clear_imsk(host);
			mmc_request_done(mmc, mrq);
			return;
		}

		hamcop_sdi_prepare_pio(host, cmd->data);
	}

	hamcop_sdi_send_command(host, cmd);
}

static void hamcop_sdi_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct hamcop_sdi_host *host = mmc_priv(mmc);
	unsigned long flags;
	bool absent;

	/*
	 * Card-detect first, and outside the lock: mmc_gpio_get_cd() may
	 * sleep when the GPIO controller does, so it must not run in atomic
	 * context.
	 */
	absent = mmc_gpio_get_cd(mmc) == 0;

	/*
	 * Everything from here down is shared with hamcop_sdi_irq() and the
	 * tasklet: host->mrq, host->complete_what, the PIO state and, most
	 * importantly, the host->imsk1/imsk2 shadows, which the submit path
	 * read-modify-writes while a handler may be zeroing them.
	 */
	spin_lock_irqsave(&host->lock, flags);

	host->cmd_is_stop = false;
	host->mrq = mrq;

	if (absent) {
		/* Drop the reference before handing the mrq back: the core
		 * owns (and may free) it again the moment mmc_request_done()
		 * returns, and nothing else on this path clears host->mrq the
		 * way hamcop_sdi_finalize_request() does.
		 */
		host->mrq = NULL;
		host->complete_what = COMPLETION_NONE;
		mrq->cmd->error = -ENOMEDIUM;
	} else {
		hamcop_sdi_send_request(mmc);
	}

	spin_unlock_irqrestore(&host->lock, flags);

	if (absent)
		mmc_request_done(mmc, mrq);
}

static void hamcop_sdi_set_clock(struct hamcop_sdi_host *host, unsigned int clock)
{
	unsigned int psc;

	if (clock) {
		psc = host->clk_rate / clock;
		psc = psc ? psc - 1 : 0;
		if (host->clk_rate / (psc + 1) > clock)
			psc++;
		if (psc > 255)
			psc = 255;
	} else {
		/*
		 * clock == 0 is the core asking for SDCLK to *stop*, not for
		 * "any frequency" -- mmc_power_up()'s own comment is explicit
		 * that the card is powered with the clock off first and that
		 * "we must _NOT_ enable the clock prior to power stablising",
		 * and both legs of every power cycle (MMC_POWER_UP via
		 * mmc_set_initial_state(), MMC_POWER_OFF directly) pass it.
		 * HAMCOP has no SDCLK gate to honour that with -- CON bit 0 is
		 * the SD-vs-MMC protocol select here, not SAMCOP/S3C2410's
		 * clock enable (see its comment above) -- so the slowest
		 * divider is the closest this controller can get. Letting psc
		 * default to 0 instead runs SDCLK at the full gclk rate (tens
		 * of MHz on PXA25x) straight across the card's VDD ramp, which
		 * leaves cards unable to finish their internal power-up: ACMD41
		 * then answers every time but never sets OCR bit 31, until
		 * __mmc_poll_for_busy() gives up with "Card stuck being busy!"
		 * and -ETIMEDOUT. s3cmci, the same IP's modern reference, ends
		 * up at 255 here too -- its search loop simply runs out, since
		 * no divider makes real_rate <= 0.
		 */
		psc = 255;
	}

	host->prescaler = psc;
	host->real_rate = host->clk_rate / (psc + 1);
	hamcop_sdi_write(host, HAMCOP_SDI_PRE, host->prescaler);

	/* HAMCOP_SDI_INIT_CYCLES SDCLKs at the new rate. Matches
	 * the old out-of-tree hamcop_sdi_card_power(), which runs this same
	 * wait unconditionally after every prescaler change, not just the
	 * first one after power-up.
	 */
	hamcop_sdi_settle(host, HAMCOP_SDI_INIT_CYCLES);
}

/*
 * Slot power rail: the "vmmc" regulator, if the node has one.
 *
 * The SD bus pull-ups move with the rail, matching the old out-of-tree
 * hamcop_sdi_card_power(): pull-ups
 * present while the slot is powered (SPUCR disable bits cleared), removed
 * again when it is not, so an empty slot doesn't leak current through
 * them. See include/linux/mfd/hamcop.h's SPUCR comment.
 */
static void hamcop_sdi_set_power(struct hamcop_sdi_host *host, bool on,
				 unsigned char vdd)
{
	struct mmc_host *mmc = host->mmc;
	u16 sd_pullups = HAMCOP_GPIO_SPUCR_PU_SDCM | HAMCOP_GPIO_SPUCR_PU_SDDA;

	if (!IS_ERR(mmc->supply.vmmc))
		mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, on ? vdd : 0);

	/* Runs even without a supply: a board whose rail is hardwired on
	 * still needs its pull-ups managed.
	 */
	hamcop_set_spucr(host->dev->parent, sd_pullups, on ? 0 : sd_pullups);
}

static void hamcop_sdi_scan_retry_work(struct work_struct *work)
{
	struct hamcop_sdi_host *host = container_of(to_delayed_work(work),
						    struct hamcop_sdi_host,
						    scan_retry_work);
	struct mmc_host *mmc = host->mmc;
	unsigned int max = READ_ONCE(hamcop_sdi_scan_retries);

	if (host->scan_retries_done >= max || mmc_gpio_get_cd(mmc) == 0 ||
	    READ_ONCE(mmc->card))
		return;

	host->scan_retries_done++;
	dev_info(host->dev, "no card found, rescanning (%u of %u)\n",
		 host->scan_retries_done, max);
	/* the blip runs in set_ios(), where the core has claimed the host */
	WRITE_ONCE(host->blip_next, true);
	mmc_detect_change(mmc, 0);
	schedule_delayed_work(&host->scan_retry_work, msecs_to_jiffies(5000));
}

static void hamcop_sdi_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct hamcop_sdi_host *host = mmc_priv(mmc);

	switch (ios->power_mode) {
	case MMC_POWER_ON:
	case MMC_POWER_UP:
		if (ios->power_mode == MMC_POWER_UP &&
		    READ_ONCE(host->blip_next)) {
			WRITE_ONCE(host->blip_next, false);
			hamcop_sdi_set_power(host, true, ios->vdd);
			msleep(READ_ONCE(hamcop_sdi_blip_on_ms));
			hamcop_sdi_set_power(host, false, 0);
			msleep(READ_ONCE(hamcop_sdi_blip_off_ms));
		}
		/* Read-modify-write, matching the old out-of-tree
		 * hamcop_sdi_card_power() -- CON isn't fully understood
		 * (see the bitfield comment above), so avoid clobbering any
		 * bit this driver doesn't know about.
		 */
		hamcop_sdi_write(host, HAMCOP_SDI_CON,
				 (hamcop_sdi_read(host, HAMCOP_SDI_CON) &
				   ~HAMCOP_SDICON_CLOCKTYPE_MMC) | HAMCOP_SDICON_FIFORESET);
		hamcop_sdi_set_power(host, true, ios->vdd);
		break;
	case MMC_POWER_OFF:
	default:
		hamcop_sdi_set_power(host, false, 0);
		break;
	}

	/* ios->clock may be 0 (host->real_rate then saturates to
	 * clk_rate/256, the slowest this controller can go) -- it is
	 * not special-cased into an explicit clock-stop: HAMCOP_SDI_CON bit
	 * 0 isn't an SDCLK gate on this chip (see its comment above), and
	 * "setting 0Hz as any other frequency works" is explicitly sanctioned
	 * for such controllers by struct mmc_host_ops's own set_ios doc
	 * comment.
	 */
	hamcop_sdi_set_clock(host, ios->clock);

	host->bus_width = ios->bus_width;
}

static const struct mmc_host_ops hamcop_sdi_ops = {
	.request	= hamcop_sdi_request,
	.set_ios	= hamcop_sdi_set_ios,
	.get_cd		= mmc_gpio_get_cd,
	.get_ro		= mmc_gpio_get_ro,
};

/* ---------------------------------------------------------------------
 * Probe/lifecycle
 * ---------------------------------------------------------------------
 */

static int hamcop_sdi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mmc_host *mmc;
	struct hamcop_sdi_host *host;
	struct resource *mem;
	int ret, irq;

	mmc = devm_mmc_alloc_host(dev, sizeof(*host));
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	host->dev = dev;
	host->mmc = mmc;
	spin_lock_init(&host->lock);
	tasklet_setup(&host->pio_tasklet, hamcop_sdi_tasklet);
	timer_setup(&host->req_timer, hamcop_sdi_req_timeout, 0);

	/*
	 * Not devm_ioremap_resource(): it reserves the region exclusively.
	 * This resource is a sub-range of HAMCOP's own register window, which
	 * drivers/mfd/hamcop.c already maps exclusively before it registers
	 * this cell, so an exclusive claim here always fails with -EBUSY.
	 * devm_ioremap() maps the range without reserving it.
	 */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -ENODEV;
	host->base = devm_ioremap(dev, mem->start, resource_size(mem));
	if (!host->base)
		return -ENOMEM;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	host->irq = irq;

	/* "hamcop-sdi"/NULL clkdev alias onto the "hamcop-sd" ClockControl
	 * gate -- registered by drivers/mfd/hamcop.c's
	 * hamcop_clocks_register(). Its parent, "hamcop-gclk", already
	 * implements the old out-of-tree hamcop_gclk() formula exactly
	 * (mclk or mclk/2, from HAMCOP's own ClockSleep CLKSEL bit), so
	 * clk_get_rate() here *is* what the 2007 driver called gclk --
	 * no separate divider constant is needed on this driver's side.
	 */
	host->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(host->clk))
		return dev_err_probe(dev, PTR_ERR(host->clk), "failed to get sdi clock\n");

	host->clk_rate = clk_get_rate(host->clk);
	if (!host->clk_rate)
		return dev_err_probe(dev, -EINVAL, "sdi clock reports 0 Hz\n");

	/*
	 * Take the block over from whoever had it last, before mmc_add_host()
	 * starts issuing commands into it.
	 *
	 * This matters on h2200 specifically: the device's own bootloader
	 * (LAB) has a full, working driver for this same SDI block and uses
	 * it to read the SD card's partition table, and the normal way to
	 * reach this kernel is a warm jump out of LAB with no intervening
	 * chip reset. Only CON was being written here, so DCON, BNUM, BSIZE,
	 * TIMER, the interrupt masks and every status register entered Linux
	 * holding whatever the previous owner left in them -- and this
	 * driver's first DCON write did not happen until the first
	 * hamcop_sdi_finalize_request(), i.e. after CMD0 had already been
	 * sent into a data engine that might still be armed. A host driver
	 * inheriting an unknown data-engine state is a defect regardless of
	 * how it got there.
	 *
	 * CON is written whole rather than read-modify-written, so the
	 * SD-vs-MMC protocol select lands in a known state too; from here on
	 * hamcop_sdi_set_ios() preserves the rest of the register.
	 *
	 * No settle wait: nothing is in flight to wait for, host->real_rate
	 * is not published until the first .set_ios(), and the MMC core's own
	 * power-up delays follow immediately.
	 *
	 * Under host->lock only to satisfy the lockdep assertion in
	 * hamcop_sdi_clear_imsk(): no interrupt handler is registered yet.
	 */
	spin_lock_irq(&host->lock);
	hamcop_sdi_write(host, HAMCOP_SDI_CON, HAMCOP_SDICON_CLOCKTYPE_SD);
	hamcop_sdi_clear_imsk(host);
	hamcop_sdi_reset_data(host);
	hamcop_sdi_write(host, HAMCOP_SDI_CMDCON, 0);
	hamcop_sdi_write(host, HAMCOP_SDI_CMDSTAT, 0xffff);
	hamcop_sdi_write(host, HAMCOP_SDI_DSTA, 0xffff);
	hamcop_sdi_write(host, HAMCOP_SDI_FSTA, 0xffff);
	spin_unlock_irq(&host->lock);

	/*
	 * Card detect comes from the optional "cd-gpios" property. Without
	 * one, fall back to polling rather than fail probe.
	 *
	 * Debounce for 25 ms, as the vendor bootloader does before rescanning
	 * after a card-detect edge. The argument is in microseconds.
	 */
	ret = mmc_gpiod_request_cd(mmc, "cd", 0, false, 25000);
	if (ret == -ENOENT)
		mmc->caps |= MMC_CAP_NEEDS_POLL;
	else if (ret)
		return dev_err_probe(dev, ret, "failed to get card-detect gpio\n");

	ret = mmc_gpiod_request_ro(mmc, "wp", 0, 0);
	if (ret && ret != -ENOENT)
		return dev_err_probe(dev, ret, "failed to get write-protect gpio\n");

	/*
	 * Slot power. mmc_regulator_get_supply() uses
	 * devm_regulator_get_optional() (drivers/mmc/core/regulator.c), which
	 * never substitutes the dummy regulator, so a board that hardwires
	 * the rail on and describes no "vmmc-supply" still probes: the
	 * supply stays an error pointer and hamcop_sdi_set_power() skips it.
	 */
	ret = mmc_regulator_get_supply(mmc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get vmmc supply\n");

	mmc->ops = &hamcop_sdi_ops;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps |= MMC_CAP_4_BIT_DATA;
	if (hamcop_sdi_power_delay_ms)
		mmc->ios.power_delay_ms = hamcop_sdi_power_delay_ms;
	mmc->f_min = host->clk_rate / 256;
	mmc->f_max = host->clk_rate;
	mmc->max_blk_count = HAMCOP_SDI_BNUM_MASK;
	mmc->max_blk_size = 4095;
	mmc->max_req_size = HAMCOP_SDI_BNUM_MASK * 512;
	mmc->max_seg_size = mmc->max_req_size;
	mmc->max_segs = 128;

	ret = devm_request_irq(dev, host->irq, hamcop_sdi_irq, 0, "hamcop-sdi", host);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request irq\n");

	INIT_DELAYED_WORK(&host->scan_retry_work, hamcop_sdi_scan_retry_work);

	ret = mmc_add_host(mmc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add mmc host\n");
	schedule_delayed_work(&host->scan_retry_work, msecs_to_jiffies(6000));

	platform_set_drvdata(pdev, host);

	dev_info(dev, "HAMCOP SD/MMC host ready (PIO%s)\n",
		 mmc->caps & MMC_CAP_NEEDS_POLL ? ", polled card-detect" : "");

	return 0;
}

static void hamcop_sdi_remove(struct platform_device *pdev)
{
	struct hamcop_sdi_host *host = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&host->scan_retry_work);
	mmc_remove_host(host->mmc);

	/* Process context here, so the synchronous delete is both safe and
	 * the right one: nothing may re-enter the callback after this.
	 */
	timer_delete_sync(&host->req_timer);
	tasklet_kill(&host->pio_tasklet);
}

static const struct of_device_id hamcop_sdi_of_match[] = {
	{ .compatible = "samsung,hamcop-sdi" },
	{ }
};
MODULE_DEVICE_TABLE(of, hamcop_sdi_of_match);

static struct platform_driver hamcop_sdi_driver = {
	.driver = {
		.name = "hamcop-sdi",
		.of_match_table = hamcop_sdi_of_match,
	},
	.probe	= hamcop_sdi_probe,
	.remove	= hamcop_sdi_remove,
};
module_platform_driver(hamcop_sdi_driver);

MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("HAMCOP companion ASIC SD/MMC host driver (HP iPAQ h2200)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:hamcop-sdi");
