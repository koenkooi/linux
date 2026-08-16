// SPDX-License-Identifier: GPL-2.0-only
/*
 * MMC host driver for the SDI (SD/MMC) block on the Samsung SAMCOP
 * companion ASIC (HP iPAQ h51xx/h54xx/h55xx). This is the physical SD
 * card slot on these boards; the PXA25x SoC's own MMC controller is
 * unwired, as on the HAMCOP-backed slot of the h2200.
 *
 * PIO only. The block's DMA channel can only reach SAMCOP's 32 KiB
 * on-chip SRAM, which the samcop-ohci cell claims whole through
 * usb_hcd_setup_local_mem(). The PXA DMA controller cannot substitute: it
 * does flow-controlled peripheral DMA only for on-SoC requestors with a
 * DRCMR request line, SAMCOP sits behind a static chip select with no
 * handshake pin, and DMA_MEMCPY into the FIFO polled on FSTA would be
 * PIO with extra setup latency, the 16-bit VLIO bus cycle being the
 * bottleneck.
 *
 * Unlike HAMCOP, which halves every SDI register to 16 bits and
 * reshuffles some bitfields, SAMCOP keeps the block at its native 32-bit
 * width and stride: SDICON at 0x00 through SDIIMSK at 0x40, see
 * SAMCOP_SDI_SIZE in include/linux/mfd/samcop.h. The block is the
 * S3C2410-generation Samsung SDI IP, so the removed
 * drivers/mmc/host/s3cmci.c (last in 0b14558977a7^) is the reference for
 * its behaviour:
 *
 *  - SDICON bit 0 (CLOCKTYPE) is a plain SDCLK enable, set exactly when
 *    ios->clock is nonzero (s3cmci_set_ios()). On HAMCOP the same bit is
 *    a repurposed SD/MMC mode select that is never toggled.
 *  - SDIDATA is a word-addressed FIFO port, 4 bytes per access
 *    (s3cmci's do_pio_read()/do_pio_write() walk it in u32 units).
 *
 * The async command/response/PIO state machine (mmc_request_done() from
 * a tasklet-driven PIO loop) has the same shape as hamcop-sdi.c, which
 * follows s3cmci.c. SDICON is initialized and maintained here (FIFORESET
 * at power-up and on transfer error, CLOCKTYPE tracking ios->clock).
 *
 * Card-detect, write-protect and slot power are board resources and not
 * SAMCOP registers: the slot only asks for the generic "cd"/"wp" gpiod
 * slot lines (mmc/slot-gpio.h) and a "vmmc" supply through
 * mmc_regulator_get_supply(), and polls for card changes when no
 * card-detect line is described.
 *
 * Clock: devm_clk_get_enabled(dev, NULL) on the "samcop-sdi" clkdev
 * alias that drivers/mfd/samcop.c registers on the SD ClockControl gate,
 * parented to "samcop-gclk" (the PXA25x "memory" clock, halved unless
 * ClockSleep.HALF_CLK is set). A rate of 0 Hz fails the probe.
 *
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/mmc/host.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/scatterlist.h>
#include <linux/spinlock.h>
#include <linux/time64.h>
#include <linux/timer.h>
#include <linux/unaligned.h>

/* ---------------------------------------------------------------------
 * Register offsets, relative to this cell's own mem resource (i.e.
 * already SAMCOP_SDI_BASE-relative -- see include/linux/mfd/samcop.h and
 * drivers/mfd/samcop.c's samcop_sdi_resources[]). Recovered from
 * hackndev/master's include/asm-arm/hardware/samcop-sdi.h.
 * ---------------------------------------------------------------------
 */
#define SAMCOP_SDI_CON		0x00
#define SAMCOP_SDI_PRE		0x04
#define SAMCOP_SDI_CMDARG	0x08
#define SAMCOP_SDI_CMDCON	0x0c
#define SAMCOP_SDI_CMDSTAT	0x10
#define SAMCOP_SDI_RSP0		0x14
#define SAMCOP_SDI_RSP1		0x18
#define SAMCOP_SDI_RSP2		0x1c
#define SAMCOP_SDI_RSP3		0x20
#define SAMCOP_SDI_TIMER	0x24
#define SAMCOP_SDI_BSIZE	0x28
#define SAMCOP_SDI_DCON		0x2c
#define SAMCOP_SDI_DCNT		0x30
#define SAMCOP_SDI_DSTA		0x34
#define SAMCOP_SDI_FSTA		0x38
#define SAMCOP_SDI_DATA		0x3c
#define SAMCOP_SDI_IMSK		0x40

/*
 * SDICON bit 0: plain SDCLK enable/disable, see file header comment.
 * FIFORESET is written at power-up and after a failed transfer, matching
 * s3cmci.c's own non-2440 (S3C2410-class) branch.
 */
#define SAMCOP_SDICON_FIFORESET	BIT(1)
#define SAMCOP_SDICON_CLOCKTYPE	BIT(0)

#define SAMCOP_SDICMDCON_LONGRSP	BIT(10)
#define SAMCOP_SDICMDCON_WAITRSP	BIT(9)
#define SAMCOP_SDICMDCON_CMDSTART	BIT(8)
/*
 * Taken from the factory driver, which ORs it into every command whose
 * transfer class is not "no data"; the name is inferred from S3C2410-family
 * SDICmdCon semantics, there being no SAMCOP datasheet. See the cmdcon_withdata
 * module parameter for why this is gated rather than unconditional.
 */
#define SAMCOP_SDICMDCON_WITHDATA	BIT(11)
#define SAMCOP_SDICMDCON_SENDERHOST	BIT(6)
#define SAMCOP_SDICMDCON_INDEX		0x3f

#define SAMCOP_SDICMDSTAT_CRCFAIL	BIT(12)
#define SAMCOP_SDICMDSTAT_CMDSENT	BIT(11)
#define SAMCOP_SDICMDSTAT_CMDTIMEOUT	BIT(10)
#define SAMCOP_SDICMDSTAT_RSPFIN	BIT(9)

/*
 * SDIDCON: unlike HAMCOP (which moved the block count out to its own
 * SDIBNUM register), SAMCOP keeps BLKNUM packed directly into this
 * register's low bits alongside the control flags -- samcop-sdi.h's own
 * SAMCOP_SDIDCON_BLKNUM_MASK.
 *
 * The field is 12 bits wide (0xFFF): the WinCE SDI driver (XSCSDCard.dll)
 * masks it that way (clear-mask 0xFFFFF000, set-mask 0xFFF), not as 11.
 * Bit 11 sits in genuinely unused space below BLOCKMODE (bit 17) --
 * no collision with any other named flag. Widening costs nothing (this
 * driver has never issued a >2047-block transfer, so behavior is unchanged
 * below that threshold) and matches the real hardware's own documented
 * field width instead of silently truncating a future large transfer.
 */
#define SAMCOP_SDIDCON_TXAFTERRESP	BIT(20)
#define SAMCOP_SDIDCON_RXAFTERCMD	BIT(19)
#define SAMCOP_SDIDCON_BLOCKMODE	BIT(17)
#define SAMCOP_SDIDCON_WIDEBUS		BIT(16)
#define SAMCOP_SDIDCON_STOP		BIT(14)
#define SAMCOP_SDIDCON_XFER_RXSTART	(2 << 12)
#define SAMCOP_SDIDCON_XFER_TXSTART	(3 << 12)
#define SAMCOP_SDIDCON_BLKNUM_MASK	0xfff

#define SAMCOP_SDIDSTA_FIFOFAIL	BIT(8)
#define SAMCOP_SDIDSTA_CRCFAIL		BIT(7)
#define SAMCOP_SDIDSTA_RXCRCFAIL	BIT(6)
#define SAMCOP_SDIDSTA_DATATIMEOUT	BIT(5)
#define SAMCOP_SDIDSTA_XFERFINISH	BIT(4)
#define SAMCOP_SDIDSTA_TXDATAON	BIT(1)
#define SAMCOP_SDIDSTA_RXDATAON	BIT(0)
#define SAMCOP_SDIDSTA_ERRORS		(SAMCOP_SDIDSTA_FIFOFAIL | \
					 SAMCOP_SDIDSTA_CRCFAIL | \
					 SAMCOP_SDIDSTA_RXCRCFAIL | \
					 SAMCOP_SDIDSTA_DATATIMEOUT)

#define SAMCOP_SDIFSTA_TFDET		BIT(13)
#define SAMCOP_SDIFSTA_RFDET		BIT(12)
/*
 * samcop-sdi.h's own COUNTMASK (0x7f) allows values up to 127, but
 * mainline's removed s3cmci.c (same IP, S3C2410 branch) treats the real
 * FIFO as 64 bytes deep ("63 - fifostat" in its fifo_free()). Inherited
 * the same way hamcop-sdi.c inherits it -- not independently confirmed
 * for SAMCOP; worth double-checking against real hardware if PIO
 * throughput looks wrong.
 */
#define SAMCOP_SDIFSTA_COUNTMASK	0x7f
#define SAMCOP_SDI_FIFO_DEPTH		64

/*
 * A minimum SDIPRE for data stages, for testing a slower clock. Data stages
 * run at the divider negotiated for the card, or at this one if it is slower
 * (SDCLK = gclk / (PRE + 1)), so it can never overclock a card. Byte-wide
 * because SDIPRE is an 8-bit divider field, not a flag.
 *
 * Writes used to default to 0xff on the premise that the TX FIFO running
 * empty between CPU refills is destructive. The factory XSCSDCard.dll is
 * interrupt-driven PIO on this same block and writes at the negotiated
 * clock with no clamp, and on the HAMCOP sibling PIO writes at 24.9 MHz
 * completed correctly, so the premise was never confirmed.
 */
static u8 samcop_sdi_pio_pre;
module_param_named(pio_pre, samcop_sdi_pio_pre, byte, 0644);
MODULE_PARM_DESC(pio_pre,
		 "minimum SDIPRE divider for write data stages, and for reads with read_clamp set (SDCLK = gclk / (PRE + 1)); default 0, the negotiated clock. Takes effect on the next request.");

/*
 * Whether the pio_pre minimum above also applies to READ data stages. It has
 * no effect unless pio_pre is raised above the negotiated divider.
 *
 * Off by default. The clamp's premise is a transmit-side FIFO underrun
 * with no equivalent on the receive side, and with it on a read stage
 * starts at the slow divider until samcop_sdi_pio_read() restores
 * host->prescaler at its first RX service, in the middle of a block.
 * The factory driver never writes SDIPRE from FIFO service. On the HAMCOP
 * sibling, the same IP generation, that mid-transfer divider change is
 * what corrupted PIO reads of a 4 GiB SD card in 13 of 13 passes, a
 * one-nibble shift with no error logged, against 12 of 12 correct with
 * the clamp off.
 *
 * Boolean rather than a second byte divider for the same reason as the
 * HAMCOP sibling -- the value a read wants is host->prescaler, negotiated
 * per card at .set_ios() time, which cannot be written down as a constant.
 * Y is kept so a test can reproduce the old behaviour.
 */
static bool samcop_sdi_read_clamp;
module_param_named(read_clamp, samcop_sdi_read_clamp, bool, 0644);
MODULE_PARM_DESC(read_clamp,
		 "apply the pio_pre clamp to read data stages as well as writes (default N: reads run at the negotiated divider; Y starts them at pio_pre and switches mid-block, which corrupted reads on the HAMCOP sibling). Takes effect on the next request.");

/*
 * Whether samcop_sdi_setup_data() resets the FIFO before every data stage.
 *
 * Default Y, which is both the factory driver's behaviour and the HAMCOP
 * sibling's; see setup_data() for the derivation. A parameter rather than
 * unconditional code because it lands in the one code path that has only
 * just started working on h5550 silicon, and an A/B of it should cost a
 * boot argument rather than a rebuild and a second serial kernel upload.
 */
static bool samcop_sdi_reset_fifo_per_stage = true;
module_param_named(reset_fifo_per_stage, samcop_sdi_reset_fifo_per_stage, bool, 0644);
MODULE_PARM_DESC(reset_fifo_per_stage,
		 "reset the SDI FIFO before every data stage, as the factory driver does (default Y); N restores the previous behaviour of resetting only at probe, at power-up, on stuck-data recovery and on error. Takes effect on the next request.");

/*
 * Whether samcop_sdi_send_command() sets SDICMDCON bit 11 for a command
 * that carries a data phase.
 *
 * Default N: this bit is not currently defined or written anywhere in this
 * driver or its HAMCOP sibling, and every data-carrying command on a
 * cold-boot-present card already completes correctly without it (on
 * hardware). The factory driver (h5550-XSCSDCard.c,
 * FUN_02163bf8) sets it for every command whose transfer-class field is not
 * "no data" -- byte-verified against the decompile, not inferred from a
 * datasheet, since none exists in this tree for SAMCOP. Every command that
 * failed to arm its data engine in a hot-insert boot (CMD51,
 * ACMD13, CMD6 -- all data-carrying) is exactly the class this bit gates,
 * which is why it is kept as a real, cheap A/B rather than dropped: default
 * off preserves the one SD path that currently works, and
 * samcop_sdi.cmdcon_withdata=1 tests the factory's own convention with no
 * rebuild.
 */
static bool samcop_sdi_cmdcon_withdata;
module_param_named(cmdcon_withdata, samcop_sdi_cmdcon_withdata, bool, 0644);
MODULE_PARM_DESC(cmdcon_withdata,
		 "set SDICMDCON bit 11 (undocumented; factory driver sets it for every data-carrying command) when the command has a data phase (default N = current behaviour). Takes effect on the next request.");

/*
 * Whether samcop_sdi_pio_read() sizes each RX service by what the transfer
 * still wants instead of by the FIFO count.
 *
 * Reads have returned wrong data with no error bit set on h5550, and the
 * default read path trusts SDIFSTA's 7-bit count -- read over VLIO, across
 * the SDCLK/bus clock-domain boundary -- for how many words to pull out of
 * SDIDATA. A count that is off by a word in either direction shifts the
 * payload on this side of the SD bus, where no CRC can see it. The factory
 * driver never sizes a read by the count: it asks for a fixed chunk and only
 * waits for the count to reach it. With this set, each service takes exactly
 * SAMCOP_SDI_RX_CHUNK bytes (exactly the remainder at the end of the
 * transfer), and the count only gates whether a service is due. Default N
 * leaves the read path as it is, so this is a one-token A/B.
 */
static bool samcop_sdi_pio_read_fixed;
module_param_named(pio_read_fixed, samcop_sdi_pio_read_fixed, bool, 0644);
MODULE_PARM_DESC(pio_read_fixed,
		 "size every RX FIFO service by the transfer (32 bytes, then the remainder) and use the FIFO count only as a gate (default N = drain by FIFO count). Takes effect on the next RX service.");

/*
 * Whether samcop_sdi_pio_read() skips its SDIPRE write when the register
 * already holds host->prescaler.
 *
 * The default rewrites SDIPRE on every RX service, and with read_clamp=Y the
 * first of those switches SDCLK from the pio_pre divider to the negotiated
 * one in the middle of a block. Every later rewrite stores the value already
 * there, but whether this IP restarts its divider on any SDIPRE store is
 * undocumented, and the factory driver never writes SDIPRE from its data
 * path at all. Default N keeps today's write-per-service.
 */
static bool samcop_sdi_pio_pre_once;
module_param_named(pio_pre_once, samcop_sdi_pio_pre_once, bool, 0644);
MODULE_PARM_DESC(pio_pre_once,
		 "write SDIPRE from the RX service only when it does not already hold the negotiated divider (default N = rewrite it on every RX service). Takes effect on the next RX service.");

/*
 * Whether a finished data command is checked for a data error that latched
 * after the handler's last look at DSTA (see samcop_sdi_check_late_error()).
 *
 * The check waits a bounded time for the last block's CRC result and turns
 * any error bit into a request error. On a CMD18 the card keeps sending
 * after the block count until CMD12, and whether this IP latches FIFOFAIL,
 * RXCRCFAIL or a data timeout from that run-on is not known; if it does, a
 * read that completes today would fail, be retried or split, and an SD root
 * would slow down or not mount. Default N leaves the completion path as it
 * was, so the first boot with it is a one-token A/B.
 */
static bool samcop_sdi_late_err_check;
module_param_named(late_err_check, samcop_sdi_late_err_check, bool, 0644);
MODULE_PARM_DESC(late_err_check,
		 "fail a data command whose DSTA error bits latch after its last FIFO word (default N = complete it as the handler saw it). Takes effect on the next request.");

/* RXFIFOHALF's threshold: the unit of one fixed-size RX service. */
#define SAMCOP_SDI_RX_CHUNK		(SAMCOP_SDI_FIFO_DEPTH / 2)

/* host->pre_written when SDIPRE's content is not known. */
#define SAMCOP_SDI_PRE_UNKNOWN		0x100

#define SAMCOP_SDIIMSK_RESPONSECRC	BIT(17)
#define SAMCOP_SDIIMSK_CMDSENT		BIT(16)
#define SAMCOP_SDIIMSK_CMDTIMEOUT	BIT(15)
#define SAMCOP_SDIIMSK_RESPONSEND	BIT(14)
#define SAMCOP_SDIIMSK_FIFOFAIL	BIT(11)
#define SAMCOP_SDIIMSK_CRCSTATUS	BIT(10)
#define SAMCOP_SDIIMSK_DATACRC		BIT(9)
#define SAMCOP_SDIIMSK_DATATIMEOUT	BIT(8)
#define SAMCOP_SDIIMSK_DATAFINISH	BIT(7)
#define SAMCOP_SDIIMSK_TXFIFOHALF	BIT(4)
#define SAMCOP_SDIIMSK_RXFIFOLAST	BIT(2)
#define SAMCOP_SDIIMSK_RXFIFOHALF	BIT(0)

/*
 * SDITIMER is a single 32-bit register here (unlike HAMCOP's TIMER1:
 * TIMER2 16+16 pair), but only its low 16 bits are used. That is not an
 * inference from s3cmci.c's non-2440 branch, which loads the same
 * 0x0000ffff for the S3C2410-class IP this block is adapted from -- there
 * is a direct source: hackndev/master's own
 * include/asm-arm/hardware/ipaq-samcop.h:350 is literally
 * "#define SAMCOP_SDI_TIMER_MAX 0xffff", and drivers/mfd/samcop_base.c:734
 * hands exactly that to its SDI driver as plat->timeout. Two independent
 * sources for this part, not one for a relative.
 */
#define SAMCOP_SDI_TIMER_MAX		0x0000ffff

/* SD protocol minimum: 74 SDCLK cycles before the card is ready for its
 * first command -- samcop-sdi.h's own SAMCOP_CARD_INIT_CYCLES.
 */
#define SAMCOP_SDI_INIT_CYCLES		74

/*
 * Bounds on the software backstop timer (host->req_timer), which is armed for
 * every command. The starting point is the MMC core's data->timeout_ns for a
 * command with a data phase and cmd->busy_timeout (zero for most) for one
 * without. Both are derived from the card's CSD or the command's class and
 * say nothing about the host block wedging, so the result is floored at a
 * second (below that a merely slow card would be aborted) and capped at ten
 * (above that a wedge is indistinguishable from the hang the timer exists to
 * break). The floor is the HAMCOP sibling's HAMCOP_SDI_REQ_TIMEOUT_MIN_MS;
 * its cap is higher.
 */
#define SAMCOP_SDI_REQ_TIMEOUT_MIN_MS	1000
#define SAMCOP_SDI_REQ_TIMEOUT_MAX_MS	10000

/*
 * Settable so that a wedge and a slow card can be told apart without a
 * rebuild: a timeout that fires at exactly the floor only proves nothing
 * answered inside it. The cap above still applies, so a floor above it is
 * treated as the cap.
 */
static unsigned int samcop_sdi_req_timeout_min_ms = SAMCOP_SDI_REQ_TIMEOUT_MIN_MS;
module_param_named(req_timeout_min_ms, samcop_sdi_req_timeout_min_ms, uint, 0644);
MODULE_PARM_DESC(req_timeout_min_ms,
		 "Floor, in ms, on the per-command software backstop timer; default 1000, capped at SAMCOP_SDI_REQ_TIMEOUT_MAX_MS (10000). Takes effect on the next command.");

enum samcop_sdi_waitfor {
	COMPLETION_NONE,
	COMPLETION_FINALIZE,
	COMPLETION_CMDSENT,
	COMPLETION_RSPFIN,
	COMPLETION_XFERFINISH,
	COMPLETION_XFERFINISH_RSPFIN,
};

enum samcop_sdi_pio_dir {
	XFER_NONE,
	XFER_READ,
	XFER_WRITE,
};

struct samcop_sdi_host {
	struct device		*dev;
	struct mmc_host		*mmc;
	void __iomem		*base;
	int			irq;
	struct clk		*clk;

	/* Guards every register access below CMDCON write time and all host
	 * state shared between samcop_sdi_irq(), the tasklet and the submit
	 * path. The block layer serializing .request() against itself is not
	 * enough: the submit path and the interrupt handler race against each
	 * other, not against a second submitter. samcop_sdi_request() takes
	 * it and samcop_sdi_send_request() runs entirely underneath it, which
	 * is also why send_request() must never be called without it held --
	 * samcop_sdi_finalize_request()'s stop-command recursion reaches it
	 * from the tasklet, which already holds it.
	 */
	spinlock_t		lock;

	struct mmc_request	*mrq;
	bool			cmd_is_stop;
	enum samcop_sdi_waitfor	complete_what;

	enum samcop_sdi_pio_dir	pio_active;
	unsigned int		pio_sgptr;
	unsigned int		pio_bytes;
	unsigned int		pio_count;
	u8			*pio_ptr;

	unsigned int		bus_width;
	unsigned long		clk_rate;
	unsigned long		real_rate;
	u8			prescaler;

	/*
	 * The value this driver last stored to SDIPRE, or
	 * SAMCOP_SDI_PRE_UNKNOWN when a store may not have landed or a reset
	 * may have changed it. Only samcop_sdi.pio_pre_once reads it; it is
	 * kept current at every SDIPRE write regardless.
	 */
	u16			pre_written;

	/*
	 * Shadow copy of SDIIMSK, replacing a read-modify-write of the
	 * register itself. Two reasons, in that order:
	 *
	 * The register read is worth dropping on its own -- SDIIMSK sits
	 * behind SAMCOP's static-memory (VLIO) window, where a read costs as
	 * much as a write, and this is the PIO hot path. Same change as
	 * 1138e7567bd8 on the HAMCOP sibling, simpler here because SAMCOP has
	 * one 32-bit mask register where HAMCOP splits it across two 16-bit
	 * ones.
	 *
	 * But a shadow is NOT self-synchronising, and it is worth being
	 * precise about that: samcop_sdi_clear_imsk() is a single store,
	 * whereas samcop_sdi_enable_imsk() is still a read-modify-write of
	 * host->imsk. If the handler's clear_imsk() lands between that OR and
	 * the register store, the shadow ends up 0 while the hardware holds
	 * the OR'd value, and the *next* enable_imsk() then programs a mask
	 * missing the data-completion sources -- a request that never
	 * completes except by the MMC core's timeout. The old hardware-RMW
	 * could not drift that way because hardware was the single source of
	 * truth. What makes the shadow safe is host->lock covering the submit
	 * path as well as the handler and the tasklet; see that field.
	 */
	u32			imsk;

	/*
	 * DSTA error and XFERFINISH bits that samcop_sdi_irq() acked while a
	 * request was already waiting for the tasklet to finalize it. Those
	 * acks are W1C, so without this record a CRC failure latching in that
	 * window would be erased before samcop_sdi_check_late_error() reads
	 * DSTA. Cleared at the start of every command and consumed there.
	 */
	u32			late_dsta;

	/*
	 * Backstop for a command or data phase that never signals completion
	 * at all. Every error this driver reports arrives as an interrupt, so a
	 * block that simply stops producing them would hang the MMC core's
	 * request wait forever with nothing in the log. Armed for every command
	 * in samcop_sdi_send_command(), the stop command that follows a
	 * timed-out data phase included, and disarmed in
	 * samcop_sdi_finalize_request().
	 */
	struct timer_list	req_timer;

	struct tasklet_struct	pio_tasklet;
};

static inline u32 samcop_sdi_read(struct samcop_sdi_host *host, u32 reg)
{
	return readl_relaxed(host->base + reg);
}

static inline void samcop_sdi_write(struct samcop_sdi_host *host, u32 reg, u32 val)
{
	writel_relaxed(val, host->base + reg);
}

/* Both of these track host->imsk rather than reading SDIIMSK back -- see
 * that field's comment for why the register read-modify-write was a race
 * and not merely a wasted bus cycle, and why the shadow needs host->lock.
 */
static void samcop_sdi_enable_imsk(struct samcop_sdi_host *host, u32 imsk)
{
	lockdep_assert_held(&host->lock);

	host->imsk |= imsk;
	samcop_sdi_write(host, SAMCOP_SDI_IMSK, host->imsk);
}

static void samcop_sdi_clear_imsk(struct samcop_sdi_host *host)
{
	host->imsk = 0;
	samcop_sdi_write(host, SAMCOP_SDI_IMSK, 0);
}

/*
 * FIFO occupancy, clamped to the real depth.
 *
 * Defensive symmetry with samcop_sdi_fifo_free() below, not a fix for
 * anything observed: SAMCOP_SDIFSTA_COUNTMASK is 7 bits and can express up
 * to 127 against a 64-entry FIFO, and fifo_free() already has to saturate
 * for exactly that reason. The read path has the same exposure with worse
 * consequences -- samcop_sdi_pio_read()'s loop takes this value as "words
 * available" and stores every one of them into the scatterlist, so an
 * out-of-range count would pull junk out of a 64-entry FIFO into a
 * caller's buffer rather than merely stalling. drivers/mmc/host/
 * hamcop-sdi.c has the same asymmetry and is left alone until this one has
 * met real silicon.
 */
static inline u32 samcop_sdi_fifo_count(struct samcop_sdi_host *host)
{
	u32 count = samcop_sdi_read(host, SAMCOP_SDI_FSTA) &
		    SAMCOP_SDIFSTA_COUNTMASK;

	return min_t(u32, count, SAMCOP_SDI_FIFO_DEPTH);
}

/*
 * Free space in the TX FIFO, saturating at zero.
 *
 * The obvious "DEPTH - 1 - count" is a trap, and on the h2200 sibling of
 * this driver it is what made that board read perfectly while every write
 * failed. SDIFSTA's occupancy field is 7 bits wide
 * (SAMCOP_SDIFSTA_COUNTMASK, values to 127) against a 64-entry FIFO, and
 * the count was observed reaching the full depth on real hardware, so the
 * subtraction goes negative -- which in unsigned arithmetic is not a small
 * error, it is 0xffffffff. samcop_sdi_pio_write() then clamps that to the
 * bytes it has left and pushes an entire multi-block transfer into a
 * 64-entry FIFO in one burst, which the hardware reports back as
 * SAMCOP_SDIDSTA_FIFOFAIL with the block counter never having advanced.
 * Fixed on the sibling by 4d743bf7b4a7; see drivers/mmc/host/hamcop-sdi.c's
 * hamcop_sdi_fifo_free() for the same comment against observed silicon.
 *
 * Saturating here also gives the write path its correct pacing: free space
 * of zero ends the fill loop, which arms TXFIFOHALF and lets the interrupt
 * handler resume as the FIFO drains.
 */
static inline u32 samcop_sdi_fifo_free(struct samcop_sdi_host *host)
{
	u32 count = samcop_sdi_fifo_count(host);

	if (count >= SAMCOP_SDI_FIFO_DEPTH - 1)
		return 0;

	return SAMCOP_SDI_FIFO_DEPTH - 1 - count;
}

/*
 * Wait @cycles SDCLK periods at the currently programmed prescaler.
 *
 * Denominated in SDCLK cycles rather than a hand-picked wall-clock figure
 * because SDCLK is what clocks the blocks being waited on, and its rate
 * moves by a factor of 128 between card identification and full speed.
 * Callers must run after the first .set_ios() has published a non-zero
 * host->real_rate; the probe-time reset does not wait, having
 * nothing in flight to wait for.
 */
static void samcop_sdi_settle(struct samcop_sdi_host *host, unsigned int cycles)
{
	udelay(DIV_ROUND_UP(cycles * 1000000UL, host->real_rate));
}

/*
 * Bound, in SDCLK cycles, on waiting for SDIDCON.STOP to read back clear.
 * The stop has no documented duration, and a DCON write that lands while it
 * is still pending is lost: on real h5550s the arming write of the next data
 * stage read back as 0x4001 (STOP still set, only the new block count taken)
 * or 0x0001 instead of 0xa2001, in 15 of 24 identification-time stages at
 * dividers 124-255 and 0 of 374 at divider 1, each such stage then timing
 * out. The bound is the worst a
 * force-stop could plausibly wait out -- one 512-byte block plus CRC on a
 * 1-bit bus -- so a genuinely stuck bit is reported rather than waited on
 * forever.
 */
#define SAMCOP_SDI_CTRL_SETTLE_CYCLES	((512 + 2) * 8 + SAMCOP_SDI_INIT_CYCLES)

/*
 * Poll a self-clearing control bit until it reads back clear, bounded by
 * SAMCOP_SDI_CTRL_SETTLE_CYCLES at the SDCLK divider actually programmed
 * right now. That is read back from SDIPRE rather than derived from
 * host->real_rate, because error and timeout recovery for a data stage can
 * run while setup_data()'s slow pio_pre clamp is still in force; only
 * finalize_request() puts the negotiated divider back. host->clk_rate is
 * published in probe before the first reset, so it is always valid here.
 */
static void samcop_sdi_wait_clear(struct samcop_sdi_host *host, u32 reg,
				  u32 bit, const char *what)
{
	unsigned int pre, bound_us, i;
	u32 val;

	pre = samcop_sdi_read(host, SAMCOP_SDI_PRE) & 0xff;
	bound_us = DIV_ROUND_UP_ULL((u64)SAMCOP_SDI_CTRL_SETTLE_CYCLES *
				    USEC_PER_SEC * (pre + 1), host->clk_rate);

	for (i = 0; ; i++) {
		val = samcop_sdi_read(host, reg);
		if (!(val & bit))
			return;
		if (i == bound_us)
			break;
		udelay(1);
	}

	dev_err_ratelimited(host->dev,
			    "%s still set %u us after it was written (reg %#04x = %#010x, PRE %#04x, SDCLK %lu Hz)\n",
			    what, bound_us, reg, val, pre,
			    host->clk_rate / (pre + 1));
}

/*
 * Park the data engine the way a request that finished normally leaves it:
 * DCON zeroed, which on this part zeroes the block count with it, and
 * nothing forced. This is exactly what hackndev/master's
 * drivers/mmc/samcop_sdi.c does after every request (its
 * sdi_wrreg(host, SAMCOP_SDIDCON, 0) at :442), and the WinCE driver does
 * not even that -- no reference driver force-stops a transfer that
 * completed.
 */
static void samcop_sdi_idle_data(struct samcop_sdi_host *host)
{
	samcop_sdi_write(host, SAMCOP_SDI_DCON, 0);
}

/*
 * Put the data engine back in its idle state: force-stop, then disarm.
 * Only for paths where a transfer may still be running -- errors, timeouts,
 * stuck-line recovery and probe (this driver has no data-engine reset at
 * teardown). A request that completed normally goes through
 * samcop_sdi_idle_data() instead.
 *
 * SDIDCON bit 14 is taken as the S3C2410 generation's StopDat, not the
 * S3C2440's DatStart -- mainline's s3cmci.c carried both spellings of the
 * bit, S3C2410_SDIDCON_STOP and S3C2440_SDIDCON_DATSTART, before its
 * removal in 0b14558977a7. The block-count width does not settle which
 * generation this part belongs to: s3cmci uses one 12-bit BLKNUM mask for
 * both.
 *
 * Writing bit 14 and then zeroing the
 * register lands on the same idle state whichever meaning the bit has.
 * Writing it alone would leave the register holding a stopped-but-
 * configured transfer, block count and direction included.
 *
 * The stop is not instantaneous, and a DCON write issued while it is still
 * pending is dropped (SAMCOP_SDI_CTRL_SETTLE_CYCLES), so the bit is polled
 * clear before the idle write that follows -- and, more to the point,
 * before whatever arms the next transfer. The idle write itself is the
 * state samcop_sdi_idle_data() documents.
 */
static void samcop_sdi_stop_data(struct samcop_sdi_host *host)
{
	samcop_sdi_write(host, SAMCOP_SDI_DCON, SAMCOP_SDIDCON_STOP);
	samcop_sdi_wait_clear(host, SAMCOP_SDI_DCON, SAMCOP_SDIDCON_STOP,
			      "SDIDCON.STOP");
	samcop_sdi_idle_data(host);
}

/*
 * FIFORESET alone, leaving SDIDCON untouched: the WinCE data-stage setup's
 * own shape (FUN_02163b18 ORs it into SDICON and reads on with no wait and
 * no poll), which is why it is not followed by one here either.
 */
static void samcop_sdi_fifo_reset(struct samcop_sdi_host *host)
{
	samcop_sdi_write(host, SAMCOP_SDI_CON,
			 samcop_sdi_read(host, SAMCOP_SDI_CON) |
			  SAMCOP_SDICON_FIFORESET);
}

/*
 * The above plus a force-stop, for the paths that need the block genuinely
 * reset rather than merely disarmed: probe (which inherits whatever the
 * previous owner of this chip left behind), the stuck-data-line recovery
 * in samcop_sdi_setup_data() and finalizing a request that failed.
 *
 * FIFORESET stands in for the S3C2440-only SDRESET that s3cmci pairs with
 * its own force-stop in exactly this recovery loop (s3cmci_reset(), called
 * from s3cmci_setup_data()); SAMCOP's recovered register documentation
 * describes no such bit, so none is written blind. FIFORESET itself is
 * known-safe here -- hackndev's samcop_sdi_card_power() sets it on every
 * power-on (samcop_base.c:658).
 */
static void samcop_sdi_reset_data(struct samcop_sdi_host *host)
{
	samcop_sdi_stop_data(host);
	samcop_sdi_fifo_reset(host);
	/*
	 * Nothing documents whether a force-stop or FIFO reset leaves SDIPRE
	 * alone, so the next RX service must not assume it does.
	 */
	host->pre_written = SAMCOP_SDI_PRE_UNKNOWN;
}

/* ---------------------------------------------------------------------
 * PIO -- word-at-a-time (SDIDATA is a 32-bit-wide FIFO port on this
 * chip, see file header comment), with a byte tail for anything not a
 * multiple of 4.
 * ---------------------------------------------------------------------
 */

static int samcop_sdi_get_data_buffer(struct samcop_sdi_host *host,
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

/*
 * samcop_sdi.pio_read_fixed's RX service: move exactly SAMCOP_SDI_RX_CHUNK
 * bytes per service while at least that much of the transfer is left, then
 * exactly the remainder, and read the FIFO count only to decide whether the
 * FIFO holds that much yet.
 *
 * RXFIFOHALF asserts at SAMCOP_SDI_RX_CHUNK bytes and RXFIFOLAST once the
 * FIFO holds the end of the transfer, so while a full chunk is still wanted
 * a real HALF always finds one, and once less than a chunk is wanted the
 * FIFO never reaches HALF and LAST brings the remainder. A service that
 * finds less than it wants reads nothing: that is either samcop_sdi_irq()
 * running for a command or data-status source while RFDET happens to be
 * set, which is harmless, or a count that under-reports the FIFO, which the
 * default path would have turned into a short read and a shifted payload.
 * Here it can at worst leave the FIFO full and the stage to the MMC core's request timeout --
 * visible, not silent.
 *
 * A chunk may straddle scatterlist segments. Block I/O segments are whole
 * 512-byte sectors, so a chunk ends where a segment does; a segment that
 * ends inside a word is handled exactly as the default path does, by
 * keeping its 1-3 bytes of the last word read.
 */
static void samcop_sdi_pio_read_chunked(struct samcop_sdi_host *host)
{
	void __iomem *from = host->base + SAMCOP_SDI_DATA;
	struct mmc_data *data = host->mrq->data;
	unsigned int total = data->blocks * data->blksz;
	unsigned int chunk, n, left;

	while (host->pio_count < total) {
		chunk = min_t(unsigned int, total - host->pio_count,
			      SAMCOP_SDI_RX_CHUNK);
		if (samcop_sdi_fifo_count(host) < chunk)
			break;

		while (chunk) {
			if (!host->pio_bytes &&
			    samcop_sdi_get_data_buffer(host, &host->pio_bytes,
						       &host->pio_ptr))
				goto done;

			n = min(chunk, host->pio_bytes);
			chunk -= n;
			host->pio_bytes -= n;
			host->pio_count += n;

			for (left = n; left >= 4; left -= 4) {
				put_unaligned(readl_relaxed(from),
					      (u32 *)host->pio_ptr);
				host->pio_ptr += 4;
			}
			if (left) {
				u32 word = readl_relaxed(from);

				while (left--) {
					*host->pio_ptr++ = word;
					word >>= 8;
				}
			}
		}
	}

	if (host->pio_count < total) {
		samcop_sdi_enable_imsk(host, SAMCOP_SDIIMSK_RXFIFOHALF |
					     SAMCOP_SDIIMSK_RXFIFOLAST);
		return;
	}

done:
	host->pio_active = XFER_NONE;
	host->complete_what = COMPLETION_FINALIZE;
}

static void samcop_sdi_pio_read(struct samcop_sdi_host *host)
{
	void __iomem *from = host->base + SAMCOP_SDI_DATA;
	unsigned int fifo;

	/*
	 * Reads normally start at the negotiated rate already; this write
	 * only changes the clock when samcop_sdi.read_clamp put the stage on
	 * a slower pio_pre, and then it does so in the middle of a block,
	 * which is what corrupted reads on the HAMCOP sibling.
	 * samcop_sdi_pio_write() leaves the prescaler alone.
	 * samcop_sdi_finalize_request() puts the real rate back at the end of
	 * the request in both directions.
	 */
	if (!READ_ONCE(samcop_sdi_pio_pre_once) ||
	    host->pre_written != host->prescaler) {
		samcop_sdi_write(host, SAMCOP_SDI_PRE, host->prescaler);
		host->pre_written = host->prescaler;
	}

	if (READ_ONCE(samcop_sdi_pio_read_fixed)) {
		samcop_sdi_pio_read_chunked(host);
		return;
	}

	while ((fifo = samcop_sdi_fifo_count(host))) {
		if (!host->pio_bytes &&
		    samcop_sdi_get_data_buffer(host, &host->pio_bytes, &host->pio_ptr)) {
			host->pio_active = XFER_NONE;
			host->complete_what = COMPLETION_FINALIZE;
			return;
		}

		/*
		 * At the end of the buffer the byte tail below may take a
		 * whole word out of the FIFO and keep only the 1-3 bytes
		 * still wanted -- there is nothing after them to lose. In the
		 * middle of the buffer it must not: every FIFO entry taken
		 * has to be stored somewhere, so round down to a whole
		 * multiple of 4 and leave the remainder for the next pass.
		 * Without this "else" the tail path ran mid-buffer too and
		 * discarded up to 3 bytes per iteration, with no error
		 * reported anywhere -- silent data corruption. s3cmci.c's
		 * do_pio_read() (git show 0b14558977a7^) has both legs and
		 * spells out the same reasoning; only the write path below
		 * had inherited it.
		 *
		 * ">=", not ">", also from s3cmci: taking the tail is correct
		 * exactly when the FIFO holds everything still wanted, the
		 * equal case included. With ">" that case fell into the
		 * round-down instead and stranded a 1-3 byte tail forever.
		 */
		if (fifo >= host->pio_bytes)
			fifo = host->pio_bytes;
		else
			fifo -= fifo & 3;

		/*
		 * The round-down can leave nothing to do this pass: 1-3 bytes
		 * sitting in the FIFO with more of the buffer still to come.
		 * Stop rather than spin -- this loop runs in hard IRQ context
		 * now, and re-reading an unchanged FIFO count would wedge the
		 * CPU. The bytes are picked up by the next RXFIFOHALF, or by
		 * RXFIFOLAST at the end of the block, both re-armed below.
		 */
		if (!fifo)
			break;

		host->pio_bytes -= fifo;
		host->pio_count += fifo;

		while (fifo >= 4) {
			put_unaligned(readl_relaxed(from), (u32 *)host->pio_ptr);
			host->pio_ptr += 4;
			fifo -= 4;
		}
		if (fifo) {
			u32 data = readl_relaxed(from);
			u8 *p = host->pio_ptr;

			while (fifo--) {
				*p++ = data;
				data >>= 8;
			}
			host->pio_ptr = p;
		}
	}

	if (!host->pio_bytes &&
	    samcop_sdi_get_data_buffer(host, &host->pio_bytes, &host->pio_ptr)) {
		host->pio_active = XFER_NONE;
		host->complete_what = COMPLETION_FINALIZE;
		return;
	}

	samcop_sdi_enable_imsk(host, SAMCOP_SDIIMSK_RXFIFOHALF | SAMCOP_SDIIMSK_RXFIFOLAST);
}

static void samcop_sdi_pio_write(struct samcop_sdi_host *host)
{
	void __iomem *to = host->base + SAMCOP_SDI_DATA;
	unsigned int fifo;

	while ((fifo = samcop_sdi_fifo_free(host)) > 3) {
		if (!host->pio_bytes &&
		    samcop_sdi_get_data_buffer(host, &host->pio_bytes, &host->pio_ptr)) {
			host->pio_active = XFER_NONE;
			return;
		}

		/*
		 * ">=", not ">": the last word of a buffer is allowed to be a
		 * partial one, and that is exactly the case where the FIFO
		 * has room for everything still to send. Under ">" it went to
		 * the round-down below instead, which cannot make progress on
		 * a 1-3 byte remainder.
		 */
		if (fifo >= host->pio_bytes)
			fifo = host->pio_bytes;
		else
			fifo -= fifo & 3;

		/* Nothing to do this pass -- 1-3 bytes of room with more of
		 * the buffer to come. Stop rather than re-read an unchanged
		 * free count; this runs in hard IRQ context now. TXFIFOHALF
		 * is armed on the way out and brings us back.
		 */
		if (!fifo)
			break;

		host->pio_bytes -= fifo;
		host->pio_count += fifo;

		while (fifo >= 4) {
			writel_relaxed(get_unaligned((u32 *)host->pio_ptr), to);
			host->pio_ptr += 4;
			fifo -= 4;
		}

		/*
		 * The remaining 1-3 bytes go out as one padded word. They
		 * were already counted in pio_bytes/pio_count above, so
		 * without this they were reported as sent and never actually
		 * written -- the card silently receiving short, wrong data
		 * for the block. s3cmci.c's do_pio_write() covers the same
		 * case with "fifo = (fifo + 3) >> 2", rounding the word count
		 * up; the bytes are assembled by hand here instead so that
		 * nothing is read past the end of the scatterlist buffer.
		 *
		 * The padding only makes sense because these bytes end a
		 * buffer. A short buffer followed by another one inside the
		 * same block would need the two spliced into one word, which
		 * this copier cannot do -- but that needs an sg segment whose
		 * length is not a multiple of 4 with more segments behind it,
		 * which block I/O never produces and which
		 * samcop_sdi_setup_data()'s blksz check already refuses for
		 * multi-block transfers.
		 */
		if (fifo) {
			u32 data = 0;
			unsigned int shift = 0;

			while (fifo--) {
				data |= (u32)*host->pio_ptr++ << shift;
				shift += 8;
			}
			writel_relaxed(data, to);
		}
	}

	samcop_sdi_enable_imsk(host, SAMCOP_SDIIMSK_TXFIFOHALF);
}

static void samcop_sdi_finalize_request(struct samcop_sdi_host *host);
static void samcop_sdi_send_request(struct mmc_host *mmc);

static void samcop_sdi_tasklet(struct tasklet_struct *t)
{
	struct samcop_sdi_host *host = from_tasklet(host, t, pio_tasklet);
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);

	/*
	 * Data movement now happens inline in the interrupt handler, so this
	 * is purely the completion path. Anything still marked active here
	 * means the transfer ended before the buffer was exhausted.
	 */
	if (host->complete_what == COMPLETION_FINALIZE) {
		samcop_sdi_clear_imsk(host);

		if (host->pio_active != XFER_NONE) {
			dev_err(host->dev,
				"unfinished %s -- pio_count:%u pio_bytes:%u\n",
				host->pio_active == XFER_READ ? "read" : "write",
				host->pio_count, host->pio_bytes);
			if (host->mrq->data)
				host->mrq->data->error = -EIO;
			host->pio_active = XFER_NONE;
		}

		samcop_sdi_finalize_request(host);
	}

	spin_unlock_irqrestore(&host->lock, flags);
}

/*
 * host->req_timer's callback: a command or data phase produced no completion
 * interrupt at all inside its budget.
 *
 * Same design as the HAMCOP sibling's hamcop_sdi_req_timeout(): reuse the
 * request-abort shape samcop_sdi_irq()'s own fail_transfer/close_transfer
 * labels apply on a real error, rather than inventing a second one -- put
 * the data engine back in its idle
 * state, record the error on the in-flight command, then hand the request
 * to the tasklet through COMPLETION_FINALIZE. It takes only host->lock and
 * touches no mask bit beyond samcop_sdi_clear_imsk(), so it adds no new
 * lock interaction with the ISR. No DMA engine to stop here -- this driver
 * is PIO-only (see file header comment) -- so this is samcop_sdi_irq()'s
 * fail_transfer/close_transfer shape minus that one step.
 *
 * The command in flight may have no data phase (a status poll) or be the
 * stop command a failed transfer is followed by; the abort is the same for
 * both. For the stop command that is the recovery that matters: the data
 * engine of the transfer it was meant to end is force-stopped, and the
 * request completes with the stop command's -ETIMEDOUT instead of leaving
 * the host claimed.
 */
static void samcop_sdi_req_timeout(struct timer_list *t)
{
	struct samcop_sdi_host *host = timer_container_of(host, t, req_timer);
	struct mmc_command *cmd;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);

	/*
	 * A finalize that reached the lock first has already cleared
	 * host->mrq. timer_delete() cannot cancel a callback that is
	 * already spinning here, so this check, not the delete, is what
	 * makes that race harmless -- same reasoning as the HAMCOP sibling.
	 *
	 * Finalize may also have started the stop command, or the core a new
	 * request, before this callback got the lock. That command re-armed
	 * the timer, and only its own expiry may abort it, so a pending timer
	 * here means this expiry is stale.
	 */
	if (!host->mrq || host->complete_what == COMPLETION_NONE ||
	    host->complete_what == COMPLETION_FINALIZE ||
	    timer_pending(&host->req_timer))
		goto out;

	cmd = host->cmd_is_stop ? host->mrq->stop : host->mrq->cmd;

	dev_err(host->dev,
		"request timed out: CMD%u, pio_count:%u pio_bytes:%u DSTA %#010x FSTA %#010x DCON %#010x\n",
		cmd->opcode, host->pio_count, host->pio_bytes,
		samcop_sdi_read(host, SAMCOP_SDI_DSTA),
		samcop_sdi_read(host, SAMCOP_SDI_FSTA),
		samcop_sdi_read(host, SAMCOP_SDI_DCON));

	samcop_sdi_reset_data(host);

	/*
	 * Cleared before handing over so the tasklet's "unfinished
	 * transfer" leg does not overwrite -ETIMEDOUT with its own -EIO;
	 * the counts it would have reported are in the line above.
	 */
	host->pio_active = XFER_NONE;

	cmd->error = -ETIMEDOUT;
	if (cmd->data)
		cmd->data->error = -ETIMEDOUT;

	host->complete_what = COMPLETION_FINALIZE;
	samcop_sdi_clear_imsk(host);
	tasklet_schedule(&host->pio_tasklet);

out:
	spin_unlock_irqrestore(&host->lock, flags);
}

/* ---------------------------------------------------------------------
 * Command/response state machine
 * ---------------------------------------------------------------------
 */

static irqreturn_t samcop_sdi_irq(int irq, void *dev_id)
{
	struct samcop_sdi_host *host = dev_id;
	struct mmc_command *cmd;
	u32 csta, dsta, fsta;
	u32 cclear = 0, dclear = 0;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);

	csta = samcop_sdi_read(host, SAMCOP_SDI_CMDSTAT);
	dsta = samcop_sdi_read(host, SAMCOP_SDI_DSTA);
	fsta = samcop_sdi_read(host, SAMCOP_SDI_FSTA);

	if (host->complete_what == COMPLETION_NONE ||
	    host->complete_what == COMPLETION_FINALIZE)
		goto ack_and_out;

	if (!host->mrq)
		goto ack_and_out;

	cmd = host->cmd_is_stop ? host->mrq->stop : host->mrq->cmd;
	if (!cmd)
		goto ack_and_out;

	if (csta & SAMCOP_SDICMDSTAT_CMDTIMEOUT) {
		cmd->error = -ETIMEDOUT;
		goto fail_transfer;
	}

	if (csta & SAMCOP_SDICMDSTAT_CMDSENT) {
		if (host->complete_what == COMPLETION_CMDSENT)
			goto close_transfer;
		cclear |= SAMCOP_SDICMDSTAT_CMDSENT;
	}

	if (csta & SAMCOP_SDICMDSTAT_CRCFAIL) {
		/*
		 * Same fixup s3cmci/samcop_sdi carry: this SDI IP generation
		 * misreports CRC failure for long (136-bit) responses --
		 * CMD2/CMD9/CMD10 -- so don't fail the command over it.
		 */
		if ((cmd->flags & MMC_RSP_CRC) && !(cmd->flags & MMC_RSP_136)) {
			cmd->error = -EILSEQ;
			goto fail_transfer;
		}
		cclear |= SAMCOP_SDICMDSTAT_CRCFAIL;
	}

	if (csta & SAMCOP_SDICMDSTAT_RSPFIN) {
		if (host->complete_what == COMPLETION_RSPFIN)
			goto close_transfer;
		if (host->complete_what == COMPLETION_XFERFINISH_RSPFIN)
			host->complete_what = COMPLETION_XFERFINISH;
		cclear |= SAMCOP_SDICMDSTAT_RSPFIN;
	}

	if (!cmd->data)
		goto clear_status;

	if (dsta & SAMCOP_SDIDSTA_FIFOFAIL) {
		cmd->data->error = -EILSEQ;
		goto fail_transfer;
	}

	if (dsta & (SAMCOP_SDIDSTA_RXCRCFAIL | SAMCOP_SDIDSTA_CRCFAIL)) {
		cmd->data->error = -EILSEQ;
		goto fail_transfer;
	}

	if (dsta & SAMCOP_SDIDSTA_DATATIMEOUT) {
		cmd->data->error = -ETIMEDOUT;
		goto fail_transfer;
	}

	/*
	 * Refill the FIFO here rather than handing off to the tasklet.
	 *
	 * A 64-byte FIFO at a 4-bit 24.9MHz bus drains in about 5us, and
	 * TXFIFOHALF/RXFIFOHALF only ever move 32 bytes per interrupt, so
	 * throughput is set almost entirely by how long it takes to get back
	 * to the FIFO. Separating what was measured from what is arithmetic:
	 * the HAMCOP sibling measured 774kB/s on real h2200 hardware, which
	 * works out at 32 bytes every 41us, with the copy itself put at
	 * roughly 5us and the remainder attributed to interrupt exit plus
	 * softirq dispatch (drivers/mmc/host/hamcop-sdi.c's own
	 * hamcop_sdi_irq(), commit a4094f4da245). The block does not
	 * flow-control the SD bus in PIO mode, so that round trip is not
	 * merely slow, it is the window in which an overrun happens. Doing
	 * the copy inline removes it, and with it the mask-off/mask-on pair
	 * the hand-off needed. SAMCOP's FIFO port moves 4 bytes per access
	 * where HAMCOP's moves 2, which halves only the ~5us copy; the round
	 * trip that dominates is identical on both parts.
	 *
	 * This has to sit below the fault checks: those end at fail_transfer,
	 * which parks pio_active at XFER_NONE precisely so that no further
	 * data is moved for a transfer already known to have failed. It also
	 * has to sit above the XFERFINISH check, so a completed transfer is
	 * not topped up behind the card's back.
	 */
	if (host->pio_active == XFER_WRITE && (fsta & SAMCOP_SDIFSTA_TFDET))
		samcop_sdi_pio_write(host);
	else if (host->pio_active == XFER_READ && (fsta & SAMCOP_SDIFSTA_RFDET))
		samcop_sdi_pio_read(host);

	/*
	 * pio_read() reports a drained buffer this way. Finalizing still has
	 * to run in the tasklet: it calls back into the MMC core, which is
	 * not something to do from hardirq context.
	 */
	if (host->complete_what == COMPLETION_FINALIZE) {
		samcop_sdi_clear_imsk(host);
		tasklet_schedule(&host->pio_tasklet);
		goto clear_status;
	}

	if (dsta & SAMCOP_SDIDSTA_XFERFINISH) {
		if (host->complete_what == COMPLETION_XFERFINISH)
			goto close_transfer;
		if (host->complete_what == COMPLETION_XFERFINISH_RSPFIN)
			host->complete_what = COMPLETION_RSPFIN;
		dclear |= SAMCOP_SDIDSTA_XFERFINISH;
	}

clear_status:
	samcop_sdi_write(host, SAMCOP_SDI_CMDSTAT, cclear);
	samcop_sdi_write(host, SAMCOP_SDI_DSTA, dclear);
	goto out;

fail_transfer:
	host->pio_active = XFER_NONE;

close_transfer:
	host->complete_what = COMPLETION_FINALIZE;
	samcop_sdi_clear_imsk(host);
	tasklet_schedule(&host->pio_tasklet);
	goto out;

ack_and_out:
	/*
	 * The finalize still pending may be a read whose CRC result is in
	 * this dsta, and the ack below erases it; see host->late_dsta.
	 */
	if (host->complete_what == COMPLETION_FINALIZE)
		host->late_dsta |= dsta & (SAMCOP_SDIDSTA_ERRORS |
					   SAMCOP_SDIDSTA_XFERFINISH);

	/*
	 * Nothing in flight (no request this handler currently owns), but
	 * csta/dsta were read live off real hardware above and may carry a
	 * genuinely latched condition regardless -- ack unconditionally
	 * (W1C, so this is a no-op if nothing was actually set) before
	 * masking, or the IC tier keeps re-presenting a stale status
	 * forever. See this function's own entry comment for the real
	 * occurrence that found this gap.
	 */
	samcop_sdi_write(host, SAMCOP_SDI_CMDSTAT, csta);
	samcop_sdi_write(host, SAMCOP_SDI_DSTA, dsta);
	samcop_sdi_clear_imsk(host);

out:
	spin_unlock_irqrestore(&host->lock, flags);
	return IRQ_HANDLED;
}

/*
 * Budget for the last read block's CRC to be checked after its last data
 * word has left the FIFO: 16 CRC bits and the end bit follow the last data
 * bit on every line, plus the clock-domain crossing into DSTA, rounded up
 * to 32 SDCLK. Capped because this runs with interrupts off; at
 * identification-time dividers the cap is shorter than the budget and the
 * check below only sees what has latched by then.
 */
#define SAMCOP_SDI_LATE_ERR_CYCLES	32
#define SAMCOP_SDI_LATE_ERR_MAX_US	20

/*
 * Turn a data-stage error that latched after samcop_sdi_irq() last looked
 * into a request error. A read completes from samcop_sdi_pio_read() the
 * moment the last byte leaves the FIFO, and the handler takes that exit
 * with nothing newer than its entry snapshot of DSTA -- which cannot yet
 * hold the last block's CRC result. Without this the request completes
 * successfully with whatever was drained.
 *
 * Only a read waits: a write completes on XFERFINISH, and the handler has
 * already tested every error bit in the snapshot that carried it. Stale
 * bits cannot fail a good transfer here, because samcop_sdi_send_request()
 * clears DSTA before every data stage and the handler's first interrupt of
 * a stage would already have failed it on any bit left set.
 */
static void samcop_sdi_check_late_error(struct samcop_sdi_host *host,
					struct mmc_data *data)
{
	unsigned int pre, budget_us, bound_us, waited = 0;
	u32 dsta, done = SAMCOP_SDIDSTA_ERRORS | SAMCOP_SDIDSTA_XFERFINISH;

	dsta = host->late_dsta | samcop_sdi_read(host, SAMCOP_SDI_DSTA);
	host->late_dsta = 0;

	if ((data->flags & MMC_DATA_READ) && !(dsta & done)) {
		pre = samcop_sdi_read(host, SAMCOP_SDI_PRE) & 0xff;
		budget_us = DIV_ROUND_UP_ULL((u64)SAMCOP_SDI_LATE_ERR_CYCLES *
					     USEC_PER_SEC * (pre + 1),
					     host->clk_rate);
		bound_us = min_t(unsigned int, budget_us,
				 SAMCOP_SDI_LATE_ERR_MAX_US);

		while (!(dsta & done) && waited < bound_us) {
			udelay(1);
			waited++;
			dsta |= samcop_sdi_read(host, SAMCOP_SDI_DSTA);
		}
	}

	if (!(dsta & SAMCOP_SDIDSTA_ERRORS))
		return;

	if (dsta & (SAMCOP_SDIDSTA_FIFOFAIL | SAMCOP_SDIDSTA_RXCRCFAIL |
		    SAMCOP_SDIDSTA_CRCFAIL))
		data->error = -EILSEQ;
	else
		data->error = -ETIMEDOUT;
}

static void samcop_sdi_finalize_request(struct samcop_sdi_host *host)
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
	 * host->mrq check makes the non-synchronous delete safe, see
	 * samcop_sdi_req_timeout().
	 */
	timer_delete(&host->req_timer);

	cmd = host->cmd_is_stop ? mrq->stop : mrq->cmd;

	cmd->resp[0] = samcop_sdi_read(host, SAMCOP_SDI_RSP0);
	cmd->resp[1] = samcop_sdi_read(host, SAMCOP_SDI_RSP1);
	cmd->resp[2] = samcop_sdi_read(host, SAMCOP_SDI_RSP2);
	cmd->resp[3] = samcop_sdi_read(host, SAMCOP_SDI_RSP3);

	/*
	 * Before the PRE restore below, so the wait for the last block's CRC
	 * is sized against the divider the block was actually clocked at,
	 * and before the reset/idle choice, which an error must steer.
	 */
	if (!host->cmd_is_stop && cmd->data && !cmd->error &&
	    !cmd->data->error && READ_ONCE(samcop_sdi_late_err_check))
		samcop_sdi_check_late_error(host, cmd->data);

	/* Restore the real clock rate in case setup_data() slowed it down
	 * for a write, or for a read with read_clamp set that never got its
	 * own FIFO-half interrupt (e.g. a response-only error abort).
	 */
	samcop_sdi_write(host, SAMCOP_SDI_PRE, host->prescaler);
	host->pre_written = host->prescaler;

	samcop_sdi_write(host, SAMCOP_SDI_CMDARG, 0);
	if (cmd->error || (mrq->data && mrq->data->error))
		samcop_sdi_reset_data(host);
	else
		samcop_sdi_idle_data(host);
	samcop_sdi_write(host, SAMCOP_SDI_CMDCON, 0);
	samcop_sdi_clear_imsk(host);

	if (cmd->data && cmd->error)
		cmd->data->error = cmd->error;

	if (cmd->data && cmd->data->stop && !host->cmd_is_stop) {
		host->cmd_is_stop = true;
		samcop_sdi_send_request(host->mmc);
		return;
	}

	if (!mrq->data)
		goto request_done;

	if (mrq->data->error == 0)
		mrq->data->bytes_xfered = mrq->data->blocks * mrq->data->blksz;
	else
		mrq->data->bytes_xfered = 0;

	/*
	 * FIFORESET on failure, and also after any write whose block size had
	 * a 1-3 byte tail: samcop_sdi_pio_write() pushes that tail as one
	 * padded 32-bit word, the controller sends only blksz bytes, and the
	 * pad stays behind in the TX FIFO to be prepended to the next
	 * transfer. Nothing else on this path clears it -- samcop_sdi_idle_data()
	 * above only disarms DCON, and samcop_sdi_send_request() only clears
	 * status registers. samcop_sdi_setup_data()'s per-stage reset would
	 * also catch it, but only at the start of the next data stage;
	 * clearing the pad in the request that created it keeps that
	 * independent.
	 *
	 * Believed unreachable rather than a live bug: samcop_sdi_setup_data()
	 * rejects a non-word-multiple blksz for anything multi-block, and SD/
	 * MMC block writes are 512 bytes. Kept because it costs one register
	 * read/write on a path that by construction never runs, and because
	 * the failure it would otherwise produce is silent data corruption in
	 * an unrelated later request.
	 */
	if (mrq->data->error != 0 ||
	    ((mrq->data->flags & MMC_DATA_WRITE) && (mrq->data->blksz & 3))) {
		u32 con = samcop_sdi_read(host, SAMCOP_SDI_CON);

		samcop_sdi_write(host, SAMCOP_SDI_CON, con | SAMCOP_SDICON_FIFORESET);
	}

request_done:
	host->complete_what = COMPLETION_NONE;
	host->mrq = NULL;
	mmc_request_done(host->mmc, mrq);
}

/*
 * How long samcop_sdi_send_command() gives a command before host->req_timer
 * aborts it. A data command is sized by the core's data->timeout_ns, a
 * command without data by its busy_timeout, which the floor lifts whenever
 * the core left it unset.
 */
static unsigned long samcop_sdi_req_budget_ms(const struct mmc_command *cmd)
{
	unsigned long min_ms = min_t(unsigned long,
				     READ_ONCE(samcop_sdi_req_timeout_min_ms),
				     SAMCOP_SDI_REQ_TIMEOUT_MAX_MS);
	unsigned long ms = cmd->data ? cmd->data->timeout_ns / NSEC_PER_MSEC :
				       cmd->busy_timeout;

	return clamp_t(unsigned long, ms, min_ms, SAMCOP_SDI_REQ_TIMEOUT_MAX_MS);
}

static void samcop_sdi_send_command(struct samcop_sdi_host *host, struct mmc_command *cmd)
{
	u32 ccon;

	samcop_sdi_enable_imsk(host,
			       SAMCOP_SDIIMSK_CRCSTATUS | SAMCOP_SDIIMSK_CMDTIMEOUT |
				SAMCOP_SDIIMSK_RESPONSEND | SAMCOP_SDIIMSK_CMDSENT |
				SAMCOP_SDIIMSK_RESPONSECRC);

	if (cmd->data)
		host->complete_what = COMPLETION_XFERFINISH_RSPFIN;
	else if (cmd->flags & MMC_RSP_PRESENT)
		host->complete_what = COMPLETION_RSPFIN;
	else
		host->complete_what = COMPLETION_CMDSENT;

	samcop_sdi_write(host, SAMCOP_SDI_CMDARG, cmd->arg);

	ccon = cmd->opcode & SAMCOP_SDICMDCON_INDEX;
	ccon |= SAMCOP_SDICMDCON_SENDERHOST | SAMCOP_SDICMDCON_CMDSTART;
	if (cmd->flags & MMC_RSP_PRESENT)
		ccon |= SAMCOP_SDICMDCON_WAITRSP;
	if (cmd->flags & MMC_RSP_136)
		ccon |= SAMCOP_SDICMDCON_LONGRSP;
	if (cmd->data && READ_ONCE(samcop_sdi_cmdcon_withdata))
		ccon |= SAMCOP_SDICMDCON_WITHDATA;

	samcop_sdi_write(host, SAMCOP_SDI_CMDCON, ccon);

	/*
	 * Arm the backstop for every command. CMDSENT, RSPFIN and CMDTIMEOUT
	 * cover a command in hardware, but each is delivered as a SAMCOP
	 * interrupt, and a stall after which none is delivered leaves the
	 * command with nothing to end it. The stop command that follows a
	 * timed-out data phase is the one that hung an h5550 with the host
	 * claimed. See host->req_timer.
	 */
	mod_timer(&host->req_timer,
		  jiffies + msecs_to_jiffies(samcop_sdi_req_budget_ms(cmd)));
}

static int samcop_sdi_setup_data(struct samcop_sdi_host *host, struct mmc_data *data)
{
	u32 dcon, dsta, pre;
	int retries = 3;

	/* The FIFO port moves 4 bytes/access -- a multi-block transfer
	 * whose block size isn't a multiple of 4 can't be walked cleanly.
	 */
	if ((data->blksz & 3) && data->blocks > 1) {
		dev_warn(host->dev, "block size %u not word-aligned, unsupported for multi-block transfers\n",
			 data->blksz);
		return -EINVAL;
	}

	while ((dsta = samcop_sdi_read(host, SAMCOP_SDI_DSTA)) &
	       (SAMCOP_SDIDSTA_TXDATAON | SAMCOP_SDIDSTA_RXDATAON)) {
		if (retries-- == 0) {
			/* Dump what is actually stuck rather than just saying
			 * that something is -- this is the one place the
			 * driver gets to see the data engine wedged, and
			 * s3cmci_setup_data() spends its own last retry on a
			 * register dump for the same reason.
			 */
			dev_err(host->dev,
				"data line stuck busy, refusing new transfer (DSTA %#010x DCON %#010x FSTA %#010x CON %#010x PRE %#010x)\n",
				dsta,
				samcop_sdi_read(host, SAMCOP_SDI_DCON),
				samcop_sdi_read(host, SAMCOP_SDI_FSTA),
				samcop_sdi_read(host, SAMCOP_SDI_CON),
				samcop_sdi_read(host, SAMCOP_SDI_PRE));
			return -EBUSY;
		}

		samcop_sdi_reset_data(host);

		/*
		 * Then give the engine real SDCLK edges to act on before
		 * looking again. Without a wait this loop re-read DSTA a
		 * couple of hundred nanoseconds after the write -- all three
		 * iterations fit inside a single SDCLK period at the 400kHz
		 * the core runs card identification at (2.5us) -- so it could
		 * only ever observe the same pre-stop value three times over
		 * and report a stuck data line that was merely mid-stop.
		 * Same fix as the HAMCOP sibling's setup_data().
		 */
		samcop_sdi_settle(host, SAMCOP_SDI_INIT_CYCLES);
	}

	dcon = data->blocks & SAMCOP_SDIDCON_BLKNUM_MASK;
	dcon |= SAMCOP_SDIDCON_BLOCKMODE;
	if (host->bus_width == MMC_BUS_WIDTH_4)
		dcon |= SAMCOP_SDIDCON_WIDEBUS;

	if (data->flags & MMC_DATA_WRITE)
		dcon |= SAMCOP_SDIDCON_TXAFTERRESP | SAMCOP_SDIDCON_XFER_TXSTART;
	if (data->flags & MMC_DATA_READ)
		dcon |= SAMCOP_SDIDCON_RXAFTERCMD | SAMCOP_SDIDCON_XFER_RXSTART;

	/*
	 * Put the block into a known state before every data phase, not only
	 * on the recovery path above. Nothing else resets the FIFO on a
	 * request that succeeds: finalize_request()'s FIFORESET is
	 * conditional on an error or an unaligned write, so any residue the
	 * previous stage left behind would prefix this one's data -- a shift
	 * the host's own CRC checking cannot see, because it happens on this
	 * side of the SD bus.
	 *
	 * The factory WinCE driver does it unconditionally here: its
	 * data-stage setup FUN_02163b18 opens with SDICON |= FIFORESET before
	 * touching any SDIDCON bit (XSCSDCard.dll bytes
	 * 0x02163b34-0x02163b3c).
	 *
	 * Hygiene with precedent rather than a fix for an observed fault:
	 * unlike WinCE's PIO read, which takes only the requested byte count
	 * out of SDIDATA and structurally leaves a tail behind,
	 * samcop_sdi_pio_read() drains until the FIFO reads empty, so the
	 * residue window here is only its early return when a scatterlist
	 * runs out with bytes still latched. That requires the controller to
	 * hold more than the transfer asked for, which is unproven on this
	 * silicon.
	 *
	 * A bare FIFORESET, never the full samcop_sdi_reset_data(): that
	 * force-stops through SDIDCON.STOP, and a stop still pending when the
	 * DCON write below lands swallows it -- the arming word then reads
	 * back with STOP still set or with every direction and mode bit gone,
	 * and the stage times out (SAMCOP_SDI_CTRL_SETTLE_CYCLES has the
	 * hardware evidence). WinCE never force-stops here either. The data
	 * engine is already idle at this point: the previous request's
	 * finalize left DCON zeroed, and a data line still busy was
	 * force-stopped by the recovery loop above.
	 */
	if (samcop_sdi_reset_fifo_per_stage)
		samcop_sdi_fifo_reset(host);

	samcop_sdi_write(host, SAMCOP_SDI_BSIZE, data->blksz);
	samcop_sdi_write(host, SAMCOP_SDI_DCON, dcon);

	samcop_sdi_enable_imsk(host,
			       SAMCOP_SDIIMSK_FIFOFAIL | SAMCOP_SDIIMSK_DATACRC |
				SAMCOP_SDIIMSK_DATATIMEOUT | SAMCOP_SDIIMSK_DATAFINISH);

	samcop_sdi_write(host, SAMCOP_SDI_TIMER, SAMCOP_SDI_TIMER_MAX);

	/*
	 * Run the data stage at the negotiated divider, or at samcop_sdi.pio_pre
	 * if that is slower. PIO feeds SDIDATA from the interrupt handler, so at
	 * the negotiated clock the TX FIFO can run empty between refills; the
	 * factory driver does the same on this block, and PIO writes at 24.9 MHz
	 * have completed correctly on the HAMCOP sibling. Reads ignore the
	 * minimum unless samcop_sdi.read_clamp is set; see that parameter.
	 */
	pre = ((data->flags & MMC_DATA_READ) && !samcop_sdi_read_clamp) ?
		host->prescaler :
		max(READ_ONCE(samcop_sdi_pio_pre), host->prescaler);
	samcop_sdi_write(host, SAMCOP_SDI_PRE, pre);
	host->pre_written = pre;

	dev_dbg(host->dev, "data stage: PRE %#04x (real rate divider %u), %s %u x %u\n",
		pre, host->prescaler,
		(data->flags & MMC_DATA_WRITE) ? "write" : "read",
		data->blocks, data->blksz);

	return 0;
}

static void samcop_sdi_prepare_pio(struct samcop_sdi_host *host, struct mmc_data *data)
{
	host->pio_sgptr = 0;
	host->pio_bytes = 0;
	host->pio_count = 0;

	if (data->flags & MMC_DATA_WRITE) {
		host->pio_active = XFER_WRITE;
		samcop_sdi_pio_write(host);
	} else {
		host->pio_active = XFER_READ;
		samcop_sdi_enable_imsk(host, SAMCOP_SDIIMSK_RXFIFOHALF | SAMCOP_SDIIMSK_RXFIFOLAST);
	}
}

/*
 * Runs with host->lock held, from samcop_sdi_request() or from
 * samcop_sdi_finalize_request()'s stop-command recursion (which is itself
 * called from the tasklet, already holding it). Nothing below here sleeps:
 * the only delay on the path is samcop_sdi_settle()'s udelay(), and
 * mmc_request_done() on the setup_data() failure leg is atomic-safe -- the
 * tasklet already calls it under this same lock.
 */
static void samcop_sdi_send_request(struct mmc_host *mmc)
{
	struct samcop_sdi_host *host = mmc_priv(mmc);
	struct mmc_request *mrq = host->mrq;
	struct mmc_command *cmd = host->cmd_is_stop ? mrq->stop : mrq->cmd;

	lockdep_assert_held(&host->lock);

	samcop_sdi_write(host, SAMCOP_SDI_CMDSTAT, 0xffffffff);
	samcop_sdi_write(host, SAMCOP_SDI_DSTA, 0xffffffff);
	samcop_sdi_write(host, SAMCOP_SDI_FSTA, 0xffffffff);
	host->late_dsta = 0;

	if (cmd->data) {
		int ret = samcop_sdi_setup_data(host, cmd->data);

		if (ret) {
			cmd->error = ret;
			cmd->data->error = ret;
			/* Nothing has been armed yet, so there is no
			 * completion to wait for and no interrupt source to
			 * leave enabled -- and, as on the -ENOMEDIUM path in
			 * samcop_sdi_request(), the core owns (and may free)
			 * mrq again the moment mmc_request_done() returns, so
			 * the host must stop pointing at it. Only
			 * samcop_sdi_finalize_request() used to do that, and
			 * this path never reaches it.
			 */
			host->mrq = NULL;
			host->complete_what = COMPLETION_NONE;
			samcop_sdi_clear_imsk(host);
			mmc_request_done(mmc, mrq);
			return;
		}

		samcop_sdi_prepare_pio(host, cmd->data);
	}

	samcop_sdi_send_command(host, cmd);
}

static void samcop_sdi_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct samcop_sdi_host *host = mmc_priv(mmc);
	unsigned long flags;
	bool absent;

	/*
	 * Card-detect first, and outside the lock: mmc_gpio_get_cd() takes
	 * gpiod_get_value_cansleep() whenever the descriptor's controller can
	 * sleep (drivers/mmc/core/slot-gpio.c:98-101), so it must not run in
	 * atomic context. SAMCOP's own gpio_chip does not sleep today, but
	 * this is the core's contract, not a per-board fact.
	 */
	absent = mmc_gpio_get_cd(mmc) == 0;

	/*
	 * Everything from here down is shared with samcop_sdi_irq() and the
	 * tasklet: host->mrq, host->complete_what, host->pio_*, and -- the
	 * one that is a real bug without this -- host->imsk, which
	 * samcop_sdi_enable_imsk() read-modify-writes from setup_data(),
	 * prepare_pio() and send_command() while the handler may be zeroing
	 * it. See the host->imsk field comment.
	 */
	spin_lock_irqsave(&host->lock, flags);

	host->cmd_is_stop = false;
	host->mrq = mrq;

	if (absent) {
		/* Drop the reference before handing the mrq back: the core
		 * owns (and may free) it again the moment mmc_request_done()
		 * returns, and nothing else on this path clears host->mrq the
		 * way samcop_sdi_finalize_request() does.
		 */
		host->mrq = NULL;
		host->complete_what = COMPLETION_NONE;
		mrq->cmd->error = -ENOMEDIUM;
	} else {
		samcop_sdi_send_request(mmc);
	}

	spin_unlock_irqrestore(&host->lock, flags);

	if (absent)
		mmc_request_done(mmc, mrq);
}

static void samcop_sdi_set_clock(struct samcop_sdi_host *host, unsigned int clock)
{
	unsigned int psc;

	if (clock) {
		psc = host->clk_rate / clock;
		psc = psc ? psc - 1 : 0;
		if (host->clk_rate / (psc + 1) > clock)
			psc++;
		if (psc > 255)
			psc = 255;
		/*
		 * SDIPRE 0 divides gclk by 1, i.e. ~49.8MHz straight onto
		 * SDCLK -- twice this host's advertised f_max, which is
		 * clk_rate / 2 for exactly this reason. The core never asks
		 * for it, but the historical driver guards the same lower end
		 * explicitly rather than trusting that (hackndev/master's
		 * samcop_base.c:697-703, "calculated prescaler %d is too low
		 * ... Hardcoding prescaler to 1"), so guard it here too.
		 */
		if (psc < 1)
			psc = 1;
	} else {
		/*
		 * clock == 0 is the core asking for SDCLK to *stop*, which on
		 * this chip is SDICON's CLOCKTYPE gate, handled by the caller
		 * -- but some divider still has to be programmed, and 0 is the
		 * worst possible choice: it runs SDCLK at full gclk for the
		 * window between this write and the gate closing, which on a
		 * MMC_POWER_UP is across the card's VDD ramp. Park at the
		 * slowest divider instead, matching what the same reasoning
		 * produced on the HAMCOP sibling (which has no gate at all and
		 * relies on this entirely).
		 */
		psc = 255;
	}

	host->prescaler = psc;
	host->real_rate = host->clk_rate / (psc + 1);
	samcop_sdi_write(host, SAMCOP_SDI_PRE, host->prescaler);
	host->pre_written = host->prescaler;
}

static void samcop_sdi_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct samcop_sdi_host *host = mmc_priv(mmc);
	u32 con;

	switch (ios->power_mode) {
	case MMC_POWER_ON:
	case MMC_POWER_UP:
		con = samcop_sdi_read(host, SAMCOP_SDI_CON);
		samcop_sdi_write(host, SAMCOP_SDI_CON, con | SAMCOP_SDICON_FIFORESET);
		if (!IS_ERR(mmc->supply.vmmc))
			mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, ios->vdd);
		break;
	case MMC_POWER_OFF:
	default:
		if (!IS_ERR(mmc->supply.vmmc))
			mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);
		break;
	}

	/* SDICON bit 0 is a real SDCLK gate on this chip (unlike HAMCOP's
	 * repurposed one, see file header comment) -- track it against
	 * ios->clock directly, matching s3cmci_set_ios()'s own "Set
	 * CLOCK_ENABLE" handling for the same IP.
	 *
	 * Order matters around samcop_sdi_set_clock(): close the gate before
	 * reprogramming SDIPRE and open it only afterwards, so no divider
	 * this function passes through is ever visible on SDCLK. Doing it the
	 * other way round leaves the old rate running while the new one is
	 * loaded -- which is how a clock-stop request used to put full-rate
	 * SDCLK on the bus for the length of set_clock()'s own settle delay.
	 */
	con = samcop_sdi_read(host, SAMCOP_SDI_CON);
	samcop_sdi_write(host, SAMCOP_SDI_CON, con & ~SAMCOP_SDICON_CLOCKTYPE);

	samcop_sdi_set_clock(host, ios->clock);

	if (ios->clock) {
		samcop_sdi_write(host, SAMCOP_SDI_CON, con | SAMCOP_SDICON_CLOCKTYPE);

		/* SAMCOP_SDI_INIT_CYCLES SDCLKs at the new rate, in
		 * microseconds -- the wait samcop_sdi.c's plat->card_power()
		 * ran after every prescaler change (samcop-sdi.h's own
		 * SAMCOP_CARD_INIT_CYCLES). It has to come after the gate is
		 * reopened, since the point of it is that the card sees those
		 * cycles; the 2005 driver skips it entirely for a clock-stop
		 * request (samcop_base.c:708) and so does this.
		 */
		udelay(DIV_ROUND_UP(SAMCOP_SDI_INIT_CYCLES * 1000000UL,
				    host->real_rate));
	}

	host->bus_width = ios->bus_width;
}

static const struct mmc_host_ops samcop_sdi_ops = {
	.request	= samcop_sdi_request,
	.set_ios	= samcop_sdi_set_ios,
	.get_cd		= mmc_gpio_get_cd,
	.get_ro		= mmc_gpio_get_ro,
};

/* ---------------------------------------------------------------------
 * Probe/lifecycle
 * ---------------------------------------------------------------------
 */

static int samcop_sdi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mmc_host *mmc;
	struct samcop_sdi_host *host;
	struct resource *mem;
	int ret, irq;

	mmc = devm_mmc_alloc_host(dev, sizeof(*host));
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	host->dev = dev;
	host->mmc = mmc;
	spin_lock_init(&host->lock);
	tasklet_setup(&host->pio_tasklet, samcop_sdi_tasklet);
	timer_setup(&host->req_timer, samcop_sdi_req_timeout, 0);

	/*
	 * Do not use devm_ioremap_resource() here: it reserves the region
	 * exclusively via devm_request_mem_region(). This resource is a
	 * sub-range [SAMCOP_SDI_BASE, SAMCOP_SDI_BASE + SAMCOP_SDI_SIZE) of
	 * SAMCOP's own larger register window, and drivers/mfd/samcop.c's
	 * own probe already devm_ioremap_resource()s that entire window
	 * exclusively for itself, so this driver's own exclusive claim over
	 * a piece of the same window always loses with -EBUSY. Map with
	 * plain devm_ioremap() instead, which does not reserve the region --
	 * same fix as a6f7616fd9a3 ("mmc: hamcop-sdi: don't claim its MMIO
	 * sub-range exclusively"), applied to hamcop-sdi.c's SAMCOP sibling.
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

	/* drivers/mfd/samcop.c's "samcop-sdi" clkdev alias resolves to
	 * SAMCOP's SD gate, parented to "samcop-gclk" (PXA25x "memory" /1
	 * or /2 on ClockSleep.HALF_CLK) -- so clk_get_rate() below reports
	 * a real rate. It reported 0 Hz until that gclk parent landed, and
	 * this probe failed outright on the check below; the 0 Hz case is
	 * still treated as a failure rather than guessed around.
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
	 * Only SDICON was being written here, so SDIDCON, SDIBSIZE, SDITIMER,
	 * the interrupt mask and every status register entered Linux holding
	 * whatever the previous owner left in them -- and this driver's first
	 * SDIDCON write did not happen until the first
	 * samcop_sdi_finalize_request(), i.e. after CMD0 had already been sent
	 * into a data engine that might still be armed. A host driver
	 * inheriting an unknown data-engine state is a defect regardless of
	 * how it got there, and this chip has plenty of previous owners to
	 * inherit from: bootldr and WinCE both drive this slot, and the normal
	 * way to reach this kernel is a warm jump with no intervening chip
	 * reset.
	 *
	 * SDICON is written whole rather than read-modify-written, so
	 * CLOCKTYPE lands in a known (gated) state too; from here on
	 * samcop_sdi_set_ios() preserves the rest of the register.
	 *
	 * No settle wait: nothing is in flight to wait for, host->real_rate is
	 * not published until the first .set_ios(), and the MMC core's own
	 * power-up delays follow immediately.
	 */
	samcop_sdi_write(host, SAMCOP_SDI_CON, 0);
	samcop_sdi_clear_imsk(host);

	samcop_sdi_reset_data(host);
	samcop_sdi_write(host, SAMCOP_SDI_CMDCON, 0);
	samcop_sdi_write(host, SAMCOP_SDI_CMDSTAT, 0xffffffff);
	samcop_sdi_write(host, SAMCOP_SDI_DSTA, 0xffffffff);
	samcop_sdi_write(host, SAMCOP_SDI_FSTA, 0xffffffff);

	/*
	 * Fall back to polling when no card-detect GPIO is described rather
	 * than fail probe.
	 */
	ret = mmc_gpiod_request_cd(mmc, "cd", 0, false, 0);
	if (ret == -ENOENT)
		mmc->caps |= MMC_CAP_NEEDS_POLL;
	else if (ret)
		return dev_err_probe(dev, ret, "failed to get card-detect gpio\n");

	ret = mmc_gpiod_request_ro(mmc, "wp", 0, 0);
	if (ret && ret != -ENOENT)
		return dev_err_probe(dev, ret, "failed to get write-protect gpio\n");

	/* Likewise for slot power (PXA GPIO60 behind a board-registered
	 * fixed regulator): mmc_regulator_get_supply() happily returns a
	 * usable, always-on dummy "vmmc" regulator until board code
	 * registers the real one.
	 */
	ret = mmc_regulator_get_supply(mmc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get vmmc supply\n");

	mmc->ops = &samcop_sdi_ops;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps |= MMC_CAP_4_BIT_DATA;
	/* f_min/f_max match hackndev/master's own samcop_base.c:600-601
	 * plat->f_min/f_max, and its own comments there spell the divisor
	 * out: "SDIPRE max + 1" (255 + 1 = 256) and "SDIPRE min + 1". The
	 * divider is clk_rate / (psc + 1), so f_max = clk_rate / 2 is
	 * prescaler 1, NOT prescaler 0 -- prescaler 0 would be clk_rate / 1,
	 * which is why samcop_sdi_set_clock() above floors psc at 1. A real,
	 * cited SAMCOP value, not carried over from hamcop-sdi.c's
	 * f_max=clk_rate.
	 */
	mmc->f_min = host->clk_rate / 256;
	mmc->f_max = host->clk_rate / 2;
	mmc->max_blk_count = SAMCOP_SDIDCON_BLKNUM_MASK;
	/* max_blk_size: SDIBSIZE's real width isn't documented in
	 * samcop-sdi.h; 4095 is carried over from hamcop-sdi.c's own
	 * same-generation-IP value, not independently confirmed here.
	 */
	mmc->max_blk_size = 4095;
	mmc->max_req_size = SAMCOP_SDIDCON_BLKNUM_MASK * 512;
	mmc->max_seg_size = mmc->max_req_size;
	mmc->max_segs = 128;

	ret = devm_request_irq(dev, host->irq, samcop_sdi_irq, 0, "samcop-sdi", host);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request irq\n");

	ret = mmc_add_host(mmc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add mmc host\n");

	platform_set_drvdata(pdev, host);

	dev_info(dev, "SAMCOP SD/MMC host ready (PIO%s)\n",
		 mmc->caps & MMC_CAP_NEEDS_POLL ? ", polled card-detect" : "");

	return 0;
}

static void samcop_sdi_remove(struct platform_device *pdev)
{
	struct samcop_sdi_host *host = platform_get_drvdata(pdev);

	mmc_remove_host(host->mmc);

	/*
	 * The IRQ is devm-managed, so it stays live until well after this
	 * function returns -- which means tasklet_kill() below would be
	 * racing a handler still free to call tasklet_schedule(), and a
	 * tasklet rescheduled after being killed runs against a host whose
	 * mmc_host is already gone. Drop the handler first, by hand.
	 */
	devm_free_irq(host->dev, host->irq, host);
	/*
	 * Process context here, so the synchronous delete is both safe and
	 * the right one: nothing may re-enter the callback after this.
	 */
	timer_delete_sync(&host->req_timer);
	tasklet_kill(&host->pio_tasklet);
}

static const struct of_device_id samcop_sdi_of_match[] = {
	{ .compatible = "samsung,samcop-sdi" },
	{ }
};
MODULE_DEVICE_TABLE(of, samcop_sdi_of_match);

static struct platform_driver samcop_sdi_driver = {
	.driver = {
		.name = "samcop-sdi",
		.of_match_table = samcop_sdi_of_match,
	},
	.probe	= samcop_sdi_probe,
	.remove	= samcop_sdi_remove,
};
module_platform_driver(samcop_sdi_driver);

MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("SAMCOP companion ASIC SD/MMC host driver (HP iPAQ h5xxx)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:samcop-sdi");
