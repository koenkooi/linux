// SPDX-License-Identifier: GPL-2.0-only
/*
 * MMC host driver for the SDI (SD/MMC) block on the Samsung HAMCOP
 * companion ASIC (HP iPAQ h2200/h2210/h2212/h2215) -- the real, physical
 * SD card slot on this board. The PXA25x SoC's own MMC controller
 * (pxamci) is deliberately never used here: h2200's slot hangs entirely
 * off HAMCOP, and CONFIG_MMC_PXA staying unset is correct, not an
 * oversight.
 *
 * Two data paths: PIO, and a DMA bounce through HAMCOP's on-chip SRAM.
 * The engine cannot address DRAM at all, so DMA means copying the request
 * in and out of an SRAM window claimed from the h2200 SRAM gen_pool. A
 * board with no SRAM pool producer, an exhausted pool, or a DMA engine
 * that cannot be set up falls back to PIO.
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
 *    (DCNT1 = blocks remaining, DCNT2 = bytes remaining). Only checked
 *    for zero when the DMA engine reports completion.
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
#include <linux/genalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/mfd/hamcop.h>
#include <linux/minmax.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/platform_data/h2200-sram.h>
#include <linux/regulator/consumer.h>
#include <linux/scatterlist.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

/*
 * Smallest transfer worth bouncing through SRAM. The engine was observed
 * not to complete a transfer as short as an SCR read (four halfwords): the
 * card answers and the data sits in the FIFO, but the engine's request
 * line never asserts and the request times out. Anything shorter than one
 * block is therefore drained by the PIO path, which is also cheaper than a
 * bounce copy, an engine arm and a two-interrupt rendezvous for a few
 * halfwords.
 */
#define HAMCOP_SDI_DMA_MIN		512

/*
 * Whether SDIO function transfers may use the SRAM bounce DMA. Off by
 * default, so CMD53 data goes through PIO, the path the SDIO camera has
 * been run on. DMA reads of the camera delivered 20 of 20 frames complete
 * in one test, but that is one card and one transfer size. SD memory
 * transfers are unaffected.
 */
static bool hamcop_sdi_dma_sdio;
module_param_named(dma_sdio, hamcop_sdi_dma_sdio, bool, 0644);
MODULE_PARM_DESC(dma_sdio,
		 "route SDIO function data transfers (CMD53) through the SRAM DMA bounce too (default N: SDIO uses PIO). Takes effect on the next request.");

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
#define HAMCOP_SDI_DCNT_MASK	0x0fff
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
#define HAMCOP_SDICMDCON_ABORT		BIT(12)
#define HAMCOP_SDICMDCON_WITHDATA	BIT(11)
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

/*
 * The chip's single-channel DMA engine, which is the S3C24xx DMA IP. The
 * register semantics come from the s3c24xx dmaengine driver that was
 * removed by commit a7ddf74b784b ("dmaengine: remove s3c24xx driver").
 * The vendor driver splits each 32-bit register of that IP across a pair
 * of 16-bit ones:
 *
 *	DISRC  = DISRC0 << 29 | DISRC1	(LOC/INC at bits 30:29 | address 14:0)
 *	DIDST  = DIDST0 << 29 | DIDST1
 *	DCON   = DCON0 << 20 | DCON1	(count 19:0 | control in DCON0)
 *	DSTAT  = DSTAT0 << 20 | DSTAT1	(BUSY at bit 20, CURR_TC 19:0)
 *
 * so DISRC0, DCON0 and DSTAT0 are the high halves and carry only the top
 * few bits of their fields. The vendor ORs the address and the two control
 * bits into one register, so DISRC0/DIDST0 hold the control bits and
 * DISRC1/DIDST1 the fifteen address bits.
 *
 * The offsets below are relative to the engine's own memory resource, the
 * second one of the cell, and are reached through host->dma_base and the
 * read_dma()/write_dma() accessors, never through host->base.
 *
 * The engine cannot address DRAM, only the chip's own SRAM, so its one
 * job here is to move a request between the SRAM bounce buffer and the
 * SDI FIFO port at the real clock rate.
 */
#define HAMCOP_DMA_DISRC0	0x00	/* LOC[1] INC[0] */
#define HAMCOP_DMA_DISRC1	0x04	/* address[14:0] */
#define HAMCOP_DMA_DIDST0	0x08
#define HAMCOP_DMA_DIDST1	0x0c
#define HAMCOP_DMA_DCON0	0x10	/* control (the full DCON's high bits) */
#define HAMCOP_DMA_DCON1	0x14	/* transfer count (the low bits) */
#define HAMCOP_DMA_DSTAT0	0x18	/* full-DSTAT bits 31:16 */
#define HAMCOP_DMA_DSTAT1	0x1c	/* full-DSTAT bits 15:0: CURR_TC */
#define HAMCOP_DMA_DMASKTRIG	0x28

/* DISRC0/DIDST0 bits: the LOC and INC control bits of the full register. */
#define HAMCOP_DI_INC_FIXED		BIT(0)
#define HAMCOP_DI_BUS_APB		BIT(1)

/*
 * DCON0 bits, i.e. the control bits of the full DCON register shifted
 * down by 20. The completion interrupt request is bit 28 of the full
 * register, so bit 8 here. Bit 9 (full bit 29) is not the interrupt enable
 * in this layout; setting it instead leaves the completion interrupt off
 * and the engine runs without ever reporting done. The SD request line
 * is selected by the HWSRC field, whose zero value is the only channel
 * there is, so it needs no explicit bit.
 */
#define HAMCOP_DCON_HALFWORD		BIT(0)	/* DSZ, full bit 20 */
#define HAMCOP_DCON_NORELOAD		BIT(2)	/* full bit 22 */
#define HAMCOP_DCON_HWTRIG		BIT(3)	/* full bit 23 */
#define HAMCOP_DCON_INTREQ		BIT(8)	/* full bit 28 */

/*
 * The one DCON0 value this driver arms the engine with, and what the probe
 * time self-test writes. DMD_HS (bit 10) stays clear although the datasheet
 * labels 0 "Reserved": the vendor bootloader arms the same value and reads
 * its zImage off SD through this engine on every boot.
 */
#define HAMCOP_SDI_DMA_DCON0		(HAMCOP_DCON_INTREQ | HAMCOP_DCON_HWTRIG | \
					 HAMCOP_DCON_NORELOAD | HAMCOP_DCON_HALFWORD)

/* DSTAT0 bit 0 is the full-DSTAT BUSY bit (bit 20) shifted down. */
#define HAMCOP_DSTAT_BUSY		BIT(0)

#define HAMCOP_DMASKTRIG_STOP		BIT(2)
#define HAMCOP_DMASKTRIG_ON		BIT(1)

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
#define HAMCOP_SDIDCON_DMAEN		BIT(3)
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

/*
 * How long hamcop_sdi_dma_irq() waits, in microseconds, for DSTAT.STAT_BUSY
 * to settle after the engine has already raised its completion interrupt.
 * Long enough to cover a register that lags its own INTREQ by a bus cycle
 * or two, short enough that a genuinely stalled engine is reported rather
 * than waited on.
 */
#define HAMCOP_SDI_DMA_IDLE_US		100

/*
 * How long hamcop_sdi_quiesce() waits, in microseconds, for DMASKTRIG.ON_OFF
 * to read back 0 after STOP. Stopping an in-flight atomic transfer may take
 * several cycles, and ON_OFF reading 0 is the controller's definition of the
 * actual stop.
 */
#define HAMCOP_SDI_DMA_STOP_US		100

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
	/*
	 * The DMA engine's own window, a second memory resource of this cell
	 * nowhere near the SDI block. It is a separate mapping so that a
	 * HAMCOP_DMA_* offset cannot be added to host->base by accident; use
	 * hamcop_sdi_read_dma()/hamcop_sdi_write_dma() only.
	 */
	void __iomem		*dma_base;
	dma_addr_t		sdi_base_phys;
	int			irq;
	struct clk		*clk;

	/*
	 * Guards every register access below CMDCON write time and all host
	 * state shared between hamcop_sdi_irq(), hamcop_sdi_dma_irq(), the
	 * tasklet and the submit path. hamcop_sdi_request() takes it and
	 * hamcop_sdi_send_request() runs entirely underneath it, so
	 * send_request() must never be called without it held;
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
	 * Bounce buffer carved out of HAMCOP's on-chip SRAM, the only memory
	 * the chip's DMA engine can address. NULL means DMA is unavailable
	 * and PIO is used, as on any board whose SRAM has no pool producer
	 * and after any allocation failure.
	 *
	 * sram_buf is __iomem although genalloc returns a plain pointer: it
	 * is an ioremap() cookie for a device aperture, so it may only be
	 * touched with memcpy_toio() and memcpy_fromio(). sram_dma is the
	 * bus address the DMA engine uses, with no mapping behind it and no
	 * cache to sync.
	 */
	void __iomem		*sram_buf;
	dma_addr_t		sram_dma;
	size_t			sram_size;

	/*
	 * DMA engine state (HAMCOP's one channel). dma_clk is the engine's
	 * clock gate. dma_irq is its completion interrupt, distinct from
	 * host->irq (the SDI's): a DMA request finishes only once both the
	 * controller has reported XFERFINISH and the engine has reported
	 * done, in either order. use_dma latches whether this request
	 * bounces through SRAM; pio_active stays XFER_NONE for the whole
	 * transfer in that case, which keeps the interrupt handler's PIO
	 * refill out of the way.
	 */
	struct clk		*dma_clk;
	int			dma_irq;
	bool			dma_done;
	bool			use_dma;

	/*
	 * Controller half of the DMA completion rendezvous: XFERFINISH seen
	 * and acked while the engine was still busy. It is set only by
	 * hamcop_sdi_irq() and consumed only by hamcop_sdi_dma_irq(), which
	 * re-drives finalization once the engine reports idle, as the SDI
	 * source is acked and masked by then and no further SDI interrupt is
	 * coming.
	 */
	bool			xfer_done;

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
	 * a block (or a DMA engine) that simply stops producing them would
	 * hang the MMC core's request wait forever with nothing in the log.
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

/*
 * The DMA engine lives in a window of its own, so its registers get their
 * own accessors. Every HAMCOP_DMA_* access must go through these two:
 * through hamcop_sdi_write() the offset would be added to the SDI block's
 * mapping instead of the engine's.
 */
static inline u16 hamcop_sdi_read_dma(struct hamcop_sdi_host *host, u32 reg)
{
	return readw_relaxed(host->dma_base + reg);
}

static inline void hamcop_sdi_write_dma(struct hamcop_sdi_host *host, u32 reg, u16 val)
{
	writew_relaxed(val, host->dma_base + reg);
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
 * host->real_rate); the probe-time reset below deliberately does not wait,
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
static void hamcop_sdi_dma_stop(struct hamcop_sdi_host *host);
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
 * Both are defined further down the file, next to the interrupt handlers
 * they were factored out of.
 */
static void hamcop_sdi_process_status(struct hamcop_sdi_host *host);
static void hamcop_sdi_process_dma_done(struct hamcop_sdi_host *host);

/*
 * host->req_timer's callback: a data phase produced no completion
 * interrupt at all inside its budget.
 *
 * Before declaring a timeout, ask the question the interrupt would have
 * answered: has the transfer actually finished? The completion logic of the
 * interrupt handlers is run directly, reading the SDI block's and the DMA
 * engine's own status registers rather than relying on the interrupt
 * controller. DSTA.XFERFINISH remains the completion criterion, only polled
 * instead of signalled.
 *
 * If that finds no completion, the abort path reuses the interrupt handler's
 * own: stop the engine, put the block back in a known state, record the
 * error on the in-flight command, then hand the request to the tasklet
 * through COMPLETION_FINALIZE, exactly what hamcop_sdi_irq()'s close_transfer
 * leg does. It takes only host->lock and touches no mask bit beyond
 * hamcop_sdi_clear_imsk(), so it adds no new lock interaction with the ISR.
 */
static void hamcop_sdi_req_timeout(struct timer_list *t)
{
	struct hamcop_sdi_host *host = timer_container_of(host, t, req_timer);
	struct mmc_command *cmd;
	unsigned long flags;
	int err = -ETIMEDOUT;

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
	 * Poll fallback. Run the completion logic in the order a real
	 * XFERFINISH or DMA rendezvous would deliver it. The DMA side is only
	 * consulted once the engine has gone idle, and a still-busy engine falls
	 * through to the abort path below with -ETIMEDOUT. An idle engine with
	 * data still outstanding fails the request with -EIO instead, as the
	 * buffer is short.
	 */
	hamcop_sdi_process_status(host);
	if (host->use_dma && host->complete_what != COMPLETION_FINALIZE) {
		bool engine_on, outstanding;

		/*
		 * An idle engine is not proof of completion: DSTAT0 reads
		 * idle just as readily for an engine that never ran. Accept
		 * the DMA side only when the SDI block's own remaining
		 * counters and its FIFO are empty as well.
		 */
		engine_on = (hamcop_sdi_read_dma(host, HAMCOP_DMA_DSTAT0) &
			     HAMCOP_DSTAT_BUSY) ||
			    (hamcop_sdi_read_dma(host, HAMCOP_DMA_DMASKTRIG) &
			     HAMCOP_DMASKTRIG_ON);
		outstanding = (hamcop_sdi_read(host, HAMCOP_SDI_DCNT1) &
			       HAMCOP_SDI_DCNT_MASK) ||
			      (hamcop_sdi_read(host, HAMCOP_SDI_DCNT2) &
			       HAMCOP_SDI_DCNT_MASK) ||
			      hamcop_sdi_fifo_count(host);

		if (!engine_on && !outstanding)
			hamcop_sdi_process_dma_done(host);
		else if (!engine_on)
			err = -EIO;
	}

	if (host->complete_what == COMPLETION_FINALIZE) {
		dev_dbg(host->dev,
			"CMD%u completed by polling, the completion interrupt was not delivered\n",
			cmd->opcode);
		goto out;
	}

	dev_err(host->dev,
		"request timed out: CMD%u %s, pio_count:%u pio_bytes:%u DSTA %#06x FSTA %#06x DCON %#06x\n",
		cmd->opcode, host->use_dma ? "DMA" : "PIO",
		host->pio_count, host->pio_bytes,
		hamcop_sdi_read(host, HAMCOP_SDI_DSTA),
		hamcop_sdi_read(host, HAMCOP_SDI_FSTA),
		hamcop_sdi_read(host, HAMCOP_SDI_DCON));
	if (err == -EIO)
		dev_err(host->dev, "DMA engine idle with data outstanding, failing the request with -EIO\n");

	if (host->use_dma)
		hamcop_sdi_dma_stop(host);

	hamcop_sdi_reset_data(host);

	/*
	 * Cleared before handing over so the tasklet's "unfinished
	 * transfer" leg does not overwrite this error with its own -EIO;
	 * the counts it would have reported are in the line above.
	 */
	host->pio_active = XFER_NONE;

	cmd->error = err;
	if (cmd->data)
		cmd->data->error = err;

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
		/*
		 * In DMA mode XFERFINISH means the card has clocked the data
		 * out, but the engine's own completion interrupt may not
		 * have run yet. Finalize only once both have fired. On a
		 * write the copy-in happened before the engine was armed, on
		 * a read the DMA handler did the copy-out when the engine
		 * went idle.
		 *
		 * Ack XFERFINISH first, before anything decides to park on
		 * it. It is a latched event, and jumping to clear_status
		 * without it would write DSTA = 0 and return with the source
		 * still asserted and DATAFINISH still unmasked. Against an
		 * edge-mapped demux that re-reads the pending registers on
		 * every pass, that burns the whole dispatch budget inside one
		 * chained handler invocation, long before host->req_timer
		 * can fire.
		 */
		dclear |= HAMCOP_SDIDSTA_XFERFINISH;

		if (host->use_dma && !host->dma_done) {
			/*
			 * Acking costs the wake-up that would have brought
			 * the handler back here, so hand finalization to
			 * hamcop_sdi_dma_irq() through host->xfer_done and
			 * take DATAFINISH out of the mask. RESPONSEND stays
			 * enabled -- a COMPLETION_XFERFINISH_RSPFIN request
			 * still owes its response.
			 */
			host->xfer_done = true;
			hamcop_sdi_disable_imsk(host,
						HAMCOP_SDIIMSK1_DATAFINISH, 0);

			goto clear_status;
		}

		if (host->complete_what == COMPLETION_XFERFINISH)
			goto close_transfer;
		if (host->complete_what == COMPLETION_XFERFINISH_RSPFIN)
			host->complete_what = COMPLETION_RSPFIN;
	}

clear_status:
	hamcop_sdi_write(host, HAMCOP_SDI_CMDSTAT, cclear);
	hamcop_sdi_write(host, HAMCOP_SDI_DSTA, dclear);
	return;

fail_transfer:
	host->pio_active = XFER_NONE;
	if (host->use_dma)
		hamcop_sdi_dma_stop(host);

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
	 * FSTA is deliberately not written: its TFDET and RFDET bits are
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
	host->use_dma = false;
	host->xfer_done = false;
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

/* ---------------------------------------------------------------------
 * DMA bounce path
 *
 * The engine can only see the chip's own SRAM, so a DMA transfer is a
 * bounce: the CPU copies the request between the scatterlist and the SRAM
 * window, and the engine moves it between SRAM and the SDI FIFO port at
 * the real clock rate, sparing the card the refill latency of PIO.
 * ---------------------------------------------------------------------
 */

/*
 * Copy the request's scatterlist into (write) or out of (read) the SRAM
 * bounce buffer. sg_miter does the kmap, so this is highmem safe. The
 * buffer is an ioremap()ped aperture, so the move is memcpy_toio() or
 * memcpy_fromio(). Returns the number of bytes copied.
 */
static unsigned int hamcop_sdi_bounce_copy(struct hamcop_sdi_host *host,
					   struct mmc_data *data, bool to_sram)
{
	struct sg_mapping_iter miter;
	unsigned int done = 0;
	unsigned int len = data->sg->length;
	size_t flags;

	if (len > host->sram_size)
		len = host->sram_size;

	flags = SG_MITER_ATOMIC | (to_sram ? SG_MITER_FROM_SG : SG_MITER_TO_SG);
	sg_miter_start(&miter, data->sg, data->sg_len, flags);
	while (sg_miter_next(&miter) && done < len) {
		unsigned int chunk = min_t(unsigned int, miter.length,
					   len - done);

		if (to_sram)
			memcpy_toio(host->sram_buf + done, miter.addr, chunk);
		else
			memcpy_fromio(miter.addr, host->sram_buf + done, chunk);
		done += chunk;
	}
	sg_miter_stop(&miter);

	return done;
}

/*
 * Program and arm the engine for a single bounce of len bytes between the
 * SRAM and the FIFO port: SRAM (AHB, incrementing) to FIFO (APB, fixed)
 * for a write, and the reverse for a read. Caller holds host->lock.
 */
static void hamcop_sdi_dma_start(struct hamcop_sdi_host *host,
				 struct mmc_data *data, unsigned int len)
{
	bool is_write = data->flags & MMC_DATA_WRITE;
	u32 fifo = (u32)host->sdi_base_phys + HAMCOP_SDI_DATA;
	u32 sram = (u32)host->sram_dma;
	u32 src, dst, src_ctl, dst_ctl;

	/*
	 * The SRAM side is the pool's bus address, the FIFO side is the
	 * physical address of the SDI data register. Neither is a
	 * dma_map_*() cookie.
	 */
	if (is_write) {
		src = sram;
		src_ctl = 0;			/* AHB, increment */
		dst = fifo;
		dst_ctl = HAMCOP_DI_INC_FIXED | HAMCOP_DI_BUS_APB;
	} else {
		src = fifo;
		src_ctl = HAMCOP_DI_INC_FIXED | HAMCOP_DI_BUS_APB;
		dst = sram;
		dst_ctl = 0;			/* AHB, increment */
	}

	/*
	 * The control bits are at bits 30:29 of the full register, so they
	 * land in the high half and the low half carries the 15-bit address.
	 * The bounce endpoints are chip-internal addresses and fit that field.
	 */
	hamcop_sdi_write_dma(host, HAMCOP_DMA_DISRC0, src_ctl);
	hamcop_sdi_write_dma(host, HAMCOP_DMA_DISRC1, src & 0x7fff);
	hamcop_sdi_write_dma(host, HAMCOP_DMA_DIDST0, dst_ctl);
	hamcop_sdi_write_dma(host, HAMCOP_DMA_DIDST1, dst & 0x7fff);

	/*
	 * Control goes in DCON0 and the transfer count in DCON1. The count
	 * is in transfer units, not bytes, so a halfword transfer of len
	 * bytes is len / 2. HWTRIG drives the transfer from the request
	 * line of the SDI block, NORELOAD because each chunk is programmed
	 * once, and HALFWORD matches the 2-byte access unit of the FIFO.
	 * The count field is 20 bits wide with its top four bits in DCON0,
	 * which stay zero since a chunk is at most the 16 KiB pool.
	 */
	hamcop_sdi_write_dma(host, HAMCOP_DMA_DCON0, HAMCOP_SDI_DMA_DCON0);
	hamcop_sdi_write_dma(host, HAMCOP_DMA_DCON1, (len / 2) & 0xffff);

	/*
	 * The engine is hardware triggered, so there is no software trigger.
	 * Read-modify-write like the vendor bootloader does, so that anything
	 * else latched in the register is preserved.
	 */
	hamcop_sdi_write_dma(host, HAMCOP_DMA_DMASKTRIG,
			     (hamcop_sdi_read_dma(host, HAMCOP_DMA_DMASKTRIG) &
			      ~HAMCOP_DMASKTRIG_STOP) | HAMCOP_DMASKTRIG_ON);
}

/*
 * Abort an in-flight engine transfer. This is one register read-modify-write,
 * with no dmaengine channel to race a timeout handler against.
 */
static void hamcop_sdi_dma_stop(struct hamcop_sdi_host *host)
{
	hamcop_sdi_write_dma(host, HAMCOP_DMA_DMASKTRIG,
			     hamcop_sdi_read_dma(host, HAMCOP_DMA_DMASKTRIG) |
			     HAMCOP_DMASKTRIG_STOP);
}

/*
 * The engine's own completion interrupt, a plain interrupt rather than a
 * dmaengine callback. Done means the engine is no longer busy. All this
 * records is that the engine pushed the last halfword; the card still has
 * to clock it out, so the request is not completed here, the controller's
 * XFERFINISH is authoritative.
 */
static irqreturn_t hamcop_sdi_dma_irq(int irq, void *dev_id)
{
	struct hamcop_sdi_host *host = dev_id;
	unsigned long flags;
	u16 dstat0;
	int ret;

	spin_lock_irqsave(&host->lock, flags);

	/*
	 * DSTAT.STAT_BUSY is confirmation that the engine finished, not the
	 * event itself: the event is INTREQ, which this engine raises only at
	 * terminal count. Gating on the status read loses completions
	 * outright, because handle_edge_irq() has already acked the pending
	 * latch by the time the handler runs. If STAT_BUSY had not settled
	 * yet, returning here left dma_done false with nothing left to set
	 * it, and the request then hung until host->req_timer fired, on every
	 * transfer.
	 *
	 * Wait for it to settle instead, bounded, with a visible error at the
	 * bound. Past the bound the engine is stalled rather than lagging, so
	 * the completion is deliberately not taken: leaving dma_done false
	 * routes the request to the timeout and its register dump, which is
	 * better than finalizing a read whose SRAM copy-out may be half done.
	 */
	ret = read_poll_timeout_atomic(hamcop_sdi_read_dma, dstat0,
				       !(dstat0 & HAMCOP_DSTAT_BUSY), 1,
				       HAMCOP_SDI_DMA_IDLE_US, false,
				       host, HAMCOP_DMA_DSTAT0);
	if (ret) {
		dev_err_ratelimited(host->dev,
				    "DMA completion interrupt with the engine still busy after %u us (DSTAT0 %#06x DSTAT1 %#06x)\n",
				    HAMCOP_SDI_DMA_IDLE_US, dstat0,
				    hamcop_sdi_read_dma(host, HAMCOP_DMA_DSTAT1));
		goto out;
	}

	hamcop_sdi_process_dma_done(host);

out:
	spin_unlock_irqrestore(&host->lock, flags);

	/*
	 * Always IRQ_HANDLED: the engine raised INTREQ and the demux has
	 * already acked the pending bit, so an IRQ_NONE here buys nothing and
	 * feeds note_interrupt()'s spurious-interrupt detector on a line that
	 * was not spurious.
	 */
	return IRQ_HANDLED;
}

/*
 * The engine-idle-and-rendezvous processing of hamcop_sdi_dma_irq(), factored
 * out so host->req_timer's callback can run it when the interrupt never
 * arrives. Caller holds host->lock.
 */
static void hamcop_sdi_process_dma_done(struct hamcop_sdi_host *host)
{
	u16 dcnt1, dcnt2;

	if (!host->use_dma) {
		/*
		 * A completion with no transfer in flight, such as a stale
		 * interrupt left latched by whatever ran before this driver.
		 * Stop the engine and claim it: returning IRQ_NONE would
		 * leave the line asserted and storm the chained parent.
		 */
		hamcop_sdi_dma_stop(host);
		return;
	}

	host->dma_done = true;

	/*
	 * At terminal count on a read the engine has drained every unit out of
	 * the FIFO, which it cannot have done unless the card sent all of them,
	 * so a nonzero remaining count means the buffer is short. Copying it up
	 * would hand the block layer stale SRAM, so fail the read instead. A
	 * write is still being clocked out at this point, so its count says
	 * nothing yet and XFERFINISH judges it.
	 */
	if (host->mrq && host->mrq->data &&
	    (host->mrq->data->flags & MMC_DATA_READ)) {
		dcnt1 = hamcop_sdi_read(host, HAMCOP_SDI_DCNT1) & HAMCOP_SDI_DCNT_MASK;
		dcnt2 = hamcop_sdi_read(host, HAMCOP_SDI_DCNT2) & HAMCOP_SDI_DCNT_MASK;

		if (dcnt1 || dcnt2) {
			dev_err_ratelimited(host->dev,
					    "DMA engine idle but %u block(s) + %u byte(s) of the read are outstanding\n",
					    dcnt1, dcnt2);
			host->mrq->data->error = -EIO;
			host->xfer_done = false;
			host->pio_active = XFER_NONE;
			hamcop_sdi_dma_stop(host);
			host->complete_what = COMPLETION_FINALIZE;
			hamcop_sdi_clear_imsk(host);
			tasklet_schedule(&host->pio_tasklet);
			return;
		}
	}

	/*
	 * For a read the data is now fully in the SRAM, so move it to the
	 * request buffer before XFERFINISH finalizes. A write needs no
	 * copy-out, what is left in the FIFO is the card's concern and
	 * XFERFINISH tracks it.
	 */
	if (host->mrq && host->mrq->data &&
	    (host->mrq->data->flags & MMC_DATA_READ))
		hamcop_sdi_bounce_copy(host, host->mrq->data, false);

	/*
	 * The other half of the rendezvous. If the controller got there
	 * first, hamcop_sdi_irq() parked with XFERFINISH acked and DATAFINISH
	 * masked, so no further SDI interrupt is coming and nothing short of
	 * host->req_timer would ever finalize this request. Apply exactly the
	 * transition that leg would have made.
	 */
	if (host->xfer_done) {
		host->xfer_done = false;

		if (host->complete_what == COMPLETION_XFERFINISH) {
			host->complete_what = COMPLETION_FINALIZE;
			hamcop_sdi_clear_imsk(host);
			tasklet_schedule(&host->pio_tasklet);
		} else if (host->complete_what == COMPLETION_XFERFINISH_RSPFIN) {
			/*
			 * RESPONSEND is still unmasked; the SDI side comes
			 * back for the response on its own.
			 */
			host->complete_what = COMPLETION_RSPFIN;
		}
	}
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
	 * Decide between DMA and PIO before DCON is written: DMAEN has to
	 * reach the card in the DCON that starts the transfer, or the card
	 * never drives the request line of the engine and the armed engine
	 * waits forever. The bounce copy and arming the engine come after
	 * the write, at the end of this function.
	 */
	if (host->sram_buf && !(data->blksz & 1) && data->sg_len == 1 &&
	    data->sg->length <= host->sram_size &&
	    data->sg->length >= HAMCOP_SDI_DMA_MIN &&
	    (data->mrq->cmd->opcode != SD_IO_RW_EXTENDED || hamcop_sdi_dma_sdio)) {
		host->use_dma = true;
		host->dma_done = false;
		host->xfer_done = false;
		host->pio_active = XFER_NONE;
		dcon |= HAMCOP_SDIDCON_DMAEN;
	} else {
		host->use_dma = false;
	}

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

	if (host->use_dma) {
		/*
		 * Copy the write payload into the SRAM now. A read is copied
		 * out by the DMA completion interrupt.
		 */
		if (data->flags & MMC_DATA_WRITE)
			hamcop_sdi_bounce_copy(host, data, true);

		hamcop_sdi_dma_start(host, data, data->sg->length);
	}

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
 * hamcop_sdi_settle()'s udelay() and the bounce copy maps with
 * SG_MITER_ATOMIC.
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

		if (!host->use_dma)
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
	 * Everything from here down is shared with hamcop_sdi_irq(),
	 * hamcop_sdi_dma_irq() and the tasklet: host->mrq, host->complete_what,
	 * the PIO and DMA state and, most importantly, the host->imsk1/imsk2
	 * shadows, which the submit path read-modify-writes while a handler
	 * may be zeroing them.
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
	 * clk_rate/256, the slowest this controller can go) -- deliberately
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

/*
 * Claim a DMA bounce buffer out of HAMCOP's SRAM, if a producer has
 * published one. Everything here is best-effort: no pool, or no room in
 * it, just means DMA is unavailable and PIO is used, so none of these
 * paths return an error. The SRAM is shared with other potential DMA
 * users, which is why this suballocates instead of taking the whole
 * window.
 */
static void hamcop_sdi_claim_sram(struct hamcop_sdi_host *host)
{
	struct gen_pool *pool = h2200_sram_pool();
	size_t avail;

	if (!pool) {
		dev_dbg(host->dev, "no SRAM pool, using PIO\n");
		return;
	}

	/*
	 * Take what is free rather than a fixed size: a bigger bounce buffer
	 * means fewer, larger requests. Nothing else allocates from the pool
	 * at the moment; a second user would find it exhausted and have to
	 * fall back to PIO. Cap at max_req_size: more than one whole request
	 * can ever occupy is dead space.
	 */
	avail = min_t(size_t, gen_pool_avail(pool), host->mmc->max_req_size);
	avail = rounddown(avail, 512);
	if (!avail) {
		dev_dbg(host->dev, "SRAM pool exhausted, using PIO\n");
		return;
	}

	/*
	 * genalloc is typed void * since most SRAM consumers treat their
	 * pool as ordinary memory, but this is an ioremap()ped aperture that
	 * is only touched with the io accessors.
	 */
	host->sram_buf = (void __iomem __force *)
		gen_pool_dma_alloc(pool, avail, &host->sram_dma);
	if (!host->sram_buf) {
		dev_dbg(host->dev, "SRAM allocation failed, using PIO\n");
		return;
	}

	host->sram_size = avail;
}

static void hamcop_sdi_release_sram(struct hamcop_sdi_host *host)
{
	struct gen_pool *pool = h2200_sram_pool();

	if (!host->sram_buf || !pool)
		return;

	gen_pool_free(pool, (unsigned long __force)host->sram_buf,
		      host->sram_size);
	host->sram_buf = NULL;
	host->sram_size = 0;
}

static void hamcop_sdi_put_dma_clk(void *data)
{
	struct clk *clk = data;

	clk_disable_unprepare(clk);
	clk_put(clk);
}

/*
 * Does the engine hold what is written to it? DCON0 and DCON1 are plain
 * read/write registers, so with the channel off, writing the arming value and
 * a count and reading them back is harmless. It catches an engine that is
 * unclocked or not answering before the first transfer times out on it.
 */
static bool hamcop_sdi_dma_selftest(struct hamcop_sdi_host *host)
{
	const u16 count = 0x0155;
	u16 dcon0, dcon1;

	hamcop_sdi_write_dma(host, HAMCOP_DMA_DCON0, HAMCOP_SDI_DMA_DCON0);
	hamcop_sdi_write_dma(host, HAMCOP_DMA_DCON1, count);
	dcon0 = hamcop_sdi_read_dma(host, HAMCOP_DMA_DCON0);
	dcon1 = hamcop_sdi_read_dma(host, HAMCOP_DMA_DCON1);
	hamcop_sdi_write_dma(host, HAMCOP_DMA_DCON0, 0);
	hamcop_sdi_write_dma(host, HAMCOP_DMA_DCON1, 0);

	if (dcon0 == HAMCOP_SDI_DMA_DCON0 && dcon1 == count)
		return true;

	dev_err(host->dev,
		"DMA engine does not hold its registers: DCON0 wrote %#06x read %#06x, DCON1 wrote %#06x read %#06x\n",
		(u16)HAMCOP_SDI_DMA_DCON0, dcon0, count, dcon1);
	return false;
}

/*
 * Bring up the rest of the DMA path once the SRAM bounce buffer is there:
 * the clock gate of the engine and its completion interrupt. Failing just
 * means PIO, and sram_buf is the flag every DMA decision keys off.
 */
static int hamcop_sdi_setup_dma(struct hamcop_sdi_host *host,
				struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *mem;
	int ret;

	/*
	 * The engine sits in its own window, the second memory resource of
	 * the cell. Map it like the SDI window, and for the same reason
	 * without reserving it: it is a sub-range of the window the HAMCOP
	 * core already claims. Without it the engine is unreachable, so the
	 * driver runs PIO.
	 */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!mem)
		return -ENODEV;
	host->dma_base = devm_ioremap(dev, mem->start, resource_size(mem));
	if (!host->dma_base)
		return -ENOMEM;

	/*
	 * The engine's CLKCON gate, which the HAMCOP core leaves off.
	 *
	 * The core registers it as a clkdev alias with dev_id "hamcop-dma" and
	 * no con_id. A lookup through this device, devm_clk_get_enabled(dev,
	 * "hamcop-dma"), asks for dev_id "hamcop-sdi" with con_id "hamcop-dma",
	 * which does not match that entry; clk_find() returns the SD gate
	 * registered for dev_id "hamcop-sdi" on the device match alone. That left
	 * the SD gate enabled twice and the engine unclocked, while probe still
	 * reported DMA as available. Look the gate up by its own dev_id instead.
	 *
	 * The gate is optional: any failure to get it, including an
	 * -EPROBE_DEFER for a gate that is not registered yet, means PIO.
	 * Propagating the defer would retry the probe for ever.
	 */
	host->dma_clk = clk_get_sys("hamcop-dma", NULL);
	if (IS_ERR(host->dma_clk)) {
		host->dma_clk = NULL;
		return -ENODEV;
	}

	ret = clk_prepare_enable(host->dma_clk);
	if (ret) {
		clk_put(host->dma_clk);
		host->dma_clk = NULL;
		return -ENODEV;
	}

	ret = devm_add_action_or_reset(dev, hamcop_sdi_put_dma_clk, host->dma_clk);
	if (ret) {
		host->dma_clk = NULL;
		return ret;
	}

	/*
	 * Stop the engine the instant its clock is on. A warm boot out of a
	 * firmware that used the same engine can leave the channel armed,
	 * and an armed engine resumes as soon as the gate opens, driving
	 * the FIFO lines or the SRAM with a stale transfer. This also has
	 * to happen before the interrupt below is requested: unmasking it
	 * while the channel is armed and latched would leave the line
	 * asserted with nobody to clear it. STOP is harmless on an idle
	 * engine.
	 */
	hamcop_sdi_write_dma(host, HAMCOP_DMA_DMASKTRIG, HAMCOP_DMASKTRIG_STOP);

	if (!hamcop_sdi_dma_selftest(host))
		return -ENODEV;

	host->dma_irq = platform_get_irq(pdev, 1);
	if (host->dma_irq < 0)
		return host->dma_irq;

	ret = devm_request_irq(dev, host->dma_irq, hamcop_sdi_dma_irq, 0,
			       "hamcop-sdi-dma", host);
	if (ret) {
		host->dma_irq = 0;
		return ret;
	}

	/*
	 * A copy-based bounce takes one linear segment sized to the pool,
	 * as the Amlogic MMC driver does for its SRAM bounce. max_blk_count
	 * stays at the BNUM field: max_req_size already bounds block I/O,
	 * and sram_size / 512 would cap an SDIO CMD53 at 32 blocks of the
	 * function's block size, 2 KiB with 64-byte blocks, splitting larger
	 * reads into several commands.
	 */
	host->mmc->max_req_size = host->sram_size;
	host->mmc->max_seg_size = host->mmc->max_req_size;
	host->mmc->max_segs = 1;

	return 0;
}

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
	host->sdi_base_phys = mem->start;

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

	/*
	 * After the limits above: the claim sizes itself against
	 * max_req_size.
	 */
	hamcop_sdi_claim_sram(host);

	if (host->sram_buf && hamcop_sdi_setup_dma(host, pdev)) {
		dev_dbg(dev, "DMA unavailable, using PIO\n");
		hamcop_sdi_release_sram(host);
	}

	ret = devm_request_irq(dev, host->irq, hamcop_sdi_irq, 0, "hamcop-sdi", host);
	if (ret) {
		hamcop_sdi_release_sram(host);
		return dev_err_probe(dev, ret, "failed to request irq\n");
	}

	INIT_DELAYED_WORK(&host->scan_retry_work, hamcop_sdi_scan_retry_work);

	ret = mmc_add_host(mmc);
	if (ret) {
		hamcop_sdi_release_sram(host);
		return dev_err_probe(dev, ret, "failed to add mmc host\n");
	}
	schedule_delayed_work(&host->scan_retry_work, msecs_to_jiffies(6000));

	platform_set_drvdata(pdev, host);

	dev_info(dev, "HAMCOP SD/MMC host ready (%s%s)\n",
		 host->sram_buf ? "DMA" : "PIO",
		 mmc->caps & MMC_CAP_NEEDS_POLL ? ", polled card-detect" : "");

	return 0;
}

/*
 * Disarm the DMA engine, stop the data engine and mask every interrupt
 * source. A hardware-triggered channel left armed keeps driving the SDI
 * FIFO lines or the SRAM window after this driver has stopped caring, and
 * on h2200 that window is where the board's restart code puts the
 * bootloader thunk back.
 */
static void hamcop_sdi_quiesce(struct hamcop_sdi_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);

	if (host->dma_clk) {
		u16 trig;

		/*
		 * Without waiting for the actual stop, a transfer between the SDI
		 * FIFO and the SRAM bounce buffer could still be live when the
		 * board restart code overwrites that window with the bootloader
		 * thunk. Leave ON_OFF to STOP, it must not be changed directly.
		 */
		hamcop_sdi_dma_stop(host);

		if (read_poll_timeout_atomic(hamcop_sdi_read_dma, trig,
					     !(trig & HAMCOP_DMASKTRIG_ON), 1,
					     HAMCOP_SDI_DMA_STOP_US, false,
					     host, HAMCOP_DMA_DMASKTRIG))
			dev_err(host->dev,
				"DMA engine still on %u us after STOP (DMASKTRIG %#06x)\n",
				HAMCOP_SDI_DMA_STOP_US, trig);
	}

	hamcop_sdi_stop_data(host);
	hamcop_sdi_clear_imsk(host);

	spin_unlock_irqrestore(&host->lock, flags);
}

static void hamcop_sdi_remove(struct platform_device *pdev)
{
	struct hamcop_sdi_host *host = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&host->scan_retry_work);
	mmc_remove_host(host->mmc);

	/*
	 * Both interrupts are devm-managed, which would keep them live for
	 * the whole of remove() and free them only after it returns. A late
	 * hamcop_sdi_irq() could then reschedule the tasklet killed below
	 * and run finalize_request() against a removed host, and
	 * hamcop_sdi_dma_irq() could copy out of the SRAM window after it
	 * has been handed back to the pool. Free them first, so everything
	 * below runs with the handlers gone.
	 */
	devm_free_irq(&pdev->dev, host->irq, host);
	if (host->dma_irq > 0)
		devm_free_irq(&pdev->dev, host->dma_irq, host);

	/*
	 * The hardware can still be running with the handlers gone: an
	 * armed, hardware-triggered DMA channel needs a register write.
	 */
	hamcop_sdi_quiesce(host);

	/* Process context here, so the synchronous delete is both safe and
	 * the right one: nothing may re-enter the callback after this.
	 */
	timer_delete_sync(&host->req_timer);
	tasklet_kill(&host->pio_tasklet);
	/* Last, so nothing can still be using the buffer. */
	hamcop_sdi_release_sram(host);
}

/*
 * device_shutdown() runs every .shutdown method before kernel_restart() fires
 * the restart-prepare handlers. On h2200 one of them copies the bootloader
 * thunk back over all of HAMCOP's SRAM, which is the window the DMA bounce
 * buffer is allocated from, and it relies on any DMA user having been
 * quiesced by then. The devm-managed interrupts are freed too late for that
 * and nothing else disarms the engine, so do it here.
 */
static void hamcop_sdi_shutdown(struct platform_device *pdev)
{
	struct hamcop_sdi_host *host = platform_get_drvdata(pdev);

	if (!host)
		return;

	hamcop_sdi_quiesce(host);
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
	.shutdown = hamcop_sdi_shutdown,
};
module_platform_driver(hamcop_sdi_driver);

MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("HAMCOP companion ASIC SD/MMC host driver (HP iPAQ h2200)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:hamcop-sdi");
