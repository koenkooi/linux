// SPDX-License-Identifier: GPL-2.0-only
/*
 * MMC host driver for the SD/MMC block on the HTC ASIC1 companion chip
 * (HP/Compaq iPAQ h38xx family), the SD card slot of that hardware.
 *
 * ASIC1 has no chip select of its own: the SoC reaches it through a
 * sub-range of ASIC2's register window (ASIC2's base + ASIC1_MMC_BASE, see
 * include/linux/mfd/asic2.h), with ASIC2 forwarding the bus cycles. The
 * driver is therefore instantiated as an mfd_cell by drivers/mfd/asic2.c
 * when the devicetree describes a "htc,asic1-mmc" child, and it does not
 * claim its window: devm_ioremap() is used instead of
 * devm_ioremap_resource(), as in drivers/gpio/gpio-asic1.c, because the
 * window is a sub-range of the ASIC2 window, which the parent maps without
 * a claim, and the cells inside it map their ranges the same way.
 *
 * The register layout and the command sequence follow the ASIC1 SD driver
 * of the handhelds.org 2.4 kernel for these machines ("the 2.4 driver"
 * below), whose register header is the only complete description of this
 * block. The block belongs to the same IP family as the Freescale i.MX SDHC
 * handled by mxcmmc.c, which is used as a model where the 2.4 driver
 * leaves a gap. The registers have a 4-byte
 * stride and are 16 bits wide.
 *
 *   StartStopClock  0x00   ClockRate      0x08   CmdDataCont    0x14
 *   ResponseTimeout 0x18   ReadTimeout    0x1c   BlockLength    0x20
 *   NumOfBlocks     0x24   InterruptMask  0x34   CommandNumber  0x38
 *   ArgumentH       0x3c   ArgumentL      0x40   ResFifo        0x44
 *   DataBuffer      0x4c   BufferPartFull 0x50
 *
 * Status (0x04) is read-only. SPIRegister (0x10) belongs to a different
 * consumer, and offsets 0x28-0x30 are not described; none of these is
 * touched.
 *
 * Revision (0x0c) must never be read or written: a write resets the whole
 * ASIC, on a bus that has no bus-error timeout to even report the result.
 * For that reason there is no register-walk probe, no dump-all-registers
 * path and no register helper generic enough to be pointed at that offset
 * by mistake.
 *
 * Clock: the card clock is the SD selector of ASIC2 (ASIC2_CLOCK_SD_MASK
 * in include/linux/mfd/asic2.h), which the board selects with the
 * "htc,sd-clock-hz" property of its ASIC2 node. The driver only reads the
 * resulting rate with clk_get_rate() and holds the clock enabled from probe
 * to remove. ASIC2 parents that gate on the crystal behind the selected
 * tap (EX2 for SD_3) and reports the real rate of the tap, so the divisor
 * arithmetic here needs nothing board-specific. If that crystal is not
 * running, Status bit 8 stays set after CLOCK_START and every command
 * times out without ever completing.
 * ClockRate selects a divisor of 2^n, from FULL = 0 to DIV_64 = 6, and the
 * driver programs the fastest divisor that does not exceed the rate the MMC
 * core asks for.
 *
 * CmdDataCont (0x14) selects the response format (none 0, R1 1, R2 2, R3
 * 3), data enable, direction, stream/multi-block mode, the busy bit for R1b
 * and an initialize bit. R3, the OCR of CMD1 and ACMD41 and the R4 of CMD5,
 * carries no CRC and must not be sent as R1, since that would make the
 * controller CRC-check a response that has none.
 *
 * Status is read-only and has no acknowledge. It is cleared by the
 * CLOCK_STOP that opens every command, so that stop is the acknowledge
 * point of the previous command. Bits 0-13 are the error, FIFO, clock and
 * completion flags defined below. Bits 14 and 15 are sticky card-detect
 * event latches; they are not decoded because card detect uses a GPIO of
 * ASIC2 through the generic "cd-gpios" property.
 *
 * InterruptMask (0x34): 0 = enabled, 1 = masked, with one bit each for
 * data transfer done, program done, end of command response and buffer
 * ready. This matters for correctness. Status bits 6 and 7 (FIFO empty and
 * full) are set permanently at idle, so leaving the buffer-ready source
 * unmasked holds the interrupt output permanently asserted. MMC_INT is a
 * plain SoC GPIO interrupt, wired rising-edge in the board devicetree, and
 * SA-1100 GPIO interrupts are edge-only, so a line that never falls
 * produces no edge and no command would ever see a completion. The driver
 * therefore keeps all four sources masked at idle, unmasks only the source
 * a command waits for (the end of the command response for a command
 * without data, every source for one with data), and enables the SoC
 * interrupt only while a command is outstanding, so that the line can fall
 * again between commands.
 *
 * ResFifo (0x44) is one address that is read in sequence. A 136-bit (R2)
 * response is its first eight words and a short one its first three,
 * assembled as mxcmmc.c does, but the FIFO holds more words than either
 * and is always read ASIC1_MMC_RES_FIFO_WORDS deep: once for the response
 * and once more before each command, see ASIC1_MMC_RES_FIFO_WORDS.
 *
 * DataBuffer (0x4c) holds one whole block and is moved a block at a time,
 * gated on Status rather than on any per-word flag. The 2.4 driver and the
 * factory WinCE driver both wait for FIFO_FULL or DATA_XFER_DONE
 * (TIME_OUT_READ and CRC_READ_ERR end the wait as errors) and then read
 * BlockLength/2 words back to back. Both write paths wait for
 * FIFO_EMPTY, write the block and pulse CLOCK_START; WinCE then waits for
 * END_PROGRAM and DATA_XFER_DONE after the last block. Both put the first
 * byte of each 16-bit word in its high half, as ResFifo does.
 *
 * BufferPartFull (0x50) is not a ready flag. Neither reference ever reads
 * it; WinCE writes 1 to it after draining a read block shorter than 512
 * bytes. It stays 0 through a completed 8-byte SCR read while Status already
 * shows DATA_XFER_DONE, so polling it as a ready flag times every short read
 * out. The driver never writes it: nothing says what the write does, and
 * short reads such as the SCR complete without it.
 *
 * Write protect is a line of the SA-1100 PPC block, described with the
 * generic "wp-gpios" property. Slot power is the SD_PWR_ON output of the
 * ASIC1 GPIO block, described as a fixed regulator that is consumed here
 * through the standard vmmc supply.
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
#include <linux/minmax.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/scatterlist.h>
#include <linux/spinlock.h>
#include <linux/time64.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

/* ---------------------------------------------------------------------
 * Register offsets, relative to the memory resource of this cell, which is
 * already relative to ASIC1_MMC_BASE.
 * ---------------------------------------------------------------------
 */
#define ASIC1_MMC_START_STOP_CLOCK	0x00
#define ASIC1_MMC_STATUS		0x04
#define ASIC1_MMC_CLOCK_RATE		0x08
/* 0x0c: Revision -- never touched, see file header. */
/* 0x10: SPIRegister -- belongs to a different consumer, not touched. */
#define ASIC1_MMC_CMD_DATA_CONT		0x14
#define ASIC1_MMC_RESPONSE_TIMEOUT	0x18
#define ASIC1_MMC_READ_TIMEOUT		0x1c
#define ASIC1_MMC_BLOCK_LENGTH		0x20
#define ASIC1_MMC_NUM_OF_BLOCKS		0x24
/* 0x28-0x30: not described. */
#define ASIC1_MMC_INTERRUPT_MASK	0x34
#define ASIC1_MMC_COMMAND_NUMBER	0x38
#define ASIC1_MMC_ARGUMENT_H		0x3c
#define ASIC1_MMC_ARGUMENT_L		0x40
#define ASIC1_MMC_RES_FIFO		0x44
#define ASIC1_MMC_DATA_BUFFER		0x4c
#define ASIC1_MMC_BUFFER_PART_FULL	0x50

/*
 * StartStopClock has two bits, STOP (BIT(0)) and START (BIT(1)), at the
 * same positions as mxcmmc.c. Unlike mxcmmc.c, which only writes this
 * register on a clock or power change, this chip needs it pulsed around
 * every command: write CLOCK_STOP, poll Status bit 8 until it sets (the
 * clock has stopped), load Argument, CommandNumber and CmdDataCont, and
 * only then write CLOCK_START, which launches the command.
 * asic1_mmc_start_cmd() implements this sequence.
 *
 * Status bit 8 is named "clock stopped" in the 2.4 driver, so it is set after a
 * CLOCK_STOP write, which is the opposite polarity of what the name of the
 * corresponding bit in mxcmmc.c suggests.
 *
 * The stop is also the only acknowledge of this block: the 2.4 driver
 * reads Status straight after the stop and treats anything outside
 * CLOCK_ENABLE, FIFO_EMPTY and FIFO_FULL as a failure, i.e. the stop clears the
 * completion and error bits of the previous command.
 */
#define ASIC1_MMC_CLOCK_STOP		BIT(0)
#define ASIC1_MMC_CLOCK_START		BIT(1)

/*
 * Every write to this block is followed by 1 us of settling, as the 2.4
 * driver does after every register write, including between ArgumentH and
 * ArgumentL and between CommandNumber and CmdDataCont. ASIC1 is reached through ASIC2
 * forwarding a VLIO cycle, so back-to-back writes are not safe. Reads are
 * not delayed, except for the DataBuffer read (see
 * asic1_mmc_pio_read_block()), which the 2.4 driver delays as well.
 */
#define ASIC1_MMC_WRITE_SETTLE_US	1

/* ClockRate divisor ladder: the divisor is 2^n. */
#define ASIC1_MMC_CLOCK_RATE_FULL	0
#define ASIC1_MMC_CLOCK_RATE_DIV_64	6

/* CmdDataCont fields, see file header. */
#define ASIC1_CMDCONT_RESP_MASK		GENMASK(1, 0)
#define ASIC1_CMDCONT_RESP_NONE		0
#define ASIC1_CMDCONT_RESP_R1		1	/* R1, R1b, R5, R6, R7 */
#define ASIC1_CMDCONT_RESP_R2		2	/* 136-bit */
#define ASIC1_CMDCONT_RESP_R3		3	/* R3, R4: no CRC */
#define ASIC1_CMDCONT_DATA_ENABLE	BIT(2)
#define ASIC1_CMDCONT_WRITE		BIT(3)
#define ASIC1_CMDCONT_STREAM_BLOCK	BIT(4)
#define ASIC1_CMDCONT_MULTI_BLOCK	BIT(5)
#define ASIC1_CMDCONT_BC_MODE		BIT(6)
#define ASIC1_CMDCONT_BUSY_BIT		BIT(7)	/* R1b */
#define ASIC1_CMDCONT_INITIALIZE	BIT(8)

/*
 * Status (read-only, see file header). Bits 0-13 are the flags below, which
 * agree with the shape of mxcmmc.c's STATUS_* bits. Bits 14 and 15 are the
 * card-detect event latches, which are not decoded.
 *
 * FIFO_EMPTY and FIFO_FULL are what mxcmmc.c calls STATUS_BUF_WRITE_RDY and
 * STATUS_BUF_READ_RDY. Both are set at idle, with no card transaction in
 * flight.
 */
#define ASIC1_MMC_STATUS_TIME_OUT_READ	BIT(0)
#define ASIC1_MMC_STATUS_TIME_OUT_RESP	BIT(1)
#define ASIC1_MMC_STATUS_CRC_WRITE_ERR	BIT(2)
#define ASIC1_MMC_STATUS_CRC_READ_ERR	BIT(3)
#define ASIC1_MMC_STATUS_SPI_READ_ERR	BIT(4)
#define ASIC1_MMC_STATUS_RESP_CRC_ERR	BIT(5)
#define ASIC1_MMC_STATUS_FIFO_EMPTY	BIT(6)	/* POR value 0x0040 */
#define ASIC1_MMC_STATUS_FIFO_FULL	BIT(7)
/*
 * "Clock ready" is set when the card clock has actually stopped, which is
 * the precondition for loading a new command after a CLOCK_STOP write. It
 * is named "clock stopped" in the 2.4 driver, the opposite of what the
 * corresponding bit of mxcmmc.c suggests.
 */
#define ASIC1_MMC_STATUS_CLOCK_READY	BIT(8)
#define ASIC1_MMC_STATUS_WR_CRC_ERR	GENMASK(10, 9)
#define ASIC1_MMC_STATUS_DATA_XFER_DONE	BIT(11)
#define ASIC1_MMC_STATUS_END_PROGRAM	BIT(12)
#define ASIC1_MMC_STATUS_CMD_RESP_END	BIT(13)
/*
 * The status bits that are fatal: the two timeouts, the CRC and SPI error
 * sources and the two-bit write CRC error code.
 */
#define ASIC1_MMC_STATUS_ERR_MASK	(ASIC1_MMC_STATUS_TIME_OUT_READ | \
					 ASIC1_MMC_STATUS_TIME_OUT_RESP | \
					 ASIC1_MMC_STATUS_CRC_WRITE_ERR | \
					 ASIC1_MMC_STATUS_CRC_READ_ERR | \
					 ASIC1_MMC_STATUS_SPI_READ_ERR | \
					 ASIC1_MMC_STATUS_RESP_CRC_ERR | \
					 ASIC1_MMC_STATUS_WR_CRC_ERR)
/*
 * Bounded, atomic-context poll for CLOCK_READY before loading a new command
 * (asic1_mmc_start_cmd() runs under host->lock with interrupts off, so it
 * cannot sleep): 1 us granularity and a 1 ms bound, which is a conservative
 * estimate of the order of magnitude, not a measured value.
 */
#define ASIC1_MMC_CLOCK_READY_TIMEOUT_US	1000

/*
 * InterruptMask: 0 = enabled, 1 = masked. See the file header for why
 * leaving every source enabled holds the edge-triggered MMC_INT line high.
 */
#define ASIC1_MMC_INT_DATA_XFER_DONE	BIT(0)
#define ASIC1_MMC_INT_PROGRAM_DONE	BIT(1)
#define ASIC1_MMC_INT_CMD_RESP_END	BIT(2)
#define ASIC1_MMC_INT_BUFFER_READY	BIT(3)
#define ASIC1_MMC_INT_MASK_ALL		0xf

/*
 * ResponseTimeout is an 8-bit register counting card clocks. 0x40 is the
 * value the 2.4 driver programs and the NCR maximum of 64 clocks of the MMC/SD
 * specification. ReadTimeout is the full 16 bits and is programmed to its
 * maximum.
 */
#define ASIC1_MMC_RESPONSE_TIMEOUT_CLKS	0x40
#define ASIC1_MMC_READ_TIMEOUT_CLKS	0xffff

/*
 * Software bound for each Status wait of the data phase, at least a
 * second and never shorter than the card's own data timeout: WinCE's
 * loops are bounded too, and a data engine that never reports anything
 * must not hold the work item forever.
 */
#define ASIC1_MMC_DATA_POLL_US		10
#define ASIC1_MMC_DATA_TIMEOUT_US	USEC_PER_SEC
#define ASIC1_MMC_REQ_TIMEOUT_MS	5000

/*
 * ResFifo holds more words than a response consumes, and what a response
 * leaves behind is read back as the start of the next one. A short
 * response read as exactly three words then shows the command index where
 * the card status belongs (CMD55 reads back as 0x0037xxxx), so the MMC
 * core never sees R1_APP_CMD and SD initialisation fails with -EOPNOTSUPP.
 * Every response is therefore read as this many words, and the words
 * beyond what the response needs are discarded. A response that is never
 * read, because its command ended in an error, a timeout of the request
 * timer or a clock that did not stop, would leave its words behind in the
 * same way, so asic1_mmc_start_cmd() also reads the FIFO this deep before
 * it loads each command.
 *
 * The depth of the FIFO is not documented. Twelve words is what has run on
 * hardware: an R2 response plus four, and SD initialisation completes with
 * it.
 */
#define ASIC1_MMC_RES_FIFO_WORDS	12

enum asic1_mmc_pio_dir {
	ASIC1_MMC_XFER_NONE,
	ASIC1_MMC_XFER_READ,
	ASIC1_MMC_XFER_WRITE,
};

struct asic1_mmc_host {
	struct device		*dev;
	struct mmc_host		*mmc;
	void __iomem		*base;
	int			irq;
	struct clk		*clk;

	/*
	 * Guards every register access below command-issue time and all host
	 * state shared between the interrupt handler and the PIO work item.
	 * The PIO loop runs from a work item so that the interrupt handler
	 * stays minimal.
	 */
	spinlock_t		lock;

	struct mmc_request	*mrq;
	enum asic1_mmc_pio_dir	pio_dir;
	struct scatterlist	*pio_sg;	/* next sg entry, NULL when exhausted */
	u8			*pio_ptr;	/* position within the current entry */
	unsigned int		pio_bytes;	/* bytes left in the current entry */
	unsigned int		pio_remain;	/* bytes left in the whole transfer */
	/*
	 * Once pio_work has been scheduled for a request it is the only
	 * thing allowed to complete it: it runs unlocked against the PIO
	 * cursor above, so the interrupt handler and the request timer must
	 * not call mmc_request_done() on the same request out from under
	 * it. They record the error on the request and set pio_abort
	 * instead; pio_work stops at its next word and completes.
	 */
	bool			pio_active;
	bool			pio_abort;

	/* Block of the running transfer, for the timeout message only. */
	unsigned int		pio_blk;

	/*
	 * mrq->stop is outstanding: the data phase is over and the request
	 * completes on the stop's own CMD_RESP_END, error or timer.
	 */
	bool			stop_active;

	/*
	 * MMC_INT is only armed while a command is outstanding, so that the
	 * edge-triggered line can fall again between commands. Tracked as a
	 * boolean under host->lock rather than through the disable depth of
	 * the irq core, so that a completion interrupt racing the request
	 * timer cannot disarm twice and leave the line dead.
	 */
	bool			irq_armed;

	unsigned long		clk_rate;
	struct work_struct	pio_work;
	struct timer_list	req_timer;
};

static inline u16 asic1_mmc_read(struct asic1_mmc_host *host, u32 reg)
{
	return readw(host->base + reg);
}

static inline void asic1_mmc_write(struct asic1_mmc_host *host, u16 val, u32 reg)
{
	writew(val, host->base + reg);
	udelay(ASIC1_MMC_WRITE_SETTLE_US);
}

/*
 * Arm/disarm MMC_INT. Both are safe in atomic context and both are
 * called with host->lock held: enable_irq() never sleeps, and the
 * disarm side uses disable_irq_nosync() precisely because
 * disable_irq() would. The irq core does not hold desc->lock across an
 * action handler, so taking it from inside asic1_mmc_irq() is fine.
 */
static void asic1_mmc_irq_arm(struct asic1_mmc_host *host)
{
	lockdep_assert_held(&host->lock);

	if (!host->irq_armed) {
		host->irq_armed = true;
		enable_irq(host->irq);
	}
}

static void asic1_mmc_irq_disarm(struct asic1_mmc_host *host)
{
	lockdep_assert_held(&host->lock);

	if (host->irq_armed) {
		host->irq_armed = false;
		disable_irq_nosync(host->irq);
	}
}

/*
 * Put the block back to idle once a request is resolved: every source
 * masked at the controller and MMC_INT disarmed at the SoC, so that the
 * edge-triggered line is low before the next command arms it. The 2.4
 * driver leaves END_COMMAND_RESPONSE enabled at idle where this masks
 * everything.
 */
static void asic1_mmc_idle(struct asic1_mmc_host *host)
{
	lockdep_assert_held(&host->lock);

	asic1_mmc_write(host, ASIC1_MMC_INT_MASK_ALL, ASIC1_MMC_INTERRUPT_MASK);
	asic1_mmc_irq_disarm(host);
}

/*
 * Wait until Status shows any bit of @ready (every bit, if @all), any
 * bit of @err, or the request is aborted from the interrupt handler or
 * the request timer. Sleeps: only ever called from pio_work.
 */
static int asic1_mmc_wait_status(struct asic1_mmc_host *host, u16 ready,
				 bool all, u16 err, unsigned int timeout_us)
{
	u16 status;
	int ret;

	ret = readw_poll_timeout(host->base + ASIC1_MMC_STATUS, status,
				 (all ? (status & ready) == ready :
					(status & ready)) ||
				 (status & err) || READ_ONCE(host->pio_abort),
				 ASIC1_MMC_DATA_POLL_US, timeout_us);
	if (ret)
		return ret;
	if (status & err & ASIC1_MMC_STATUS_TIME_OUT_READ)
		return -ETIMEDOUT;
	if (status & err)
		return -EILSEQ;

	return 0;
}

/* ---------------------------------------------------------------------
 * Clock
 * ---------------------------------------------------------------------
 */
static void asic1_mmc_set_clock(struct asic1_mmc_host *host, unsigned int hz)
{
	unsigned long base = host->clk_rate;
	unsigned int divisor;
	unsigned long rate;

	if (!hz || !base) {
		asic1_mmc_write(host, ASIC1_MMC_CLOCK_STOP,
				ASIC1_MMC_START_STOP_CLOCK);
		return;
	}

	/*
	 * Fastest divisor (2^divisor, FULL..DIV_64) that does not exceed the
	 * requested rate.
	 */
	for (divisor = ASIC1_MMC_CLOCK_RATE_FULL;
	     divisor < ASIC1_MMC_CLOCK_RATE_DIV_64; divisor++) {
		rate = base >> divisor;
		if (rate <= hz)
			break;
	}

	/*
	 * Stop the clock, program the divisor, reload both timeouts and
	 * restart, in the order the 2.4 driver uses. That driver reloads the
	 * timeouts on every clock change, and nothing shows whether they
	 * survive one.
	 */
	asic1_mmc_write(host, ASIC1_MMC_CLOCK_STOP, ASIC1_MMC_START_STOP_CLOCK);
	asic1_mmc_write(host, divisor, ASIC1_MMC_CLOCK_RATE);
	asic1_mmc_write(host, ASIC1_MMC_RESPONSE_TIMEOUT_CLKS,
			ASIC1_MMC_RESPONSE_TIMEOUT);
	asic1_mmc_write(host, ASIC1_MMC_READ_TIMEOUT_CLKS, ASIC1_MMC_READ_TIMEOUT);
	asic1_mmc_write(host, ASIC1_MMC_CLOCK_START, ASIC1_MMC_START_STOP_CLOCK);
}

/* ---------------------------------------------------------------------
 * Command issue and response readout
 * ---------------------------------------------------------------------
 */
static void asic1_mmc_drain_res_fifo(struct asic1_mmc_host *host)
{
	int i;

	for (i = 0; i < ASIC1_MMC_RES_FIFO_WORDS; i++)
		asic1_mmc_read(host, ASIC1_MMC_RES_FIFO);
}

static void asic1_mmc_read_response(struct asic1_mmc_host *host,
				    struct mmc_command *cmd)
{
	u16 w[ASIC1_MMC_RES_FIFO_WORDS];
	int i;

	if (!(cmd->flags & MMC_RSP_PRESENT))
		return;

	/*
	 * ResFifo: one address, of which a 136-bit response uses the first
	 * eight words and a short one the first three, assembled as
	 * mxcmci_read_response() does. The FIFO is read
	 * ASIC1_MMC_RES_FIFO_WORDS deep, see there.
	 */
	for (i = 0; i < ASIC1_MMC_RES_FIFO_WORDS; i++)
		w[i] = asic1_mmc_read(host, ASIC1_MMC_RES_FIFO);

	if (cmd->flags & MMC_RSP_136) {
		/*
		 * The eight words are the whole R2 frame minus its CRC byte:
		 * the 0x3f start byte first, then CSD/CID bits 127:8. The core
		 * wants bits 127:0 in resp[0..3] with no start byte, so take
		 * the words one byte late, as the short response below does;
		 * the CRC byte, which the core never reads, is left zero.
		 */
		for (i = 0; i < 4; i++)
			cmd->resp[i] = ((u32)w[2 * i] << 24) |
				       ((u32)w[2 * i + 1] << 8) |
				       (i < 3 ? w[2 * i + 2] >> 8 : 0);
	} else {
		cmd->resp[0] = ((u32)w[0] << 24) | ((u32)w[1] << 8) |
			       (w[2] >> 8);
	}
}

static void asic1_mmc_start_cmd(struct asic1_mmc_host *host,
				struct mmc_command *cmd,
				struct mmc_data *data, u16 cmdcont)
{
	u16 status;
	u16 intmask;
	int ret;

	lockdep_assert_held(&host->lock);

	/*
	 * R1 and R5/R6/R7 (the same 48-bit CRC-protected shape) take R1 format,
	 * R1b adds the busy bit, R2 takes R2 format, and R3, which the MMC core
	 * shares with R4 as responses without CRC and opcode echo, takes R3
	 * format. Sending an R3 as R1 asks the controller to CRC-check a
	 * response that carries no CRC.
	 */
	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_NONE:
		break;
	case MMC_RSP_R2:
		cmdcont |= ASIC1_CMDCONT_RESP_R2;
		break;
	case MMC_RSP_R3:	/* == MMC_RSP_R4 */
		cmdcont |= ASIC1_CMDCONT_RESP_R3;
		break;
	default:
		cmdcont |= ASIC1_CMDCONT_RESP_R1;
		if (mmc_resp_type(cmd) == MMC_RSP_R1B)
			cmdcont |= ASIC1_CMDCONT_BUSY_BIT;
		break;
	}

	/*
	 * The "arm" half of every command, see the comment of
	 * ASIC1_MMC_CLOCK_STOP. If the clock never reports stopped, the command
	 * is not loaded: writing Argument, CommandNumber and CmdDataCont
	 * against a controller that has not acknowledged the stop is not
	 * safe. The request timer (asic1_mmc_req_timeout()) then resolves the
	 * request, later but with a clearer log line than a silent missing
	 * response.
	 */
	asic1_mmc_write(host, ASIC1_MMC_CLOCK_STOP, ASIC1_MMC_START_STOP_CLOCK);
	ret = readw_poll_timeout_atomic(host->base + ASIC1_MMC_STATUS, status,
					status & ASIC1_MMC_STATUS_CLOCK_READY,
					1, ASIC1_MMC_CLOCK_READY_TIMEOUT_US);
	if (ret) {
		dev_err(host->dev,
			"CMD%u: clock-ready (status bit 8) never set, status=%#06x\n",
			cmd->opcode, status);
		return;
	}

	/*
	 * Empty ResFifo of whatever an earlier command left behind, whether or
	 * not its response was read, so that this command's response starts
	 * at the first word. See ASIC1_MMC_RES_FIFO_WORDS.
	 */
	asic1_mmc_drain_res_fifo(host);

	/*
	 * Load order of the 2.4 driver, with the clock stopped for all of it: Argument
	 * and CommandNumber, then BlockLength and NumOfBlocks for a data
	 * command, then CmdDataCont, then InterruptMask, then the start.
	 */
	asic1_mmc_write(host, cmd->arg >> 16, ASIC1_MMC_ARGUMENT_H);
	asic1_mmc_write(host, cmd->arg & 0xffff, ASIC1_MMC_ARGUMENT_L);
	asic1_mmc_write(host, cmd->opcode, ASIC1_MMC_COMMAND_NUMBER);

	if (data) {
		asic1_mmc_write(host, data->blksz, ASIC1_MMC_BLOCK_LENGTH);
		asic1_mmc_write(host, data->blocks, ASIC1_MMC_NUM_OF_BLOCKS);
	}

	asic1_mmc_write(host, cmdcont, ASIC1_MMC_CMD_DATA_CONT);

	/*
	 * Unmask every source for a command with a data phase and only
	 * END_COMMAND_RESPONSE for one without. Enabling BUFFER_READY for a
	 * command without data would hold MMC_INT permanently asserted, see the
	 * file header.
	 */
	intmask = data ? 0 :
		(ASIC1_MMC_INT_MASK_ALL & ~ASIC1_MMC_INT_CMD_RESP_END);
	asic1_mmc_write(host, intmask, ASIC1_MMC_INTERRUPT_MASK);

	/* The "go" pulse -- see ASIC1_MMC_CLOCK_STOP/_START's own comment. */
	asic1_mmc_write(host, ASIC1_MMC_CLOCK_START, ASIC1_MMC_START_STOP_CLOCK);

	asic1_mmc_irq_arm(host);
}

/* ---------------------------------------------------------------------
 * PIO -- one block per Status wait, see file header.
 * ---------------------------------------------------------------------
 */
/*
 * Advance to the next scatterlist entry, one entry at a time. host->pio_sg
 * is NULL once every entry has been consumed, which the caller sees as
 * -EINVAL and which cannot happen while host->pio_remain is nonzero.
 */
static int asic1_mmc_next_buffer(struct asic1_mmc_host *host)
{
	if (!host->pio_sg)
		return -EINVAL;

	host->pio_bytes = host->pio_sg->length;
	host->pio_ptr = sg_virt(host->pio_sg);
	host->pio_sg = sg_next(host->pio_sg);

	return 0;
}

/* The next byte of the transfer, crossing scatterlist entries. */
static u8 *asic1_mmc_pio_next(struct asic1_mmc_host *host)
{
	while (!host->pio_bytes)
		if (asic1_mmc_next_buffer(host))
			return NULL;

	host->pio_bytes--;
	host->pio_remain--;

	return host->pio_ptr++;
}

/* The first byte of each DataBuffer word is its high half, as in ResFifo. */
static int asic1_mmc_pio_read_block(struct asic1_mmc_host *host,
				    unsigned int blksz)
{
	unsigned int i;
	u16 word;
	u8 *p;

	for (i = 0; i < blksz; i += 2) {
		word = asic1_mmc_read(host, ASIC1_MMC_DATA_BUFFER);

		/* The 2.4 driver delays after this read. */
		udelay(ASIC1_MMC_WRITE_SETTLE_US);

		p = asic1_mmc_pio_next(host);
		if (!p)
			return -EINVAL;
		*p = word >> 8;

		if (i + 1 < blksz) {
			p = asic1_mmc_pio_next(host);
			if (!p)
				return -EINVAL;
			*p = word & 0xff;
		}
	}

	return 0;
}

static int asic1_mmc_pio_write_block(struct asic1_mmc_host *host,
				     unsigned int blksz)
{
	unsigned int i;
	u16 word;
	u8 *p;

	for (i = 0; i < blksz; i += 2) {
		p = asic1_mmc_pio_next(host);
		if (!p)
			return -EINVAL;
		word = *p << 8;

		if (i + 1 < blksz) {
			p = asic1_mmc_pio_next(host);
			if (!p)
				return -EINVAL;
			word |= *p;
		}

		asic1_mmc_write(host, word, ASIC1_MMC_DATA_BUFFER);
	}

	return 0;
}

/*
 * The request timer budgets one block, not the whole request: a transfer of
 * up to max_blk_count blocks at a few hundred microseconds a block plus the
 * card's programming time outlasts any fixed per-request budget, and the
 * timer would kill a transfer that is still making progress.
 */
static void asic1_mmc_pio_progress(struct asic1_mmc_host *host,
				   unsigned int blk)
{
	host->pio_blk = blk;
	mod_timer(&host->req_timer,
		  jiffies + msecs_to_jiffies(ASIC1_MMC_REQ_TIMEOUT_MS));
}

/*
 * A read block is complete in DataBuffer once Status shows FIFO_FULL
 * (512 bytes) or DATA_XFER_DONE (the last or only, possibly shorter,
 * block); the block engine then holds the card clock stopped until
 * CLOCK_START asks for the next block.
 */
static int asic1_mmc_pio_read(struct asic1_mmc_host *host,
			      struct mmc_data *data, unsigned int timeout_us)
{
	unsigned int blk;
	int ret;

	for (blk = 0; blk < data->blocks; blk++) {
		asic1_mmc_pio_progress(host, blk);

		ret = asic1_mmc_wait_status(host,
					    ASIC1_MMC_STATUS_FIFO_FULL |
					    ASIC1_MMC_STATUS_DATA_XFER_DONE,
					    false,
					    ASIC1_MMC_STATUS_TIME_OUT_READ |
					    ASIC1_MMC_STATUS_CRC_READ_ERR,
					    timeout_us);
		if (ret || READ_ONCE(host->pio_abort))
			return ret;

		ret = asic1_mmc_pio_read_block(host, data->blksz);
		if (ret)
			return ret;

		if (blk + 1 < data->blocks)
			asic1_mmc_write(host, ASIC1_MMC_CLOCK_START,
					ASIC1_MMC_START_STOP_CLOCK);
	}

	return 0;
}

/*
 * Each block goes into an empty buffer and is sent by CLOCK_START; the
 * transfer is over only once the card has programmed the last block
 * (END_PROGRAM and DATA_XFER_DONE both set), WinCE's own end condition.
 */
static int asic1_mmc_pio_write(struct asic1_mmc_host *host,
			       struct mmc_data *data, unsigned int timeout_us)
{
	const u16 err = ASIC1_MMC_STATUS_CRC_WRITE_ERR |
			ASIC1_MMC_STATUS_WR_CRC_ERR;
	unsigned int blk;
	bool last;
	int ret;

	for (blk = 0; blk < data->blocks; blk++) {
		last = blk + 1 == data->blocks;
		asic1_mmc_pio_progress(host, blk);

		ret = asic1_mmc_wait_status(host, ASIC1_MMC_STATUS_FIFO_EMPTY,
					    false, err, timeout_us);
		if (ret || READ_ONCE(host->pio_abort))
			return ret;

		ret = asic1_mmc_pio_write_block(host, data->blksz);
		if (ret)
			return ret;

		asic1_mmc_write(host, ASIC1_MMC_CLOCK_START,
				ASIC1_MMC_START_STOP_CLOCK);

		ret = asic1_mmc_wait_status(host,
					    last ? ASIC1_MMC_STATUS_END_PROGRAM |
						   ASIC1_MMC_STATUS_DATA_XFER_DONE :
						   ASIC1_MMC_STATUS_FIFO_EMPTY,
					    last, err, timeout_us);
		if (ret || READ_ONCE(host->pio_abort))
			return ret;
	}

	return 0;
}

/*
 * A transfer closed by CMD23 (mrq->sbc) needs its stop only after an
 * error, the rule __sdhci_finish_data() applies.
 */
static bool asic1_mmc_want_stop(struct mmc_request *mrq)
{
	return mrq->stop && (mrq->data->error || !mrq->sbc);
}

/*
 * Issue mrq->stop once the data phase is over, as mxcmci_datawork() does:
 * R1 after a read, R1b after a write, both mapped by start_cmd(). The
 * request completes from asic1_mmc_finish_stop().
 */
static void asic1_mmc_start_stop(struct asic1_mmc_host *host,
				 struct mmc_request *mrq)
{
	unsigned int timeout_ms;

	lockdep_assert_held(&host->lock);

	host->stop_active = true;
	timeout_ms = max_t(unsigned int, mrq->stop->busy_timeout,
			   ASIC1_MMC_REQ_TIMEOUT_MS);
	mod_timer(&host->req_timer, jiffies + msecs_to_jiffies(timeout_ms));
	asic1_mmc_start_cmd(host, mrq->stop, NULL, 0);
}

/*
 * Resolve the stop and leave the request for the caller to complete
 * after dropping host->lock. The data and command results stand as
 * the data phase left them.
 */
static void asic1_mmc_finish_stop(struct asic1_mmc_host *host,
				  struct mmc_request *mrq, int err)
{
	lockdep_assert_held(&host->lock);

	if (err)
		mrq->stop->error = err;
	else
		asic1_mmc_read_response(host, mrq->stop);

	timer_delete(&host->req_timer);
	host->stop_active = false;
	host->mrq = NULL;
	asic1_mmc_idle(host);
}

static void asic1_mmc_pio_work(struct work_struct *work)
{
	struct asic1_mmc_host *host = container_of(work, struct asic1_mmc_host,
						    pio_work);
	struct mmc_request *mrq;
	struct mmc_data *data;
	unsigned int timeout_us;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&host->lock, flags);
	mrq = host->mrq;
	if (!mrq || !mrq->data || !host->pio_active) {
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}
	spin_unlock_irqrestore(&host->lock, flags);

	data = mrq->data;
	timeout_us = max_t(unsigned int, ASIC1_MMC_DATA_TIMEOUT_US,
			   data->timeout_ns / NSEC_PER_USEC);

	/* Unlocked: see struct asic1_mmc_host's pio_active/pio_abort comment. */
	if (host->pio_dir == ASIC1_MMC_XFER_WRITE)
		ret = asic1_mmc_pio_write(host, data, timeout_us);
	else
		ret = asic1_mmc_pio_read(host, data, timeout_us);

	spin_lock_irqsave(&host->lock, flags);

	timer_delete(&host->req_timer);

	/* On an abort the error is already on the request. */
	if (!host->pio_abort) {
		if (ret)
			mrq->data->error = ret;
		else
			mrq->data->bytes_xfered = mrq->data->blksz * mrq->data->blocks;
	}

	host->pio_active = false;
	host->pio_abort = false;
	host->pio_dir = ASIC1_MMC_XFER_NONE;

	if (asic1_mmc_want_stop(mrq)) {
		asic1_mmc_start_stop(host, mrq);
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}

	host->mrq = NULL;
	asic1_mmc_idle(host);

	spin_unlock_irqrestore(&host->lock, flags);

	mmc_request_done(host->mmc, mrq);
}

/* ---------------------------------------------------------------------
 * Interrupt handler -- decodes the event from Status; InterruptMask only
 * chooses which sources may raise MMC_INT at all.
 * ---------------------------------------------------------------------
 */
static irqreturn_t asic1_mmc_irq(int irq, void *data)
{
	struct asic1_mmc_host *host = data;
	struct mmc_request *mrq;
	u16 status;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);

	/*
	 * Disarm first, as the 2.4 driver does. Nothing is written back to Status to
	 * acknowledge it: it is read-only and the CLOCK_STOP that opens the
	 * next command is the acknowledge.
	 */
	asic1_mmc_irq_disarm(host);

	status = asic1_mmc_read(host, ASIC1_MMC_STATUS);

	mrq = host->mrq;

	if (!mrq || !mrq->cmd) {
		asic1_mmc_idle(host);
		spin_unlock_irqrestore(&host->lock, flags);
		return IRQ_HANDLED;
	}

	/*
	 * The CLOCK_STOP opening the stop acknowledged what the data phase
	 * left in Status (file header), and host->lock was held from there
	 * to its CLOCK_START, so what is seen here is the stop's own.
	 */
	if (host->stop_active) {
		int err = 0;

		if (!(status & (ASIC1_MMC_STATUS_ERR_MASK |
				ASIC1_MMC_STATUS_CMD_RESP_END))) {
			asic1_mmc_irq_arm(host);
			spin_unlock_irqrestore(&host->lock, flags);
			return IRQ_HANDLED;
		}

		if (status & (ASIC1_MMC_STATUS_TIME_OUT_RESP |
			      ASIC1_MMC_STATUS_TIME_OUT_READ))
			err = -ETIMEDOUT;
		else if (status & ASIC1_MMC_STATUS_ERR_MASK)
			err = -EILSEQ;

		asic1_mmc_finish_stop(host, mrq, err);
		spin_unlock_irqrestore(&host->lock, flags);
		mmc_request_done(host->mmc, mrq);
		return IRQ_HANDLED;
	}

	if (status & ASIC1_MMC_STATUS_ERR_MASK) {
		if (status & (ASIC1_MMC_STATUS_TIME_OUT_RESP |
			      ASIC1_MMC_STATUS_TIME_OUT_READ))
			mrq->cmd->error = -ETIMEDOUT;
		else
			mrq->cmd->error = -EILSEQ;

		/* A running PIO work item completes the request itself. */
		if (host->pio_active) {
			mrq->data->error = mrq->cmd->error;
			WRITE_ONCE(host->pio_abort, true);
			spin_unlock_irqrestore(&host->lock, flags);
			return IRQ_HANDLED;
		}

		timer_delete(&host->req_timer);
		host->pio_dir = ASIC1_MMC_XFER_NONE;
		host->mrq = NULL;
		asic1_mmc_idle(host);
		spin_unlock_irqrestore(&host->lock, flags);
		mmc_request_done(host->mmc, mrq);
		return IRQ_HANDLED;
	}

	/*
	 * Nothing to act on yet: the command is still in flight, or its data
	 * phase is, which pio_work paces from Status itself, so re-arm and
	 * wait. CMD_RESP_END stays set through the data phase and reading the
	 * response again would pop ResFifo a second time and overwrite resp[].
	 */
	if (host->pio_active || !(status & ASIC1_MMC_STATUS_CMD_RESP_END)) {
		asic1_mmc_irq_arm(host);
		spin_unlock_irqrestore(&host->lock, flags);
		return IRQ_HANDLED;
	}

	/*
	 * Read the response as soon as CMD_RESP_END fires, whether or not the
	 * command carries data, in the order of mxcmci_cmd_done(). It is not
	 * known how ResFifo behaves across a following data burst, so reading it
	 * at the same point as the model driver is the safer choice.
	 */
	asic1_mmc_read_response(host, mrq->cmd);

	if (mrq->data && host->pio_dir != ASIC1_MMC_XFER_NONE) {
		host->pio_active = true;
		/* An error during the data phase must still reach us. */
		asic1_mmc_irq_arm(host);
		spin_unlock_irqrestore(&host->lock, flags);
		schedule_work(&host->pio_work);
		return IRQ_HANDLED;
	}

	timer_delete(&host->req_timer);
	host->mrq = NULL;
	asic1_mmc_idle(host);
	spin_unlock_irqrestore(&host->lock, flags);

	mmc_request_done(host->mmc, mrq);
	return IRQ_HANDLED;
}

/*
 * Callback of host->req_timer: no completion interrupt arrived within the
 * budget of the request. Every error and completion arrives as an
 * interrupt, and the Status bits of the data phase are taken from
 * references rather than confirmed on this silicon, so a wedged data engine
 * must not be able to hang mmc_wait_for_req() forever without a message in
 * the log.
 */
static void asic1_mmc_req_timeout(struct timer_list *t)
{
	struct asic1_mmc_host *host = timer_container_of(host, t, req_timer);
	struct mmc_request *mrq;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);

	mrq = host->mrq;
	if (!mrq) {
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}

	if (host->stop_active) {
		/*
		 * Pending again means this run fired for the data phase and
		 * lost host->lock to pio_work re-arming the timer for the stop.
		 */
		if (timer_pending(&host->req_timer)) {
			spin_unlock_irqrestore(&host->lock, flags);
			return;
		}

		asic1_mmc_finish_stop(host, mrq, -ETIMEDOUT);
		spin_unlock_irqrestore(&host->lock, flags);
		mmc_request_done(host->mmc, mrq);
		return;
	}

	dev_err(host->dev, "request timed out: CMD%u status=%#06x block=%u/%u\n",
		mrq->cmd->opcode, asic1_mmc_read(host, ASIC1_MMC_STATUS),
		host->pio_blk, mrq->data ? mrq->data->blocks : 0);

	mrq->cmd->error = -ETIMEDOUT;
	if (mrq->data)
		mrq->data->error = -ETIMEDOUT;

	/* Same rule as asic1_mmc_irq()'s error path. */
	if (host->pio_active) {
		WRITE_ONCE(host->pio_abort, true);
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}

	host->pio_dir = ASIC1_MMC_XFER_NONE;
	host->mrq = NULL;
	asic1_mmc_idle(host);

	spin_unlock_irqrestore(&host->lock, flags);

	mmc_request_done(host->mmc, mrq);
}

/* ---------------------------------------------------------------------
 * mmc_host_ops
 * ---------------------------------------------------------------------
 */
static void asic1_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct asic1_mmc_host *host = mmc_priv(mmc);
	unsigned long flags;
	u16 cmdcont = 0;
	unsigned int timeout_ms;

	spin_lock_irqsave(&host->lock, flags);

	if (WARN_ON(host->mrq)) {
		spin_unlock_irqrestore(&host->lock, flags);
		mrq->cmd->error = -EBUSY;
		mmc_request_done(mmc, mrq);
		return;
	}

	host->mrq = mrq;
	host->pio_dir = ASIC1_MMC_XFER_NONE;
	host->pio_active = false;
	host->pio_abort = false;
	host->pio_blk = 0;
	host->stop_active = false;

	if (mrq->data) {
		struct mmc_data *data = mrq->data;

		/*
		 * BlockLength and NumOfBlocks are programmed from
		 * asic1_mmc_start_cmd(), with the card clock stopped.
		 */
		host->pio_sg = data->sg;
		host->pio_bytes = 0;
		host->pio_remain = data->blksz * data->blocks;
		host->pio_dir = (data->flags & MMC_DATA_WRITE) ?
			ASIC1_MMC_XFER_WRITE : ASIC1_MMC_XFER_READ;

		cmdcont |= ASIC1_CMDCONT_DATA_ENABLE;
		if (data->flags & MMC_DATA_WRITE)
			cmdcont |= ASIC1_CMDCONT_WRITE;

		timeout_ms = max_t(unsigned int,
				   DIV_ROUND_UP(data->timeout_ns, NSEC_PER_MSEC),
				   ASIC1_MMC_REQ_TIMEOUT_MS);
	} else {
		timeout_ms = ASIC1_MMC_REQ_TIMEOUT_MS;
	}

	mod_timer(&host->req_timer, jiffies + msecs_to_jiffies(timeout_ms));

	asic1_mmc_start_cmd(host, mrq->cmd, mrq->data, cmdcont);

	spin_unlock_irqrestore(&host->lock, flags);
}

static void asic1_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct asic1_mmc_host *host = mmc_priv(mmc);

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		if (!IS_ERR(mmc->supply.vmmc))
			mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);
		break;
	case MMC_POWER_UP:
		if (!IS_ERR(mmc->supply.vmmc))
			mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, ios->vdd);
		break;
	default:
		break;
	}

	asic1_mmc_set_clock(host, ios->clock);
}

static const struct mmc_host_ops asic1_mmc_ops = {
	.request	= asic1_mmc_request,
	.set_ios	= asic1_mmc_set_ios,
	.get_ro		= mmc_gpio_get_ro,
	.get_cd		= mmc_gpio_get_cd,
};

/* ---------------------------------------------------------------------
 * probe
 * ---------------------------------------------------------------------
 */
static int asic1_mmc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct asic1_mmc_host *host;
	struct mmc_host *mmc;
	struct resource *res;
	int ret;

	mmc = devm_mmc_alloc_host(dev, sizeof(*host));
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	host->dev = dev;
	host->mmc = mmc;
	spin_lock_init(&host->lock);
	INIT_WORK(&host->pio_work, asic1_mmc_pio_work);
	timer_setup(&host->req_timer, asic1_mmc_req_timeout, 0);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	/*
	 * devm_ioremap() instead of devm_ioremap_resource(): the window of
	 * this cell is a sub-range of the ASIC2 window, which no in-window
	 * cell claims; see the file header.
	 */
	host->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!host->base)
		return -ENOMEM;

	/*
	 * The completion interrupt of the SD host is a SoC GPIO (MMC_INT),
	 * resolved through the of_node of this device, the "htc,asic1-mmc"
	 * child of the board, rather than through an ASIC2 hwirq.
	 */
	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0)
		return host->irq;

	host->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(host->clk))
		return dev_err_probe(dev, PTR_ERR(host->clk),
				     "failed to get the SD clock\n");

	ret = clk_prepare_enable(host->clk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable the SD clock\n");

	host->clk_rate = clk_get_rate(host->clk);
	if (!host->clk_rate) {
		ret = dev_err_probe(dev, -EINVAL, "SD clock reports 0 Hz\n");
		goto err_clk;
	}

	mmc->ops = &asic1_mmc_ops;
	mmc->f_min = host->clk_rate >> ASIC1_MMC_CLOCK_RATE_DIV_64;
	mmc->f_max = host->clk_rate;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	/*
	 * A block is moved through DataBuffer in one piece and FIFO_FULL marks
	 * 512 bytes in it, so no block can be larger. NumOfBlocks is a full
	 * 16-bit register with no narrower documented field, so the block
	 * count is the register width, not a confirmed hardware maximum.
	 * There is no DMA descriptor to size, as the scatterlist is walked one
	 * word at a time, so max_segs and max_seg_size have no hardware
	 * counterpart either.
	 */
	mmc->max_blk_size = 512;
	/* WinCE clamps one CMD18/CMD25 to 128 blocks. */
	mmc->max_blk_count = 128;
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_seg_size = mmc->max_req_size;
	mmc->max_segs = 128;

	ret = mmc_of_parse(mmc);
	if (ret)
		goto err_clk;

	ret = mmc_regulator_get_supply(mmc);
	if (ret)
		goto err_clk;

	/*
	 * mmc_of_parse() has already requested "cd-gpios" and "wp-gpios", so
	 * requesting them again here would fail the probe with -EBUSY. The
	 * card-detect line gives an edge interrupt for free: the gpio_chip of
	 * ASIC2 supports gpiod_to_irq(), which mmc_gpiod_request_cd_irq() uses
	 * from mmc_add_host(). Polling is only the fallback for a devicetree
	 * without cd-gpios.
	 */
	if (!mmc_host_can_gpio_cd(mmc) && !(mmc->caps & MMC_CAP_NONREMOVABLE))
		mmc->caps |= MMC_CAP_NEEDS_POLL;

	/*
	 * Both timeouts are loaded before the first command is issued, and
	 * reloaded by asic1_mmc_set_clock() on every clock change.
	 */
	asic1_mmc_write(host, ASIC1_MMC_RESPONSE_TIMEOUT_CLKS,
			ASIC1_MMC_RESPONSE_TIMEOUT);
	asic1_mmc_write(host, ASIC1_MMC_READ_TIMEOUT_CLKS, ASIC1_MMC_READ_TIMEOUT);

	/*
	 * Every source masked; asic1_mmc_start_cmd() unmasks exactly the one
	 * event it waits for.
	 */
	asic1_mmc_write(host, ASIC1_MMC_INT_MASK_ALL, ASIC1_MMC_INTERRUPT_MASK);

	asic1_mmc_write(host, ASIC1_MMC_CLOCK_STOP, ASIC1_MMC_START_STOP_CLOCK);

	ret = devm_request_irq(dev, host->irq, asic1_mmc_irq, 0,
			       dev_name(dev), host);
	if (ret) {
		ret = dev_err_probe(dev, ret, "failed to request IRQ %d\n", host->irq);
		goto err_clk;
	}

	/* Armed only while a command is outstanding. */
	spin_lock_irq(&host->lock);
	host->irq_armed = true;
	asic1_mmc_irq_disarm(host);
	spin_unlock_irq(&host->lock);

	platform_set_drvdata(pdev, host);

	ret = mmc_add_host(mmc);
	if (ret)
		goto err_clk;

	return 0;

err_clk:
	clk_disable_unprepare(host->clk);
	return ret;
}

static void asic1_mmc_remove(struct platform_device *pdev)
{
	struct asic1_mmc_host *host = platform_get_drvdata(pdev);

	mmc_remove_host(host->mmc);
	asic1_mmc_write(host, ASIC1_MMC_INT_MASK_ALL, ASIC1_MMC_INTERRUPT_MASK);
	asic1_mmc_write(host, ASIC1_MMC_CLOCK_STOP, ASIC1_MMC_START_STOP_CLOCK);

	/*
	 * pio_work re-arms the request timer, so it has to be stopped first.
	 * Both would otherwise run on the host freed with the device.
	 */
	cancel_work_sync(&host->pio_work);
	timer_delete_sync(&host->req_timer);

	/*
	 * Hand the descriptor back to devm_free_irq() with a balanced
	 * disable depth; every source is masked at the controller above,
	 * so re-arming here cannot deliver anything.
	 */
	spin_lock_irq(&host->lock);
	asic1_mmc_irq_arm(host);
	spin_unlock_irq(&host->lock);

	clk_disable_unprepare(host->clk);
}

static const struct of_device_id asic1_mmc_of_match[] = {
	{ .compatible = "htc,asic1-mmc" },
	{ }
};
MODULE_DEVICE_TABLE(of, asic1_mmc_of_match);

static struct platform_driver asic1_mmc_driver = {
	.driver = {
		.name		= "asic1-mmc",
		.of_match_table	= asic1_mmc_of_match,
	},
	.probe	= asic1_mmc_probe,
	.remove	= asic1_mmc_remove,
};
module_platform_driver(asic1_mmc_driver);

MODULE_ALIAS("platform:asic1-mmc");
MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("HTC ASIC1 SD/MMC host driver");
MODULE_LICENSE("GPL");
