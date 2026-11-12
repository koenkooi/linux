/* SPDX-License-Identifier: GPL-2.0 */
#ifndef MFD_TMIO_H
#define MFD_TMIO_H

#include <linux/platform_device.h>
#include <linux/types.h>

/* TMIO MMC platform flags */

/*
 * Card presence is reported in the "_A" status bits (CARD_REMOVE_A,
 * CARD_INSERT_A, SIGSTATE_A) rather than the usual CARD_REMOVE, CARD_INSERT
 * and SIGSTATE.
 */
#define TMIO_MMC_CD_IN_ALT_BITS		BIT(0)

/*
 * Some controllers can support a 2-byte block size when the bus width is
 * configured in 4-bit mode.
 */
#define TMIO_MMC_BLKSZ_2BYTES		BIT(1)

/* Some controllers can support SDIO IRQ signalling */
#define TMIO_MMC_SDIO_IRQ		BIT(2)

/* Some features are only available or tested on R-Car Gen2 or later */
#define TMIO_MMC_MIN_RCAR2		BIT(3)

/*
 * Some controllers require waiting for the SD bus to become idle before
 * writing to some registers.
 */
#define TMIO_MMC_HAS_IDLE_WAIT		BIT(4)

/*
 * Use the busy timeout feature. Probably all TMIO versions support it. Yet,
 * we don't have documentation for old variants, so we enable only known good
 * variants with this flag. Can be removed once all variants are known good.
 */
#define TMIO_MMC_USE_BUSY_TIMEOUT	BIT(5)

/*
 * The host's interrupt reaches the CPU through an edge-only parent
 * interrupt controller, so every unmasked source must be consumed before
 * the handler returns or the remaining ones are lost. CTL_STATUS is a
 * level-style aggregate behind a single output pin: acking one source
 * while another is still asserted leaves that pin low, and a parent that
 * can only detect edges never reports the survivor. Set by ASIC3 for
 * boards whose SD interrupt is a raw PXA GPIO.
 */
#define TMIO_MMC_EDGE_IRQ_PARENT	BIT(6)

/* Some controllers have CMD12 automatically issue/non-issue register */
#define TMIO_MMC_HAVE_CMD12_CTRL	BIT(7)

/* Controller has some SDIO status bits which must be 1 */
#define TMIO_MMC_SDIO_STATUS_SETBITS	BIT(8)

/* Some controllers have a 32-bit wide data port register */
#define TMIO_MMC_32BIT_DATA_PORT	BIT(9)

/* Some controllers allows to set SDx actual clock */
#define TMIO_MMC_CLK_ACTUAL		BIT(10)

/* Some controllers have a CBSY bit */
#define TMIO_MMC_HAVE_CBSY		BIT(11)

/* Some controllers have a 64-bit wide data port register */
#define TMIO_MMC_64BIT_DATA_PORT	BIT(12)

/*
 * The hardware auto-stop (CTL_STOP_INTERNAL_ACTION = TMIO_STOP_SEC) does not
 * execute when the multi-block transfer it was armed for ended in error, so
 * the card is left in read/write-multiple state and the response register
 * holds nothing meaningful. The host driver must then issue a real CMD12.
 */
#define TMIO_MMC_NO_AUTO_CMD12_ON_ERROR	BIT(13)

struct tmio_mmc_data {
	void				*chan_priv_tx;
	void				*chan_priv_rx;
	unsigned int			hclk;
	unsigned long			capabilities;
	unsigned long			capabilities2;
	unsigned long			flags;
	u32				ocr_mask;	/* available voltages */
	dma_addr_t			dma_rx_offset;
	unsigned int			max_blk_count;
	unsigned short			max_segs;
};
#endif
