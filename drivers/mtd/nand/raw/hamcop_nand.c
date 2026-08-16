// SPDX-License-Identifier: GPL-2.0-only
/*
 * NAND flash controller driver for the Samsung HAMCOP companion ASIC
 * (HP iPAQ h2200/h2210/h2212/h2215).
 *
 * HAMCOP's NAND block is the same S3C24xx-generation "SAMCOP" IP as the
 * one on-die on S3C2410/S3C2412/S3C2440 SoCs (see drivers/mfd/hamcop.c's
 * file comment) -- CMD/ADDR/DATA latch registers, a CONT0 mode/chip-enable/
 * ECC-init register, and a 3-byte-per-512-byte Hamming ECC generator read
 * back from ECCL0/ECCL1. This driver is a from-scratch rewrite against the
 * current ->exec_op() controller interface, not a port of the old
 * out-of-tree hamcop_nand.c (Matt Reimer, 2004-2006) -- that driver
 * predates ->exec_op() entirely (chip->cmdfunc()/read_byte()-era API) and
 * its shape doesn't survive the switch. What *does* transfer verbatim,
 * because it's real hardware behaviour rather than API shape: the register
 * offsets and bit values (cross-checked against the old out-of-tree
 * ipaq-hamcop.h), the WinCE-derived CONF0 timing
 * value, and the single-bit ECC correction algorithm -- ported from the
 * deleted mainline drivers/mtd/nand/raw/s3c2410.c
 * (s3c2410_nand_correct_data(), same Samsung ECC IP family), which
 * corrects a single-bit error instead of failing on any mismatch.
 * Structural template (exec_op() built from raw CMD/ADDR/DATA
 * register writes plus a polled ready register, no RnB interrupt) is this
 * tree's own drivers/mtd/nand/raw/technologic-nand-controller.c.
 *
 * Polling-only: RnB interrupt handling was never written even in the 2007
 * driver ("irq = -1, /\* XXX until RnB interrupt code is written *\/"), so
 * this cell has no IRQ resource and NAND_OP_WAITRDY_INSTR always polls the
 * FLASH_RNB status bit directly.
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * The single-bit error test and error position decoding in
 * hamcop_nand_correct_data() are taken from s3c2410_nand_correct_data()
 * of the s3c2410 NAND driver:
 * Copyright (C) 2004-2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/rawnand.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>

/*
 * Register offsets, relative to this driver's own resource mapping (the
 * "hamcop-nand" mfd_cell's MEM resource, HAMCOP_NF_BASE..+HAMCOP_NF_SIZE
 * from include/linux/mfd/hamcop.h) -- not to HAMCOP's chip-wide base.
 * hamcop.h leaves these undefined, so they live here
 * instead.
 * Values translated from ipaq-hamcop.h's _HAMCOP_NF_* (chip-absolute
 * 0x5000-relative offsets there; this cell's window already starts at
 * that base, so they're used as-is once re-based to 0).
 */
#define HAMCOP_NF_CONF0			0x0000
#define HAMCOP_NF_CONT0			0x0008
#define HAMCOP_NF_CONT1			0x000c
#define HAMCOP_NF_CMMD			0x0010
#define HAMCOP_NF_ADDR0			0x0014
#define HAMCOP_NF_DATA			0x001c
#define HAMCOP_NF_ECCL0			0x0020
#define HAMCOP_NF_ECCL1			0x0024
#define HAMCOP_NF_STAT1			0x0034

/*
 * CONF0 timing value "from wince via haret" (the old out-of-tree h2200.c
 * comment: "PocketPC uses 0x1213"). The three fields below (TACLS/TWRPH0/
 * TWRPH1) are the only writable bits in that value -- 0x1213's remaining
 * bits (PAGESIZE/ADDRCYCLE/BUSWIDTH) are read-only status reflecting the
 * attached flash chip's own properties, not something software sets, so
 * OR'ing only the writable fields together (0x1210) has the same effect
 * as writing WinCE's full dumped value. No datasheet exists to derive
 * these cycle counts from a clock rate instead (c.f. s3c2410.c's
 * s3c2410_nand_setrate(), not portable here without one) -- ported
 * verbatim as the only real-hardware-verified source available.
 */
#define HAMCOP_NF_CONF0_TACLS_1		0x1000
#define HAMCOP_NF_CONF0_TWRPH0_2	0x0200
#define HAMCOP_NF_CONF0_TWRPH1_1	0x0010

#define HAMCOP_NF_CONT0_WP_EN		BIT(12)
#define HAMCOP_NF_CONT0_INITECC		BIT(11)
#define HAMCOP_NF_CONT0_NFNCE		BIT(10)	/* 0 = flash selected */
#define HAMCOP_NF_CONT0_MODE_NFMODE	0x3

#define HAMCOP_NF_STAT1_FLASH_RNB	BIT(0)	/* 1 = ready */

/* Matches the old out-of-tree h2200_partitions[]: HTC's first-stage
 * bootstrap and rescue bootloader are read-only from Linux (the only
 * copy in flash -- h2200_pm.c's SRAM dance is a resume-vector patch, not
 * a substitute for this image), the rest is ours to use.
 */
static const struct mtd_partition hamcop_nand_partitions[] = {
	{
		.name		= "HTC first-stage bootstrap",
		.offset		= 0,
		.size		= SZ_16K,
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name		= "HTC rescue bootloader",
		.offset		= MTDPART_OFS_NXTBLK,
		.size		= 144 * SZ_1K,
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name		= "second-stage loader",
		.offset		= MTDPART_OFS_NXTBLK,
		.size		= 1280 * SZ_1K,
	},
	{
		.name		= "root fs",
		.offset		= MTDPART_OFS_NXTBLK,
		.size		= MTDPART_SIZ_FULL,
	},
};

struct hamcop_nand {
	struct nand_controller	controller;
	struct nand_chip	chip;
	struct clk		*clk;
	void __iomem		*regs;
	struct device		*dev;
};

static struct hamcop_nand *to_hamcop_nand(struct nand_chip *chip)
{
	return container_of(chip, struct hamcop_nand, chip);
}

static u16 hamcop_nand_read(struct hamcop_nand *hn, u32 reg)
{
	return readw_relaxed(hn->regs + reg);
}

static void hamcop_nand_write(struct hamcop_nand *hn, u32 reg, u16 val)
{
	writew_relaxed(val, hn->regs + reg);
}

static void hamcop_nand_select(struct hamcop_nand *hn, bool select)
{
	u16 cont0 = hamcop_nand_read(hn, HAMCOP_NF_CONT0);

	if (select)
		cont0 &= ~HAMCOP_NF_CONT0_NFNCE;
	else
		cont0 |= HAMCOP_NF_CONT0_NFNCE;

	hamcop_nand_write(hn, HAMCOP_NF_CONT0, cont0);
}

/*
 * tR for a page read is about 10-25us, so the ready bit is usually already
 * set when software starts polling. Spin for a short bounded time first and
 * only fall back to the sleeping poll for the long program/erase waits.
 */
#define HAMCOP_NF_WAITRDY_SPIN_US	20

static int hamcop_nand_waitrdy(struct hamcop_nand *hn, unsigned int timeout_ms)
{
	u16 stat1;
	int ret;

	ret = readw_poll_timeout_atomic(hn->regs + HAMCOP_NF_STAT1, stat1,
					stat1 & HAMCOP_NF_STAT1_FLASH_RNB,
					1, HAMCOP_NF_WAITRDY_SPIN_US);
	if (!ret)
		return 0;

	return readw_poll_timeout(hn->regs + HAMCOP_NF_STAT1, stat1,
				   stat1 & HAMCOP_NF_STAT1_FLASH_RNB,
				   10, timeout_ms * 1000);
}

static int hamcop_nand_exec_instr(struct hamcop_nand *hn,
				   const struct nand_op_instr *instr)
{
	unsigned int i;

	switch (instr->type) {
	case NAND_OP_CMD_INSTR:
		hamcop_nand_write(hn, HAMCOP_NF_CMMD, instr->ctx.cmd.opcode);
		break;

	case NAND_OP_ADDR_INSTR:
		for (i = 0; i < instr->ctx.addr.naddrs; i++)
			hamcop_nand_write(hn, HAMCOP_NF_ADDR0,
					   instr->ctx.addr.addrs[i]);
		break;

	case NAND_OP_DATA_IN_INSTR:
		ioread8_rep(hn->regs + HAMCOP_NF_DATA,
			    instr->ctx.data.buf.in, instr->ctx.data.len);
		break;

	case NAND_OP_DATA_OUT_INSTR:
		iowrite8_rep(hn->regs + HAMCOP_NF_DATA,
			     instr->ctx.data.buf.out, instr->ctx.data.len);
		break;

	case NAND_OP_WAITRDY_INSTR:
		return hamcop_nand_waitrdy(hn, instr->ctx.waitrdy.timeout_ms);

	default:
		return -EINVAL;
	}

	if (instr->delay_ns)
		ndelay(instr->delay_ns);

	return 0;
}

static int hamcop_nand_exec_op(struct nand_chip *chip,
				const struct nand_operation *op,
				bool check_only)
{
	struct hamcop_nand *hn = to_hamcop_nand(chip);
	unsigned int i;
	int ret = 0;

	if (check_only)
		return 0;

	hamcop_nand_select(hn, true);

	for (i = 0; i < op->ninstrs; i++) {
		ret = hamcop_nand_exec_instr(hn, &op->instrs[i]);
		if (ret)
			break;
	}

	hamcop_nand_select(hn, false);

	return ret;
}

/*
 * ECC: HAMCOP's NF block computes a 3-byte Hamming code per 512-byte step,
 * same as the on-die generator on S3C2410/S3C2412/S3C2440 (drivers/mfd/
 * hamcop.c's file comment -- HAMCOP is S3C24xx-class IP). hwctl's mode
 * argument is ignored, matching both the 2007 driver and the s3c2410.c
 * reference: the ECC-init bit resets the accumulator the same way whether
 * the following transfer is a read or a write.
 *
 * The generator, read back and packed the way hamcop_nand_calculate_ecc()
 * below does it, is bit-for-bit mainline's own software Hamming in
 * SmartMedia byte order -- ecc_sw_hamming_calculate(buf, 512, code,
 * sm_order = true) in drivers/mtd/nand/ecc-sw-hamming.c. That was
 * established against real hardware output, not assumed: recomputing that
 * software ECC over a physically-verified read of all 2560 pages of this
 * device's "second-stage loader" partition reproduces every ECC decision
 * this driver made on the same flash, exactly (see
 * hamcop_nand_correct_data()). Worth knowing because it pins down the
 * readback and packing here as correct, and because the software fallback
 * below is only equivalent to this generator in SmartMedia order -- its
 * plain byte order produces a different code over the same data.
 */
static void hamcop_nand_hwctl(struct nand_chip *chip, int mode)
{
	struct hamcop_nand *hn = to_hamcop_nand(chip);

	hamcop_nand_write(hn, HAMCOP_NF_CONT0,
			   hamcop_nand_read(hn, HAMCOP_NF_CONT0) |
			   HAMCOP_NF_CONT0_INITECC);
}

static int hamcop_nand_calculate_ecc(struct nand_chip *chip, const u8 *dat,
				      u8 *ecc_code)
{
	struct hamcop_nand *hn = to_hamcop_nand(chip);
	u16 eccl0 = hamcop_nand_read(hn, HAMCOP_NF_ECCL0);

	ecc_code[0] = eccl0 & 0xff;
	ecc_code[1] = (eccl0 >> 8) & 0xff;
	ecc_code[2] = hamcop_nand_read(hn, HAMCOP_NF_ECCL1) & 0xff;

	return 0;
}

/*
 * Single-bit correction, ported from the deleted mainline s3c2410.c's
 * s3c2410_nand_correct_data() (same Samsung 3-byte/512-byte Hamming ECC
 * family HAMCOP's NF block belongs to) rather than the old out-of-tree
 * hamcop_nand_correct_data(), which was a stub that only ever compared
 * the two ECCs and returned -1 on any mismatch -- real correction was
 * never implemented in 2007. The row/column parity check below can fix
 * exactly one flipped bit per 512-byte step; anything else is reported
 * uncorrectable.
 */
static int hamcop_nand_correct_data(struct nand_chip *chip, u8 *dat,
				     u8 *read_ecc, u8 *calc_ecc)
{
	unsigned int diff0, diff1, diff2;
	unsigned int bit, byte;

	/*
	 * A blank stored ECC means the page is unprotected, not correctable.
	 * Factory images on this NAND carry no ECC, so the OOB bytes holding
	 * it are erased. Against read_ecc = ff ff ff the difference is just
	 * ~calc_ecc, and since each parity bit is paired with its complement
	 * the single-bit pattern test below passes for every block of odd
	 * parity, roughly half of all data. Correcting would then flip a bit
	 * of good data and report success.
	 *
	 * Return "clean" rather than -EBADMSG: the raw data is the best
	 * answer available, and failing the read would make those pages
	 * unreadable instead of merely unverified. Pages written by this
	 * driver always get an ECC.
	 */
	if (read_ecc[0] == 0xff && read_ecc[1] == 0xff && read_ecc[2] == 0xff) {
		dev_warn_once(to_hamcop_nand(chip)->dev,
			      "no stored ECC (blank OOB) -- data returned unverified\n");
		return 0;
	}

	diff0 = read_ecc[0] ^ calc_ecc[0];
	diff1 = read_ecc[1] ^ calc_ecc[1];
	diff2 = read_ecc[2] ^ calc_ecc[2];

	if (!diff0 && !diff1 && !diff2)
		return 0;

	/* Row/column parity check: exactly one bit differs in each byte
	 * of the pairwise XOR iff there's a single correctable bit error.
	 */
	if (((diff0 ^ (diff0 >> 1)) & 0x55) == 0x55 &&
	    ((diff1 ^ (diff1 >> 1)) & 0x55) == 0x55 &&
	    ((diff2 ^ (diff2 >> 1)) & 0x55) == 0x55) {
		bit  = ((diff2 >> 3) & 1) |
		       ((diff2 >> 4) & 2) |
		       ((diff2 >> 5) & 4);

		byte = ((diff2 << 7) & 0x100) |
		       ((diff1 << 0) & 0x80)  |
		       ((diff1 << 1) & 0x40)  |
		       ((diff1 << 2) & 0x20)  |
		       ((diff1 << 3) & 0x10)  |
		       ((diff0 >> 4) & 0x08)  |
		       ((diff0 >> 3) & 0x04)  |
		       ((diff0 >> 2) & 0x02)  |
		       ((diff0 >> 1) & 0x01);

		dat[byte] ^= (1 << bit);
		return 1;
	}

	/* A single bit set across the combined ECC most likely means the
	 * error is in the ECC bytes themselves, not the data.
	 */
	diff0 |= (diff1 << 8) | (diff2 << 16);
	if ((diff0 & (diff0 - 1)) == 0)
		return 1;

	return -EBADMSG;
}

static int hamcop_nand_attach_chip(struct nand_chip *chip)
{
	switch (chip->ecc.engine_type) {
	case NAND_ECC_ENGINE_TYPE_ON_HOST:
		chip->ecc.size = 512;
		chip->ecc.bytes = 3;
		chip->ecc.strength = 1;
		chip->ecc.hwctl = hamcop_nand_hwctl;
		chip->ecc.calculate = hamcop_nand_calculate_ecc;
		chip->ecc.correct = hamcop_nand_correct_data;
		break;

	case NAND_ECC_ENGINE_TYPE_SOFT:
		if (chip->ecc.algo == NAND_ECC_ALGO_UNKNOWN)
			chip->ecc.algo = NAND_ECC_ALGO_HAMMING;
		break;

	default:
		break;
	}

	return 0;
}

static const struct nand_controller_ops hamcop_nand_controller_ops = {
	.attach_chip	= hamcop_nand_attach_chip,
	.exec_op	= hamcop_nand_exec_op,
};

/*
 * Bring the NF block into the same idle state the 2007 driver's own
 * hamcop_nand_up()/reset dance ended in: controller enabled in NAND mode,
 * WinCE flash timing, no RnB interrupts, chip deselected. Unlike 2007,
 * there's no manual NAND_CMD_RESET/poll loop here -- nand_scan() issues
 * that generically through ->exec_op() once this driver is wired up.
 */
static void hamcop_nand_inithw(struct hamcop_nand *hn)
{
	hamcop_nand_write(hn, HAMCOP_NF_CONF0,
			   HAMCOP_NF_CONF0_TACLS_1 | HAMCOP_NF_CONF0_TWRPH0_2 |
			   HAMCOP_NF_CONF0_TWRPH1_1);

	hamcop_nand_write(hn, HAMCOP_NF_CONT1, 0);

	hamcop_nand_write(hn, HAMCOP_NF_CONT0,
			   HAMCOP_NF_CONT0_WP_EN | HAMCOP_NF_CONT0_NFNCE |
			   HAMCOP_NF_CONT0_MODE_NFMODE);
}

static int hamcop_nand_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hamcop_nand *hn;
	struct mtd_info *mtd;
	struct resource *mem;
	int ret;

	hn = devm_kzalloc(dev, sizeof(*hn), GFP_KERNEL);
	if (!hn)
		return -ENOMEM;

	hn->dev = dev;

	/*
	 * platform_get_resource() + devm_ioremap(), not
	 * devm_platform_ioremap_resource() -- the latter reserves the
	 * region exclusively via devm_request_mem_region(), which always
	 * loses to drivers/mfd/hamcop.c's own devm_ioremap_resource() of
	 * the entire HAMCOP MMIO window with -EBUSY.
	 * HAMCOP_NF_BASE (0x1000) is a sub-range of that same
	 * already-claimed window.
	 */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -ENODEV;
	hn->regs = devm_ioremap(dev, mem->start, resource_size(mem));
	if (!hn->regs)
		return -ENOMEM;

	/*
	 * A real clk_get() consumer, matching drivers/mfd/hamcop.c's
	 * "hamcop-nand" clkdev alias for HAMCOP_GATE_NAND -- but not what
	 * keeps that gate powered: it's marked CLK_IS_CRITICAL in the
	 * parent MFD because it also powers HAMCOP's SRAM block (see that
	 * driver's comment), a duty that has to hold regardless of whether
	 * this driver is even built, let alone bound. This clk_get() is
	 * ordinary refcounting hygiene on top of that, not a replacement
	 * for it.
	 */
	hn->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(hn->clk))
		return dev_err_probe(dev, PTR_ERR(hn->clk),
				      "failed to get nand clock\n");

	nand_controller_init(&hn->controller);
	hn->controller.ops = &hamcop_nand_controller_ops;
	hn->chip.controller = &hn->controller;

	mtd = nand_to_mtd(&hn->chip);
	mtd->name = "hamcop-nand";
	mtd->dev.parent = dev;

	platform_set_drvdata(pdev, hn);

	hamcop_nand_inithw(hn);

	/*
	 * HAMCOP always has real hardware ECC available -- no devicetree
	 * property and no platform_data ever overrides this, so there's
	 * exactly one policy to express, set unconditionally before
	 * nand_scan().
	 */
	hn->chip.ecc.engine_type = NAND_ECC_ENGINE_TYPE_ON_HOST;

	ret = nand_scan(&hn->chip, 1);
	if (ret)
		return dev_err_probe(dev, ret, "NAND scan failed\n");

	ret = mtd_device_register(mtd, hamcop_nand_partitions,
				   ARRAY_SIZE(hamcop_nand_partitions));
	if (ret) {
		nand_cleanup(&hn->chip);
		return ret;
	}

	return 0;
}

static void hamcop_nand_remove(struct platform_device *pdev)
{
	struct hamcop_nand *hn = platform_get_drvdata(pdev);

	WARN_ON(mtd_device_unregister(nand_to_mtd(&hn->chip)));
	nand_cleanup(&hn->chip);
}

static const struct of_device_id hamcop_nand_of_match[] = {
	{ .compatible = "samsung,hamcop-nand" },
	{ }
};
MODULE_DEVICE_TABLE(of, hamcop_nand_of_match);

static struct platform_driver hamcop_nand_driver = {
	.driver = {
		.name = "hamcop-nand",
		.of_match_table = hamcop_nand_of_match,
	},
	.probe	= hamcop_nand_probe,
	.remove	= hamcop_nand_remove,
};
module_platform_driver(hamcop_nand_driver);

MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("Samsung HAMCOP NAND controller driver (HP iPAQ h2200)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:hamcop-nand");
