// SPDX-License-Identifier: GPL-2.0-only
/*
 * Board setup for the HP/Compaq iPAQ h39xx family (h3950, h3955, h3970,
 * h3975).
 *
 * Copyright 2000-2003 Hewlett-Packard Company.
 * Copyright 2003-2005 Phil Blundell
 * Copyright (C) 2026 Koen Kooi
 *
 * The board is a plain devicetree machine, the generic "marvell,pxa250"
 * one of arch/arm/mach-pxa/pxa-dt.c, and the pin muxing is the pin
 * controller's, from the devicetree. This file only programs the nCS_5
 * timing, for which there is no binding, from an initcall gated behind
 * of_machine_is_compatible() since the generic machine has no per-board
 * hook. No device is registered from here.
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>

#include "smemc.h"

/*
 * RBUFF=0, RRR=7, RDN=4, RDF=0xa, RT=100: nCS_5 in variable latency I/O
 * mode, which is how ASIC2 and ASIC3 are wired. The ROM type and bus width
 * are the bus protocol, not a timing margin; a different value there
 * stops the companion chips from answering. The low half of MSC2 is nCS_4
 * and stays as the boot loader left it.
 */
#define H3900_MSC2_NCS5		0x74a4

/*
 * nCS_5 has no bus error timeout, so the timing has to be in place before
 * the companion chip drivers bind at device_initcall.
 */
static int __init h3900_init(void)
{
	if (!of_machine_is_compatible("hp,ipaq-h3900"))
		return 0;

	__raw_writel((__raw_readl(MSC2) & 0x0000ffff) |
		     (H3900_MSC2_NCS5 << 16), MSC2);

	return 0;
}
arch_initcall(h3900_init);
