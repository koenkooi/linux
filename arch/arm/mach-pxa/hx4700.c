// SPDX-License-Identifier: GPL-2.0-only
/*
 * Board setup for the HP iPAQ hx4700.
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * The board is a plain devicetree machine, the generic "marvell,pxa270"
 * one of arch/arm/mach-pxa/pxa-dt.c, and the pin muxing is the pin
 * controller's, from the devicetree. This file only writes the power
 * configuration register, for which there is no binding, from an initcall
 * gated behind of_machine_is_compatible() since the generic machine has no
 * per-board hook. No device is registered from here.
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>

#include "pxa2xx-regs.h"

static int __init hx4700_init(void)
{
	if (!of_machine_is_compatible("hp,ipaq-hx4700"))
		return 0;

	/*
	 * PCFR_PI2CEN hands GPIO3 and GPIO4 to the power I2C controller
	 * that talks to the MAX1587A core regulator. The stock OS sets it
	 * before every power I2C transaction, and nothing else sets it on
	 * a devicetree boot; without it every access times out waiting for
	 * the bus to become free.
	 */
	PCFR = PCFR_GPR_EN | PCFR_OPDE | PCFR_PI2CEN;

	return 0;
}
arch_initcall(hx4700_init);
