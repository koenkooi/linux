// SPDX-License-Identifier: GPL-2.0-only
/*
 * Support for the HP iPAQ h3800 (h38xx) family handheld computers
 * (h3815/h3830/h3835/h3845/h3850/h3870/h3875), SA-1110-based.
 *
 * Devicetree-only: the machine is matched by its root compatible rather
 * than by a legacy mach-types ID.
 *
 * h3xxx_map_io() is shared with h3600: its three map_desc entries (CS2,
 * CS4, H3600_EGPIO_PHYS) cover the windows h3800 needs too.
 * h3xxx_mach_init() is not reused: its htc-egpio platform
 * device, ipaq-h3xxx-micro on UART1 and gpio-keys on GPIO18 are h36xx-only,
 * and GPIO18 is h3800's MMC_INT, the ASIC1 SD host interrupt. Reusing it
 * would register a gpio-keys consumer on the line the SD host needs.
 */

#include <linux/init.h>
#include <linux/kernel.h>

#include <asm/mach/arch.h>

#include <mach/h3xxx.h>
#include <mach/irqs.h>

#include "generic.h"

static const char * const h3800_dt_compat[] __initconst = {
	"hp,ipaq-h3800",
	NULL,
};

DT_MACHINE_START(H3800_DT, "HP iPAQ h3800")
	.map_io		= h3xxx_map_io,
	.nr_irqs	= SA1100_NR_IRQS,
	.init_late	= sa11x0_init_late,
	.restart	= sa11x0_restart,
	.dt_compat	= h3800_dt_compat,
MACHINE_END
