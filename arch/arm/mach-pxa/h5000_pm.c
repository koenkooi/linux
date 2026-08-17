// SPDX-License-Identifier: GPL-2.0-only
/*
 * HP iPAQ h5000 family suspend/resume support.
 *
 * The board's platform_suspend_ops are composed around the PXA core PM
 * entry points (pxa_pm_prepare(), pxa_pm_enter(), pxa_pm_finish()) and
 * installed with suspend_set_ops(), as sharpsl_pm.c does. The last caller
 * wins, so this overrides the generic pxa_pm_ops registered by pm.c.
 * pxa_pm_prepare() is needed: it stores the physical address of
 * cpu_resume in PSPR, which the CPU is told to jump to when it wakes from
 * sleep, and pxa_pm_finish() clears it again.
 *
 * Unlike the h2200, nothing has to be preserved for the boot loader:
 * CS0 is the flash that holds it, not a companion-chip SRAM at the reset
 * vector. What this file owns is MSC0-2 and MDREFR, which the boot loader's
 * wake path rewrites before it jumps through PSPR. They are saved around
 * pxa_pm_enter() and written back, rather than programmed from constants a
 * second time.
 *
 * The SAMCOP's own state (IC_INTMSK, ClockControl) is saved and restored by
 * the dev_pm_ops of drivers/mfd/samcop.c. The power button is a wake source
 * through the wakeup-source property of its gpio-keys node and the
 * WAKEUP_ON_EDGE_BOTH marker of GPIO0 in the pin table of h5000.c; the RTC
 * alarm is armed through the generic PXA25x irq_set_wake path.
 *
 * Nothing here is a device: the suspend operations describe no hardware,
 * so they are installed from an initcall gated on the machine compatible,
 * after pm.c has installed the generic ones.
 *
 * Copyright (C) 2026 Koen Kooi
 */

#define pr_fmt(fmt) "h5000-pm: " fmt

#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/suspend.h>

#include "pm.h"
#include "pxa2xx-regs.h"
#include "smemc.h"

static u32 h5000_read_clkcfg(void)
{
	u32 clkcfg;

	asm volatile("mrc p14, 0, %0, c6, c0, 0" : "=r" (clkcfg));
	return clkcfg;
}

/*
 * Restore before any device's resume runs (IRQs are still off here);
 * bootldr's wake path rewrites MSC0/MSC2 and MDREFR before it jumps
 * through PSPR.
 *
 * MDREFR_K1DB2 is taken from the wake value, not the saved one:
 * SDCLK1's divider is only changed together with a frequency change
 * sequence elsewhere (pxa2xx_cpll_change()), and the saved word carries
 * it set when sleep was entered from the 99.5 MHz cpufreq row while
 * bootldr comes back with it clear. Clear is SDRAM at the plain memory
 * clock, which every other row uses and which the saved DRI is already
 * computed for. All other MDREFR fields, K2DB2/K2FREE included, still
 * come from the saved word.
 *
 * The clock and memory controller state on both sides of the sleep is
 * logged at debug level for the first hardware tests.
 */
static int h5000_pm_enter(suspend_state_t state)
{
	u32 msc0, msc1, msc2, mdrefr, mdcnfg, cccr, clkcfg;
	u32 w_msc0, w_msc1, w_msc2, w_mdrefr, w_mdcnfg, w_cccr, w_clkcfg;
	int ret;

	cccr = __raw_readl(CCCR);
	clkcfg = h5000_read_clkcfg();
	mdcnfg = __raw_readl(MDCNFG);
	msc0 = __raw_readl(MSC0);
	msc1 = __raw_readl(MSC1);
	msc2 = __raw_readl(MSC2);
	mdrefr = __raw_readl(MDREFR);

	ret = pxa_pm_enter(state);

	w_cccr = __raw_readl(CCCR);
	w_clkcfg = h5000_read_clkcfg();
	w_mdcnfg = __raw_readl(MDCNFG);
	w_msc0 = __raw_readl(MSC0);
	w_msc1 = __raw_readl(MSC1);
	w_msc2 = __raw_readl(MSC2);
	w_mdrefr = __raw_readl(MDREFR);

	__raw_writel(msc0, MSC0);
	__raw_writel(msc1, MSC1);
	__raw_writel(msc2, MSC2);
	__raw_writel((mdrefr & ~MDREFR_K1DB2) | (w_mdrefr & MDREFR_K1DB2),
		     MDREFR);

	pr_debug("sleep: CCCR %08x CLKCFG %08x MDREFR %08x MDCNFG %08x MSC0-2 %08x %08x %08x\n",
		 cccr, clkcfg, mdrefr, mdcnfg, msc0, msc1, msc2);
	pr_debug("wake:  CCCR %08x CLKCFG %08x MDREFR %08x MDCNFG %08x MSC0-2 %08x %08x %08x\n",
		 w_cccr, w_clkcfg, w_mdrefr, w_mdcnfg, w_msc0, w_msc1, w_msc2);
	if ((mdrefr ^ w_mdrefr) & MDREFR_K1DB2)
		pr_debug("MDREFR K1DB2 %u before sleep, %u on wake; keeping the wake state\n",
			 !!(mdrefr & MDREFR_K1DB2), !!(w_mdrefr & MDREFR_K1DB2));

	return ret;
}

static const struct platform_suspend_ops h5000_suspend_ops = {
	.enter		= h5000_pm_enter,
	.prepare	= pxa_pm_prepare,
	.finish		= pxa_pm_finish,
	.valid		= suspend_valid_only_mem,
};

static int __init h5000_pm_init(void)
{
	if (!of_machine_is_compatible("hp,ipaq-h5000"))
		return 0;

	suspend_set_ops(&h5000_suspend_ops);

	return 0;
}
late_initcall(h5000_pm_init);
