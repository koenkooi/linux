// SPDX-License-Identifier: GPL-2.0-only
/*
 * HP iPAQ h39xx family suspend/resume support.
 *
 * The board's platform_suspend_ops are composed around the PXA core PM
 * entry points (pxa_pm_prepare(), pxa_pm_enter(), pxa_pm_finish()) and
 * installed with suspend_set_ops(), as sharpsl_pm.c and h5000_pm.c do. The
 * last caller wins, so this overrides the generic pxa_pm_ops registered by
 * pm.c. pxa_pm_prepare() is needed: it stores the physical address of
 * cpu_resume in PSPR, which the CPU is told to jump to when it wakes from
 * sleep, and pxa_pm_finish() clears it again.
 *
 * Nothing has to be preserved for the boot loader: CS0 is the flash that
 * holds it, not a companion-chip SRAM at the reset vector. ASIC2 and ASIC3
 * both sit on nCS_5, the high half of MSC2, which h3900.c programs at
 * boot. MSC2 is saved around pxa_pm_enter() and written back rather than
 * programmed from the constant a second time.
 *
 * The power button and the AC plug detect are wake sources through the
 * wakeup-source property of the gpio-keys node and the wake arming of
 * gpio-charger. On a devicetree boot a pad wakes on the edges its
 * interrupt was requested with, both edges for GPIO0 and GPIO8.
 * GPIO10, the ASIC2 interrupt line, could wake as well, but no ASIC2 cell
 * calls enable_irq_wake() or device_init_wakeup(), so
 * asic2_irq_set_wake() is never reached and nothing arms it. The RTC
 * alarm is armed through the generic PXA25x irq_set_wake path.
 *
 * Whether the boot loader's cold start honours PSPR and jumps to
 * cpu_resume is a question for the boot loader; if it always re-runs a
 * full bring-up, the board cannot resume from this side.
 *
 * Nothing here is a device: the suspend operations describe no hardware,
 * so they are installed from an initcall gated on the machine compatible,
 * after pm.c has installed the generic ones.
 *
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/suspend.h>

#include "pm.h"
#include "smemc.h"

/*
 * Restored before any device's resume runs, while IRQs are still off.
 */
static int h3900_pm_enter(suspend_state_t state)
{
	u32 msc2;
	int ret;

	msc2 = __raw_readl(MSC2);

	ret = pxa_pm_enter(state);

	__raw_writel(msc2, MSC2);

	return ret;
}

static const struct platform_suspend_ops h3900_suspend_ops = {
	.enter		= h3900_pm_enter,
	.prepare	= pxa_pm_prepare,
	.finish		= pxa_pm_finish,
	.valid		= suspend_valid_only_mem,
};

static int __init h3900_pm_init(void)
{
	if (!of_machine_is_compatible("hp,ipaq-h3900"))
		return 0;

	suspend_set_ops(&h3900_suspend_ops);

	return 0;
}
late_initcall(h3900_pm_init);
