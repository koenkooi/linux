// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung SAMCOP companion ASIC MFD driver (HP iPAQ h51xx/h54xx/h55xx)
 *
 * SAMCOP hangs off PXA25x static-memory chip select 5 as a plain 32-bit
 * VLIO-mode MMIO device with a single upstream GPIO interrupt line
 * (PXA GPIO10). It bundles a USB host controller, an ADC + resistive
 * touchscreen front-end, a DS1WM-compatible 1-Wire master, a 5-channel LED
 * blink controller, a five-bank GPIO block, PCMCIA/expansion-pack status
 * glue, and 32 KiB of on-chip SRAM used as a DMA bounce pool.
 *
 * The shape follows drivers/mfd/hamcop.c, but SAMCOP shares no code with
 * HAMCOP and three things do not transfer:
 *
 *  1. The IRQ tree is three levels deep, not two. The IC block has 26 flat
 *     sources, and two of them are themselves demux points: source 25 (GPIO)
 *     fans out to a sparse 14-line subset of GPA/GPB pins, and source 5
 *     (PCMCIA) fans out to 6 EPS status lines. The driver builds one linear
 *     irq_domain of SAMCOP_NR_IRQS (46) hwirqs -- IC 0-25, GPIO 26-39, EPS
 *     40-45 -- with the nesting expressed as two chained sub-demux handlers
 *     on the domain's own virqs for hwirq 25 and 5 (see samcop_irq_init()),
 *     not as three irq_domains. HAMCOP's hwirq == gpio-offset identity does
 *     not hold here: see samcop_gpio_to_irq()'s sparse lookup table.
 *
 *  2. The GPIO chip is five banks (A-E), not two, and GPA is 20 lines wide
 *     and needs two config registers (CON1 for pins 0-15, CON2 for 16-19)
 *     where every other bank needs one.
 *
 *  3. There is no SRAM API. SAMCOP's 32 KiB SRAM is a DMA bounce pool for the
 *     OHCI cell, not a WinCE bootloader resume vector, so it is handed to
 *     that cell as a second memory resource like any other sub-range.
 *
 * Register access, locking, the IRQ chip and domain shape, the gpio_chip
 * shape, the clk_ops gates and the single devm_mfd_add_devices() call
 * otherwise follow hamcop.c. SAMCOP's registers are 32-bit, not HAMCOP's
 * 16-bit VLIO width, and its ClockControl gate bits differ (SAMCOP_CLK_GPIO
 * is bit 3, not bit 9).
 *
 * Nine of the IC block's 26 sources (APPBUTTON1-4, RECORD, WAKEUP1-3,
 * SD_WAKEUP) are backed by GPA pins wired in EXTIN mode. Each needs its pin
 * muxed and a GPIO_INT1/GPIO_ENINT1 trigger and enable pair programmed, on
 * top of the IC_INTMSK unmask, before it fires. The probe writes the board's
 * static CON1/CON2/GPIO_INT/ENINT values for the eight that are not
 * SD_WAKEUP (see samcop_init), before any consumer requests an interrupt;
 * SD_WAKEUP is armed by samcop_ic_irq_set_type().
 *
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/mfd/ds1wm.h>
#include <linux/mfd/samcop.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

/* Chained-handler loop bound, at the top level (IC) and inside each nested
 * sub-demux. 100 tripped on error-free high-throughput SD traffic (a fast
 * multi-block read followed by CMD12), so it matches the vendor driver's
 * MAX_ASIC_ISR_LOOPS; a source that is really stuck is still caught, just
 * after more headroom.
 */
#define SAMCOP_MAX_IRQ_LOOPS		1000

/*
 * A source pending in at least this many of one round's passes is storming
 * and gets masked when the round hits its bound; one pending in fewer is a
 * bystander that happened to fire inside the window and is left enabled.
 * A bystander is serviced in one pass and its consumer cannot re-arm it
 * while the handler runs with interrupts off, so it scores one or two.
 */
#define SAMCOP_STORM_PERSIST_PASSES	(SAMCOP_MAX_IRQ_LOOPS / 4)

/*
 * Bound on the re-drains samcop_irq_handler() runs when IC_INTPND is found
 * pending after a drain. Small on purpose: each one is a full drain with
 * interrupts off. Past it, samcop_late_isolate() masks what is still
 * pending for SAMCOP_LATE_REARM_MS.
 */
#define SAMCOP_MAX_IRQ_RECHECKS		3

/*
 * How long a source isolated by samcop_late_isolate() stays masked before
 * the re-arm work unmasks it. Short against every consumer's own timeout
 * (libata's shortest command timeout is seconds), long enough that a
 * source which really is stuck costs at most one isolation per period.
 */
#define SAMCOP_LATE_REARM_MS		10

/* The three demux tiers, each with its own drain loop and storm backstop. */
enum {
	SAMCOP_TIER_IC,
	SAMCOP_TIER_GPIO,
	SAMCOP_TIER_EPS,
	SAMCOP_NR_TIERS,
};

enum {
	SAMCOP_GATE_USBHOST,
	SAMCOP_GATE_DMAC,
	SAMCOP_GATE_GPIO,
	SAMCOP_GATE_FCD,
	SAMCOP_GATE_SD,
	SAMCOP_GATE_ADC,
	SAMCOP_GATE_UART2,
	SAMCOP_GATE_UART1,
	SAMCOP_GATE_LED,
	SAMCOP_GATE_MISC,
	SAMCOP_GATE_1WIRE,
	SAMCOP_NR_CLK_GATES,
};

struct samcop;

struct samcop_clk_gate {
	struct clk_hw hw;
	struct samcop *samcop;
	u32 bit;
};

struct samcop_gclk {
	struct clk_hw hw;
	struct samcop *samcop;
};

struct samcop {
	struct device *dev;
	void __iomem *regs;
	int irq;

	/* Guards every register read-modify-write below (GPIO CON/DAT,
	 * ClockControl, IC/GPIO/EPS mask registers) -- the various pending
	 * -ack registers (SRCPND, INTPND, GPIO_INTPND, PCMCIA_IP) are
	 * write-1-to-clear and need no lock. raw_spinlock_t because the IRQ
	 * chip callbacks and the chained handlers both take it.
	 */
	raw_spinlock_t lock;

	/*
	 * Sources the storm backstop has masked, per tier, in that tier's
	 * pending-register bit positions. The drain loops ignore these bits,
	 * so a masked source whose pending bit still latches cannot hold a
	 * loop at its bound on every later entry. Written only under @lock;
	 * each tier's irq_unmask clears its bit before the hardware unmask.
	 */
	u32 storm_masked[SAMCOP_NR_TIERS];

	/*
	 * The subset of storm_masked that samcop_late_isolate() masked, and
	 * which late_rearm unmasks again. Written only under @lock.
	 */
	u32 late_masked[SAMCOP_NR_TIERS];
	struct delayed_work late_rearm;

	struct irq_domain *domain;
	struct gpio_chip gc;

	struct samcop_gclk gclk;
	struct samcop_clk_gate gates[SAMCOP_NR_CLK_GATES];

	/* Register values saved across suspend, see samcop_suspend(). */
	u32 saved_clockcontrol;
	u32 saved_intmsk;

	/*
	 * Set from suspend to resume: samcop_late_isolate() does not queue
	 * late_rearm while it is set. Written only under @lock.
	 */
	bool late_rearm_blocked;
};

/*
 * Chip-level init values written at probe. The GPIO values are opaque raw
 * register contents: GPA_CON1/CON2 mux GPA0-3, 12, 14, 15 and 16 to EXTIN,
 * GPIO_INT1-3 and GPIO_ENINT1/2 arm the matching sources, GPIO_FLTCONFIG1-7
 * set the glitch filters.
 */
static const struct {
	u32 clocksleep;
	u32 pllcontrol;
	u32 gpacon1;
	u32 gpacon2;
	u32 gpadat;
	u32 gpaup;
	u32 gpioint[3];
	u32 gpioflt[7];
	u32 gpioenint[2];
} samcop_init = {
	.clocksleep = SAMCOP_CLKSLEEP_UCLK_ON,
	.pllcontrol = 0x60002,
	.gpacon1 = 0xa2aaaaaa,
	.gpacon2 = 0x5a,
	.gpadat = 0x0,
	.gpaup = 0x0,
	.gpioint = { 0x66666666, 0x60006, 0x0 },
	.gpioflt = { 0x3fff0000, 0x3fff3fff, 0x3fff, 0, 0, 0x3fff3fff, 0 },
	.gpioenint = { 0x3f, 0x7f },
};

static const struct of_device_id samcop_of_match[] = {
	{ .compatible = "samsung,samcop" },
	{ }
};
MODULE_DEVICE_TABLE(of, samcop_of_match);

static inline u32 samcop_read(struct samcop *s, u32 reg)
{
	return readl_relaxed(s->regs + reg);
}

static inline void samcop_write(struct samcop *s, u32 reg, u32 val)
{
	writel_relaxed(val, s->regs + reg);
}

static void samcop_rmw(struct samcop *s, u32 reg, u32 mask, u32 bits)
{
	unsigned long flags;
	u32 val;

	raw_spin_lock_irqsave(&s->lock, flags);
	val = samcop_read(s, reg);
	val = (val & ~mask) | bits;
	samcop_write(s, reg, val);
	raw_spin_unlock_irqrestore(&s->lock, flags);
}

/*
 * Every unmask also re-arms a source the storm backstop masked, clearing its
 * storm_masked bit before the hardware unmask so the drain loop sees it again
 * the moment it can fire: request_irq() after free_irq(), enable_irq() after
 * a masking disable and a oneshot thread's final unmask all undo a storm
 * mask exactly as they undo any other.
 */
static void samcop_storm_unmark(struct samcop *s, unsigned int tier, u32 bits)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&s->lock, flags);
	WRITE_ONCE(s->storm_masked[tier], s->storm_masked[tier] & ~bits);
	raw_spin_unlock_irqrestore(&s->lock, flags);
}

/* ---------------------------------------------------------------------
 * IRQ chips: one per domain tier. IC (0-25) is flat ack/mask/unmask for 25 of
 * its 26 sources, plus one .irq_set_type (samcop_ic_irq_set_type()) for
 * SAMCOP_IRQ_SD_WAKEUP, whose consumer sets its trigger type at runtime. The
 * GPIO fan-out (26-39) has a generic .irq_set_type. The EPS fan-out (40-45)
 * is flat: ack, mask and unmask only.
 * ---------------------------------------------------------------------
 */

static void samcop_ic_irq_ack(struct irq_data *d)
{
	struct samcop *s = irq_data_get_irq_chip_data(d);
	u32 bit = BIT(irqd_to_hwirq(d));

	/* Write-1-to-clear on both registers -- no read-modify-write, no
	 * lock needed.
	 */
	samcop_write(s, SAMCOP_IC_SRCPND, bit);
	samcop_write(s, SAMCOP_IC_INTPND, bit);
}

static void samcop_ic_irq_mask(struct irq_data *d)
{
	struct samcop *s = irq_data_get_irq_chip_data(d);
	u32 bit = BIT(irqd_to_hwirq(d));

	samcop_rmw(s, SAMCOP_IC_INTMSK, bit, bit);
}

static void samcop_ic_irq_unmask(struct irq_data *d)
{
	struct samcop *s = irq_data_get_irq_chip_data(d);
	u32 bit = BIT(irqd_to_hwirq(d));

	samcop_storm_unmark(s, SAMCOP_TIER_IC, bit);
	samcop_rmw(s, SAMCOP_IC_INTMSK, bit, 0);
}

/*
 * SAMCOP_IRQ_SD_WAKEUP (hwirq 13) is the one IC-tier source a consumer arms at
 * runtime rather than through the static values written at probe: the SD host
 * hands this hwirq to the MMC core via mmc_gpio_set_cd_irq(), and
 * mmc_gpiod_request_cd_irq() requests it with IRQF_TRIGGER_RISING |
 * IRQF_TRIGGER_FALLING, which reaches this callback as IRQ_TYPE_EDGE_BOTH
 * through the ordinary request_irq() path.
 *
 * GPA16 is already muxed to EXTIN by the static gpacon2 = 0x5a (field 0 =
 * 0b10) written at probe, so this only programs the trigger-type nibble and
 * the ENINT1 enable bit. The WinCE OAL arms GPA16 at GPIO_INT3 (register
 * index 2) nibble 0 -- register index * 8 + nibble is the GPA pin number for
 * this IC-tier EXTIN allocation -- and sets GPIO_ENINT1 bit 8 in the same
 * function, and the 2.4/2.6 driver's table for this source agrees bit for bit.
 *
 * Every other IC source returns 0 rather than -EINVAL, so a type request on
 * them is a no-op: the other eight GPA-EXTIN sources are already fully armed
 * by the static values, the remaining 17 have no trigger-type concept, and
 * __irq_set_trigger() treats a nonzero return from an existing .irq_set_type
 * as a hard request_irq() failure.
 */
static int samcop_ic_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct samcop *s = irq_data_get_irq_chip_data(d);
	u32 trigger;

	if (irqd_to_hwirq(d) != SAMCOP_IRQ_SD_WAKEUP)
		return 0;

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		trigger = SAMCOP_GPIO_INTLV_RISING(0);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		trigger = SAMCOP_GPIO_INTLV_FALLING(0);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		trigger = SAMCOP_GPIO_INTLV_BOTH(0);
		break;
	default:
		return -EINVAL;
	}

	irq_set_handler_locked(d, handle_edge_irq);

	/* GPIO_INT3 (SAMCOP_GPIO_INT(2)) nibble 0 == GPA16. The filter bit is
	 * left clear, as in samcop_gpio_irq_set_type().
	 */
	samcop_rmw(s, SAMCOP_GPIO_INT(2),
		   SAMCOP_GPIO_INTLV_MASK(0) | SAMCOP_GPIO_INTF_MASK(0),
		   trigger);
	samcop_rmw(s, SAMCOP_GPIO_ENINT1, BIT(8), BIT(8));

	return 0;
}

/*
 * SAMCOP has no wake-configuration register: any enabled, unmasked source in
 * any tier pulls the one shared ASIC_INT_N, so arming a hwirq for wakeup
 * reduces to arming the chained parent, PXA GPIO10. All three chips forward
 * to the same s->irq; irq_set_irq_wake() refcounts through wake_depth, so
 * children of different tiers arming and disarming independently balance
 * out. Without this callback enable_irq_wake() on any SAMCOP line fails with
 * -ENXIO, as it did for gpio-keys on HAMCOP. It arms nothing until the
 * parent pin itself is configured as a wake source.
 *
 * IRQCHIP_MASK_ON_SUSPEND lets genirq mask every non-wake source at suspend
 * while an IRQD_WAKEUP_STATE one keeps asserting, which a blanket IC_INTMSK
 * write cannot do. The two chained sub-demux descriptors are skipped by
 * suspend_device_irqs(), so genirq never masks IC sources 5 and 25; only
 * the blanket IC_INTMSK write in samcop_suspend() does, and that is
 * skipped with selective_suspend_mask set.
 */
static int samcop_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct samcop *s = irq_data_get_irq_chip_data(d);

	return irq_set_irq_wake(s->irq, on);
}

static struct irq_chip samcop_ic_irq_chip = {
	.name		= "samcop-ic",
	.irq_ack	= samcop_ic_irq_ack,
	.irq_mask	= samcop_ic_irq_mask,
	.irq_unmask	= samcop_ic_irq_unmask,
	.irq_set_wake	= samcop_irq_set_wake,
	.flags		= IRQCHIP_MASK_ON_SUSPEND,
	.irq_set_type	= samcop_ic_irq_set_type,
};

static void samcop_gpio_irq_ack(struct irq_data *d)
{
	struct samcop *s = irq_data_get_irq_chip_data(d);
	unsigned int idx = irqd_to_hwirq(d) - SAMCOP_GPIO_IRQ_BASE;

	/*
	 * GPIO_INTPND only -- the IC-level routing bit (source 25) is
	 * acked unconditionally by samcop_irq_demux_gpio() itself before
	 * this leaf ack ever runs, see that function's comment.
	 */
	samcop_write(s, SAMCOP_GPIO_INTPND, BIT(idx));
}

static void samcop_gpio_irq_mask(struct irq_data *d)
{
	struct samcop *s = irq_data_get_irq_chip_data(d);
	u32 bit = BIT(irqd_to_hwirq(d) - SAMCOP_GPIO_IRQ_BASE);

	samcop_rmw(s, SAMCOP_GPIO_ENINT2, bit, 0);
}

static void samcop_gpio_irq_unmask(struct irq_data *d)
{
	struct samcop *s = irq_data_get_irq_chip_data(d);
	u32 bit = BIT(irqd_to_hwirq(d) - SAMCOP_GPIO_IRQ_BASE);

	samcop_storm_unmark(s, SAMCOP_TIER_GPIO, bit);
	samcop_rmw(s, SAMCOP_GPIO_ENINT2, bit, bit);
}

static int samcop_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct samcop *s = irq_data_get_irq_chip_data(d);
	unsigned int idx = irqd_to_hwirq(d) - SAMCOP_GPIO_IRQ_BASE;
	unsigned int reg_idx = idx / 8;
	unsigned int nibble = idx % 8;
	u32 intlv;

	switch (type) {
	case IRQ_TYPE_LEVEL_LOW:
		intlv = SAMCOP_GPIO_INTLV_LOW(nibble);
		irq_set_handler_locked(d, handle_level_irq);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		intlv = SAMCOP_GPIO_INTLV_HIGH(nibble);
		irq_set_handler_locked(d, handle_level_irq);
		break;
	case IRQ_TYPE_EDGE_RISING:
		intlv = SAMCOP_GPIO_INTLV_RISING(nibble);
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		intlv = SAMCOP_GPIO_INTLV_FALLING(nibble);
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		intlv = SAMCOP_GPIO_INTLV_BOTH(nibble);
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	default:
		return -EINVAL;
	}

	/*
	 * Filter bit left clear (glitch filter off): gpioflt[] programs the
	 * FLTCONFIG side independently at probe, and no per-hwirq mapping
	 * between the two is known.
	 */
	samcop_rmw(s, SAMCOP_GPIO_INT(reg_idx),
		   SAMCOP_GPIO_INTLV_MASK(nibble) | SAMCOP_GPIO_INTF_MASK(nibble),
		   intlv);

	return 0;
}

static struct irq_chip samcop_gpio_irq_chip = {
	.name		= "samcop-gpio",
	.irq_ack	= samcop_gpio_irq_ack,
	.irq_mask	= samcop_gpio_irq_mask,
	.irq_unmask	= samcop_gpio_irq_unmask,
	.irq_set_wake	= samcop_irq_set_wake,
	.flags		= IRQCHIP_MASK_ON_SUSPEND,
	.irq_set_type	= samcop_gpio_irq_set_type,
};

/* EPS mask table, index = hwirq - SAMCOP_EPS_IRQ_BASE. ODET combines both
 * ODET0_N/ODET1_N under one hwirq.
 */
static const u32 samcop_eps_bits[SAMCOP_EPS_NR_IRQS] = {
	SAMCOP_PCMCIA_EPS_CD0_N,
	SAMCOP_PCMCIA_EPS_CD1_N,
	SAMCOP_PCMCIA_EPS_IRQ0_N,
	SAMCOP_PCMCIA_EPS_IRQ1_N,
	SAMCOP_PCMCIA_EPS_ODET0_N | SAMCOP_PCMCIA_EPS_ODET1_N,
	SAMCOP_PCMCIA_EPS_BATT_FLT,
};

static void samcop_eps_irq_ack(struct irq_data *d)
{
	struct samcop *s = irq_data_get_irq_chip_data(d);
	unsigned int idx = irqd_to_hwirq(d) - SAMCOP_EPS_IRQ_BASE;

	/* PCMCIA_IP only -- the IC-level routing bit (source 5) is acked
	 * unconditionally by samcop_irq_demux_eps() itself, same reasoning
	 * as the GPIO tier above.
	 */
	samcop_write(s, SAMCOP_PCMCIA_IP, samcop_eps_bits[idx]);
}

static void samcop_eps_irq_mask(struct irq_data *d)
{
	struct samcop *s = irq_data_get_irq_chip_data(d);
	unsigned int idx = irqd_to_hwirq(d) - SAMCOP_EPS_IRQ_BASE;

	samcop_rmw(s, SAMCOP_PCMCIA_IC, samcop_eps_bits[idx], 0);
}

static void samcop_eps_irq_unmask(struct irq_data *d)
{
	struct samcop *s = irq_data_get_irq_chip_data(d);
	unsigned int idx = irqd_to_hwirq(d) - SAMCOP_EPS_IRQ_BASE;

	samcop_storm_unmark(s, SAMCOP_TIER_EPS, samcop_eps_bits[idx]);
	samcop_rmw(s, SAMCOP_PCMCIA_IC, samcop_eps_bits[idx],
		   samcop_eps_bits[idx]);
}

static struct irq_chip samcop_eps_irq_chip = {
	.name		= "samcop-eps",
	.irq_ack	= samcop_eps_irq_ack,
	.irq_mask	= samcop_eps_irq_mask,
	.irq_unmask	= samcop_eps_irq_unmask,
	.irq_set_wake	= samcop_irq_set_wake,
	.flags		= IRQCHIP_MASK_ON_SUSPEND,
};

static int samcop_irq_domain_map(struct irq_domain *d, unsigned int virq,
				  irq_hw_number_t hwirq)
{
	struct irq_chip *chip;

	if (hwirq < SAMCOP_IC_NR_IRQS)
		chip = &samcop_ic_irq_chip;
	else if (hwirq < SAMCOP_EPS_IRQ_BASE)
		chip = &samcop_gpio_irq_chip;
	else
		chip = &samcop_eps_irq_chip;

	irq_set_chip_and_handler(virq, chip, handle_edge_irq);
	irq_set_chip_data(virq, d->host_data);
	irq_set_probe(virq);

	return 0;
}

static void samcop_irq_domain_unmap(struct irq_domain *d, unsigned int virq)
{
	irq_set_chip_and_handler(virq, NULL, NULL);
	irq_set_chip_data(virq, NULL);
}

static const struct irq_domain_ops samcop_irq_domain_ops = {
	.map	= samcop_irq_domain_map,
	.unmap	= samcop_irq_domain_unmap,
	.xlate	= irq_domain_xlate_onetwocell,
};

/*
 * Per-tier description for the shared storm backstop below: how many
 * sources the tier has, the mask register and its polarity, and the
 * write-1-to-clear pending registers to ack, in order.
 */
static const struct samcop_storm_tier {
	const char *log_prefix;
	unsigned int nr;
	u32 mask_reg;
	bool set_masks;
	unsigned int nr_ack;
	u32 ack_reg[2];
} samcop_storm_tiers[SAMCOP_NR_TIERS] = {
	[SAMCOP_TIER_IC] = {
		.log_prefix	= "IC ",
		.nr		= SAMCOP_IC_NR_IRQS,
		.mask_reg	= SAMCOP_IC_INTMSK,
		.set_masks	= true,
		.nr_ack		= 2,
		.ack_reg	= { SAMCOP_IC_SRCPND, SAMCOP_IC_INTPND },
	},
	[SAMCOP_TIER_GPIO] = {
		.log_prefix	= "GPIO sub-",
		.nr		= SAMCOP_GPIO_NR_IRQS,
		.mask_reg	= SAMCOP_GPIO_ENINT2,
		.nr_ack		= 1,
		.ack_reg	= { SAMCOP_GPIO_INTPND },
	},
	[SAMCOP_TIER_EPS] = {
		.log_prefix	= "EPS sub-",
		.nr		= SAMCOP_EPS_NR_IRQS,
		.mask_reg	= SAMCOP_PCMCIA_IC,
		.nr_ack		= 1,
		.ack_reg	= { SAMCOP_PCMCIA_IP },
	},
};

/* Pending-register bits of source @i of @tier. */
static u32 samcop_storm_bits(unsigned int tier, unsigned int i)
{
	return tier == SAMCOP_TIER_EPS ? samcop_eps_bits[i] : BIT(i);
}

/* Count one drain pass: every source pending in it scores one. */
static void samcop_storm_count(unsigned int tier, u16 *cnt, u32 pending)
{
	unsigned int i;

	for (i = 0; i < samcop_storm_tiers[tier].nr; i++)
		if (pending & samcop_storm_bits(tier, i))
			cnt[i]++;
}

/*
 * Storm backstop: one drain round of @tier hit SAMCOP_MAX_IRQ_LOOPS with
 * something still pending. @cnt holds, per source, the number of the
 * round's passes it was pending in. Returns true when it masked a
 * persistent set or spared a routing bit, in which case the caller runs one
 * more round with @final set.
 *
 * Returning with a source still asserted is not survivable: ASIC_INT_N
 * reaches the PXA through an edge-only GPIO, so a line that never
 * deasserts never produces another edge and every source behind this chip
 * falls silent. Each masked source is therefore masked and then acked
 * before the round ends: before the top-level handler rechecks IC_INTPND,
 * and before a nested demux returns, after its IC routing-bit ack.
 *
 * Masking a whole tier retires every working source behind it with the
 * stuck one; with root on SD that stops all rootfs I/O while the core keeps
 * running. Masking the union of everything seen in the round has the same
 * flaw on a smaller scale: a source that fired once inside the window is
 * retired too, and its consumer never hears from it again. So only the
 * sources pending in at least SAMCOP_STORM_PERSIST_PASSES of the round's
 * passes are masked. Counts, not the intersection of the per-pass sets:
 * two sources that alternate pass by pass have an empty intersection
 * although both storm, and counted they score half the bound each. The
 * union is the fallback, used when no source reaches the threshold or when
 * the round after a persistent-set mask hits the bound too (@final):
 * something is still stuck, and returning with ASIC_INT_N asserted would
 * retire the whole chip.
 *
 * IC sources 5 and 25 are the routing bits of the two nested tiers. A
 * sub-source that re-asserts between two IC passes is invisible to its
 * nested demux's own count but makes the routing bit persistent here, and
 * masking that bit silences the whole nested tier. In the first round they
 * are therefore only counted, never masked, and the round asks for a second
 * one so that the nested tiers' own backstops get to isolate the
 * sub-source. Only when the second round still finds one stuck is it
 * masked with everything else seen, as the last resort that keeps
 * ASIC_INT_N from retiring the chip; no consumer ever unmasks a routing
 * bit, so it stays masked until the next probe.
 *
 * The mask register is shared with the irq_chip callbacks, so it is
 * updated by a read-modify-write under s->lock, in the same hold that
 * records the bits in storm_masked: hardware mask first, then the bit, the
 * reverse of samcop_storm_unmark() plus the unmask. This is fault isolation
 * only: it does not fix whatever leaves a source stuck.
 */
static bool samcop_storm_overrun(struct samcop *s, unsigned int tier,
				 const u16 *cnt, unsigned int dispatched,
				 bool final)
{
	const struct samcop_storm_tier *t = &samcop_storm_tiers[tier];
	const u32 keep = tier == SAMCOP_TIER_IC && !final ?
			 BIT(SAMCOP_IRQ_GPIO) | BIT(SAMCOP_IRQ_PCMCIA) : 0;
	unsigned int i, p_min = UINT_MAX, rest_max = 0;
	u32 seen = 0, mask = 0, bits, val;
	unsigned long flags;
	bool fallback;

	for (i = 0; i < t->nr; i++) {
		bits = samcop_storm_bits(tier, i);
		if (!cnt[i])
			continue;

		seen |= bits;
		if (!final && cnt[i] >= SAMCOP_STORM_PERSIST_PASSES &&
		    !(bits & keep))
			mask |= bits;
	}

	fallback = !mask;
	if (fallback)
		mask = seen & ~keep;

	for (i = 0; i < t->nr; i++) {
		if (!cnt[i])
			continue;

		if (samcop_storm_bits(tier, i) & mask)
			p_min = min_t(unsigned int, p_min, cnt[i]);
		else
			rest_max = max_t(unsigned int, rest_max, cnt[i]);
	}

	if (mask) {
		raw_spin_lock_irqsave(&s->lock, flags);
		val = samcop_read(s, t->mask_reg);
		samcop_write(s, t->mask_reg,
			     t->set_masks ? val | mask : val & ~mask);
		WRITE_ONCE(s->storm_masked[tier], s->storm_masked[tier] | mask);
		raw_spin_unlock_irqrestore(&s->lock, flags);

		for (i = 0; i < t->nr_ack; i++)
			samcop_write(s, t->ack_reg[i], mask);
	}

	dev_err_ratelimited(s->dev,
		"%sinterrupt storm after %u passes (dispatched %u): masking %#x, sparing %#x; counts masked>=%u spared<=%u%s\n",
		t->log_prefix, SAMCOP_MAX_IRQ_LOOPS, dispatched, mask,
		seen & ~mask, mask ? p_min : 0, rest_max,
		fallback ? " [fallback]" : "");

	return !fallback || (seen & ~mask);
}

/*
 * Common tail of one drain round: if it hit the bound, run the backstop.
 * Returns true only when one more round should run.
 */
static bool samcop_storm_round_end(struct samcop *s, unsigned int tier,
				   const u16 *cnt, unsigned int loops,
				   unsigned int dispatched, bool final)
{
	if (likely(loops < SAMCOP_MAX_IRQ_LOOPS))
		return false;

	return samcop_storm_overrun(s, tier, cnt, dispatched, final);
}

static bool samcop_gpio_drain_round(struct samcop *s, bool final)
{
	u16 cnt[SAMCOP_GPIO_NR_IRQS] = { };
	unsigned int loops, dispatched = 0;
	u32 pending;

	for (loops = 0; loops < SAMCOP_MAX_IRQ_LOOPS; loops++) {
		pending = samcop_read(s, SAMCOP_GPIO_INTPND) &
			  GENMASK(SAMCOP_GPIO_NR_IRQS - 1, 0) &
			  ~READ_ONCE(s->storm_masked[SAMCOP_TIER_GPIO]);
		if (!pending)
			break;

		samcop_storm_count(SAMCOP_TIER_GPIO, cnt, pending);

		while (pending) {
			unsigned int idx = __ffs(pending);

			generic_handle_domain_irq(s->domain, SAMCOP_GPIO_IRQ_BASE + idx);
			pending &= ~BIT(idx);
			dispatched++;
		}
	}

	return samcop_storm_round_end(s, SAMCOP_TIER_GPIO, cnt, loops,
				      dispatched, final);
}

/*
 * Nested sub-demux for IC source 25 (GPIO fan-out, 14 sparse GPA/GPB
 * pins). Installed as a chained handler directly on the domain's own
 * virq for hwirq SAMCOP_IRQ_GPIO, so the top-level IC demux below
 * dispatches into it via the ordinary generic_handle_domain_irq() path.
 *
 * The IC-level routing bit is acked *before* reading GPIO_INTPND, not
 * after draining it: if GPIO_INTPND is already empty on entry (a
 * spurious or already-handled dispatch), nothing below would otherwise
 * ever clear IC_SRCPND/INTPND bit 25, and the outer chained handler
 * would see it pending forever. Acking first guarantees forward
 * progress at the top level regardless of what -- if anything -- is
 * found in the sub-block; each leaf's own ack (samcop_gpio_irq_ack)
 * only needs to clear its own GPIO_INTPND bit.
 */
static void samcop_irq_demux_gpio(struct samcop *s)
{
	samcop_write(s, SAMCOP_IC_SRCPND, BIT(SAMCOP_IRQ_GPIO));
	samcop_write(s, SAMCOP_IC_INTPND, BIT(SAMCOP_IRQ_GPIO));

	/* Two rounds at most, see samcop_irq_handler(). */
	if (samcop_gpio_drain_round(s, false))
		samcop_gpio_drain_round(s, true);
}

static bool samcop_eps_drain_round(struct samcop *s, bool final)
{
	u16 cnt[SAMCOP_EPS_NR_IRQS] = { };
	unsigned int loops, dispatched = 0;
	u32 pending, all_mask = 0;
	unsigned int i;

	for (i = 0; i < SAMCOP_EPS_NR_IRQS; i++)
		all_mask |= samcop_eps_bits[i];

	for (loops = 0; loops < SAMCOP_MAX_IRQ_LOOPS; loops++) {
		pending = samcop_read(s, SAMCOP_PCMCIA_IP) & all_mask &
			  ~READ_ONCE(s->storm_masked[SAMCOP_TIER_EPS]);
		if (!pending)
			break;

		samcop_storm_count(SAMCOP_TIER_EPS, cnt, pending);

		for (i = 0; i < SAMCOP_EPS_NR_IRQS; i++) {
			if (pending & samcop_eps_bits[i]) {
				generic_handle_domain_irq(s->domain, SAMCOP_EPS_IRQ_BASE + i);
				dispatched++;
			}
		}
	}

	return samcop_storm_round_end(s, SAMCOP_TIER_EPS, cnt, loops,
				      dispatched, final);
}

/* Nested sub-demux for IC source 5 (PCMCIA/EPS fan-out, 6 sources).
 * Same ack-first reasoning as samcop_irq_demux_gpio() above.
 */
static void samcop_irq_demux_eps(struct samcop *s)
{
	samcop_write(s, SAMCOP_IC_SRCPND, BIT(SAMCOP_IRQ_PCMCIA));
	samcop_write(s, SAMCOP_IC_INTPND, BIT(SAMCOP_IRQ_PCMCIA));

	/* Two rounds at most, see samcop_irq_handler(). */
	if (samcop_eps_drain_round(s, false))
		samcop_eps_drain_round(s, true);
}

static void samcop_irq_gpio_chained(struct irq_desc *desc)
{
	samcop_irq_demux_gpio(irq_desc_get_handler_data(desc));
}

static void samcop_irq_eps_chained(struct irq_desc *desc)
{
	samcop_irq_demux_eps(irq_desc_get_handler_data(desc));
}

/*
 * IC sources pending and not storm-masked. The drain loop and the late-assert
 * recheck share this view, so a source the backstop has just masked can
 * drive neither of them.
 */
static u32 samcop_ic_pending(struct samcop *s)
{
	return samcop_read(s, SAMCOP_IC_INTPND) &
	       GENMASK(SAMCOP_IC_NR_IRQS - 1, 0) &
	       ~READ_ONCE(s->storm_masked[SAMCOP_TIER_IC]);
}

/* One bounded drain round of the IC tier. IC_INTPND is a single 32-bit
 * register giving all 26 sources directly -- unlike HAMCOP's paired
 * 16-bit halves, no combining needed. Highest hwirq first, matching the
 * vendor driver's priority order.
 */
static bool samcop_ic_drain_round(struct samcop *s, bool final)
{
	u16 cnt[SAMCOP_IC_NR_IRQS] = { };
	unsigned int loops, dispatched = 0;
	u32 pending;

	for (loops = 0; loops < SAMCOP_MAX_IRQ_LOOPS; loops++) {
		pending = samcop_ic_pending(s);
		if (!pending)
			break;

		samcop_storm_count(SAMCOP_TIER_IC, cnt, pending);

		while (pending) {
			unsigned int hwirq = fls(pending) - 1;

			generic_handle_domain_irq(s->domain, hwirq);
			pending &= ~BIT(hwirq);
			dispatched++;
		}
	}

	return samcop_storm_round_end(s, SAMCOP_TIER_IC, cnt, loops,
				      dispatched, final);
}

/* Tier of @hwirq, and its source index within that tier in @idx. */
static unsigned int samcop_hwirq_tier(unsigned int hwirq, unsigned int *idx)
{
	if (hwirq < SAMCOP_GPIO_IRQ_BASE) {
		*idx = hwirq;
		return SAMCOP_TIER_IC;
	}
	if (hwirq < SAMCOP_EPS_IRQ_BASE) {
		*idx = hwirq - SAMCOP_GPIO_IRQ_BASE;
		return SAMCOP_TIER_GPIO;
	}
	*idx = hwirq - SAMCOP_EPS_IRQ_BASE;
	return SAMCOP_TIER_EPS;
}

/* Widen EPS pending bits to whole sources: ODET is two bits, one hwirq. */
static u32 samcop_eps_sources(u32 bits)
{
	u32 srcs = 0;
	unsigned int i;

	for (i = 0; i < SAMCOP_EPS_NR_IRQS; i++)
		if (bits & samcop_eps_bits[i])
			srcs |= samcop_eps_bits[i];

	return srcs;
}

/* Mask @bits per tier in hardware and record them as late-isolated. */
static void samcop_late_mask_locked(struct samcop *s, const u32 *bits)
{
	unsigned int t;
	u32 val;

	lockdep_assert_held(&s->lock);

	for (t = 0; t < SAMCOP_NR_TIERS; t++) {
		const struct samcop_storm_tier *st = &samcop_storm_tiers[t];

		if (!bits[t])
			continue;

		val = samcop_read(s, st->mask_reg);
		samcop_write(s, st->mask_reg,
			     st->set_masks ? val | bits[t] : val & ~bits[t]);
		WRITE_ONCE(s->storm_masked[t], s->storm_masked[t] | bits[t]);
		s->late_masked[t] |= bits[t];
	}
}

/*
 * The recheck in samcop_irq_handler() found IC sources pending after
 * SAMCOP_MAX_IRQ_RECHECKS re-drains in a row. Returning now would leave
 * ASIC_INT_N low with nothing to produce the next edge on the PXA side, and
 * every source behind the chip would fall silent. Mask the sources behind
 * @pending instead, so the line can deassert, and have late_rearm unmask
 * them again shortly after.
 *
 * Behind a routing bit (IC 5 or 25) the source is the enabled sub-source
 * latched in that tier, not the routing bit: masking it there keeps every
 * other source of the tier working. Its sub-tier pending bit is left set,
 * so the event is still there for its handler when the re-arm unmasks it;
 * only the IC copy of the routing bit is acked. A disabled GPIO sub-source
 * latched in GPIO_INTPND does not raise IC 25 again once that is acked (the
 * chip has been read with GPIO INTPND 0x3c00, ENINT2 0x7f and IC 25 clear),
 * and this assumes PCMCIA_IC gates the EPS tier the same way. If the
 * routing bit comes straight back anyway, it is masked itself.
 *
 * A direct IC source is masked in IC_INTMSK with only its INTPND bit acked.
 * The IC arbitrates like the S3C24xx one it resembles (it has been read
 * with SRCPND 0x10a0, INTPND 0x20 and sources 7 and 12 unmasked: a pending
 * unmasked source is not promoted into INTPND while another holds it), so
 * SRCPND keeps the event for the unmask. A masked bit still in INTPND after
 * that is acked in both registers, losing the event rather than the line.
 *
 * Every call masks at least one source not masked before, since @pending
 * and the sub-tier reads all exclude storm_masked, so the caller's loop
 * ends after at most one call per source.
 */
static void samcop_late_isolate(struct samcop *s, u32 pending)
{
	u32 bits[SAMCOP_NR_TIERS] = { };
	u32 back[SAMCOP_NR_TIERS] = { };
	u32 routed = 0, stuck;
	unsigned long flags;

	raw_spin_lock_irqsave(&s->lock, flags);

	if (pending & BIT(SAMCOP_IRQ_PCMCIA)) {
		bits[SAMCOP_TIER_EPS] =
			samcop_eps_sources(samcop_read(s, SAMCOP_PCMCIA_IP) &
					   samcop_read(s, SAMCOP_PCMCIA_IC)) &
			~s->storm_masked[SAMCOP_TIER_EPS];
		if (bits[SAMCOP_TIER_EPS])
			routed |= BIT(SAMCOP_IRQ_PCMCIA);
	}

	if (pending & BIT(SAMCOP_IRQ_GPIO)) {
		bits[SAMCOP_TIER_GPIO] = samcop_read(s, SAMCOP_GPIO_INTPND) &
					 samcop_read(s, SAMCOP_GPIO_ENINT2) &
					 GENMASK(SAMCOP_GPIO_NR_IRQS - 1, 0) &
					 ~s->storm_masked[SAMCOP_TIER_GPIO];
		if (bits[SAMCOP_TIER_GPIO])
			routed |= BIT(SAMCOP_IRQ_GPIO);
	}

	bits[SAMCOP_TIER_IC] = pending & ~routed;
	samcop_late_mask_locked(s, bits);

	raw_spin_unlock_irqrestore(&s->lock, flags);

	samcop_write(s, SAMCOP_IC_SRCPND, routed);
	samcop_write(s, SAMCOP_IC_INTPND, pending);

	back[SAMCOP_TIER_IC] = samcop_read(s, SAMCOP_IC_INTPND) & routed;
	if (back[SAMCOP_TIER_IC]) {
		raw_spin_lock_irqsave(&s->lock, flags);
		samcop_late_mask_locked(s, back);
		raw_spin_unlock_irqrestore(&s->lock, flags);
		bits[SAMCOP_TIER_IC] |= back[SAMCOP_TIER_IC];
	}

	stuck = samcop_read(s, SAMCOP_IC_INTPND) &
		READ_ONCE(s->storm_masked[SAMCOP_TIER_IC]);
	if (stuck) {
		samcop_write(s, SAMCOP_IC_SRCPND, stuck);
		samcop_write(s, SAMCOP_IC_INTPND, stuck);
	}

	/* Queued under the lock, so samcop_suspend() cannot miss it. */
	raw_spin_lock_irqsave(&s->lock, flags);
	if (!s->late_rearm_blocked)
		schedule_delayed_work(&s->late_rearm,
				      msecs_to_jiffies(SAMCOP_LATE_REARM_MS));
	raw_spin_unlock_irqrestore(&s->lock, flags);

	dev_err_ratelimited(s->dev,
		"late asserts persisted past %u rechecks (IC INTPND %#010x): isolating IC %#x GPIO sub- %#x EPS sub- %#x, IC acked %#x, re-arming in %u ms\n",
		SAMCOP_MAX_IRQ_RECHECKS, pending, bits[SAMCOP_TIER_IC],
		bits[SAMCOP_TIER_GPIO], bits[SAMCOP_TIER_EPS], stuck,
		SAMCOP_LATE_REARM_MS);
}

/*
 * Unmasking is only known to raise the IC when the IC itself promotes a kept
 * SRCPND bit; whether an unmasked EPS or GPIO leaf with its pending bit still
 * latched raises its routing bit again is not established. A latched edge
 * source would never latch a new edge, so the source would stay silent for
 * good. If the leaf is still pending and nothing on the IC routes it, run its
 * handler here.
 */
static bool samcop_late_routed(struct samcop *s, unsigned int tier,
			       unsigned int hwirq)
{
	u32 bit;

	switch (tier) {
	case SAMCOP_TIER_GPIO:
		bit = BIT(SAMCOP_IRQ_GPIO);
		break;
	case SAMCOP_TIER_EPS:
		bit = BIT(SAMCOP_IRQ_PCMCIA);
		break;
	default:
		bit = BIT(hwirq);
		return samcop_read(s, SAMCOP_IC_INTPND) & bit;
	}

	return (samcop_read(s, SAMCOP_IC_SRCPND) |
		samcop_read(s, SAMCOP_IC_INTPND)) & bit;
}

/*
 * Unmask what samcop_late_isolate() masked. A source genirq has masked in
 * the meantime keeps its hardware mask, and its own unmask clears
 * storm_masked later. A source only lazily disabled is unmasked here:
 * genirq expects it unmasked in hardware, and its enable_irq() would not
 * call the chip's unmask.
 */
static void samcop_late_rearm_work(struct work_struct *work)
{
	struct samcop *s = container_of(to_delayed_work(work), struct samcop,
					late_rearm);
	unsigned int hwirq, virq, tier, idx;
	u32 late[SAMCOP_NR_TIERS];
	struct irq_desc *desc;
	unsigned long flags;
	struct irq_data *d;
	bool masked;

	raw_spin_lock_irqsave(&s->lock, flags);
	for (tier = 0; tier < SAMCOP_NR_TIERS; tier++) {
		late[tier] = s->late_masked[tier] & s->storm_masked[tier];
		s->late_masked[tier] = 0;
	}
	raw_spin_unlock_irqrestore(&s->lock, flags);

	for (hwirq = 0; hwirq < SAMCOP_NR_IRQS; hwirq++) {
		tier = samcop_hwirq_tier(hwirq, &idx);
		if (!(late[tier] & samcop_storm_bits(tier, idx)))
			continue;

		virq = irq_find_mapping(s->domain, hwirq);
		desc = virq ? irq_to_desc(virq) : NULL;
		if (!desc)
			continue;

		/*
		 * Under the descriptor lock, which genirq holds around its own
		 * mask and unmask, so a disable_irq() cannot slip in between
		 * the test and the unmask. The tier's own unmask clears
		 * storm_masked first.
		 */
		scoped_guard(raw_spinlock_irqsave, &desc->lock) {
			d = irq_desc_get_irq_data(desc);
			masked = irqd_irq_masked(d);
			if (!masked)
				irq_data_get_irq_chip(d)->irq_unmask(d);
		}
		if (masked)
			continue;

		udelay(2);
		if ((samcop_read(s, samcop_storm_tiers[tier].ack_reg[0]) &
		     samcop_storm_bits(tier, idx)) &&
		    !samcop_late_routed(s, tier, hwirq))
			generic_handle_irq_safe(virq);
	}
}

/* Top-level demux on the real PXA GPIO10 chained IRQ. */
static void samcop_irq_handler(struct irq_desc *desc)
{
	struct samcop *s = irq_desc_get_handler_data(desc);
	struct irq_data *parent = irq_desc_get_irq_data(desc);
	struct irq_chip *chip = irq_data_get_irq_chip(parent);
	unsigned int recheck;
	u32 pending;

	/*
	 * Ack the parent but do not mask it, which chained_irq_enter() would.
	 * Masking a PXA GPIO clears its GRER/GFER bits, so ASIC_INT_N falling
	 * while the drain runs is never latched in GEDR; a source asserting
	 * after the drain's last IC_INTPND read then holds the line low with
	 * no edge to show for it, and the chip falls silent. Left armed, the
	 * edge is latched, and pxa_gpio_demux_handler(), which loops until
	 * GEDR reads empty, runs this handler again straight after it
	 * returns. Local interrupts stay off throughout, so that cannot
	 * re-enter the drain.
	 */
	if (chip->irq_ack)
		chip->irq_ack(parent);

	for (recheck = 0; ; recheck++) {
		/*
		 * At most two rounds: the second runs only after the first
		 * masked a persistent set, to drain whatever the masked storm
		 * was hiding, and normally finds nothing pending. See
		 * samcop_storm_overrun().
		 */
		if (samcop_ic_drain_round(s, false))
			samcop_ic_drain_round(s, true);

		/*
		 * A source asserting after the drain's last empty read is caught
		 * by the parent's edge detector. Read IC_INTPND once more anyway
		 * and drain again if anything is pending: it costs one read, and
		 * it does not depend on ASIC_INT_N having risen between that
		 * read and the source asserting.
		 */
		pending = samcop_ic_pending(s);
		if (!pending)
			return;

		/*
		 * Still pending after the last re-drain. Isolate what holds the
		 * line and drain once more for whatever else the IC promotes
		 * into INTPND once that is acked.
		 */
		if (recheck >= SAMCOP_MAX_IRQ_RECHECKS) {
			samcop_late_isolate(s, pending);
			continue;
		}

		dev_dbg_ratelimited(s->dev,
				    "late-assert recovered: IC INTPND %#010x pending after the drain, re-draining (recheck %u/%u)\n",
				    pending, recheck + 1,
				    SAMCOP_MAX_IRQ_RECHECKS);
	}
}

/*
 * The three chained handlers installed by samcop_irq_init() are the only
 * pieces of this driver that are not devm-managed by themselves, and two of
 * the three sit on virqs belonging to s->domain, which *is* devm-managed
 * (devm_irq_domain_instantiate()). Registered as a devm action at the end of
 * samcop_irq_init() rather than unhooked from a .remove() callback so that a
 * later probe failure -- samcop_clocks_register(), samcop_gpio_register() or
 * devm_mfd_add_devices() all return without touching them -- cannot tear the
 * domain, and then s->regs, out from under live handlers.
 *
 * Parent first: unhooking s->irq before the two nested sub-demux virqs means
 * the top-level demux can no longer dispatch into a sub-demux that is already
 * gone.
 *
 * irq_find_mapping() returning 0 (domain already partly gone) is tolerated:
 * irq_set_chained_handler_and_data() resolves the descriptor first and
 * returns without doing anything when there is none.
 */
static void samcop_irq_teardown(void *data)
{
	struct samcop *s = data;

	irq_set_chained_handler_and_data(s->irq, NULL, NULL);
	irq_set_chained_handler_and_data(irq_find_mapping(s->domain, SAMCOP_IRQ_GPIO),
					  NULL, NULL);
	irq_set_chained_handler_and_data(irq_find_mapping(s->domain, SAMCOP_IRQ_PCMCIA),
					  NULL, NULL);
}

static int samcop_irq_init(struct samcop *s)
{
	const struct irq_domain_info info = {
		.fwnode		= dev_fwnode(s->dev),
		.size		= SAMCOP_NR_IRQS,
		.hwirq_max	= SAMCOP_NR_IRQS,
		.ops		= &samcop_irq_domain_ops,
		.host_data	= s,
	};
	unsigned int gpio_virq, eps_virq;
	int ret;

	/* Mask everything and ack whatever the bootloader left pending,
	 * across all three tiers, before any chained handler can fire.
	 */
	samcop_write(s, SAMCOP_IC_INTMSK, GENMASK(SAMCOP_IC_NR_IRQS - 1, 0));
	samcop_write(s, SAMCOP_IC_SRCPND, GENMASK(SAMCOP_IC_NR_IRQS - 1, 0));
	samcop_write(s, SAMCOP_IC_INTPND, GENMASK(SAMCOP_IC_NR_IRQS - 1, 0));
	samcop_write(s, SAMCOP_GPIO_ENINT1, 0);
	samcop_write(s, SAMCOP_GPIO_ENINT2, 0);
	samcop_write(s, SAMCOP_GPIO_INTPND, GENMASK(SAMCOP_GPIO_NR_IRQS - 1, 0));
	samcop_write(s, SAMCOP_PCMCIA_IC, 0);
	samcop_write(s, SAMCOP_PCMCIA_IP, GENMASK(6, 0));

	s->domain = devm_irq_domain_instantiate(s->dev, &info);
	if (IS_ERR(s->domain))
		return PTR_ERR(s->domain);

	/*
	 * Before any handler can isolate a source and queue it, and after
	 * the domain the work looks up mappings in: devm cancels it after
	 * samcop_irq_teardown() has unhooked the handlers and before the
	 * domain is freed.
	 */
	ret = devm_delayed_work_autocancel(s->dev, &s->late_rearm,
					   samcop_late_rearm_work);
	if (ret)
		return ret;

	/*
	 * irq_create_mapping(), not irq_find_mapping(), here and in
	 * samcop_gpio_to_irq()/samcop_hwirq_to_irq() below: this is exactly
	 * the bug class that hit hamcop.c twice on the h2200 stream
	 * (7911766249e2, then again 5732b98c4572 -- the second one silently
	 * prevented gpio-keys from ever registering). find_mapping() on a
	 * hwirq nothing has mapped yet returns 0, which callers read as
	 * "no IRQ" -- defeating the whole point of a consumer resolving one
	 * of these hwirqs for the first time. create_mapping() is
	 * idempotent if a mapping already exists, so there is no reason to
	 * ever prefer find_mapping() here.
	 *
	 * Both mappings are created before any handler is installed, so a
	 * failure here returns with no handler live on a domain devres is
	 * about to free.
	 */
	gpio_virq = irq_create_mapping(s->domain, SAMCOP_IRQ_GPIO);
	if (!gpio_virq)
		return -ENXIO;

	eps_virq = irq_create_mapping(s->domain, SAMCOP_IRQ_PCMCIA);
	if (!eps_virq)
		return -ENXIO;

	/* ASIC_INT_N (PXA GPIO10) is active low.
	 *
	 * Arm the parent before the two sub-demux handlers below are
	 * installed: installing a chained handler starts its virq, and that
	 * unmasks the routing bit. The PXA GPIO has edge detection only, so
	 * a routing bit that latches into IC_INTPND with the parent not yet
	 * armed holds ASIC_INT_N low with no edge ever seen.
	 */
	irq_set_irq_type(s->irq, IRQ_TYPE_EDGE_FALLING);
	irq_set_chained_handler_and_data(s->irq, samcop_irq_handler, s);
	irq_set_chained_handler_and_data(gpio_virq, samcop_irq_gpio_chained, s);
	irq_set_chained_handler_and_data(eps_virq, samcop_irq_eps_chained, s);

	/*
	 * Sources 25 (GPIO) and 5 (PCMCIA) are pure routing plumbing with
	 * no "device" of their own ever calling request_irq()/enable_irq()
	 * on them -- the two chained sub-demux handlers just installed are
	 * their only consumer. Unmask them here, permanently; every other
	 * IC source stays masked until whatever sub-device driver requests
	 * it unmasks it through the normal irq_chip path.
	 */
	samcop_rmw(s, SAMCOP_IC_INTMSK, BIT(SAMCOP_IRQ_GPIO) | BIT(SAMCOP_IRQ_PCMCIA), 0);

	return devm_add_action_or_reset(s->dev, samcop_irq_teardown, s);
}

/* ---------------------------------------------------------------------
 * GPIO chip: five banks (A-E), 32-line-spaced with holes -- see
 * include/linux/mfd/samcop.h's addressing comment. GPA's CON1/CON2
 * split is the one two-register special case; every other bank has a
 * single CON/DAT/PUP register set.
 * ---------------------------------------------------------------------
 */

static const u32 samcop_gpio_con_regs[SAMCOP_GPIO_NR_BANKS] = {
	[SAMCOP_GPIO_BANK_A] = SAMCOP_GPIO_GPA_CON1,
	[SAMCOP_GPIO_BANK_B] = SAMCOP_GPIO_GPB_CON,
	[SAMCOP_GPIO_BANK_C] = SAMCOP_GPIO_GPC_CON,
	[SAMCOP_GPIO_BANK_D] = SAMCOP_GPIO_GPD_CON,
	[SAMCOP_GPIO_BANK_E] = SAMCOP_GPIO_GPE_CON,
};

static const u32 samcop_gpio_dat_regs[SAMCOP_GPIO_NR_BANKS] = {
	[SAMCOP_GPIO_BANK_A] = SAMCOP_GPIO_GPA_DAT,
	[SAMCOP_GPIO_BANK_B] = SAMCOP_GPIO_GPB_DAT,
	[SAMCOP_GPIO_BANK_C] = SAMCOP_GPIO_GPC_DAT,
	[SAMCOP_GPIO_BANK_D] = SAMCOP_GPIO_GPD_DAT,
	[SAMCOP_GPIO_BANK_E] = SAMCOP_GPIO_GPE_DAT,
};

static void samcop_gpio_bank_pin(unsigned int offset, unsigned int *bank,
				  unsigned int *pin)
{
	*bank = offset / SAMCOP_GPIO_BANK_STRIDE;
	*pin = offset % SAMCOP_GPIO_BANK_STRIDE;
}

/* Resolves (bank, pin) to a CON register + 2-bit field index, handling
 * GPA's CON1/CON2 split. GPB_CON is the exception to the 2-bit layout:
 * one bit per pin, set for output; the callers handle it by pin. Returns
 * -EINVAL for any pin beyond what that bank's real CON register(s) can
 * address (see samcop.h's addressing comment on the GPB pin-16/WAN5
 * vendor-header inconsistency).
 */
static int samcop_gpio_con_info(unsigned int bank, unsigned int pin, u32 *reg,
				 unsigned int *field)
{
	if (bank == SAMCOP_GPIO_BANK_A && pin >= 16) {
		if (pin >= SAMCOP_GPIO_GPA_NR_PINS)
			return -EINVAL;
		*reg = SAMCOP_GPIO_GPA_CON2;
		*field = pin - 16;
		return 0;
	}

	if (pin >= SAMCOP_GPIO_PINS_PER_BANK)
		return -EINVAL;

	*reg = samcop_gpio_con_regs[bank];
	*field = pin;
	return 0;
}

static int samcop_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct samcop *s = gpiochip_get_data(gc);
	unsigned int bank, pin, field;
	u32 reg, con;
	int ret;

	samcop_gpio_bank_pin(offset, &bank, &pin);
	ret = samcop_gpio_con_info(bank, pin, &reg, &field);
	if (ret)
		return ret;

	if (bank == SAMCOP_GPIO_BANK_B)
		return samcop_read(s, reg) & BIT(pin) ?
			GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN;

	con = (samcop_read(s, reg) >> (2 * field)) & 0x3;

	return con == SAMCOP_GPIO_CON_MODE_OUTPUT ?
		GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN;
}

static int samcop_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	struct samcop *s = gpiochip_get_data(gc);
	unsigned int bank, pin, field;
	u32 reg;
	int ret;

	samcop_gpio_bank_pin(offset, &bank, &pin);
	ret = samcop_gpio_con_info(bank, pin, &reg, &field);
	if (ret)
		return ret;

	/*
	 * A GPA pin in EXTIN mode is already an input: the factory keyboard
	 * driver reads the record button from GPA_DAT with its pin in EXTIN.
	 * Switching it to plain input would cut the pin off from its
	 * interrupt source, and nothing would ever switch it back.
	 */
	if (bank == SAMCOP_GPIO_BANK_A &&
	    ((samcop_read(s, reg) >> (2 * field)) & 0x3) ==
	    SAMCOP_GPIO_CON_MODE_EXTIN)
		return 0;

	if (bank == SAMCOP_GPIO_BANK_B)
		samcop_rmw(s, reg, BIT(pin), 0);
	else
		samcop_rmw(s, reg, SAMCOP_GPIO_CON_MASK(field), 0);

	return 0;
}

static int samcop_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct samcop *s = gpiochip_get_data(gc);
	unsigned int bank, pin, field;
	u32 reg;
	int ret;

	samcop_gpio_bank_pin(offset, &bank, &pin);
	/* Reject the same bank-width "holes" the direction callbacks do. */
	ret = samcop_gpio_con_info(bank, pin, &reg, &field);
	if (ret)
		return ret;

	return !!(samcop_read(s, samcop_gpio_dat_regs[bank]) & BIT(pin));
}

static int samcop_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct samcop *s = gpiochip_get_data(gc);
	unsigned int bank, pin, field;
	u32 reg;
	int ret;

	samcop_gpio_bank_pin(offset, &bank, &pin);
	/* Reject the same bank-width "holes" the direction callbacks do. */
	ret = samcop_gpio_con_info(bank, pin, &reg, &field);
	if (ret)
		return ret;

	samcop_rmw(s, samcop_gpio_dat_regs[bank], BIT(pin), value ? BIT(pin) : 0);

	return 0;
}

static int samcop_gpio_direction_output(struct gpio_chip *gc, unsigned int offset,
					 int value)
{
	struct samcop *s = gpiochip_get_data(gc);
	unsigned int bank, pin, field;
	u32 reg;
	int ret;

	/* Set the level before switching direction, to avoid a glitch. */
	samcop_gpio_set(gc, offset, value);

	samcop_gpio_bank_pin(offset, &bank, &pin);
	ret = samcop_gpio_con_info(bank, pin, &reg, &field);
	if (ret)
		return ret;

	if (bank == SAMCOP_GPIO_BANK_B)
		samcop_rmw(s, reg, BIT(pin), BIT(pin));
	else
		samcop_rmw(s, reg, SAMCOP_GPIO_CON_MASK(field),
			   SAMCOP_GPIO_CON_OUTPUT(field));

	return 0;
}

/*
 * Sparse offset-to-hwirq table: only ~20 of SAMCOP_NR_GPIO (160)
 * addressable offsets have a real interrupt behind them, split across
 * two entirely different mechanisms (see include/linux/mfd/samcop.h):
 * GPA0-3 and GPA12 identity-map to IC-tier hwirqs (APPBUTTON1-4,
 * RECORD); GPA4-11, GPA18 and GPB12-16 map to the GPIO-fanout tier.
 * Transcribed from the 2.4/2.6 driver's samcop_gpio_to_irq().
 */
static const struct {
	u16 offset;
	u16 hwirq;
} samcop_gpio_irq_map[] = {
	{ SAMCOP_GPIO(A, 0),	SAMCOP_IRQ_APPBUTTON1 },
	{ SAMCOP_GPIO(A, 1),	SAMCOP_IRQ_APPBUTTON2 },
	{ SAMCOP_GPIO(A, 2),	SAMCOP_IRQ_APPBUTTON3 },
	{ SAMCOP_GPIO(A, 3),	SAMCOP_IRQ_APPBUTTON4 },
	{ SAMCOP_GPIO(A, 4),	SAMCOP_IRQ_GPIO_GPA4 },
	{ SAMCOP_GPIO(A, 5),	SAMCOP_IRQ_GPIO_GPA5 },
	{ SAMCOP_GPIO(A, 6),	SAMCOP_IRQ_GPIO_GPA6 },
	{ SAMCOP_GPIO(A, 7),	SAMCOP_IRQ_GPIO_GPA7 },
	{ SAMCOP_GPIO(A, 8),	SAMCOP_IRQ_GPIO_GPA8 },
	{ SAMCOP_GPIO(A, 9),	SAMCOP_IRQ_GPIO_GPA9 },
	{ SAMCOP_GPIO(A, 10),	SAMCOP_IRQ_GPIO_GPA10 },
	{ SAMCOP_GPIO(A, 11),	SAMCOP_IRQ_GPIO_GPA11 },
	{ SAMCOP_GPIO(A, 12),	SAMCOP_IRQ_RECORD },
	{ SAMCOP_GPIO(A, 18),	SAMCOP_IRQ_GPIO_GPA18 },
	{ SAMCOP_GPIO(B, 12),	SAMCOP_IRQ_GPIO_GPB12 },
	{ SAMCOP_GPIO(B, 13),	SAMCOP_IRQ_GPIO_GPB13 },
	{ SAMCOP_GPIO(B, 14),	SAMCOP_IRQ_GPIO_GPB14 },
	{ SAMCOP_GPIO(B, 15),	SAMCOP_IRQ_GPIO_GPB15 },
	{ SAMCOP_GPIO(B, 16),	SAMCOP_IRQ_GPIO_GPB16 },
};

static int samcop_gpio_to_irq(struct gpio_chip *gc, unsigned int offset)
{
	struct samcop *s = gpiochip_get_data(gc);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(samcop_gpio_irq_map); i++) {
		if (samcop_gpio_irq_map[i].offset != offset)
			continue;

		/* irq_create_mapping(), not irq_find_mapping() -- see
		 * samcop_irq_init()'s comment on this exact bug class.
		 */
		return irq_create_mapping(s->domain, samcop_gpio_irq_map[i].hwirq);
	}

	return -ENXIO;
}

static int samcop_gpio_register(struct samcop *s)
{
	s->gc.label = "samcop";
	s->gc.parent = s->dev;
	s->gc.owner = THIS_MODULE;
	s->gc.get_direction = samcop_gpio_get_direction;
	s->gc.direction_input = samcop_gpio_direction_input;
	s->gc.direction_output = samcop_gpio_direction_output;
	s->gc.get = samcop_gpio_get;
	s->gc.set = samcop_gpio_set;
	s->gc.to_irq = samcop_gpio_to_irq;
	s->gc.base = -1;
	s->gc.ngpio = SAMCOP_NR_GPIO;
	s->gc.can_sleep = false;

	return devm_gpiochip_add_data(s->dev, &s->gc, s);
}

/* ---------------------------------------------------------------------
 * Clocks (CCF): one "samcop-gclk" parent, computed from the PXA25x "memory"
 * bus clock (clk-pxa25x.c registers it under exactly that global name), plus
 * eleven gate clk_hws on CPM_ClockControl bits. The custom clk_ops follow
 * hamcop.c; ClockControl is a 32-bit register here, so the generic gate
 * helper would work too, but keeping the form of both MFD drivers' clock code
 * the same helps reading them side by side.
 *
 * The gclk rate is memory clock, divided by 1 or 2 on a ClockSleep bit, as in
 * hamcop_gclk_recalc_rate() and in the 2.4/2.6 driver's samcop_gclk():
 *
 *	half_clk ? memory_clock : memory_clock / 2
 *
 * The SD host takes its f_min/f_max from this rate, so probe fails there
 * without it. The bit's name (HALF_CLK) and its effect are opposites: set
 * means the full memory clock, clear means memory / 2. That follows the
 * 2.4/2.6 code, corroborated by HAMCOP's identically shaped CLKSEL, which is
 * confirmed on real h2200 hardware, and not the name.
 * ---------------------------------------------------------------------
 */

static unsigned long samcop_gclk_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct samcop_gclk *gclk = container_of(hw, struct samcop_gclk, hw);
	u32 sleep = samcop_read(gclk->samcop, SAMCOP_CPM_CLOCKSLEEP);

	return (sleep & SAMCOP_CLKSLEEP_HALF_CLK) ? parent_rate : parent_rate / 2;
}

static const struct clk_ops samcop_gclk_ops = {
	.recalc_rate	= samcop_gclk_recalc_rate,
};

static int samcop_clk_gate_enable(struct clk_hw *hw)
{
	struct samcop_clk_gate *gate = container_of(hw, struct samcop_clk_gate, hw);

	samcop_rmw(gate->samcop, SAMCOP_CPM_CLOCKCONTROL, gate->bit, gate->bit);

	return 0;
}

static void samcop_clk_gate_disable(struct clk_hw *hw)
{
	struct samcop_clk_gate *gate = container_of(hw, struct samcop_clk_gate, hw);

	samcop_rmw(gate->samcop, SAMCOP_CPM_CLOCKCONTROL, gate->bit, 0);
}

static int samcop_clk_gate_is_enabled(struct clk_hw *hw)
{
	struct samcop_clk_gate *gate = container_of(hw, struct samcop_clk_gate, hw);

	return !!(samcop_read(gate->samcop, SAMCOP_CPM_CLOCKCONTROL) & gate->bit);
}

static const struct clk_ops samcop_clk_gate_ops = {
	.enable		= samcop_clk_gate_enable,
	.disable	= samcop_clk_gate_disable,
	.is_enabled	= samcop_clk_gate_is_enabled,
};

static const struct {
	const char *name;
	u32 bit;
} samcop_clk_defs[SAMCOP_NR_CLK_GATES] = {
	[SAMCOP_GATE_USBHOST]	= { "samcop-usbhost",	SAMCOP_CLK_USBHOST },
	[SAMCOP_GATE_DMAC]	= { "samcop-dma",	SAMCOP_CLK_DMAC },
	[SAMCOP_GATE_GPIO]	= { "samcop-gpio",	SAMCOP_CLK_GPIO },
	[SAMCOP_GATE_FCD]	= { "samcop-fsi",	SAMCOP_CLK_FCD },
	[SAMCOP_GATE_SD]	= { "samcop-sd",	SAMCOP_CLK_SD },
	[SAMCOP_GATE_ADC]	= { "samcop-adc-clk",	SAMCOP_CLK_ADC },
	[SAMCOP_GATE_UART2]	= { "samcop-uart2",	SAMCOP_CLK_UART2 },
	[SAMCOP_GATE_UART1]	= { "samcop-uart1",	SAMCOP_CLK_UART1 },
	[SAMCOP_GATE_LED]	= { "samcop-led",	SAMCOP_CLK_LED },
	[SAMCOP_GATE_MISC]	= { "samcop-misc",	SAMCOP_CLK_MISC },
	[SAMCOP_GATE_1WIRE]	= { "samcop-1wire",	SAMCOP_CLK_1WIRE },
};

static const char * const samcop_gclk_parents[] = { "memory" };
static const char * const samcop_gate_parents[] = { "samcop-gclk" };

static int samcop_clocks_register(struct samcop *s)
{
	struct clk_init_data gclk_init = {
		.name		= "samcop-gclk",
		.ops		= &samcop_gclk_ops,
		.parent_names	= samcop_gclk_parents,
		.num_parents	= ARRAY_SIZE(samcop_gclk_parents),
	};
	int i, ret;

	s->gclk.samcop = s;
	s->gclk.hw.init = &gclk_init;
	ret = devm_clk_hw_register(s->dev, &s->gclk.hw);
	if (ret)
		return ret;

	for (i = 0; i < SAMCOP_NR_CLK_GATES; i++) {
		struct clk_init_data gate_init = {
			.name	= samcop_clk_defs[i].name,
			.ops	= &samcop_clk_gate_ops,
			.parent_names	= samcop_gate_parents,
			.num_parents	= ARRAY_SIZE(samcop_gate_parents),
			/*
			 * GPIO is forced on directly in probe() (our own
			 * gpio_chip needs it immediately, before any
			 * sub-device probes) but that write doesn't give
			 * the gate a nonzero CCF enable_count, so without
			 * CLK_IS_CRITICAL the late_initcall_sync
			 * clk_disable_unused() pass would turn it back off
			 * shortly after boot -- exactly the same reasoning
			 * hamcop.c documents for its own GPIO/NAND gates.
			 */
			.flags	= (i == SAMCOP_GATE_GPIO) ? CLK_IS_CRITICAL : 0,
		};

		s->gates[i].samcop = s;
		s->gates[i].bit = samcop_clk_defs[i].bit;
		s->gates[i].hw.init = &gate_init;

		ret = devm_clk_hw_register(s->dev, &s->gates[i].hw);
		if (ret)
			return ret;
	}

	/*
	 * clkdev aliases for the sub-device drivers to clk_get() against:
	 * chip-internal wiring, not board policy, as in hamcop.c. There is no RTC
	 * alias for "leds-samcop" as HAMCOP's LED cell needed, because SAMCOP's
	 * ClockControl has no separate RTC gate bit.
	 */
	ret = devm_clk_hw_register_clkdev(s->dev, &s->gates[SAMCOP_GATE_SD].hw,
					   NULL, "samcop-sdi");
	if (ret)
		return ret;

	ret = devm_clk_hw_register_clkdev(s->dev, &s->gates[SAMCOP_GATE_LED].hw,
					   NULL, "leds-samcop");
	if (ret)
		return ret;

	ret = devm_clk_hw_register_clkdev(s->dev, &s->gates[SAMCOP_GATE_ADC].hw,
					   NULL, "samcop-adc");
	if (ret)
		return ret;

	ret = devm_clk_hw_register_clkdev(s->dev, &s->gates[SAMCOP_GATE_1WIRE].hw,
					   NULL, "ds1wm");
	if (ret)
		return ret;

	ret = devm_clk_hw_register_clkdev(s->dev, &s->gates[SAMCOP_GATE_USBHOST].hw,
					   NULL, "samcop-ohci");
	if (ret)
		return ret;

	/*
	 * "fsi" cell: CPM_ClockControl bit 4, SAMCOP_CLK_FCD, the fingerprint
	 * block's clock gate. Same NULL con_id and cell-name dev_id as every
	 * other alias here; the fingerprint driver calls devm_clk_get_enabled(dev,
	 * NULL).
	 */
	return devm_clk_hw_register_clkdev(s->dev, &s->gates[SAMCOP_GATE_FCD].hw,
					    NULL, "samcop-fsi");
}

/* ---------------------------------------------------------------------
 * Sub-devices. The fingerprint cell is registered unconditionally like every
 * other cell: it is h54xx/h55xx-only hardware, but nothing in the devicetree
 * distinguishes an h51xx from an h54xx to gate it on, and the register window
 * exists and probes cleanly whether or not a sensor is wired to it.
 * ---------------------------------------------------------------------
 */

/*
 * Two hwirqs, not one. SAMCOP_IRQ_SD (12) is the SDI block's own
 * command/data/FIFO interrupt; SAMCOP_IRQ_SD_WAKEUP (13) is the separate
 * IC-tier source backed by the EXTIN-muxed GPA16 card-detect pin (see
 * include/linux/mfd/samcop.h's SAMCOP_GPIO_GPA_SD_DETECT comment). The
 * wakeup line is handed to the MMC core as the card-detect interrupt. The
 * order matters: the SD host takes platform_get_irq(pdev, 0) as the block IRQ
 * and platform_get_irq_optional(pdev, 1) as card-detect.
 */
static const struct resource samcop_sdi_resources[] = {
	DEFINE_RES_MEM(SAMCOP_SDI_BASE, SAMCOP_SDI_SIZE),
	DEFINE_RES_IRQ(SAMCOP_IRQ_SD),
	DEFINE_RES_IRQ(SAMCOP_IRQ_SD_WAKEUP),
};

static const struct resource samcop_adc_resources[] = {
	DEFINE_RES_MEM(SAMCOP_ADC_BASE, SAMCOP_ADC_SIZE),
	DEFINE_RES_IRQ(SAMCOP_IRQ_ADC),
	DEFINE_RES_IRQ(SAMCOP_IRQ_ADCTS),
};

/*
 * bus_shift=2: every other SAMCOP register in this MFD is accessed
 * 32-bit-strided (see include/linux/mfd/samcop.h's OWM size comment), and the
 * ds1wm driver documents 2 as the value on h5xxx and h2200. The chip exposes no
 * trigger-polarity register for the ONEWIRE IC source, so active_high = 0 is
 * DS1WM's power-on-reset default, unverified on hardware.
 */
static const struct ds1wm_driver_data samcop_ds1wm_pdata = {
	.bus_shift		= 2,
	.active_high		= 0,
	.reset_recover_delay	= 1,
};

static const struct resource samcop_ds1wm_resources[] = {
	DEFINE_RES_MEM(SAMCOP_OWM_BASE, SAMCOP_OWM_SIZE),
	DEFINE_RES_IRQ(SAMCOP_IRQ_ONEWIRE),
};

static const struct resource samcop_led_resources[] = {
	DEFINE_RES_MEM(SAMCOP_LED_BASE, SAMCOP_LED_SIZE),
};

/*
 * Two mem resources: the USB host controller's own register window, and
 * SAMCOP's 32 KiB SRAM as the local-memory pool for
 * usb_hcd_setup_local_mem(). SAMCOP's DMA master can only reach its own SRAM,
 * not system RAM, so the OHCI cell owns that window directly.
 */
static const struct resource samcop_ohci_resources[] = {
	DEFINE_RES_MEM(SAMCOP_USBHOST_BASE, SAMCOP_USBHOST_SIZE),
	DEFINE_RES_MEM(SAMCOP_SRAM_BASE, SAMCOP_SRAM_SIZE),
	DEFINE_RES_IRQ(SAMCOP_IRQ_USBH),
};

/*
 * drivers/misc/samcop-fsi.c: 32-byte register window and hwirq 6 (FCD). No DMA
 * resource: the block encodes a DMA read mode that was never used.
 */
static const struct resource samcop_fsi_resources[] = {
	DEFINE_RES_MEM(SAMCOP_FSI_BASE, SAMCOP_FSI_SIZE),
	DEFINE_RES_IRQ(SAMCOP_IRQ_FCD),
};

/*
 * drivers/misc/samcop-sleeve.c: the h5xxx expansion-pack ("sleeve")
 * connector's EPS status, IRQ and control register block (card-detect, IRQ
 * and option-detect lines, edge-type config, card-reset control), plus the
 * option-detect hwirq that the generic ipaq-sleeve bus core's presence
 * interrupt binds to.
 * The cell requests SAMCOP_IRQ_EPS_ODET (hwirq 44) itself; CD0/CD1/IRQ0/IRQ1
 * are handed on to a PCMCIA socket consumer through the backend's egpio_irq()
 * vtable. BATT_FLT is both read off the EPS register and requested as its own
 * demuxed interrupt, SAMCOP_IRQ_EPS_BATT_FAULT, which the backend maps
 * through samcop_hwirq_to_irq() and triggers on both edges
 * (SAMCOP_PCMCIA_IM_DEFAULT) so the sleeve bus core sees a fault asserting
 * and clearing. The SPI transport (the ID EEPROM read) and the two SAMCOP GPA
 * power-control lines (OPT_ON and OPT_RESET) are resolved by the cell driver
 * itself, so neither needs a resource here. OPT_NVRAM_ON is a plain PXA GPIO
 * on the board and is not described by this chip.
 */

/*
 * The order matters: drivers/misc/samcop-sleeve.c indexes these positionally
 * with platform_get_irq(), ODET at 0 (its own presence interrupt) and
 * CD0/CD1/IRQ0/IRQ1 at 1-4 (handed on to a PCMCIA socket consumer through the
 * ipaq_sleeve_ops egpio_irq() vtable).
 *
 * These are five distinct hwirqs, not one aggregated PCMCIA interrupt: the EPS
 * irq chip demuxes SAMCOP_IRQ_PCMCIA into hwirq 40-45, so card-detect and
 * card-IRQ each reach Linux as their own virq. devm_mfd_add_devices() is
 * passed this chip's irq_domain, so the MFD core does the hwirq -> virq
 * translation and the backend only ever sees finished virqs.
 */
static const struct resource samcop_sleeve_resources[] = {
	DEFINE_RES_MEM(SAMCOP_PCMCIA_BASE, SAMCOP_PCMCIA_SIZE),
	DEFINE_RES_IRQ(SAMCOP_IRQ_EPS_ODET),
	DEFINE_RES_IRQ(SAMCOP_IRQ_EPS_CD0),
	DEFINE_RES_IRQ(SAMCOP_IRQ_EPS_CD1),
	DEFINE_RES_IRQ(SAMCOP_IRQ_EPS_IRQ0),
	DEFINE_RES_IRQ(SAMCOP_IRQ_EPS_IRQ1),
};

static const struct mfd_cell samcop_cells[] = {
	{
		/*
		 * drivers/mmc/host/samcop-sdi.c: PIO-only, the same S3C24xx-class
		 * SDI IP as HAMCOP's cell. Card-detect, write-protect and slot
		 * power are board wiring, not SAMCOP registers.
		 */
		.name		= "samcop-sdi",
		.id		= PLATFORM_DEVID_NONE,
		.of_compatible	= "samsung,samcop-sdi",
		.resources	= samcop_sdi_resources,
		.num_resources	= ARRAY_SIZE(samcop_sdi_resources),
	},
	{
		/*
		 * drivers/iio/adc/samcop-adc.c: IIO front-end for the shared
		 * ADC/touchscreen block, owns both its hwirqs.
		 */
		.name		= "samcop-adc",
		.id		= PLATFORM_DEVID_NONE,
		.of_compatible	= "samsung,samcop-adc",
		.resources	= samcop_adc_resources,
		.num_resources	= ARRAY_SIZE(samcop_adc_resources),
	},
	{
		/*
		 * mainline's generic resistive-adc-touch driver, consuming
		 * "samcop-adc"'s four touch channels -- no register access or
		 * IRQ of its own, so no resources. Wired through the DT
		 * "io-channels" phandle, so the lookup returns -EPROBE_DEFER and
		 * is retried until the ADC provider has registered.
		 */
		.name		= "resistive-adc-touch",
		.id		= PLATFORM_DEVID_NONE,
		.of_compatible	= "resistive-adc-touch",
	},
	{
		/*
		 * DS1WM-compatible 1-Wire master (drivers/w1/masters/ds1wm.c).
		 */
		.name		= "ds1wm",
		.id		= PLATFORM_DEVID_NONE,
		.of_compatible	= "samsung,samcop-onewire",
		.platform_data	= &samcop_ds1wm_pdata,
		.pdata_size	= sizeof(samcop_ds1wm_pdata),
		.resources	= samcop_ds1wm_resources,
		.num_resources	= ARRAY_SIZE(samcop_ds1wm_resources),
	},
	{
		/*
		 * drivers/leds/leds-samcop.c: 5 channels, described as child
		 * nodes of the devicetree node.
		 */
		.name		= "leds-samcop",
		.id		= PLATFORM_DEVID_NONE,
		.of_compatible	= "samsung,samcop-leds",
		.resources	= samcop_led_resources,
		.num_resources	= ARRAY_SIZE(samcop_led_resources),
	},
	{
		/*
		 * drivers/usb/host/samcop-ohci.c: usb_hcd_setup_local_mem() over
		 * the SRAM resource above, because the DMA master can only reach
		 * the SRAM and not system RAM.
		 */
		.name		= "samcop-ohci",
		.id		= PLATFORM_DEVID_NONE,
		.of_compatible	= "samsung,samcop-ohci",
		.resources	= samcop_ohci_resources,
		.num_resources	= ARRAY_SIZE(samcop_ohci_resources),
	},
	{
		/*
		 * drivers/misc/samcop-fsi.c: h54xx/h55xx fingerprint sensor
		 * raw-acquisition cell, registered unconditionally.
		 */
		.name		= "samcop-fsi",
		.id		= PLATFORM_DEVID_NONE,
		.of_compatible	= "samsung,samcop-fsi",
		.resources	= samcop_fsi_resources,
		.num_resources	= ARRAY_SIZE(samcop_fsi_resources),
	},
	{
		/*
		 * drivers/misc/samcop-sleeve.c, the h5xxx SAMCOP EPS backend for
		 * the generic ipaq-sleeve bus core (drivers/misc/ipaq-sleeve.c),
		 * see samcop_sleeve_resources above.
		 */
		.name		= "samcop-sleeve",
		.id		= PLATFORM_DEVID_NONE,
		.of_compatible	= "samsung,samcop-sleeve",
		.resources	= samcop_sleeve_resources,
		.num_resources	= ARRAY_SIZE(samcop_sleeve_resources),
	},
	{
		/*
		 * drivers/power/supply/gpio-charger.c for the AC adapter, matched
		 * by that driver's own "gpio-charger" compatible against the single
		 * such DT child of this chip. No .resources: the power_supply name
		 * and GPIOs all come from that node. A cell with no matching child
		 * would get no of_node, and gpio-charger fails such a device with
		 * -ENOENT.
		 */
		.name		= "gpio-charger",
		.id		= 0,
		.of_compatible	= "gpio-charger",
	},
	{
		/*
		 * drivers/input/keyboard/gpio_keys.c for the 12 of the board's 13
		 * buttons that go through SAMCOP (KEY_POWER is a plain PXA GPIO
		 * and sits in a top-level "gpio-keys" node).
		 *
		 * These are an mfd cell, not folded into that node, to route
		 * around a gpio_keys bug: gpio_keys_get_devtree_pdata() resolves
		 * each button's IRQ with of_irq_get_byname() or
		 * irq_of_parse_and_map() and only checks `if (irq > 0)`, so an
		 * -EPROBE_DEFER is dropped and never retried, leaving that button
		 * with no IRQ if it loses the probe race against this driver's
		 * irq_domain. devm_mfd_add_devices() runs after
		 * devm_irq_domain_instantiate() in samcop_probe(), so the domain
		 * always exists by the time this cell's gpio-keys is created.
		 *
		 * No .resources: gpio-keys is matched by its own "gpio-keys"
		 * compatible against this cell's DT child, and every button's
		 * GPA offset comes from that node's "interrupts" cells.
		 *
		 * Explicit .id 0 rather than PLATFORM_DEVID_NONE: that would name
		 * this device plain "gpio-keys", the same as the top-level node's
		 * device. They have different parents so they do not collide, but
		 * the id keeps them distinct in sysfs and dmesg.
		 */
		.name		= "gpio-keys",
		.id		= 0,
		.of_compatible	= "gpio-keys",
	},
};

/* ---------------------------------------------------------------------
 * Probe/lifecycle
 * ---------------------------------------------------------------------
 */

static int samcop_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct samcop *s;
	struct resource *mem;
	int irq, ret, i;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->dev = dev;
	raw_spin_lock_init(&s->lock);
	platform_set_drvdata(pdev, s);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -EINVAL;

	s->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(s->regs))
		return PTR_ERR(s->regs);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	s->irq = irq;

	/*
	 * Bring the GPIO block clock up first: everything below, including
	 * the IRQ mask and ack init in samcop_irq_init(), touches GPIO-block
	 * registers. UCLK_EN (bit 0) is never set by the 2.4/2.6 driver
	 * either, so it is left clear.
	 */
	samcop_write(s, SAMCOP_CPM_CLOCKCONTROL, SAMCOP_CLK_GPIO);
	samcop_write(s, SAMCOP_CPM_CLOCKSLEEP, samcop_init.clocksleep);
	samcop_write(s, SAMCOP_CPM_PLLCONTROL, samcop_init.pllcontrol);

	ret = samcop_irq_init(s);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set up IRQ domain\n");

	/*
	 * Board GPIO init, from the h5400 values the 2.4/2.6 driver used. The
	 * gpioint[], gpioflt[] and gpioenint[] values are opaque raw register
	 * contents, not decoded per pin, like clocksleep and pllcontrol above.
	 *
	 * gpioenint[] arms GPIO fan-out sources in hardware ahead of any
	 * consumer's request_irq() on the matching hwirq. A source that fires
	 * before something maps and enables its hwirq is a spurious demux
	 * dispatch, bounded by SAMCOP_MAX_IRQ_LOOPS.
	 */
	samcop_write(s, SAMCOP_GPIO_GPA_CON1, samcop_init.gpacon1);
	samcop_write(s, SAMCOP_GPIO_GPA_CON2, samcop_init.gpacon2);
	samcop_write(s, SAMCOP_GPIO_GPA_DAT, samcop_init.gpadat);
	samcop_write(s, SAMCOP_GPIO_GPA_PUP, samcop_init.gpaup);

	for (i = 0; i < ARRAY_SIZE(samcop_init.gpioint); i++)
		samcop_write(s, SAMCOP_GPIO_INT(i), samcop_init.gpioint[i]);

	for (i = 0; i < ARRAY_SIZE(samcop_init.gpioflt); i++)
		samcop_write(s, SAMCOP_GPIO_FLTCONFIG(i), samcop_init.gpioflt[i]);

	samcop_write(s, SAMCOP_GPIO_ENINT1, samcop_init.gpioenint[0]);
	samcop_write(s, SAMCOP_GPIO_ENINT2, samcop_init.gpioenint[1]);

	/* EPS default edge config, see SAMCOP_PCMCIA_IM_DEFAULT's comment. */
	samcop_write(s, SAMCOP_PCMCIA_IM, SAMCOP_PCMCIA_IM_DEFAULT);

	ret = samcop_clocks_register(s);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register clocks\n");

	ret = samcop_gpio_register(s);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register gpio_chip\n");

	/*
	 * Top-level id=0, PLATFORM_DEVID_NONE on every cell: mfd_add_device()
	 * computes each child's platform_device id as id + cell->id, so this
	 * combination yields unnumbered devices for each singleton sub
	 * -device -- matching hamcop.c's own convention (itself matching
	 * drivers/mfd/ioc3.c).
	 */
	ret = devm_mfd_add_devices(dev, 0, samcop_cells, ARRAY_SIZE(samcop_cells),
				    mem, 0, s->domain);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add sub-devices\n");

	dev_info(dev, "SAMCOP companion ASIC ready, chained on IRQ %d\n", irq);

	return 0;
}

/*
 * Public API: convert a SAMCOP hwirq to a Linux IRQ number, like
 * hamcop_hwirq_to_irq(). For consumers that resolve IC-tier button hwirqs
 * (SAMCOP_IRQ_APPBUTTON1 etc.) that samcop_gpio_to_irq()'s table also reaches
 * through the gpio_chip, and any hwirq with no GPIO-chip path at all (the
 * fixed-function IC sources 6-25 excluding the two demux bits).
 */
int samcop_hwirq_to_irq(struct device *samcop_dev, unsigned int hwirq)
{
	struct samcop *s = dev_get_drvdata(samcop_dev);
	int irq;

	if (!s || !s->domain) {
		pr_err("samcop: invalid device or missing irq_domain\n");
		return -EINVAL;
	}

	if (hwirq >= SAMCOP_NR_IRQS) {
		pr_err("samcop: hwirq %u out of range\n", hwirq);
		return -EINVAL;
	}

	/* irq_create_mapping(), not irq_find_mapping() -- see
	 * samcop_irq_init()'s comment on this exact bug class.
	 */
	irq = irq_create_mapping(s->domain, hwirq);
	if (irq <= 0) {
		pr_err("samcop: cannot map hwirq %u to IRQ\n", hwirq);
		return -EINVAL;
	}

	return irq;
}
EXPORT_SYMBOL_GPL(samcop_hwirq_to_irq);

/*
 * Mux GPD0-7 and GPE0-2 to the FSI/FCD hardware sequencer's alternate
 * function -- not driven as gpiod GPIOs. GPD_CON = 0xaaaa selects the
 * FCD_DATA0..7 alternate function on GPD's eight 2-bit CON fields (encoding
 * 0x2 per pin); GPE_CON = 0x7 selects FCD_RESET/FCD_PCLK/FCD_TPE on GPE0-2.
 * ACKN and OE have no GPD/GPE bit and are presumed sequencer-internal.
 *
 * These two values equal GPD_CON/GPE_CON's reset defaults, and are written
 * anyway because nothing guarantees the bootloader or a suspend cycle left
 * them there; the fingerprint driver calls this at every open().
 */
int samcop_reset_fcd(struct device *samcop_dev)
{
	struct samcop *s = dev_get_drvdata(samcop_dev);
	unsigned long flags;

	if (!s)
		return -EINVAL;

	raw_spin_lock_irqsave(&s->lock, flags);
	samcop_write(s, SAMCOP_GPIO_GPD_CON, 0xaaaa);
	samcop_write(s, SAMCOP_GPIO_GPE_CON, 0x7);
	raw_spin_unlock_irqrestore(&s->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(samcop_reset_fcd);

/*
 * Public API for sub-cell drivers: set/clear bits in the shared SPCR pull-up
 * control register, a masked read-modify-write under the chip lock. SPCR sits
 * in this MFD core's register window, not in any cell's memory resource, so a
 * cell driver cannot map it itself.
 */
void samcop_set_spcr(struct device *samcop_dev, u32 mask, u32 bits)
{
	struct samcop *s;

	/* A cell driver reaches this through its own dev->parent; guard the
	 * case where it is somehow bound outside this MFD rather than
	 * dereferencing a NULL device.
	 */
	if (WARN_ON(!samcop_dev))
		return;

	s = dev_get_drvdata(samcop_dev);
	if (WARN_ON(!s))
		return;

	samcop_rmw(s, SAMCOP_GPIO_SPCR, mask, bits & mask);
}
EXPORT_SYMBOL_GPL(samcop_set_spcr);

/*
 * Public API for sub-cell drivers: set/clear bits in GPA_PUP, the GPIO-A
 * bank's per-pin pull-up register, for the same reason as samcop_set_spcr()
 * above. The GPIO-A gpio_chip has no .set_config, so there is no generic
 * pinconf/bias path to it.
 */
void samcop_set_gpio_a_pullup(struct device *samcop_dev, u32 mask, u32 bits)
{
	struct samcop *s;

	if (WARN_ON(!samcop_dev))
		return;

	s = dev_get_drvdata(samcop_dev);
	if (WARN_ON(!s))
		return;

	samcop_rmw(s, SAMCOP_GPIO_GPA_PUP, mask, bits & mask);
}
EXPORT_SYMBOL_GPL(samcop_set_gpio_a_pullup);

/* ---------------------------------------------------------------------
 * Suspend/resume: save IC_INTMSK and ClockControl, restore ClockControl
 * first and then IC_INTMSK. Chip state beyond those two registers is not
 * saved here.
 * ---------------------------------------------------------------------
 */

/*
 * samcop.selective_suspend_mask=1 stops samcop_suspend() force-masking all
 * 26 IC sources and leaves the masking to genirq. The blanket write also
 * masks the wake-named sources (SD_WAKEUP, WAKEUP1-3), so a source armed
 * with enable_irq_wake() would be masked out from under itself, and the
 * chip has no separate wake register that survives that. With the write
 * skipped, suspend_device_irqs() masks each non-wake source through the
 * irq_chip callbacks (IRQCHIP_MASK_ON_SUSPEND) and leaves a wake source
 * asserting; it runs at the head of dpm_suspend_noirq(), after this
 * callback, so the job goes to a later writer rather than to nobody.
 *
 * Off by default, because it changes what the first suspend does and the
 * blanket mask is the behaviour the vendor driver had. With it on, IC
 * sources 5 and 25 stay unmasked across sleep, so any EPS or GPIO fan-out
 * source still enabled in PCMCIA_IC or GPIO_ENINT2 can assert ASIC_INT_N
 * whether or not it was asked to be a wake source.
 */
static bool selective_suspend_mask;
module_param(selective_suspend_mask, bool, 0444);
MODULE_PARM_DESC(selective_suspend_mask,
		 "let genirq mask non-wake IC sources at suspend instead of force-masking all 26 (default off)");

static int samcop_suspend(struct device *dev)
{
	struct samcop *s = dev_get_drvdata(dev);
	unsigned long flags;

	/*
	 * No late-assert re-arm may unmask a source after the snapshot
	 * below: stop new ones being queued, then cancel a queued one.
	 * Sources it would have unmasked stay isolated until resume.
	 */
	raw_spin_lock_irqsave(&s->lock, flags);
	s->late_rearm_blocked = true;
	raw_spin_unlock_irqrestore(&s->lock, flags);
	cancel_delayed_work_sync(&s->late_rearm);

	raw_spin_lock_irqsave(&s->lock, flags);
	s->saved_clockcontrol = samcop_read(s, SAMCOP_CPM_CLOCKCONTROL);
	s->saved_intmsk = samcop_read(s, SAMCOP_IC_INTMSK);
	if (!selective_suspend_mask)
		samcop_write(s, SAMCOP_IC_INTMSK,
			     GENMASK(SAMCOP_IC_NR_IRQS - 1, 0));
	raw_spin_unlock_irqrestore(&s->lock, flags);

	return 0;
}

static int samcop_resume(struct device *dev)
{
	struct samcop *s = dev_get_drvdata(dev);
	unsigned long flags;
	unsigned int tier;

	raw_spin_lock_irqsave(&s->lock, flags);
	samcop_write(s, SAMCOP_CPM_CLOCKCONTROL, s->saved_clockcontrol);
	samcop_write(s, SAMCOP_IC_INTMSK, s->saved_intmsk);

	/* Unmask what suspend kept isolated. */
	s->late_rearm_blocked = false;
	for (tier = 0; tier < SAMCOP_NR_TIERS; tier++) {
		if (s->late_masked[tier]) {
			schedule_delayed_work(&s->late_rearm,
					      msecs_to_jiffies(SAMCOP_LATE_REARM_MS));
			break;
		}
	}
	raw_spin_unlock_irqrestore(&s->lock, flags);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(samcop_pm_ops, samcop_suspend, samcop_resume);

static struct platform_driver samcop_driver = {
	.driver = {
		.name	= "samcop",
		.pm	= pm_sleep_ptr(&samcop_pm_ops),
		.of_match_table = samcop_of_match,
	},
	.probe	= samcop_probe,
};
module_platform_driver(samcop_driver);

MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("Samsung SAMCOP companion ASIC MFD driver (HP iPAQ h5xxx)");
MODULE_LICENSE("GPL");
