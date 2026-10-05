// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung HAMCOP companion ASIC MFD driver (HP iPAQ h2200/h2210/h2212/h2215)
 *
 * HAMCOP hangs off PXA25x static-memory chip select 0 as a plain 16-bit
 * VLIO-mode MMIO device with a single upstream GPIO interrupt line. It
 * bundles a NAND controller, an SD/MMC host (SDI), an ADC + resistive
 * touchscreen front-end, a DS1WM-compatible 1-Wire master, a 5-channel LED
 * blink controller, a 32-line GPIO block that doubles as 16 of its own
 * interrupt sources, and 16 KiB of on-chip SRAM.
 *
 * This is a from-scratch rewrite against current subsystem conventions, not
 * a port of the old out-of-tree hamcop_base.c. Every 2007 wart
 * of that driver is gone: the file-scope "hd" global, the
 * h2200_led_hook global, the 20-function exported hamcop_set_gpio_*() zoo,
 * the hand-rolled subdevice-registration loop, and the pre-CCF struct clk
 * array. What replaces them:
 *
 *  - Register access: plain readw_relaxed()/writew_relaxed() plus one
 *    raw_spinlock_t for read-modify-write, not regmap-mmio -- register
 *    strides vary and the IRQ path needs lock-free reads. Modelled on
 *    drivers/mfd/ioc3.c, the closest living MFD to this shape (one
 *    upstream IRQ, flat pending/mask, mfd_add_devices()).
 *
 *  - IRQ demux: one linear irq_domain of HAMCOP_NR_IRQS hwirqs behind the
 *    single chained GPIO IRQ. hwirq 0-15 are GPIO bank A/B pins and get a
 *    real .irq_set_type backed by the GPAINT/GPBINT INTLV field -- the
 *    hardware natively supports level-low/high and rising/falling/both
 *    edge, so none of ASIC3's both-edge emulation is needed. hwirq 16-25
 *    are fixed-function block interrupts (DMA/TICK/ALARM/ONEWIRE/SDHC/
 *    ADCTS/ADC/WDT/KEYPAD/NFIF) and reject .irq_set_type.
 *
 *  - GPIO: a new capability -- the 2007 driver never registered a
 *    gpio_chip. hwirq == gpio offset identity for offset < 16 lets the
 *    buttons and joypad be described as a stock gpio-keys node.
 *
 *  - Clocks: a common-clock-framework "hamcop-gclk" parent plus one gate
 *    clk_hw per CPM_ClockControl bit (bar PLLON, forced on unconditionally
 *    at probe rather than exposed as a gate -- nothing ever turns the PLL
 *    back off). Sub-devices are expected to clk_get() their own gate via
 *    the clkdev aliases registered below; current struct mfd_cell has no
 *    .enable/.disable to do this for them (that API is gone).
 *
 *  - Sub-devices: one devm_mfd_add_devices() call. Six function cells, each served by
 *    its own driver: "hamcop-nand" (drivers/mtd/nand/raw/hamcop_nand.c),
 *    "hamcop-sdi" (drivers/mmc/host/hamcop-sdi.c), "leds-hamcop"
 *    (drivers/leds/leds-hamcop.c), "ds1wm" (drivers/w1/masters/ds1wm.c,
 *    resurrected from mainline commit 652719b1003a, with a real
 *    platform_data below), "hamcop-adc" (drivers/iio/adc/hamcop-adc.c,
 *    a from-scratch IIO ADC driver) and "resistive-adc-touch" (mainline's
 *    existing generic ADC-touchscreen consumer, wired to hamcop-adc's
 *    channels through the io-channels property of its node -- no bespoke
 *    HAMCOP touchscreen driver needed). No
 *    cell exists for HAMCOP's internal KEYPAD matrix block (h2200 wires
 *    buttons to individual GPIO pins instead, handled by the gpio_chip
 *    above plus a gpio-keys node) or for the DMA block, which only
 *    hamcop-sdi uses and maps through its own resource.
 *
 * What deliberately is NOT here: any access to HAMCOP's 16 KiB SRAM window
 * or the bootloader-preservation dance around it. The SRAM is the DT node's
 * second "reg" entry; this driver never maps it, and only hands it on,
 * as-is, to a "hamcop-sram" sub-device, whose driver on h2200 is the
 * suspend and reboot code in arch/arm/mach-pxa/h2200_pm.c. This
 * driver's own register mapping starts HAMCOP_SRAM_SIZE bytes into the chip
 * for exactly that reason: two devices both trying to
 * devm_ioremap_resource() the same physical range would fail the second
 * one with -EBUSY.
 *
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/mfd/core.h>
#include <linux/mfd/ds1wm.h>
#include <linux/mfd/hamcop.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reboot.h>
#include <linux/spinlock.h>

/*
 * PllControl value "from wince via haret" (the old out-of-tree h2200.c) and
 * the CLKOUTCON value enabling only CLKOUT3M (3.6864 MHz) -- both genuine
 * h2200-board-specific tuning baked directly into chip init rather than
 * threaded through platform_data. HAMCOP has no other known board, so
 * there is no second caller pdata would ever serve; adding that
 * indirection back would just be unused ceremony.
 */
#define HAMCOP_PLLCONTROL_DEFAULT	0xd15e
#define HAMCOP_CLKOUTCON_3M6_ONLY	0x0054

/* Chained-handler storm guard: bound the "keep draining INTPND" outer
 * loop, matching the 2007 driver's own safety net (it capped at 1000).
 * Not itself a hot path, so the exact number isn't load-bearing.
 */
#define HAMCOP_MAX_IRQ_LOOPS		100

/*
 * A source pending in at least this many of a round's passes is what a storm
 * is made of. A bystander is serviced in a pass or two, and its consumer
 * cannot re-arm it while the handler runs with interrupts off, so it never
 * gets near this.
 */
#define HAMCOP_STORM_PERSIST_PASSES	(HAMCOP_MAX_IRQ_LOOPS / 4)

/*
 * A round that hits the bound masks the persistent sources and gets one more
 * round to drain the rest, see hamcop_irq_handler().
 */
#define HAMCOP_STORM_ROUNDS		2

/*
 * Bound on re-draining after a source asserts while the parent GPIO edge is
 * being re-armed. Each retry is a full enter/drain/exit cycle; a source that
 * keeps re-asserting in this window is a storm and belongs to the guard above.
 */
#define HAMCOP_MAX_IRQ_RECHECKS		3

enum {
	HAMCOP_GATE_KEYPAD,
	HAMCOP_GATE_DMA,
	HAMCOP_GATE_NAND,
	HAMCOP_GATE_LED,
	HAMCOP_GATE_RTC,
	HAMCOP_GATE_SD,
	HAMCOP_GATE_1WIRE,
	HAMCOP_GATE_ADC,
	HAMCOP_GATE_GPIO,
	HAMCOP_NR_CLK_GATES,
};

struct hamcop;

struct hamcop_gclk {
	struct clk_hw hw;
	struct hamcop *hamcop;
};

struct hamcop_clk_gate {
	struct clk_hw hw;
	struct hamcop *hamcop;
	u16 bit;
};

struct hamcop {
	struct device *dev;
	void __iomem *regs;
	int irq;

	/* Guards every register read-modify-write below (GPIO CON/DAT,
	 * ClockControl, IC INTMSK) and every write of storm_masked -- IC
	 * SRCPND/INTPND ack writes are write-1-to-clear and need no lock.
	 * raw_spinlock_t because the IRQ chip callbacks and the chained
	 * handler both take it.
	 */
	raw_spinlock_t lock;

	struct irq_domain *domain;
	struct gpio_chip gc;

	struct hamcop_gclk gclk;
	struct hamcop_clk_gate gates[HAMCOP_NR_CLK_GATES];

	/* Sources the storm backstop masked, INTMSK0's half in bits 0-15. The
	 * drain path reads it without the lock and filters it out of INTPND.
	 */
	u32 storm_masked;
};

static inline u16 hamcop_read(struct hamcop *hamcop, u32 reg)
{
	return readw_relaxed(hamcop->regs + reg);
}

static inline void hamcop_write(struct hamcop *hamcop, u32 reg, u16 val)
{
	writew_relaxed(val, hamcop->regs + reg);
}

static void hamcop_rmw(struct hamcop *hamcop, u32 reg, u16 mask, u16 bits)
{
	unsigned long flags;
	u16 val;

	raw_spin_lock_irqsave(&hamcop->lock, flags);
	val = hamcop_read(hamcop, reg);
	val = (val & ~mask) | bits;
	hamcop_write(hamcop, reg, val);
	raw_spin_unlock_irqrestore(&hamcop->lock, flags);
}

/* ---------------------------------------------------------------------
 * IRQ chip: flat 26-line pending/mask in the IC block. hwirq < 16 are
 * additionally routed through the GPIO block's own per-pin trigger-type
 * config (GPAINT0/1, GPBINT0/1); hwirq >= 16 are fixed edge lines with no
 * such config and reject .irq_set_type.
 * ---------------------------------------------------------------------
 */

static void hamcop_irq_ack(struct irq_data *d)
{
	struct hamcop *hamcop = irq_data_get_irq_chip_data(d);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	u32 srcpnd = hwirq < 16 ? HAMCOP_IC_SRCPND0 : HAMCOP_IC_SRCPND1;
	u32 intpnd = hwirq < 16 ? HAMCOP_IC_INTPND0 : HAMCOP_IC_INTPND1;
	u16 bit = BIT(hwirq & 15);

	/* Write-1-to-clear -- no read-modify-write, no lock needed. */
	hamcop_write(hamcop, srcpnd, bit);
	hamcop_write(hamcop, intpnd, bit);
}

static void hamcop_irq_mask(struct irq_data *d)
{
	struct hamcop *hamcop = irq_data_get_irq_chip_data(d);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	u32 reg = hwirq < 16 ? HAMCOP_IC_INTMSK0 : HAMCOP_IC_INTMSK1;
	u16 bit = BIT(hwirq & 15);

	hamcop_rmw(hamcop, reg, bit, bit);
}

/*
 * The storm bit goes first: a source unmasked in hardware while the drain
 * path still filters it out would leave INT asserted with nobody to service
 * it, and on an edge-only parent that is permanent.
 */
static void hamcop_irq_unmask(struct irq_data *d)
{
	struct hamcop *hamcop = irq_data_get_irq_chip_data(d);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	u32 reg = hwirq < 16 ? HAMCOP_IC_INTMSK0 : HAMCOP_IC_INTMSK1;
	u16 bit = BIT(hwirq & 15);
	unsigned long flags;

	raw_spin_lock_irqsave(&hamcop->lock, flags);
	WRITE_ONCE(hamcop->storm_masked, hamcop->storm_masked & ~BIT(hwirq));
	raw_spin_unlock_irqrestore(&hamcop->lock, flags);

	hamcop_rmw(hamcop, reg, bit, 0);
}

/* hwirq (0-15) -> (GPAINT/GPBINT register, 4-bit nibble within it). Pins
 * 0-3 -> GPAINT0, 4-7 -> GPAINT1, 8-11 (GPB0-3) -> GPBINT0,
 * 12-15 (GPB4-7) -> GPBINT1 -- see hamcop.h's GPIO block comment.
 */
static void hamcop_irq_gpio_reg(unsigned int hwirq, u32 *reg, unsigned int *nibble)
{
	static const u32 regs[4] = {
		HAMCOP_GPIO_GPAINT0, HAMCOP_GPIO_GPAINT1,
		HAMCOP_GPIO_GPBINT0, HAMCOP_GPIO_GPBINT1,
	};

	*reg = regs[hwirq / 4];
	*nibble = hwirq % 4;
}

static int hamcop_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct hamcop *hamcop = irq_data_get_irq_chip_data(d);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	unsigned int nibble;
	u32 reg;
	u16 intlv;

	if (hwirq >= HAMCOP_IRQ_GPIO_MAX)
		return -EINVAL;

	switch (type) {
	case IRQ_TYPE_LEVEL_LOW:
		intlv = HAMCOP_GPIO_INT_INTLV_LOW;
		irq_set_handler_locked(d, handle_level_irq);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		intlv = HAMCOP_GPIO_INT_INTLV_HIGH;
		irq_set_handler_locked(d, handle_level_irq);
		break;
	case IRQ_TYPE_EDGE_RISING:
		intlv = HAMCOP_GPIO_INT_INTLV_RISING;
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		intlv = HAMCOP_GPIO_INT_INTLV_FALLING;
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		intlv = HAMCOP_GPIO_INT_INTLV_BOTH;
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	default:
		return -EINVAL;
	}

	hamcop_irq_gpio_reg(hwirq, &reg, &nibble);

	/* INTEN is set here, permanently, alongside the trigger type: the
	 * GPIO block's per-pin detector is a prerequisite gate for these 16
	 * lines to ever reach the IC block at all, separate from the IC's
	 * own INTMSK mask/unmask above. Runtime enable/disable of a hwirq
	 * is entirely IC INTMSK's job; once a trigger type is programmed
	 * here it stays armed at the GPIO-block level. This two-tier split
	 * is not documented -- it's inferred from the register layout and
	 * is worth double-checking against real hardware.
	 */
	hamcop_rmw(hamcop, reg, HAMCOP_GPIO_INT_MASK(nibble),
		   (intlv | HAMCOP_GPIO_INT_INTEN) << (4 * nibble));

	return 0;
}

/*
 * HAMCOP has no wake-configuration register of its own: an enabled line
 * that fires in sleep simply pulls ASIC_INT_N, so arming a line for wakeup
 * reduces to arming the chained parent interrupt. irq_set_irq_wake()
 * refcounts on the parent, so several children arming and disarming
 * independently balance out.
 *
 * Without this callback, enable_irq_wake() on any HAMCOP line fails with
 * -ENXIO, which gpio-keys reports as "failed to configure IRQ as wakeup
 * source" for a wakeup-source key.
 *
 * On a devicetree boot the parent's pchip->set_wake backend is the
 * pxa_gpio_dt_set_wake hook that the SoC code points at gpio_set_wake(), so
 * the refcount propagated here reaches PWER/PFER. The edge that gets armed
 * is the static WAKEUP_ON_EDGE_* setting for the parent pin in the board's
 * pin configuration table, not this hwirq's runtime trigger type.
 */
static int hamcop_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct hamcop *hamcop = irq_data_get_irq_chip_data(d);

	return irq_set_irq_wake(hamcop->irq, on);
}

static struct irq_chip hamcop_irq_chip = {
	.name		= "hamcop",
	.irq_ack	= hamcop_irq_ack,
	.irq_mask	= hamcop_irq_mask,
	.irq_unmask	= hamcop_irq_unmask,
	.irq_set_type	= hamcop_irq_set_type,
	.irq_set_wake	= hamcop_irq_set_wake,
};

static int hamcop_irq_domain_map(struct irq_domain *d, unsigned int virq,
				  irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(virq, &hamcop_irq_chip, handle_edge_irq);
	irq_set_chip_data(virq, d->host_data);
	irq_set_probe(virq);

	return 0;
}

static void hamcop_irq_domain_unmap(struct irq_domain *d, unsigned int virq)
{
	irq_set_chip_and_handler(virq, NULL, NULL);
	irq_set_chip_data(virq, NULL);
}

static const struct irq_domain_ops hamcop_irq_domain_ops = {
	.map	= hamcop_irq_domain_map,
	.unmap	= hamcop_irq_domain_unmap,
	.xlate	= irq_domain_xlate_onetwocell,
};

/*
 * INTPND0 in bits 0-15, INTPND1 in bits 16-31, without the sources the storm
 * backstop has masked, so that one of them stays out of the drain even if its
 * INTMSK bit is lost again before the source is re-armed.
 */
static u32 hamcop_irq_pending(struct hamcop *hamcop)
{
	u32 pending;

	pending = hamcop_read(hamcop, HAMCOP_IC_INTPND0);
	pending |= (u32)hamcop_read(hamcop, HAMCOP_IC_INTPND1) << 16;

	return pending & ~READ_ONCE(hamcop->storm_masked);
}

/*
 * Mask, then ack. ASIC_INT_N must be back high before chained_irq_exit()
 * re-enables edge detection on the parent, and it only goes high once the
 * source is masked and its pending bits are gone; acking first would let the
 * source set them again in between. INTMSK and storm_masked change together
 * under the lock, hardware first, so a reader that sees the bit set has a
 * source that is masked.
 */
static void hamcop_irq_storm_mask(struct hamcop *hamcop, u32 bits)
{
	unsigned long flags;
	unsigned int half;

	for (half = 0; half < 2; half++) {
		u32 intmsk = half ? HAMCOP_IC_INTMSK1 : HAMCOP_IC_INTMSK0;
		u32 srcpnd = half ? HAMCOP_IC_SRCPND1 : HAMCOP_IC_SRCPND0;
		u32 intpnd = half ? HAMCOP_IC_INTPND1 : HAMCOP_IC_INTPND0;
		u16 half_bits = bits >> (16 * half);

		if (!half_bits)
			continue;

		raw_spin_lock_irqsave(&hamcop->lock, flags);
		hamcop_write(hamcop, intmsk, hamcop_read(hamcop, intmsk) | half_bits);
		WRITE_ONCE(hamcop->storm_masked,
			   hamcop->storm_masked | (u32)half_bits << (16 * half));
		raw_spin_unlock_irqrestore(&hamcop->lock, flags);

		hamcop_write(hamcop, srcpnd, half_bits);
		hamcop_write(hamcop, intpnd, half_bits);
	}
}

/*
 * A round of HAMCOP_MAX_IRQ_LOOPS passes never saw INTPND empty. cnt[] is how
 * many of those passes each source was pending in, seen is every source that
 * was pending in any of them.
 *
 * The persistent sources are the storm: a bystander serviced in the same
 * passes scores one or two, and sources that alternate pass by pass each score
 * half the bound although no pass ever had both, which an intersection of the
 * per-pass sets would miss. Masking them alone leaves the rest of the chip
 * running -- a blanket mask took the 1-wire bus down with an SD card storm.
 * When nothing persists, or the round after the first mask still does not
 * drain, everything seen is masked so that the handler cannot spin forever.
 *
 * Returns true when the caller should run another round.
 */
static bool hamcop_irq_storm(struct hamcop *hamcop, const u16 *cnt, u32 seen,
			     unsigned int dispatched, bool last_round)
{
	u16 min_masked = U16_MAX, max_spared = 0;
	u32 persistent = 0, masked, spared;
	unsigned int hwirq;
	bool fallback;

	for (hwirq = 0; hwirq < 32; hwirq++)
		if (cnt[hwirq] >= HAMCOP_STORM_PERSIST_PASSES)
			persistent |= BIT(hwirq);

	fallback = last_round || !persistent;
	masked = fallback ? seen : persistent;
	spared = seen & ~masked;

	for (hwirq = 0; hwirq < 32; hwirq++) {
		if (masked & BIT(hwirq))
			min_masked = min(min_masked, cnt[hwirq]);
		else if (spared & BIT(hwirq))
			max_spared = max(max_spared, cnt[hwirq]);
	}

	dev_err_ratelimited(hamcop->dev,
			    "interrupt storm after %u passes (dispatched %u): masking %04x/%04x, sparing %04x/%04x; counts masked>=%u spared<=%u%s\n",
			    HAMCOP_MAX_IRQ_LOOPS, dispatched,
			    masked & 0xffff, masked >> 16,
			    spared & 0xffff, spared >> 16,
			    min_masked, max_spared,
			    fallback ? " [fallback]" : "");

	hamcop_irq_storm_mask(hamcop, masked);

	return !fallback;
}

static void hamcop_irq_handler(struct irq_desc *desc)
{
	struct hamcop *hamcop = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int recheck;
	u32 pending;

	for (recheck = 0; recheck < HAMCOP_MAX_IRQ_RECHECKS; recheck++) {
		unsigned int round;

		chained_irq_enter(chip, desc);

		for (round = 0; round < HAMCOP_STORM_ROUNDS; round++) {
			unsigned int dispatched = 0;
			unsigned int loops;
			u16 cnt[32] = {};
			u32 seen = 0;

			for (loops = 0; loops < HAMCOP_MAX_IRQ_LOOPS; loops++) {
				pending = hamcop_irq_pending(hamcop);
				if (!pending)
					break;

				seen |= pending;

				/* Service highest hwirq first, matching the priority order
				 * the 2007 driver's demux used (31 - clz == fls - 1).
				 */
				while (pending) {
					unsigned int hwirq = fls(pending) - 1;

					cnt[hwirq]++;
					dispatched++;
					generic_handle_domain_irq(hamcop->domain, hwirq);
					pending &= ~BIT(hwirq);
				}
			}

			if (likely(loops < HAMCOP_MAX_IRQ_LOOPS))
				break;

			if (!hamcop_irq_storm(hamcop, cnt, seen, dispatched,
					      round == HAMCOP_STORM_ROUNDS - 1))
				break;
		}

		chained_irq_exit(chip, desc);

		/*
		 * chained_irq_exit() re-enables edge detection on the parent
		 * GPIO, which chained_irq_enter() disabled. A source asserting
		 * after the last INTPND == 0 read but before that re-enable
		 * pulls INT low without a latched edge, and nothing would
		 * trigger this handler again. Look once more now that edge
		 * detection is back and drain again if something is pending.
		 */
		if (!hamcop_irq_pending(hamcop))
			return;
	}

	/*
	 * A source that was pending again after every recheck has no latched
	 * edge to wake this handler with, and holds INT low, so every other
	 * source goes silent with it. Treat it as a storm and mask it.
	 */
	pending = hamcop_irq_pending(hamcop);
	if (pending) {
		dev_err_ratelimited(hamcop->dev,
				    "interrupt source %04x/%04x still pending after %u rechecks: masking it\n",
				    pending & 0xffff, pending >> 16,
				    HAMCOP_MAX_IRQ_RECHECKS);
		hamcop_irq_storm_mask(hamcop, pending);
	}
}

static void hamcop_irq_teardown(void *data)
{
	struct hamcop *hamcop = data;

	irq_set_chained_handler_and_data(hamcop->irq, NULL, NULL);
}

static int hamcop_irq_init(struct hamcop *hamcop)
{
	const struct irq_domain_info info = {
		.fwnode		= dev_fwnode(hamcop->dev),
		.size		= HAMCOP_NR_IRQS,
		.hwirq_max	= HAMCOP_NR_IRQS,
		.ops		= &hamcop_irq_domain_ops,
		.host_data	= hamcop,
	};

	/* Mask everything and ack whatever the bootloader left pending
	 * before the chained handler can ever fire.
	 */
	hamcop_write(hamcop, HAMCOP_IC_INTMSK0, 0xffff);
	hamcop_write(hamcop, HAMCOP_IC_INTMSK1, 0xffff);
	hamcop_write(hamcop, HAMCOP_IC_SRCPND0, 0xffff);
	hamcop_write(hamcop, HAMCOP_IC_SRCPND1, 0xffff);
	hamcop_write(hamcop, HAMCOP_IC_INTPND0, 0xffff);
	hamcop_write(hamcop, HAMCOP_IC_INTPND1, 0xffff);

	hamcop->domain = devm_irq_domain_instantiate(hamcop->dev, &info);
	if (IS_ERR(hamcop->domain))
		return PTR_ERR(hamcop->domain);

	/* ASIC_INT_N is active low -- the old out-of-tree h2200.c programmed
	 * IRQT_FALLING on this same PXA GPIO line.
	 */
	irq_set_irq_type(hamcop->irq, IRQ_TYPE_EDGE_FALLING);
	irq_set_chained_handler_and_data(hamcop->irq, hamcop_irq_handler, hamcop);

	/*
	 * devm knows nothing about the chained handler on a GPIO interrupt,
	 * so without this action every probe failure after this point would
	 * return with the handler still installed and pointing at a struct
	 * hamcop that devm frees. This is registered after the register
	 * mapping and the irq_domain that the handler dereferences, so the
	 * LIFO unwind removes the handler before they go away.
	 */
	return devm_add_action_or_reset(hamcop->dev, hamcop_irq_teardown, hamcop);
}

/* ---------------------------------------------------------------------
 * GPIO chip: banks A-D, 8 pins each. Only A/B (offsets 0-15) are also
 * hwirqs in the domain above -- that identity is what lets the buttons
 * be a stock gpio-keys node.
 * ---------------------------------------------------------------------
 */

static void hamcop_gpio_bank_pin(unsigned int offset, unsigned int *bank,
				  unsigned int *pin)
{
	*bank = offset / HAMCOP_GPIO_PINS_PER_BANK;
	*pin = offset % HAMCOP_GPIO_PINS_PER_BANK;
}

static int hamcop_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct hamcop *hamcop = gpiochip_get_data(gc);
	unsigned int bank, pin;
	u16 con;

	hamcop_gpio_bank_pin(offset, &bank, &pin);
	con = hamcop_read(hamcop, HAMCOP_GPIO_BANK_CON(bank));

	return (con & HAMCOP_GPIO_CON_OUTPUT(pin)) ?
		GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN;
}

static int hamcop_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	struct hamcop *hamcop = gpiochip_get_data(gc);
	unsigned int bank, pin;

	hamcop_gpio_bank_pin(offset, &bank, &pin);
	hamcop_rmw(hamcop, HAMCOP_GPIO_BANK_CON(bank), HAMCOP_GPIO_CON_MASK(pin), 0);

	return 0;
}

static int hamcop_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct hamcop *hamcop = gpiochip_get_data(gc);
	unsigned int bank, pin;

	hamcop_gpio_bank_pin(offset, &bank, &pin);

	return !!(hamcop_read(hamcop, HAMCOP_GPIO_BANK_DAT(bank)) & BIT(pin));
}

static int hamcop_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct hamcop *hamcop = gpiochip_get_data(gc);
	unsigned int bank, pin;

	hamcop_gpio_bank_pin(offset, &bank, &pin);
	hamcop_rmw(hamcop, HAMCOP_GPIO_BANK_DAT(bank), BIT(pin), value ? BIT(pin) : 0);

	return 0;
}

static int hamcop_gpio_direction_output(struct gpio_chip *gc, unsigned int offset,
					 int value)
{
	struct hamcop *hamcop = gpiochip_get_data(gc);
	unsigned int bank, pin;

	/* Set the level before switching direction, to avoid a glitch. */
	hamcop_gpio_set(gc, offset, value);

	hamcop_gpio_bank_pin(offset, &bank, &pin);
	hamcop_rmw(hamcop, HAMCOP_GPIO_BANK_CON(bank), HAMCOP_GPIO_CON_MASK(pin),
		   HAMCOP_GPIO_CON_OUTPUT(pin));

	return 0;
}

static int hamcop_gpio_to_irq(struct gpio_chip *gc, unsigned int offset)
{
	struct hamcop *hamcop = gpiochip_get_data(gc);

	if (offset >= HAMCOP_IRQ_GPIO_MAX)
		return -ENXIO;

	/*
	 * irq_create_mapping(), not irq_find_mapping(): nothing else in
	 * this driver creates a domain mapping for the GPIO-backed hwirqs
	 * ahead of time, so find_mapping() would return 0 (gpiod_to_irq()
	 * treats that as "no IRQ", -ENXIO) for any GPIO whose interrupt
	 * hasn't already been mapped some other way -- defeating the whole
	 * point of gpio-keys or any other consumer calling gpiod_to_irq()
	 * on a HAMCOP button/line for the first time.
	 */
	return irq_create_mapping(hamcop->domain, offset);
}

static int hamcop_gpio_register(struct hamcop *hamcop)
{
	hamcop->gc.label = "hamcop";
	hamcop->gc.parent = hamcop->dev;
	hamcop->gc.owner = THIS_MODULE;
	hamcop->gc.get_direction = hamcop_gpio_get_direction;
	hamcop->gc.direction_input = hamcop_gpio_direction_input;
	hamcop->gc.direction_output = hamcop_gpio_direction_output;
	hamcop->gc.get = hamcop_gpio_get;
	hamcop->gc.set = hamcop_gpio_set;
	hamcop->gc.to_irq = hamcop_gpio_to_irq;
	hamcop->gc.base = -1;
	hamcop->gc.ngpio = HAMCOP_NR_GPIO;
	hamcop->gc.can_sleep = false;

	return devm_gpiochip_add_data(hamcop->dev, &hamcop->gc, hamcop);
}

/* ---------------------------------------------------------------------
 * Clocks (CCF): one "hamcop-gclk" parent, computed from the PXA25x
 * "memory" bus clock (clk-pxa25x.c registers it under exactly that global
 * name -- see clk_pxa25x_memory_get_rate() and its PXA25X_CKEN_1RATE()
 * consumers in that file) divided by 1 or 2 depending on ClockSleep's
 * CLKSEL bit, plus nine gate clk_hws on CPM_ClockControl bits. Custom
 * clk_ops rather than the generic clk_gate helper: HAMCOP's registers are
 * 16-bit and clk_gate assumes 32-bit.
 * ---------------------------------------------------------------------
 */

static unsigned long hamcop_gclk_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct hamcop_gclk *gclk = container_of(hw, struct hamcop_gclk, hw);
	u16 sleep = hamcop_read(gclk->hamcop, HAMCOP_CPM_CLOCKSLEEP);

	return (sleep & HAMCOP_CLKSLEEP_CLKSEL) ? parent_rate : parent_rate / 2;
}

static const struct clk_ops hamcop_gclk_ops = {
	.recalc_rate = hamcop_gclk_recalc_rate,
};

static int hamcop_clk_gate_enable(struct clk_hw *hw)
{
	struct hamcop_clk_gate *gate = container_of(hw, struct hamcop_clk_gate, hw);

	hamcop_rmw(gate->hamcop, HAMCOP_CPM_CLOCKCONTROL, gate->bit, gate->bit);

	return 0;
}

static void hamcop_clk_gate_disable(struct clk_hw *hw)
{
	struct hamcop_clk_gate *gate = container_of(hw, struct hamcop_clk_gate, hw);

	hamcop_rmw(gate->hamcop, HAMCOP_CPM_CLOCKCONTROL, gate->bit, 0);
}

static int hamcop_clk_gate_is_enabled(struct clk_hw *hw)
{
	struct hamcop_clk_gate *gate = container_of(hw, struct hamcop_clk_gate, hw);

	return !!(hamcop_read(gate->hamcop, HAMCOP_CPM_CLOCKCONTROL) & gate->bit);
}

static const struct clk_ops hamcop_clk_gate_ops = {
	.enable		= hamcop_clk_gate_enable,
	.disable	= hamcop_clk_gate_disable,
	.is_enabled	= hamcop_clk_gate_is_enabled,
};

static const struct {
	const char *name;
	u16 bit;
} hamcop_clk_defs[HAMCOP_NR_CLK_GATES] = {
	[HAMCOP_GATE_KEYPAD]	= { "hamcop-keypad",	HAMCOP_CLK_KEYPAD },
	[HAMCOP_GATE_DMA]	= { "hamcop-dma",	HAMCOP_CLK_DMAC },
	[HAMCOP_GATE_NAND]	= { "hamcop-nand-clk",	HAMCOP_CLK_NAND },
	[HAMCOP_GATE_LED]	= { "hamcop-led",	HAMCOP_CLK_LED },
	[HAMCOP_GATE_RTC]	= { "hamcop-rtc",	HAMCOP_CLK_RTC },
	[HAMCOP_GATE_SD]	= { "hamcop-sd",	HAMCOP_CLK_SD },
	[HAMCOP_GATE_1WIRE]	= { "hamcop-1wire",	HAMCOP_CLK_1WIRE },
	[HAMCOP_GATE_ADC]	= { "hamcop-adc-clk",	HAMCOP_CLK_ADC },
	[HAMCOP_GATE_GPIO]	= { "hamcop-gpio",	HAMCOP_CLK_GPIO },
};

static const char * const hamcop_gclk_parents[] = { "memory" };
static const char * const hamcop_gate_parents[] = { "hamcop-gclk" };

static int hamcop_clocks_register(struct hamcop *hamcop)
{
	struct clk_init_data gclk_init = {
		.name		= "hamcop-gclk",
		.ops		= &hamcop_gclk_ops,
		.parent_names	= hamcop_gclk_parents,
		.num_parents	= ARRAY_SIZE(hamcop_gclk_parents),
	};
	int i, ret;

	hamcop->gclk.hamcop = hamcop;
	hamcop->gclk.hw.init = &gclk_init;
	ret = devm_clk_hw_register(hamcop->dev, &hamcop->gclk.hw);
	if (ret)
		return ret;

	for (i = 0; i < HAMCOP_NR_CLK_GATES; i++) {
		struct clk_init_data gate_init = {
			.name		= hamcop_clk_defs[i].name,
			.ops		= &hamcop_clk_gate_ops,
			.parent_names	= hamcop_gate_parents,
			.num_parents	= ARRAY_SIZE(hamcop_gate_parents),
			/*
			 * probe() force-enables GPIO and NAND directly via
			 * HAMCOP_CPM_CLOCKCONTROL, outside the clk framework
			 * entirely -- but that doesn't give either gate a
			 * nonzero enable_count, so without CLK_IS_CRITICAL
			 * the late_initcall_sync clk_disable_unused() pass
			 * sees "enabled, unreferenced" and turns both back
			 * off shortly after boot. GPIO gates this driver's
			 * own gpio_chip/IRQ demux; NAND gates the SRAM block
			 * h2200_pm.c's bootloader-preservation logic depends
			 * on (see this file's header comment) -- neither has
			 * a real clk_get() consumer to hold a reference
			 * instead, so mark them critical rather than
			 * invent one.
			 */
			.flags		= (i == HAMCOP_GATE_GPIO || i == HAMCOP_GATE_NAND) ?
					  CLK_IS_CRITICAL : 0,
		};

		hamcop->gates[i].hamcop = hamcop;
		hamcop->gates[i].bit = hamcop_clk_defs[i].bit;
		hamcop->gates[i].hw.init = &gate_init;

		ret = devm_clk_hw_register(hamcop->dev, &hamcop->gates[i].hw);
		if (ret)
			return ret;
	}

	/*
	 * clkdev aliases for the sub-device drivers to clk_get() against --
	 * chip-internal wiring, not board policy, so it belongs here rather
	 * than in the board code. "hamcop-gpio" has no alias: GPIO is this
	 * driver's own internal dependency (see probe()). The DMA gate
	 * gets its own alias further down.
	 */
	ret = devm_clk_hw_register_clkdev(hamcop->dev, &hamcop->gates[HAMCOP_GATE_NAND].hw,
					   NULL, "hamcop-nand");
	if (ret)
		return ret;

	ret = devm_clk_hw_register_clkdev(hamcop->dev, &hamcop->gates[HAMCOP_GATE_SD].hw,
					   NULL, "hamcop-sdi");
	if (ret)
		return ret;

	ret = devm_clk_hw_register_clkdev(hamcop->dev, &hamcop->gates[HAMCOP_GATE_LED].hw,
					   NULL, "leds-hamcop");
	if (ret)
		return ret;

	/* LED blink timing is clocked from HAMCOP's internal RTC block
	 * (HAMCOP_LED_LEDNCON0_CLK_RTC in the 2007 driver) -- leds-hamcop
	 * needs both gates.
	 */
	ret = devm_clk_hw_register_clkdev(hamcop->dev, &hamcop->gates[HAMCOP_GATE_RTC].hw,
					   "rtc", "leds-hamcop");
	if (ret)
		return ret;

	ret = devm_clk_hw_register_clkdev(hamcop->dev, &hamcop->gates[HAMCOP_GATE_1WIRE].hw,
					   NULL, "ds1wm");
	if (ret)
		return ret;

	/*
	 * Only "hamcop-adc" (drivers/iio/adc/hamcop-adc.c) gets an alias:
	 * mainline's generic resistive-adc-touch consumer cell has no
	 * chip-specific code at all -- it never calls clk_get() itself, the
	 * IIO producer manages the shared ADC block's clock on its behalf.
	 */
	return devm_clk_hw_register_clkdev(hamcop->dev, &hamcop->gates[HAMCOP_GATE_ADC].hw,
					    NULL, "hamcop-adc");
}

/* ---------------------------------------------------------------------
 * Sub-devices. Six function cells, each served by its own driver -- see each
 * cell's comment below for which one and its reference IP. The SRAM
 * sub-device is added separately, ahead of them.
 * ---------------------------------------------------------------------
 */

/*
 * bus_shift=2: HAMCOP's OWM register file is 4-byte-strided (see
 * HAMCOP_OWM_BASE/SIZE above). active_high=0 leaves the interrupt
 * polarity (IAS) at DS1WM's power-on-reset default, active low.
 * HAMCOP's fixed-function hwirq lines (16-25,
 * ONEWIRE included) don't expose a trigger-type register for this driver
 * to double-check against -- flagged as hardware-unverified.
 */
static const struct ds1wm_driver_data hamcop_ds1wm_pdata = {
	.bus_shift		= 2,
	.active_high		= 0,
	.reset_recover_delay	= 1,
};

static const struct resource hamcop_nand_resources[] = {
	DEFINE_RES_MEM(HAMCOP_NF_BASE, HAMCOP_NF_SIZE),
};

static const struct resource hamcop_sdi_resources[] = {
	DEFINE_RES_MEM(HAMCOP_SDI_BASE, HAMCOP_SDI_SIZE),
	DEFINE_RES_IRQ(HAMCOP_IRQ_SDHC),
	DEFINE_RES_IRQ(HAMCOP_IRQ_DMA),
};

static const struct resource hamcop_led_resources[] = {
	DEFINE_RES_MEM(HAMCOP_LED_BASE, HAMCOP_LED_SIZE),
};

static const struct resource hamcop_adc_resources[] = {
	DEFINE_RES_MEM(HAMCOP_ADC_BASE, HAMCOP_ADC_SIZE),
	DEFINE_RES_IRQ(HAMCOP_IRQ_ADC),
	DEFINE_RES_IRQ(HAMCOP_IRQ_ADCTS),
};

static const struct resource hamcop_ds1wm_resources[] = {
	DEFINE_RES_MEM(HAMCOP_OWM_BASE, HAMCOP_OWM_SIZE),
	DEFINE_RES_IRQ(HAMCOP_IRQ_ONEWIRE),
};

/* Relative to the SRAM window itself, not to the register window. */
static const struct resource hamcop_sram_resources[] = {
	DEFINE_RES_MEM(0, HAMCOP_SRAM_SIZE),
};

static const struct mfd_cell hamcop_sram_cell = {
	.name		= "hamcop-sram",
	.id		= PLATFORM_DEVID_NONE,
	.resources	= hamcop_sram_resources,
	.num_resources	= ARRAY_SIZE(hamcop_sram_resources),
};

static const struct mfd_cell hamcop_cells[] = {
	{
		/*
		 * drivers/mtd/nand/raw/hamcop_nand.c, against the modern MTD
		 * ->exec_op() interface; IP is S3C24xx-class, ported with
		 * reference to the deleted mainline s3c2410.c. No IRQ
		 * resource: RnB interrupt handling was never written even in
		 * 2007 ("irq = -1, XXX until RnB interrupt code is written").
		 */
		.name		= "hamcop-nand",
		.id		= PLATFORM_DEVID_NONE,
		.of_compatible	= "samsung,hamcop-nand",
		.resources	= hamcop_nand_resources,
		.num_resources	= ARRAY_SIZE(hamcop_nand_resources),
	},
	{
		/*
		 * drivers/mmc/host/hamcop-sdi.c, PIO with an SRAM-bounced DMA
		 * path. IP is S3C24xx SDI, ported with reference to the
		 * deleted s3cmci.c. Card-detect (PXA GPIO8) and slot-power
		 * (PXA GPIO61) are h2200 board wiring, not HAMCOP registers,
		 * so the board describes them on the devicetree node of this
		 * cell. The second IRQ is the DMA engine's completion
		 * line, distinct from the SDI's own; the driver requests
		 * it only when the SRAM bounce pool is actually available.
		 */
		.name		= "hamcop-sdi",
		.id		= PLATFORM_DEVID_NONE,
		.of_compatible	= "samsung,hamcop-sdi",
		.resources	= hamcop_sdi_resources,
		.num_resources	= ARRAY_SIZE(hamcop_sdi_resources),
	},
	{
		/*
		 * drivers/leds/leds-hamcop.c. Five channels; the wired ones
		 * are child nodes of the cell's devicetree node.
		 */
		.name		= "leds-hamcop",
		.id		= PLATFORM_DEVID_NONE,
		.of_compatible	= "samsung,hamcop-leds",
		.resources	= hamcop_led_resources,
		.num_resources	= ARRAY_SIZE(hamcop_led_resources),
	},
	{
		/*
		 * drivers/iio/adc/hamcop-adc.c: IIO front-end for the shared
		 * ADC/touchscreen block, owns both its hwirqs (see
		 * hamcop_adc_resources above).
		 */
		.name		= "hamcop-adc",
		.id		= PLATFORM_DEVID_NONE,
		.of_compatible	= "samsung,hamcop-adc",
		.resources	= hamcop_adc_resources,
		.num_resources	= ARRAY_SIZE(hamcop_adc_resources),
	},
	{
		/*
		 * mainline's generic resistive-adc-touch driver, consuming
		 * the four touch channels of the "hamcop-adc" cell through
		 * the io-channels property of its devicetree node. That goes
		 * through the IIO fwnode lookup, which defers the probe until
		 * the ADC driver has registered, whatever the module load
		 * order. The cell has no register access or IRQ of its own.
		 */
		.name		= "resistive-adc-touch",
		.id		= PLATFORM_DEVID_NONE,
		.of_compatible	= "resistive-adc-touch",
	},
	{
		/*
		 * DS1WM-compatible 1-Wire master -- the entire battery
		 * monitoring path. Its consumer, ds2760_battery.c
		 * (drivers/power/supply), attaches automatically once
		 * ds1wm.c registers a w1 master; nothing else to wire up.
		 */
		.name		= "ds1wm",
		.id		= PLATFORM_DEVID_NONE,
		.of_compatible	= "samsung,hamcop-onewire",
		.platform_data	= &hamcop_ds1wm_pdata,
		.pdata_size	= sizeof(hamcop_ds1wm_pdata),
		.resources	= hamcop_ds1wm_resources,
		.num_resources	= ARRAY_SIZE(hamcop_ds1wm_resources),
	},
};

/* ---------------------------------------------------------------------
 * Restart.
 *
 * IFMODE only returns to SRAM boot-read mode on power-on, on a Cold or Warm
 * Reset command issued to HAMCOP, or on PWREN deassertion; restoring it
 * after any of those is software's job and there is no other path back. A
 * plain kernel_restart() triggers none of the three, so IFMODE is left
 * wherever software last set it. The CPU's first fetch after reset, at
 * physical address 0 (HAMCOP sits on PXA CS0), would then be serviced as a
 * VLIO/RDY-gated cycle instead of the fixed-latency read the boot protocol
 * expects, and the machine hangs on the reboot. The bootloader image in the
 * SRAM window is restored by h2200_pm.c, which has no access to this chip's
 * CIF registers because the two are disjoint resources on separate platform
 * devices. A Warm Reset command (0xfc to BOOTMODE) would reach the same
 * state but needs a WarmBootStatus poll before any further access, for no
 * benefit over a direct register write.
 */
static int hamcop_restart_prepare(struct sys_off_data *data)
{
	struct hamcop *hamcop = data->cb_data;

	hamcop_write(hamcop, HAMCOP_CIF_IFMODE, HAMCOP_CIF_IFMODE_SRAM);

	return NOTIFY_DONE;
}

/* ---------------------------------------------------------------------
 * Probe/lifecycle
 * ---------------------------------------------------------------------
 */

static int hamcop_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hamcop *hamcop;
	struct resource *mem, *sram;
	int irq, ret;

	hamcop = devm_kzalloc(dev, sizeof(*hamcop), GFP_KERNEL);
	if (!hamcop)
		return -ENOMEM;

	hamcop->dev = dev;
	raw_spin_lock_init(&hamcop->lock);
	platform_set_drvdata(pdev, hamcop);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -EINVAL;

	hamcop->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(hamcop->regs))
		return PTR_ERR(hamcop->regs);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	hamcop->irq = irq;

	/*
	 * Bring the chip out of its post-bootloader state: normal VLIO bus
	 * mode (boot-time SRAM-read mode is meaningless once Linux is
	 * running), the PLL forced on (nothing ever turns it back off), the
	 * NAND clock forced on (it also powers HAMCOP's SRAM block that
	 * h2200_pm.c depends on -- see this file's header comment; NAND and
	 * SDI must not race each other's clk_disable() of it, which is
	 * exactly what registering it as a normal CCF gate below, always
	 * requested-but-never-fully-released by this probe, prevents), and
	 * the GPIO clock forced on (our own gpio_chip needs it immediately,
	 * before any sub-device probes). See HAMCOP_PLLCONTROL_DEFAULT's
	 * definition above for why ClockSleep/PllControl are hardcoded
	 * rather than pdata.
	 */
	hamcop_write(hamcop, HAMCOP_CIF_IFMODE, HAMCOP_CIF_IFMODE_VLIO);

	/*
	 * WTCON's reset value is 0x8021 (watchdog ch.9 p.9-3): both the
	 * timer-enable bit[5] and the reset-output-enable bit[0] are set out
	 * of reset, and WTCNT already holds its own nonzero reset default
	 * (0x8000, ch.9 p.9-4) rather than needing a software-written initial
	 * count -- so the timer is live and counting down from HAMCOP's own
	 * power-on reset with no software action at all. Its GCLK feed is one
	 * of only two blocks (with the interrupt controller) wired as
	 * unconditional "Clock enable" rather than a CLKCON-gated
	 * "enable/disable" line in the power-management clock-distribution
	 * diagram (ch.14 p.14-7), so nothing in CLKCON stops it either. A
	 * timeout asserts nRSTOUT for 128 GCLK cycles (ch.9 p.9-1); Cotulla
	 * interface ch.2 p.2-11's reset-cause table lists the watchdog as a
	 * H/W trigger of Cold Reset, and nRSTOUT is HAMCOP's own reset-control
	 * output to the Cotulla/PXA CPU (product overview ch.1 p.1-4/1-8, ball
	 * D4) -- i.e. an un-petted, un-disabled watchdog left at its reset
	 * defaults resets the CPU, on the order of single-digit-to-tens of
	 * seconds after HAMCOP's own power-on reset depending on GCLK's
	 * source (MCLK or 1/4 X-tal, CLKSLEEP.CLKSEL). Disabled here,
	 * unconditionally and first, before anything else that could stall
	 * probe long enough to matter.
	 */
	hamcop_write(hamcop, HAMCOP_WT_WTCON, 0x0000);

	hamcop_write(hamcop, HAMCOP_CPM_CLOCKCONTROL,
		     HAMCOP_CLK_PLLON | HAMCOP_CLK_NAND | HAMCOP_CLK_GPIO);
	hamcop_write(hamcop, HAMCOP_CPM_CLOCKSLEEP,
		     HAMCOP_CLKSLEEP_XTCON | HAMCOP_CLKSLEEP_UCLK_ON |
		     HAMCOP_CLKSLEEP_CLKSEL);
	hamcop_write(hamcop, HAMCOP_CPM_PLLCONTROL, HAMCOP_PLLCONTROL_DEFAULT);
	hamcop_write(hamcop, HAMCOP_GPIO_CLKOUTCON, HAMCOP_CLKOUTCON_3M6_ONLY);

	ret = hamcop_irq_init(hamcop);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set up IRQ domain\n");

	ret = hamcop_clocks_register(hamcop);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register clocks\n");

	ret = hamcop_gpio_register(hamcop);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register gpio_chip\n");

	/*
	 * The SRAM sub-device goes first: hamcop-sdi claims its DMA bounce
	 * buffer from the pool the SRAM's driver publishes exactly once, at
	 * its own probe, and falls back to PIO for good if the pool is not
	 * there yet. Its window is a separate resource below the register
	 * window, so it cannot share the mem_base the other cells are
	 * offset from.
	 */
	sram = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (sram) {
		ret = devm_mfd_add_devices(dev, 0, &hamcop_sram_cell, 1, sram,
					   0, NULL);
		if (ret)
			return dev_err_probe(dev, ret, "failed to add SRAM sub-device\n");
	}

	/*
	 * Top-level id=0 here, PLATFORM_DEVID_NONE on every cell above:
	 * mfd_add_device() computes each child's platform_device id as
	 * id + cell->id, so this combination is what actually yields
	 * PLATFORM_DEVID_NONE (unnumbered "hamcop-nand" instead of
	 * "hamcop-nand.0") for each singleton sub-device -- matching
	 * drivers/mfd/ioc3.c's ioc3_led_setup()/ioc3_m48t35_setup() use of
	 * the same id=0 convention for their own singleton cells.
	 *
	 * Only added when the device has an of_node: five of the cells carry
	 * .of_compatible, which mfd_add_device() can only match against a
	 * child of that node.
	 */
	if (dev->of_node) {
		ret = devm_mfd_add_devices(dev, 0, hamcop_cells, ARRAY_SIZE(hamcop_cells),
					   mem, 0, hamcop->domain);
		if (ret)
			return dev_err_probe(dev, ret, "failed to add sub-devices\n");
	}

	ret = devm_register_sys_off_handler(dev, SYS_OFF_MODE_RESTART_PREPARE,
					     SYS_OFF_PRIO_DEFAULT,
					     hamcop_restart_prepare, hamcop);
	if (ret)
		dev_warn(dev, "failed to register restart handler: %d\n", ret);

	dev_info(dev, "HAMCOP companion ASIC ready, chained on IRQ %d\n", irq);

	return 0;
}

/*
 * Public API for sub-cell drivers: set/clear bits in the shared SPUCR
 * pull-up control register (see include/linux/mfd/hamcop.h for what the
 * bits mean and why this isn't just a second ioremap in the caller).
 * Same signature and semantics as hamcop_set_spucr() of the old
 * out-of-tree port, which is the only recovered source describing this
 * register's use.
 */
void hamcop_set_spucr(struct device *hamcop_dev, u16 mask, u16 bits)
{
	struct hamcop *hamcop;

	/* A cell driver reaches this through its own dev->parent; guard the
	 * case where it is somehow bound outside this MFD rather than
	 * dereferencing a NULL device.
	 */
	if (WARN_ON(!hamcop_dev))
		return;

	hamcop = dev_get_drvdata(hamcop_dev);
	if (WARN_ON(!hamcop))
		return;

	hamcop_rmw(hamcop, HAMCOP_GPIO_SPUCR, mask, bits & mask);
}
EXPORT_SYMBOL_GPL(hamcop_set_spucr);

static const struct of_device_id hamcop_of_match[] = {
	{ .compatible = "samsung,hamcop" },
	{ }
};
MODULE_DEVICE_TABLE(of, hamcop_of_match);

static struct platform_driver hamcop_driver = {
	.driver = {
		.name = "hamcop",
		.of_match_table = hamcop_of_match,
	},
	.probe	= hamcop_probe,
};
module_platform_driver(hamcop_driver);

MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("Samsung HAMCOP companion ASIC MFD driver (HP iPAQ h2200)");
MODULE_LICENSE("GPL");
