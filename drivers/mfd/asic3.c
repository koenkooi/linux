// SPDX-License-Identifier: GPL-2.0-only
/*
 * Compaq/HTC ASIC3 companion ASIC MFD driver
 *
 * ASIC3 is a plain 16-bit VLIO-mode MMIO device with a single upstream
 * GPIO interrupt line, plus a second, physically disjoint window for its
 * SD/MMC host block. It bundles a 64-line GPIO block that doubles as its
 * own interrupt sources, a 3-channel LED blink controller, a
 * DS1WM-compatible 1-Wire master, an SPI block and an SD/SDIO host
 * (TMIO-core-compatible), behind one flat clock-gate register.
 *
 * This is a rewrite against current subsystem conventions of the driver
 * removed by commit 8971bb812e3c ("mfd: remove toshiba tmio drivers"),
 * keeping its register map, bit definitions and hardware behaviour. The
 * framework contracts that driver relied on are gone (struct
 * mfd_cell.enable/.disable, static irq_base numbering,
 * tmio_mmc_data.set_clk_div, gpiochip_add_data(), __init-marked probe
 * helpers).
 *
 *  - Register access: plain ioread16_relaxed()/iowrite16_relaxed() plus one
 *    raw_spinlock_t for read-modify-write. asic3_read_register() and
 *    asic3_write_register() are exported for sub-device drivers that need
 *    direct register access, which the gpio_chip/clk_hw API does not
 *    cover.
 *
 *  - IRQ demux: one linear irq_domain of ASIC3_NR_IRQS (70) hwirqs behind
 *    the single chained GPIO IRQ. hwirq 0-63 are the flat GPIO block (banks
 *    A-D, hwirq == gpio offset); hwirq 64-69 are six fixed-function block
 *    interrupts (LED0-2, SPI, SMBUS, OWM) behind a second,
 *    differently-polarised mask register. The hardware cannot trigger on
 *    both edges, so that is emulated in software by flipping EDGE_TRIGGER
 *    after each interrupt according to the pin's current level (see
 *    asic3_gpio_irq_set_type() and asic3_irq_flip_edge()).
 *
 *  - GPIO: devm_gpiochip_add_data(). gpio_chip.to_irq() uses
 *    irq_create_mapping(), since nothing pre-creates mappings for GPIO
 *    hwirqs and irq_find_mapping() would report "no IRQ" for them.
 *    GPIO hogs are supported; with htc,gpio-default-output-low every line
 *    that is not hogged is set as a low output before the gpio_chip is
 *    registered.
 *
 *  - Clocks: twelve fixed-rate gate clk_hws on the single CLOCK_CDEX
 *    register (SPI, OWM, PWM0, PWM1, LED0-2, SD_HOST, SD_BUS, SMBUS, EX0,
 *    EX1), with custom clk_ops because the registers are 16 bits wide and
 *    clk_gate assumes 32. Each gate is a known fixed-frequency crystal tap,
 *    so each is a root clk_hw with its own rate. EX0 and EX1, the two
 *    crystal enables OWM and the SD host both need, are CLK_IS_CRITICAL
 *    because no clk_get() consumer holds a reference on them.
 *
 * The chip is described by a devicetree binding
 * (Documentation/devicetree/bindings/mfd/htc,asic3.yaml). An optional
 * "reset-gpios" is pulsed at probe before any register access. Board
 * policy (LED names and triggers, keys, socket GPIOs) stays out of this
 * file. "leds-asic3" (drivers/leds/leds-asic3.c) is matched to a child
 * device tree node by of_compatible. The SD/MMC host
 * (drivers/mmc/host/tmio_mmc.c, "tmio-mmc") is registered from the second
 * "sd" window through a second devm_mfd_add_devices() call and is matched
 * to an optional "mmc" child node by of_compatible, which only supplies
 * generic MMC properties such as cd-gpios. This file is the MFD core:
 * register access, IRQ demux, gpio_chip, clocks, plus the sub-device
 * registration.
 *
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/mfd/asic3.h>
#include <linux/mfd/core.h>
#include <linux/mfd/ds1wm.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_data/tmio.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/spinlock.h>

/*
 * Chained-handler storm guard. When the bound is hit the handler masks the
 * sources that stayed pending, so a bound that is too low retires a source
 * that was merely busy. ASIC3 has no source that re-asserts within one demux
 * pass, so 100 consecutive non-empty passes mean a stuck source.
 */
#define ASIC3_MAX_IRQ_LOOPS		100

/*
 * A source is persistent, and so masked when the bound is reached, if it was
 * pending in at least this many of the bound's passes. A bystander is
 * serviced once and its consumer cannot re-arm it while the handler runs with
 * interrupts off, so it scores a pass or two; an offender scores nearly all
 * of them, and two offenders that alternate pass by pass score half each.
 */
#define ASIC3_STORM_PERSIST_PASSES	(ASIC3_MAX_IRQ_LOOPS / 4)

/*
 * Bound on the per-bank read-clear-reread of ASIC3_GPIO_INT_STATUS in
 * asic3_irq_handler(). It runs with the lock held and interrupts off, so it
 * is bounded by a count rather than by the register going quiet.
 */
#define ASIC3_MAX_INT_STATUS_PASSES	8

enum {
	ASIC3_GATE_SPI,
	ASIC3_GATE_OWM,
	ASIC3_GATE_PWM0,
	ASIC3_GATE_PWM1,
	ASIC3_GATE_LED0,
	ASIC3_GATE_LED1,
	ASIC3_GATE_LED2,
	ASIC3_GATE_SD_HOST,
	ASIC3_GATE_SD_BUS,
	ASIC3_GATE_SMBUS,
	ASIC3_GATE_EX0,
	ASIC3_GATE_EX1,
	ASIC3_NR_CLK_GATES,
};

struct asic3;

struct asic3_clk_gate {
	struct clk_hw hw;
	struct asic3 *asic;
	u16 cdex_bit;
	unsigned long rate;
};

struct asic3 {
	struct device *dev;
	void __iomem *regs;
	unsigned int bus_shift;
	int irq;

	/* SD_CONFIG sub-window inside the "sd" chip-select resource -- only
	 * mapped when a board wires up ASIC3's SD/MMC host (the "sd" MEM
	 * resource and a named "sd" IRQ both present). NULL otherwise;
	 * asic3_sd_set_clock_mode() is only ever reached through a probed
	 * tmio-mmc cell, which only exists when this is non-NULL.
	 */
	void __iomem *sd_config;

	/* Set once the card-detect debounce wait has been done; see
	 * asic3_sd_wait_card_detect_settle().
	 */
	bool sd_settled;

	/* Guards every register read-modify-write below (GPIO bank
	 * MASK, DIRECTION, OUT and the *_TRIGGER registers, CLOCK_CDEX,
	 * INTR_INT_MASK). raw_spinlock_t because the IRQ chip callbacks
	 * and the chained handler both take it.
	 */
	raw_spinlock_t lock;

	struct irq_domain *domain;
	struct gpio_chip gc;

	/* Both-edge emulation state, one bit per GPIO line, indexed the
	 * same way as the hwirq/gpio offset -- ASIC3 cannot trigger on
	 * both edges natively (unlike HAMCOP); see
	 * asic3_gpio_irq_set_type()/asic3_irq_flip_edge().
	 */
	u16 irq_bothedge[ASIC3_NUM_GPIO_BANKS];

	/*
	 * Sources the storm backstop in asic3_irq_handler() has masked, as
	 * the raw bits the handler tests: P_INT_STAT positions for the fixed
	 * half, GPIO bank bit positions for the banks. Written only under
	 * lock, read locklessly by the handler to narrow what it dispatches.
	 * A source's bit is cleared again by its .irq_unmask.
	 */
	struct {
		u32 fixed;
		u16 bank[ASIC3_NUM_GPIO_BANKS];
	} storm_masked;

	struct asic3_clk_gate gates[ASIC3_NR_CLK_GATES];
};

u32 asic3_read_register(struct asic3 *asic, unsigned int reg)
{
	return ioread16(asic->regs + (reg >> asic->bus_shift));
}
EXPORT_SYMBOL_GPL(asic3_read_register);

void asic3_write_register(struct asic3 *asic, unsigned int reg, u32 value)
{
	iowrite16(value, asic->regs + (reg >> asic->bus_shift));
}
EXPORT_SYMBOL_GPL(asic3_write_register);

static void asic3_rmw(struct asic3 *asic, u32 reg, u32 mask, u32 bits)
{
	unsigned long flags;
	u32 val;

	raw_spin_lock_irqsave(&asic->lock, flags);
	val = asic3_read_register(asic, reg);
	val = (val & ~mask) | (bits & mask);
	asic3_write_register(asic, reg, val);
	raw_spin_unlock_irqrestore(&asic->lock, flags);
}

/* ---------------------------------------------------------------------
 * GPIO chip: banks A-D, 16 pins each, 64 total. hwirq == gpio offset in
 * the domain below for the whole range -- the flat identity that lets a
 * future stock gpio-keys DT node work off this chip with no lookup table.
 * ---------------------------------------------------------------------
 */

static void asic3_gpio_bank_bit(unsigned int offset, u32 *base, unsigned int *bit)
{
	*base = ASIC3_BANK_TO_BASE(offset / ASIC3_GPIOS_PER_BANK);
	*bit = offset % ASIC3_GPIOS_PER_BANK;
}

static int asic3_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct asic3 *asic = gpiochip_get_data(gc);
	u32 base;
	unsigned int bit;

	asic3_gpio_bank_bit(offset, &base, &bit);

	return !!(asic3_read_register(asic, base + ASIC3_GPIO_STATUS) & BIT(bit));
}

static int asic3_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct asic3 *asic = gpiochip_get_data(gc);
	u32 base;
	unsigned int bit;

	asic3_gpio_bank_bit(offset, &base, &bit);
	asic3_rmw(asic, base + ASIC3_GPIO_OUT, BIT(bit), value ? BIT(bit) : 0);

	return 0;
}

static int asic3_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct asic3 *asic = gpiochip_get_data(gc);
	u32 base;
	unsigned int bit;

	asic3_gpio_bank_bit(offset, &base, &bit);

	/* DIRECTION: R/W 0:input, matching ASIC3_GPIO_DIRECTION's own
	 * header comment.
	 */
	return (asic3_read_register(asic, base + ASIC3_GPIO_DIRECTION) & BIT(bit)) ?
		GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN;
}

static int asic3_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	struct asic3 *asic = gpiochip_get_data(gc);
	u32 base;
	unsigned int bit;

	asic3_gpio_bank_bit(offset, &base, &bit);
	asic3_rmw(asic, base + ASIC3_GPIO_DIRECTION, BIT(bit), 0);

	return 0;
}

static int asic3_gpio_direction_output(struct gpio_chip *gc, unsigned int offset,
				       int value)
{
	struct asic3 *asic = gpiochip_get_data(gc);
	u32 base;
	unsigned int bit;

	/*
	 * Set the output level before switching direction, to avoid a
	 * glitch. (The removed driver ignored the value argument entirely.)
	 */
	asic3_gpio_set(gc, offset, value);

	asic3_gpio_bank_bit(offset, &base, &bit);
	asic3_rmw(asic, base + ASIC3_GPIO_DIRECTION, BIT(bit), BIT(bit));

	return 0;
}

static int asic3_gpio_to_irq(struct gpio_chip *gc, unsigned int offset)
{
	struct asic3 *asic = gpiochip_get_data(gc);

	/*
	 * irq_create_mapping(), not irq_find_mapping(): nothing pre-creates a
	 * domain mapping for a GPIO hwirq, so find_mapping() would return 0,
	 * which gpiod_to_irq() treats as "no IRQ".
	 */
	return irq_create_mapping(asic->domain, offset);
}

static int asic3_gpio_register(struct asic3 *asic)
{
	asic->gc.label = "asic3";
	asic->gc.parent = asic->dev;
	asic->gc.owner = THIS_MODULE;
	asic->gc.get_direction = asic3_gpio_get_direction;
	asic->gc.direction_input = asic3_gpio_direction_input;
	asic->gc.direction_output = asic3_gpio_direction_output;
	asic->gc.get = asic3_gpio_get;
	asic->gc.set = asic3_gpio_set;
	asic->gc.to_irq = asic3_gpio_to_irq;
	asic->gc.base = -1;
	asic->gc.ngpio = ASIC3_NUM_GPIOS;
	asic->gc.can_sleep = false;

	return devm_gpiochip_add_data(asic->dev, &asic->gc, asic);
}

/*
 * Output-low default for the lines no gpio-hog claims, applied before the
 * gpio_chip is registered: gpiolib reads every line's direction at
 * registration and applies the hogs right after, so a hogged line goes
 * from its reset state straight to the hog level without a low glitch.
 */
static void asic3_gpio_set_default_low(struct asic3 *asic)
{
	u16 hogged[ASIC3_NUM_GPIO_BANKS] = { };
	unsigned int bank;

	for_each_available_child_of_node_scoped(asic->dev->of_node, child) {
		u32 offset;
		int i, n;

		if (!of_property_read_bool(child, "gpio-hog"))
			continue;

		/* #gpio-cells is 2: offset, flags */
		n = of_property_count_u32_elems(child, "gpios");
		for (i = 0; i + 1 < n; i += 2) {
			if (of_property_read_u32_index(child, "gpios", i, &offset) ||
			    offset >= ASIC3_NUM_GPIOS)
				continue;
			hogged[offset / ASIC3_GPIOS_PER_BANK] |=
				BIT(offset % ASIC3_GPIOS_PER_BANK);
		}
	}

	for (bank = 0; bank < ASIC3_NUM_GPIO_BANKS; bank++) {
		u32 base = ASIC3_BANK_TO_BASE(bank);
		u16 lines = ~hogged[bank];

		asic3_rmw(asic, base + ASIC3_GPIO_OUT, lines, 0);
		asic3_rmw(asic, base + ASIC3_GPIO_DIRECTION, lines, lines);
		asic3_write_register(asic, base + ASIC3_GPIO_ALT_FUNCTION, 0);
	}
}

/* ---------------------------------------------------------------------
 * IRQ chip, GPIO half (hwirq 0-63). ASIC3_GPIO_MASK is "0: don't mask",
 * so a set bit masks.
 *
 * .irq_ack is a no-op. Edge-configured lines are dispatched
 * through handle_edge_irq(), whose per-interrupt path calls only .irq_ack
 * and never .irq_mask, so using the mask as the ack would mask every
 * edge-triggered line on its first interrupt and never unmask it. The
 * hardware ack (clearing each bank's GPIO_INT_STATUS) happens in bulk in
 * asic3_irq_handler() before any hwirq in that bank is dispatched, which
 * is correct for both trigger classes.
 * ---------------------------------------------------------------------
 */

static void asic3_gpio_irq_ack(struct irq_data *d)
{
}

static void asic3_gpio_irq_mask(struct irq_data *d)
{
	struct asic3 *asic = irq_data_get_irq_chip_data(d);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	u32 base;
	unsigned int bit;

	asic3_gpio_bank_bit(hwirq, &base, &bit);
	asic3_rmw(asic, base + ASIC3_GPIO_MASK, BIT(bit), BIT(bit));
}

/*
 * Forget that the storm backstop masked @hwirq. Every .irq_unmask calls this
 * before it unmasks in hardware, not after: with the order reversed, a GPIO
 * detect latched in between would be acked by the IntStatus clear loop and
 * then narrowed away undispatched.
 */
static void asic3_storm_clear(struct asic3 *asic, irq_hw_number_t hwirq)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&asic->lock, flags);
	if (hwirq < ASIC3_NUM_GPIOS) {
		u16 *bank = &asic->storm_masked.bank[hwirq / ASIC3_GPIOS_PER_BANK];

		WRITE_ONCE(*bank, *bank & ~BIT(hwirq % ASIC3_GPIOS_PER_BANK));
	} else {
		u32 *fixed = &asic->storm_masked.fixed;

		WRITE_ONCE(*fixed, *fixed & ~(ASIC3_INTR_LED0 << (hwirq - ASIC3_NUM_GPIOS)));
	}
	raw_spin_unlock_irqrestore(&asic->lock, flags);
}

static void asic3_gpio_irq_unmask(struct irq_data *d)
{
	struct asic3 *asic = irq_data_get_irq_chip_data(d);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	u32 base;
	unsigned int bit;

	asic3_gpio_bank_bit(hwirq, &base, &bit);
	asic3_storm_clear(asic, hwirq);
	asic3_rmw(asic, base + ASIC3_GPIO_MASK, BIT(bit), 0);
}

static void asic3_irq_flip_edge(struct asic3 *asic, u32 base, unsigned int bit)
{
	unsigned long flags;
	u16 edge;

	raw_spin_lock_irqsave(&asic->lock, flags);
	edge = asic3_read_register(asic, base + ASIC3_GPIO_EDGE_TRIGGER);
	edge ^= BIT(bit);
	asic3_write_register(asic, base + ASIC3_GPIO_EDGE_TRIGGER, edge);
	raw_spin_unlock_irqrestore(&asic->lock, flags);
}

static int asic3_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct asic3 *asic = irq_data_get_irq_chip_data(d);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	unsigned int bank = hwirq / ASIC3_GPIOS_PER_BANK;
	u32 base;
	unsigned int bit;
	u16 trigger, level, edge;
	unsigned long flags;

	asic3_gpio_bank_bit(hwirq, &base, &bit);

	raw_spin_lock_irqsave(&asic->lock, flags);
	level = asic3_read_register(asic, base + ASIC3_GPIO_LEVEL_TRIGGER);
	edge = asic3_read_register(asic, base + ASIC3_GPIO_EDGE_TRIGGER);
	trigger = asic3_read_register(asic, base + ASIC3_GPIO_TRIGGER_TYPE);
	asic->irq_bothedge[bank] &= ~BIT(bit);

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		trigger |= BIT(bit);
		edge |= BIT(bit);
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		trigger |= BIT(bit);
		edge &= ~BIT(bit);
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		/*
		 * ASIC3 has no native both-edge trigger. Emulate it in
		 * software: arm the edge opposite the pin's current level, and flip
		 * EDGE_TRIGGER again after each interrupt
		 * (asic3_irq_flip_edge(), called from the chained handler)
		 * so the next one catches the opposite transition.
		 */
		trigger |= BIT(bit);
		if (asic3_gpio_get(&asic->gc, hwirq))
			edge &= ~BIT(bit);
		else
			edge |= BIT(bit);
		asic->irq_bothedge[bank] |= BIT(bit);
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		trigger &= ~BIT(bit);
		level &= ~BIT(bit);
		irq_set_handler_locked(d, handle_level_irq);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		trigger &= ~BIT(bit);
		level |= BIT(bit);
		irq_set_handler_locked(d, handle_level_irq);
		break;
	default:
		raw_spin_unlock_irqrestore(&asic->lock, flags);
		return -EINVAL;
	}

	asic3_write_register(asic, base + ASIC3_GPIO_LEVEL_TRIGGER, level);
	asic3_write_register(asic, base + ASIC3_GPIO_EDGE_TRIGGER, edge);
	asic3_write_register(asic, base + ASIC3_GPIO_TRIGGER_TYPE, trigger);
	raw_spin_unlock_irqrestore(&asic->lock, flags);

	return 0;
}

static int asic3_gpio_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct asic3 *asic = irq_data_get_irq_chip_data(d);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	u32 base;
	unsigned int bit;

	asic3_gpio_bank_bit(hwirq, &base, &bit);

	/* SLEEP_MASK: R/W 0:don't mask in sleep mode -- bit=1 masks
	 * (matches GPIO_MASK's own polarity), so waking on this line means
	 * *clearing* the bit.
	 */
	asic3_rmw(asic, base + ASIC3_GPIO_SLEEP_MASK, BIT(bit), on ? 0 : BIT(bit));

	return 0;
}

static struct irq_chip asic3_gpio_irq_chip = {
	.name		= "asic3-gpio",
	.irq_ack	= asic3_gpio_irq_ack,
	.irq_mask	= asic3_gpio_irq_mask,
	.irq_unmask	= asic3_gpio_irq_unmask,
	.irq_set_type	= asic3_gpio_irq_set_type,
	.irq_set_wake	= asic3_gpio_irq_set_wake,
};

/* ---------------------------------------------------------------------
 * IRQ chip, fixed-function half (hwirq 64-69: LED0-2, SPI, SMBUS, OWM).
 * ASIC3_INTMASK_GINTMASK is "1: enable", the opposite polarity of the
 * GPIO-bank ASIC3_GPIO_MASK register above.
 * ---------------------------------------------------------------------
 */

static void asic3_fixed_irq_mask(struct irq_data *d)
{
	struct asic3 *asic = irq_data_get_irq_chip_data(d);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	u32 bit = ASIC3_INTMASK_MASK0 << (hwirq - ASIC3_NUM_GPIOS);

	asic3_rmw(asic, ASIC3_INTR_BASE + ASIC3_INTR_INT_MASK, bit, 0);
}

static void asic3_fixed_irq_unmask(struct irq_data *d)
{
	struct asic3 *asic = irq_data_get_irq_chip_data(d);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	u32 bit = ASIC3_INTMASK_MASK0 << (hwirq - ASIC3_NUM_GPIOS);

	asic3_storm_clear(asic, hwirq);
	asic3_rmw(asic, ASIC3_INTR_BASE + ASIC3_INTR_INT_MASK, bit, bit);
}

static struct irq_chip asic3_fixed_irq_chip = {
	.name		= "asic3",
	.irq_ack	= asic3_gpio_irq_ack,	/* shared no-op, see above */
	.irq_mask	= asic3_fixed_irq_mask,
	.irq_unmask	= asic3_fixed_irq_unmask,
};

/* ---------------------------------------------------------------------
 * IRQ domain + chained handler.
 * ---------------------------------------------------------------------
 */

static int asic3_irq_domain_map(struct irq_domain *d, unsigned int virq,
				irq_hw_number_t hwirq)
{
	struct irq_chip *chip = hwirq < ASIC3_NUM_GPIOS ?
		&asic3_gpio_irq_chip : &asic3_fixed_irq_chip;

	/*
	 * handle_level_irq() is the default for every hwirq at map time,
	 * matching the hardware reset default (GPIO_TRIGGER_TYPE resets to
	 * "0:level"). GPIO-half consumers that need edge triggering switch
	 * via asic3_gpio_irq_set_type()'s irq_set_handler_locked() call; the
	 * fixed half has no .irq_set_type and stays level.
	 */
	irq_set_chip_and_handler(virq, chip, handle_level_irq);
	irq_set_chip_data(virq, d->host_data);
	irq_set_probe(virq);

	return 0;
}

static void asic3_irq_domain_unmap(struct irq_domain *d, unsigned int virq)
{
	irq_set_chip_and_handler(virq, NULL, NULL);
	irq_set_chip_data(virq, NULL);
}

static const struct irq_domain_ops asic3_irq_domain_ops = {
	.map	= asic3_irq_domain_map,
	.unmap	= asic3_irq_domain_unmap,
	.xlate	= irq_domain_xlate_twocell,
};

/*
 * State of one bounded drain round of the chained handler: on how many
 * passes each source was pending, and the union of the narrowed pending
 * sets. Sources are indexed by hwirq; the fixed ones keep their P_INT_STAT
 * bit positions in seen_fixed.
 */
struct asic3_storm_round {
	u16 cnt[ASIC3_NR_IRQS];
	u16 seen_gpio[ASIC3_NUM_GPIO_BANKS];
	u32 seen_fixed;
};

/*
 * One drain round. Returns true when it reached ASIC3_MAX_IRQ_LOOPS with
 * something still pending.
 */
static bool asic3_irq_round(struct asic3 *asic, struct asic3_storm_round *r)
{
	unsigned int loops, bank, i;
	u32 status;

	for (loops = 0; loops < ASIC3_MAX_IRQ_LOOPS; loops++) {
		status = asic3_read_register(asic, ASIC3_INTR_BASE + ASIC3_INTR_P_INT_STAT);
		/*
		 * Bits 0-3 are bank routing bits and stay as read; a bank's
		 * own pins are narrowed below, after its IntStatus is read.
		 */
		status &= ~READ_ONCE(asic->storm_masked.fixed);
		if (!(status & ASIC3_INTR_PENDING_MASK))
			break;

		for (bank = 0; bank < ASIC3_NUM_GPIO_BANKS; bank++) {
			u32 base = ASIC3_BANK_TO_BASE(bank);
			unsigned long istat = 0;
			unsigned long flags;
			unsigned int pass;

			if (!(status & BIT(bank)))
				continue;

			/*
			 * Clearing IntStatus is a plain write of 0, not a
			 * write-1-to-clear, and clears the whole bank rather
			 * than a single pin. The register is documented only
			 * as "R/W 0:none, 1:detect", so writing ~istat to
			 * clear just the bits observed is not an option.
			 *
			 * A pin that latches a detect between the read and
			 * the write would be wiped without being dispatched,
			 * and behind the edge-only parent that is lost until
			 * the pin produces another edge. So read, clear and
			 * re-read, accumulating what was seen, until a read
			 * after a clear comes back empty.
			 */
			raw_spin_lock_irqsave(&asic->lock, flags);
			for (pass = 0; pass < ASIC3_MAX_INT_STATUS_PASSES; pass++) {
				u32 pending;

				pending = asic3_read_register(asic,
							      base + ASIC3_GPIO_INT_STATUS);
				if (!pending)
					break;
				istat |= pending;
				asic3_write_register(asic, base + ASIC3_GPIO_INT_STATUS, 0);
			}
			raw_spin_unlock_irqrestore(&asic->lock, flags);

			if (unlikely(pass == ASIC3_MAX_INT_STATUS_PASSES))
				dev_err_ratelimited(asic->dev,
						    "bank %u IntStatus will not clear (istat %#lx)\n",
						    bank, istat);

			/*
			 * The clear above has already acked every pin a storm
			 * backstop masked; they only have to stay undispatched.
			 */
			istat &= ~(unsigned long)READ_ONCE(asic->storm_masked.bank[bank]);
			r->seen_gpio[bank] |= istat;

			for_each_set_bit(i, &istat, ASIC3_GPIOS_PER_BANK) {
				unsigned int hwirq = bank * ASIC3_GPIOS_PER_BANK + i;

				r->cnt[hwirq]++;
				generic_handle_domain_irq(asic->domain, hwirq);
				if (asic->irq_bothedge[bank] & BIT(i))
					asic3_irq_flip_edge(asic, base, i);
			}
		}

		for (i = 0; i < 6; i++) {
			if (status & BIT(i + 4)) {
				r->seen_fixed |= BIT(i + 4);
				r->cnt[ASIC3_NUM_GPIOS + i]++;
				generic_handle_domain_irq(asic->domain, ASIC3_NUM_GPIOS + i);
			}
		}
	}

	return loops == ASIC3_MAX_IRQ_LOOPS;
}

static bool asic3_storm_has(irq_hw_number_t hwirq, u32 fixed, const u16 *bank)
{
	if (hwirq < ASIC3_NUM_GPIOS)
		return bank[hwirq / ASIC3_GPIOS_PER_BANK] &
		       BIT(hwirq % ASIC3_GPIOS_PER_BANK);

	return fixed & (ASIC3_INTR_LED0 << (hwirq - ASIC3_NUM_GPIOS));
}

/*
 * Mask the given sources in hardware and then record them in storm_masked,
 * in that order, so the handler never narrows away a source that is still
 * live in hardware.
 *
 * The two halves mask through registers of opposite polarity:
 * ASIC3_GPIO_MASK is "0: don't mask", so masking sets bits, while the
 * MASK0-5 bits of ASIC3_INTR_INT_MASK are "1: enable", so masking clears
 * them.
 *
 * No acknowledge is added: the GPIO half is already cleared on every pass,
 * and the fixed sources have none.
 */
static void asic3_storm_mask(struct asic3 *asic, u32 fixed, const u16 *bank)
{
	u32 fixed_mask_bits = 0;
	unsigned long flags;
	unsigned int i;
	u32 val;

	for (i = 0; i < 6; i++)
		if (fixed & (ASIC3_INTR_LED0 << i))
			fixed_mask_bits |= ASIC3_INTMASK_MASK0 << i;

	raw_spin_lock_irqsave(&asic->lock, flags);

	if (fixed_mask_bits) {
		val = asic3_read_register(asic, ASIC3_INTR_BASE + ASIC3_INTR_INT_MASK);
		asic3_write_register(asic, ASIC3_INTR_BASE + ASIC3_INTR_INT_MASK,
				     val & ~fixed_mask_bits);
		WRITE_ONCE(asic->storm_masked.fixed, asic->storm_masked.fixed | fixed);
	}

	for (i = 0; i < ASIC3_NUM_GPIO_BANKS; i++) {
		u32 reg = ASIC3_BANK_TO_BASE(i) + ASIC3_GPIO_MASK;

		if (!bank[i])
			continue;

		val = asic3_read_register(asic, reg);
		asic3_write_register(asic, reg, val | bank[i]);
		WRITE_ONCE(asic->storm_masked.bank[i], asic->storm_masked.bank[i] | bank[i]);
	}

	raw_spin_unlock_irqrestore(&asic->lock, flags);
}

/*
 * A round reached its bound. Returns true if it masked the union of what the
 * round saw rather than just the persistent sources, which ends the handler's
 * rounds.
 *
 * Fault isolation, not a fix for whatever leaves a source stuck. The parent
 * is edge-only and ASIC3_IRQ stays asserted while any enabled source is
 * pending, so returning with a source still asserted would lose the whole
 * domain silently.
 *
 * Only the sources pending on at least ASIC3_STORM_PERSIST_PASSES passes are
 * masked, never everything the round saw and never a whole tier. Masking the
 * union takes out a bystander that fired once in the window together with the
 * offender, and it also trips on a source that is merely busy. The counts, not
 * the intersection of the per-pass sets, decide, because two sources that
 * alternate pass by pass have an empty intersection although both storm.
 * Once the persistent sources are masked a second round drains whatever is
 * left; only when there is no persistent source, or the second round hits its
 * bound as well, is the union masked, so the backstop still ends every storm.
 *
 * Whether a masked source still shows up in P_INT_STAT is undocumented. A
 * fixed source is narrowed by storm_masked.fixed and cannot keep the loop
 * spinning; a bank routing bit that stays up costs another bounded round of
 * passes that dispatch nothing.
 */
static bool asic3_irq_storm(struct asic3 *asic, struct asic3_storm_round *r,
			    unsigned int round)
{
	unsigned int min_masked = UINT_MAX, max_spared = 0;
	u32 fixed = 0, spared_fixed;
	u16 bank[ASIC3_NUM_GPIO_BANKS] = { 0 };
	u16 spared_bank[ASIC3_NUM_GPIO_BANKS];
	unsigned int hwirq, i;
	bool fallback;

	for (hwirq = 0; hwirq < ASIC3_NR_IRQS; hwirq++) {
		if (r->cnt[hwirq] < ASIC3_STORM_PERSIST_PASSES)
			continue;
		if (hwirq < ASIC3_NUM_GPIOS)
			bank[hwirq / ASIC3_GPIOS_PER_BANK] |=
				BIT(hwirq % ASIC3_GPIOS_PER_BANK);
		else
			fixed |= ASIC3_INTR_LED0 << (hwirq - ASIC3_NUM_GPIOS);
	}

	fallback = round > 0 || !(fixed | bank[0] | bank[1] | bank[2] | bank[3]);
	if (fallback) {
		fixed = r->seen_fixed;
		for (i = 0; i < ASIC3_NUM_GPIO_BANKS; i++)
			bank[i] = r->seen_gpio[i];
	}

	spared_fixed = r->seen_fixed & ~fixed;
	for (i = 0; i < ASIC3_NUM_GPIO_BANKS; i++)
		spared_bank[i] = r->seen_gpio[i] & ~bank[i];

	for (hwirq = 0; hwirq < ASIC3_NR_IRQS; hwirq++) {
		if (asic3_storm_has(hwirq, fixed, bank)) {
			if (r->cnt[hwirq] < min_masked)
				min_masked = r->cnt[hwirq];
		} else if (asic3_storm_has(hwirq, spared_fixed, spared_bank)) {
			if (r->cnt[hwirq] > max_spared)
				max_spared = r->cnt[hwirq];
		}
	}
	if (min_masked == UINT_MAX)
		min_masked = 0;

	dev_err_ratelimited(asic->dev,
			    "interrupt storm after %u passes: masking fixed %#x banks %04x/%04x/%04x/%04x, sparing fixed %#x banks %04x/%04x/%04x/%04x; counts masked>=%u spared<=%u%s\n",
			    ASIC3_MAX_IRQ_LOOPS,
			    fixed, bank[0], bank[1], bank[2], bank[3],
			    spared_fixed, spared_bank[0], spared_bank[1], spared_bank[2],
			    spared_bank[3], min_masked, max_spared,
			    fallback ? " [fallback]" : "");

	asic3_storm_mask(asic, fixed, bank);

	return fallback;
}

static void asic3_irq_handler(struct irq_desc *desc)
{
	struct asic3 *asic = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int round;

	chained_irq_enter(chip, desc);

	for (round = 0; round < 2; round++) {
		struct asic3_storm_round r = { };

		if (likely(!asic3_irq_round(asic, &r)))
			break;
		if (asic3_irq_storm(asic, &r, round))
			break;
	}

	chained_irq_exit(chip, desc);
}

static void asic3_irq_teardown(void *data)
{
	struct asic3 *asic = data;

	irq_set_chained_handler_and_data(asic->irq, NULL, NULL);
}

static int asic3_irq_init(struct asic3 *asic)
{
	const struct irq_domain_info info = {
		.fwnode		= dev_fwnode(asic->dev),
		.size		= ASIC3_NR_IRQS,
		.hwirq_max	= ASIC3_NR_IRQS,
		.ops		= &asic3_irq_domain_ops,
		.host_data	= asic,
	};
	unsigned int bank;

	/* Mask every GPIO line ("0:don't mask" is GPIO_MASK's own reset
	 * default assumption; write it explicitly regardless of whatever
	 * HaRET/bootldr left behind) before the chained handler can fire.
	 */
	for (bank = 0; bank < ASIC3_NUM_GPIO_BANKS; bank++)
		asic3_write_register(asic, ASIC3_BANK_TO_BASE(bank) + ASIC3_GPIO_MASK, 0xffff);

	/*
	 * CLOCK_SEL_CX turns on the clock feeding ASIC3's own interrupt
	 * controller block, which P_INT_STAT needs in order to update. It is
	 * not part of the CLOCK_CDEX gate roster below.
	 */
	asic3_write_register(asic, ASIC3_CLOCK_BASE + ASIC3_CLOCK_SEL, CLOCK_SEL_CX);

	/*
	 * INTR_INT_MASK: GINTMASK only. This is a plain write, so it also
	 * zeroes MASK0-5 (bits 2-7), masking all six fixed-function sources
	 * by default, like the GPIO loop above.
	 */
	asic3_write_register(asic, ASIC3_INTR_BASE + ASIC3_INTR_INT_MASK,
			     ASIC3_INTMASK_GINTMASK);

	asic->domain = devm_irq_domain_instantiate(asic->dev, &info);
	if (IS_ERR(asic->domain))
		return PTR_ERR(asic->domain);

	/* The upstream interrupt is active-high, edge-rising. */
	irq_set_irq_type(asic->irq, IRQ_TYPE_EDGE_RISING);
	irq_set_chained_handler_and_data(asic->irq, asic3_irq_handler, asic);

	/*
	 * The chained handler dereferences devres-managed memory, the mapping
	 * and the IRQ domain, so unhook it from a devres action registered
	 * after them: devres unwinds in reverse order and the handler is gone
	 * before they are released, on probe failure and on unbind alike.
	 * This function only runs when a "core" IRQ resource exists.
	 */
	return devm_add_action_or_reset(asic->dev, asic3_irq_teardown, asic);
}

/* ---------------------------------------------------------------------
 * Clocks (CCF): twelve fixed-rate gates on the single CLOCK_CDEX
 * register. The rates are the crystal taps and derived rates of the
 * hardware.
 * ---------------------------------------------------------------------
 */

/*
 * CLOCK_CDEX_SD_BUS selects the rate of the SD bus interface: set for
 * 24.576 MHz, clear for 12.288 MHz. The h3900 runs it clear, as its WinCE
 * driver does, while the hx4700 runs it set. The enable path of the gate
 * and the rate the gate reports have to agree, because the tmio-mmc driver
 * derives its card clock divisor from clk_get_rate(), so both use this one
 * predicate.
 */
static bool asic3_sd_bus_is_fast(void)
{
	return !of_machine_is_compatible("hp,ipaq-h3900");
}

static int asic3_clk_gate_enable(struct clk_hw *hw)
{
	struct asic3_clk_gate *gate = container_of(hw, struct asic3_clk_gate, hw);

	if (gate->cdex_bit == CLOCK_CDEX_SD_BUS && !asic3_sd_bus_is_fast())
		return 0;

	asic3_rmw(gate->asic, ASIC3_CLOCK_BASE + ASIC3_CLOCK_CDEX,
		  gate->cdex_bit, gate->cdex_bit);

	return 0;
}

static void asic3_clk_gate_disable(struct clk_hw *hw)
{
	struct asic3_clk_gate *gate = container_of(hw, struct asic3_clk_gate, hw);

	asic3_rmw(gate->asic, ASIC3_CLOCK_BASE + ASIC3_CLOCK_CDEX, gate->cdex_bit, 0);
}

static int asic3_clk_gate_is_enabled(struct clk_hw *hw)
{
	struct asic3_clk_gate *gate = container_of(hw, struct asic3_clk_gate, hw);

	return !!(asic3_read_register(gate->asic, ASIC3_CLOCK_BASE + ASIC3_CLOCK_CDEX) &
		  gate->cdex_bit);
}

static unsigned long asic3_clk_gate_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct asic3_clk_gate *gate = container_of(hw, struct asic3_clk_gate, hw);

	return gate->rate;
}

static const struct clk_ops asic3_clk_gate_ops = {
	.enable		= asic3_clk_gate_enable,
	.disable	= asic3_clk_gate_disable,
	.is_enabled	= asic3_clk_gate_is_enabled,
	.recalc_rate	= asic3_clk_gate_recalc_rate,
};

static const struct {
	const char *name;
	u16 cdex_bit;
	unsigned long rate;
	unsigned long flags;
} asic3_clk_defs[ASIC3_NR_CLK_GATES] = {
	[ASIC3_GATE_SPI]     = { "asic3-spi",     CLOCK_CDEX_SPI,     0, 0 },
	[ASIC3_GATE_OWM]     = { "asic3-owm",     CLOCK_CDEX_OWM,     5000000, 0 },
	[ASIC3_GATE_PWM0]    = { "asic3-pwm0",    CLOCK_CDEX_PWM0,    0, 0 },
	[ASIC3_GATE_PWM1]    = { "asic3-pwm1",    CLOCK_CDEX_PWM1,    0, 0 },
	[ASIC3_GATE_LED0]    = { "asic3-led0",    CLOCK_CDEX_LED0,    0, 0 },
	[ASIC3_GATE_LED1]    = { "asic3-led1",    CLOCK_CDEX_LED1,    0, 0 },
	[ASIC3_GATE_LED2]    = { "asic3-led2",    CLOCK_CDEX_LED2,    0, 0 },
	[ASIC3_GATE_SMBUS]   = { "asic3-smbus",   CLOCK_CDEX_SMBUS,   0, 0 },
	/*
	 * EX0/EX1: the 32.768 kHz and 24.576 MHz crystal enables. Both are
	 * shared prerequisites of OWM and the SD host with no single
	 * clk_get() consumer to hold a reference, so they are
	 * CLK_IS_CRITICAL rather than left to clk_disable_unused().
	 */
	[ASIC3_GATE_EX0]     = { "asic3-ex0", CLOCK_CDEX_EX0, 32768, CLK_IS_CRITICAL },
	[ASIC3_GATE_EX1]     = { "asic3-ex1", CLOCK_CDEX_EX1, 24576000, CLK_IS_CRITICAL },
	/*
	 * SD_HOST/SD_BUS are CLK_IS_CRITICAL for ordering rather than for lack
	 * of a consumer: asic3_sd_host_init() writes the SD_CONFIG block
	 * synchronously from probe, while the tmio-mmc cell probes
	 * asynchronously. Registering the gates as critical turns them on
	 * before that sequence runs; the driver's own devm_clk_get_enabled()
	 * then layers a balanced enable/disable pair on top for runtime PM.
	 * The cost is that both gates stay enabled on a board that does not
	 * wire up SD, which is only an internal divider tap.
	 */
	[ASIC3_GATE_SD_HOST] = { "asic3-sd-host", CLOCK_CDEX_SD_HOST, 24576000, CLK_IS_CRITICAL },
	/* The SD_BUS rate is board dependent, see asic3_sd_bus_is_fast(). */
	[ASIC3_GATE_SD_BUS]  = { "asic3-sd-bus",  CLOCK_CDEX_SD_BUS,  12288000, CLK_IS_CRITICAL },
};

static int asic3_clocks_register(struct asic3 *asic)
{
	int i, ret;

	for (i = 0; i < ASIC3_NR_CLK_GATES; i++) {
		struct clk_init_data init = {
			.name	= asic3_clk_defs[i].name,
			.ops	= &asic3_clk_gate_ops,
			.flags	= asic3_clk_defs[i].flags,
		};

		asic->gates[i].asic = asic;
		asic->gates[i].cdex_bit = asic3_clk_defs[i].cdex_bit;
		asic->gates[i].rate = asic3_clk_defs[i].rate;
		if (i == ASIC3_GATE_SD_BUS && asic3_sd_bus_is_fast())
			asic->gates[i].rate = 24576000;
		asic->gates[i].hw.init = &init;

		ret = devm_clk_hw_register(asic->dev, &asic->gates[i].hw);
		if (ret)
			return ret;
	}

	/*
	 * "ds1wm": drivers/w1/masters/ds1wm.c calls devm_clk_get(dev, NULL).
	 * The alias is registered ahead of the cell that will use it.
	 */
	ret = devm_clk_hw_register_clkdev(asic->dev, &asic->gates[ASIC3_GATE_OWM].hw,
					  NULL, "ds1wm");
	if (ret)
		return ret;

	/*
	 * "sd_host"/"sd_bus" against dev_name "tmio-mmc" (the SD mfd_cell
	 * below): drivers/mmc/host/tmio_mmc.c acquires both with
	 * devm_clk_get_enabled() before touching the SD_CTRL registers. Both
	 * gates are already CLK_IS_CRITICAL, so these aliases give the
	 * driver a balanced enable/disable handle rather than being what
	 * first turns the hardware on.
	 */
	ret = devm_clk_hw_register_clkdev(asic->dev, &asic->gates[ASIC3_GATE_SD_HOST].hw,
					  "sd_host", "tmio-mmc");
	if (ret)
		return ret;

	ret = devm_clk_hw_register_clkdev(asic->dev, &asic->gates[ASIC3_GATE_SD_BUS].hw,
					  "sd_bus", "tmio-mmc");
	if (ret)
		return ret;

	/*
	 * "led0"/"led1"/"led2" against dev_name "leds-asic3" (the mfd_cell
	 * below): one alias per channel, as the three CLOCK_CDEX_LEDn gates
	 * are independent. The LED driver uses devm_clk_get_enabled() so that
	 * a failed probe does not leak the prepare/enable counts.
	 */
	ret = devm_clk_hw_register_clkdev(asic->dev, &asic->gates[ASIC3_GATE_LED0].hw,
					  "led0", "leds-asic3");
	if (ret)
		return ret;

	ret = devm_clk_hw_register_clkdev(asic->dev, &asic->gates[ASIC3_GATE_LED1].hw,
					  "led1", "leds-asic3");
	if (ret)
		return ret;

	return devm_clk_hw_register_clkdev(asic->dev, &asic->gates[ASIC3_GATE_LED2].hw,
					    "led2", "leds-asic3");
}

/* ---------------------------------------------------------------------
 * SD/MMC host bring-up, for the "sd" resource window (a second, physically
 * disjoint window from the "core" one everything above uses). Neither the
 * mfd_cell .enable/.disable hooks nor tmio_core.c exist any more, so the
 * chip-level setup is done here unconditionally from asic3_probe() rather
 * than from the child cell's probe: the tmio-mmc cell probes
 * asynchronously, so there is no guarantee it has probed before this
 * returns. That is also why SD_HOST/SD_BUS are CLK_IS_CRITICAL in the
 * table above.
 * ---------------------------------------------------------------------
 */

static void asic3_sd_config_write8(struct asic3 *asic, unsigned int reg, u8 value)
{
	/*
	 * The shift is 1 - bus_shift rather than bus_shift: the SD_CONFIG
	 * offsets are expressed in "32 bit addressing", one step coarser than
	 * the 16-bit VLIO offsets of the "core" window, so with bus_shift = 1
	 * they are plain byte offsets.
	 */
	iowrite8(value, asic->sd_config + (reg << (1 - asic->bus_shift)));
}

static void asic3_sd_config_write16(struct asic3 *asic, unsigned int reg, u16 value)
{
	iowrite16(value, asic->sd_config + (reg << (1 - asic->bus_shift)));
}

/* CTL_BASE is a 32-bit register accessed as two 16-bit halves */
static void asic3_sd_config_write32(struct asic3 *asic, unsigned int reg, u32 value)
{
	unsigned int shift = 1 - asic->bus_shift;

	iowrite16(value, asic->sd_config + (reg << shift));
	iowrite16(value >> 16, asic->sd_config + ((reg + 2) << shift));
}

void asic3_sd_set_clock_mode(struct asic3 *asic, bool gated)
{
	asic3_sd_config_write8(asic, ASIC3_SD_CONFIG_CLOCK_MODE, gated ? 1 : 0);
}
EXPORT_SYMBOL_GPL(asic3_sd_set_clock_mode);

/*
 * Power the SD slot's 3.3V rail (SDHWCTRL.SDPWR). This cannot be done from
 * asic3_sd_host_init(), which runs from probe before the tmio-mmc cell
 * resets the SD_CTRL block; the rail has to come up after that reset, so
 * the tmio-mmc reset hook calls this instead.
 */
void asic3_sd_set_power(struct asic3 *asic, bool on)
{
	asic3_rmw(asic, ASIC3_SDHWCTRL_BASE + ASIC3_SDHWCTRL_SDCONF,
		  ASIC3_SDHWCTRL_SDPWR, on ? ASIC3_SDHWCTRL_SDPWR : 0);
}
EXPORT_SYMBOL_GPL(asic3_sd_set_power);

/*
 * Power the SD bus (SD_CONFIG.PWR2), which is separate from the slot rail
 * above. PWR2 is an internal TMIO bus power bit, not a regulator. It is
 * left off by asic3_sd_host_init(), switched on from the tmio-mmc reset
 * hook, which also runs again on runtime PM resume, and cycled with
 * MMC_POWER_UP and MMC_POWER_OFF by the set_pwr callback of the tmio-mmc
 * host.
 */
void asic3_sd_set_bus_power(struct asic3 *asic, bool on)
{
	asic3_sd_config_write8(asic, ASIC3_SD_CONFIG_PWR2, on ? 0x02 : 0x00);
}
EXPORT_SYMBOL_GPL(asic3_sd_set_bus_power);

/*
 * Power the SD slot of the h3900 through SD_CONFIG.PWR1 and PWR2. The
 * WinCE SD driver writes the two as one native 32-bit register, 0 before
 * power-down and 0x00000200 after power-up, and never addresses PWR2 on its
 * own, so do the same instead of using asic3_sd_set_bus_power().
 */
void asic3_sd_set_slot_voltage(struct asic3 *asic, bool on)
{
	iowrite32(on ? 0x00000200 : 0x00000000,
		  asic->sd_config +
		  (ASIC3_SD_CONFIG_PWR1 << (1 - asic->bus_shift)));
}
EXPORT_SYMBOL_GPL(asic3_sd_set_slot_voltage);

/*
 * SDHC_CardDetect mode 2 samples the card-detect pin with a debounce of
 * about 512 ms, so the status read in the first card scan right after probe
 * would report an empty slot. Wait it out once, from the reset hook, which
 * runs before mmc_add_host() can start that scan. Later resets (error
 * recovery, runtime PM resume) do not need to wait again.
 */
void asic3_sd_wait_card_detect_settle(struct asic3 *asic)
{
	if (asic->sd_settled)
		return;

	msleep(600);
	asic->sd_settled = true;
}
EXPORT_SYMBOL_GPL(asic3_sd_wait_card_detect_settle);

/*
 * SD_CTRL_BASE right-shifted by bus_shift is both the resource offset the
 * "sd" tmio-mmc cell gets and the base address written to the CTL_BASE
 * register of SD_CONFIG. The cell gets a per-probe copy of the resource,
 * see asic3_probe(), so a re-probe does not shift it twice.
 */
static const struct tmio_mmc_data asic3_sd_cell_data = {
	/*
	 * Fallback only: tmio_mmc_probe() replaces its copy of this with the
	 * rate of the "sd_bus" clock, which depends on the board.
	 */
	.hclk		= 24576000,
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	/* SD_CTRL reports the slot in the "_A" status bits only. */
	.flags		= TMIO_MMC_CD_IN_ALT_BITS,
};

static int asic3_sd_host_init(struct asic3 *asic, struct resource *sd_mem)
{
	u32 ctl_base;

	asic->sd_config = devm_ioremap(asic->dev,
				       sd_mem->start + (ASIC3_SD_CONFIG_BASE >> asic->bus_shift),
					ASIC3_SD_CONFIG_SIZE >> asic->bus_shift);
	if (!asic->sd_config)
		return -ENOMEM;

	/*
	 * SDHWCTRL, CLOCK_SEL and EXTCF setup. CLOCK_SEL is overwritten
	 * rather than modified: its only earlier writer is asic3_irq_init(),
	 * always called first from probe, which sets CLOCK_SEL_CX, so
	 * re-asserting that bit along with CLOCK_SEL_SD_HCLK_SEL is safe.
	 */
	asic3_rmw(asic, ASIC3_SDHWCTRL_BASE + ASIC3_SDHWCTRL_SDCONF,
		  ASIC3_SDHWCTRL_LEVCD, ASIC3_SDHWCTRL_LEVCD);
	asic3_rmw(asic, ASIC3_SDHWCTRL_BASE + ASIC3_SDHWCTRL_SDCONF,
		  ASIC3_SDHWCTRL_LEVWP, ASIC3_SDHWCTRL_LEVWP);
	asic3_rmw(asic, ASIC3_SDHWCTRL_BASE + ASIC3_SDHWCTRL_SDCONF,
		  ASIC3_SDHWCTRL_SUSPEND, 0);
	asic3_rmw(asic, ASIC3_SDHWCTRL_BASE + ASIC3_SDHWCTRL_SDCONF,
		  ASIC3_SDHWCTRL_PCLR, 0);

	/*
	 * The vendor firmware enables the two clock stages in a fixed order
	 * with 1 ms settle delays: EX0/EX1, then CLOCK_SEL, then SD_HOST/
	 * SD_BUS, then the SD memory enable. Do the same, even though the
	 * gates are already kept on, so the hardware sees that sequence.
	 */
	asic3_rmw(asic, ASIC3_CLOCK_BASE + ASIC3_CLOCK_CDEX,
		  CLOCK_CDEX_EX0 | CLOCK_CDEX_EX1,
		  CLOCK_CDEX_EX0 | CLOCK_CDEX_EX1);
	usleep_range(1000, 2000);

	asic3_write_register(asic, ASIC3_CLOCK_BASE + ASIC3_CLOCK_SEL,
			     CLOCK_SEL_CX | CLOCK_SEL_SD_HCLK_SEL);

	asic3_rmw(asic, ASIC3_CLOCK_BASE + ASIC3_CLOCK_CDEX,
		  CLOCK_CDEX_SD_HOST | CLOCK_CDEX_SD_BUS,
		  CLOCK_CDEX_SD_HOST |
		  (asic3_sd_bus_is_fast() ? CLOCK_CDEX_SD_BUS : 0));
	usleep_range(1000, 2000);

	asic3_rmw(asic, ASIC3_EXTCF_BASE + ASIC3_EXTCF_SELECT,
		  ASIC3_EXTCF_SD_MEM_ENABLE, ASIC3_EXTCF_SD_MEM_ENABLE);

	/*
	 * SD_CONFIG bring-up: enable access to the SD_CTRL registers, tell
	 * SD_CONFIG where SD_CTRL sits, then the power and clock housekeeping
	 * writes, including the STOP_CLK_CTL one whose purpose is unknown but
	 * which the removed driver also made. SDHC_CardDetect routes the
	 * physical card-detect pin into the SD_CTRL card status bits; without
	 * it the slot always looks empty. The slot rail and the bus power
	 * are switched on later, see asic3_sd_set_power() and
	 * asic3_sd_set_bus_power().
	 *
	 * CTL_BASE holds the position of the SD_CTRL window in units of
	 * 16-bit words, whatever the wiring: 0x0800 words for the window at
	 * byte offset 0x1000, which is the value the h3900 reads under
	 * WinCE. It is therefore shifted by a fixed 1, as in the removed
	 * driver, and not by bus_shift like the cell resource, which is the
	 * address at which the CPU sees the window and does depend on the
	 * wiring. Scaling it by bus_shift puts the window at the wrong place
	 * where bus_shift is 0.
	 */
	ctl_base = (ASIC3_SD_CTRL_BASE >> 1) & 0xfffe;
	asic3_sd_config_write16(asic, ASIC3_SD_CONFIG_COMMAND, ASIC3_SD_CONFIG_COMMAND_MAE);
	asic3_sd_config_write32(asic, ASIC3_SD_CONFIG_CTL_BASE, ctl_base);
	/*
	 * PWR3 and PWR1/PWR2 differ between the boards. The hx4700 stack
	 * writes PWR3 = 0x01 and a bus power byte of 0x00 to PWR2. On the
	 * h3900, whose ASIC3 is an older revision, the working WinCE state
	 * reads PWR3 = 0x00 and PWR1 = 0x0200 (bit 9) with a card inserted,
	 * which is what the WinCE SD driver's power-up leaves set. That
	 * driver writes PWR1 and PWR2 as one 32-bit register, so the slot
	 * voltage helper does too and PWR2 is not written on its own.
	 */
	if (of_machine_is_compatible("hp,ipaq-h3900")) {
		asic3_sd_config_write8(asic, ASIC3_SD_CONFIG_PWR3, 0x00);
		asic3_sd_set_slot_voltage(asic, true);
	} else {
		asic3_sd_config_write8(asic, ASIC3_SD_CONFIG_PWR3, 0x01);
	}
	asic3_sd_config_write8(asic, ASIC3_SD_CONFIG_CLK_STOP, 0x1f);
	if (!of_machine_is_compatible("hp,ipaq-h3900"))
		asic3_sd_config_write8(asic, ASIC3_SD_CONFIG_PWR2, 0x00);
	asic3_sd_config_write16(asic, ASIC3_SD_CONFIG_SDHC_CARDDETECT, 2);

	return 0;
}

/* ---------------------------------------------------------------------
 * Sub-devices. The LED blink controller (drivers/leds/leds-asic3.c) is
 * matched by of_compatible against a child device tree node, so that board
 * policy (LED names and triggers) stays in the device tree. The DS1WM
 * 1-Wire master (drivers/w1/masters/ds1wm.c) and the SD/MMC host
 * (drivers/mmc/host/tmio_mmc.c) get an .of_compatible only when the board
 * describes them, so that mfd-core does not warn on a board that does not.
 * ---------------------------------------------------------------------
 */

/*
 * ds1wm maps its own registers from a MEM resource, unlike leds-asic3 which
 * goes through asic3_read_register()/asic3_write_register(). The resource
 * starts out as an offset in the "core" window's raw convention and is
 * shifted by asic->bus_shift in asic3_probe() before the cells are added,
 * like asic3_sd_cell_resources[], so it cannot be const. The IRQ is a plain
 * hwirq number, resolved in the chip's irq_domain by mfd_add_device().
 *
 * The DS1WM register stride (bus_shift in the platform data) is board
 * wiring and unrelated to the outer ASIC3 bus_shift. The default below is
 * meant to be overridden by the board's "onewire" node; on the hx4700,
 * where the OWM block is five 8-bit registers on 16-bit bus words, it is 1.
 */
static const struct ds1wm_driver_data asic3_ds1wm_pdata = {
	.bus_shift		= 0,
	.active_high		= 1,
	.reset_recover_delay	= 1,
};

static struct resource asic3_ds1wm_resources[] = {
	DEFINE_RES_MEM(ASIC3_OWM_BASE, ASIC3_OWM_SIZE),
	DEFINE_RES_IRQ(ASIC3_IRQ_OWM),
};

static const struct mfd_cell asic3_leds_cell = {
	.name		= "leds-asic3",
	.id		= PLATFORM_DEVID_NONE,
	.of_compatible	= "htc,asic3-leds",
};

static const struct mfd_cell asic3_ds1wm_cell = {
	.name		= "ds1wm",
	.id		= PLATFORM_DEVID_NONE,
	.platform_data	= &asic3_ds1wm_pdata,
	.pdata_size	= sizeof(asic3_ds1wm_pdata),
	.resources	= asic3_ds1wm_resources,
	.num_resources	= ARRAY_SIZE(asic3_ds1wm_resources),
};

/*
 * Whether the node has an available child with the given compatible. mfd-core
 * warns for a cell that sets .of_compatible when no such child exists, so a
 * cell that is optional in the devicetree is only added, or only gets its
 * .of_compatible, when the board describes it.
 */
static bool asic3_has_child(struct device *dev, const char *compatible)
{
	struct device_node *np;
	bool present;

	np = of_get_compatible_child(dev->of_node, compatible);
	present = np && of_device_is_available(np);
	of_node_put(np);

	return present;
}

/*
 * ds1wm only ungates the OWM clock. The block also needs its reset bit
 * pulsed with that clock running, and EXTCF_SELECT.OWM_EN set with OWM_SMB
 * clear, or its registers do not answer. The two crystal enables are held
 * on by the critical gates, ds1wm derives its clock divisor from the gate
 * rate, and the OWM interrupt is unmasked when ds1wm requests it, so none
 * of those is repeated here.
 *
 * Only done for a board that describes a "onewire" node; others keep the
 * block in reset.
 */
static void asic3_owm_init(struct asic3 *asic)
{
	struct clk *owm_clk;

	owm_clk = clk_hw_get_clk(&asic->gates[ASIC3_GATE_OWM].hw, "owm");
	if (IS_ERR(owm_clk) || clk_prepare_enable(owm_clk)) {
		dev_warn(asic->dev, "cannot ungate the OWM clock, 1-Wire reset skipped\n");
		if (!IS_ERR(owm_clk))
			clk_put(owm_clk);
		return;
	}

	asic3_rmw(asic, ASIC3_EXTCF_BASE + ASIC3_EXTCF_RESET,
		  ASIC3_EXTCF_OWM_RESET, ASIC3_EXTCF_OWM_RESET);
	usleep_range(1000, 2000);
	asic3_rmw(asic, ASIC3_EXTCF_BASE + ASIC3_EXTCF_RESET,
		  ASIC3_EXTCF_OWM_RESET, 0);

	asic3_rmw(asic, ASIC3_EXTCF_BASE + ASIC3_EXTCF_SELECT,
		  ASIC3_EXTCF_OWM_SMB | ASIC3_EXTCF_OWM_EN,
		  ASIC3_EXTCF_OWM_EN);

	/* ds1wm takes its own reference when it probes. */
	clk_disable_unprepare(owm_clk);
	clk_put(owm_clk);
}

/* ---------------------------------------------------------------------
 * Probe/lifecycle
 * ---------------------------------------------------------------------
 */

static int asic3_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct asic3 *asic;
	struct resource *mem, *sd_mem;
	struct gpio_desc *reset;
	struct clk *sd_clk;
	struct mfd_cell owm_cell = asic3_ds1wm_cell;
	int irq, sd_irq, ret;

	asic = devm_kzalloc(dev, sizeof(*asic), GFP_KERNEL);
	if (!asic)
		return -ENOMEM;

	asic->dev = dev;
	raw_spin_lock_init(&asic->lock);
	platform_set_drvdata(pdev, asic);

	mem = platform_get_resource_byname(pdev, IORESOURCE_MEM, "core");
	if (!mem)
		return dev_err_probe(dev, -EINVAL, "no \"core\" MEM resource\n");

	asic->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(asic->regs))
		return PTR_ERR(asic->regs);

	/*
	 * bus_shift from the "core" window's own size: a 0x1000 window on a
	 * 16-bit VLIO wiring gives bus_shift = 1.
	 */
	asic->bus_shift = 2 - (resource_size(mem) >> 12);

	/*
	 * "sd" is a second, physically disjoint window for the SD host block,
	 * with its own interrupt, which is not part of this chip's chained
	 * GPIO IRQ domain but a raw SoC GPIO interrupt requested directly by
	 * the tmio-mmc cell. Both are optional, as ASIC3 is also used on
	 * boards that do not wire up SD.
	 */
	sd_mem = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sd");
	if (!sd_mem)
		dev_dbg(dev, "no \"sd\" MEM resource\n");

	sd_irq = platform_get_irq_byname_optional(pdev, "sd");
	if (sd_irq < 0)
		dev_dbg(dev, "no \"sd\" IRQ resource\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	asic->irq = irq;

	/*
	 * Reset pulse before any other register access. The line is
	 * optional and active-low; GPIOD_OUT_HIGH asserts it at request
	 * time.
	 */
	reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset))
		return PTR_ERR(reset);
	if (reset) {
		usleep_range(10000, 20000);
		gpiod_set_value_cansleep(reset, 0);
		usleep_range(10000, 20000);
	}

	/*
	 * The SD card clock can come from outside the chip and must be
	 * running before asic3_sd_host_init(). It is taken before anything is
	 * registered, so that a deferral while its provider is missing leaves
	 * no sub-device or gpio_chip behind to be torn down again.
	 */
	if (sd_mem && sd_irq >= 0) {
		sd_clk = devm_clk_get_optional_enabled(dev, NULL);
		if (IS_ERR(sd_clk))
			return dev_err_probe(dev, PTR_ERR(sd_clk),
					     "failed to get SD clock\n");
	}

	/* CLOCK_SEL reset to a known state before anything else touches it. */
	asic3_write_register(asic, ASIC3_CLOCK_BASE + ASIC3_CLOCK_SEL, 0);

	ret = asic3_irq_init(asic);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set up IRQ domain\n");

	if (device_property_read_bool(dev, "htc,gpio-default-output-low"))
		asic3_gpio_set_default_low(asic);

	ret = asic3_gpio_register(asic);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register gpio_chip\n");

	ret = asic3_clocks_register(asic);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register clocks\n");

	/*
	 * The LED driver fails to probe without its "led-controller" child,
	 * so a board that has none gets no LED cell. id 0 with
	 * PLATFORM_DEVID_NONE on the cell gives an unnumbered "leds-asic3".
	 */
	if (asic3_has_child(dev, "htc,asic3-leds")) {
		ret = devm_mfd_add_devices(dev, 0, &asic3_leds_cell, 1, mem, 0,
					   asic->domain);
		if (ret)
			return dev_err_probe(dev, ret, "failed to add sub-devices\n");
	}

	asic3_ds1wm_resources[0].start >>= asic->bus_shift;
	asic3_ds1wm_resources[0].end   >>= asic->bus_shift;

	/*
	 * The board's "onewire" node, if any, overrides the compiled-in
	 * tunables above.
	 */
	if (asic3_has_child(dev, "htc,asic3-onewire")) {
		owm_cell.of_compatible = "htc,asic3-onewire";
		asic3_owm_init(asic);
	}

	ret = devm_mfd_add_devices(dev, 0, &owm_cell, 1, mem, 0,
				   asic->domain);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add the 1-Wire sub-device\n");

	/*
	 * SD/MMC: a second devm_mfd_add_devices() call with its own mem_base,
	 * since mfd_add_device() applies a single mem_base to all the memory
	 * resources of a call and the "sd" window is physically disjoint from
	 * "core". There is no irq_domain: the SDIO interrupt is a raw SoC
	 * interrupt, not a hwirq of this chip.
	 */
	if (sd_mem && sd_irq >= 0) {
		struct resource sd_res[] = {
			DEFINE_RES_MEM(ASIC3_SD_CTRL_BASE, 0x400),
			DEFINE_RES_IRQ(0),
		};
		struct mfd_cell sd_cell = {
			.name		= "tmio-mmc",
			.id		= PLATFORM_DEVID_NONE,
			.platform_data	= &asic3_sd_cell_data,
			.pdata_size	= sizeof(asic3_sd_cell_data),
			.resources	= sd_res,
			.num_resources	= ARRAY_SIZE(sd_res),
		};

		ret = asic3_sd_host_init(asic, sd_mem);
		if (ret)
			return dev_err_probe(dev, ret, "failed to set up SD/MMC host\n");

		sd_res[0].start >>= asic->bus_shift;
		sd_res[0].end >>= asic->bus_shift;

		/*
		 * Lets a board give the host a firmware node, from which
		 * mmc_of_parse() picks up properties such as cd-gpios. A
		 * board that has no such child gets none.
		 */
		if (asic3_has_child(dev, "htc,asic3-sd"))
			sd_cell.of_compatible = "htc,asic3-sd";

		ret = devm_mfd_add_devices(dev, 0, &sd_cell, 1, sd_mem, sd_irq, NULL);
		if (ret)
			return dev_err_probe(dev, ret, "failed to add SD/MMC sub-device\n");
	}

	dev_dbg(dev, "ASIC3 companion ASIC ready, chained on IRQ %d\n", irq);

	return 0;
}

static const struct of_device_id asic3_of_match[] = {
	{ .compatible = "htc,asic3" },
	{ }
};
MODULE_DEVICE_TABLE(of, asic3_of_match);

static struct platform_driver asic3_driver = {
	.driver = {
		.name = "asic3",
		.of_match_table = asic3_of_match,
	},
	.probe	= asic3_probe,
};
module_platform_driver(asic3_driver);

MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("Compaq/HTC ASIC3 companion ASIC MFD driver");
MODULE_LICENSE("GPL");
