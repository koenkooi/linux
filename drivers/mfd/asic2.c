// SPDX-License-Identifier: GPL-2.0-only
/*
 * Compaq/HTC ASIC2 companion ASIC core driver (HP iPAQ h38xx/h39xx family)
 *
 * ASIC2 never had a mainline driver. The register layout follows the
 * Compaq register header of the out-of-tree handhelds.org kernels
 * (ipaq-asic2.h) and is corroborated by the 2.4 handhelds.org
 * asic2_io.c/asic2_shared.c drivers. The structure follows the other
 * companion-chip MFD cores:
 *
 *  - Register access: readl_relaxed()/writel_relaxed() plus one
 *    raw_spinlock_t for read-modify-write. It is 32 bit because every
 *    ASIC2 register has a flat 4-byte stride regardless of its field
 *    width (see include/linux/mfd/asic2.h).
 *  - IRQ demux: one linear irq_domain of ASIC2_NR_IRQS (16) hwirqs behind
 *    the single chained GPIO IRQ. hwirq 0-8 live in the KPIO register
 *    block, 9-13 in GPIO2 (see asic2_irq_map[]). KPIO and GPIO2 share an
 *    identical internal register layout, so one set of helpers serves
 *    both through a block-base parameter.
 *  - GPIO: the 16-line GPIO2 bank is a gpio_chip, with hwirq == gpio
 *    offset identity for the five lines that are also interrupt sources
 *    (5, 6, 7, 10, 11: PEN, SD_DETECT, EAR_IN, USB_DETECT, SD_CON_SLT).
 *    KPIO's 12 keypad lines are deliberately not a gpio_chip: they are
 *    interrupt-source bits sharing a register with eight unrelated
 *    peripheral interrupt bits, not addressable GPIO lines.
 *  - Clocks: CCF clk_hw gates on the flat Enable register, one per bit a
 *    cell could need, with clkdev aliases. EX0, the clock of the chip's
 *    interrupt controller, is forced on at probe instead of being exposed
 *    as a gate. EX1 and EX2 are plain gates and are not modelled as
 *    parents of the peripheral gates that depend on them, as no consumer
 *    needs rate information.
 *  - Sub-devices: one devm_mfd_add_devices() call registers the cells:
 *    "ds1wm" (the 1-Wire driver as is, since the register layout matches
 *    it with bus_shift = 2), "asic2-spi" (drivers/spi/spi-asic2.c),
 *    "asic2-pwm" (drivers/pwm/pwm-asic2.c), "asic2-keys"
 *    (drivers/input/keyboard/asic2-keys.c, the KPIO keypad, which is not
 *    gpio-keys because its bits are not addressable GPIO lines),
 *    "asic2-adc" (drivers/iio/adc/asic2-adc.c) and "resistive-adc-touch"
 *    (the generic touchscreen driver, bound to a "touchscreen" child
 *    node whose io-channels phandles point into the "adc" cell, and added
 *    only when the board has that child). The
 *    ASIC2 ADC does not switch the touchscreen plates itself, so GPIO2
 *    bits have to be driven by software and
 *    asic2_touch_gpios_register() hands asic2-adc.c the GPIO descriptors
 *    for them. "leds-asic2" covers the three-channel status LED blink-timer
 *    block, which drives the status LEDs of the h3800; it is added only
 *    when the board has a "led-controller" child.
 *
 * KPIO's InterruptEnable register (offset 0x001c) must not be read back,
 * so this driver keeps a software shadow of its state instead of doing
 * read-modify-write on the hardware register.
 *
 * Copyright 2001 Compaq Computer Corporation.
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/machine.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/mfd/asic2.h>
#include <linux/mfd/core.h>
#include <linux/mfd/ds1wm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/spinlock.h>

/*
 * Chained-handler storm guard. When the bound is hit the handler masks the
 * persistent sources, so a bound that is too low retires a source that was
 * merely busy. No ASIC2 source re-asserts within the time of one demux
 * pass: the fastest is the ADC conversion-complete, tens of microseconds
 * per conversion, so 100 consecutive non-empty passes mean a stuck source.
 */
#define ASIC2_MAX_IRQ_LOOPS		100

/*
 * A source pending in at least this many of one round's passes is storming
 * and gets masked when the round hits its bound; one pending in fewer is a
 * bystander that happened to fire inside the window and is left enabled.
 * A bystander is serviced in one pass and its consumer cannot re-arm it
 * while the handler runs with interrupts off, so it scores one or two.
 */
#define ASIC2_STORM_PERSIST_PASSES	(ASIC2_MAX_IRQ_LOOPS / 4)

/*
 * Bound on re-draining after a source asserts late, see asic2_irq_handler().
 * Each retry is a whole further drain cycle of register reads on a chip
 * select with no bus-error timeout, taken with interrupts off, so it is
 * kept small: a source that keeps re-asserting inside this window is stuck
 * and belongs to the storm guard.
 */
#define ASIC2_MAX_IRQ_RECHECKS		3

#define ASIC2_GPIO_NR			16

enum {
	ASIC2_GATE_ADC,
	ASIC2_GATE_SPI,
	/* Before OWM and SD, which are parented on it and so need it registered first */
	ASIC2_GATE_EX1,
	ASIC2_GATE_OWM,
	ASIC2_GATE_PWM,
	ASIC2_GATE_UART_0,
	ASIC2_GATE_UART_1,
	ASIC2_GATE_EX2,
	ASIC2_GATE_SD,
	ASIC2_NR_CLK_GATES,
};

struct asic2;

/*
 * .bit is the value written to turn the gate on, .mask the field it is
 * written into. For every single-bit gate the two are identical; SD is the
 * exception: a 3-bit rate selector whose "on" value picks a frequency and
 * whose "off" value is all zeroes, so clearing it needs the whole field.
 */
struct asic2_clk_gate {
	struct clk_hw hw;
	struct asic2 *asic;
	u32 bit;
	u32 mask;
	/*
	 * Set only on the gates that something asks a rate of, EX1 and OWM.
	 * Everything else leaves it 0 and falls back to the parent rate in
	 * asic2_clk_gate_recalc_rate().
	 */
	unsigned long rate;
};

/*
 * Where a hwirq lives: the register block, and the bit(s) within that
 * block's Type/Edge/Level/Clear-Flag/Enable registers. ASIC2_IRQ_KEY is
 * the one multi-bit entry: the 12 keypad bits of KPIO share one hwirq.
 */
struct asic2_irq_map {
	u32 block_base;
	u32 mask;
};

static const struct asic2_irq_map asic2_irq_map[ASIC2_NR_IRQS] = {
	[ASIC2_IRQ_KEY]		= { ASIC2_KPIO_BASE, ASIC2_KPIO_KEY_ALL },
	[ASIC2_IRQ_SPI]		= { ASIC2_KPIO_BASE, ASIC2_KPIO_SPI_INT },
	[ASIC2_IRQ_OWM]		= { ASIC2_KPIO_BASE, ASIC2_KPIO_OWM_INT },
	[ASIC2_IRQ_ADC]		= { ASIC2_KPIO_BASE, ASIC2_KPIO_ADC_INT },
	[ASIC2_IRQ_UART_0]	= { ASIC2_KPIO_BASE, ASIC2_KPIO_UART_0_INT },
	[ASIC2_IRQ_UART_1]	= { ASIC2_KPIO_BASE, ASIC2_KPIO_UART_1_INT },
	[ASIC2_IRQ_TIMER_0]	= { ASIC2_KPIO_BASE, ASIC2_KPIO_TIMER_0_INT },
	[ASIC2_IRQ_TIMER_1]	= { ASIC2_KPIO_BASE, ASIC2_KPIO_TIMER_1_INT },
	[ASIC2_IRQ_TIMER_2]	= { ASIC2_KPIO_BASE, ASIC2_KPIO_TIMER_2_INT },
	[ASIC2_IRQ_PEN]		= { ASIC2_GPIO_BASE, ASIC2_GPIO2_PEN_IRQ },
	[ASIC2_IRQ_SD_DETECT]	= { ASIC2_GPIO_BASE, ASIC2_GPIO2_SD_DETECT },
	[ASIC2_IRQ_EAR_IN]	= { ASIC2_GPIO_BASE, ASIC2_GPIO2_EAR_IN_N },
	[ASIC2_IRQ_USB_DETECT]	= { ASIC2_GPIO_BASE, ASIC2_GPIO2_USB_DETECT_N },
	[ASIC2_IRQ_SD_CON_SLT]	= { ASIC2_GPIO_BASE, ASIC2_GPIO2_SD_CON_SLT },
};

/*
 * The union of asic2_irq_map[]'s entries per register block: every flag bit
 * the domain covers, and so every bit the demux may treat as pending, mask
 * or acknowledge. Other bits belong to no hwirq and are never touched.
 */
#define ASIC2_KPIO_IRQ_ALL	(ASIC2_KPIO_KEY_ALL | ASIC2_KPIO_SPI_INT | \
				 ASIC2_KPIO_OWM_INT | ASIC2_KPIO_ADC_INT | \
				 ASIC2_KPIO_UART_0_INT | ASIC2_KPIO_UART_1_INT | \
				 ASIC2_KPIO_TIMER_0_INT | ASIC2_KPIO_TIMER_1_INT | \
				 ASIC2_KPIO_TIMER_2_INT)
#define ASIC2_GPIO2_IRQ_ALL	(ASIC2_GPIO2_PEN_IRQ | ASIC2_GPIO2_SD_DETECT | \
				 ASIC2_GPIO2_EAR_IN_N | ASIC2_GPIO2_USB_DETECT_N | \
				 ASIC2_GPIO2_SD_CON_SLT)

/* GPIO2 offset (0-15) to hwirq, for the five interrupt-capable lines only */
static int asic2_gpio_offset_to_hwirq(unsigned int offset)
{
	switch (offset) {
	case 5:  return ASIC2_IRQ_PEN;
	case 6:  return ASIC2_IRQ_SD_DETECT;
	case 7:  return ASIC2_IRQ_EAR_IN;
	case 10: return ASIC2_IRQ_USB_DETECT;
	case 11: return ASIC2_IRQ_SD_CON_SLT;
	default: return -ENXIO;
	}
}

struct asic2 {
	struct device *dev;
	void __iomem *regs;
	int irq;

	/* Guards every register read-modify-write below, including the
	 * KPIO enable shadow -- the IRQ chip callbacks and the chained
	 * handler both take it.
	 */
	raw_spinlock_t lock;

	/*
	 * Shadow of KPIO's InterruptEnable, which is write-only in practice:
	 * mask/unmask of a KPIO hwirq (0-8) uses this instead of reading the
	 * register. GPIO2's InterruptEnable can be read normally.
	 */
	u32 kpio_int_enable_shadow;

	/* Bitmask over hwirq (ASIC2_NR_IRQS is 16, fits comfortably) of
	 * every hwirq currently emulating IRQ_TYPE_EDGE_BOTH -- see
	 * asic2_irq_set_type()/asic2_irq_flip_edge(). Mirrors asic3.c's own
	 * irq_bothedge[], flat rather than per-bank since ASIC2's whole IRQ
	 * space already is (one flat ASIC2_NR_IRQS window, not banked).
	 */
	u32 irq_bothedge;

	/*
	 * Flag bits the storm backstop has masked, per block, in register bit
	 * positions. The drain loop and the late-assert recheck ignore these
	 * bits: whether a flag still latches with its enable bit clear is not
	 * established, and a masked stuck source must not hold the loop at its
	 * bound on every later entry. Written only under @lock; cleared by
	 * asic2_irq_unmask() before the hardware unmask.
	 */
	u32 kpio_storm_masked;
	u32 gpio2_storm_masked;

	struct irq_domain *domain;
	struct gpio_chip gc;

	struct asic2_clk_gate gates[ASIC2_NR_CLK_GATES];

	/* Handed to the "asic2-adc" cell as platform_data -- see
	 * asic2_touch_gpios_register()'s own comment.
	 */
	struct asic2_adc_touch_pdata touch_gpios;
};

static inline u32 asic2_read(struct asic2 *asic, u32 reg)
{
	return readl_relaxed(asic->regs + reg);
}

static inline void asic2_write(struct asic2 *asic, u32 reg, u32 val)
{
	writel_relaxed(val, asic->regs + reg);
}

static void asic2_rmw(struct asic2 *asic, u32 reg, u32 mask, u32 bits)
{
	unsigned long flags;
	u32 val;

	raw_spin_lock_irqsave(&asic->lock, flags);
	val = asic2_read(asic, reg);
	val = (val & ~mask) | bits;
	asic2_write(asic, reg, val);
	raw_spin_unlock_irqrestore(&asic->lock, flags);
}

/*
 * The cell drivers map the same register window themselves, so a
 * read-modify-write of a register this core also modifies (KPIO's
 * direction, interrupt type and interrupt level registers share bits with
 * the peripheral-interrupt sources this core owns) would race the
 * read-modify-writes above. Cells that must change such a register go
 * through here, under the same lock, and change only the bits they own.
 * @asic2_dev is the cell's parent (the ASIC2 platform device), @reg is
 * relative to the start of the whole window.
 */
void asic2_update_bits(struct device *asic2_dev, u32 reg, u32 mask, u32 bits)
{
	asic2_rmw(dev_get_drvdata(asic2_dev), reg, mask, bits);
}
EXPORT_SYMBOL_GPL(asic2_update_bits);

/* ---------------------------------------------------------------------
 * GPIO chip: GPIO2 bank only, 16 lines, flat offset 0-15.
 * ---------------------------------------------------------------------
 */

static int asic2_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct asic2 *asic = gpiochip_get_data(gc);

	return !!(asic2_read(asic, ASIC2_GPIO_BASE + ASIC2_REG_DATA) & BIT(offset));
}

static int asic2_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct asic2 *asic = gpiochip_get_data(gc);

	asic2_rmw(asic, ASIC2_GPIO_BASE + ASIC2_REG_DATA, BIT(offset),
		  value ? BIT(offset) : 0);

	return 0;
}

static int asic2_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct asic2 *asic = gpiochip_get_data(gc);
	u32 dir = asic2_read(asic, ASIC2_GPIO_BASE + ASIC2_REG_DIRECTION);

	/* The register has 1 for input and 0 for output */
	return (dir & BIT(offset)) ? GPIO_LINE_DIRECTION_IN : GPIO_LINE_DIRECTION_OUT;
}

static int asic2_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	struct asic2 *asic = gpiochip_get_data(gc);

	asic2_rmw(asic, ASIC2_GPIO_BASE + ASIC2_REG_DIRECTION, BIT(offset), BIT(offset));

	return 0;
}

static int asic2_gpio_direction_output(struct gpio_chip *gc, unsigned int offset,
				       int value)
{
	struct asic2 *asic = gpiochip_get_data(gc);

	/* Set the level before switching direction, to avoid a glitch */
	asic2_gpio_set(gc, offset, value);
	asic2_rmw(asic, ASIC2_GPIO_BASE + ASIC2_REG_DIRECTION, BIT(offset), 0);

	return 0;
}

static int asic2_gpio_to_irq(struct gpio_chip *gc, unsigned int offset)
{
	struct asic2 *asic = gpiochip_get_data(gc);
	int hwirq = asic2_gpio_offset_to_hwirq(offset);

	if (hwirq < 0)
		return hwirq;

	/*
	 * irq_create_mapping(), not irq_find_mapping(): nothing pre-creates
	 * the mapping, and 0 would be taken as "no IRQ".
	 */
	return irq_create_mapping(asic->domain, hwirq);
}

static int asic2_gpio_register(struct asic2 *asic)
{
	asic->gc.label = "asic2";
	asic->gc.parent = asic->dev;
	asic->gc.owner = THIS_MODULE;
	asic->gc.get_direction = asic2_gpio_get_direction;
	asic->gc.direction_input = asic2_gpio_direction_input;
	asic->gc.direction_output = asic2_gpio_direction_output;
	asic->gc.get = asic2_gpio_get;
	asic->gc.set = asic2_gpio_set;
	asic->gc.to_irq = asic2_gpio_to_irq;
	asic->gc.base = -1;
	asic->gc.ngpio = ASIC2_GPIO_NR;
	asic->gc.can_sleep = false;

	return devm_gpiochip_add_data(asic->dev, &asic->gc, asic);
}

static void asic2_touch_gpio_free(void *desc)
{
	gpiochip_free_own_desc(desc);
}

/*
 * Each line is released through devm, so that a later probe failure or an
 * unbind frees it before devm_gpiochip_add_data() removes the chip.
 */
static struct gpio_desc *asic2_touch_gpio_request(struct asic2 *asic,
						  unsigned int hwnum,
						  const char *label,
						  enum gpiod_flags flags)
{
	struct gpio_desc *desc;
	int ret;

	desc = gpiochip_request_own_desc(&asic->gc, hwnum, label,
					 GPIO_LOOKUP_FLAGS_DEFAULT, flags);
	if (IS_ERR(desc))
		return desc;

	ret = devm_add_action_or_reset(asic->dev, asic2_touch_gpio_free, desc);
	if (ret)
		return ERR_PTR(ret);

	return desc;
}

/*
 * The 4 touch-plate lines (GPIO2 offsets 0-3) and the pen-contact line
 * (offset 5) are requested here, by the gpio_chip's own driver, through
 * gpiochip_request_own_desc(), rather than by asic2-adc.c: that cell has no
 * of_node and no gpiod_lookup_table exists for GPIO2, so a gpiod_get() from
 * asic2-adc.c would have nothing to resolve against. The descriptors are
 * ordinary ones backed by this gpio_chip, so asic2-adc.c's
 * gpiod_set_value() calls still serialize through the same raw spinlock as
 * every other GPIO2 access in this file. A second mapping of the GPIO2 data
 * register in asic2-adc.c would have to read-modify-write the plate bits
 * without disturbing the unrelated bits of that register, unsynchronized
 * with this driver.
 *
 * The plate lines start as outputs driven low, an inert starting point
 * until asic2-adc.c configures the plates. The pen line is an input used
 * for the level poll at the end of a sweep; its falling-edge interrupt is
 * configured separately, through ASIC2_IRQ_PEN.
 */
static int asic2_touch_gpios_register(struct asic2 *asic)
{
	struct asic2_adc_touch_pdata *tp = &asic->touch_gpios;

	tp->gpio_y1_n = asic2_touch_gpio_request(asic, 0, "touch-y1-n",
						 GPIOD_OUT_LOW);
	if (IS_ERR(tp->gpio_y1_n))
		return PTR_ERR(tp->gpio_y1_n);

	tp->gpio_x0 = asic2_touch_gpio_request(asic, 1, "touch-x0",
					       GPIOD_OUT_LOW);
	if (IS_ERR(tp->gpio_x0))
		return PTR_ERR(tp->gpio_x0);

	tp->gpio_y0 = asic2_touch_gpio_request(asic, 2, "touch-y0",
					       GPIOD_OUT_LOW);
	if (IS_ERR(tp->gpio_y0))
		return PTR_ERR(tp->gpio_y0);

	tp->gpio_x1_n = asic2_touch_gpio_request(asic, 3, "touch-x1-n",
						 GPIOD_OUT_LOW);
	if (IS_ERR(tp->gpio_x1_n))
		return PTR_ERR(tp->gpio_x1_n);

	tp->gpio_pen = asic2_touch_gpio_request(asic, 5, "touch-pen", GPIOD_IN);
	if (IS_ERR(tp->gpio_pen))
		return PTR_ERR(tp->gpio_pen);

	return 0;
}

/* ---------------------------------------------------------------------
 * IRQ chip + domain: one flat ASIC2_NR_IRQS-hwirq window, KPIO (0-8)
 * and GPIO2 (9-13) sharing an identical per-block register layout, see
 * asic2_irq_map[] above.
 * ---------------------------------------------------------------------
 */

static void asic2_irq_ack(struct irq_data *d)
{
	struct asic2 *asic = irq_data_get_irq_chip_data(d);
	const struct asic2_irq_map *m = &asic2_irq_map[irqd_to_hwirq(d)];

	/*
	 * Write-1-to-clear: writing the interrupt's own bit to InterruptClear
	 * acknowledges it, as on the sibling companion ASICs. The register
	 * documentation is inconsistent about the polarity.
	 */
	asic2_write(asic, m->block_base + ASIC2_REG_INT_CLEAR, m->mask);
}

static void asic2_irq_mask(struct irq_data *d)
{
	struct asic2 *asic = irq_data_get_irq_chip_data(d);
	const struct asic2_irq_map *m = &asic2_irq_map[irqd_to_hwirq(d)];
	unsigned long flags;

	if (m->block_base == ASIC2_KPIO_BASE) {
		/* Shadow only: KPIO's InterruptEnable is never read back */
		raw_spin_lock_irqsave(&asic->lock, flags);
		asic->kpio_int_enable_shadow &= ~m->mask;
		asic2_write(asic, ASIC2_KPIO_BASE + ASIC2_REG_INT_ENABLE,
			    asic->kpio_int_enable_shadow);
		raw_spin_unlock_irqrestore(&asic->lock, flags);
	} else {
		asic2_rmw(asic, m->block_base + ASIC2_REG_INT_ENABLE, m->mask, 0);
	}
}

static void asic2_irq_unmask(struct irq_data *d)
{
	struct asic2 *asic = irq_data_get_irq_chip_data(d);
	const struct asic2_irq_map *m = &asic2_irq_map[irqd_to_hwirq(d)];
	unsigned long flags;

	/*
	 * Every unmask also re-arms a source the storm backstop masked,
	 * clearing its storm_masked bit before the hardware unmask so the
	 * handler sees it again the moment it can fire: request_irq() after
	 * free_irq(), enable_irq() after a masking disable and a oneshot
	 * thread's final unmask undo a storm mask exactly as they undo any
	 * other.
	 */
	if (m->block_base == ASIC2_KPIO_BASE) {
		raw_spin_lock_irqsave(&asic->lock, flags);
		WRITE_ONCE(asic->kpio_storm_masked,
			   asic->kpio_storm_masked & ~m->mask);
		asic->kpio_int_enable_shadow |= m->mask;
		asic2_write(asic, ASIC2_KPIO_BASE + ASIC2_REG_INT_ENABLE,
			    asic->kpio_int_enable_shadow);
		raw_spin_unlock_irqrestore(&asic->lock, flags);
	} else {
		raw_spin_lock_irqsave(&asic->lock, flags);
		WRITE_ONCE(asic->gpio2_storm_masked,
			   asic->gpio2_storm_masked & ~m->mask);
		raw_spin_unlock_irqrestore(&asic->lock, flags);

		asic2_rmw(asic, m->block_base + ASIC2_REG_INT_ENABLE, m->mask, m->mask);
	}
}

/*
 * ASIC2_IRQ_KEY's mask spans all 12 KPIO keypad bits, so one trigger type
 * applies to the whole group. A keypad cell that needs per-key polarity
 * re-arms it from outside this generic chip.
 *
 * The trigger registers only select edge versus level per block and rising
 * versus falling edge, so IRQ_TYPE_EDGE_BOTH has no hardware encoding. It is
 * emulated like asic3 does: arm the edge opposite the pin's current level,
 * and flip the edge after each interrupt so the next one catches the
 * opposite transition.
 */
static void asic2_irq_flip_edge(struct asic2 *asic, u32 block_base, u32 mask)
{
	unsigned long flags;
	u32 edge;

	raw_spin_lock_irqsave(&asic->lock, flags);
	edge = asic2_read(asic, block_base + ASIC2_REG_INT_EDGE);
	edge ^= mask;
	asic2_write(asic, block_base + ASIC2_REG_INT_EDGE, edge);
	raw_spin_unlock_irqrestore(&asic->lock, flags);
}

static int asic2_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct asic2 *asic = irq_data_get_irq_chip_data(d);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	const struct asic2_irq_map *m = &asic2_irq_map[hwirq];
	unsigned long flags;

	raw_spin_lock_irqsave(&asic->lock, flags);
	asic->irq_bothedge &= ~BIT(hwirq);
	raw_spin_unlock_irqrestore(&asic->lock, flags);

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		asic2_rmw(asic, m->block_base + ASIC2_REG_INT_EDGE, m->mask, m->mask);
		asic2_rmw(asic, m->block_base + ASIC2_REG_INT_TYPE, m->mask, m->mask);
		irq_set_handler_locked(d, handle_edge_irq);
		return 0;
	case IRQ_TYPE_EDGE_FALLING:
		asic2_rmw(asic, m->block_base + ASIC2_REG_INT_EDGE, m->mask, 0);
		asic2_rmw(asic, m->block_base + ASIC2_REG_INT_TYPE, m->mask, m->mask);
		irq_set_handler_locked(d, handle_edge_irq);
		return 0;
	case IRQ_TYPE_EDGE_BOTH:
		/*
		 * No native both-edge trigger (see the header comment
		 * above). Arm the edge opposite the pin's *current* level
		 * -- read straight off the block's Data register, same
		 * register asic2_gpio_get() reads for GPIO2 -- and flip
		 * ASIC2_REG_INT_EDGE again after each interrupt
		 * (asic2_irq_flip_edge(), called from asic2_irq_handler())
		 * so the next one catches the opposite transition.
		 */
		asic2_rmw(asic, m->block_base + ASIC2_REG_INT_TYPE, m->mask, m->mask);
		if (asic2_read(asic, m->block_base + ASIC2_REG_DATA) & m->mask)
			asic2_rmw(asic, m->block_base + ASIC2_REG_INT_EDGE, m->mask, 0);
		else
			asic2_rmw(asic, m->block_base + ASIC2_REG_INT_EDGE, m->mask, m->mask);
		raw_spin_lock_irqsave(&asic->lock, flags);
		asic->irq_bothedge |= BIT(hwirq);
		raw_spin_unlock_irqrestore(&asic->lock, flags);
		irq_set_handler_locked(d, handle_edge_irq);
		return 0;
	case IRQ_TYPE_LEVEL_HIGH:
		asic2_rmw(asic, m->block_base + ASIC2_REG_INT_LEVEL, m->mask, m->mask);
		asic2_rmw(asic, m->block_base + ASIC2_REG_INT_TYPE, m->mask, 0);
		irq_set_handler_locked(d, handle_level_irq);
		return 0;
	case IRQ_TYPE_LEVEL_LOW:
		asic2_rmw(asic, m->block_base + ASIC2_REG_INT_LEVEL, m->mask, 0);
		asic2_rmw(asic, m->block_base + ASIC2_REG_INT_TYPE, m->mask, 0);
		irq_set_handler_locked(d, handle_level_irq);
		return 0;
	default:
		return -EINVAL;
	}
}

static struct irq_chip asic2_irq_chip = {
	.name		= "asic2",
	.irq_ack	= asic2_irq_ack,
	.irq_mask	= asic2_irq_mask,
	.irq_unmask	= asic2_irq_unmask,
	.irq_set_type	= asic2_irq_set_type,
};

static int asic2_irq_domain_map(struct irq_domain *d, unsigned int virq,
				irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(virq, &asic2_irq_chip, handle_level_irq);
	irq_set_chip_data(virq, d->host_data);
	irq_set_probe(virq);

	return 0;
}

static void asic2_irq_domain_unmap(struct irq_domain *d, unsigned int virq)
{
	irq_set_chip_and_handler(virq, NULL, NULL);
	irq_set_chip_data(virq, NULL);
}

static const struct irq_domain_ops asic2_irq_domain_ops = {
	.map	= asic2_irq_domain_map,
	.unmap	= asic2_irq_domain_unmap,
	.xlate	= irq_domain_xlate_onetwocell,
};

/* Both blocks' pending words, narrowed to the bits this domain owns. */
static void asic2_irq_read_pending(struct asic2 *asic, u32 *kpio_flag, u32 *gpio_flag)
{
	*kpio_flag = asic2_read(asic, ASIC2_KPIO_BASE + ASIC2_REG_INT_FLAG) &
		     ASIC2_KPIO_IRQ_ALL;
	*gpio_flag = asic2_read(asic, ASIC2_GPIO_BASE + ASIC2_REG_INT_FLAG) &
		     ASIC2_GPIO2_IRQ_ALL;
}

/*
 * Like asic2_irq_read_pending(), but narrowed again to the sources that are
 * still enabled, so that the late-assert recheck in asic2_irq_handler()
 * never re-enters on a bit the storm guard has just masked. KPIO's
 * InterruptEnable cannot be read back, so its enable state comes from the
 * shadow.
 */
static bool asic2_irq_pending_enabled(struct asic2 *asic, u32 *kpio_flag,
				      u32 *gpio_flag)
{
	unsigned long flags;
	u32 kpio_enable;

	asic2_irq_read_pending(asic, kpio_flag, gpio_flag);

	raw_spin_lock_irqsave(&asic->lock, flags);
	kpio_enable = asic->kpio_int_enable_shadow;
	raw_spin_unlock_irqrestore(&asic->lock, flags);

	*kpio_flag &= kpio_enable & ~READ_ONCE(asic->kpio_storm_masked);
	*gpio_flag &= asic2_read(asic, ASIC2_GPIO_BASE + ASIC2_REG_INT_ENABLE) &
		      ~READ_ONCE(asic->gpio2_storm_masked);

	return *kpio_flag || *gpio_flag;
}

/*
 * The drain loop's view: every pending bit except the ones the storm
 * backstop has masked.
 */
static void asic2_irq_read_live(struct asic2 *asic, u32 *kpio_flag, u32 *gpio_flag)
{
	asic2_irq_read_pending(asic, kpio_flag, gpio_flag);

	*kpio_flag &= ~READ_ONCE(asic->kpio_storm_masked);
	*gpio_flag &= ~READ_ONCE(asic->gpio2_storm_masked);
}

/*
 * One demux pass over a pair of already-read pending words: dispatch every
 * mapped hwirq found set. Shared by the chained handler and the resume
 * drain so that the emulated both-edge re-arm cannot be forgotten in one of
 * them. @from_process selects generic_handle_domain_irq_safe() for callers
 * outside hardirq context.
 */
static void asic2_irq_dispatch(struct asic2 *asic, u32 kpio_flag,
			       u32 gpio_flag, bool from_process)
{
	unsigned int hwirq;

	for (hwirq = 0; hwirq < ASIC2_NR_IRQS; hwirq++) {
		const struct asic2_irq_map *m = &asic2_irq_map[hwirq];
		u32 flag = m->block_base == ASIC2_KPIO_BASE ? kpio_flag : gpio_flag;

		if (!(flag & m->mask))
			continue;

		if (from_process)
			generic_handle_domain_irq_safe(asic->domain, hwirq);
		else
			generic_handle_domain_irq(asic->domain, hwirq);

		if (asic->irq_bothedge & BIT(hwirq))
			asic2_irq_flip_edge(asic, m->block_base, m->mask);
	}
}

/* Count one drain pass: every hwirq pending in it scores one. */
static void asic2_irq_count(u16 *cnt, u32 kpio_flag, u32 gpio_flag)
{
	unsigned int hwirq;

	for (hwirq = 0; hwirq < ASIC2_NR_IRQS; hwirq++) {
		const struct asic2_irq_map *m = &asic2_irq_map[hwirq];
		u32 flag = m->block_base == ASIC2_KPIO_BASE ? kpio_flag : gpio_flag;

		if (flag & m->mask)
			cnt[hwirq]++;
	}
}

/*
 * One drain round in asic2_irq_handler() hit ASIC2_MAX_IRQ_LOOPS with
 * something still pending. @cnt holds, per hwirq, the number of that
 * round's passes it was pending in. Returns true when it masked a
 * persistent set, in which case the caller runs one more round with @final
 * set.
 *
 * The parent is a PXA GPIO interrupt, which is edge-only, and ASIC2_INT
 * stays asserted while any enabled source is pending. Returning with a
 * source still asserted would mean no further edge ever arrives and the
 * whole domain is lost silently, so mask and acknowledge. Never a whole
 * block, so that unrelated working sources keep running. Masking the
 * union of everything seen in the round has the same flaw on a smaller
 * scale: a source that fired once inside the window is retired along with
 * the stuck one, and its consumer never hears from it again. On h3800 a
 * stuck UART0 took the ADC conversion-complete interrupt with it, and
 * every later conversion timed out. So only the sources pending in at
 * least ASIC2_STORM_PERSIST_PASSES of the round's passes are masked.
 * Counts, not the intersection of the per-pass sets: two sources that
 * alternate pass by pass have an empty intersection although both storm,
 * and counted they score half the bound each. The union is the fallback,
 * used when no source reaches the threshold or when the round after a
 * persistent-set mask hits the bound too (@final): something is still
 * stuck, and returning with ASIC2_INT asserted would retire the whole
 * domain.
 *
 * Whether ASIC2_REG_INT_FLAG latches independently of ASIC2_REG_INT_ENABLE
 * is not established (the header documents it only as "R: pending
 * status"), so a masked source may keep showing in the flag register;
 * storm_masked removes it from the drain loop's view either way, and
 * asic2_irq_unmask() puts it back.
 *
 * KPIO's InterruptEnable is write-only, so it is masked through its shadow;
 * GPIO2's can be read and is updated with a read-modify-write. Both happen
 * under asic->lock (the handler holds no lock here), in the same hold that
 * records the bits in storm_masked: hardware mask first, then the bit, the
 * reverse of asic2_irq_unmask(). Each masked source is acked after its
 * mask, before the round ends and so before chained_irq_exit() re-arms the
 * edge-only parent.
 */
static bool asic2_irq_overrun(struct asic2 *asic, const u16 *cnt, bool final)
{
	unsigned int hwirq, p_min = UINT_MAX, all_min = UINT_MAX, rest_max = 0;
	u32 kpio_seen = 0, gpio_seen = 0, kpio_mask = 0, gpio_mask = 0;
	unsigned long flags;
	bool fallback;
	u32 en;

	for (hwirq = 0; hwirq < ASIC2_NR_IRQS; hwirq++) {
		const struct asic2_irq_map *m = &asic2_irq_map[hwirq];
		bool kpio = m->block_base == ASIC2_KPIO_BASE;

		if (!cnt[hwirq])
			continue;

		if (kpio)
			kpio_seen |= m->mask;
		else
			gpio_seen |= m->mask;
		all_min = min_t(unsigned int, all_min, cnt[hwirq]);

		if (!final && cnt[hwirq] >= ASIC2_STORM_PERSIST_PASSES) {
			if (kpio)
				kpio_mask |= m->mask;
			else
				gpio_mask |= m->mask;
			p_min = min_t(unsigned int, p_min, cnt[hwirq]);
		} else {
			rest_max = max_t(unsigned int, rest_max, cnt[hwirq]);
		}
	}

	fallback = !kpio_mask && !gpio_mask;
	if (fallback) {
		kpio_mask = kpio_seen;
		gpio_mask = gpio_seen;
		p_min = all_min;
		rest_max = 0;
	}

	raw_spin_lock_irqsave(&asic->lock, flags);
	if (kpio_mask) {
		asic->kpio_int_enable_shadow &= ~kpio_mask;
		asic2_write(asic, ASIC2_KPIO_BASE + ASIC2_REG_INT_ENABLE,
			    asic->kpio_int_enable_shadow);
		WRITE_ONCE(asic->kpio_storm_masked,
			   asic->kpio_storm_masked | kpio_mask);
	}
	if (gpio_mask) {
		en = asic2_read(asic, ASIC2_GPIO_BASE + ASIC2_REG_INT_ENABLE);
		asic2_write(asic, ASIC2_GPIO_BASE + ASIC2_REG_INT_ENABLE,
			    en & ~gpio_mask);
		WRITE_ONCE(asic->gpio2_storm_masked,
			   asic->gpio2_storm_masked | gpio_mask);
	}
	raw_spin_unlock_irqrestore(&asic->lock, flags);

	if (kpio_mask)
		asic2_write(asic, ASIC2_KPIO_BASE + ASIC2_REG_INT_CLEAR, kpio_mask);
	if (gpio_mask)
		asic2_write(asic, ASIC2_GPIO_BASE + ASIC2_REG_INT_CLEAR, gpio_mask);

	dev_err_ratelimited(asic->dev,
			    "interrupt storm after %u passes: masking KPIO %#x GPIO2 %#x, sparing KPIO %#x GPIO2 %#x; counts masked>=%u spared<=%u%s\n",
			    ASIC2_MAX_IRQ_LOOPS, kpio_mask, gpio_mask,
			    kpio_seen & ~kpio_mask, gpio_seen & ~gpio_mask,
			    p_min, rest_max, fallback ? " [fallback]" : "");

	return !fallback;
}

/*
 * One bounded drain round. Returns true only when it hit the bound and
 * masked a persistent set, i.e. when one more round should run.
 */
static bool asic2_irq_drain_round(struct asic2 *asic, bool final)
{
	u16 cnt[ASIC2_NR_IRQS] = { };
	unsigned int loops;
	u32 kpio_flag, gpio_flag;

	for (loops = 0; loops < ASIC2_MAX_IRQ_LOOPS; loops++) {
		asic2_irq_read_live(asic, &kpio_flag, &gpio_flag);

		if (!kpio_flag && !gpio_flag)
			break;

		asic2_irq_count(cnt, kpio_flag, gpio_flag);
		asic2_irq_dispatch(asic, kpio_flag, gpio_flag, false);
	}

	if (likely(loops < ASIC2_MAX_IRQ_LOOPS))
		return false;

	return asic2_irq_overrun(asic, cnt, final);
}

static void asic2_irq_handler(struct irq_desc *desc)
{
	struct asic2 *asic = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int recheck;
	u32 kpio_flag, gpio_flag;

	for (recheck = 0; recheck < ASIC2_MAX_IRQ_RECHECKS; recheck++) {
		chained_irq_enter(chip, desc);

		/*
		 * At most two rounds: the second runs only after the first
		 * masked a persistent set, to drain whatever the masked
		 * storm was hiding, and normally finds nothing pending.
		 */
		if (asic2_irq_drain_round(asic, false))
			asic2_irq_drain_round(asic, true);

		chained_irq_exit(chip, desc);

		/*
		 * chained_irq_exit() re-enables edge detection on the parent
		 * GPIO, which chained_irq_enter() cleared. A source that
		 * asserted after the last all-zero flag read and before that
		 * re-enable raises ASIC2_INT with no edge detector armed, and
		 * as the parent is edge-only and the line stays asserted, no
		 * later edge arrives and the whole domain is lost silently.
		 * Look again and, if something is already pending, go round
		 * once more.
		 */
		if (!asic2_irq_pending_enabled(asic, &kpio_flag, &gpio_flag))
			return;

		dev_warn_ratelimited(asic->dev,
				     "late assert: KPIO %#x GPIO2 %#x pending after chained_irq_exit(), re-draining (%u/%u)\n",
				     kpio_flag, gpio_flag, recheck + 1,
				     ASIC2_MAX_IRQ_RECHECKS);
	}

	/*
	 * Still pending after the last retry. Only report it: which source to
	 * retire is the storm guard's decision, and this path has no evidence
	 * to make it.
	 */
	dev_err_ratelimited(asic->dev,
			    "late asserts persisted past %u rechecks, ASIC2_INT may be stuck\n",
			    ASIC2_MAX_IRQ_RECHECKS);
}

/*
 * Put the 12 keypad bits of the KPIO block into a known state at probe time,
 * instead of leaving them at the reset default until a consumer sets a
 * trigger type: inputs, level triggered, with the level armed opposite to
 * each pin's current one so that no interrupt is pending before anything
 * listens. A later irq_set_type() on ASIC2_IRQ_KEY reprograms these bits
 * anyway. The Alternate register is left alone, as its bits are not
 * documented.
 */
static void asic2_kpio_keypad_baseline_init(struct asic2 *asic)
{
	u32 data;

	/* Direction is 1 = input; the other KPIO bits belong to other blocks */
	asic2_rmw(asic, ASIC2_KPIO_BASE + ASIC2_REG_DIRECTION, ASIC2_KPIO_KEY_ALL,
		  ASIC2_KPIO_KEY_ALL);

	/* Level-triggered (INT_TYPE bit clear), not edge. */
	asic2_rmw(asic, ASIC2_KPIO_BASE + ASIC2_REG_INT_TYPE, ASIC2_KPIO_KEY_ALL, 0);

	data = asic2_read(asic, ASIC2_KPIO_BASE + ASIC2_REG_DATA);
	asic2_rmw(asic, ASIC2_KPIO_BASE + ASIC2_REG_INT_LEVEL, ASIC2_KPIO_KEY_ALL,
		  ~data & ASIC2_KPIO_KEY_ALL);
}

static void asic2_irq_teardown(void *data)
{
	struct asic2 *asic = data;

	irq_set_chained_handler_and_data(asic->irq, NULL, NULL);
}

static int asic2_irq_init(struct asic2 *asic)
{
	unsigned int trigger;
	const struct irq_domain_info info = {
		.fwnode		= dev_fwnode(asic->dev),
		.size		= ASIC2_NR_IRQS,
		.hwirq_max	= ASIC2_NR_IRQS,
		.ops		= &asic2_irq_domain_ops,
		.host_data	= asic,
	};

	/* Mask everything before the chained handler can ever fire.
	 * KPIO via the shadow (already zeroed by devm_kzalloc), GPIO2
	 * via a direct register write.
	 */
	asic2_write(asic, ASIC2_KPIO_BASE + ASIC2_REG_INT_ENABLE, 0);
	asic2_write(asic, ASIC2_GPIO_BASE + ASIC2_REG_INT_ENABLE, 0);
	asic2_write(asic, ASIC2_KPIO_BASE + ASIC2_REG_INT_CLEAR, 0xffffffff);
	asic2_write(asic, ASIC2_GPIO_BASE + ASIC2_REG_INT_CLEAR, 0xffffffff);

	asic2_kpio_keypad_baseline_init(asic);

	asic->domain = devm_irq_domain_instantiate(asic->dev, &info);
	if (IS_ERR(asic->domain))
		return PTR_ERR(asic->domain);

	/*
	 * The upstream line is rising-edge on the hardware. Use the trigger
	 * type supplied by firmware when there is one, and fall back to
	 * rising edge when the line carries none.
	 */
	trigger = irq_get_trigger_type(asic->irq);
	if (trigger == IRQ_TYPE_NONE)
		trigger = IRQ_TYPE_EDGE_RISING;
	irq_set_irq_type(asic->irq, trigger);
	irq_set_chained_handler_and_data(asic->irq, asic2_irq_handler, asic);

	/*
	 * The chained handler dereferences devres-managed memory, the
	 * mapping and the IRQ domain, so unhook it from a devres action
	 * registered after them: devres unwinds in reverse order and the
	 * handler is gone before they are released, on probe failure and on
	 * unbind alike.
	 */
	return devm_add_action_or_reset(asic->dev, asic2_irq_teardown, asic);
}

/* ---------------------------------------------------------------------
 * Clocks (CCF): one gate per Enable-register bit a cell could need. EX0
 * is not one of them; see the header comment.
 * ---------------------------------------------------------------------
 */

static int asic2_clk_gate_enable(struct clk_hw *hw)
{
	struct asic2_clk_gate *gate = container_of(hw, struct asic2_clk_gate, hw);

	asic2_rmw(gate->asic, ASIC2_CLOCK_BASE + ASIC2_CLOCK_ENABLE, gate->mask, gate->bit);

	return 0;
}

static void asic2_clk_gate_disable(struct clk_hw *hw)
{
	struct asic2_clk_gate *gate = container_of(hw, struct asic2_clk_gate, hw);

	asic2_rmw(gate->asic, ASIC2_CLOCK_BASE + ASIC2_CLOCK_ENABLE, gate->mask, 0);
}

static int asic2_clk_gate_is_enabled(struct clk_hw *hw)
{
	struct asic2_clk_gate *gate = container_of(hw, struct asic2_clk_gate, hw);

	return (asic2_read(gate->asic, ASIC2_CLOCK_BASE + ASIC2_CLOCK_ENABLE) &
		gate->mask) == gate->bit;
}

/*
 * The rate of a gate is its own .rate when it has one, EX1 the crystal and
 * OWM the one divided output with a consumer, and otherwise whatever the
 * parent reports. That is right for SD, whose selector is pinned to
 * ASIC2_CLOCK_SD_2, EX1's own 24.576 MHz. ADC, SPI and PWM have neither a
 * parent nor a rate and report 0, "unknown", which is correct for a bare
 * gate that nothing queries.
 */
static unsigned long asic2_clk_gate_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct asic2_clk_gate *gate = container_of(hw, struct asic2_clk_gate, hw);

	return gate->rate ? gate->rate : parent_rate;
}

static const struct clk_ops asic2_clk_gate_ops = {
	.enable		= asic2_clk_gate_enable,
	.disable	= asic2_clk_gate_disable,
	.is_enabled	= asic2_clk_gate_is_enabled,
	.recalc_rate	= asic2_clk_gate_recalc_rate,
};

/*
 * .mask defaults to .bit at registration time when left 0.
 *
 * SD is not critical: on the h39xx the card clock of the ASIC3 SD host is
 * sourced from this chip, so the host has to claim the gate itself and
 * nothing keeps the SD clock, and through its parent the EX1 crystal,
 * running while no host uses it.
 */

/*
 * EX1 runs free (crystal in, no divider, no parent of its own in this chip)
 * at a fixed 24.576 MHz.
 *
 * The peripheral CX taps are integer divisions of that crystal, fixed in
 * silicon with no divider register to program: OWM (CX6) and SPI (CX5) at
 * 24.576 / 6 = 4.096 MHz, PWM (CX7) at /12 = 2.048 MHz and ADC (CX4) at /24
 * = 1.024 MHz. A CX gate therefore has to state its own rate even when its
 * parent is EX1 for the enable: ds1wm_find_divisor() picks 0x8d for
 * 24.576 MHz against 0x88 for 4.096 MHz, so inheriting the crystal rate
 * would clock every 1-Wire time slot six times too short, and a pack would
 * look like it never answers.
 */
#define ASIC2_EX1_RATE		24576000UL
#define ASIC2_OWM_RATE		4096000UL

static const struct {
	const char *name;
	const char *alias;
	u32 bit;
	u32 mask;
	unsigned long flags;
	unsigned long rate;
} asic2_clk_defs[ASIC2_NR_CLK_GATES] = {
	[ASIC2_GATE_ADC]	= { "asic2-adc",	"asic2-adc",	ASIC2_CLOCK_ADC },
	[ASIC2_GATE_SPI]	= { "asic2-spi",	"asic2-spi",	ASIC2_CLOCK_SPI },
	[ASIC2_GATE_EX1]	= { "asic2-ex1",	NULL,		ASIC2_CLOCK_EX1,
				    0, 0, ASIC2_EX1_RATE },
	[ASIC2_GATE_OWM]	= { "asic2-owm",	"ds1wm",	ASIC2_CLOCK_OWM,
				    0, 0, ASIC2_OWM_RATE },
	[ASIC2_GATE_PWM]	= { "asic2-pwm",	"asic2-pwm",	ASIC2_CLOCK_PWM },
	[ASIC2_GATE_UART_0]	= { "asic2-uart0",	NULL,		ASIC2_CLOCK_UART_0 },
	[ASIC2_GATE_UART_1]	= { "asic2-uart1",	NULL,		ASIC2_CLOCK_UART_1 },
	[ASIC2_GATE_EX2]	= { "asic2-ex2",	NULL,		ASIC2_CLOCK_EX2 },
	[ASIC2_GATE_SD]		= { "asic2-sd",		NULL,		ASIC2_CLOCK_SD_2,
				    ASIC2_CLOCK_SD_MASK },
};

static int asic2_clocks_register(struct asic2 *asic)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ASIC2_NR_CLK_GATES; i++) {
		const struct clk_hw *ex1_parent = &asic->gates[ASIC2_GATE_EX1].hw;
		struct clk_init_data init = {
			.name	= asic2_clk_defs[i].name,
			.ops	= &asic2_clk_gate_ops,
			.flags	= asic2_clk_defs[i].flags,
		};

		/*
		 * OWM and SD have a real CCF parent instead of the usual
		 * convention of each consumer acquiring "ex1" as a second
		 * clock. The ds1wm driver takes a single clock and calls
		 * clk_get_rate() on it, so the OWM gate has to pull the
		 * crystal up through its own clk_hw, or the rate is 0 Hz and
		 * the probe fails with no suitable divisor. The link carries
		 * only the enable, the rate is the own rate of OWM, see
		 * ASIC2_OWM_RATE. SD needs the same link: its host takes a
		 * single clock, and its rate selector is useless without the
		 * 24.576 MHz crystal running. EX1 registers first (lower
		 * index), so its clk_hw is valid here.
		 */
		if (i == ASIC2_GATE_OWM || i == ASIC2_GATE_SD) {
			init.parent_hws = &ex1_parent;
			init.num_parents = 1;
		}

		asic->gates[i].asic = asic;
		asic->gates[i].bit = asic2_clk_defs[i].bit;
		asic->gates[i].mask = asic2_clk_defs[i].mask ? : asic2_clk_defs[i].bit;
		asic->gates[i].rate = asic2_clk_defs[i].rate;
		asic->gates[i].hw.init = &init;

		ret = devm_clk_hw_register(asic->dev, &asic->gates[i].hw);
		if (ret)
			return ret;

		if (asic2_clk_defs[i].alias) {
			ret = devm_clk_hw_register_clkdev(asic->dev, &asic->gates[i].hw,
							  NULL, asic2_clk_defs[i].alias);
			if (ret)
				return ret;
		}
	}

	/*
	 * EX1 (24.576 MHz) is the shared crystal feeding ADC, SPI, OWM and
	 * PWM. It is not modelled as a parent of the ADC, SPI and PWM gates,
	 * so each of those consumers acquires it as a second clock named
	 * "ex1" besides its own peripheral gate. OWM is the exception, see
	 * above.
	 */
	ret = devm_clk_hw_register_clkdev(asic->dev, &asic->gates[ASIC2_GATE_EX1].hw,
					  "ex1", "asic2-spi");
	if (ret)
		return ret;
	ret = devm_clk_hw_register_clkdev(asic->dev, &asic->gates[ASIC2_GATE_EX1].hw,
					  "ex1", "asic2-adc");
	if (ret)
		return ret;
	ret = devm_clk_hw_register_clkdev(asic->dev, &asic->gates[ASIC2_GATE_EX1].hw,
					  "ex1", "asic2-pwm");
	if (ret)
		return ret;

	return 0;
}

static const struct ds1wm_driver_data asic2_ds1wm_pdata = {
	.bus_shift		= 2,
	/*
	 * The driver sets the IAS bit of DS1WM_INT_EN only when active_high
	 * is set, and the OWM block of this chip needs INT_EN = 0x13
	 * (ERBF | IAS | EPD) to deliver the presence-detect interrupt.
	 */
	.active_high		= 1,
	.reset_recover_delay	= 1,
};

static const struct resource asic2_ds1wm_resources[] = {
	DEFINE_RES_MEM(ASIC2_OWM_BASE, 0x14),
	DEFINE_RES_IRQ(ASIC2_IRQ_OWM),
};

static const struct resource asic2_spi_resources[] = {
	DEFINE_RES_MEM(ASIC2_SPI_BASE, 0x0c),
	DEFINE_RES_IRQ(ASIC2_IRQ_SPI),
};

static const struct resource asic2_pwm_resources[] = {
	DEFINE_RES_MEM(ASIC2_PWM_0_BASE, ASIC2_PWM_1_BASE - ASIC2_PWM_0_BASE + 0x0c),
};

/*
 * One MEM resource spanning the three LED blocks. The extra 0x10 covers the
 * last register (relative 0x0c) of the third channel.
 */
static const struct resource asic2_leds_resources[] = {
	DEFINE_RES_MEM(ASIC2_LED0_BASE, ASIC2_LED2_BASE - ASIC2_LED0_BASE + 0x10),
};

/*
 * The 12 keypad lines live in the KPIO block's Data register, with one
 * grouped hwirq (ASIC2_IRQ_KEY). They are not gpio-keys: KPIO's bits are
 * not registered as GPIOs, so asic2-keys reads and diffs the Data register
 * directly. The window covers the Data (relative 0x14) and Alternate
 * (relative 0x3c) registers.
 */
static const struct resource asic2_keys_resources[] = {
	DEFINE_RES_MEM(ASIC2_KPIO_BASE, 0x40),
	DEFINE_RES_IRQ(ASIC2_IRQ_KEY),
};

/*
 * Two IRQ resources: ASIC2_IRQ_ADC, the conversion-complete interrupt
 * (platform_get_irq(pdev, 0) in asic2-adc.c), and ASIC2_IRQ_PEN, GPIO2's
 * falling-edge "pen just touched" line (platform_get_irq(pdev, 1)).
 */
static const struct resource asic2_adc_resources[] = {
	DEFINE_RES_MEM(ASIC2_ADC_BASE, 0x0c),
	DEFINE_RES_IRQ(ASIC2_IRQ_ADC),
	DEFINE_RES_IRQ(ASIC2_IRQ_PEN),
};

/*
 * Named indices into asic2_cells[]: asic2_probe() has to reach the
 * "asic2-adc" entry at run time to attach the per-probe touch GPIO pointer
 * as its platform data.
 */
enum {
	ASIC2_CELL_DS1WM,
	ASIC2_CELL_SPI,
	ASIC2_CELL_PWM,
	ASIC2_CELL_KEYS,
	ASIC2_CELL_ADC,
	ASIC2_NR_CELLS,
};

static struct mfd_cell asic2_cells[ASIC2_NR_CELLS] = {
	[ASIC2_CELL_DS1WM] = {
		/*
		 * drivers/w1/masters/ds1wm.c. MMIO and IRQ come from
		 * asic2_ds1wm_resources[]; the of_compatible only gives a
		 * board's "onewire" child node a device to bind to, so the
		 * driver can read its "dallas,*" properties in preference to
		 * asic2_ds1wm_pdata, which stays as the fallback.
		 */
		.name		= "ds1wm",
		.id		= PLATFORM_DEVID_NONE,
		.of_compatible	= "htc,asic2-onewire",
		.platform_data	= &asic2_ds1wm_pdata,
		.pdata_size	= sizeof(asic2_ds1wm_pdata),
		.resources	= asic2_ds1wm_resources,
		.num_resources	= ARRAY_SIZE(asic2_ds1wm_resources),
	},
	[ASIC2_CELL_SPI] = {
		/* drivers/spi/spi-asic2.c */
		.name		= "asic2-spi",
		.id		= PLATFORM_DEVID_NONE,
		.resources	= asic2_spi_resources,
		.num_resources	= ARRAY_SIZE(asic2_spi_resources),
	},
	[ASIC2_CELL_PWM] = {
		/*
		 * drivers/pwm/pwm-asic2.c. The of_compatible gives the cell a
		 * device node when the board has a "pwm" child: of_pwm_get()
		 * resolves the "pwms" phandle of a pwm-backlight consumer
		 * through fwnode_to_pwmchip(), which cannot match a chip with
		 * no of_node, and the driver reads the board's divider and
		 * period from that node.
		 */
		.name		= "asic2-pwm",
		.id		= PLATFORM_DEVID_NONE,
		.of_compatible	= "htc,asic2-pwm",
		.resources	= asic2_pwm_resources,
		.num_resources	= ARRAY_SIZE(asic2_pwm_resources),
	},
	[ASIC2_CELL_KEYS] = {
		/*
		 * drivers/input/keyboard/asic2-keys.c. The MMIO and IRQ come
		 * from asic2_keys_resources[], not from the node. The
		 * of_compatible gives the cell the "keys" child node of the
		 * board, which the driver reads its key table from.
		 */
		.name		= "asic2-keys",
		.id		= PLATFORM_DEVID_NONE,
		.of_compatible	= "htc,asic2-keys",
		.resources	= asic2_keys_resources,
		.num_resources	= ARRAY_SIZE(asic2_keys_resources),
	},
	[ASIC2_CELL_ADC] = {
		/*
		 * drivers/iio/adc/asic2-adc.c: the light sensor, Imin and
		 * battery channels plus the TP_X0/TP_Y1 touchpanel channels.
		 * .platform_data is filled in at run time in asic2_probe().
		 *
		 * The of_compatible gives the ADC a firmware node, which the
		 * driver does not need itself. Consumers that resolve an
		 * io-channels phandle, such as a battery node, need the
		 * provider to have one.
		 */
		.name		= "asic2-adc",
		.id		= PLATFORM_DEVID_NONE,
		.of_compatible	= "htc,asic2-adc",
		.resources	= asic2_adc_resources,
		.num_resources	= ARRAY_SIZE(asic2_adc_resources),
	},
};

/*
 * mainline's generic resistive-adc-touch driver -- no register access or IRQ
 * of its own (asic2-adc.c owns the whole touch state machine and pushes
 * finished samples to it via an IIO callback buffer), so no resources.
 *
 * The cell binds to a board's "touchscreen" child node, whose "io-channels"
 * phandles point into the "adc" child. The driver's probe rejects a device
 * without an "io-channel-names" property, so the cell is only added when the
 * board has such a child. The phandle also turns a probe-ordering race
 * against asic2-adc.c into an ordinary -EPROBE_DEFER.
 */
static const struct mfd_cell asic2_touch_cell = {
	.name		= "resistive-adc-touch",
	.id		= PLATFORM_DEVID_NONE,
	.of_compatible	= "resistive-adc-touch",
};

/*
 * The status LEDs are described by a "led-controller" child, one "led" per
 * channel. The cell is only added when the board has that child, see
 * asic2_add_optional_cell().
 */
static const struct mfd_cell asic2_leds_cell = {
	.name		= "leds-asic2",
	.id		= PLATFORM_DEVID_NONE,
	.of_compatible	= "htc,asic2-leds",
	.resources	= asic2_leds_resources,
	.num_resources	= ARRAY_SIZE(asic2_leds_resources),
};

/*
 * Add a cell only when the node has an available child it can bind to.
 * mfd-core creates a cell whose of_compatible matches no child with only a
 * warning, and the driver of a cell that needs its child would then fail to
 * probe on every boot of a board that has none.
 */
static int asic2_add_optional_cell(struct asic2 *asic, struct resource *mem,
				   const struct mfd_cell *cell)
{
	struct device_node *np;
	bool present;

	np = of_get_compatible_child(asic->dev->of_node, cell->of_compatible);
	if (!np)
		return 0;

	present = of_device_is_available(np);
	of_node_put(np);
	if (!present)
		return 0;

	return devm_mfd_add_devices(asic->dev, 0, cell, 1, mem, 0, asic->domain);
}

/*
 * PXA GPIO interrupts are edge-only and ASIC2_INT stays asserted while any
 * enabled source is pending. If the edge that woke the core is not delivered
 * to asic2_irq_handler(), no further edge can occur and every ASIC2 source
 * is lost. Drain the pending flags once on resume, when interrupts are back
 * on and the hwirqs have been unmasked; if the edge did arrive, the chained
 * handler has already acknowledged it and this finds nothing.
 *
 * The drain is bounded like the chained handler, since a single snapshot
 * would miss a source asserting just after the read and an unbounded
 * re-read of a stuck source would never end. No chip state is saved or
 * restored: what ASIC2 loses across sleep is not known.
 */
static int asic2_resume(struct device *dev)
{
	struct asic2 *asic = dev_get_drvdata(dev);
	unsigned int loops;
	u32 kpio_flag, gpio_flag;

	for (loops = 0; loops < ASIC2_MAX_IRQ_LOOPS; loops++) {
		asic2_irq_read_live(asic, &kpio_flag, &gpio_flag);

		if (!kpio_flag && !gpio_flag)
			break;

		asic2_irq_dispatch(asic, kpio_flag, gpio_flag, true);
	}

	if (unlikely(loops == ASIC2_MAX_IRQ_LOOPS))
		dev_err_ratelimited(dev, "interrupt storm during resume drain\n");

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(asic2_pm_ops, NULL, asic2_resume);

/* ---------------------------------------------------------------------
 * Probe/lifecycle
 * ---------------------------------------------------------------------
 */

static int asic2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct asic2 *asic;
	struct resource *mem;
	int irq, ret;

	asic = devm_kzalloc(dev, sizeof(*asic), GFP_KERNEL);
	if (!asic)
		return -ENOMEM;

	asic->dev = dev;
	raw_spin_lock_init(&asic->lock);
	platform_set_drvdata(pdev, asic);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -EINVAL;

	asic->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(asic->regs))
		return PTR_ERR(asic->regs);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	asic->irq = irq;

	/* EX0 is forced on unconditionally, see the header comment */
	asic2_write(asic, ASIC2_CLOCK_BASE + ASIC2_CLOCK_ENABLE, ASIC2_CLOCK_EX0);

	/*
	 * Start the INTR block's own time base, as the vendor driver does
	 * right after enabling EX0: prescaler 0x0e with the enable bit set.
	 * The periodic timer (TimerSet) is deliberately left alone: its only
	 * outlet is the KPIO timer interrupt bits, which no cell claims, so
	 * nothing could dispatch or acknowledge it.
	 */
	asic2_write(asic, ASIC2_INTR_BASE + ASIC2_INTR_CLOCK_PRESCALE,
		    ASIC2_INTCPS_CPS(0x0e) | ASIC2_INTCPS_SET);

	ret = asic2_irq_init(asic);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set up IRQ domain\n");

	/*
	 * Open the chip-wide interrupt gate, and OWM's own source bit, before
	 * any cell probes: the SPI cell does a synchronous transfer from
	 * inside its probe() and needs its completion interrupt delivered.
	 * OWM's polarity is the opposite of the UART and timer bits (see
	 * asic2.h). It is set permanently since ds1wm has no teardown hook
	 * to clear it from. The UART and timer sources are masked rather than
	 * left as the bootloader left them: no cell claims them, so nothing
	 * could dispatch or acknowledge them if they asserted the line.
	 */
	asic2_rmw(asic, ASIC2_INTR_BASE + ASIC2_INTR_MASK_AND_FLAG,
		  ASIC2_INTMASK_GLOBAL | ASIC2_INTMASK_OWM |
		  ASIC2_INTMASK_UART_0 | ASIC2_INTMASK_UART_1 |
		  ASIC2_INTMASK_TIMER,
		  ASIC2_INTMASK_GLOBAL | ASIC2_INTMASK_OWM |
		  ASIC2_INTMASK_UART_0 | ASIC2_INTMASK_UART_1 |
		  ASIC2_INTMASK_TIMER);

	ret = asic2_gpio_register(asic);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register gpio_chip\n");

	ret = asic2_touch_gpios_register(asic);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request touch GPIOs\n");

	ret = asic2_clocks_register(asic);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register clocks\n");

	/*
	 * Per-probe pointer, not a compile-time initializer -- see
	 * asic2_cells[]'s own ASIC2_CELL_ADC comment. Safe to mutate the
	 * shared static array in place: ASIC2 is singleton hardware on
	 * every board this project supports, this probe() only ever runs
	 * once.
	 */
	asic2_cells[ASIC2_CELL_ADC].platform_data = &asic->touch_gpios;
	asic2_cells[ASIC2_CELL_ADC].pdata_size = sizeof(asic->touch_gpios);

	ret = devm_mfd_add_devices(dev, 0, asic2_cells, ARRAY_SIZE(asic2_cells),
				   mem, 0, asic->domain);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add sub-devices\n");

	ret = asic2_add_optional_cell(asic, mem, &asic2_touch_cell);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to add touchscreen sub-device\n");

	ret = asic2_add_optional_cell(asic, mem, &asic2_leds_cell);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to add LED sub-device\n");

	dev_dbg(dev, "ASIC2 core ready, chained on IRQ %d\n", irq);

	return 0;
}

static const struct of_device_id asic2_of_match[] = {
	{ .compatible = "htc,asic2" },
	{ }
};
MODULE_DEVICE_TABLE(of, asic2_of_match);

static struct platform_driver asic2_driver = {
	.driver = {
		.name		= "asic2",
		.of_match_table	= asic2_of_match,
		.pm		= pm_sleep_ptr(&asic2_pm_ops),
	},
	.probe	= asic2_probe,
};
module_platform_driver(asic2_driver);

MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("Compaq/HTC ASIC2 companion ASIC core driver");
MODULE_LICENSE("GPL");
