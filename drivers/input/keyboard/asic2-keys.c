// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keypad driver for the KPIO block of the ASIC2 companion ASIC (HP iPAQ
 * h38xx/h39xx family).
 *
 * This is not gpio-keys: KPIO's 12 keypad bits are not addressable GPIO
 * lines (they share a register with eight unrelated peripheral interrupt
 * bits, and the whole group is demuxed behind a single hwirq,
 * ASIC2_IRQ_KEY). The driver reads and diffs KPIO's Data register
 * directly.
 *
 * The interrupt is level triggered with "something in the keypad changed"
 * semantics: the handler re-reads the 12-bit Data register, diffs it against
 * the cached state, reports each transition and then re-arms each changed
 * bit's level polarity to the opposite of its new state, so that the next
 * physical transition fires the interrupt again. ASIC2_REG_INT_TYPE stays
 * at level throughout; only the per-bit polarity of ASIC2_REG_INT_LEVEL
 * moves.
 *
 * All 12 lines are active-low direct inputs, not a scanned matrix. Seven
 * map to one physical button each (Record, Left, Right, AP1-4); five
 * belong to the 5-way navigation pad, whose Up, Down and centre positions
 * are each an exact-match combination of three bits of the group in
 * ASIC2_KEYS_ALT_MASK, not one bit apiece. That decode is gated by writing
 * ASIC2_REG_ALTERNATE.
 *
 * The key table lives in the devicetree: the "keys" child node of the
 * ASIC2 node has one "button-*" child per key, with a KPIO Data bitmask, a
 * match value and a "linux,code", decoded as
 * "pressed = ((~Data & mask) == match)". The vendor kernels use a single
 * table for h3800 and h3900. Bits 0 to 11 are, in order, Record, the five
 * 5-way bits, Left, Right and AP1 to AP4.
 *
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/asic2.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>

#define ASIC2_KPIO_DIRECTION	0x00	/* 1=input, 0=output */
#define ASIC2_KPIO_INT_TYPE	0x04	/* 1=edge, 0=level */
#define ASIC2_KPIO_INT_LEVEL	0x0c	/* 1=high, 0=low */
#define ASIC2_KPIO_INT_CLEAR	0x10	/* W: write 1 to clear, never 0 */
#define ASIC2_KPIO_DATA		0x14
#define ASIC2_KPIO_ALTERNATE	0x3c	/* R/W: gates the 5-way pad decode */

#define ASIC2_KEYS_MASK		GENMASK(11, 0)
/* The three-bit combinations of the 5-way pad, independent of the board */
#define ASIC2_KEYS_ALT_MASK	GENMASK(5, 1)

/*
 * One "button-*" child node. keys->pressed is a u32 indexed by position in
 * keys->buttons, which is bounded by ASIC2_KEYS_MAX_BUTTONS.
 */
struct asic2_keys_button {
	u32 mask;
	u32 match;
	unsigned int keycode;
};

#define ASIC2_KEYS_MAX_BUTTONS	16

struct asic2_keys {
	struct device *dev;
	struct device *asic2;	/* the ASIC2 core, owner of the register lock */
	void __iomem *regs;
	struct input_dev *input;
	struct asic2_keys_button *buttons;
	unsigned int nbuttons;
	u32 pressed;	/* decoded state, bit i is keys->buttons[i] */
};

static inline u32 asic2_keys_read(struct asic2_keys *keys, u32 reg)
{
	return readl_relaxed(keys->regs + reg);
}

static inline void asic2_keys_write(struct asic2_keys *keys, u32 reg, u32 val)
{
	writel_relaxed(val, keys->regs + reg);
}

/*
 * Direction, interrupt type, interrupt level and alternate registers are
 * shared with the peripheral interrupt sources (SPI, one-wire, ADC, UART0,
 * ...) whose bits the ASIC2 core programs, so this driver only ever changes
 * its own bits, and does so under the core's lock. A plain write of the
 * whole register would silently reset the other users' bits.
 */
static inline void asic2_keys_update(struct asic2_keys *keys, u32 reg,
				     u32 mask, u32 bits)
{
	asic2_update_bits(keys->asic2, ASIC2_KPIO_BASE + reg, mask, bits);
}

/* Exact-match decode of an (active-low) Data value against the key table */
static u32 asic2_keys_decode(struct asic2_keys *keys, u32 data)
{
	u32 pressed = 0;
	unsigned int i;

	for (i = 0; i < keys->nbuttons; i++) {
		const struct asic2_keys_button *b = &keys->buttons[i];

		if ((~data & b->mask) == b->match)
			pressed |= BIT(i);
	}

	return pressed;
}

/*
 * Re-arm the level polarity of all 12 bits against the Data value just read
 * and acknowledge them all, not only the bits that changed. The vendor
 * interrupt handler does the same, and it keeps a key that is already held
 * across two interrupts from losing its flag, which a changed-bits-only
 * re-arm would do.
 */
static void asic2_keys_rearm(struct asic2_keys *keys, u32 data)
{
	asic2_keys_write(keys, ASIC2_KPIO_INT_CLEAR, ASIC2_KEYS_MASK);
	asic2_keys_update(keys, ASIC2_KPIO_INT_LEVEL, ASIC2_KEYS_MASK,
			  ~data & ASIC2_KEYS_MASK);
}

static irqreturn_t asic2_keys_irq(int irq, void *data)
{
	struct asic2_keys *keys = data;
	u32 now = asic2_keys_read(keys, ASIC2_KPIO_DATA) & ASIC2_KEYS_MASK;
	u32 now_pressed = asic2_keys_decode(keys, now);
	u32 changed = now_pressed ^ keys->pressed;
	unsigned int i;

	for (i = 0; i < keys->nbuttons; i++) {
		if (changed & BIT(i))
			input_report_key(keys->input, keys->buttons[i].keycode,
					 !!(now_pressed & BIT(i)));
	}
	if (changed)
		input_sync(keys->input);

	keys->pressed = now_pressed;

	asic2_keys_rearm(keys, now);

	return IRQ_HANDLED;
}

static int asic2_keys_parse_dt(struct device *dev, struct asic2_keys *keys)
{
	unsigned int n, i = 0;

	n = device_get_child_node_count(dev);
	if (!n)
		return dev_err_probe(dev, -EINVAL, "no \"button-*\" child nodes\n");
	if (n > ASIC2_KEYS_MAX_BUTTONS)
		return dev_err_probe(dev, -EINVAL,
				     "too many key child nodes (%u > %u)\n",
				     n, ASIC2_KEYS_MAX_BUTTONS);

	keys->buttons = devm_kcalloc(dev, n, sizeof(*keys->buttons), GFP_KERNEL);
	if (!keys->buttons)
		return -ENOMEM;

	device_for_each_child_node_scoped(dev, child) {
		struct asic2_keys_button *b = &keys->buttons[i];
		u32 mask, match, code;
		int ret;

		ret = fwnode_property_read_u32(child, "htc,key-mask", &mask);
		if (ret)
			return dev_err_probe(dev, ret,
					     "bad or missing \"htc,key-mask\"\n");

		ret = fwnode_property_read_u32(child, "htc,key-match", &match);
		if (ret)
			return dev_err_probe(dev, ret,
					     "bad or missing \"htc,key-match\"\n");

		ret = fwnode_property_read_u32(child, "linux,code", &code);
		if (ret)
			return dev_err_probe(dev, ret,
					     "bad or missing \"linux,code\"\n");

		/* A match outside the mask could never be satisfied */
		if (!mask || (mask & ~ASIC2_KEYS_MASK) || (match & ~mask))
			return dev_err_probe(dev, -EINVAL,
					     "\"htc,key-mask\" or \"htc,key-match\" out of range\n");

		b->mask = mask;
		b->match = match;
		b->keycode = code;
		i++;
	}

	keys->nbuttons = n;

	return 0;
}

static int asic2_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct asic2_keys *keys;
	struct resource *mem;
	unsigned int i;
	int irq, ret;
	u32 data;

	keys = devm_kzalloc(dev, sizeof(*keys), GFP_KERNEL);
	if (!keys)
		return -ENOMEM;
	keys->dev = dev;
	keys->asic2 = dev->parent;

	ret = asic2_keys_parse_dt(dev, keys);
	if (ret)
		return ret;

	/*
	 * devm_ioremap() rather than devm_platform_ioremap_resource(), which
	 * would reserve the region exclusively: it is a sub-range of the
	 * window the MFD core has already reserved, so the request would
	 * fail with -EBUSY.
	 */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -ENODEV;
	keys->regs = devm_ioremap(dev, mem->start, resource_size(mem));
	if (!keys->regs)
		return -ENOMEM;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	keys->input = devm_input_allocate_device(dev);
	if (!keys->input)
		return -ENOMEM;

	keys->input->name = "asic2-keys";
	keys->input->phys = "asic2-keys/input0";
	keys->input->dev.parent = dev;

	for (i = 0; i < keys->nbuttons; i++)
		input_set_capability(keys->input, EV_KEY, keys->buttons[i].keycode);

	/*
	 * All 12 keypad lines are level-triggered inputs. Arm each bit's
	 * polarity against the Data value read here instead of forcing
	 * "trigger on low" over an unknown baseline, or a key that is already
	 * held at probe time would never drop its flag.
	 */
	asic2_keys_update(keys, ASIC2_KPIO_DIRECTION, ASIC2_KEYS_MASK,
			  ASIC2_KEYS_MASK);
	asic2_keys_update(keys, ASIC2_KPIO_INT_TYPE, ASIC2_KEYS_MASK, 0);

	/* Without this the 5-way pad does not decode at all */
	asic2_keys_update(keys, ASIC2_KPIO_ALTERNATE, ASIC2_KEYS_ALT_MASK,
			  ASIC2_KEYS_ALT_MASK);

	data = asic2_keys_read(keys, ASIC2_KPIO_DATA) & ASIC2_KEYS_MASK;
	keys->pressed = asic2_keys_decode(keys, data);
	asic2_keys_rearm(keys, data);

	ret = devm_request_irq(dev, irq, asic2_keys_irq, 0, "asic2-keys", keys);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	ret = input_register_device(keys->input);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register input device\n");

	platform_set_drvdata(pdev, keys);

	return 0;
}

/* The Alternate register is not preserved across suspend, so arm it again */
static int asic2_keys_resume(struct device *dev)
{
	struct asic2_keys *keys = dev_get_drvdata(dev);
	u32 data;

	asic2_keys_update(keys, ASIC2_KPIO_ALTERNATE, ASIC2_KEYS_ALT_MASK,
			  ASIC2_KEYS_ALT_MASK);

	data = asic2_keys_read(keys, ASIC2_KPIO_DATA) & ASIC2_KEYS_MASK;
	keys->pressed = asic2_keys_decode(keys, data);
	asic2_keys_rearm(keys, data);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(asic2_keys_pm_ops, NULL, asic2_keys_resume);

static struct platform_driver asic2_keys_driver = {
	.driver = {
		.name = "asic2-keys",
		.pm = pm_sleep_ptr(&asic2_keys_pm_ops),
	},
	.probe	= asic2_keys_probe,
};
module_platform_driver(asic2_keys_driver);

MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("ASIC2 companion ASIC keypad (HP iPAQ h38xx/h39xx)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:asic2-keys");
