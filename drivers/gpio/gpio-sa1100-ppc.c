// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel SA-1100/SA-1110 Peripheral Pin Controller (PPC) GPIO driver
 *
 * The PPC block lets a handful of SoC pins that are normally driven by
 * their own on-chip peripheral (the LCD controller, Ser1 to Ser4) be used
 * as software-controlled GPIO. A pin's PPDR and PPSR bits act while its
 * peripheral is disabled; a set PPFR bit gives the pin to them even while
 * the peripheral is enabled.
 *
 * Only the Ser1 to Ser3 lines have a PPFR bit of their own, and requesting
 * one of them sets that bit, so a line is taken from its peripheral only
 * when a consumer asks for it, and freeing it puts the bit back. The LCD
 * lines share one PPFR bit and the four Ser4 lines another; setting either
 * would take a whole group from its peripheral, so those lines are left
 * to the peripheral's enable. A store to the whole register window would
 * also hit HSCR2 and MCCR1, two unrelated registers that share the page.
 *
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/generic.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>

/* Register offsets, from the SA-1110 Developer's Manual. */
#define PPC_PPDR	0x00	/* pin direction: 0=input, 1=output */
#define PPC_PPSR	0x04	/* pin state: read level, write to drive it */
#define PPC_PPAR	0x08	/* pin assignment (Ser1/Ser4 reassignment) */
#define PPC_PSDR	0x0c	/* sleep-mode direction */
#define PPC_PPFR	0x10	/* pin flag: 0=peripheral owns the pin, 1=PPC does */

/*
 * The register window stops after PPFR: HSCR2 (+0x28) and MCCR1 (+0x30)
 * live in the same page and must not be claimed by this driver.
 */
#define PPC_REG_WINDOW_SIZE	0x14

/* The pin count of this register file, LDD0-7 up to SFRM. */
#define PPC_NR_GPIO		22

/* TXD1 to RXD3 have a PPFR bit of their own, at the line's position. */
#define PPC_PPFR_OWN_MASK	GENMASK(17, 12)

struct sa1100_ppc {
	struct gpio_generic_chip chip;
	void __iomem *ppfr;
	/* PPFR bits of the requested lines as they were before the request */
	u32 saved_ppfr;
};

static int sa1100_ppc_request(struct gpio_chip *gc, unsigned int offset)
{
	struct sa1100_ppc *ppc = gpiochip_get_data(gc);
	u32 flag = BIT(offset) & PPC_PPFR_OWN_MASK;
	u32 val;

	if (!flag)
		return 0;

	guard(gpio_generic_lock_irqsave)(&ppc->chip);

	val = readl_relaxed(ppc->ppfr);
	ppc->saved_ppfr = (ppc->saved_ppfr & ~flag) | (val & flag);
	writel_relaxed(val | flag, ppc->ppfr);

	return 0;
}

static void sa1100_ppc_free(struct gpio_chip *gc, unsigned int offset)
{
	struct sa1100_ppc *ppc = gpiochip_get_data(gc);
	u32 flag = BIT(offset) & PPC_PPFR_OWN_MASK;
	u32 val;

	if (!flag)
		return;

	guard(gpio_generic_lock_irqsave)(&ppc->chip);

	val = readl_relaxed(ppc->ppfr);
	writel_relaxed((val & ~flag) | (ppc->saved_ppfr & flag), ppc->ppfr);
}

static int sa1100_ppc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_generic_chip_config config;
	struct gpio_generic_chip *chip;
	struct sa1100_ppc *ppc;
	struct resource *res;
	void __iomem *regs;
	int ret;

	ppc = devm_kzalloc(dev, sizeof(*ppc), GFP_KERNEL);
	if (!ppc)
		return -ENOMEM;
	chip = &ppc->chip;

	/*
	 * The window is exclusive to this driver, so the claim is safe as
	 * long as it stays within PPDR..PPFR; check that rather than trust
	 * the devicetree.
	 */
	regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	if (resource_size(res) > PPC_REG_WINDOW_SIZE)
		return dev_err_probe(dev, -EINVAL,
				     "reg %pR extends past PPDR..PPFR\n",
				     res);

	ppc->ppfr = regs + PPC_PPFR;

	config = (struct gpio_generic_chip_config) {
		.dev = dev,
		.sz = 4,
		.dat = regs + PPC_PPSR,
		.set = regs + PPC_PPSR,
		.dirout = regs + PPC_PPDR,
	};

	chip->gc.ngpio = PPC_NR_GPIO;

	ret = gpio_generic_chip_init(chip, &config);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to initialize the generic GPIO chip\n");

	chip->gc.request = sa1100_ppc_request;
	chip->gc.free = sa1100_ppc_free;
	chip->gc.owner = THIS_MODULE;

	return devm_gpiochip_add_data(dev, &chip->gc, ppc);
}

static const struct of_device_id sa1100_ppc_of_match[] = {
	{ .compatible = "intel,sa1110-ppc" },
	{ }
};
MODULE_DEVICE_TABLE(of, sa1100_ppc_of_match);

static struct platform_driver sa1100_ppc_driver = {
	.driver = {
		.name		= "sa1100-ppc",
		.of_match_table	= sa1100_ppc_of_match,
	},
	.probe	= sa1100_ppc_probe,
};
module_platform_driver(sa1100_ppc_driver);

MODULE_ALIAS("platform:sa1100-ppc");
MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("Intel SA-1100/SA-1110 Peripheral Pin Controller GPIO driver");
MODULE_LICENSE("GPL");
