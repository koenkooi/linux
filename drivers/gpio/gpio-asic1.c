// SPDX-License-Identifier: GPL-2.0-only
/*
 * HTC ASIC1 GPIO block driver (HP/Compaq iPAQ h38xx family)
 *
 * ASIC1 has no chip select of its own: the SoC reaches it through a
 * sub-range of ASIC2's register window and ASIC2 forwards the bus cycles,
 * so this driver is instantiated as an mfd_cell by drivers/mfd/asic2.c
 * and never claims its window exclusively (see the probe comment).
 *
 * The block is 16-bit -- every accessor in the reference implementation,
 * the h3600_asic.h header of the handhelds.org 2.4 kernel for these
 * machines ("the 2.4 kernel" below), is u16 -- not ASIC2's own flat 32-bit
 * stride. All 15 lines are power-rail or routing outputs on every known
 * board, and the Out register reads back: the 2.4 kernel read-modify-writes
 * it at every rail site with no software shadow. The block's own interrupt
 * controller (Mask, TriggerType/EdgeTrigger/LevelTrigger, and the two
 * independent write-1-to-clear LevelStatus/EdgeStatus registers) has no
 * output routed to the SoC on any board, so it is left masked and
 * unmodelled.
 *
 * Copyright 2001 Compaq Computer Corporation.
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/generic.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>

/* Register offsets within the block, with the 2.4 kernel's comments. */
#define ASIC1_GPIO_MASK			0x00	/* 0: don't mask, 1: mask interrupt */
#define ASIC1_GPIO_DIRECTION		0x04	/* 0: input, 1: output */
#define ASIC1_GPIO_OUT			0x08	/* 0: low, 1: high */
#define ASIC1_GPIO_SLEEP_MASK		0x28	/* trigger mask in sleep mode */
#define ASIC1_GPIO_SLEEP_DIR		0x2c	/* direction in sleep mode */
#define ASIC1_GPIO_SLEEP_OUT		0x30	/* level in sleep mode */
#define ASIC1_GPIO_STATUS		0x34	/* R: pin status */
#define ASIC1_GPIO_BATT_FAULT_DIR	0x38	/* direction on battery fault */
#define ASIC1_GPIO_BATT_FAULT_OUT	0x3c	/* level on battery fault */

#define ASIC1_GPIO_NR			15
#define ASIC1_GPIO_ALL			GENMASK(ASIC1_GPIO_NR - 1, 0)

/* Headphone amplifier enable, active low. */
#define ASIC1_GPIO_EAR_ON_N		BIT(13)

struct asic1_gpio {
	struct gpio_generic_chip chip;
	void __iomem *regs;
	u16 out_saved;
};

static int asic1_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_generic_chip_config config;
	struct asic1_gpio *asic1;
	struct resource *res;
	int ret;

	asic1 = devm_kzalloc(dev, sizeof(*asic1), GFP_KERNEL);
	if (!asic1)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	/*
	 * platform_get_resource() + devm_ioremap(), not
	 * devm_ioremap_resource(): this cell's window is a sub-range of the
	 * ASIC2 window, which drivers/mfd/asic2.c maps without claiming it,
	 * and the cells inside that window map their ranges the same way,
	 * as drivers/w1/masters/ds1wm.c does for its own cell.
	 */
	asic1->regs = devm_ioremap(dev, res->start, resource_size(res));
	if (!asic1->regs)
		return -ENOMEM;

	/*
	 * One-time block programming, the reference implementation's own
	 * board-init values (h3800_map_io() of the 2.4 kernel): the unused
	 * interrupt controller fully masked, and every line driven as an
	 * output -- also in sleep mode and on a battery fault, low except
	 * EAR_ON_N.
	 * Every line is a rail enable and the block's power-on state is
	 * undocumented, so a line left as an input is a floating enable.
	 * Written before the generic chip reads the registers back, so its
	 * direction shadow starts from what is actually programmed. Out is
	 * deliberately not written: the levels the bootloader left (console
	 * transceiver, frontlight) are the ones the consumers of those
	 * lines expect to find, and the generic chip reads it back.
	 *
	 * The 2.4 kernel never rewrites any of these on resume ("it doesn't
	 * appear that ASIC1 GPIO has the same problem" as ASIC2's own GPIO
	 * bank); only Out is restored there, see asic1_gpio_resume().
	 */
	writew(ASIC1_GPIO_ALL, asic1->regs + ASIC1_GPIO_MASK);
	writew(ASIC1_GPIO_ALL, asic1->regs + ASIC1_GPIO_SLEEP_MASK);
	writew(ASIC1_GPIO_ALL, asic1->regs + ASIC1_GPIO_SLEEP_DIR);
	writew(ASIC1_GPIO_EAR_ON_N, asic1->regs + ASIC1_GPIO_SLEEP_OUT);
	writew(ASIC1_GPIO_ALL, asic1->regs + ASIC1_GPIO_BATT_FAULT_DIR);
	writew(ASIC1_GPIO_EAR_ON_N, asic1->regs + ASIC1_GPIO_BATT_FAULT_OUT);
	writew(ASIC1_GPIO_ALL, asic1->regs + ASIC1_GPIO_DIRECTION);

	/*
	 * Direction is 1 = output, the inverse of ASIC2's own GPIO bank, so
	 * it is the generic chip's dirout register, not dirin. Out is the
	 * set register and is read back for the initial shadow (no
	 * GPIO_GENERIC_UNREADABLE_REG_SET); Status is the pin-state
	 * register, and GPIO_GENERIC_READ_OUTPUT_REG_SET makes a get() on
	 * an output line read the Out latch rather than the pin.
	 */
	config = (struct gpio_generic_chip_config) {
		.dev = dev,
		.sz = 2,
		.dat = asic1->regs + ASIC1_GPIO_STATUS,
		.set = asic1->regs + ASIC1_GPIO_OUT,
		.dirout = asic1->regs + ASIC1_GPIO_DIRECTION,
		.flags = GPIO_GENERIC_READ_OUTPUT_REG_SET,
	};

	/* 15 of the 16 register bits are lines. */
	asic1->chip.gc.ngpio = ASIC1_GPIO_NR;

	ret = gpio_generic_chip_init(&asic1->chip, &config);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to initialize the generic GPIO chip\n");

	asic1->chip.gc.owner = THIS_MODULE;

	platform_set_drvdata(pdev, asic1);

	return devm_gpiochip_add_data(dev, &asic1->chip.gc, asic1);
}

/*
 * Out is snapshotted at suspend and restored verbatim, not reprogrammed
 * from a fixed baseline: whatever was live when suspend was entered
 * (backlight, LCD rails, charger enable, audio routing) is what resume
 * must come back to. The other registers are left alone, matching the 2.4
 * kernel.
 */
static int asic1_gpio_suspend(struct device *dev)
{
	struct asic1_gpio *asic1 = dev_get_drvdata(dev);

	asic1->out_saved = readw(asic1->regs + ASIC1_GPIO_OUT);

	return 0;
}

static int asic1_gpio_resume(struct device *dev)
{
	struct asic1_gpio *asic1 = dev_get_drvdata(dev);

	writew(asic1->out_saved, asic1->regs + ASIC1_GPIO_OUT);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(asic1_gpio_pm_ops,
				asic1_gpio_suspend, asic1_gpio_resume);

static const struct of_device_id asic1_gpio_of_match[] = {
	{ .compatible = "htc,asic1-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, asic1_gpio_of_match);

static struct platform_driver asic1_gpio_driver = {
	.driver = {
		.name		= "asic1-gpio",
		.of_match_table	= asic1_gpio_of_match,
		.pm		= pm_sleep_ptr(&asic1_gpio_pm_ops),
	},
	.probe	= asic1_gpio_probe,
};
module_platform_driver(asic1_gpio_driver);

MODULE_ALIAS("platform:asic1-gpio");
MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("HTC ASIC1 GPIO block driver");
MODULE_LICENSE("GPL");
