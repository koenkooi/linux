/*
 * TI Beaglebone capebus controller - Generic devices
 *
 * Copyright (C) 2012 Pantelis Antoniou <panto@antoniou-consulting.com>
 * Copyright (C) 2012 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <video/da8xx-fb.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/consumer.h>
#include <linux/atomic.h>
#include <linux/clk.h>
#include <asm/barrier.h>
#include <plat/clock.h>
#include <plat/omap_device.h>
#include <linux/clkdev.h>
#include <linux/input/ti_am335x_tsc.h>
#include <linux/platform_data/ti_am335x_adc.h>
#include <linux/mfd/ti_am335x_tscadc.h>

#include <linux/capebus/capebus-bone.h>

int bone_capebus_probe_prolog(struct cape_dev *dev,
		const struct cape_device_id *id)
{
	char boardbuf[33];
	char versionbuf[5];
	const char *board_name;
	const char *version;
	const struct of_device_id *match;
	struct pinctrl *pinctrl;

	/* get the board name (also matches the cntrlboard before checking) */
	board_name = bone_capebus_id_get_field(id, BONE_CAPEBUS_BOARD_NAME,
			boardbuf, sizeof(boardbuf));
	if (board_name == NULL)
		return -ENODEV;

	/* match compatible? */
	match = capebus_of_match_device(dev, "board-name", board_name);
	if (match == NULL)
		return -ENODEV;

	/* get the board version */
	version = bone_capebus_id_get_field(id, BONE_CAPEBUS_VERSION,
			versionbuf, sizeof(versionbuf));
	if (version == NULL)
		return -ENODEV;

	pinctrl = devm_pinctrl_get_select_default(&dev->dev);
	if (IS_ERR(pinctrl))
		dev_warn(&dev->dev,
			"pins are not configured from the driver\n");

	dev_info(&dev->dev, "%s: V=%s '%s'\n", board_name,
			version, match->compatible);

	return 0;
}
EXPORT_SYMBOL(bone_capebus_probe_prolog);

static const struct bone_capebus_generic_device_data gendevs[] = {
	{
		.name	= "leds",
		.of_match = (const struct of_device_id []) {
				{ .compatible = "gpio-leds", }, { },
			},
		.units	 = 0,	/* no limit */
	}, {
		.name	= "tps-bl",
		.of_match = (const struct of_device_id []) {
				{ .compatible = "tps65217-backlight", }, { },
			},
		.units	 = 0,	/* no limit */
	}, {
		.name	= "keys",
		.of_match = (const struct of_device_id []) {
				{ .compatible = "gpio-keys", }, { },
			},
		.units	 = 0,	/* no limit */
	}, {
		.name	= "tscadc",
		.of_match = (const struct of_device_id []) {
				{ .compatible = "ti-tscadc-dt", }, { },
			},
		.units	 = 1,
	}, {
		.name	= "lcdc",
		.of_match = (const struct of_device_id []) {
				{ .compatible = "da8xx-dt", }, { },
			},
		.units	 = 1,
	},{
		.name	= "i2c-dt",
		.of_match = (const struct of_device_id []) {
				{ .compatible = "i2c-dt", }, { },
			},
		.units	 = 0,
	}, {
		.name	= "w1-gpio",
		.of_match = (const struct of_device_id []) {
				{ .compatible = "w1-gpio", }, { },
			},
		.units	 = 0,
	}, {
		.name	= "pwm-backlight",
		.of_match = (const struct of_device_id []) {
				{ .compatible = "pwm-backlight", }, { },
			},
		.units	 = 0,	/* no limit */
	}, {
		.name	= "spi-dt",
		.of_match = (const struct of_device_id []) {
				{ .compatible = "spi-dt", }, { },
			},
		.units	 = 0,	/* no limit */
	}
};

struct bone_capebus_generic_info *
bone_capebus_probe_generic(struct cape_dev *dev,
		const struct cape_device_id *id)
{
	struct bone_capebus_generic_info *info;
	char boardbuf[33];
	char versionbuf[5];
	const char *board_name;
	const char *version;
	struct platform_device *pdev;
	const struct bone_capebus_generic_device_data *dd;
	struct bone_capebus_generic_device_entry *de;
	int i;

	/* get the board name (also matches the cntrlboard before checking) */
	board_name = bone_capebus_id_get_field(id, BONE_CAPEBUS_BOARD_NAME,
			boardbuf, sizeof(boardbuf));
	/* get the board version */
	version = bone_capebus_id_get_field(id, BONE_CAPEBUS_VERSION,
			versionbuf, sizeof(versionbuf));

	/* should never happen, but it doesn't hurt to play it safe */
	if (board_name == NULL || version == NULL)
		return ERR_PTR(-ENODEV);

	info = devm_kzalloc(&dev->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		dev_err(&dev->dev, "Failed to allocate info\n");
		return ERR_PTR(-ENOMEM);
	}
	info->dev = dev;
	INIT_LIST_HEAD(&info->pdev_list);

	/* iterate over the supported devices */
	for (i = 0, dd = gendevs; i < ARRAY_SIZE(gendevs); i++, dd++) {

		pdev = capebus_of_platform_compatible_device_create(dev,
			dd->of_match, dd->name, "version", version);

		/* node not found (mostly harmless) */
		if (IS_ERR(pdev) && PTR_ERR(pdev) == -ENXIO) {
			/* TODO: deal with required nodes */
			continue;
		}

		/* failed to create due to an error; fatal */
		if (IS_ERR_OR_NULL(pdev)) {
			dev_err(&dev->dev, "failed to create device %s\n",
					dd->name);
			goto err_fail;
		}

		de = devm_kzalloc(&dev->dev, sizeof(*de), GFP_KERNEL);
		if (de == NULL) {
			dev_err(&dev->dev, "failed to allocate entry for %s\n",
					dd->name);
			goto err_fail;
		}

		/* add it to the list */
		de->data = dd;
		de->pdev = pdev;
		list_add_tail(&de->node, &info->pdev_list);
	}

	return info;

err_fail:
	bone_capebus_remove_generic(info);
	return NULL;
}
EXPORT_SYMBOL(bone_capebus_probe_generic);

void bone_capebus_remove_generic(struct bone_capebus_generic_info *info)
{
	struct list_head *lh, *lhn;
	struct bone_capebus_generic_device_entry *de;

	if (info == NULL || info->dev == NULL)
		return;

	list_for_each_safe(lh, lhn, &info->pdev_list) {
		de = list_entry(lh, struct bone_capebus_generic_device_entry,
				node);
		list_del(lh);
		platform_device_unregister(de->pdev);
		devm_kfree(&info->dev->dev, de);
	}
	devm_kfree(&info->dev->dev, info);
}
EXPORT_SYMBOL(bone_capebus_remove_generic);
