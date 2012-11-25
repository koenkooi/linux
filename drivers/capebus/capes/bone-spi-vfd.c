/*
 * SPI VFD driver
 *
 * Copyright (C) 2012 Matt Ranostay <mranostay@gmail.com>
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
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/consumer.h>
#include <linux/spi/spi.h>
#include <plat/omap_device.h>
#include "bone-spi-vfd.h"

/*
 * VFD screen update functions
 */

static void spi_display_update(struct work_struct *work)
{
 	struct delayed_work *dwork = to_delayed_work(work);
	struct bonespivfd_info *info = container_of(dwork,
				struct bonespivfd_info, vfd_update);
	struct spi_device *spi = info->spi;
	u8 buf[4];
	int retval;
	int i;

	for (i = 0; i < info->max_digits; i++) {
		u32 val = info->buf[i];

		buf[0] = (val >> 24) & 0xff;
		buf[1] = (val >> 16) & 0xff;
		buf[2] = (val >> 8) & 0xff;
		buf[3] = val & 0xff;

		retval = spi_write(spi, &buf, sizeof(buf));
		if (retval) {
			dev_err(&spi->dev, "cannot write vfd data: %x. err. %d\n",
				val, retval);
			return;
		}
	}

	memset(&buf, 0, sizeof(buf));
	retval = spi_write(spi, &buf, sizeof(buf));
	if (retval) {
		dev_err(&spi->dev, "cannot write blanking data. err. %d\n", retval);
		return;
	}

	cancel_delayed_work(&info->vfd_update);
	schedule_delayed_work(&info->vfd_update, msecs_to_jiffies(info->refresh_rate));
}


static inline int is_valid_value(char val)
{
	int i;

	for (i = 0; i < sizeof(nixie_value_array) - 1; i++) {
		if (nixie_value_array[i] == val)
			return i;
	}

	return -EINVAL;
}

static uint32_t char_to_segment(struct bonespivfd_info *info,
				int digit, int idx, int period)
{
	int i;
	uint32_t retval = 0;
	uint32_t val = nixie_segment_values[idx];
	
	if (period) {
		val |= SEG_H;
	}
	val = val & info->digits_mask[digit];

	for (i = 0; i < SEGMENT_COUNT; i++) {
		if (val & (1 << i)) {
			retval |= 1<< info->segments_cache[i];
		}
	}

	retval |= 1<< info->digits_cache[digit];

	return retval;
}

static void segment_to_char(struct bonespivfd_info *info, char *buf, int idx)
{
	uint32_t retval = 0;
	int i;

	for (i = 0; i < SEGMENT_COUNT; i++) {
		int seg = 1<< info->segments_cache[i];
		if (!!(seg & info->buf[idx])) {
			retval |= 1<<i;
		}
	}

	for (i = 0; i < sizeof(nixie_segment_values); i++) {
		/* filter out SEG_H */
		if (nixie_segment_values[i] == (retval & ~SEG_H)) {
			char val = nixie_value_array[i];

			/* space + period corner case */
			if (val != ' ' && !(retval & SEG_H))
				strncat(buf, &val, 1);

			if (retval & SEG_H)
				strcat(buf, ".");
			break;
		}
	}
}

static ssize_t bonespivfd_show_display(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct bonespivfd_info *info = spi_get_drvdata(spi);
	int i;

	for (i = info->max_digits - 1; i >= 0; i--) {
		segment_to_char(info, buf, i);
	}
	strcat(buf, "\n");

	return strlen(buf);
}

static ssize_t bonespivfd_store_display(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct bonespivfd_info *info = spi_get_drvdata(spi);

	int digit = 0;
	int period = 0;
	int val;
	int i;

	memset(info->buf, 0, sizeof(u32) * info->max_digits);

	/*
	 * Cycle right to left from input string
	 */ 
	for (i = count; i >= 0; i--) {
		char chr = buf[i];

		/*
		 * DP are part of digits
		 */
		if (chr == '.') {
			period = 1;
			continue;
		}

		val = is_valid_value(chr);
		if (val < 0)
			continue;

		val = char_to_segment(info, digit, val, period);
		if (val < 0)
			continue;

		info->buf[digit] =  val;
		period = 0;
		digit++;
	}

	/*
	 * corner case with DP leading the other digits
	 */

	if (period) {
		val = is_valid_value('.');
		if (val < 0)
			return count;
		val = char_to_segment(info, digit, val ,0);
		info->buf[digit] = val;
	}
	schedule_delayed_work(&info->vfd_update, 0);

	return count;
}

static DEVICE_ATTR(vfd_display, S_IRUGO | S_IWUSR,
		bonespivfd_show_display, bonespivfd_store_display);


static int bonespivfd_sysfs_register(struct spi_device *spi)
{
	return device_create_file(&spi->dev, &dev_attr_vfd_display);
};

static void bonespivfd_sysfs_unregister(struct spi_device *spi)
{
	device_remove_file(&spi->dev, &dev_attr_vfd_display);
};

static const struct spi_device_id spivfd_device_id[] = {
	{
		.name = "max6921",
		.driver_data = MAX6921_DEVICE,
	}, {
		.name = "max6931",
		.driver_data = MAX6931_DEVICE,
	}, {
		.name = "generic",
		.driver_data = GENERIC_DEVICE,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(spi, spivfd_device_id);

static const struct of_device_id spivfd_of_match[] = {
	{ .compatible = "bone-spi-vfd,max6921", .data = (void *) MAX6921_DEVICE, },
	{ .compatible = "bone-spi-vfd,max6931", .data = (void *) MAX6931_DEVICE, },
	{ .compatible = "bone-spi-vfd,generic", .data = (void *) GENERIC_DEVICE, },
};
MODULE_DEVICE_TABLE(of, spivfd_of_match);

static int bonespivfd_parse_dt(struct spi_device *spi,
				struct bonespivfd_info *info)
{	
	struct device_node *np = spi->dev.of_node;
 	struct property *prop;
	int length;
	u32 value;
	int ret;

	memset(info, 0, sizeof(*info));

	if (!np)
		return -ENODEV;

	prop = of_find_property(np, "digits-idx", &length);
	if (!prop)
		return -EINVAL;
	
	info->max_digits = length / sizeof(u32);
	if (info->max_digits > 0) {
		size_t size = sizeof(*info->digits_cache) * info->max_digits;
		info->digits_cache = devm_kzalloc(&spi->dev, size, GFP_KERNEL);
		if (!info->digits_cache)
			return -ENOMEM;

		ret = of_property_read_u32_array(np, "digits-idx",
						info->digits_cache,
						info->max_digits);
		if (ret < 0)
			return ret;
	}

	prop = of_find_property(np, "segments-idx", &length);
	if (!prop)
		return -EINVAL;
	
	info->max_segments = length / sizeof(u32);

	if (info->max_segments == SEGMENT_COUNT) {
		size_t size = sizeof(*info->segments_cache) * info->max_segments;
		info->segments_cache = devm_kzalloc(&spi->dev, size, GFP_KERNEL);
		if (!info->segments_cache)
			return -ENOMEM;

		ret = of_property_read_u32_array(np, "segments-idx",
						info->segments_cache,
						info->max_segments);
		if (ret < 0)
			return ret;
	} else {
		dev_err(&spi->dev, "invalid number of segments defined!");
		return -EINVAL;
	}

	prop = of_find_property(np, "digits-mask", &length);
	if (!prop)
		return -EINVAL;

	if (info->max_digits == (length / sizeof(u32))) {
		size_t size = sizeof(*info->digits_mask) * info->max_digits;
		info->digits_mask = devm_kzalloc(&spi->dev, size, GFP_KERNEL);
		if (!info->digits_mask)
			return -ENOMEM;

		ret = of_property_read_u32_array(np, "digits-mask",
						info->digits_mask,
						info->max_digits);
		if (ret < 0)
			return ret;
	} else {
		dev_err(&spi->dev, "digits segment mask isn't the same size "
			   	"as segments index");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "refresh-rate", &value) != 0) {
		value = 150;
		dev_warn(&spi->dev, "no refresh-rate set defaulting to '%d'", value);
	}
	info->refresh_rate = value;

	return 0;
}

static int __devinit bonespivfd_probe(struct spi_device *spi)
{
	struct bonespivfd_info *info;
	const struct spi_device_id *spi_id = spi_get_device_id(spi);
	int retval = -ENOMEM;
	u32 *buf;

	if (!spi_id) {
		dev_err(&spi->dev,
			"device id not supported!\n");
		return -EINVAL;
	}

	info = devm_kzalloc(&spi->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		dev_err(&spi->dev,
			"no memory left!\n");
		return -ENOMEM;
	}

	buf = devm_kzalloc(&spi->dev,
		sizeof(u32) * info->max_digits, GFP_KERNEL);
	if (buf == NULL) {
		dev_err(&spi->dev,
			"no memory left for digits buffer!\n");
		retval = -ENOMEM;
		goto err_no_mem;
	}

	retval = bonespivfd_sysfs_register(spi);
	if (retval < 0) {
		dev_err(&spi->dev, "unable to register sysfs\n");
		goto err_no_sysfs;
	}

	retval = bonespivfd_parse_dt(spi, info);
	if (retval < 0) {
		dev_err(&spi->dev, "unable to parse dt\n");
		goto err_no_dt;
	}	

	INIT_DELAYED_WORK(&info->vfd_update, spi_display_update);

	info->spi = spi;
	info->buf = buf;

	spi_set_drvdata(spi, info);
	return 0;

err_no_dt:
	bonespivfd_sysfs_unregister(spi);
err_no_sysfs:
	devm_kfree(&spi->dev, buf);
err_no_mem:
	devm_kfree(&spi->dev, info);

	return retval;
}

static int __devexit bonespivfd_remove(struct spi_device *spi)
{
	struct bonespivfd_info *info = spi_get_drvdata(spi);
	
	cancel_delayed_work_sync(&info->vfd_update);
	bonespivfd_sysfs_unregister(spi);

	spi_set_drvdata(spi, NULL);

	return 0;
}

static struct spi_driver bonespivfd_driver = {
	.id_table = spivfd_device_id,
	.driver = {
		.name   = "bone-spi-vfd",
		.owner  = THIS_MODULE,
		.of_match_table = spivfd_of_match,
	},
	.probe  = bonespivfd_probe,
	.remove = __devexit_p(bonespivfd_remove),
};

static int __init bonespivfd_init(void)
{
	return spi_register_driver(&bonespivfd_driver);
}

static void __exit bonespivfd_exit(void)
{
	spi_unregister_driver(&bonespivfd_driver);
}

/* ------------------------------------------------------------------------- */


module_init(bonespivfd_init);
module_exit(bonespivfd_exit);

MODULE_AUTHOR("Matt Ranostay");
MODULE_DESCRIPTION("VFD display driver");
MODULE_LICENSE("GPL");
