/*
 * Nixie cape driver
 *
 *  Copyright (C) 2012 Matt Ranostay <mranostay@gmail.com>
 *
 * Based on original work by
 *  Copyright (C) 2012 Pantelis Antoniou <panto@antoniou-consulting.com>
 *  Copyright (C) 2012 Texas Instruments Inc.
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
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/consumer.h>
#include <asm/barrier.h>
#include <plat/clock.h>
#include <plat/omap_device.h>
#include <linux/pwm.h>
#include <plat/omap_device.h>
#include <linux/leds.h>

#include <linux/capebus/capebus-bone.h>

extern struct cape_driver bonenixie_driver;

/* IV-18 tube */
#define DIGIT_COUNT	9

struct bone_nixie_info {
	struct cape_dev *dev;
	struct bone_capebus_generic_info *geninfo;
	struct pwm_device *pwm_dev;
	struct led_trigger *run_led;		/* running */

	int pwm_frequency;
	int pwm_duty_cycle;
	int run;
};

static const struct of_device_id bonenixie_of_match[] = {
	{
		.compatible = "bone-nixie-cape",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, bonenixie_of_match);

static int bonenixie_start(struct cape_dev *dev)
{
	struct bone_nixie_info *info = dev->drv_priv;
	int duty, period;

	if (info->run != 0)
		return 0;

	/* checks */
	if (info->pwm_frequency < 1000 || info->pwm_frequency > 50000) {
		dev_err(&dev->dev, "Cowardly refusing to use a "
				"frequency of %d\n",
				info->pwm_frequency);
		return -EINVAL;
	}
	if (info->pwm_duty_cycle > 80) {
		dev_err(&dev->dev, "Cowardly refusing to use a "
				"duty cycle of %d\n",
				info->pwm_duty_cycle);
		return -EINVAL;
	}

	period = div_u64(1000000000LLU, info->pwm_frequency);
	duty = (period * info->pwm_duty_cycle) / 100;

	dev_info(&dev->dev, "starting nixie tube with "
			"duty=%duns period=%dus\n",
			duty, period);

	pwm_config(info->pwm_dev, duty, period);
	pwm_enable(info->pwm_dev);

	info->run = 1;
	led_trigger_event(info->run_led, LED_FULL);

	return 0;
}

static int bonenixie_stop(struct cape_dev *dev)
{
	struct bone_nixie_info *info = dev->drv_priv;

	if (info->run == 0)
		return 0;

	dev_info(&dev->dev, "disabling nixie tube\n");
	pwm_config(info->pwm_dev, 0, 50000);	/* 0% duty cycle, 20KHz */
	pwm_disable(info->pwm_dev);

	info->run = 0;
	led_trigger_event(info->run_led, LED_OFF);

	return 0;
}

static ssize_t bonenixie_show_run(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct cape_dev *cdev = to_cape_dev(dev);
	struct bone_nixie_info *info = cdev->drv_priv;

	return sprintf(buf, "%d\n", info->run);
}


static ssize_t bonenixie_store_run(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cape_dev *cdev = to_cape_dev(dev);
	int run, err;

	if (sscanf(buf, "%i", &run) != 1)
		return -EINVAL;

	if (run)
		err = bonenixie_start(cdev);
	else
		err = bonenixie_stop(cdev);

	return err ? err : count;
}

static DEVICE_ATTR(run, S_IRUGO | S_IWUSR,
		bonenixie_show_run, bonenixie_store_run);

static int bonenixie_sysfs_register(struct cape_dev *cdev)
{
	int err;

	err = device_create_file(&cdev->dev, &dev_attr_run);
	if (err != 0)
		goto err_no_run;

	return 0;

err_no_run:
	return err;	
}

static void bonenixie_sysfs_unregister(struct cape_dev *cdev)
{
	device_remove_file(&cdev->dev, &dev_attr_run);
}

static int bonenixie_probe(struct cape_dev *dev, const struct cape_device_id *id)
{
	char boardbuf[33];
	char versionbuf[5];
	const char *board_name;
	const char *version;
	struct bone_nixie_info *info;
	struct pinctrl *pinctrl;
	struct device_node *node, *pwm_node;
	phandle phandle;
	u32 val;
	int err;

	/* boiler plate probing */
	err = bone_capebus_probe_prolog(dev, id);
	if (err != 0)
		return err;

	/* get the board name (after check of cntrlboard match) */
	board_name = bone_capebus_id_get_field(id, BONE_CAPEBUS_BOARD_NAME,
			boardbuf, sizeof(boardbuf));
	/* get the board version */
	version = bone_capebus_id_get_field(id, BONE_CAPEBUS_VERSION,
			versionbuf, sizeof(versionbuf));
	/* should never happen; but check anyway */
	if (board_name == NULL || version == NULL)
		return -ENODEV;

	dev->drv_priv = devm_kzalloc(&dev->dev, sizeof(*info), GFP_KERNEL);
	if (dev->drv_priv == NULL) {
		dev_err(&dev->dev, "Failed to allocate info\n");
		err = -ENOMEM;
		goto err_no_mem;
	}
	info = dev->drv_priv;

	pinctrl = devm_pinctrl_get_select_default(&dev->dev);
	if (IS_ERR(pinctrl))
		dev_warn(&dev->dev,
			"pins are not configured from the driver\n");

	node = capebus_of_find_property_node(dev, "version", version, "pwms");
	if (node == NULL) {
		dev_err(&dev->dev, "unable to find pwms property\n");
		err = -ENODEV;
		goto err_no_pwm;
	}

	err = of_property_read_u32(node, "pwms", &val);
	if (err != 0) {
		dev_err(&dev->dev, "unable to read pwm handle\n");
		goto err_no_pwm;
	}
	phandle = val;

	pwm_node = of_find_node_by_phandle(phandle);
	if (pwm_node == NULL) {
		dev_err(&dev->dev, "Failed to pwm node\n");
		err = -EINVAL;
		goto err_no_pwm;
	}

	err = capebus_of_platform_device_enable(pwm_node);
	of_node_put(pwm_node);
	if (err != 0) {
		dev_err(&dev->dev, "Failed to pwm node\n");
		goto err_no_pwm;
	}

	info->pwm_dev = of_pwm_request(node, NULL);
	of_node_put(node);
	if (IS_ERR(info->pwm_dev)) {
		dev_err(&dev->dev, "unable to request PWM\n");
		err = PTR_ERR(info->pwm_dev);
		goto err_no_pwm;
	}

	if (capebus_of_property_read_u32(dev,
				"version", version,
				"pwm-frequency", &val) != 0) {
		val = 9250;
		dev_warn(&dev->dev, "Could not read pwm-frequency property; "
				"using default %u\n",
				val);
	}
	info->pwm_frequency = val;

	if (capebus_of_property_read_u32(dev,
				"version", version,
				"pwm-duty-cycle", &val) != 0) {
		val = 35;
		dev_warn(&dev->dev, "Could not read pwm-duty-cycle property; "
				"using default %u\n",
				val);
	}
	info->pwm_duty_cycle = val;

	err = bonenixie_sysfs_register(dev);
	if (err != 0) {
		dev_err(&dev->dev, "unable to register sysfs\n");
		goto err_no_sysfs;
	}

	led_trigger_register_simple("nixie-run", &info->run_led);

	/* pick up the generics; spi and leds */
	info->geninfo = bone_capebus_probe_generic(dev, id);
	if (info->geninfo == NULL) {
		dev_err(&dev->dev, "Could not probe generic\n");
		goto err_no_generic;
	}

	led_trigger_event(info->run_led, LED_OFF);

	dev_info(&dev->dev, "ready\n");

	err = bonenixie_start(dev);
	if (err != 0) {
		dev_err(&dev->dev, "Could not start nixie device\n");
		goto err_no_start;
	}

	return 0;

err_no_start:
	led_trigger_unregister_simple(info->run_led);
	bone_capebus_remove_generic(info->geninfo);
err_no_generic:
	bonenixie_sysfs_unregister(dev);
err_no_sysfs:
err_no_pwm:
	devm_kfree(&dev->dev, info);
err_no_mem:
	return err;
}

static void bonenixie_remove(struct cape_dev *dev)
{
	struct bone_nixie_info *info = dev->drv_priv;

	dev_info(&dev->dev, "Remove nixie cape driver...\n");

	bonenixie_stop(dev);

	bone_capebus_remove_generic(info->geninfo);
	led_trigger_unregister_simple(info->run_led);
	bonenixie_sysfs_unregister(dev);
}

struct cape_driver bonenixie_driver = {
	.driver = {
		.name		= "bonenixie",
		.owner		= THIS_MODULE,
		.of_match_table = bonenixie_of_match,
	},
	.probe		= bonenixie_probe,
	.remove		= bonenixie_remove,
};

module_capebus_driver(bonenixie_driver);


MODULE_AUTHOR("Matt Ranostay");
MODULE_DESCRIPTION("Beaglebone nixie cape");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bone-nixie-cape");
