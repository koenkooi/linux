/*
 * TI Beaglebone capebus controller - Platform adapters
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
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/consumer.h>
#include <linux/atomic.h>
#include <linux/clk.h>
#include <asm/barrier.h>
#include <linux/i2c.h>
#include <linux/of_i2c.h>
#include <linux/spi/spi.h>

#include <linux/capebus/capebus-bone.h>

struct i2c_priv {
	struct i2c_adapter *i2c_adapter;
	phandle parent_handle;
};

static const struct of_device_id of_i2c_dt_match[] = {
	{ .compatible = "i2c-dt", },
	{},
};

static int __devinit i2c_dt_probe(struct platform_device *pdev)
{
	struct i2c_priv *priv = NULL;
	int ret = -EINVAL;
	struct device_node *adap_node;
	u32 val;

	if (pdev->dev.of_node == NULL) {
		dev_err(&pdev->dev, "Only support OF case\n");
		return -ENOMEM;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL) {
		dev_err(&pdev->dev, "Failed to allocate priv\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "parent", &val);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to find parent property\n");
		goto err_prop_fail;
	}
	priv->parent_handle = val;

	adap_node = of_find_node_by_phandle(priv->parent_handle);
	if (adap_node == NULL) {
		dev_err(&pdev->dev, "Failed to find i2c adapter node\n");
		ret = -EINVAL;
		goto err_node_fail;
	}

	ret = capebus_of_platform_device_enable(adap_node);
	if (ret != 0) {
		dev_info(&pdev->dev, "I2C adapter platform device failed "
				"to enable\n");
		goto err_enable_fail;
	}

	priv->i2c_adapter = of_find_i2c_adapter_by_node(adap_node);
	if (priv->i2c_adapter == NULL) {
		dev_err(&pdev->dev, "Failed to find i2c adapter node\n");
		ret = -EINVAL;
		goto err_adap_fail;
	}

	of_i2c_register_node_devices(priv->i2c_adapter, pdev->dev.of_node);

	of_node_put(adap_node);

	dev_info(&pdev->dev, "Registered bone I2C OK.\n");

	platform_set_drvdata(pdev, priv);

	return 0;
err_adap_fail:
	of_node_put(adap_node);
err_enable_fail:
	/* nothing */
err_node_fail:
	/* nothing */
err_prop_fail:
	devm_kfree(&pdev->dev, priv);
	return ret;
}

static int __devexit i2c_dt_remove(struct platform_device *pdev)
{
	return -EINVAL;	/* not supporting removal yet */
}

static struct platform_driver i2c_dt_driver = {
	.probe		= i2c_dt_probe,
	.remove		= __devexit_p(i2c_dt_remove),
	.driver		= {
		.name	= "i2c-dt",
		.owner	= THIS_MODULE,
		.of_match_table = of_i2c_dt_match,
	},
};

struct spi_priv {
	struct spi_master *master;
	phandle parent_handle;
};

static const struct of_device_id of_spi_dt_match[] = {
	{ .compatible = "spi-dt", },
	{},
};

static int of_dev_node_match(struct device *dev, void *data)
{
        return dev->of_node == data;
}

/* must call put_device() when done with returned i2c_adapter device */
static struct spi_master *of_find_spi_master_by_node(struct device_node *node)
{
	struct device *dev;
	struct spi_master *master;

	dev = class_find_device(&spi_master_class, NULL, node,
					 of_dev_node_match);
	if (!dev)
		return NULL;

	master = container_of(dev, struct spi_master, dev);

	/* TODO: No checks what-so-ever... be careful. */
	return master;
}

static int __devinit spi_dt_probe(struct platform_device *pdev)
{
	struct spi_priv *priv = NULL;
	int ret = -EINVAL;
	struct device_node *master_node;
	u32 val;

	if (pdev->dev.of_node == NULL) {
		dev_err(&pdev->dev, "Only support OF case\n");
		return -ENOMEM;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL) {
		dev_err(&pdev->dev, "Failed to allocate priv\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "parent", &val);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to find parent property\n");
		goto err_prop_fail;
	}
	priv->parent_handle = val;

	master_node = of_find_node_by_phandle(priv->parent_handle);
	if (master_node == NULL) {
		dev_err(&pdev->dev, "Failed to find spi bus master node\n");
		ret = -EINVAL;
		goto err_node_fail;
	}

	ret = capebus_of_platform_device_enable(master_node);
	if (ret != 0) {
		dev_info(&pdev->dev, "SPI platform device failed to enable\n");
		goto err_enable_fail;
	}

	priv->master = of_find_spi_master_by_node(master_node);
	if (priv->master == NULL) {
		dev_err(&pdev->dev, "Failed to find bus master node\n");
		ret = -EINVAL;
		goto err_master_fail;
	}

	of_register_node_spi_devices(priv->master, pdev->dev.of_node);

	of_node_put(master_node);

	dev_info(&pdev->dev, "Registered bone SPI OK.\n");

	platform_set_drvdata(pdev, priv);

	return 0;
err_master_fail:
	of_node_put(master_node);
err_enable_fail:
	/* nothing */
err_node_fail:
	/* nothing */
err_prop_fail:
	devm_kfree(&pdev->dev, priv);
	return ret;
}

static int __devexit spi_dt_remove(struct platform_device *pdev)
{
	return -EINVAL;	/* not supporting removal yet */
}

static struct platform_driver spi_dt_driver = {
	.probe		= spi_dt_probe,
	.remove		= __devexit_p(spi_dt_remove),
	.driver		= {
		.name	= "spi-dt",
		.owner	= THIS_MODULE,
		.of_match_table = of_spi_dt_match,
	},
};

/*
 *
 */
struct bone_capebus_pdev_driver {
	struct platform_driver *driver;
	unsigned int registered : 1;
	/* more? */
};

static struct bone_capebus_pdev_driver pdev_drivers[] = {
	{
		.driver		= &i2c_dt_driver,
	},
	{
		.driver		= &spi_dt_driver,
	},
	{
		.driver		= NULL,
	}
};

int bone_capebus_register_pdev_adapters(struct bone_capebus_bus *bus)
{
	struct bone_capebus_pdev_driver *drvp;
	int err;

	/* first check if we do it twice */
	for (drvp = pdev_drivers; drvp->driver != NULL; drvp++)
		if (drvp->registered)
			return -EBUSY;

	for (drvp = pdev_drivers; drvp->driver != NULL; drvp++) {

		err = platform_driver_register(drvp->driver);
		if (err != 0)
			goto err_out;

		drvp->registered = 1;

		dev_info(bus->dev, "Registered %s "
				"platform driver\n", drvp->driver->driver.name);
	}

	return 0;

err_out:
	dev_err(bus->dev, "Failed to register %s "
			"platform driver\n", drvp->driver->driver.name);

	/* unregister */
	while (--drvp >= pdev_drivers) {

		if (!drvp->registered)
			continue;

		platform_driver_unregister(drvp->driver);
	}

	return err;
}

void bone_capebus_unregister_pdev_adapters(struct bone_capebus_bus *bus)
{
	struct bone_capebus_pdev_driver *drvp;

	/* unregister */
	drvp = &pdev_drivers[ARRAY_SIZE(pdev_drivers)];
	while (--drvp >= pdev_drivers) {

		if (drvp->driver == NULL)	/* skip terminator */
			continue;

		if (!drvp->registered)
			continue;

		platform_driver_unregister(drvp->driver);

		drvp->registered = 0;
	}
}
