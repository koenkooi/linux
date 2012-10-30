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
#include <linux/i2c.h>
#include <linux/of_i2c.h>
#include <linux/spi/spi.h>

#include <linux/capebus/capebus-bone.h>

#if defined(CONFIG_FB_DA8XX) || defined(CONFIG_FB_DA8XX_MODULE)

struct da8xx_priv {
	struct da8xx_lcdc_platform_data lcd_pdata;
	struct lcd_ctrl_config lcd_cfg;
	struct display_panel lcd_panel;
	struct platform_device *lcdc_pdev;
	struct omap_hwmod *lcdc_oh;
	struct resource lcdc_res[1];
	int power_dn_gpio;
};

static const struct of_device_id of_da8xx_dt_match[] = {
	{ .compatible = "da8xx-dt", },
	{},
};

static int __devinit da8xx_dt_probe(struct platform_device *pdev)
{
	struct da8xx_priv *priv;
	struct clk *disp_pll;
	struct pinctrl *pinctrl;
	u32 disp_pll_val;
	const char *panel_type;
	int ret = -EINVAL;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL) {
		dev_err(&pdev->dev, "Failed to allocate priv\n");
		return -ENOMEM;
	}
	priv->power_dn_gpio = -1;

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl))
		dev_warn(&pdev->dev,
			"pins are not configured from the driver\n");

	ret = of_property_read_u32(pdev->dev.of_node, "disp-pll", &disp_pll_val);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to read disp-pll property\n");
		return ret;
	}

	ret = of_property_read_string(pdev->dev.of_node, "panel-type", &panel_type);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to read panel-type property\n");
		return ret;
	}

	/* conf_disp_pll(disp_pll); */
	disp_pll = clk_get(NULL, "dpll_disp_ck");
	if (IS_ERR(disp_pll)) {
		dev_err(&pdev->dev, "Cannot clk_get disp_pll\n");
		return PTR_ERR(disp_pll);
	}
	ret = clk_set_rate(disp_pll, disp_pll_val);
	clk_put(disp_pll);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to set disp_pll\n");
		return ret;
	}

	ret = of_get_named_gpio_flags(pdev->dev.of_node, "powerdn-gpio",
			0, NULL);
	if (IS_ERR_VALUE(ret)) {
		dev_info(&pdev->dev, "No power down GPIO\n");
	} else {
		priv->power_dn_gpio = ret;

		ret = devm_gpio_request(&pdev->dev, priv->power_dn_gpio, "bone-dvi-cape:DVI_PDN");
		if (ret != 0) {
			dev_err(&pdev->dev, "Failed to gpio_request\n");
			return ret;
		}

		ret = gpio_direction_output(priv->power_dn_gpio, 1);
		if (ret != 0) {
			dev_err(&pdev->dev, "Failed to set powerdn to 1\n");
			return ret;
		}
	}

	/* display_panel */
	priv->lcd_panel.panel_type	= QVGA;
	priv->lcd_panel.max_bpp		= 16;
	priv->lcd_panel.min_bpp		= 16;
	priv->lcd_panel.panel_shade	= COLOR_ACTIVE;

	/* lcd_ctrl_config */
	priv->lcd_cfg.p_disp_panel	= &priv->lcd_panel;
	priv->lcd_cfg.ac_bias		= 255;
	priv->lcd_cfg.ac_bias_intrpt	= 0;
	priv->lcd_cfg.dma_burst_sz	= 16;
	priv->lcd_cfg.bpp		= 16;
	priv->lcd_cfg.fdd		= 0x80;
	priv->lcd_cfg.tft_alt_mode	= 0;
	priv->lcd_cfg.stn_565_mode	= 0;
	priv->lcd_cfg.mono_8bit_mode	= 0;
	priv->lcd_cfg.invert_line_clock	= 1;
	priv->lcd_cfg.invert_frm_clock	= 1;
	priv->lcd_cfg.sync_edge		= 0;
	priv->lcd_cfg.sync_ctrl		= 1;
	priv->lcd_cfg.raster_order	= 0;

	/* da8xx_lcdc_platform_data */
	strcpy(priv->lcd_pdata.manu_name, "BBToys");
	priv->lcd_pdata.controller_data = &priv->lcd_cfg;
	strcpy(priv->lcd_pdata.type, panel_type);

	priv->lcdc_oh = omap_hwmod_lookup("lcdc");
	if (priv->lcdc_oh == NULL) {
		dev_err(&pdev->dev, "Failed to lookup omap_hwmod lcdc\n");
		return -ENODEV;
	}

	priv->lcdc_pdev = omap_device_build("da8xx_lcdc", 0, priv->lcdc_oh,
			&priv->lcd_pdata,
			sizeof(struct da8xx_lcdc_platform_data),
			NULL, 0, 0);
	if (priv->lcdc_pdev == NULL) {
		dev_err(&pdev->dev, "Failed to build LCDC device\n");
		return -ENODEV;
	}

	dev_info(&pdev->dev, "Registered bone LCDC OK.\n");

	platform_set_drvdata(pdev, priv);

	return 0;
}

static int __devexit da8xx_dt_remove(struct platform_device *pdev)
{
	return -EINVAL;	/* not supporting removal yet */
}

static struct platform_driver da8xx_dt_driver = {
	.probe		= da8xx_dt_probe,
	.remove		= __devexit_p(da8xx_dt_remove),
	.driver		= {
		.name	= "da8xx-dt",
		.owner	= THIS_MODULE,
		.of_match_table = of_da8xx_dt_match,
	},
};

#endif

#if defined(CONFIG_MFD_TI_AM335X_TSCADC) || defined(CONFIG_MFD_TI_AM335X_TSCADC_MODULE)

struct ti_tscadc_priv {
	struct omap_hwmod *tsc_oh;
	struct tsc_data tsc_data;
	struct adc_data adc_data;
	struct mfd_tscadc_board tscadc_data;
	struct platform_device *tscadc_pdev;
};

static const struct of_device_id of_ti_tscadc_dt_match[] = {
	{ .compatible = "ti-tscadc-dt", },
	{},
};

static int __devinit ti_tscadc_dt_probe(struct platform_device *pdev)
{
	struct ti_tscadc_priv *priv;
	struct pinctrl *pinctrl;
	u32 val;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL) {
		dev_err(&pdev->dev, "Failed to allocate priv\n");
		return -ENOMEM;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl))
		dev_warn(&pdev->dev,
			"pins are not configured from the driver\n");

	ret = of_property_read_u32(pdev->dev.of_node, "tsc-wires", &val);
	if (ret != 0) {
		dev_info(&pdev->dev, "no tsc-wires property; disabling TSC\n");
		val = 0;
	}
	priv->tsc_data.wires = val;

	if (priv->tsc_data.wires > 0) {
		ret = of_property_read_u32(pdev->dev.of_node,
				"tsc-x-plate-resistance", &val);
		if (ret != 0) {
			dev_err(&pdev->dev, "Failed to read "
					"tsc-x-plate-resistance property\n");
			return ret;
		}
		priv->tsc_data.x_plate_resistance = val;

		ret = of_property_read_u32(pdev->dev.of_node,
				"tsc-steps", &val);
		if (ret != 0) {
			dev_err(&pdev->dev, "Failed to read "
					"tsc-steps property\n");
			return ret;
		}
		priv->tsc_data.steps_to_configure = val;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "adc-channels", &val);
	if (ret != 0) {
		dev_info(&pdev->dev, "No adc-channels property; disabling adc\n");
		val = 0;
	}
	priv->adc_data.adc_channels = val;

	priv->tscadc_data.tsc_init = &priv->tsc_data;
	priv->tscadc_data.adc_init = &priv->adc_data;

	priv->tsc_oh = omap_hwmod_lookup("adc_tsc");
	if (priv->tsc_oh == NULL) {
		dev_err(&pdev->dev, "Could not lookup HWMOD %s\n", "adc_tsc");
		return -ENODEV;
	}

	priv->tscadc_pdev = omap_device_build("ti_tscadc", -1, priv->tsc_oh,
			&priv->tscadc_data, sizeof(priv->tscadc_data),
			NULL, 0, 0);
	if (priv->tscadc_pdev == NULL) {
		dev_err(&pdev->dev, "Could not create tsc_adc device\n");
		return -ENODEV;
	}

	dev_info(&pdev->dev, "TI tscadc pdev created OK\n");

	platform_set_drvdata(pdev, priv);

	return 0;
}

static int __devexit ti_tscadc_dt_remove(struct platform_device *pdev)
{
	return -EINVAL;	/* not supporting removal yet */
}

static struct platform_driver ti_tscadc_dt_driver = {
	.probe		= ti_tscadc_dt_probe,
	.remove		= __devexit_p(ti_tscadc_dt_remove),
	.driver		= {
		.name	= "ti_tscadc-dt",
		.owner	= THIS_MODULE,
		.of_match_table = of_ti_tscadc_dt_match,
	},
};

#endif

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
#if defined(CONFIG_FB_DA8XX) || defined(CONFIG_FB_DA8XX_MODULE)
	{
		.driver		= &da8xx_dt_driver,
	},
#endif
#if defined(CONFIG_MFD_TI_AM335X_TSCADC) || defined(CONFIG_MFD_TI_AM335X_TSCADC_MODULE)
	{
		.driver		= &ti_tscadc_dt_driver,
	},
#endif
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
