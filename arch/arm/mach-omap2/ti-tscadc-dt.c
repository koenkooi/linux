/*
 * TI-TSCADC-DT: Device tree adapter using the legacy driver
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
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>
#include <linux/input/ti_am335x_tsc.h>
#include <linux/platform_data/ti_am335x_adc.h>
#include <linux/mfd/ti_am335x_tscadc.h>
#include <plat/clock.h>
#include <plat/omap_device.h>

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
		dev_info(&pdev->dev, "No adc-channels property; "
				"disabling adc\n");
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

static __init int ti_tscadc_dt_init(void)
{
	return platform_driver_register(&ti_tscadc_dt_driver);
}

static __exit void ti_tscadc_dt_exit(void)
{
	platform_driver_unregister(&ti_tscadc_dt_driver);
}

postcore_initcall(ti_tscadc_dt_init);
module_exit(ti_tscadc_dt_exit);
