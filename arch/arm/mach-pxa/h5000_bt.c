// SPDX-License-Identifier: GPL-2.0-only
/*
 * LMX9814 Bluetooth power-sequencing rfkill for the HP iPAQ h5000 family.
 *
 * A tiny platform driver wrapping rfkill_ops.set_block around the module's
 * control lines, like h2200_bt.c does for the h2200's Zeevo module. The
 * lines are properties of the devicetree node:
 *
 *  - "reset": the module's active-low reset input.
 *
 *  - "env0" and "env1": operating environment strap lines. Both high
 *    selects normal operation; env0 low with env1 high would select
 *    in-system programming. They are driven high on power-on and left
 *    alone on power-off, as the old 2.6 board driver does.
 *
 *  - "power-3v0": enable of the 3.0 V supply, a SAMCOP GPB output.
 *
 *  - "power-2v8": enable of the 2.8 V supply, a PXA GPIO. It is active
 *    high: the factory firmware sets it to power the module on and clears
 *    it to power it off, although the old 2.6 driver named it BT_2V8_N and
 *    drove it low for on.
 *
 * The factory order is to raise the supplies first and only then hold
 * reset for at least 4 ms and release it. Power-off only drops the two
 * supplies.
 *
 * BT_WU (GPIO14, the module's host wakeup output) is not claimed: nothing
 * consumes it yet.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>

struct h5000_bt {
	struct gpio_desc *reset;
	struct gpio_desc *env0;
	struct gpio_desc *env1;
	struct gpio_desc *power_3v0;
	struct gpio_desc *power_2v8;
};

static int h5000_bt_set_block(void *data, bool blocked)
{
	struct h5000_bt *bt = data;

	if (!blocked) {
		gpiod_set_value_cansleep(bt->env0, 1);
		gpiod_set_value_cansleep(bt->env1, 1);
		gpiod_set_value_cansleep(bt->power_3v0, 1);
		gpiod_set_value_cansleep(bt->power_2v8, 1);
		gpiod_set_value_cansleep(bt->reset, 1);	/* assert */
		usleep_range(4000, 5000);
		gpiod_set_value_cansleep(bt->reset, 0);	/* release */
	} else {
		gpiod_set_value_cansleep(bt->power_3v0, 0);
		gpiod_set_value_cansleep(bt->power_2v8, 0);
	}

	return 0;
}

static const struct rfkill_ops h5000_bt_rfkill_ops = {
	.set_block = h5000_bt_set_block,
};

static void h5000_bt_rfkill_unregister(void *data)
{
	struct rfkill *rfkill = data;

	rfkill_unregister(rfkill);
	rfkill_destroy(rfkill);
}

static int h5000_bt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct h5000_bt *bt;
	struct rfkill *rfkill;
	int ret;

	bt = devm_kzalloc(dev, sizeof(*bt), GFP_KERNEL);
	if (!bt)
		return -ENOMEM;

	/*
	 * The initial values hold the module in reset with both supplies
	 * off: "reset" is active low, so GPIOD_OUT_HIGH asserts it.
	 */
	bt->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(bt->reset))
		return dev_err_probe(dev, PTR_ERR(bt->reset),
				     "failed to get reset gpio\n");

	bt->env0 = devm_gpiod_get(dev, "env0", GPIOD_OUT_LOW);
	if (IS_ERR(bt->env0))
		return dev_err_probe(dev, PTR_ERR(bt->env0),
				     "failed to get env0 gpio\n");

	bt->env1 = devm_gpiod_get(dev, "env1", GPIOD_OUT_LOW);
	if (IS_ERR(bt->env1))
		return dev_err_probe(dev, PTR_ERR(bt->env1),
				     "failed to get env1 gpio\n");

	bt->power_3v0 = devm_gpiod_get(dev, "power-3v0", GPIOD_OUT_LOW);
	if (IS_ERR(bt->power_3v0))
		return dev_err_probe(dev, PTR_ERR(bt->power_3v0),
				     "failed to get power-3v0 gpio\n");

	bt->power_2v8 = devm_gpiod_get(dev, "power-2v8", GPIOD_OUT_LOW);
	if (IS_ERR(bt->power_2v8))
		return dev_err_probe(dev, PTR_ERR(bt->power_2v8),
				     "failed to get power-2v8 gpio\n");

	rfkill = rfkill_alloc("h5000-bt", dev, RFKILL_TYPE_BLUETOOTH,
			      &h5000_bt_rfkill_ops, bt);
	/*
	 * With CONFIG_RFKILL=n the stubs make rfkill_alloc() return
	 * ERR_PTR(-ENODEV) and rfkill_register() return 0 for that pointer,
	 * so a NULL-only check would probe successfully with ->set_block
	 * never called and the radio left unpowered.
	 */
	if (IS_ERR(rfkill))
		return dev_err_probe(dev, PTR_ERR(rfkill),
				     "no rfkill core (CONFIG_RFKILL): the radio would never be powered on\n");
	if (!rfkill)
		return -ENOMEM;

	ret = rfkill_register(rfkill);
	if (ret) {
		rfkill_destroy(rfkill);
		return ret;
	}

	return devm_add_action_or_reset(dev, h5000_bt_rfkill_unregister, rfkill);
}

static const struct of_device_id h5000_bt_of_match[] = {
	{ .compatible = "hp,ipaq-h5000-bt" },
	{ }
};

static struct platform_driver h5000_bt_driver = {
	.probe	= h5000_bt_probe,
	.driver	= {
		.name = "h5000-bt",
		.of_match_table = h5000_bt_of_match,
	},
};
builtin_platform_driver(h5000_bt_driver);
