// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI BRF6150 Bluetooth power-sequencing rfkill for the HP iPAQ hx4700.
 *
 * A tiny platform driver wrapping rfkill_ops.set_block around the module's
 * control lines, like h5000_bt.c does for the h5000 family. The lines are
 * properties of the devicetree node:
 *
 *  - "reset": the module's active-low reset input.
 *
 *  - "power": enable of the module's 3.3 V supply, a line of the nCS5
 *    htc-egpio chip.
 *
 * Power-on raises the supply, holds reset for 1 ms and releases it. The
 * old 2.6 board driver then polled CTS of the BTUART for the module's
 * boot, typically 150 ms and at most 500 ms; the fixed settle below is
 * that upper bound, as a serial port the driver does not own cannot be
 * polled from here. Power-off only drops the supply and leaves reset
 * released, as the old driver does.
 *
 * No wake line is claimed: the module's host wakeup output is an ASIC3
 * input that nothing consumes yet.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>

struct hx4700_bt {
	struct gpio_desc *reset;
	struct gpio_desc *power;
};

static int hx4700_bt_set_block(void *data, bool blocked)
{
	struct hx4700_bt *bt = data;

	if (!blocked) {
		gpiod_set_value_cansleep(bt->power, 1);
		gpiod_set_value_cansleep(bt->reset, 1);	/* assert */
		usleep_range(1000, 2000);
		gpiod_set_value_cansleep(bt->reset, 0);	/* release */
		msleep(500);
	} else {
		gpiod_set_value_cansleep(bt->power, 0);
		usleep_range(1000, 2000);
	}

	return 0;
}

static const struct rfkill_ops hx4700_bt_rfkill_ops = {
	.set_block = hx4700_bt_set_block,
};

static void hx4700_bt_rfkill_unregister(void *data)
{
	struct rfkill *rfkill = data;

	rfkill_unregister(rfkill);
	rfkill_destroy(rfkill);
}

static int hx4700_bt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hx4700_bt *bt;
	struct rfkill *rfkill;
	int ret;

	bt = devm_kzalloc(dev, sizeof(*bt), GFP_KERNEL);
	if (!bt)
		return -ENOMEM;

	/*
	 * The initial values hold the module in reset with the supply off:
	 * "reset" is active low, so GPIOD_OUT_HIGH asserts it.
	 */
	bt->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(bt->reset))
		return dev_err_probe(dev, PTR_ERR(bt->reset),
				     "failed to get reset gpio\n");

	bt->power = devm_gpiod_get(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(bt->power))
		return dev_err_probe(dev, PTR_ERR(bt->power),
				     "failed to get power gpio\n");

	rfkill = rfkill_alloc("hx4700-bt", dev, RFKILL_TYPE_BLUETOOTH,
			      &hx4700_bt_rfkill_ops, bt);
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

	/* A stable name for an LED trigger, instead of "rfkill<N>". */
	rfkill_set_led_trigger_name(rfkill, "hx4700-bt");

	ret = rfkill_register(rfkill);
	if (ret) {
		rfkill_destroy(rfkill);
		return ret;
	}

	return devm_add_action_or_reset(dev, hx4700_bt_rfkill_unregister, rfkill);
}

static const struct of_device_id hx4700_bt_of_match[] = {
	{ .compatible = "hp,ipaq-hx4700-bt" },
	{ }
};

static struct platform_driver hx4700_bt_driver = {
	.probe	= hx4700_bt_probe,
	.driver	= {
		.name = "hx4700-bt",
		.of_match_table = hx4700_bt_of_match,
	},
};
builtin_platform_driver(hx4700_bt_driver);
