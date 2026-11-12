// SPDX-License-Identifier: GPL-2.0-only
/*
 * AT76C503A WLAN power-sequencing rfkill for the HP iPAQ h5000 family.
 *
 * The radio is attached by USB behind the root hub of the SAMCOP OHCI
 * controller and has two power rails, RF_POWER_ON and WLAN_POWER_ON, both
 * SAMCOP GPB outputs given as properties of the devicetree node. They are
 * sequenced here and not in samcop-ohci.c because the controller only
 * needs its clock to work, while whether the radio is powered is a policy
 * that rfkill can switch on its own. To the host controller the radio
 * going away is an ordinary USB disconnect.
 *
 * The sequence is the one of the old 2.4 board driver: both rails low when
 * requested, both high on unblock followed by one settle delay, both low
 * on block. That driver's own comment on raising both asks "both needed?",
 * which was never answered, so both are raised.
 *
 * The settle delay has no register or errata backing and is a module
 * parameter for that reason.
 *
 * Suspend drops the rails. Resume raises them again only if rfkill does
 * not have the radio blocked.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>

static unsigned int h5000_wlan_settle_ms = 250;
module_param_named(powerup_delay_ms, h5000_wlan_settle_ms, uint, 0644);
MODULE_PARM_DESC(powerup_delay_ms,
		 "settle delay in ms after raising the RF/WLAN power rails, transcribed from the 2.4 driver with no register backing (default 250). Takes effect on the next unblock/resume.");

struct h5000_wlan {
	struct gpio_desc *rf_power;
	struct gpio_desc *wlan_power;
	struct rfkill *rfkill;
};

static void h5000_wlan_rails_up(struct h5000_wlan *wlan)
{
	gpiod_set_value_cansleep(wlan->rf_power, 1);
	gpiod_set_value_cansleep(wlan->wlan_power, 1);
	msleep(h5000_wlan_settle_ms);
}

static void h5000_wlan_rails_down(struct h5000_wlan *wlan)
{
	gpiod_set_value_cansleep(wlan->wlan_power, 0);
	gpiod_set_value_cansleep(wlan->rf_power, 0);
}

static int h5000_wlan_set_block(void *data, bool blocked)
{
	struct h5000_wlan *wlan = data;

	if (blocked)
		h5000_wlan_rails_down(wlan);
	else
		h5000_wlan_rails_up(wlan);

	return 0;
}

static const struct rfkill_ops h5000_wlan_rfkill_ops = {
	.set_block = h5000_wlan_set_block,
};

static void h5000_wlan_rfkill_unregister(void *data)
{
	struct rfkill *rfkill = data;

	rfkill_unregister(rfkill);
	rfkill_destroy(rfkill);
}

static int h5000_wlan_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct h5000_wlan *wlan;
	int ret;

	wlan = devm_kzalloc(dev, sizeof(*wlan), GFP_KERNEL);
	if (!wlan)
		return -ENOMEM;

	wlan->rf_power = devm_gpiod_get(dev, "rf-power", GPIOD_OUT_LOW);
	if (IS_ERR(wlan->rf_power))
		return dev_err_probe(dev, PTR_ERR(wlan->rf_power),
				     "failed to get rf-power gpio\n");

	wlan->wlan_power = devm_gpiod_get(dev, "wlan-power", GPIOD_OUT_LOW);
	if (IS_ERR(wlan->wlan_power))
		return dev_err_probe(dev, PTR_ERR(wlan->wlan_power),
				     "failed to get wlan-power gpio\n");

	wlan->rfkill = rfkill_alloc("h5000-wlan", dev, RFKILL_TYPE_WLAN,
				    &h5000_wlan_rfkill_ops, wlan);
	/*
	 * With CONFIG_RFKILL=n the stubs make rfkill_alloc() return
	 * ERR_PTR(-ENODEV) and rfkill_register() return 0 for that pointer,
	 * so a NULL-only check would probe successfully with ->set_block
	 * never called and the radio left unpowered.
	 */
	if (IS_ERR(wlan->rfkill))
		return dev_err_probe(dev, PTR_ERR(wlan->rfkill),
				     "no rfkill core (CONFIG_RFKILL): the radio would never be powered on\n");
	if (!wlan->rfkill)
		return -ENOMEM;

	ret = rfkill_register(wlan->rfkill);
	if (ret) {
		rfkill_destroy(wlan->rfkill);
		return ret;
	}

	ret = devm_add_action_or_reset(dev, h5000_wlan_rfkill_unregister,
				       wlan->rfkill);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, wlan);

	return 0;
}

static int h5000_wlan_suspend(struct device *dev)
{
	struct h5000_wlan *wlan = dev_get_drvdata(dev);

	h5000_wlan_rails_down(wlan);

	return 0;
}

static int h5000_wlan_resume(struct device *dev)
{
	struct h5000_wlan *wlan = dev_get_drvdata(dev);

	if (!rfkill_blocked(wlan->rfkill))
		h5000_wlan_rails_up(wlan);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(h5000_wlan_pm_ops,
				h5000_wlan_suspend, h5000_wlan_resume);

static const struct of_device_id h5000_wlan_of_match[] = {
	{ .compatible = "hp,ipaq-h5000-wlan", },
	{ }
};
MODULE_DEVICE_TABLE(of, h5000_wlan_of_match);

static struct platform_driver h5000_wlan_driver = {
	.probe	= h5000_wlan_probe,
	.driver	= {
		.name		= "h5000-wlan",
		.of_match_table	= h5000_wlan_of_match,
		.pm		= pm_ptr(&h5000_wlan_pm_ops),
	},
};
module_platform_driver(h5000_wlan_driver);

MODULE_DESCRIPTION("AT76C503A WLAN power-sequencing rfkill for the HP iPAQ h5000 family");
MODULE_AUTHOR("Koen Kooi");
MODULE_LICENSE("GPL");
