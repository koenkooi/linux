// SPDX-License-Identifier: GPL-2.0-only
/*
 * Zeevo Bluetooth power-sequencing rfkill for the HP iPAQ h2200.
 *
 * Structural template: mainline's tosa-bt.c (Dmitry Baryshkov, 2008),
 * deleted alongside the rest of tosa's/h5000's board files in
 * d6df7df7ae5a ("ARM: pxa: remove unused board files", 2022), so it no
 * longer lives anywhere in current mainline; this driver is a
 * from-scratch rewrite in its shape (a tiny platform_driver wrapping
 * rfkill_ops.set_block around two GPIOs), not a copy.
 *
 * The actual power-on/off sequence is instead translated from
 * the old out-of-tree h2200.c:
 * h2200_bluetooth_power() -- h2200's own chip (Zeevo) and GPIO map,
 * which tosa-bt.c never had any knowledge of.
 *
 * Open question: BT_WAKE (GPIO10, "wakeup signal to bluetooth
 * module") is not driven by the power-on/off sequence this driver
 * translates, and what asserts it, and when, is unknown. The
 * sibling h4000_bt.c (Broadcom
 * BRF6xxx on an asic3 companion chip, not Zeevo/hamcop, but the same
 * "separate wake pin alongside reset/power" shape) *does* assert its
 * equivalent line as the last power-on step and the first power-off
 * step, alongside reset -- that precedent is followed here since
 * leaving BT_WAKE permanently deasserted risks the chip never coming
 * out of UART sleep mode. Unverified on real h2200 hardware.
 *
 * The HWUART pins (GPIO42-45) are muxed to their UART function statically
 * at boot (h2200_pin_config[] in h2200.c) instead of being parked as GPIO
 * until this driver powers the chip on. It is not known whether that can
 * phantom-power the Zeevo chip through its RXD/CTS input protection
 * diodes while BT_POWER_ON is low, as no datasheet for it is available.
 * The tosa board did the same for roughly 15 years: tosa_pin_config[]
 * muxed its own GPIO42-45 to BTUART statically while tosa-bt.c switched a
 * separate reset/power pair on a SCOOP expander, the same partitioning
 * used here.
 *
 * BT_IDENT (GPIO75, "0 -> SMART, 1 -> Zeevo module") is read in probe,
 * which declines to bind on a "SMART" reading: the driver only knows the
 * Zeevo sequence and nothing describes the power sequencing of the other
 * module. Whether any unit ships it is unknown; this only stops the
 * Zeevo sequence from running on hardware that reports it is not Zeevo.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>

struct h2200_bt {
	struct gpio_desc *reset;
	struct gpio_desc *power;
	struct gpio_desc *wake;
};

/*
 * Translated from the old out-of-tree h2200_bluetooth_power(): power
 * off first, deassert reset, wait, power on, then the driver's own
 * "XXX This seems too long, but anything shorter makes hcid not work"
 * 2-second settle before the chip is usable -- ported verbatim since
 * there's no way to re-derive a tighter bound without hardware.
 * BT_WAKE handling (see the file comment above) is this driver's own
 * addition, not present in the 2007 original.
 */
static int h2200_bt_set_block(void *data, bool blocked)
{
	struct h2200_bt *bt = data;

	if (!blocked) {
		gpiod_set_value_cansleep(bt->power, 0);
		gpiod_set_value_cansleep(bt->reset, 0);
		mdelay(5);
		gpiod_set_value_cansleep(bt->power, 1);
		msleep(2000);
		gpiod_set_value_cansleep(bt->wake, 1);
	} else {
		gpiod_set_value_cansleep(bt->wake, 0);
		/*
		 * Reset is left deasserted on power-off as well. Whether
		 * the chip wants it asserted here is not known, and the
		 * translated sequence is kept without hardware to test a
		 * change against.
		 */
		gpiod_set_value_cansleep(bt->power, 0);
	}

	return 0;
}

static const struct rfkill_ops h2200_bt_rfkill_ops = {
	.set_block = h2200_bt_set_block,
};

static void h2200_bt_rfkill_unregister(void *data)
{
	struct rfkill *rfkill = data;

	rfkill_unregister(rfkill);
	rfkill_destroy(rfkill);
}

static int h2200_bt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct h2200_bt *bt;
	struct gpio_desc *ident;
	struct rfkill *rfkill;
	int ret;

	bt = devm_kzalloc(dev, sizeof(*bt), GFP_KERNEL);
	if (!bt)
		return -ENOMEM;

	/*
	 * BT_IDENT first, before touching reset/power/wake at all: see the
	 * file comment above. A plain input, so there is no glitch risk in
	 * acquiring it early.
	 */
	ident = devm_gpiod_get(dev, "ident", GPIOD_IN);
	if (IS_ERR(ident))
		return dev_err_probe(dev, PTR_ERR(ident),
				      "failed to get ident gpio\n");

	ret = gpiod_get_value_cansleep(ident);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to read ident gpio\n");
	if (ret == 0) {
		dev_info(dev,
			 "BT_IDENT reads SMART module, not Zeevo -- no driver for this module, not probing\n");
		return -ENODEV;
	}

	/*
	 * Initial values match h2200_pin_config[]'s boot-time DRIVE_LOW
	 * level for GPIO74/76/10 (h2200.c), so acquiring the descriptors
	 * here doesn't glitch either line: "reset" is active-low
	 * (GPIOD_OUT_HIGH -> physical low -> asserted, chip held in
	 * reset), "power"/"device-wakeup" are active-high (GPIOD_OUT_LOW ->
	 * physical low -> off/idle).
	 */
	bt->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(bt->reset))
		return dev_err_probe(dev, PTR_ERR(bt->reset),
				      "failed to get reset gpio\n");

	bt->power = devm_gpiod_get(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(bt->power))
		return dev_err_probe(dev, PTR_ERR(bt->power),
				      "failed to get power gpio\n");

	bt->wake = devm_gpiod_get(dev, "device-wakeup", GPIOD_OUT_LOW);
	if (IS_ERR(bt->wake))
		return dev_err_probe(dev, PTR_ERR(bt->wake),
				      "failed to get wake gpio\n");

	rfkill = rfkill_alloc("h2200-bt", dev, RFKILL_TYPE_BLUETOOTH,
			      &h2200_bt_rfkill_ops, bt);
	/*
	 * With CONFIG_RFKILL=n, the stubs in rfkill.h make rfkill_alloc()
	 * return ERR_PTR(-ENODEV) and rfkill_register() return 0 for that
	 * pointer, so a NULL-only check would probe "successfully" with
	 * ->set_block never called and the radio left unpowered. Not
	 * reachable today as MACH_H2200 selects RFKILL; kept so it does not
	 * depend on that select staying.
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

	return devm_add_action_or_reset(dev, h2200_bt_rfkill_unregister, rfkill);
}

static const struct of_device_id h2200_bt_of_match[] = {
	{ .compatible = "hp,ipaq-h2200-bt" },
	{ }
};

static struct platform_driver h2200_bt_driver = {
	.probe	= h2200_bt_probe,
	.driver	= {
		.name = "h2200-bt",
		.of_match_table = h2200_bt_of_match,
	},
};
builtin_platform_driver(h2200_bt_driver);
