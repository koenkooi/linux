// SPDX-License-Identifier: GPL-2.0-only
/*
 * LED driver for the Samsung SAMCOP companion ASIC (HP iPAQ h51xx/h54xx/h55xx)
 *
 * SAMCOP has a 5-channel blink-capable LED controller: one 32-bit control
 * register per channel (LEDCON0-4, 4-byte stride) plus a single prescaler
 * register (LEDPS) shared by all five channels. Unlike HAMCOP's, a channel
 * is a single register that is always written in full: enable bits [1:0],
 * 5-bit duty at [8:4] and 5-bit period at [16:12].
 *
 * There is no blink clock select and no RTC gate bit, so the only clock is
 * the LED gate, which also covers LEDCON/LEDPS register access. The gate
 * has no rate-providing parent, so a clock rate of 0 fails the probe.
 *
 * Copyright (C) 2007 Anton Vorontsov <cbou@mail.ru>
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/mfd/samcop.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>

/* LED register offsets, relative to the SAMCOP_LED_BASE cell resource.
 * Channel n's control register sits at a plain 4-byte stride -- no
 * CON1/CON2 split the way HAMCOP or SAMCOP's own GPA GPIO bank need.
 * ipaq-samcop.h: _SAMCOP_LED_LEDCON0..4 (0x00-0x10), _SAMCOP_LED_LEDPS
 * (0x14).
 */
#define SAMCOP_LED_CON(n)		((n) * 0x0004)
#define SAMCOP_LED_PS			0x0014

/* LEDCON bitfields, transcribed from samcop_set_led()'s own con formula
 * (drivers/mfd/samcop_base.c, hackndev/master):
 *   con = duty ? 0x3 | (duty << 4) | (cycle << 12) : 0;
 * Both enable bits are always set together and always cleared together --
 * there is no independently documented meaning for each, so this driver
 * treats them as one opaque "on" field like the source does.
 */
#define SAMCOP_LED_CON_ENABLE		GENMASK(1, 0)
#define SAMCOP_LED_CON_DUTY_SHIFT	4
#define SAMCOP_LED_CON_DUTY_MASK	GENMASK(8, 4)
#define SAMCOP_LED_CON_PERIOD_SHIFT	12
#define SAMCOP_LED_CON_PERIOD_MASK	GENMASK(16, 12)
#define SAMCOP_LED_CON_FIELD_MAX	31	/* 5-bit duty/period fields */

/* Base tick rate the shared LEDPS prescaler is programmed for at probe.
 * Matches leds-hamcop.c's own 8 Hz (125 ms/unit) convention for the same
 * 5-bit duty/period field width, purely so the two companion-chip LED
 * drivers stay the same shape to read side by side -- SAMCOP's hardware
 * does not require this particular rate, unlike HAMCOP's fixed RTC
 * frequency.
 */
#define SAMCOP_LED_TICK_HZ		8

struct samcop_led {
	struct led_classdev cdev;
	struct samcop_leds *leds;
	unsigned int channel;
};

struct samcop_leds {
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;
	struct samcop_led chans[SAMCOP_LED_NR_CHANNELS];
};

static void samcop_leds_set_brightness(struct led_classdev *cdev,
					enum led_brightness brightness)
{
	struct samcop_led *led = container_of(cdev, struct samcop_led, cdev);
	struct samcop_leds *leds = led->leds;
	u32 con = 0;

	/* Full-register write, matching samcop_set_led()'s own convention
	 * (see this file's header comment) -- there is no partial state to
	 * preserve in SAMCOP's single-register-per-channel format. Full
	 * duty == full period is "always on", the same relationship
	 * blink_set() below relies on when on_units == period_units.
	 */
	if (brightness) {
		con = SAMCOP_LED_CON_ENABLE |
		      (SAMCOP_LED_CON_FIELD_MAX << SAMCOP_LED_CON_DUTY_SHIFT) |
		      (SAMCOP_LED_CON_FIELD_MAX << SAMCOP_LED_CON_PERIOD_SHIFT);
	}

	writel_relaxed(con, leds->regs + SAMCOP_LED_CON(led->channel));
}

static int samcop_leds_blink_set(struct led_classdev *cdev,
				  unsigned long *delay_on,
				  unsigned long *delay_off)
{
	struct samcop_led *led = container_of(cdev, struct samcop_led, cdev);
	struct samcop_leds *leds = led->leds;
	unsigned long on_units, period_units;
	u32 con;

	/* delay_on/delay_off are milliseconds; convert to counts of the
	 * shared LEDPS tick (1/SAMCOP_LED_TICK_HZ seconds each) and clamp
	 * to the 5-bit duty/period fields, same shape as
	 * leds-hamcop.c's own hamcop_leds_blink_set().
	 */
	on_units = DIV_ROUND_CLOSEST(*delay_on * SAMCOP_LED_TICK_HZ, 1000);
	period_units = DIV_ROUND_CLOSEST((*delay_on + *delay_off) * SAMCOP_LED_TICK_HZ, 1000);

	if (on_units > SAMCOP_LED_CON_FIELD_MAX)
		on_units = SAMCOP_LED_CON_FIELD_MAX;
	if (period_units > SAMCOP_LED_CON_FIELD_MAX)
		period_units = SAMCOP_LED_CON_FIELD_MAX;
	if (!on_units)
		on_units = 1;
	if (!period_units)
		period_units = 1;
	if (on_units > period_units)
		on_units = period_units;

	con = SAMCOP_LED_CON_ENABLE |
	      (on_units << SAMCOP_LED_CON_DUTY_SHIFT) |
	      (period_units << SAMCOP_LED_CON_PERIOD_SHIFT);

	writel_relaxed(con, leds->regs + SAMCOP_LED_CON(led->channel));

	*delay_on = (on_units * 1000) / SAMCOP_LED_TICK_HZ;
	*delay_off = ((period_units - on_units) * 1000) / SAMCOP_LED_TICK_HZ;

	return 0;
}

static int samcop_leds_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct samcop_leds *leds;
	struct resource *mem;
	unsigned long rate, prescaler;
	unsigned int i;
	int ret;

	leds = devm_kzalloc(dev, sizeof(*leds), GFP_KERNEL);
	if (!leds)
		return -ENOMEM;

	leds->dev = dev;

	/*
	 * platform_get_resource() + devm_ioremap(), not
	 * devm_platform_ioremap_resource() -- the latter reserves the
	 * region exclusively via devm_request_mem_region(), which always
	 * loses to drivers/mfd/samcop.c's own devm_ioremap_resource() of
	 * the entire SAMCOP MMIO window with -EBUSY. Same bug class fixed
	 * for real on physical h2210 hardware for pxa27x/pxa25x pinctrl,
	 * hamcop-sdi (a6f7616fd9a3) and leds-hamcop (198098879172) -- see
	 * drivers/mfd/hamcop.c's own header comment for the general
	 * pattern. SAMCOP_LED_BASE (0xc1000) is a sub-range of that same
	 * already-claimed window.
	 */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -ENODEV;
	leds->regs = devm_ioremap(dev, mem->start, resource_size(mem));
	if (!leds->regs)
		return -ENOMEM;

	/* The one and only clock this block needs -- see this file's header
	 * comment on why no second "rtc"-style alias is requested here.
	 */
	leds->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(leds->clk))
		return dev_err_probe(dev, PTR_ERR(leds->clk), "failed to get clock\n");

	rate = clk_get_rate(leds->clk);
	if (!rate)
		return dev_err_probe(dev, -EINVAL, "clock rate is 0\n");

	/* Shared prescaler, programmed once for all five channels -- see
	 * SAMCOP_LED_TICK_HZ's comment.
	 */
	prescaler = DIV_ROUND_CLOSEST(rate, SAMCOP_LED_TICK_HZ);
	writel_relaxed(prescaler, leds->regs + SAMCOP_LED_PS);

	/* Ensure every channel starts off -- LEDCON's reset value is not
	 * documented in ipaq-samcop.h, unlike HAMCOP's CON0/CON1.
	 */
	for (i = 0; i < SAMCOP_LED_NR_CHANNELS; i++)
		writel_relaxed(0, leds->regs + SAMCOP_LED_CON(i));

	device_for_each_child_node_scoped(dev, child) {
		struct led_init_data init_data = { .fwnode = child };
		struct samcop_led *led;
		u32 chan;

		ret = fwnode_property_read_u32(child, "reg", &chan);
		if (ret || chan >= SAMCOP_LED_NR_CHANNELS)
			return dev_err_probe(dev, -EINVAL,
					     "%pfw: invalid LED channel\n", child);

		led = &leds->chans[chan];
		if (led->leds)
			return dev_err_probe(dev, -EINVAL,
					     "%pfw: LED channel %u described twice\n",
					     child, chan);

		led->leds = leds;
		led->channel = chan;
		led->cdev.brightness_set = samcop_leds_set_brightness;
		led->cdev.blink_set = samcop_leds_blink_set;
		led->cdev.max_brightness = 1;

		ret = devm_led_classdev_register_ext(dev, &led->cdev, &init_data);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to register LED %u\n", chan);
	}

	platform_set_drvdata(pdev, leds);
	return 0;
}

/* No .remove() needed: devm_clk_get_enabled() unwinds prepare/enable
 * state automatically, and every other resource here is devm-managed too.
 */
static const struct of_device_id samcop_leds_of_match[] = {
	{ .compatible = "samsung,samcop-leds" },
	{ }
};
MODULE_DEVICE_TABLE(of, samcop_leds_of_match);

static struct platform_driver samcop_leds_driver = {
	.probe	= samcop_leds_probe,
	.driver = {
		.name	= "leds-samcop",
		.of_match_table = samcop_leds_of_match,
	},
};

module_platform_driver(samcop_leds_driver);

MODULE_AUTHOR("Anton Vorontsov <cbou@mail.ru>");
MODULE_AUTHOR("Koen Kooi <koen@dominion.thruhere.net>");
MODULE_DESCRIPTION("LED driver for Samsung SAMCOP companion ASIC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:leds-samcop");
