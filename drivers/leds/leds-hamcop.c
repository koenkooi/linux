// SPDX-License-Identifier: GPL-2.0-only
/*
 * LED driver for Samsung HAMCOP companion ASIC (HP iPAQ h2200/h2210/h2212/h2215)
 *
 * HAMCOP has a 5-channel blink-capable LED controller with independent
 * duty-cycle and period timers per channel, clocked from the internal RTC
 * (32.768 KHz). Each channel can be driven at full brightness (INITV_ON with
 * CNTEN_OFF) or set to blink at variable duty/period via the compare and
 * divisor fields.
 *
 * The hardware interface is simple: two 16-bit control registers per channel
 * (LEDnCON0/LEDnCON1) plus global prescaler (LEDPS0/LEDPS1). The channels
 * the board wires are child nodes of the controller's devicetree node, and
 * one led_classdev is registered for each.
 *
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/spinlock.h>
#include <linux/mfd/hamcop.h>

/* LED register offsets, relative to the HAMCOP_LED_BASE of the
 * platform resource. Each channel n has two 16-bit registers at
 * +0x0/+0x4 relative to channel 0 at +0x00, in a 0x08 stride pattern.
 */
#define HAMCOP_LED_CON0(n)		(0x0000 + (n) * 0x0008)
#define HAMCOP_LED_CON1(n)		(0x0004 + (n) * 0x0008)
#define HAMCOP_LED_PS0			0x0028	/* prescaler 0 (bits 0-15) */
#define HAMCOP_LED_PS1			0x002c	/* prescaler 1 (bits 16-26) */

/* CON0 bitfields */
#define HAMCOP_LED_CON0_ENB		BIT(0)	/* Enable */
#define HAMCOP_LED_CON0_INITV		BIT(1)	/* Initial value (1 = on) */
#define HAMCOP_LED_CON0_CMP_SHIFT	4	/* Duty cycle compare value */
#define HAMCOP_LED_CON0_CMP_MASK	(0x1f << HAMCOP_LED_CON0_CMP_SHIFT)
#define HAMCOP_LED_CON0_CNTEN		BIT(9)	/* Counter enable (blink mode) */
#define HAMCOP_LED_CON0_CLK_RTC		(0 << 10)	/* Use RTC clock */
#define HAMCOP_LED_CON0_CLK_GCLK	BIT(10)	/* Use GCLK */

/* CON1 bitfields */
#define HAMCOP_LED_CON1_DIV_SHIFT	11	/* Divisor (period) */
#define HAMCOP_LED_CON1_DIV_MASK	(0x1f << HAMCOP_LED_CON1_DIV_SHIFT)

/* Prescaler value for RTC clock: 4096 = 32768 Hz / (1/8 sec)
 * This gives 8 base-time units per second. The duty and period
 * fields (5-bit values 0-31) then express timing in units of that
 * base time. Compare value = duty (0-31 * base time), divisor =
 * total period (0-31 * base time).
 */
#define HAMCOP_LED_PRESCALER		4096
#define HAMCOP_LED_UNIT_MS		125

struct hamcop_led {
	struct led_classdev cdev;
	struct hamcop_leds *leds;
	unsigned int channel;
};

struct hamcop_leds {
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;
	struct clk *rtc_clk;
	struct hamcop_led chans[HAMCOP_LED_NR_CHANNELS];

	/*
	 * This block has its own ioremap, so the register lock in the MFD core
	 * does not cover it. Serialises the CON0 read-modify-write in
	 * set_brightness() against the CON0/CON1 pair write in blink_set().
	 */
	spinlock_t lock;
};

static void hamcop_leds_set_brightness(struct led_classdev *cdev,
				       enum led_brightness brightness)
{
	struct hamcop_led *led = container_of(cdev, struct hamcop_led, cdev);
	struct hamcop_leds *leds = led->leds;
	unsigned int chan = led->channel;
	unsigned long flags;
	u16 con0;

	/* Read-modify-write the CON0 register: update ENB and INITV bits,
	 * leave everything else (duty/counter/clock bits) intact.
	 * When brightness == 0, disable the LED entirely. Otherwise
	 * enable it and set INITV to turn it on (unless blinking via
	 * a trigger, in which case the trigger owns the duty/period bits).
	 */
	spin_lock_irqsave(&leds->lock, flags);

	con0 = readw_relaxed(leds->regs + HAMCOP_LED_CON0(chan));
	con0 &= ~(HAMCOP_LED_CON0_ENB | HAMCOP_LED_CON0_INITV);

	if (brightness)
		con0 |= HAMCOP_LED_CON0_ENB | HAMCOP_LED_CON0_INITV;

	writew_relaxed(con0, leds->regs + HAMCOP_LED_CON0(chan));

	spin_unlock_irqrestore(&leds->lock, flags);
}

static int hamcop_leds_blink_set(struct led_classdev *cdev,
				 unsigned long *delay_on,
				 unsigned long *delay_off)
{
	struct hamcop_led *led = container_of(cdev, struct hamcop_led, cdev);
	struct hamcop_leds *leds = led->leds;
	unsigned int chan = led->channel;
	unsigned long flags;
	u16 cmp, div, con0, con1;

	/* delay_on and delay_off are milliseconds. Convert to base-time units
	 * (1/8 second = 125 ms per unit), where the CMP field sets the duty
	 * (compare, time LED is on) and DIV field sets the total period
	 * (compare + time LED is off).
	 */
	unsigned long on_units, period_units;

	/* 0/0 asks the driver to pick a default rate. */
	if (!*delay_on && !*delay_off) {
		*delay_on = 500;
		*delay_off = 500;
	}

	on_units = DIV_ROUND_CLOSEST(*delay_on, HAMCOP_LED_UNIT_MS);
	period_units = DIV_ROUND_CLOSEST(*delay_on + *delay_off,
					 HAMCOP_LED_UNIT_MS);

	/* Clamp to 5-bit fields (0-31). */
	if (on_units > 31)
		on_units = 31;
	if (period_units > 31)
		period_units = 31;
	if (on_units == 0)
		on_units = 1;
	if (period_units == 0)
		period_units = 1;
	if (on_units > period_units)
		on_units = period_units;

	/*
	 * LEDxCON0[8:4] LEDx_CMP and LEDxCON1[15:11] LEDx_DIV are both
	 * "value - 1" fields (datasheet ch.12 p.12-2, worked example
	 * p.12-4): writing the literal on_units/period_units counts makes
	 * the hardware run one base-time unit (125 ms) longer than
	 * on_units/period_units actually mean.
	 */
	cmp = (u16)(on_units - 1);
	div = (u16)(period_units - 1);

	/* Update the control registers: enable the LED, set it to
	 * initial-on, and set the CMP (duty) field. CNTEN is left clear
	 * (free-running blink, ch.12: CNTEN=0 means "always blinking");
	 * CNTEN=1 gates blinking to LEDx_CNT cycles and this driver never
	 * programs CNT, which would otherwise stop the LED almost
	 * immediately instead of blinking continuously as a trigger needs.
	 */
	con0 = HAMCOP_LED_CON0_ENB | HAMCOP_LED_CON0_INITV |
	       (cmp << HAMCOP_LED_CON0_CMP_SHIFT) |
	       HAMCOP_LED_CON0_CLK_RTC;

	con1 = (div << HAMCOP_LED_CON1_DIV_SHIFT);

	/*
	 * The CON0/CON1 pair must land atomically with respect to
	 * set_brightness(), or its CON0 read-modify-write can revert the
	 * blink bits written here.
	 */
	spin_lock_irqsave(&leds->lock, flags);
	writew_relaxed(con0, leds->regs + HAMCOP_LED_CON0(chan));
	writew_relaxed(con1, leds->regs + HAMCOP_LED_CON1(chan));
	spin_unlock_irqrestore(&leds->lock, flags);

	/* Return the actual timings in milliseconds */
	*delay_on = on_units * HAMCOP_LED_UNIT_MS;
	*delay_off = (period_units - on_units) * HAMCOP_LED_UNIT_MS;

	return 0;
}

static int hamcop_leds_probe(struct platform_device *pdev)
{
	struct hamcop_leds *leds;
	struct resource *mem;
	unsigned int i;
	int ret;

	leds = devm_kzalloc(&pdev->dev, sizeof(*leds), GFP_KERNEL);
	if (!leds)
		return -ENOMEM;

	leds->dev = &pdev->dev;
	spin_lock_init(&leds->lock);

	/*
	 * platform_get_resource() + devm_ioremap(), not
	 * devm_platform_ioremap_resource() -- the latter reserves the
	 * region exclusively via devm_request_mem_region(), which always
	 * loses to drivers/mfd/hamcop.c's own devm_ioremap_resource() of
	 * the entire HAMCOP MMIO window with -EBUSY.
	 */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -ENODEV;
	leds->regs = devm_ioremap(&pdev->dev, mem->start, resource_size(mem));
	if (!leds->regs)
		return -ENOMEM;

	/* The LED clock is registered by the HAMCOP MFD core and aliased
	 * as "leds-hamcop". We take a reference so the clock stays on.
	 * devm_clk_get_enabled() (not a bare devm_clk_get() +
	 * clk_prepare_enable()) so a later probe failure can't leak the
	 * enable/prepare count -- devm only auto-releases the handle from
	 * the plain form, not the enable state, and .remove() never runs
	 * on a failed probe to clean it up.
	 */
	leds->clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(leds->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(leds->clk),
				     "failed to get LED clock\n");

	/*
	 * Blink timing (hamcop_leds_blink_set() below) is clocked from a
	 * second, separate HAMCOP_GATE_RTC alias that the MFD core registers
	 * for this device (drivers/mfd/hamcop.c). Without it the RTC gate bit
	 * in HAMCOP_CPM_CLOCKCONTROL stays cleared (the force-enable write in
	 * hamcop_probe() only covers PLL/NAND/GPIO), so the CNTEN/CLK_RTC
	 * blink mode is silently inert while plain on/off still works.
	 */
	leds->rtc_clk = devm_clk_get_enabled(&pdev->dev, "rtc");
	if (IS_ERR(leds->rtc_clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(leds->rtc_clk),
				     "failed to get LED RTC clock\n");

	/* Initialize the prescaler once. The 2007 driver only did this once
	 * after power-on and when at least one LED was turned on. We do it
	 * unconditionally at probe time since we keep the clock enabled.
	 */
	writew_relaxed(HAMCOP_LED_PRESCALER & 0xffff,
		       leds->regs + HAMCOP_LED_PS0);
	writew_relaxed((HAMCOP_LED_PRESCALER >> 16) & 0x3ff,
		       leds->regs + HAMCOP_LED_PS1);

	/* Set all channels to use RTC clock (the 2007 driver did this per
	 * channel when powering them on; we set it once for all).
	 */
	for (i = 0; i < HAMCOP_LED_NR_CHANNELS; i++) {
		u16 con0 = readw_relaxed(leds->regs + HAMCOP_LED_CON0(i));

		con0 &= ~BIT(10);  /* Select RTC clock (bit 10 = 0) */
		writew_relaxed(con0, leds->regs + HAMCOP_LED_CON0(i));
	}

	device_for_each_child_node_scoped(&pdev->dev, child) {
		struct led_init_data init_data = { .fwnode = child };
		struct hamcop_led *led;
		u32 chan;

		ret = fwnode_property_read_u32(child, "reg", &chan);
		if (ret || chan >= HAMCOP_LED_NR_CHANNELS)
			return dev_err_probe(&pdev->dev, -EINVAL,
					     "%pfw: invalid LED channel\n", child);

		led = &leds->chans[chan];
		if (led->leds)
			return dev_err_probe(&pdev->dev, -EINVAL,
					     "%pfw: LED channel %u described twice\n",
					     child, chan);

		led->leds = leds;
		led->channel = chan;
		led->cdev.brightness_set = hamcop_leds_set_brightness;
		led->cdev.blink_set = hamcop_leds_blink_set;
		led->cdev.max_brightness = 1;

		ret = devm_led_classdev_register_ext(&pdev->dev, &led->cdev,
						     &init_data);
		if (ret)
			return dev_err_probe(&pdev->dev, ret,
					     "failed to register LED %u\n", chan);
	}

	platform_set_drvdata(pdev, leds);
	return 0;
}

/* No .remove() needed: both clocks are devm_clk_get_enabled(), so devm
 * unwinds prepare/enable state (not just the handle) automatically.
 */
static const struct of_device_id hamcop_leds_of_match[] = {
	{ .compatible = "samsung,hamcop-leds" },
	{ }
};
MODULE_DEVICE_TABLE(of, hamcop_leds_of_match);

static struct platform_driver hamcop_leds_driver = {
	.probe		= hamcop_leds_probe,
	.driver = {
		.name	= "leds-hamcop",
		.of_match_table = hamcop_leds_of_match,
	},
};

module_platform_driver(hamcop_leds_driver);

MODULE_AUTHOR("Koen Kooi <koen@dominion.thruhere.net>");
MODULE_DESCRIPTION("LED driver for Samsung HAMCOP companion ASIC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:leds-hamcop");
