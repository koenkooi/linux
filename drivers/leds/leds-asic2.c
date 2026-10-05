// SPDX-License-Identifier: GPL-2.0-only
/*
 * LED driver for the blink-timer block of the Compaq/HTC ASIC2 companion
 * ASIC, which drives the status LEDs of the HP iPAQ h3800.
 *
 * ASIC2 has three independent blink-timer channels, each with its own
 * TimeBase, PeriodTime and DutyTime registers. The block has no clock
 * enable of its own: it is fed by EX0, which the MFD core forces on before
 * any cell probes. Registers are 32 bits wide on a flat 4-byte stride.
 *
 * A channel is always reprogrammed in the same order: TimeBase is cleared
 * first, then PeriodTime and DutyTime are written, and TimeBase is written
 * last to start the timer. The sequence is done under one lock, so that a
 * brightness change and a blink request on the same channel cannot
 * interleave their writes.
 *
 * The channels are described by the "led-controller" child of the ASIC2
 * devicetree node: one LED per child node, named and triggered from the
 * node's properties.
 *
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/mfd/asic2.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/spinlock.h>

/*
 * TimeBase bit 4 enables the timer; it doubles as the "on" state for
 * brightness_set(), since a timer whose DutyTime equals its PeriodTime
 * produces the same waveform as a steady output. Bits 3:0 are the time base
 * divisor.
 */
#define ASIC2_LED_TBS_BLINK		BIT(4)

/*
 * Time base divisor 5 gives 1 << (5 + 1) = 64 timer ticks per second.
 */
#define ASIC2_LED_TIMEBASE_DIVIDER	5
#define ASIC2_LED_CYCLES_PER_SEC	(1 << (ASIC2_LED_TIMEBASE_DIVIDER + 1))

/* delay_on/delay_off are in milliseconds; PeriodTime/DutyTime are 12-bit fields. */
#define ASIC2_LED_MS_TO_CLK(ms)	DIV_ROUND_CLOSEST((ms) * ASIC2_LED_CYCLES_PER_SEC, 1000)
#define ASIC2_LED_CLK_TO_MS(clk)	DIV_ROUND_CLOSEST((clk) * 1000, ASIC2_LED_CYCLES_PER_SEC)
#define ASIC2_LED_MAX_CLK		GENMASK(11, 0)
#define ASIC2_LED_MAX_MS		ASIC2_LED_CLK_TO_MS(ASIC2_LED_MAX_CLK)

/* Steady on: a one second period with a duty cycle of 100%. */
#define ASIC2_LED_FULL_ON_CLK		ASIC2_LED_CYCLES_PER_SEC

static const u32 asic2_led_base[ASIC2_NUM_LEDS] = {
	ASIC2_LED0_BASE, ASIC2_LED1_BASE, ASIC2_LED2_BASE,
};

struct asic2_led {
	struct led_classdev cdev;
	struct asic2_leds *leds;
	unsigned int channel;
};

struct asic2_leds {
	struct device *dev;
	void __iomem *regs;
	spinlock_t lock;	/* serialises a channel's register sequence */
	struct asic2_led chans[ASIC2_NUM_LEDS];
};

static inline void asic2_led_write(struct asic2_leds *leds, unsigned int ch,
				   u32 reg, u32 val)
{
	writel_relaxed(val, leds->regs + (asic2_led_base[ch] - ASIC2_LED0_BASE) + reg);
}

static void asic2_led_brightness_set(struct led_classdev *cdev,
				     enum led_brightness value)
{
	struct asic2_led *led = container_of(cdev, struct asic2_led, cdev);
	struct asic2_leds *leds = led->leds;
	unsigned int ch = led->channel;
	u32 timebase = value == LED_OFF ? 0 :
		(ASIC2_LED_TBS_BLINK | ASIC2_LED_TIMEBASE_DIVIDER);
	unsigned long flags;

	spin_lock_irqsave(&leds->lock, flags);
	asic2_led_write(leds, ch, ASIC2_LED_TIMEBASE, 0);
	if (value != LED_OFF) {
		asic2_led_write(leds, ch, ASIC2_LED_PERIODTIME, ASIC2_LED_FULL_ON_CLK);
		asic2_led_write(leds, ch, ASIC2_LED_DUTYTIME, ASIC2_LED_FULL_ON_CLK);
		asic2_led_write(leds, ch, ASIC2_LED_AUTOSTOPCOUNT, 0);
		asic2_led_write(leds, ch, ASIC2_LED_TIMEBASE, timebase);
	}
	spin_unlock_irqrestore(&leds->lock, flags);
}

static int asic2_led_blink_set(struct led_classdev *cdev,
			       unsigned long *delay_on, unsigned long *delay_off)
{
	struct asic2_led *led = container_of(cdev, struct asic2_led, cdev);
	struct asic2_leds *leds = led->leds;
	unsigned int ch = led->channel;
	unsigned long flags;
	u32 on, off;

	if (*delay_on > ASIC2_LED_MAX_MS || *delay_off > ASIC2_LED_MAX_MS)
		return -EINVAL;

	if (*delay_on == 0 && *delay_off == 0) {
		/* No preference from the trigger: a sensible default. */
		on = ASIC2_LED_MS_TO_CLK(500);
		off = ASIC2_LED_MS_TO_CLK(500);
	} else {
		on = ASIC2_LED_MS_TO_CLK(*delay_on);
		off = ASIC2_LED_MS_TO_CLK(*delay_off);
		if (on + off > ASIC2_LED_MAX_CLK)
			return -EINVAL;
	}

	spin_lock_irqsave(&leds->lock, flags);
	asic2_led_write(leds, ch, ASIC2_LED_TIMEBASE, 0);
	asic2_led_write(leds, ch, ASIC2_LED_PERIODTIME, on + off);
	asic2_led_write(leds, ch, ASIC2_LED_DUTYTIME, on);
	asic2_led_write(leds, ch, ASIC2_LED_AUTOSTOPCOUNT, 0);
	asic2_led_write(leds, ch, ASIC2_LED_TIMEBASE,
			ASIC2_LED_TBS_BLINK | ASIC2_LED_TIMEBASE_DIVIDER);
	spin_unlock_irqrestore(&leds->lock, flags);

	*delay_on = ASIC2_LED_CLK_TO_MS(on);
	*delay_off = ASIC2_LED_CLK_TO_MS(off);

	return 0;
}

static void asic2_led_init_channel(struct asic2_leds *leds, unsigned int channel)
{
	struct asic2_led *led = &leds->chans[channel];
	struct led_classdev *cdev = &led->cdev;

	led->leds = leds;
	led->channel = channel;

	cdev->brightness_set = asic2_led_brightness_set;
	cdev->blink_set = asic2_led_blink_set;
	cdev->max_brightness = 1;
}

static int asic2_led_probe_dt(struct device *dev, struct asic2_leds *leds)
{
	int ret;

	fwnode_for_each_available_child_node_scoped(dev_fwnode(dev), child) {
		struct led_init_data init_data = { .fwnode = child };
		struct asic2_led *led;
		u32 channel;

		ret = fwnode_property_read_u32(child, "reg", &channel);
		if (ret || channel >= ASIC2_NUM_LEDS)
			return dev_err_probe(dev, -EINVAL,
					      "bad or missing \"reg\" on LED child node\n");

		led = &leds->chans[channel];
		if (led->leds)
			return dev_err_probe(dev, -EINVAL,
					      "LED channel %u described twice\n", channel);

		asic2_led_init_channel(leds, channel);

		init_data.devicename = "asic2";

		ret = devm_led_classdev_register_ext(dev, &led->cdev, &init_data);
		if (ret)
			return dev_err_probe(dev, ret,
					      "failed to register LED %u\n", channel);
	}

	return 0;
}

static int asic2_led_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct asic2_leds *leds;
	struct resource *mem;
	int ret;

	leds = devm_kzalloc(dev, sizeof(*leds), GFP_KERNEL);
	if (!leds)
		return -ENOMEM;

	leds->dev = dev;
	spin_lock_init(&leds->lock);

	/*
	 * The MFD core already maps the whole ASIC2 window, so
	 * devm_platform_ioremap_resource() would fail with -EBUSY on the
	 * exclusive region request; map the cell's resource without it.
	 */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -ENODEV;
	leds->regs = devm_ioremap(dev, mem->start, resource_size(mem));
	if (!leds->regs)
		return -ENOMEM;

	ret = asic2_led_probe_dt(dev, leds);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, leds);

	return 0;
}

static struct platform_driver asic2_led_driver = {
	.probe	= asic2_led_probe,
	.driver = {
		.name = "leds-asic2",
	},
};
module_platform_driver(asic2_led_driver);

MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("LED driver for Compaq/HTC ASIC2 companion ASIC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:leds-asic2");
