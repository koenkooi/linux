// SPDX-License-Identifier: GPL-2.0-only
/*
 * LED driver for the Compaq/HTC ASIC3 companion ASIC
 *
 * ASIC3 has three independent LED channels, each with its own hardware
 * TimeBase/PeriodTime/DutyTime/AutoStopCount block (include/linux/mfd/
 * asic3.h). The LED GPIOs themselves are inputs; a channel is switched
 * entirely through this timer/duty block rather than a plain GPIO output
 * bit.
 *
 * The cell has no resources of its own and is bound by of_compatible to a
 * child node of the ASIC3 device. Register access goes through the
 * exported asic3_read_register() and asic3_write_register() of the parent,
 * and each channel's clock gate is a devm_clk_get_enabled() consumer.
 *
 * Copyright (C) 2011 Paul Parsons <lost.distance@yahoo.com>
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/leds.h>
#include <linux/mfd/asic3.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>

/* LED TimeBase bits, kept local rather than in the shared asic3.h */
#define LED_TBS			0x0f	/* low 4 bits: time base divisor, max 13 */
#define LED_EN			BIT(4)	/* 0:off 1:on */
#define LED_AUTOSTOP		BIT(5)	/* 0:disable 1:enable auto stop */
#define LED_ALWAYS		BIT(6)	/* LED interrupt mask, 0:no mask 1:mask */

/* TimeBase index 4 gives a ~32 Hz counter resolution */
#define ASIC3_LED_TIMEBASE_32HZ	4

/*
 * delay_on/delay_off are milliseconds; the hardware counts in ~32 Hz
 * (31.25 ms) ticks, with 12-bit PeriodTime/DutyTime fields.
 */
#define ASIC3_LED_MS_TO_CLK(ms)	DIV_ROUND_CLOSEST((ms) * 1024, 32000)
#define ASIC3_LED_CLK_TO_MS(clk)	(((clk) * 32000) / 1024)
#define ASIC3_LED_MAX_CLK		4000
#define ASIC3_LED_MAX_MS		ASIC3_LED_CLK_TO_MS(ASIC3_LED_MAX_CLK)

static const u32 asic3_led_base[ASIC3_NUM_LEDS] = {
	ASIC3_LED0_BASE, ASIC3_LED1_BASE, ASIC3_LED2_BASE,
};

/*
 * Per-channel clkdev con_id, matching the three aliases drivers/mfd/asic3.c
 * registers for this cell: each channel has its own CLOCK_CDEX_LEDn gate,
 * and a channel whose gate is not enabled has an unclocked TimeBase
 * counter, so both on and blink would silently do nothing.
 */
static const char * const asic3_led_clk_names[ASIC3_NUM_LEDS] = {
	"led0", "led1", "led2",
};

struct asic3_led {
	struct led_classdev cdev;
	struct asic3 *asic;
	struct clk *clk;
	u32 base;
};

static void asic3_led_brightness_set(struct led_classdev *cdev,
				     enum led_brightness value)
{
	struct asic3_led *led = container_of(cdev, struct asic3_led, cdev);
	u32 timebase = value == LED_OFF ? 0 : (LED_EN | ASIC3_LED_TIMEBASE_32HZ);

	/* Full-on is 100% duty (duty == period). */
	asic3_write_register(led->asic, led->base + ASIC3_LED_PERIODTIME, 32);
	asic3_write_register(led->asic, led->base + ASIC3_LED_DUTYTIME, 32);
	asic3_write_register(led->asic, led->base + ASIC3_LED_AUTOSTOPCOUNT, 0);
	asic3_write_register(led->asic, led->base + ASIC3_LED_TIMEBASE, timebase);
}

static int asic3_led_blink_set(struct led_classdev *cdev,
			       unsigned long *delay_on, unsigned long *delay_off)
{
	struct asic3_led *led = container_of(cdev, struct asic3_led, cdev);
	u32 on, off;

	if (*delay_on > ASIC3_LED_MAX_MS || *delay_off > ASIC3_LED_MAX_MS)
		return -EINVAL;

	if (*delay_on == 0 && *delay_off == 0) {
		/* No preference from the trigger: a sensible default. */
		on = ASIC3_LED_MS_TO_CLK(500);
		off = ASIC3_LED_MS_TO_CLK(500);
	} else {
		on = ASIC3_LED_MS_TO_CLK(*delay_on);
		off = ASIC3_LED_MS_TO_CLK(*delay_off);
		if (on + off > ASIC3_LED_MAX_CLK)
			return -EINVAL;
	}

	asic3_write_register(led->asic, led->base + ASIC3_LED_PERIODTIME, on + off);
	asic3_write_register(led->asic, led->base + ASIC3_LED_DUTYTIME, on);
	asic3_write_register(led->asic, led->base + ASIC3_LED_AUTOSTOPCOUNT, 0);
	asic3_write_register(led->asic, led->base + ASIC3_LED_TIMEBASE,
			     LED_EN | ASIC3_LED_TIMEBASE_32HZ);

	*delay_on = ASIC3_LED_CLK_TO_MS(on);
	*delay_off = ASIC3_LED_CLK_TO_MS(off);

	return 0;
}

/*
 * GPIOC0/1/2 are hardwired to the alternate function of LED0/1/2 on every
 * ASIC3 (ASIC3_GPIOC0_LED0 and siblings in include/linux/mfd/asic3.h), so
 * this is chip-internal routing rather than board policy and the driver
 * applies it directly. The direction is set explicitly rather than
 * assumed to be at its reset default (input), in case a bootloader left it
 * in output mode or the core set bank C as outputs
 * (htc,gpio-default-output-low).
 *
 * There is no locking against the gpio_chip callbacks of asic3.c, as none
 * of them touches ASIC3_GPIO_ALT_FUNCTION and bits 0-2 of bank C are not
 * shared with any other cell.
 */
static const u16 asic3_led_gpio_config[ASIC3_NUM_LEDS] = {
	ASIC3_GPIOC0_LED0, ASIC3_GPIOC1_LED1, ASIC3_GPIOC2_LED2,
};

static void asic3_led_configure_gpio(struct asic3 *asic, unsigned int channel)
{
	u16 config = asic3_led_gpio_config[channel];
	unsigned int pin = ASIC3_CONFIG_GPIO_PIN(config);
	unsigned int alt = ASIC3_CONFIG_GPIO_ALT(config);
	unsigned int dir = ASIC3_CONFIG_GPIO_DIR(config);
	u32 base = ASIC3_BANK_TO_BASE(pin / ASIC3_GPIOS_PER_BANK);
	unsigned int bit = pin % ASIC3_GPIOS_PER_BANK;
	u32 val;

	val = asic3_read_register(asic, base + ASIC3_GPIO_DIRECTION);
	val = dir ? (val | BIT(bit)) : (val & ~BIT(bit));
	asic3_write_register(asic, base + ASIC3_GPIO_DIRECTION, val);

	val = asic3_read_register(asic, base + ASIC3_GPIO_ALT_FUNCTION);
	val = alt ? (val | BIT(bit)) : (val & ~BIT(bit));
	asic3_write_register(asic, base + ASIC3_GPIO_ALT_FUNCTION, val);
}

static int asic3_led_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct asic3 *asic = dev_get_drvdata(dev->parent);
	struct asic3_led *leds;
	int num_leds, ret;

	num_leds = device_get_child_node_count(dev);
	if (!num_leds || num_leds > ASIC3_NUM_LEDS)
		return dev_err_probe(dev, -EINVAL,
				      "expected 1-%d led child nodes, got %d\n",
				      ASIC3_NUM_LEDS, num_leds);

	leds = devm_kcalloc(dev, ASIC3_NUM_LEDS, sizeof(*leds), GFP_KERNEL);
	if (!leds)
		return -ENOMEM;

	device_for_each_child_node_scoped(dev, child) {
		struct led_init_data init_data = { .fwnode = child };
		struct asic3_led *led;
		u32 channel;

		ret = fwnode_property_read_u32(child, "reg", &channel);
		if (ret || channel >= ASIC3_NUM_LEDS)
			return dev_err_probe(dev, -EINVAL,
					      "bad or missing \"reg\" on LED child node\n");

		led = &leds[channel];
		led->asic = asic;
		led->base = asic3_led_base[channel];

		/*
		 * devm_clk_get_enabled() rather than devm_clk_get() plus
		 * clk_prepare_enable(): on a failed probe the latter would let
		 * devm release only the clk handle, leaking the prepare/enable
		 * count.
		 */
		led->clk = devm_clk_get_enabled(dev, asic3_led_clk_names[channel]);
		if (IS_ERR(led->clk))
			return dev_err_probe(dev, PTR_ERR(led->clk),
					      "failed to get channel %u clock\n", channel);

		asic3_led_configure_gpio(asic, channel);

		led->cdev.brightness_set = asic3_led_brightness_set;
		led->cdev.blink_set = asic3_led_blink_set;
		led->cdev.max_brightness = 1;

		init_data.devicename = "asic3";
		/* Fallback if the device tree has no "label" or "function"/"color" */
		init_data.default_label = ":";

		ret = devm_led_classdev_register_ext(dev, &led->cdev, &init_data);
		if (ret)
			return dev_err_probe(dev, ret,
					      "failed to register LED %u\n", channel);
	}

	return 0;
}

static const struct of_device_id asic3_led_of_match[] = {
	{ .compatible = "htc,asic3-leds" },
	{ }
};
MODULE_DEVICE_TABLE(of, asic3_led_of_match);

static struct platform_driver asic3_led_driver = {
	.probe	= asic3_led_probe,
	.driver = {
		.name = "leds-asic3",
		.of_match_table = asic3_led_of_match,
	},
};
module_platform_driver(asic3_led_driver);

MODULE_AUTHOR("Paul Parsons <lost.distance@yahoo.com>");
MODULE_AUTHOR("Koen Kooi <koen@dominion.thruhere.net>");
MODULE_DESCRIPTION("LED driver for Compaq/HTC ASIC3 companion ASIC");
MODULE_LICENSE("GPL");
