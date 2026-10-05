// SPDX-License-Identifier: GPL-2.0-only
/*
 * PWM driver for the PWM0/PWM1 blocks of the ASIC2 companion ASIC (HP iPAQ
 * h38xx/h39xx family). PWM0 drives the LCD frontlight. PWM1's output pin is
 * multiplexed with GPIO13 (option-jacket power) and has no known consumer;
 * both channels are registered anyway.
 *
 * The register layout (TimeBase/PeriodTime/DutyTime at 0x00/0x04/0x08 per
 * channel, channels 0x100 apart) matches ASIC2_PWM_* in
 * include/linux/mfd/asic2.h.
 *
 * What output frequency a given TimeBase divider selector produces is not
 * documented (bits 0-3 are only "clock-divider select 0-8"). The driver
 * therefore does not derive an arbitrary period from the PWM clock rate.
 * It always programs one fixed divider and period pair and scales only the
 * duty register in proportion to the requested duty_cycle/period ratio.
 * The pair is board supplied through the "htc,pwm-divider" and
 * "htc,pwm-period" properties and defaults to what the boot loader uses
 * for the h39xx frontlight (divider 7, period 10 counts). h38xx uses
 * divider 8 with a period of 64. That
 * suffices for a pwm-backlight consumer, where brightness is a relative
 * duty ratio, but the period_ns the hardware really produces is not known.
 *
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/pwm.h>

#define ASIC2_PWM_NR_CHANNELS		2
#define ASIC2_PWM_CHANNEL_STRIDE	0x100

#define ASIC2_PWM_TIME_BASE		0x00
#define ASIC2_PWM_PERIOD_TIME		0x04
#define ASIC2_PWM_DUTY_TIME		0x08

#define ASIC2_PWM_TIME_BASE_DIVIDER	GENMASK(3, 0)
#define ASIC2_PWM_TIME_BASE_ENABLE	BIT(4)
#define ASIC2_PWM_TIME_BASE_CLEAR	BIT(5)

/* The configuration the boot loader uses on h39xx, see the header comment */
#define ASIC2_PWM_DEFAULT_DIVIDER	7
#define ASIC2_PWM_DEFAULT_PERIOD	10

/*
 * The vendor driver never programmed a duty below 21 of its period of 64
 * (about 33%). That floor is kept as a ratio and rounded up to the next
 * count of the configured period: at a period of 10, 3/10 is below it and
 * 4/10 is not, and at a period of 64 it is exactly 21.
 * pwm-backlight only enables the channel for a nonzero brightness, so a duty
 * under the floor is a dim request and not a request for off.
 */
#define ASIC2_PWM_DUTY_FLOOR_NUM	21
#define ASIC2_PWM_DUTY_FLOOR_DEN	64

struct asic2_pwm {
	struct device *dev;
	void __iomem *regs;
	u32 divider;
	u32 period;
};

static inline void asic2_pwm_write(struct asic2_pwm *pc, unsigned int ch,
				   u32 reg, u32 val)
{
	writel_relaxed(val, pc->regs + ch * ASIC2_PWM_CHANNEL_STRIDE + reg);
}

static int asic2_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			   const struct pwm_state *state)
{
	struct asic2_pwm *pc = pwmchip_get_drvdata(chip);
	unsigned int ch = pwm->hwpwm;
	u32 duty, duty_floor;

	if (state->polarity != PWM_POLARITY_NORMAL)
		return -EINVAL;

	if (!state->enabled) {
		asic2_pwm_write(pc, ch, ASIC2_PWM_TIME_BASE, ASIC2_PWM_TIME_BASE_CLEAR);
		return 0;
	}

	/*
	 * Fixed period, proportional duty (see the header comment).
	 * mul_u64_u64_div_u64() avoids overflow on 64-bit inputs.
	 */
	duty = mul_u64_u64_div_u64(pc->period, state->duty_cycle,
				   state->period);
	if (duty > pc->period)
		duty = pc->period;

	duty_floor = DIV_ROUND_UP(pc->period * ASIC2_PWM_DUTY_FLOOR_NUM,
				  ASIC2_PWM_DUTY_FLOOR_DEN);
	if (duty < duty_floor)
		duty = duty_floor;

	/* Same order as the boot loader: disable, program period and duty, enable */
	asic2_pwm_write(pc, ch, ASIC2_PWM_TIME_BASE, 0);
	asic2_pwm_write(pc, ch, ASIC2_PWM_PERIOD_TIME, pc->period);
	asic2_pwm_write(pc, ch, ASIC2_PWM_DUTY_TIME, duty);
	asic2_pwm_write(pc, ch, ASIC2_PWM_TIME_BASE,
			ASIC2_PWM_TIME_BASE_ENABLE |
			(pc->divider & ASIC2_PWM_TIME_BASE_DIVIDER));

	return 0;
}

static const struct pwm_ops asic2_pwm_ops = {
	.apply = asic2_pwm_apply,
};

static int asic2_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pwm_chip *chip;
	struct asic2_pwm *pc;
	struct resource *mem;
	int ret;

	chip = devm_pwmchip_alloc(dev, ASIC2_PWM_NR_CHANNELS, sizeof(*pc));
	if (IS_ERR(chip))
		return PTR_ERR(chip);

	pc = pwmchip_get_drvdata(chip);
	pc->dev = dev;

	/*
	 * devm_ioremap() rather than devm_platform_ioremap_resource(), which
	 * would reserve the region exclusively: it is a sub-range of the
	 * window the MFD core has already reserved, so the request would
	 * fail with -EBUSY.
	 */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -ENODEV;
	pc->regs = devm_ioremap(dev, mem->start, resource_size(mem));
	if (!pc->regs)
		return -ENOMEM;

	/* Own peripheral gate, plus the shared EX1 crystal it needs on top */
	ret = PTR_ERR_OR_ZERO(devm_clk_get_enabled(dev, NULL));
	if (ret)
		return dev_err_probe(dev, ret, "failed to get gate clock\n");
	ret = PTR_ERR_OR_ZERO(devm_clk_get_enabled(dev, "ex1"));
	if (ret)
		return dev_err_probe(dev, ret, "failed to get EX1 clock\n");

	/*
	 * The fallback stays in place when a property, or the device node
	 * itself, is absent.
	 */
	pc->divider = ASIC2_PWM_DEFAULT_DIVIDER;
	pc->period = ASIC2_PWM_DEFAULT_PERIOD;
	device_property_read_u32(dev, "htc,pwm-divider", &pc->divider);
	device_property_read_u32(dev, "htc,pwm-period", &pc->period);

	chip->ops = &asic2_pwm_ops;

	return devm_pwmchip_add(dev, chip);
}

static const struct of_device_id asic2_pwm_of_match[] = {
	{ .compatible = "htc,asic2-pwm" },
	{ }
};
MODULE_DEVICE_TABLE(of, asic2_pwm_of_match);

static struct platform_driver asic2_pwm_driver = {
	.driver = {
		.name = "asic2-pwm",
		.of_match_table = asic2_pwm_of_match,
	},
	.probe	= asic2_pwm_probe,
};
module_platform_driver(asic2_pwm_driver);

MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("ASIC2 companion ASIC PWM (HP iPAQ h38xx/h39xx)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:asic2-pwm");
