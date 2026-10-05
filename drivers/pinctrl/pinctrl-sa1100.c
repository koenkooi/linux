// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel SA-1100/SA-1110 pin control
 *
 * Modelled on drivers/pinctrl/pxa/pinctrl-pxa2xx.c, simplified for SA-1100's
 * much flatter register shape: GAFR @ 0x9004001c is one 32-bit register with
 * exactly one bit per pin (a boolean "alt function enabled", not a 2-bit
 * mux-select field like PXA's GAFRn_x), paired with one bit per pin in GPDR
 * @ 0x90040004 for direction. 28 pins (P0-P27); bits 28-31 of both registers
 * are reserved. There is no PGSR-equivalent sleep-state register anywhere in
 * this block, so unlike pinctrl-pxa2xx.c this driver implements no
 * pinconf_ops at all -- there is nothing for one to configure.
 *
 * Most SA-1100 pins have exactly one fixed alternate peripheral function,
 * so most pins carry a single named function besides "gpio_in"/"gpio_out".
 * P2-P9 (the LCD data bus, GPIO_LDD8..GPIO_LDD15) share ONE
 * function name, "LDD", rather than eight distinct per-bit names: every real
 * consumer of this bus (sa1100fb_setup_gpio() and the board setup code)
 * always muxes all eight lines together as a single unit, so a single
 * "pins = "P2".."P9"; function = "LDD";" consumer group is
 * the correct shape, not eight single-pin groups a board would always list
 * together anyway.
 *
 * GPIO21/22 each have several named alternate functions sharing one GAFR
 * bit (real pad electrical routing, not distinct mux states this driver
 * can select between); which one a board actually gets is decided by
 * TUCR_TIC/TUCR_MR, which this driver does not touch -- so all of a pin's
 * names request the same GAFR bit and only differ in the GPDR direction
 * they request, matching whichever role the board's own TUCR programming
 * has actually selected.
 *
 * GPIO27's alternate function ("32_768kHz") drives the pin with whichever
 * internal clock TUCR's TSEL field selects, the 32.768 kHz oscillator after
 * reset. This driver only routes the pin; the clock it carries is chosen by
 * the clock controller (drivers/clk/clk-sa1100.c). The board-file machines
 * have no pin controller and set the pin from the "gpio27" clock's enable
 * hook in arch/arm/mach-sa1100/clock.c instead.
 */
#include <linux/array_size.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/string.h>

#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>

#include "pinctrl-utils.h"

struct sa1100_desc_function {
	const char *name;
	bool alt;	/* GAFR: 1 enables this pin's alternate function */
	bool dir_out;	/* GPDR: 1 = output, 0 = input */
};

struct sa1100_desc_pin {
	struct pinctrl_pin_desc pin;
	struct sa1100_desc_function *functions;
};

#define SA1100_FUNCTION(_alt, _dir_out, _name)				\
	{ .name = (_name), .alt = (_alt), .dir_out = (_dir_out) }

#define SA1100_PINCTRL_PIN(nr)	PINCTRL_PIN(nr, "P" #nr)

/* Every non-GPIO-only pin still has plain "gpio_in"/"gpio_out" functions --
 * selecting either just clears GAFR's alt-function bit for that pin.
 */
#define SA1100_PIN(_pin, funcs...)					\
	{								\
		.pin = _pin,						\
		.functions = (struct sa1100_desc_function[]) {		\
			SA1100_FUNCTION(false, false, "gpio_in"),	\
			SA1100_FUNCTION(false, true, "gpio_out"),	\
			funcs,						\
			{ }						\
		},							\
	}

#define SA1100_GPIO_ONLY_PIN(_pin)					\
	{								\
		.pin = _pin,						\
		.functions = (struct sa1100_desc_function[]) {		\
			SA1100_FUNCTION(false, false, "gpio_in"),	\
			SA1100_FUNCTION(false, true, "gpio_out"),	\
			{ }						\
		},							\
	}

/*
 * Register shape verified against arch/arm/mach-sa1100/include/mach/
 * SA-1100.h:1148-1183 -- transcribed, not re-derived.
 */
static const struct sa1100_desc_pin sa1100_pins[] = {
	SA1100_GPIO_ONLY_PIN(SA1100_PINCTRL_PIN(0)),
	SA1100_GPIO_ONLY_PIN(SA1100_PINCTRL_PIN(1)),
	SA1100_PIN(SA1100_PINCTRL_PIN(2), SA1100_FUNCTION(true, true, "LDD")),
	SA1100_PIN(SA1100_PINCTRL_PIN(3), SA1100_FUNCTION(true, true, "LDD")),
	SA1100_PIN(SA1100_PINCTRL_PIN(4), SA1100_FUNCTION(true, true, "LDD")),
	SA1100_PIN(SA1100_PINCTRL_PIN(5), SA1100_FUNCTION(true, true, "LDD")),
	SA1100_PIN(SA1100_PINCTRL_PIN(6), SA1100_FUNCTION(true, true, "LDD")),
	SA1100_PIN(SA1100_PINCTRL_PIN(7), SA1100_FUNCTION(true, true, "LDD")),
	SA1100_PIN(SA1100_PINCTRL_PIN(8), SA1100_FUNCTION(true, true, "LDD")),
	SA1100_PIN(SA1100_PINCTRL_PIN(9), SA1100_FUNCTION(true, true, "LDD")),
	SA1100_PIN(SA1100_PINCTRL_PIN(10), SA1100_FUNCTION(true, true, "SSP_TXD")),
	SA1100_PIN(SA1100_PINCTRL_PIN(11), SA1100_FUNCTION(true, false, "SSP_RXD")),
	SA1100_PIN(SA1100_PINCTRL_PIN(12), SA1100_FUNCTION(true, true, "SSP_SCLK")),
	SA1100_PIN(SA1100_PINCTRL_PIN(13), SA1100_FUNCTION(true, true, "SSP_SFRM")),
	SA1100_PIN(SA1100_PINCTRL_PIN(14), SA1100_FUNCTION(true, true, "UART_TXD")),
	SA1100_PIN(SA1100_PINCTRL_PIN(15), SA1100_FUNCTION(true, false, "UART_RXD")),
	/*
	 * SA-1100.h marks SDLC_SCLK "(I/O)" -- SDLC's own clock pin can be
	 * either master (out) or slave (in) depending on board wiring. No
	 * board in this tree uses SDLC; "out" is picked as the more common
	 * real-world default and is easy to widen into a second function
	 * name later if a board ever needs "in".
	 */
	SA1100_PIN(SA1100_PINCTRL_PIN(16), SA1100_FUNCTION(true, true, "SDLC_SCLK")),
	SA1100_PIN(SA1100_PINCTRL_PIN(17), SA1100_FUNCTION(true, true, "SDLC_AAF")),
	SA1100_PIN(SA1100_PINCTRL_PIN(18), SA1100_FUNCTION(true, false, "UART_SCLK1")),
	SA1100_PIN(SA1100_PINCTRL_PIN(19), SA1100_FUNCTION(true, false, "SSP_CLK")),
	SA1100_PIN(SA1100_PINCTRL_PIN(20), SA1100_FUNCTION(true, false, "UART_SCLK3")),
	/*
	 * Three names, one GAFR bit: which pad function P21 actually carries
	 * once the bit is set is decided by TUCR_TIC/TUCR_MR, not by
	 * anything this driver touches. A board's consumer group still
	 * picks one of these three names, for the direction it implies and
	 * for self-documentation; see this file's own header comment.
	 */
	SA1100_PIN(SA1100_PINCTRL_PIN(21),
		   SA1100_FUNCTION(true, false, "MCP_CLK"),
		   SA1100_FUNCTION(true, true, "TIC_ACK"),
		   SA1100_FUNCTION(true, true, "MBGNT")),
	SA1100_PIN(SA1100_PINCTRL_PIN(22),
		   SA1100_FUNCTION(true, false, "TREQA"),
		   SA1100_FUNCTION(true, false, "MBREQ")),
	SA1100_PIN(SA1100_PINCTRL_PIN(23), SA1100_FUNCTION(true, false, "TREQB")),
	SA1100_GPIO_ONLY_PIN(SA1100_PINCTRL_PIN(24)),
	SA1100_PIN(SA1100_PINCTRL_PIN(25), SA1100_FUNCTION(true, true, "1Hz")),
	SA1100_PIN(SA1100_PINCTRL_PIN(26), SA1100_FUNCTION(true, true, "RCLK")),
	SA1100_PIN(SA1100_PINCTRL_PIN(27), SA1100_FUNCTION(true, true, "32_768kHz")),
};

struct sa1100_pinctrl {
	spinlock_t lock;	/* serialises GAFR/GPDR read-modify-write */
	void __iomem *gafr;
	void __iomem *gpdr;
	struct device *dev;
	struct pinctrl_desc desc;
	struct pinctrl_dev *pctl_dev;

	unsigned int npins;
	const struct sa1100_desc_pin *ppins;

	unsigned int ngroups;
	struct pingroup *groups;

	unsigned int nfuncs;
	struct pinfunction *functions;
};

static int sa1100_pctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct sa1100_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->ngroups;
}

static const char *sa1100_pctrl_get_group_name(struct pinctrl_dev *pctldev,
					       unsigned int selector)
{
	struct sa1100_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->groups[selector].name;
}

static int sa1100_pctrl_get_group_pins(struct pinctrl_dev *pctldev,
				       unsigned int selector,
					const unsigned int **pins,
					unsigned int *num_pins)
{
	struct sa1100_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	*pins = pctl->groups[selector].pins;
	*num_pins = pctl->groups[selector].npins;

	return 0;
}

static const struct pinctrl_ops sa1100_pctrl_ops = {
	.get_groups_count	= sa1100_pctrl_get_groups_count,
	.get_group_name		= sa1100_pctrl_get_group_name,
	.get_group_pins		= sa1100_pctrl_get_group_pins,
	.dt_node_to_map		= pinconf_generic_dt_node_to_map_all,
	.dt_free_map		= pinctrl_utils_free_map,
};

static struct sa1100_desc_function *
sa1100_desc_by_func_group(struct sa1100_pinctrl *pctl, const char *pin_name,
			  const char *func_name)
{
	unsigned int i;

	for (i = 0; i < pctl->npins; i++) {
		const struct sa1100_desc_pin *pin = &pctl->ppins[i];
		struct sa1100_desc_function *df;

		if (strcmp(pin->pin.name, pin_name))
			continue;

		for (df = pin->functions; df->name; df++)
			if (!strcmp(df->name, func_name))
				return df;
	}

	return NULL;
}

static const char *sa1100_pmx_get_func_name(struct pinctrl_dev *pctldev,
					    unsigned int selector)
{
	struct sa1100_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->functions[selector].name;
}

static int sa1100_pmx_get_functions_count(struct pinctrl_dev *pctldev)
{
	struct sa1100_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->nfuncs;
}

static int sa1100_pmx_get_func_groups(struct pinctrl_dev *pctldev,
				      unsigned int selector,
				       const char * const **groups,
				       unsigned int *num_groups)
{
	struct sa1100_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	*groups = pctl->functions[selector].groups;
	*num_groups = pctl->functions[selector].ngroups;

	return 0;
}

static int sa1100_pmx_set_mux(struct pinctrl_dev *pctldev,
			      unsigned int func_selector,
			       unsigned int group_selector)
{
	struct sa1100_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct pingroup *g = &pctl->groups[group_selector];
	unsigned int pin = g->pins[0];
	struct sa1100_desc_function *df;
	unsigned long flags;
	u32 val;

	df = sa1100_desc_by_func_group(pctl, g->name,
				       pctl->functions[func_selector].name);
	if (!df)
		return -EINVAL;

	dev_dbg(pctl->dev, "set_mux(pin=%u): alt=%d dir_out=%d\n",
		pin, df->alt, df->dir_out);

	spin_lock_irqsave(&pctl->lock, flags);

	val = readl_relaxed(pctl->gafr);
	val = df->alt ? val | BIT(pin) : val & ~BIT(pin);
	writel_relaxed(val, pctl->gafr);

	val = readl_relaxed(pctl->gpdr);
	val = df->dir_out ? val | BIT(pin) : val & ~BIT(pin);
	writel_relaxed(val, pctl->gpdr);

	spin_unlock_irqrestore(&pctl->lock, flags);

	return 0;
}

static const struct pinmux_ops sa1100_pinmux_ops = {
	.get_functions_count	= sa1100_pmx_get_functions_count,
	.get_function_name	= sa1100_pmx_get_func_name,
	.get_function_groups	= sa1100_pmx_get_func_groups,
	.set_mux		= sa1100_pmx_set_mux,
};

static struct pinctrl_desc sa1100_pinctrl_desc = {
	.pctlops	= &sa1100_pctrl_ops,
	.pmxops		= &sa1100_pinmux_ops,
	/*
	 * No .confops: there is no PGSR-equivalent sleep-state register
	 * anywhere in this 8-register block (GPLR/GPDR/GPSR/GPCR/GRER/GFER/
	 * GEDR/GAFR and nothing else), so there is no PIN_CONFIG_* parameter
	 * this driver could implement.
	 */
};

static const struct pinfunction *
sa1100_find_function(const char *fname, const struct pinfunction *functions)
{
	const struct pinfunction *func;

	for (func = functions; func->name; func++)
		if (!strcmp(fname, func->name))
			return func;

	return NULL;
}

static int sa1100_build_functions(struct sa1100_pinctrl *pctl)
{
	struct pinfunction *functions;
	unsigned int i;
	struct sa1100_desc_function *df;

	/*
	 * At most 3 named alt-functions on any one pin (P21) plus the 2
	 * always-present gpio_in/gpio_out functions -- 5 * npins is a safe
	 * high bound on the number of distinct function names, same
	 * over-allocate-then-shrink approach as pxa2xx_build_functions().
	 */
	functions = devm_kcalloc(pctl->dev, pctl->npins * 5,
				 sizeof(*functions), GFP_KERNEL);
	if (!functions)
		return -ENOMEM;

	for (i = 0; i < pctl->npins; i++)
		for (df = pctl->ppins[i].functions; df->name; df++)
			if (!sa1100_find_function(df->name, functions))
				(functions + pctl->nfuncs++)->name = df->name;

	pctl->functions = devm_kmemdup_array(pctl->dev, functions, pctl->nfuncs,
					     sizeof(*functions), GFP_KERNEL);
	if (!pctl->functions)
		return -ENOMEM;

	devm_kfree(pctl->dev, functions);

	return 0;
}

static int sa1100_build_groups(struct sa1100_pinctrl *pctl)
{
	unsigned int i, j, ngroups;
	struct sa1100_desc_function *df;
	struct pinfunction *func;
	const char **gtmp;

	gtmp = devm_kmalloc_array(pctl->dev, pctl->npins, sizeof(*gtmp),
				  GFP_KERNEL);
	if (!gtmp)
		return -ENOMEM;

	for (i = 0; i < pctl->nfuncs; i++) {
		ngroups = 0;
		for (j = 0; j < pctl->npins; j++)
			for (df = pctl->ppins[j].functions; df->name; df++)
				if (!strcmp(pctl->functions[i].name, df->name))
					gtmp[ngroups++] = pctl->ppins[j].pin.name;

		func = &pctl->functions[i];
		func->ngroups = ngroups;
		func->groups = devm_kmemdup_array(pctl->dev, gtmp, ngroups,
						  sizeof(*gtmp), GFP_KERNEL);
		if (!func->groups)
			return -ENOMEM;
	}

	devm_kfree(pctl->dev, gtmp);

	return 0;
}

static int sa1100_build_state(struct sa1100_pinctrl *pctl)
{
	struct pinctrl_pin_desc *pins;
	struct pingroup *group;
	unsigned int i;
	int ret;

	pctl->npins = ARRAY_SIZE(sa1100_pins);
	pctl->ppins = sa1100_pins;
	pctl->ngroups = pctl->npins;

	pctl->desc.npins = pctl->npins;
	pins = devm_kcalloc(pctl->dev, pctl->npins, sizeof(*pins), GFP_KERNEL);
	if (!pins)
		return -ENOMEM;
	pctl->desc.pins = pins;
	for (i = 0; i < pctl->npins; i++)
		pins[i] = pctl->ppins[i].pin;

	pctl->groups = devm_kmalloc_array(pctl->dev, pctl->ngroups,
					  sizeof(*pctl->groups), GFP_KERNEL);
	if (!pctl->groups)
		return -ENOMEM;
	for (i = 0; i < pctl->npins; i++) {
		group = &pctl->groups[i];
		group->name = pctl->ppins[i].pin.name;
		group->pins = &pctl->ppins[i].pin.number;
		group->npins = 1;
	}

	ret = sa1100_build_functions(pctl);
	if (ret)
		return ret;

	return sa1100_build_groups(pctl);
}

static int sa1100_pinctrl_probe(struct platform_device *pdev)
{
	struct sa1100_pinctrl *pctl;
	struct resource *res;
	int ret;

	pctl = devm_kzalloc(&pdev->dev, sizeof(*pctl), GFP_KERNEL);
	if (!pctl)
		return -ENOMEM;

	/*
	 * Plain devm_ioremap(), not devm_platform_ioremap_resource(): this
	 * node's "gafr"/"gpdr" windows are sub-ranges of the same physical
	 * block the "gpio" node covers. drivers/gpio/gpio-sa1100.c never
	 * calls request_mem_region() (it runs off the fixed __REG() virtual
	 * address instead), so there is no live conflict today, but claiming
	 * either window exclusively here is still the same -EBUSY bug class
	 * pinctrl-pxa25x.c's own probe avoids for the identical reason
	 * against gpio-pxa.c.
	 */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "gafr");
	if (!res)
		return -ENODEV;
	pctl->gafr = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!pctl->gafr)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "gpdr");
	if (!res)
		return -ENODEV;
	pctl->gpdr = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!pctl->gpdr)
		return -ENOMEM;

	pctl->dev = &pdev->dev;
	spin_lock_init(&pctl->lock);
	platform_set_drvdata(pdev, pctl);

	pctl->desc = sa1100_pinctrl_desc;
	pctl->desc.name = dev_name(&pdev->dev);
	pctl->desc.owner = THIS_MODULE;

	ret = sa1100_build_state(pctl);
	if (ret)
		return ret;

	pctl->pctl_dev = devm_pinctrl_register(&pdev->dev, &pctl->desc, pctl);
	if (IS_ERR(pctl->pctl_dev))
		return dev_err_probe(&pdev->dev, PTR_ERR(pctl->pctl_dev),
				      "failed to register pinctrl driver\n");

	return 0;
}

static const struct of_device_id sa1100_pinctrl_match[] = {
	{ .compatible = "intel,sa1100-pinctrl", },
	{ }
};
MODULE_DEVICE_TABLE(of, sa1100_pinctrl_match);

static struct platform_driver sa1100_pinctrl_driver = {
	.probe	= sa1100_pinctrl_probe,
	.driver	= {
		.name		= "sa1100-pinctrl",
		.of_match_table	= sa1100_pinctrl_match,
	},
};
module_platform_driver(sa1100_pinctrl_driver);

MODULE_DESCRIPTION("Intel SA-1100/SA-1110 pinctrl driver");
MODULE_LICENSE("GPL");
