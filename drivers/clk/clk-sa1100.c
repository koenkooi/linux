// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel StrongARM SA-1100/SA-1110 clock controller
 *
 * The core clock (CCLK) is the PXTAL crystal multiplied by the PLL setting
 * in the power manager's PPCR. The test unit's TUCR selects which internal
 * clock GPIO27 carries when that pin is switched to its alternate output;
 * the pin itself is muxed by the pin controller.
 */

#include <linux/bits.h>
#include <linux/clk-provider.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/overflow.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <dt-bindings/clock/intel,sa1100-clocks.h>

#define PPCR_CCF		GENMASK(4, 0)
#define PPCR_CCF_MAX		11

#define TUCR_TSEL_SHIFT		29
#define TUCR_TSEL_WIDTH		3
#define TUCR_TSEL_NUM		BIT(TUCR_TSEL_WIDTH)

#define SA1100_NR_CLKS		(SA1100_CLK_TSEL_VDDL_RING_DIV4 + 1)

struct sa1100_cclk {
	struct clk_hw hw;
	void __iomem *ppcr;
};

static DEFINE_SPINLOCK(sa1100_tucr_lock);

static unsigned long sa1100_cclk_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct sa1100_cclk *cclk = container_of(hw, struct sa1100_cclk, hw);
	u32 ccf = readl(cclk->ppcr) & PPCR_CCF;

	/*
	 * Every entry of the SA-1110 Developer's Manual table 8-1, for both
	 * the 3.6864 MHz and the 3.5795 MHz crystal, is PXTAL x 4 x (4 + CCF).
	 * CCF values above 11 are not supported.
	 */
	if (ccf > PPCR_CCF_MAX)
		return 0;

	return parent_rate * 4 * (4 + ccf);
}

static const struct clk_ops sa1100_cclk_ops = {
	.recalc_rate = sa1100_cclk_recalc_rate,
};

static struct clk_hw * __init sa1100_cclk_register(struct device_node *np,
						   void __iomem *ppcr)
{
	const struct clk_parent_data parent = { .fw_name = "pxtal" };
	struct clk_init_data init = {
		.name = "cclk",
		.ops = &sa1100_cclk_ops,
		.parent_data = &parent,
		.num_parents = 1,
		/*
		 * cpufreq reprograms PPCR directly, so the rate must be read
		 * back from the register rather than cached.
		 */
		.flags = CLK_GET_RATE_NOCACHE | CLK_IS_CRITICAL,
	};
	struct sa1100_cclk *cclk;
	int ret;

	cclk = kzalloc_obj(*cclk);
	if (!cclk)
		return ERR_PTR(-ENOMEM);

	cclk->ppcr = ppcr;
	cclk->hw.init = &init;

	ret = of_clk_hw_register(np, &cclk->hw);
	if (ret) {
		kfree(cclk);
		return ERR_PTR(ret);
	}

	return &cclk->hw;
}

static void __init sa1100_tsel_unregister(const struct clk_hw **tsel)
{
	int i;

	for (i = TUCR_TSEL_NUM - 1; i >= 0; i--) {
		struct clk_hw *hw = (struct clk_hw *)tsel[i];

		if (IS_ERR(hw))
			continue;
		/* TSEL 2, 3 and 7 have no parent and are fixed-rate clocks. */
		if (BIT(i) & (BIT(2) | BIT(3) | BIT(7)))
			clk_hw_unregister_fixed_rate(hw);
		else
			clk_hw_unregister_fixed_factor(hw);
	}
}

static void __init sa1100_clocks_init(struct device_node *np)
{
	const struct clk_hw *tsel[TUCR_TSEL_NUM];
	struct clk_hw_onecell_data *data;
	void __iomem *ppcr, *tucr;
	struct sa1100_cclk *cclk;
	struct clk_hw *hw;
	int i, ret;

	data = kzalloc_flex(*data, hws, SA1100_NR_CLKS);
	if (!data)
		return;
	data->num = SA1100_NR_CLKS;

	ppcr = of_iomap(np, 0);
	tucr = of_iomap(np, 1);
	if (!ppcr || !tucr) {
		pr_err("%pOF: failed to map registers\n", np);
		goto err_unmap;
	}

	hw = sa1100_cclk_register(np, ppcr);
	if (IS_ERR(hw))
		goto err_cclk;
	data->hws[SA1100_CLK_CPU] = hw;

	/* TUCR.TSEL inputs, in TSEL order (SA-1110 Developer's Manual D.1) */
	tsel[0] = clk_hw_register_fixed_factor_fwname(NULL, np, "tsel-osc32k",
						      "txtal", 0, 1, 1);
	tsel[1] = clk_hw_register_fixed_factor_fwname(NULL, np, "tsel-osc3m6",
						      "pxtal", 0, 1, 1);
	/* The ring oscillator frequencies are not specified. */
	tsel[2] = clk_hw_register_fixed_rate(NULL, "tsel-vdd-ring-div16",
					     NULL, 0, 0);
	tsel[3] = clk_hw_register_fixed_rate(NULL, "tsel-pll96-div4",
					     NULL, 0, 24000000);
	tsel[4] = clk_hw_register_fixed_factor_fwname(NULL, np,
						      "tsel-osc32k-rclk",
						      "txtal", 0, 1, 1);
	tsel[5] = clk_hw_register_fixed_factor_fwname(NULL, np,
						      "tsel-osc3m6-alt",
						      "pxtal", 0, 1, 1);
	tsel[6] = clk_hw_register_fixed_factor_parent_hw(NULL, "tsel-pll-div16",
							 data->hws[SA1100_CLK_CPU],
							 CLK_GET_RATE_NOCACHE,
							 1, 16);
	tsel[7] = clk_hw_register_fixed_rate(NULL, "tsel-vddl-ring-div4",
					     NULL, 0, 0);

	for (i = 0; i < TUCR_TSEL_NUM; i++) {
		if (IS_ERR(tsel[i])) {
			hw = (struct clk_hw *)tsel[i];
			goto err_tsel;
		}
		data->hws[SA1100_CLK_TSEL_OSC32K + i] = (struct clk_hw *)tsel[i];
	}

	/*
	 * TSEL 1 and 5 carry the same oscillator, and boards need a specific
	 * one of them, so a rate request must never pick the parent.
	 */
	hw = clk_hw_register_mux_hws(NULL, "gpio27-clkout", tsel, TUCR_TSEL_NUM,
				     CLK_SET_RATE_NO_REPARENT |
				     CLK_GET_RATE_NOCACHE,
				     tucr, TUCR_TSEL_SHIFT, TUCR_TSEL_WIDTH, 0,
				     &sa1100_tucr_lock);
	if (IS_ERR(hw))
		goto err_tsel;
	data->hws[SA1100_CLK_GPIO27] = hw;

	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get, data);
	if (ret) {
		clk_hw_unregister_mux(hw);
		hw = ERR_PTR(ret);
		goto err_tsel;
	}

	return;

err_tsel:
	sa1100_tsel_unregister(tsel);
	cclk = container_of(data->hws[SA1100_CLK_CPU], struct sa1100_cclk, hw);
	clk_hw_unregister(&cclk->hw);
	kfree(cclk);
err_cclk:
	pr_err("%pOF: failed to register clocks: %pe\n", np, hw);
err_unmap:
	if (ppcr)
		iounmap(ppcr);
	if (tucr)
		iounmap(tucr);
	kfree(data);
}

CLK_OF_DECLARE(sa1100_clks, "intel,sa1100-clocks", sa1100_clocks_init);
