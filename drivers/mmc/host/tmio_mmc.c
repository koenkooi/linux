// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the MMC / SD / SDIO cell found in:
 *
 * TC6393XB TC6391XB TC6387XB T7L66XB ASIC3
 *
 * ASIC3 (the Compaq/HTC companion ASIC of the HP iPAQ hx4700 and h39xx) is
 * the one chip of that list still supported; its MFD core
 * (drivers/mfd/asic3.c) is the only caller, through its "tmio-mmc"
 * mfd_cell. This is the driver removed by commit 1c37746b9fa4 ("mmc:
 * remove tmio_mmc driver"), adapted to the current tmio_mmc_core:
 *
 *  - tmio_mmc_host_free() is gone, as tmio_mmc_host_alloc() uses
 *    devm_mmc_alloc_host(), so the error path no longer frees the host.
 *  - tmio_mmc_data.set_clk_div is gone. The divider decision of
 *    tmio_mmc_set_clock() calls asic3_sd_set_clock_mode() directly,
 *    because the SD_CONFIG ClockMode register lives behind a mapping
 *    private to the MFD core.
 *  - tmio_mmc_data.set_pwr is gone, but ASIC3 still has to switch its
 *    internal SD_CONFIG power with MMC_POWER_UP and MMC_POWER_OFF. It is
 *    restored as a host callback and not as platform data, as ASIC3 is the
 *    only remaining user. The h3900 uses the native 32-bit PWR1/PWR2 unit
 *    of its WinCE driver, the hx4700 the PWR2 bus power bit.
 *  - platform_driver .remove returns void.
 *  - The mfd_cell .enable/.disable/.suspend/.resume hooks are gone. The
 *    MFD core does the SD_CONFIG/SDHWCTRL bring-up at its own probe, and
 *    power management is a clk_prepare_enable()/clk_disable_unprepare()
 *    pair on the SD_HOST and SD_BUS clock aliases it registers, wired to
 *    the clk_enable and clk_disable hooks of the host and so driven by the
 *    runtime PM hooks of tmio_mmc_core.
 *
 * Copyright (C) 2017 Renesas Electronics Corporation
 * Copyright (C) 2017 Horms Solutions, Simon Horman
 * Copyright (C) 2007 Ian Molton
 * Copyright (C) 2004 Ian Molton
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/log2.h>
#include <linux/mfd/asic3.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pagemap.h>
#include <linux/platform_data/tmio.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/scatterlist.h>

#include "tmio_mmc.h"

/* Registers specific to this variant */
#define CTL_SDIO_REGS		0x100
#define CTL_CLK_AND_WAIT_CTL	0x138
#define CTL_RESET_SDIO		0x1e0

/*
 * Driver private data, reached from the clk_enable and clk_disable hooks
 * through container_of() on host->pdata, as uniphier-sd.c does. The pdata
 * cannot simply be embedded in something bigger by the caller, because it is
 * the copy that mfd-core made of the cell data, sized to exactly
 * sizeof(struct tmio_mmc_data). Copy it into this allocation instead and
 * point host->pdata at the copy.
 */
struct tmio_mmc_asic3_priv {
	struct clk *clk_host;
	struct clk *clk_bus;
	struct tmio_mmc_data pdata;
};

static struct tmio_mmc_asic3_priv *tmio_mmc_asic3_priv(struct tmio_mmc_host *host)
{
	return container_of(host->pdata, struct tmio_mmc_asic3_priv, pdata);
}

/*
 * Balanced clk_prepare_enable()/clk_disable_unprepare() pairs on the SD_HOST
 * and SD_BUS clocks. Probe enables them once explicitly, since
 * tmio_mmc_host_probe() assumes they are live, and the runtime PM callbacks of
 * tmio_mmc_core call the hooks from then on.
 *
 * Both gates are CLK_IS_CRITICAL in drivers/mfd/asic3.c, so the clock core
 * holds a permanent enable reference of its own and disabling here cannot
 * bring either gate below it; the clocks stay on regardless of card presence
 * until that flag is revisited.
 */
static int tmio_mmc_asic3_clk_enable(struct tmio_mmc_host *host)
{
	struct tmio_mmc_asic3_priv *priv = tmio_mmc_asic3_priv(host);
	int ret;

	ret = clk_prepare_enable(priv->clk_host);
	if (ret)
		return ret;

	ret = clk_prepare_enable(priv->clk_bus);
	if (ret)
		clk_disable_unprepare(priv->clk_host);

	return ret;
}

static void tmio_mmc_asic3_clk_disable(struct tmio_mmc_host *host)
{
	struct tmio_mmc_asic3_priv *priv = tmio_mmc_asic3_priv(host);

	clk_disable_unprepare(priv->clk_bus);
	clk_disable_unprepare(priv->clk_host);
}

static void tmio_mmc_clk_start(struct tmio_mmc_host *host)
{
	sd_ctrl_write16(host, CTL_SD_CARD_CLK_CTL, CLK_CTL_SCLKEN |
		sd_ctrl_read16(host, CTL_SD_CARD_CLK_CTL));

	usleep_range(10000, 11000);
	sd_ctrl_write16(host, CTL_CLK_AND_WAIT_CTL, 0x0100);
	usleep_range(10000, 11000);
}

static void tmio_mmc_clk_stop(struct tmio_mmc_host *host)
{
	sd_ctrl_write16(host, CTL_CLK_AND_WAIT_CTL, 0x0000);
	usleep_range(10000, 11000);

	sd_ctrl_write16(host, CTL_SD_CARD_CLK_CTL, ~CLK_CTL_SCLKEN &
		sd_ctrl_read16(host, CTL_SD_CARD_CLK_CTL));

	usleep_range(10000, 11000);
}

static void tmio_mmc_set_clock(struct tmio_mmc_host *host,
			       unsigned int new_clock)
{
	/*
	 * struct tmio_mmc_host has no field for driver-private state, and
	 * the parent of the cell is always the asic3 device, so look it up
	 * on demand.
	 */
	struct asic3 *asic = dev_get_drvdata(host->pdev->dev.parent);
	unsigned int divisor;
	u32 clk = 0;
	int clk_sel;

	if (new_clock == 0) {
		tmio_mmc_clk_stop(host);
		return;
	}

	divisor = host->pdata->hclk / new_clock;

	/* bit7 set: 1/512, ... bit0 set: 1/4, all bits clear: 1/2 */
	clk_sel = (divisor <= 1);
	clk = clk_sel ? 0 : (roundup_pow_of_two(divisor) >> 2);

	/*
	 * ClockMode is 1 whenever a divider is programmed and 0 only for the
	 * undivided fastest setting, the opposite sense of clk_sel, which is
	 * true for that fastest setting.
	 */
	asic3_sd_set_clock_mode(asic, !clk_sel);

	sd_ctrl_write16(host, CTL_SD_CARD_CLK_CTL, ~CLK_CTL_SCLKEN &
			sd_ctrl_read16(host, CTL_SD_CARD_CLK_CTL));
	sd_ctrl_write16(host, CTL_SD_CARD_CLK_CTL, clk & CLK_CTL_DIV_MASK);
	usleep_range(10000, 11000);

	tmio_mmc_clk_start(host);
}

static void tmio_mmc_set_power(struct tmio_mmc_host *host, bool on)
{
	struct asic3 *asic = dev_get_drvdata(host->pdev->dev.parent);

	if (of_machine_is_compatible("hp,ipaq-h3900")) {
		asic3_sd_set_slot_voltage(asic, on);
		if (on)
			msleep(20);
	} else {
		asic3_sd_set_bus_power(asic, on);
	}
}

static void tmio_mmc_reset(struct tmio_mmc_host *host, bool preserve)
{
	struct asic3 *asic = dev_get_drvdata(host->pdev->dev.parent);

	sd_ctrl_write16(host, CTL_RESET_SDIO, 0x0000);
	usleep_range(10000, 11000);
	sd_ctrl_write16(host, CTL_RESET_SDIO, 0x0001);
	usleep_range(10000, 11000);

	/*
	 * The core only reaches this hook after set_clock(0), which leaves
	 * CLK_AND_WAIT_CTL at zero and does not touch the SD_CONFIG clock
	 * mode. Restore the post-reset state the hardware expects before the
	 * first card detect instead of relying on what the boot loader left.
	 */
	sd_ctrl_write16(host, CTL_SD_CARD_CLK_CTL, 0x0000);
	sd_ctrl_write16(host, CTL_STOP_INTERNAL_ACTION, 0x0000);
	sd_ctrl_write16(host, CTL_CLK_AND_WAIT_CTL, 0x0100);
	asic3_sd_set_clock_mode(asic, false);
	sd_ctrl_write16(host, CTL_SD_CARD_CLK_CTL, 0x0000);
	usleep_range(2000, 3000);

	/*
	 * The core writes SD_OPTION back only when it restores state across
	 * a reset (preserve), never on the first probe, so the register keeps
	 * whatever ASIC3 powers up with. The h3900 vendor driver sets it to
	 * 0x40ee as part of its post-reset baseline. Other boards sharing this
	 * driver work without the write and the value is not known to suit
	 * their silicon, so apply it on the h3900 only.
	 */
	if (of_machine_is_compatible("hp,ipaq-h3900"))
		sd_ctrl_write16(host, CTL_SD_MEM_CARD_OPT, 0x40ee);

	/*
	 * Power the slot after the SD_CTRL reset: ASIC3 probe runs before
	 * this cell does and its reset would undo any earlier power-up. The
	 * core unmasks the card insert/remove interrupts only after this hook
	 * returns, which is the closest this split driver can get to the
	 * vendor order of powering the slot last.
	 */
	asic3_sd_set_power(asic, true);
	if (!of_machine_is_compatible("hp,ipaq-h3900"))
		asic3_sd_set_bus_power(asic, true);
	/*
	 * Only the controller's own card-detect bits are debounced; with a
	 * card-detect GPIO or polling nothing reads them before they settle.
	 */
	if (host->native_hotplug)
		asic3_sd_wait_card_detect_settle(asic);

	/*
	 * Drop whatever the reset left latched so it cannot be taken for the
	 * first command's RESPEND. Reserved bit 27 is kept set, as the
	 * TMIO-compatible register definition requires.
	 */
	if (of_machine_is_compatible("hp,ipaq-h3900"))
		sd_ctrl_write32_as_16_and_16(host, CTL_STATUS,
					     TMIO_STAT_ALWAYS_SET_27);
}

static int tmio_mmc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tmio_mmc_data *pdata;
	struct tmio_mmc_host *host;
	struct tmio_mmc_asic3_priv *priv;
	struct resource *res;
	unsigned long bus_rate;
	int ret, irq;

	pdata = dev_get_platdata(dev);
	if (!pdata || !pdata->hclk)
		return -EINVAL;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	/*
	 * See tmio_mmc_asic3_priv: this copy is what lets host->pdata be
	 * converted back to priv in the clk_enable and clk_disable hooks.
	 */
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->pdata = *pdata;

	priv->clk_host = devm_clk_get(dev, "sd_host");
	if (IS_ERR(priv->clk_host))
		return dev_err_probe(dev, PTR_ERR(priv->clk_host),
				      "failed to get sd_host clock\n");

	priv->clk_bus = devm_clk_get(dev, "sd_bus");
	if (IS_ERR(priv->clk_bus))
		return dev_err_probe(dev, PTR_ERR(priv->clk_bus),
				      "failed to get sd_bus clock\n");

	/*
	 * The SD bus rate depends on the board and the clock framework is
	 * where ASIC3 records it: its SD_BUS gate reports the rate that its
	 * enable path selects, 12.288 MHz on the h3900 and 24.576 MHz
	 * elsewhere. f_max, f_min and the card clock divisor of
	 * tmio_mmc_set_clock() are all computed from it.
	 * priv->pdata is the copy of the cell data that belongs to this host, so
	 * updating it here affects this host only.
	 */
	bus_rate = clk_get_rate(priv->clk_bus);
	if (bus_rate)
		priv->pdata.hclk = bus_rate;
	else
		dev_warn(dev, "sd_bus clock reports no rate, keeping hclk %u Hz\n",
			 priv->pdata.hclk);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	host = tmio_mmc_host_alloc(pdev, &priv->pdata);
	if (IS_ERR(host))
		return PTR_ERR(host);

	host->clk_enable = tmio_mmc_asic3_clk_enable;
	host->clk_disable = tmio_mmc_asic3_clk_disable;

	/*
	 * tmio_mmc_host_probe() assumes the clocks are already live. It does
	 * not call host->clk_enable() itself, it only tells runtime PM that
	 * the device is active.
	 */
	ret = tmio_mmc_asic3_clk_enable(host);
	if (ret)
		return ret;

	/* SD control register space size is 0x200, 0x400 for bus_shift=1 */
	host->bus_shift = resource_size(res) >> 10;
	host->set_clock = tmio_mmc_set_clock;
	host->set_pwr = tmio_mmc_set_power;
	host->reset = tmio_mmc_reset;

	/*
	 * The parent interrupt is edge triggered: keep every status source
	 * masked except those the core enables, so an unhandled level cannot
	 * hold the line low and suppress the next edge.
	 */
	if (of_machine_is_compatible("hp,ipaq-h3900"))
		host->sdcard_irq_mask_all = ~0U;

	host->mmc->f_max = host->pdata->hclk;
	host->mmc->f_min = host->pdata->hclk / 512;

	/*
	 * The failure path of tmio_mmc_host_probe() already removes the host,
	 * so only the clocks need unwinding here.
	 */
	ret = tmio_mmc_host_probe(host);
	if (ret)
		goto disable_clk;

	ret = devm_request_irq(dev, irq, tmio_mmc_irq, IRQF_TRIGGER_FALLING,
			       dev_name(dev), host);
	if (ret) {
		tmio_mmc_host_remove(host);
		goto disable_clk;
	}

	return 0;

disable_clk:
	tmio_mmc_asic3_clk_disable(host);
	return ret;
}

static void tmio_mmc_remove(struct platform_device *pdev)
{
	struct tmio_mmc_host *host = platform_get_drvdata(pdev);
	struct asic3 *asic = dev_get_drvdata(pdev->dev.parent);

	tmio_mmc_host_remove(host);
	/* The reset hook switched the slot rail on and nothing else switches it off. */
	asic3_sd_set_power(asic, false);
	tmio_mmc_asic3_clk_disable(host);
}

/* ------------------- device registration ----------------------- */

static const struct dev_pm_ops tmio_mmc_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(tmio_mmc_host_runtime_suspend,
			   tmio_mmc_host_runtime_resume, NULL)
};

static struct platform_driver tmio_mmc_driver = {
	.driver = {
		.name = "tmio-mmc",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.pm = &tmio_mmc_dev_pm_ops,
	},
	.probe = tmio_mmc_probe,
	.remove = tmio_mmc_remove,
};

module_platform_driver(tmio_mmc_driver);

MODULE_DESCRIPTION("Toshiba TMIO SD/MMC driver");
MODULE_AUTHOR("Ian Molton <spyro@f2s.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tmio-mmc");
