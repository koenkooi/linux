// SPDX-License-Identifier: GPL-1.0+
/*
 * OHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>
 * (C) Copyright 2000-2005 David Brownell
 * (C) Copyright 2002 Hewlett-Packard Company
 *
 * Samsung SAMCOP companion ASIC bus glue (HP iPAQ h51xx/h54xx/h55xx)
 *
 * SAMCOP's on-chip OHCI controller can only master its own 32 KiB SRAM,
 * not system RAM. dmabounce is gone from mainline and restricted-dma-pool
 * cannot point at on-chip SRAM, so usb_hcd_setup_local_mem() is used: it
 * is the mainline mechanism for a companion chip whose OHCI reaches only a
 * small dedicated memory. The chip also masks the DMA master's addresses
 * in hardware at its reset default (SAMCOP_PCMCIA_MISC_USB_MASK).
 *
 * The only power-up requirement is the USB host clock gate, aliased
 * "samcop-ohci" by the MFD core. The UCLK gate has reversed sense and is
 * already on after the core's clock-control init, so there is nothing to
 * request for it. The onboard radio behind the root hub has its own power
 * rails, which are not controlled from here: its disconnect is an ordinary
 * USB disconnect to the root hub.
 *
 * 32 KiB is small for the HCCA, the EDs/TDs and every URB buffer of a WLAN
 * NIC; whether an 802.11b link saturates the pool is not measured.
 *
 * This file is licenced under the GPL.
 */

#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "ohci.h"

#define DRIVER_DESC "OHCI SAMCOP driver"

struct samcop_ohci_priv {
	struct clk *clk;
};

#define hcd_to_samcop_ohci_priv(h) \
	((struct samcop_ohci_priv *)hcd_to_ohci(h)->priv)

static const struct ohci_driver_overrides samcop_ohci_overrides __initconst = {
	.product_desc		= DRIVER_DESC,
	.extra_priv_size	= sizeof(struct samcop_ohci_priv),
};

static struct hc_driver __read_mostly ohci_samcop_hc_driver;

/*-------------------------------------------------------------------------*/

static int samcop_ohci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct samcop_ohci_priv *priv;
	struct resource *regs, *sram;
	struct usb_hcd *hcd;
	int irq, ret;

	if (usb_disabled())
		return -ENODEV;

	/*
	 * Cell resource order, drivers/mfd/samcop.c's samcop_ohci_resources[]:
	 * 0 = the OHCI register window (SAMCOP_USBHOST_BASE/SIZE), 1 = the
	 * 32 KiB SRAM this driver hands to usb_hcd_setup_local_mem(), same
	 * index order ohci-sm501.c's own probe() uses for its register vs.
	 * local-memory resources.
	 */
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs)
		return -ENODEV;

	sram = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!sram)
		return -ENODEV;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	hcd = usb_create_hcd(&ohci_samcop_hc_driver, dev, dev_name(dev));
	if (!hcd)
		return -ENOMEM;

	priv = hcd_to_samcop_ohci_priv(hcd);

	hcd->rsrc_start = regs->start;
	hcd->rsrc_len = resource_size(regs);

	/*
	 * SAMCOP_USBHOST_SIZE (64 KiB) is sized generously to the next
	 * block base rather than to the real, much smaller, spec-fixed
	 * OHCI register file -- include/linux/mfd/samcop.h's own comment
	 * says as much. Mapping it as given costs nothing but VA space and
	 * is not narrowed here.
	 *
	 * devm_ioremap(), not devm_ioremap_resource() -- the latter
	 * reserves the region exclusively via devm_request_mem_region(),
	 * which always loses to drivers/mfd/samcop.c's own
	 * devm_ioremap_resource() of the entire SAMCOP MMIO window with
	 * -EBUSY: SAMCOP_USBHOST_BASE (0x20000) is a sub-range of that same
	 * already-claimed window. Same bug class fixed for real on physical
	 * h2210 hardware for pxa27x/pxa25x pinctrl, hamcop-sdi
	 * (a6f7616fd9a3) and leds-hamcop (198098879172) -- see
	 * drivers/mfd/hamcop.c's own header comment for the general
	 * pattern.
	 */
	hcd->regs = devm_ioremap(dev, regs->start, resource_size(regs));
	if (!hcd->regs) {
		ret = -ENOMEM;
		goto err_put_hcd;
	}

	/*
	 * "samcop-ohci"/NULL clkdev alias, drivers/mfd/samcop.c's
	 * samcop_clocks_register() -> SAMCOP_GATE_USBHOST
	 * (SAMCOP_CLK_USBHOST). This is the only clock this driver needs
	 * to manage -- see the header comment's UCLK discussion for why
	 * there is no second clock to request here, unlike hackndev's own
	 * ohci-h5400.c.
	 */
	priv->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(priv->clk)) {
		ret = dev_err_probe(dev, PTR_ERR(priv->clk),
				    "failed to get clock\n");
		goto err_put_hcd;
	}

	/*
	 * The usb core needs to be told that it must copy data into our
	 * local memory if the buffers happen to be placed in regular
	 * memory (ohci-sm501.c's own comment on this exact call -- SAMCOP's
	 * DMA master genuinely cannot reach system RAM at all, not just
	 * "should not"). dma_addr 0: sram->start (after
	 * drivers/mfd/samcop.c's devm_mfd_add_devices() resolves this
	 * cell's resource against the chip's own physical base) minus
	 * sram->parent->start is always 0, because SAMCOP_SRAM_BASE is chip
	 * offset 0 -- the SRAM window IS the start of the chip's own
	 * address space, exactly what SAMCOP's DMA master addresses
	 * relative to. Same offset computation ohci-sm501.c uses for
	 * SM501's own local-bus-relative addressing.
	 */
	ret = usb_hcd_setup_local_mem(hcd, sram->start,
				      sram->start - sram->parent->start,
				      resource_size(sram));
	if (ret)
		goto err_put_hcd;

	ret = usb_add_hcd(hcd, irq, 0);
	if (ret)
		goto err_put_hcd;

	device_wakeup_enable(hcd->self.controller);

	return 0;

err_put_hcd:
	usb_put_hcd(hcd);
	return ret;
}

static void samcop_ohci_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
}

/*-------------------------------------------------------------------------*/

static int samcop_ohci_suspend(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct samcop_ohci_priv *priv = hcd_to_samcop_ohci_priv(hcd);
	bool do_wakeup = device_may_wakeup(dev);
	int ret;

	ret = ohci_suspend(hcd, do_wakeup);
	if (ret)
		return ret;

	clk_disable(priv->clk);

	return 0;
}

static int samcop_ohci_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct samcop_ohci_priv *priv = hcd_to_samcop_ohci_priv(hcd);
	int ret;

	ret = clk_enable(priv->clk);
	if (ret)
		return ret;

	ohci_resume(hcd, false);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(samcop_ohci_pm_ops,
				samcop_ohci_suspend, samcop_ohci_resume);

/*-------------------------------------------------------------------------*/

static const struct of_device_id samcop_ohci_of_match[] = {
	{ .compatible = "samsung,samcop-ohci" },
	{ }
};
MODULE_DEVICE_TABLE(of, samcop_ohci_of_match);

static struct platform_driver ohci_hcd_samcop_driver = {
	.probe		= samcop_ohci_probe,
	.remove		= samcop_ohci_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver		= {
		.name	= "samcop-ohci",
		.of_match_table = samcop_ohci_of_match,
		.pm	= pm_ptr(&samcop_ohci_pm_ops),
	},
};

static int __init ohci_samcop_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	ohci_init_driver(&ohci_samcop_hc_driver, &samcop_ohci_overrides);

	return platform_driver_register(&ohci_hcd_samcop_driver);
}
module_init(ohci_samcop_init);

static void __exit ohci_samcop_cleanup(void)
{
	platform_driver_unregister(&ohci_hcd_samcop_driver);
}
module_exit(ohci_samcop_cleanup);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Koen Kooi");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:samcop-ohci");
