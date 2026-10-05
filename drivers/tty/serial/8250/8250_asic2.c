// SPDX-License-Identifier: GPL-2.0-only
/*
 * 8250 core glue for the UART0/UART1 blocks of the Compaq/HTC ASIC2
 * companion ASIC (HP/Compaq iPAQ h38xx/h39xx family). UART0 is the
 * Bluetooth transport of the h3870/h3875.
 *
 * The ASIC2 core driver maps the chip's whole 0x2000 byte register window
 * without claiming it, and the other cells inside that window map their
 * ranges the same way. The only exclusive claim on this cell's sub-range
 * is the request_mem_region() that the 8250 core's
 * serial8250_request_std_resource() makes for the port; this driver maps
 * the cell with a plain devm_ioremap(). The request has to stay: clearing
 * port.mapbase to skip it makes serial8250_config_port() bail out before
 * it sets up the FIFO control register, and the port would run without
 * FIFOs, with one interrupt per byte.
 *
 * port.mapbase is the cell's physical address, UPF_IOREMAP is left clear
 * so that the 8250 core only reserves the region and never touches
 * port.membase, and UPF_FIXED_TYPE keeps autoconfig from probing the
 * port. The block's one non-16550 register, a write-only reset pulse at
 * +0x20, lies just beyond the 0x20 byte span the 8250 core knows about, so
 * this driver maps the whole 0x24 byte cell and reaches that register
 * through its own mapping.
 *
 * The registers are standard 16550A registers at a 4 byte stride, reached
 * through UPIO_MEM32 with regshift 2.
 *
 * The block needs two clocks: the default clock is its own UART reference
 * select gate, and "ex1" is the shared 24.576 MHz crystal that every ASIC2
 * peripheral gate needs running as well. Neither gate's rate is the UART
 * reference, which is a 3.6864 MHz tap that the SA-1110 supplies on GPIO27,
 * outside ASIC2; port.uartclk therefore comes from the "clock-frequency"
 * property, defaulting to that value.
 *
 * The bring-up of the block is split between the drivers that already
 * hold the relevant resource: this driver does the reset pulse and the
 * clocks, the ASIC2 core driver does the GPIO2/KPIO writes and the
 * per-source interrupt mask bits.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mfd/asic2.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/serial_8250.h>

#include "8250.h"

#define ASIC2_UART_DEFAULT_CLK_HZ	3686400

struct asic2_uart_priv {
	int line;
};

static void asic2_uart_reset_pulse(void __iomem *base)
{
	writel_relaxed(1, base + ASIC2_UART_RESET);
	writel_relaxed(0, base + ASIC2_UART_RESET);
}

static int asic2_uart_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct uart_8250_port uart = { };
	struct asic2_uart_priv *priv;
	struct resource *mem;
	struct clk *clk, *clk_ex1;
	void __iomem *base;
	u32 clk_freq = ASIC2_UART_DEFAULT_CLK_HZ;
	int irq, ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -EINVAL;

	/* The whole 0x24 byte cell, to reach the reset register at +0x20. */
	base = devm_ioremap(dev, mem->start, resource_size(mem));
	if (!base)
		return -ENOMEM;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				      "failed to get/enable gate clock\n");

	clk_ex1 = devm_clk_get_enabled(dev, "ex1");
	if (IS_ERR(clk_ex1))
		return dev_err_probe(dev, PTR_ERR(clk_ex1),
				      "failed to get/enable ex1 clock\n");

	device_property_read_u32(dev, "clock-frequency", &clk_freq);

	asic2_uart_reset_pulse(base);

	uart.port.dev		= dev;
	uart.port.mapbase	= mem->start;
	uart.port.membase	= base;
	uart.port.regshift	= 2;
	uart.port.iotype	= UPIO_MEM32;
	uart.port.uartclk	= clk_freq;
	uart.port.irq		= irq;
	/*
	 * Enabling THRI does not raise a new THRE interrupt on this UART
	 * while the transmitter is already empty. The THRE autoprobe
	 * concludes that and arms the 8250 backup timer, which then starts
	 * every transmission 0.2 s late (245 ms for 16 bytes at 9600 baud,
	 * against 18 ms nominal). UART_BUG_TXEN makes start_tx() load the
	 * FIFO directly instead.
	 */
	uart.port.flags		= UPF_SKIP_TEST | UPF_SHARE_IRQ | UPF_FIXED_TYPE |
				  UPF_NO_THRE_TEST;
	uart.port.type		= PORT_16550A;
	uart.bugs		= UART_BUG_TXEN;

	ret = serial8250_register_8250_port(&uart);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to register 8250 port\n");

	/*
	 * The register path copies flags and bugs but not quirks, and the
	 * TXEN autoprobe would clear UART_BUG_TXEN again as soon as one
	 * sample looks healthy.
	 */
	serial8250_get_port(ret)->port.quirks |= UPQ_NO_TXEN_TEST;

	priv->line = ret;
	platform_set_drvdata(pdev, priv);

	return 0;
}

static void asic2_uart_remove(struct platform_device *pdev)
{
	struct asic2_uart_priv *priv = platform_get_drvdata(pdev);

	serial8250_unregister_port(priv->line);
}

static const struct of_device_id asic2_uart_of_match[] = {
	{ .compatible = "htc,asic2-uart" },
	{ }
};
MODULE_DEVICE_TABLE(of, asic2_uart_of_match);

static struct platform_driver asic2_uart_driver = {
	.driver = {
		.name		= "asic2-uart0",
		.of_match_table	= asic2_uart_of_match,
	},
	.probe	= asic2_uart_probe,
	.remove	= asic2_uart_remove,
};
module_platform_driver(asic2_uart_driver);

/*
 * A "platform:" alias alone is unreachable for an mfd_cell that sets
 * .of_compatible, because platform_uevent() emits the "of:" modalias first
 * and returns. Provide both, so that the platform alias also works for a
 * hotplug path that does not come from devicetree.
 */
MODULE_ALIAS("platform:asic2-uart0");
MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("Compaq/HTC ASIC2 companion ASIC UART0/1 8250 glue driver");
MODULE_LICENSE("GPL");
