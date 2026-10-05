// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPI master driver for the byte-at-a-time SPI block of the ASIC2
 * companion ASIC (HP iPAQ h38xx/h39xx family).
 *
 * The register layout (Control/Data/ChipSelectDisabled at 0x00/0x04/0x08,
 * 8 significant bits in 32-bit registers) matches ASIC2_SPI_* in
 * include/linux/mfd/asic2.h. The driver maps its own copy of just that
 * window.
 *
 * Transfers are one byte at a time, normally interrupt driven with a
 * completion and a timeout; if the completion interrupt never arrives the
 * driver polls CONTROL.SPE instead, which the hardware clears when the byte
 * is done. There are two on-chip chip selects (CONTROL.SEL_CS1); which
 * device sits on which is board policy.
 *
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#define ASIC2_SPI_CONTROL		0x00
#define ASIC2_SPI_DATA			0x04

#define ASIC2_SPI_CONTROL_SPR(clk)	((clk) & 0x0f)
#define ASIC2_SPI_CONTROL_SPE		BIT(4)
#define ASIC2_SPI_CONTROL_SPIE		BIT(5)
#define ASIC2_SPI_CONTROL_SEL_CS1	BIT(6)
#define ASIC2_SPI_CONTROL_CPOL		BIT(7)

/*
 * One byte never takes anywhere near this long; the timeout only detects
 * a wedged or absent device instead of hanging forever.
 */
#define ASIC2_SPI_TIMEOUT_MS		100

/*
 * Clock-rate selectors per chip select: the ID EEPROM is chip select 0
 * with SPR(3), the jacket microcontroller chip select 1 with SPR(2). The
 * hardware selector has no established conversion to Hz, so
 * spi_device.max_speed_hz is not used.
 */
#define ASIC2_SPI_CONTROL_SPR_EEPROM	ASIC2_SPI_CONTROL_SPR(3)
#define ASIC2_SPI_CONTROL_SPR_MCU	ASIC2_SPI_CONTROL_SPR(2)

struct asic2_spi {
	struct device *dev;
	void __iomem *regs;
	struct completion done;
	int irq;
	u8 rx_byte;

	/*
	 * Set the first time a byte's completion IRQ never arrives but
	 * CONTROL.SPE is found clear. Later bytes then skip the timeout wait
	 * and poll SPE directly, so they do not each pay the timeout. SPE
	 * clearing does not by itself show that a byte was shifted: at SPR(3),
	 * nominally about 64 kHz, one byte should take about 125 us, and SPE
	 * has been seen clearing roughly three times sooner.
	 */
	bool irq_lost;
};

/* The registers carry 8 significant bits in 32-bit words. */
static inline u8 asic2_spi_read(struct asic2_spi *spi, u32 reg)
{
	return readl_relaxed(spi->regs + reg);
}

static inline void asic2_spi_write(struct asic2_spi *spi, u32 reg, u8 val)
{
	writel_relaxed(val, spi->regs + reg);
}

static irqreturn_t asic2_spi_irq(int irq, void *data)
{
	struct asic2_spi *spi = data;
	u8 control;

	/*
	 * The hardware clears CONTROL.SPE itself when the byte transfer
	 * completes. Latch DATA before CONTROL is rewritten to clear SPIE
	 * (so that the interrupt source does not stay armed after the byte
	 * has finished): the reference WinCE driver always loads DATA before
	 * a CONTROL write that follows a byte, and the opposite order is the
	 * one it never produces.
	 */
	spi->rx_byte = asic2_spi_read(spi, ASIC2_SPI_DATA);
	control = asic2_spi_read(spi, ASIC2_SPI_CONTROL);
	asic2_spi_write(spi, ASIC2_SPI_CONTROL, control & ~ASIC2_SPI_CONTROL_SPIE);
	complete(&spi->done);

	return IRQ_HANDLED;
}

static int asic2_spi_transfer_one(struct spi_controller *ctlr,
				  struct spi_device *sdev,
				   struct spi_transfer *xfer)
{
	struct asic2_spi *spi = spi_controller_get_devdata(ctlr);
	const u8 *tx = xfer->tx_buf;
	u8 *rx = xfer->rx_buf;
	u8 control;
	unsigned int i;

	if (spi_get_chipselect(sdev, 0))
		control = ASIC2_SPI_CONTROL_SPR_MCU | ASIC2_SPI_CONTROL_SEL_CS1;
	else
		control = ASIC2_SPI_CONTROL_SPR_EEPROM;

	/*
	 * A byte is started with three separate writes (CONTROL with SPIE,
	 * then DATA, then CONTROL with SPIE and SPE), and SPIE stays set until
	 * SPE has cleared and, on a receive, DATA has been read back. That
	 * holds for polled bytes too, so SPIE is set for every byte; once the
	 * completion interrupt is found lost it is disabled instead.
	 */
	control |= ASIC2_SPI_CONTROL_SPIE;

	if (sdev->mode & SPI_CPOL)
		control |= ASIC2_SPI_CONTROL_CPOL;

	for (i = 0; i < xfer->len; i++) {
		unsigned long time_left = 0;

		reinit_completion(&spi->done);

		/*
		 * control never carries SPE, so this write sets SPIE and the
		 * SPR/SEL/CPOL selection without starting the engine; SPE
		 * goes in only once DATA holds the byte.
		 */
		asic2_spi_write(spi, ASIC2_SPI_CONTROL, control);
		asic2_spi_write(spi, ASIC2_SPI_DATA, tx ? tx[i] : 0);
		asic2_spi_write(spi, ASIC2_SPI_CONTROL, control | ASIC2_SPI_CONTROL_SPE);

		if (!spi->irq_lost) {
			unsigned long timeout = msecs_to_jiffies(ASIC2_SPI_TIMEOUT_MS);

			time_left = wait_for_completion_timeout(&spi->done, timeout);
		}
		if (!time_left) {
			/*
			 * No interrupt (or none expected). The engine clears
			 * CONTROL.SPE itself when the byte is done, so poll
			 * for that: SPE clear means the engine reports the
			 * byte done, which is not proof that it was shifted
			 * (see irq_lost), SPE stuck high means the engine
			 * made no progress.
			 */
			u8 ctl;
			int ret;

			ret = read_poll_timeout(asic2_spi_read, ctl,
						!(ctl & ASIC2_SPI_CONTROL_SPE),
						100,
						ASIC2_SPI_TIMEOUT_MS * 1000,
						false,
						spi, ASIC2_SPI_CONTROL);
			if (ret) {
				asic2_spi_write(spi, ASIC2_SPI_CONTROL,
						asic2_spi_read(spi, ASIC2_SPI_CONTROL) &
						~ASIC2_SPI_CONTROL_SPIE);
				dev_err(spi->dev,
					"byte %u/%u: no IRQ and CONTROL.SPE stuck high\n",
					i, xfer->len);
				return -ETIMEDOUT;
			}

			if (!spi->irq_lost) {
				/*
				 * SPIE stays set from here on, so the
				 * interrupt is disabled instead, and unlazily:
				 * KPIO's enable is cleared now, not on the next
				 * interrupt, the state the vendor's polled
				 * engine runs in.
				 */
				spi->irq_lost = true;
				irq_set_status_flags(spi->irq,
						     IRQ_DISABLE_UNLAZY);
				disable_irq(spi->irq);
				dev_warn(spi->dev,
					 "completion IRQ never arrived but CONTROL.SPE cleared, polling SPE from now on\n");
			}

			/* Same DATA-before-CONTROL order as asic2_spi_irq(). */
			spi->rx_byte = asic2_spi_read(spi, ASIC2_SPI_DATA);
			asic2_spi_write(spi, ASIC2_SPI_CONTROL,
					control & ~ASIC2_SPI_CONTROL_SPIE);
		}

		if (rx)
			rx[i] = spi->rx_byte;
	}

	spi_finalize_current_transfer(ctlr);

	return 0;
}

/*
 * Release the chip select the way the WinCE driver does between
 * transactions: set CONTROL.SEL twice, then clear it. SEL = 1 routes the
 * select to CS1 and drives CS0 high, so the pulse gives the CS0 device (the
 * 25LC040 ID EEPROM) the rising edge that ends its command; a 25LC040 that
 * stays selected after RDSR keeps shifting out its status register and
 * reads back zeroes. Clearing SEL at the end selects CS0 again for the next
 * command. The +0x08 "ChipSelectDisabled" register is not
 * used: WinCE never writes it and nothing establishes that it raises CS0.
 */
static void asic2_spi_release_cs(struct asic2_spi *spi)
{
	u8 control = asic2_spi_read(spi, ASIC2_SPI_CONTROL);

	asic2_spi_write(spi, ASIC2_SPI_CONTROL,
			control | ASIC2_SPI_CONTROL_SEL_CS1);
	asic2_spi_write(spi, ASIC2_SPI_CONTROL,
			control | ASIC2_SPI_CONTROL_SEL_CS1);
	asic2_spi_write(spi, ASIC2_SPI_CONTROL,
			control & ~ASIC2_SPI_CONTROL_SEL_CS1);
}

static void asic2_spi_set_cs(struct spi_device *sdev, bool inactive)
{
	struct asic2_spi *spi = spi_controller_get_devdata(sdev->controller);

	/*
	 * There is no separate assert operation: a chip select asserts as a
	 * side effect of the CONTROL write of the next transfer_one() byte.
	 * The SPI core passes the physical chip select level, so true is the
	 * inactive level of the active-low devices supported here.
	 */
	if (inactive)
		asic2_spi_release_cs(spi);
}

static int asic2_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *ctlr;
	struct asic2_spi *spi;
	struct resource *mem;
	int irq, ret;

	ctlr = devm_spi_alloc_host(dev, sizeof(*spi));
	if (!ctlr)
		return -ENOMEM;

	spi = spi_controller_get_devdata(ctlr);
	spi->dev = dev;
	init_completion(&spi->done);

	/*
	 * devm_ioremap() rather than devm_platform_ioremap_resource(), which
	 * would reserve the region exclusively: it is a sub-range of the
	 * window the MFD core has already reserved, so the request would
	 * fail with -EBUSY.
	 */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -ENODEV;
	spi->regs = devm_ioremap(dev, mem->start, resource_size(mem));
	if (!spi->regs)
		return -ENOMEM;

	/*
	 * Park CONTROL at an idle value (SPR(3), CS0, no SPE or SPIE) before
	 * either clock runs. The vendor's setup also writes CONTROL before the
	 * CX5 gate comes on. asic2_probe() sets CLOCK_Enable to EX0 alone, so
	 * CX5 is still off here, and without this write it would start the
	 * engine with whatever CONTROL held at reset, which is not documented.
	 * The register itself does not need CX5 to be written.
	 */
	asic2_spi_write(spi, ASIC2_SPI_CONTROL, ASIC2_SPI_CONTROL_SPR_EEPROM);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	spi->irq = irq;

	ret = devm_request_irq(dev, irq, asic2_spi_irq, 0, "asic2-spi", spi);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	/*
	 * The shared EX1 crystal, then this block's own CX5 gate on top of it.
	 * EX1 first, as in the vendor's setup; devm releases them in
	 * reverse, CX5 before EX1.
	 */
	ret = PTR_ERR_OR_ZERO(devm_clk_get_enabled(dev, "ex1"));
	if (ret)
		return dev_err_probe(dev, ret, "failed to get EX1 clock\n");
	ret = PTR_ERR_OR_ZERO(devm_clk_get_enabled(dev, NULL));
	if (ret)
		return dev_err_probe(dev, ret, "failed to get gate clock\n");

	/* Deselect at probe, the power-on state of the hardware */
	asic2_spi_release_cs(spi);

	ctlr->dev.of_node = dev->of_node;
	ctlr->bus_num = -1;
	ctlr->num_chipselect = 2;
	ctlr->mode_bits = SPI_CPOL;
	ctlr->bits_per_word_mask = SPI_BPW_MASK(8);
	ctlr->transfer_one = asic2_spi_transfer_one;
	ctlr->set_cs = asic2_spi_set_cs;

	return devm_spi_register_controller(dev, ctlr);
}

static struct platform_driver asic2_spi_driver = {
	.driver = {
		.name = "asic2-spi",
	},
	.probe	= asic2_spi_probe,
};
module_platform_driver(asic2_spi_driver);

MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("ASIC2 companion ASIC SPI master (HP iPAQ h38xx/h39xx)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:asic2-spi");
