/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Public entry points for embedding a PXA2xx SSP as a spi_controller from
 * outside drivers/spi/ -- e.g. a companion-ASIC driver that reaches its
 * SSP through a "port" phandle rather than binding its own of_match_table
 * to the raw SSP node (mirroring sound/soc/pxa/pxa-ssp.c's own
 * pxa_ssp_request_of()-based consumption, Documentation/devicetree/
 * bindings/sound/mrvl,pxa-ssp.txt). drivers/spi/spi-pxa2xx.c already
 * exports pxa2xx_spi_probe()/pxa2xx_spi_remove() through the namespaced
 * "SPI_PXA2xx" symbol namespace for exactly this kind of external caller;
 * this header is what was missing to declare them anywhere outside
 * drivers/spi/'s own private spi-pxa2xx.h.
 */

#ifndef __LINUX_SPI_PXA2XX_SPI_H
#define __LINUX_SPI_PXA2XX_SPI_H

#include <linux/dmaengine.h>
#include <linux/pxa2xx_ssp.h>

struct dev_pm_ops;
struct device;

/*
 * The platform data for SSP controller devices
 * (resides in device.platform_data).
 */
struct pxa2xx_spi_controller {
	u8 num_chipselect;
	u8 enable_dma;
	u8 dma_burst_size;
	bool is_target;

	/* DMA engine specific config */
	dma_filter_fn dma_filter;
	void *tx_param;
	void *rx_param;

	/* For non-PXA arches */
	struct ssp_device ssp;
};

int pxa2xx_spi_probe(struct device *dev, struct ssp_device *ssp,
		     struct pxa2xx_spi_controller *platform_info);
void pxa2xx_spi_remove(struct device *dev);

extern const struct dev_pm_ops pxa2xx_spi_pm_ops;

#endif /* __LINUX_SPI_PXA2XX_SPI_H */
