// SPDX-License-Identifier: GPL-2.0-only
/*
 * IIO ADC driver for the general-purpose channels of the ASIC2 companion
 * ASIC's ADC (HP iPAQ h38xx/h39xx family): light sensor, Imin and battery
 * voltage (Vs/Mbat).
 *
 * The register layout (Multiplexer/ControlStatus/Data at 0x00/0x04/0x08)
 * matches include/linux/mfd/asic2.h. The driver maps its own copy of just
 * that window.
 *
 * The touchpanel X0/Y1 channels are not implemented: touch sampling needs
 * the four GPIO2 touchscreen bits driven in a plate-switching sequence
 * that is not covered here. Only the three plain single-ended channels are
 * wired.
 *
 * The conversion clock prescaler is 4, the value the manufacturer programs
 * for this ADC. The target conversion frequency and the per-step frequency
 * of the 4-bit prescaler field are not documented.
 *
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>

#include <linux/iio/iio.h>

#define ASIC2_ADC_MULTIPLEXER		0x00
#define ASIC2_ADC_CONTROL_STATUS	0x04
#define ASIC2_ADC_DATA			0x08

#define ASIC2_ADMUX(x)			((x) & 0x07)
#define ASIC2_ADMUX_CLKEN		BIT(3)

#define ASIC2_ADCSR_ADPS(x)		((x) & 0x0f)
#define ASIC2_ADCSR_FREE_RUN		BIT(4)
#define ASIC2_ADCSR_INT_ENABLE		BIT(5)
#define ASIC2_ADCSR_START		BIT(6)
#define ASIC2_ADCSR_ENABLE		BIT(7)

#define ASIC2_ADC_DATA_MASK		GENMASK(9, 0)	/* 10-bit conversion */

#define ASIC2_ADC_CONV_TIMEOUT_MS	50

enum asic2_adc_channel {
	ASIC2_ADC_LIGHTSENSOR,
	ASIC2_ADC_IMIN,
	ASIC2_ADC_VS_MBAT,
};

struct asic2_adc {
	struct device *dev;
	void __iomem *regs;
	struct completion done;

	/*
	 * Serializes the mux-select/start/wait/read sequence of
	 * asic2_adc_convert(): all channels share one set of conversion
	 * registers and one completion, so concurrent callers would clobber
	 * each other's pending wait and reprogram the mux mid-conversion,
	 * and one of them would time out. Callers are in process context.
	 */
	struct mutex convert_lock;
};

static inline u8 asic2_adc_read(struct asic2_adc *adc, u32 reg)
{
	return readb_relaxed(adc->regs + reg);
}

static inline void asic2_adc_write(struct asic2_adc *adc, u32 reg, u8 val)
{
	writeb_relaxed(val, adc->regs + reg);
}

static irqreturn_t asic2_adc_irq(int irq, void *data)
{
	struct asic2_adc *adc = data;

	complete(&adc->done);

	return IRQ_HANDLED;
}

static int asic2_adc_convert(struct asic2_adc *adc, unsigned int channel, u16 *val)
{
	unsigned long time_left;

	guard(mutex)(&adc->convert_lock);

	reinit_completion(&adc->done);

	asic2_adc_write(adc, ASIC2_ADC_MULTIPLEXER,
			ASIC2_ADMUX(channel) | ASIC2_ADMUX_CLKEN);
	asic2_adc_write(adc, ASIC2_ADC_CONTROL_STATUS,
			ASIC2_ADCSR_ENABLE | ASIC2_ADCSR_ADPS(4));
	asic2_adc_write(adc, ASIC2_ADC_CONTROL_STATUS,
			ASIC2_ADCSR_ENABLE | ASIC2_ADCSR_ADPS(4) |
			ASIC2_ADCSR_INT_ENABLE | ASIC2_ADCSR_START);

	time_left = wait_for_completion_timeout(&adc->done,
						msecs_to_jiffies(ASIC2_ADC_CONV_TIMEOUT_MS));
	if (!time_left)
		return -ETIMEDOUT;

	*val = (readw_relaxed(adc->regs + ASIC2_ADC_DATA)) & ASIC2_ADC_DATA_MASK;

	asic2_adc_write(adc, ASIC2_ADC_CONTROL_STATUS, 0);

	return 0;
}

static int asic2_adc_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			       int *val, int *val2, long mask)
{
	struct asic2_adc *adc = iio_priv(indio_dev);
	u16 data;
	int ret;

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	IIO_DEV_ACQUIRE_DIRECT_MODE(indio_dev, claim);
	if (IIO_DEV_ACQUIRE_FAILED(claim))
		return -EBUSY;

	ret = asic2_adc_convert(adc, chan->channel, &data);
	if (ret)
		return ret;

	*val = data;

	return IIO_VAL_INT;
}

#define ASIC2_ADC_CHAN(_idx, _name) {					\
	.type = IIO_VOLTAGE,						\
	.channel = (_idx),						\
	.indexed = 1,							\
	.scan_index = (_idx),						\
	.scan_type = {							\
		.sign = 'u',						\
		.realbits = 10,						\
		.storagebits = 16,					\
	},								\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.datasheet_name = (_name),					\
}

static const struct iio_chan_spec asic2_adc_channels[] = {
	ASIC2_ADC_CHAN(ASIC2_ADC_LIGHTSENSOR, "lightsensor"),
	ASIC2_ADC_CHAN(ASIC2_ADC_IMIN,        "imin"),
	ASIC2_ADC_CHAN(ASIC2_ADC_VS_MBAT,     "vs_mbat"),
};

static const struct iio_info asic2_adc_info = {
	.read_raw = asic2_adc_read_raw,
};

static int asic2_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct asic2_adc *adc;
	struct resource *mem;
	int irq, ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->dev = dev;
	init_completion(&adc->done);
	ret = devm_mutex_init(dev, &adc->convert_lock);
	if (ret)
		return ret;

	/*
	 * devm_ioremap() rather than devm_platform_ioremap_resource(), which
	 * would reserve the region exclusively: it is a sub-range of the
	 * window the MFD core has already reserved, so the request would
	 * fail with -EBUSY.
	 */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -ENODEV;
	adc->regs = devm_ioremap(dev, mem->start, resource_size(mem));
	if (!adc->regs)
		return -ENOMEM;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	/* Own peripheral gate, plus the shared EX1 crystal it needs on top */
	ret = PTR_ERR_OR_ZERO(devm_clk_get_enabled(dev, NULL));
	if (ret)
		return dev_err_probe(dev, ret, "failed to get gate clock\n");
	ret = PTR_ERR_OR_ZERO(devm_clk_get_enabled(dev, "ex1"));
	if (ret)
		return dev_err_probe(dev, ret, "failed to get EX1 clock\n");

	ret = devm_request_irq(dev, irq, asic2_adc_irq, 0, "asic2-adc", adc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	indio_dev->name = "asic2-adc";
	indio_dev->info = &asic2_adc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = asic2_adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(asic2_adc_channels);

	return devm_iio_device_register(dev, indio_dev);
}

static struct platform_driver asic2_adc_driver = {
	.driver = {
		.name = "asic2-adc",
	},
	.probe	= asic2_adc_probe,
};
module_platform_driver(asic2_adc_driver);

MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("ASIC2 companion ASIC ADC (HP iPAQ h38xx/h39xx)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:asic2-adc");
