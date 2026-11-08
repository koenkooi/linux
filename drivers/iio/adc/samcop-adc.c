// SPDX-License-Identifier: GPL-2.0-only
/*
 * IIO ADC driver for the ADC, resistive touchscreen and light sensor
 * front-end inside the Samsung SAMCOP companion ASIC (HP iPAQ
 * h51xx/h54xx/h55xx).
 *
 * The block has the same CONTROL/TSC/DELAY/DATA0/DATA1 layout as HAMCOP's
 * (see hamcop-adc.c) but, unlike the rest of SAMCOP, is accessed 16 bits
 * wide: no register uses a bit above 15, and CONTROL and TSC are only four
 * bytes apart, so a 32-bit access would straddle two registers.
 *
 * mainline's generic resistive-adc-touch driver is the touchscreen
 * consumer, reached through the "io-channels" property of its node. This
 * driver is a triggerless IIO buffer producer: the ADCTS pen-contact
 * interrupt starts a work item that samples X/Y/pen state every
 * SAMCOP_ADC_POLL_INTERVAL while the pen stays down and pushes one sample
 * per iteration. The ADC-complete interrupt never touches the buffer.
 *
 * The block has no Z1/Z2 pressure path: the CONTROL SEL mux offers only
 * AIN0-3 and the panel plates. Z1/Z2 are synthesized from the pen state,
 * as in hamcop-adc.c.
 *
 * AIN0 is the ambient light sensor on the h5000; AIN1 and AIN2 are
 * exposed as plain voltage channels.
 *
 * The sensor has its own switched supply, the optional "light-sensor"
 * regulator. Runtime PM of this device gates only that supply, and only
 * AIN0 reads take a runtime PM reference: touch sampling never powers
 * the sensor. The supply's settle time is the regulator's startup delay,
 * so it lives in the devicetree. The autosuspend delay is longer than the
 * factory firmware's 3 s polling period, so a polling client keeps the
 * sensor powered and pays the settle time once, as that firmware does.
 *
 * Each AIN0 read is the factory firmware's sequence: five conversions,
 * the first two discarded, the truncated mean of the last three returned.
 * The value is raw counts, 0..1023, with no scale: the firmware maps it
 * to backlight levels through a fixed table and has no lux conversion.
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
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#include <linux/iio/buffer.h>
#include <linux/iio/driver.h>
#include <linux/iio/iio.h>

/* Offsets relative to this driver's own mapping of the ADC window. */
#define SAMCOP_ADC_CONTROL			0x00
#define SAMCOP_ADC_TSC				0x04
#define SAMCOP_ADC_DELAY			0x08
#define SAMCOP_ADC_DATA0			0x0c
#define SAMCOP_ADC_DATA1			0x10

#define SAMCOP_ADC_CONTROL_FORCE_START		BIT(0)
#define SAMCOP_ADC_CONTROL_STANDBY		BIT(2)
#define SAMCOP_ADC_CONTROL_SEL_SHIFT		3
#define SAMCOP_ADC_CONTROL_SEL_AIN3		(0x3 << SAMCOP_ADC_CONTROL_SEL_SHIFT)
#define SAMCOP_ADC_CONTROL_PRESCALER_SHIFT	6
#define SAMCOP_ADC_CONTROL_PRESCALER_MAX	GENMASK(7, 0)	/* 8-bit field */
#define SAMCOP_ADC_CONTROL_PRESCALER_ENABLE	BIT(14)

#define SAMCOP_ADC_TSC_XYPST_INTERRUPT		0x3
#define SAMCOP_ADC_TSC_XYPST_NONE		0x0
#define SAMCOP_ADC_TSC_AUTO_MODE		BIT(2)
#define SAMCOP_ADC_TSC_PULL_UP_DISABLE		BIT(3)
#define SAMCOP_ADC_TSC_XP_SEN			BIT(4)
#define SAMCOP_ADC_TSC_YP_SEN			BIT(6)
#define SAMCOP_ADC_TSC_YM_SEN			BIT(7)

/* Idle, "interrupt me when touched" mode -- 0xd3 in hackndev/master's
 * samcop_adc.c (SAMCOP_ADC_TSC_WAIT).
 */
#define SAMCOP_ADC_TSC_WAIT4INT			\
	(SAMCOP_ADC_TSC_XYPST_INTERRUPT | SAMCOP_ADC_TSC_XP_SEN | \
	 SAMCOP_ADC_TSC_YP_SEN | SAMCOP_ADC_TSC_YM_SEN)
/* Continuous auto X/Y-position-sense mode -- 0xdc in hackndev/master's
 * samcop_adc.c (SAMCOP_ADC_TSC_SAMPLE_TS).
 */
#define SAMCOP_ADC_TSC_SAMPLE_TOUCH			\
	(SAMCOP_ADC_TSC_AUTO_MODE | SAMCOP_ADC_TSC_PULL_UP_DISABLE | \
	 SAMCOP_ADC_TSC_XP_SEN | SAMCOP_ADC_TSC_YP_SEN | SAMCOP_ADC_TSC_YM_SEN)
/* Single-shot plain-AIN mode -- SAMCOP_ADC_TSC_SAMPLE_AX in the 2007 driver. */
#define SAMCOP_ADC_TSC_SAMPLE_AIN		SAMCOP_ADC_TSC_XYPST_NONE
/* Deepest safe idle state while nobody has the touchscreen open -- 0x58 in
 * hackndev/master's samcop_adc.c (SAMCOP_ADC_TSC_SUSPEND).
 */
#define SAMCOP_ADC_TSC_SUSPEND				\
	(SAMCOP_ADC_TSC_PULL_UP_DISABLE | SAMCOP_ADC_TSC_XP_SEN | \
	 SAMCOP_ADC_TSC_YP_SEN)

#define SAMCOP_ADC_DATA_MASK			GENMASK(9, 0)	/* 10-bit conversion */
#define SAMCOP_ADC_DATA_PEN_UP			BIT(15)

/* Target ADC clock -- hackndev/master's samcop_adc.c:36 ADC_FREQ, same
 * value HAMCOP's own driver targets.
 */
#define SAMCOP_ADC_TARGET_FREQ			2000000
#define SAMCOP_ADC_CONV_TIMEOUT_MS		50
#define SAMCOP_ADC_POLL_INTERVAL		msecs_to_jiffies(20)
/* hackndev/master's real h5400.c board data
 * (samcop_platform_data.samcop_adc_pdata.delay, h5400.c:642) -- 500, NOT
 * the h2200/HAMCOP driver's own board-specific 3000. Chip-internal-enough
 * to bake in directly since h5400.c is the only real board SAMCOP has ever
 * shipped on, same reasoning drivers/mfd/samcop.c's own
 * SAMCOP_PCMCIA_IM_DEFAULT comment gives for the equivalent situation.
 */
#define SAMCOP_ADC_DELAY_DEFAULT		500

/*
 * The factory firmware's light sensor read: 2 conversions thrown away,
 * then the mean of 3, presumably to let AIN0 settle after the mux change.
 */
#define SAMCOP_ADC_LIGHT_DISCARD		2
#define SAMCOP_ADC_LIGHT_SAMPLES		3
#define SAMCOP_ADC_LIGHT_AUTOSUSPEND_MS		5000

enum samcop_adc_channel {
	SAMCOP_ADC_X,
	SAMCOP_ADC_Y,
	SAMCOP_ADC_Z1,
	SAMCOP_ADC_Z2,
	SAMCOP_ADC_AIN0,	/* light sensor, h5400.c:132-149 */
	SAMCOP_ADC_AIN1,
	SAMCOP_ADC_AIN2,
	SAMCOP_ADC_NR_CHANNELS,
};

struct samcop_adc {
	struct device *dev;
	struct iio_dev *indio_dev;
	void __iomem *regs;
	struct clk *clk;
	struct regulator *light_supply;
	int irq_adcts;

	/* PRESCALER_ENABLE | prescaler value, computed once from adc->clk's
	 * rate in probe() -- ORed with FORCE_START and a channel SEL field
	 * for each individual conversion.
	 */
	u16 control_base;

	/*
	 * Serializes every conversion and the TSC writes around it. The
	 * touch poll chain and AIN reads share one converter and one
	 * completion, and an AIN read must be possible while the touchscreen
	 * buffer is enabled, so the IIO direct/buffer mode claim cannot be
	 * what keeps them apart.
	 */
	struct mutex lock;
	/* TSC mode to leave the block in between conversions. */
	u16 tsc_idle;

	struct completion done;
	struct delayed_work poll_work;
};

static inline u16 samcop_adc_read(struct samcop_adc *adc, u32 reg)
{
	return readw_relaxed(adc->regs + reg);
}

static inline void samcop_adc_write(struct samcop_adc *adc, u32 reg, u16 val)
{
	writew_relaxed(val, adc->regs + reg);
}

static irqreturn_t samcop_adc_conv_irq(int irq, void *data)
{
	struct samcop_adc *adc = data;

	complete(&adc->done);

	return IRQ_HANDLED;
}

/* Runs one full ADC conversion cycle: programs DELAY/TSC/CONTROL, waits
 * for SAMCOP_IRQ_ADC's completion, returns the raw DATA0/DATA1 pair.
 * Shared by the touch poll chain and direct reads; the caller holds
 * adc->lock, so only one conversion is ever outstanding and the shared
 * completion is unambiguous.
 */
static int samcop_adc_convert(struct samcop_adc *adc, u16 tsc, u16 sel,
			      u16 *data0, u16 *data1)
{
	unsigned long time_left;

	lockdep_assert_held(&adc->lock);

	reinit_completion(&adc->done);

	samcop_adc_write(adc, SAMCOP_ADC_DELAY, SAMCOP_ADC_DELAY_DEFAULT);
	samcop_adc_write(adc, SAMCOP_ADC_TSC, tsc);
	samcop_adc_write(adc, SAMCOP_ADC_CONTROL,
			 adc->control_base | sel | SAMCOP_ADC_CONTROL_FORCE_START);

	time_left = wait_for_completion_timeout(&adc->done,
						msecs_to_jiffies(SAMCOP_ADC_CONV_TIMEOUT_MS));
	if (!time_left)
		return -ETIMEDOUT;

	*data0 = samcop_adc_read(adc, SAMCOP_ADC_DATA0);
	*data1 = samcop_adc_read(adc, SAMCOP_ADC_DATA1);

	return 0;
}

static void samcop_adc_idle(struct samcop_adc *adc, u16 tsc)
{
	samcop_adc_write(adc, SAMCOP_ADC_TSC, tsc);
}

/* One touch sample: converts X/Y in continuous auto-position mode, then
 * pushes an {x, y, z1, z2} sample to any attached IIO buffer. Returns
 * true while the pen is still down (caller keeps polling), false once
 * released (caller re-arms WAIT4INT instead).
 *
 * z1/z2: SAMCOP has no real pressure-sensing path (see this file's header
 * comment), so both channels always carry the same value -- 1 while
 * touching, 0 once released -- rather than two independent resistance
 * readings. Fed into resistive-adc-touch's Rt formula
 * (Rt = (z2-z1)*x_plate_ohms/16*x/z1/256), z2-z1 is then always exactly 0
 * while touching, so Rt collapses to 0 regardless of x_plate_ohms,
 * comfortably clearing its pressure_min gate and reporting a touch; while
 * released, z1 == 0 makes resistive-adc-touch skip the Rt computation
 * entirely ("if (likely(x && z1))") and report release instead. This
 * reproduces hackndev/master's own 0/100 pen-up/pen-down proxy value
 * through the shape resistive-adc-touch expects, without a division by
 * zero or a meaningless negative Rt anywhere in the path.
 */
static bool samcop_adc_sample_touch(struct samcop_adc *adc)
{
	u16 buf[4]; /* x, y, z1, z2 -- ascending scan_index order */
	u16 data0, data1;
	bool down;

	guard(mutex)(&adc->lock);

	if (samcop_adc_convert(adc, SAMCOP_ADC_TSC_SAMPLE_TOUCH,
			       SAMCOP_ADC_CONTROL_SEL_AIN3, &data0, &data1)) {
		dev_warn_ratelimited(adc->dev, "touch conversion timed out\n");
		return false;
	}

	down = !(data0 & SAMCOP_ADC_DATA_PEN_UP) && !(data1 & SAMCOP_ADC_DATA_PEN_UP);

	buf[0] = down ? data0 & SAMCOP_ADC_DATA_MASK : 0;
	buf[1] = down ? data1 & SAMCOP_ADC_DATA_MASK : 0;
	buf[2] = down ? 1 : 0;
	buf[3] = down ? 1 : 0;

	iio_push_to_buffers(adc->indio_dev, buf);

	return down;
}

static void samcop_adc_poll_work(struct work_struct *work)
{
	struct samcop_adc *adc = container_of(work, struct samcop_adc, poll_work.work);

	if (samcop_adc_sample_touch(adc)) {
		queue_delayed_work(system_freezable_wq, &adc->poll_work,
				   SAMCOP_ADC_POLL_INTERVAL);
		return;
	}

	guard(mutex)(&adc->lock);
	samcop_adc_idle(adc, SAMCOP_ADC_TSC_WAIT4INT);
}

static irqreturn_t samcop_adc_pen_irq(int irq, void *data)
{
	struct samcop_adc *adc = data;

	queue_delayed_work(system_freezable_wq, &adc->poll_work, 0);

	return IRQ_HANDLED;
}

static int samcop_adc_buffer_postenable(struct iio_dev *indio_dev)
{
	struct samcop_adc *adc = iio_priv(indio_dev);

	scoped_guard(mutex, &adc->lock) {
		adc->tsc_idle = SAMCOP_ADC_TSC_WAIT4INT;
		samcop_adc_idle(adc, adc->tsc_idle);
	}
	enable_irq(adc->irq_adcts);

	return 0;
}

static int samcop_adc_buffer_predisable(struct iio_dev *indio_dev)
{
	struct samcop_adc *adc = iio_priv(indio_dev);

	disable_irq(adc->irq_adcts);
	/* Not under adc->lock: the work item takes it. */
	cancel_delayed_work_sync(&adc->poll_work);

	guard(mutex)(&adc->lock);
	adc->tsc_idle = SAMCOP_ADC_TSC_SUSPEND;
	samcop_adc_idle(adc, adc->tsc_idle);

	return 0;
}

static const struct iio_buffer_setup_ops samcop_adc_buffer_setup_ops = {
	.postenable	= samcop_adc_buffer_postenable,
	.predisable	= samcop_adc_buffer_predisable,
};

/*
 * Converts analog input @ain @discard + @samples times and returns the
 * mean of the last @samples, truncated as the factory firmware does.
 */
static int samcop_adc_read_ain(struct samcop_adc *adc, unsigned int ain,
			       unsigned int discard, unsigned int samples,
			       int *val)
{
	u16 sel = ain << SAMCOP_ADC_CONTROL_SEL_SHIFT;
	unsigned int i, sum = 0;
	u16 data0, data1;
	int ret = 0;

	guard(mutex)(&adc->lock);

	for (i = 0; i < discard + samples; i++) {
		ret = samcop_adc_convert(adc, SAMCOP_ADC_TSC_SAMPLE_AIN, sel,
					 &data0, &data1);
		if (ret)
			break;

		if (i >= discard)
			sum += data0 & SAMCOP_ADC_DATA_MASK;
	}

	samcop_adc_idle(adc, adc->tsc_idle);
	if (ret)
		return ret;

	*val = sum / samples;

	return IIO_VAL_INT;
}

static int samcop_adc_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct samcop_adc *adc = iio_priv(indio_dev);
	u16 data0, data1;
	int ret;

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	switch (chan->address) {
	case SAMCOP_ADC_X:
	case SAMCOP_ADC_Y:
	case SAMCOP_ADC_Z1:
	case SAMCOP_ADC_Z2: {
		/*
		 * A one-shot touch read makes no sense while the poll chain
		 * owns the touch sense lines, so keep refusing it then.
		 */
		IIO_DEV_ACQUIRE_DIRECT_MODE(indio_dev, claim);
		if (IIO_DEV_ACQUIRE_FAILED(claim))
			return -EBUSY;

		guard(mutex)(&adc->lock);
		ret = samcop_adc_convert(adc, SAMCOP_ADC_TSC_SAMPLE_TOUCH,
					 SAMCOP_ADC_CONTROL_SEL_AIN3, &data0, &data1);
		samcop_adc_idle(adc, adc->tsc_idle);
		if (ret)
			break;

		switch (chan->address) {
		case SAMCOP_ADC_X:
			*val = data0 & SAMCOP_ADC_DATA_MASK;
			break;
		case SAMCOP_ADC_Y:
			*val = data1 & SAMCOP_ADC_DATA_MASK;
			break;
		default: /* Z1/Z2 -- see this file's header comment */
			*val = (data0 & SAMCOP_ADC_DATA_PEN_UP) ||
			       (data1 & SAMCOP_ADC_DATA_PEN_UP) ? 0 : 1;
			break;
		}
		ret = IIO_VAL_INT;
		break;
	}
	/*
	 * No direct mode claim for the AIN channels: they interleave with the
	 * touch poll chain under adc->lock, so they keep working while the
	 * touchscreen is open.
	 */
	case SAMCOP_ADC_AIN0:
		/* Before the lock: the supply's settle time must not stall touch. */
		ret = pm_runtime_resume_and_get(adc->dev);
		if (ret)
			return ret;

		ret = samcop_adc_read_ain(adc, 0, SAMCOP_ADC_LIGHT_DISCARD,
					  SAMCOP_ADC_LIGHT_SAMPLES, val);
		pm_runtime_put_autosuspend(adc->dev);
		break;
	case SAMCOP_ADC_AIN1:
	case SAMCOP_ADC_AIN2:
		ret = samcop_adc_read_ain(adc, chan->address - SAMCOP_ADC_AIN0,
					  0, 1, val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

#define SAMCOP_ADC_CHAN(_addr, _type, _idx, _name) {			\
	.type = (_type),						\
	.indexed = 1,							\
	.channel = (_idx),						\
	.address = (_addr),						\
	.scan_index = (_addr),						\
	.scan_type = {							\
		.sign = 'u',						\
		.realbits = 10,						\
		.storagebits = 16,					\
	},								\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.datasheet_name = (_name),					\
}

static const struct iio_chan_spec samcop_adc_channels[] = {
	SAMCOP_ADC_CHAN(SAMCOP_ADC_X,    IIO_POSITIONRELATIVE, 0, "x"),
	SAMCOP_ADC_CHAN(SAMCOP_ADC_Y,    IIO_POSITIONRELATIVE, 1, "y"),
	SAMCOP_ADC_CHAN(SAMCOP_ADC_Z1,   IIO_PRESSURE,         0, "z1"),
	SAMCOP_ADC_CHAN(SAMCOP_ADC_Z2,   IIO_PRESSURE,         1, "z2"),
	/* AIN0: real H5000 light sensor, h5400.c:132-149; raw counts only. */
	SAMCOP_ADC_CHAN(SAMCOP_ADC_AIN0, IIO_LIGHT,            0, "light"),
	SAMCOP_ADC_CHAN(SAMCOP_ADC_AIN1, IIO_VOLTAGE,          0, "ain1"),
	SAMCOP_ADC_CHAN(SAMCOP_ADC_AIN2, IIO_VOLTAGE,          1, "ain2"),
};

static const struct iio_info samcop_adc_info = {
	.read_raw = samcop_adc_read_raw,
};

static int samcop_adc_runtime_suspend(struct device *dev)
{
	struct samcop_adc *adc = iio_priv(dev_get_drvdata(dev));

	if (!adc->light_supply)
		return 0;

	return regulator_disable(adc->light_supply);
}

static int samcop_adc_runtime_resume(struct device *dev)
{
	struct samcop_adc *adc = iio_priv(dev_get_drvdata(dev));

	if (!adc->light_supply)
		return 0;

	return regulator_enable(adc->light_supply);
}

static DEFINE_RUNTIME_DEV_PM_OPS(samcop_adc_pm_ops, samcop_adc_runtime_suspend,
				 samcop_adc_runtime_resume, NULL);

static void samcop_adc_light_off(void *data)
{
	regulator_disable(data);
}

static int samcop_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct samcop_adc *adc;
	struct resource *mem;
	unsigned long rate, prescaler;
	int irq_adc, ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->dev = dev;
	adc->indio_dev = indio_dev;
	platform_set_drvdata(pdev, indio_dev);
	adc->tsc_idle = SAMCOP_ADC_TSC_SUSPEND;
	init_completion(&adc->done);
	INIT_DELAYED_WORK(&adc->poll_work, samcop_adc_poll_work);

	/*
	 * platform_get_resource() + devm_ioremap(), not
	 * devm_platform_ioremap_resource() -- the latter reserves the
	 * region exclusively via devm_request_mem_region(), which always
	 * loses to drivers/mfd/samcop.c's own devm_ioremap_resource() of
	 * the entire SAMCOP MMIO window with -EBUSY. Same bug class fixed
	 * for real on physical h2210 hardware for pxa27x/pxa25x pinctrl,
	 * hamcop-sdi (a6f7616fd9a3) and leds-hamcop (198098879172) -- see
	 * drivers/mfd/hamcop.c's own header comment for the general
	 * pattern. SAMCOP_ADC_BASE (0x80000) is a sub-range of that same
	 * already-claimed window.
	 */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -ENODEV;
	adc->regs = devm_ioremap(dev, mem->start, resource_size(mem));
	if (!adc->regs)
		return -ENOMEM;

	ret = devm_mutex_init(dev, &adc->lock);
	if (ret)
		return ret;

	irq_adc = platform_get_irq(pdev, 0);
	if (irq_adc < 0)
		return irq_adc;

	adc->irq_adcts = platform_get_irq(pdev, 1);
	if (adc->irq_adcts < 0)
		return adc->irq_adcts;

	adc->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(adc->clk))
		return dev_err_probe(dev, PTR_ERR(adc->clk), "failed to get clock\n");

	adc->light_supply = devm_regulator_get_optional(dev, "light-sensor");
	if (IS_ERR(adc->light_supply)) {
		if (PTR_ERR(adc->light_supply) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(adc->light_supply),
					     "failed to get light sensor supply\n");
		adc->light_supply = NULL;
	}

	rate = clk_get_rate(adc->clk);
	if (!rate)
		return dev_err_probe(dev, -EINVAL, "clock rate is 0\n");

	prescaler = rate / SAMCOP_ADC_TARGET_FREQ;
	prescaler = clamp_t(unsigned long, prescaler, 1, SAMCOP_ADC_CONTROL_PRESCALER_MAX + 1) - 1;
	adc->control_base = SAMCOP_ADC_CONTROL_PRESCALER_ENABLE |
			     (prescaler << SAMCOP_ADC_CONTROL_PRESCALER_SHIFT);

	/* Safe, low-current idle state until a consumer's buffer starts
	 * polling (samcop_adc_buffer_postenable() switches to WAIT4INT).
	 */
	samcop_adc_write(adc, SAMCOP_ADC_CONTROL,
			 adc->control_base | SAMCOP_ADC_CONTROL_STANDBY);
	samcop_adc_idle(adc, adc->tsc_idle);

	ret = devm_request_irq(dev, irq_adc, samcop_adc_conv_irq, 0,
			       "samcop-adc", adc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request ADC IRQ\n");

	/*
	 * NO_AUTOEN: the pen-detect line only matters while a consumer has
	 * the touchscreen buffer open (see samcop_adc_buffer_postenable()/
	 * _predisable()) -- masked at request time keeps SAMCOP from waking
	 * this driver over an unopened touchscreen, PDA battery life.
	 */
	ret = devm_request_irq(dev, adc->irq_adcts, samcop_adc_pen_irq,
			       IRQF_NO_AUTOEN, "samcop-adc-pen", adc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request pen IRQ\n");

	/* The device starts runtime-suspended: light sensor supply off. */
	pm_runtime_set_autosuspend_delay(dev, SAMCOP_ADC_LIGHT_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	/* Without runtime PM nothing would ever switch the sensor on. */
	if (!IS_ENABLED(CONFIG_PM) && adc->light_supply) {
		ret = regulator_enable(adc->light_supply);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to enable light sensor supply\n");

		ret = devm_add_action_or_reset(dev, samcop_adc_light_off,
					       adc->light_supply);
		if (ret)
			return ret;
	}

	indio_dev->name = "samcop-adc";
	indio_dev->info = &samcop_adc_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	indio_dev->setup_ops = &samcop_adc_buffer_setup_ops;
	indio_dev->channels = samcop_adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(samcop_adc_channels);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id samcop_adc_of_match[] = {
	{ .compatible = "samsung,samcop-adc" },
	{ }
};
MODULE_DEVICE_TABLE(of, samcop_adc_of_match);

static struct platform_driver samcop_adc_driver = {
	.driver = {
		.name = "samcop-adc",
		.of_match_table = samcop_adc_of_match,
		.pm = pm_ptr(&samcop_adc_pm_ops),
	},
	.probe	= samcop_adc_probe,
};
module_platform_driver(samcop_adc_driver);

MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("SAMCOP ADC, resistive touchscreen and light sensor front-end (HP iPAQ h5xxx)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:samcop-adc");
