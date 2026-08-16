// SPDX-License-Identifier: GPL-2.0-only
/*
 * IIO ADC driver for the ADC + resistive touchscreen front-end inside the
 * Samsung HAMCOP companion ASIC (HP iPAQ h2200/h2210/h2212/h2215).
 *
 * Register layout (CONTROL/TSC/DELAY/DATA0/DATA1 at 0x00/0x04/0x08/0x0c/
 * 0x10) is byte-for-byte identical to the since-removed mainline
 * drivers/input/touchscreen/s3c2410_ts.c --
 * real confirmation that this block is S3C24xx-class silicon.
 * The old out-of-tree samcop_adc.c (H5000-era, same IP) is
 * the reference for the 2007 register-level conversion sequence this
 * driver's hamcop_adc_convert_*() helpers are built from; the composite
 * TSC values below reproduce that driver's
 * SAMCOP_ADC_TSC_WAIT/_SAMPLE_TS/_SUSPEND macros bit-for-bit
 * (cross-checked against the "resets to 0x58"/"0xd3" comments in
 * the old out-of-tree samcop-adc.h).
 *
 * Shape: mainline's generic drivers/input/touchscreen/resistive-adc-touch.c
 * ("grts") is the touchscreen consumer, not a bespoke HAMCOP touchscreen
 * driver. grts drives itself entirely through IIO's push-callback-buffer API
 * (iio_channel_get_all_cb()/iio_channel_start_all_cb()), so this driver
 * has to be a real triggerless IIO buffer producer, not just a set of
 * IIO_CHAN_INFO_RAW channels: HAMCOP_IRQ_ADCTS (the block's own
 * pen-contact interrupt) starts a poll_work chain that samples X/Y/pen
 * state at HAMCOP_ADC_POLL_INTERVAL while the pen stays down, and pushes
 * one sample per iteration via iio_push_to_buffers(). The IRQ handler
 * itself never touches the buffer directly -- drivers/iio/adc/
 * at91-sama5d2_adc.c's at91_adc_touch_data_handler() comment explains why
 * (running an unrelated driver's callback from this driver's own IRQ
 * context is worth avoiding), and this driver follows the same
 * defer-to-a-work-item shape.
 *
 * HAMCOP has no distinct Z1/Z2 pressure-sensing path: the CONTROL
 * register's 3-bit SEL mux only ever offers AIN0-3/XP/XM/YP/YM, and the
 * 2007 driver's own samcop_adc_read_sense() never derives a real
 * resistance pair either -- for a Z1/Z2 request it re-reads the exact
 * same DATA0/DATA1 X/Y conversion and collapses it to a bare 0/100
 * pen-up/pen-down flag. This driver is honest about that limitation
 * rather than inventing hardware that isn't there: see
 * hamcop_adc_sample_touch()'s comment for how z1/z2 are synthesized so
 * that resistive-adc-touch's Rt formula still resolves to the intended
 * touch/no-touch result despite carrying no real pressure information.
 *
 * AIN0-2 are genuine general-purpose HAMCOP ADC inputs (the h5000 wires
 * AIN0 to a light sensor) but nothing on the h2200 is known to use any of
 * them -- h2200's battery telemetry comes entirely from the DS1WM/DS2760
 * one-wire path (drivers/w1/masters/ds1wm.c + ds2760_battery.c), not this
 * ADC. They are exposed here as plain "ain0"/"ain1"/"ain2" IIO_VOLTAGE
 * channels on the real hardware capability, without inventing a "battery"
 * label this board has no evidence for.
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
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include <linux/iio/buffer.h>
#include <linux/iio/driver.h>
#include <linux/iio/iio.h>
#include <linux/iio/machine.h>

/* Register offsets, relative to this driver's own ioremap of just the
 * HAMCOP_ADC_BASE/HAMCOP_ADC_SIZE window (see include/linux/mfd/hamcop.h)
 * -- unlike drivers/mfd/hamcop.c's own GPIO/IC/CPM decodes, which share
 * that driver's single MFD-wide mapping, this block gets its own separate
 * platform_device resource and hence its own 0-based offsets.
 */
#define HAMCOP_ADC_CONTROL			0x00
#define HAMCOP_ADC_TSC				0x04
#define HAMCOP_ADC_DELAY			0x08
#define HAMCOP_ADC_DATA0			0x0c
#define HAMCOP_ADC_DATA1			0x10

#define HAMCOP_ADC_CONTROL_FORCE_START		BIT(0)
#define HAMCOP_ADC_CONTROL_STANDBY		BIT(2)
#define HAMCOP_ADC_CONTROL_SEL_SHIFT		3
#define HAMCOP_ADC_CONTROL_SEL_AIN3		(0x3 << HAMCOP_ADC_CONTROL_SEL_SHIFT)
#define HAMCOP_ADC_CONTROL_PRESCALER_SHIFT	6
#define HAMCOP_ADC_CONTROL_PRESCALER_MAX	GENMASK(7, 0)	/* 8-bit field */
#define HAMCOP_ADC_CONTROL_PRESCALER_ENABLE	BIT(14)
#define HAMCOP_ADC_CONTROL_ECFLG		BIT(15)	/* end-of-conversion, read-only */

#define HAMCOP_ADC_TSC_XYPST_INTERRUPT		0x3
#define HAMCOP_ADC_TSC_XYPST_NONE		0x0
#define HAMCOP_ADC_TSC_AUTO_MODE		BIT(2)
#define HAMCOP_ADC_TSC_PULL_UP_DISABLE		BIT(3)
#define HAMCOP_ADC_TSC_XP_SEN			BIT(4)
#define HAMCOP_ADC_TSC_YP_SEN			BIT(6)
#define HAMCOP_ADC_TSC_YM_SEN			BIT(7)

/* Idle, "interrupt me when touched" mode -- 0xd3 in the 2007 driver. */
#define HAMCOP_ADC_TSC_WAIT4INT			\
	(HAMCOP_ADC_TSC_XYPST_INTERRUPT | HAMCOP_ADC_TSC_XP_SEN | \
	 HAMCOP_ADC_TSC_YP_SEN | HAMCOP_ADC_TSC_YM_SEN)
/* Continuous auto X/Y-position-sense mode -- 0xdc in the 2007 driver. */
#define HAMCOP_ADC_TSC_SAMPLE_TOUCH			\
	(HAMCOP_ADC_TSC_AUTO_MODE | HAMCOP_ADC_TSC_PULL_UP_DISABLE | \
	 HAMCOP_ADC_TSC_XP_SEN | HAMCOP_ADC_TSC_YP_SEN | HAMCOP_ADC_TSC_YM_SEN)
/* Single-shot plain-AIN mode. */
#define HAMCOP_ADC_TSC_SAMPLE_AIN		HAMCOP_ADC_TSC_XYPST_NONE
/* Deepest safe idle state while nobody has the touchscreen open --
 * 0x58 in the 2007 driver.
 */
#define HAMCOP_ADC_TSC_SUSPEND				\
	(HAMCOP_ADC_TSC_PULL_UP_DISABLE | HAMCOP_ADC_TSC_XP_SEN | \
	 HAMCOP_ADC_TSC_YP_SEN)

#define HAMCOP_ADC_DATA_MASK			GENMASK(9, 0)	/* 10-bit conversion */
#define HAMCOP_ADC_DATA_PEN_UP			BIT(15)

/* Target ADC clock, matching the old out-of-tree ADC_FREQ. */
#define HAMCOP_ADC_TARGET_FREQ			2000000
#define HAMCOP_ADC_CONV_TIMEOUT_MS		50
#define HAMCOP_ADC_POLL_INTERVAL		msecs_to_jiffies(20)
/* The h2200-specific ADC delay value, not the generic 500 default --
 * board-specific tuning baked in directly since HAMCOP has no other known
 * board (see drivers/mfd/hamcop.c's HAMCOP_PLLCONTROL_DEFAULT comment for
 * the same reasoning applied there).
 */
#define HAMCOP_ADC_DELAY_DEFAULT		3000

enum hamcop_adc_channel {
	HAMCOP_ADC_X,
	HAMCOP_ADC_Y,
	HAMCOP_ADC_Z1,
	HAMCOP_ADC_Z2,
	HAMCOP_ADC_AIN0,
	HAMCOP_ADC_AIN1,
	HAMCOP_ADC_AIN2,
	HAMCOP_ADC_NR_CHANNELS,
};

struct hamcop_adc {
	struct device *dev;
	struct iio_dev *indio_dev;
	void __iomem *regs;
	struct clk *clk;
	int irq_adcts;

	/* PRESCALER_ENABLE | prescaler value, computed once from adc->clk's
	 * rate in probe() -- ORed with FORCE_START and a channel SEL field
	 * for each individual conversion.
	 */
	u16 control_base;

	struct completion done;
	struct delayed_work poll_work;

	/*
	 * Written only from poll_work and, for pen_irq_masked, from the pen
	 * hardirq; buffer_predisable() reads it after disable_irq() and
	 * cancel_delayed_work_sync() have quiesced both, so no locking.
	 */
	bool pen_irq_masked;
};

static inline u16 hamcop_adc_read(struct hamcop_adc *adc, u32 reg)
{
	return readw_relaxed(adc->regs + reg);
}

static inline void hamcop_adc_write(struct hamcop_adc *adc, u32 reg, u16 val)
{
	writew_relaxed(val, adc->regs + reg);
}

static irqreturn_t hamcop_adc_conv_irq(int irq, void *data)
{
	struct hamcop_adc *adc = data;

	complete(&adc->done);

	return IRQ_HANDLED;
}

/* Runs one full ADC conversion cycle: programs DELAY/TSC/CONTROL, waits
 * for HAMCOP_IRQ_ADC's completion, returns the raw DATA0/DATA1 pair.
 * Shared by the touch poll chain and direct AIN reads -- IIO's own
 * direct/buffer mode-claim mutual exclusion (see hamcop_adc_read_raw())
 * is what keeps those two callers from ever racing each other here.
 */
static int hamcop_adc_convert(struct hamcop_adc *adc, u16 tsc, u16 sel,
			      u16 *data0, u16 *data1)
{
	unsigned long time_left;

	reinit_completion(&adc->done);

	hamcop_adc_write(adc, HAMCOP_ADC_DELAY, HAMCOP_ADC_DELAY_DEFAULT);
	hamcop_adc_write(adc, HAMCOP_ADC_TSC, tsc);
	hamcop_adc_write(adc, HAMCOP_ADC_CONTROL,
			 adc->control_base | sel | HAMCOP_ADC_CONTROL_FORCE_START);

	time_left = wait_for_completion_timeout(&adc->done,
						msecs_to_jiffies(HAMCOP_ADC_CONV_TIMEOUT_MS));
	if (!time_left)
		return -ETIMEDOUT;

	*data0 = hamcop_adc_read(adc, HAMCOP_ADC_DATA0);
	*data1 = hamcop_adc_read(adc, HAMCOP_ADC_DATA1);

	/* If the completion ever runs ahead of the conversion, DATA0/DATA1
	 * (and bit 15) are one sample stale.
	 */
	if (!(hamcop_adc_read(adc, HAMCOP_ADC_CONTROL) & HAMCOP_ADC_CONTROL_ECFLG))
		dev_dbg(adc->dev,
			"completion IRQ arrived with ECFLG clear -- conversion data may be stale\n");

	return 0;
}

static void hamcop_adc_idle(struct hamcop_adc *adc, u16 tsc)
{
	hamcop_adc_write(adc, HAMCOP_ADC_TSC, tsc);
}

static void hamcop_adc_arm_pen_irq(struct hamcop_adc *adc)
{
	hamcop_adc_idle(adc, HAMCOP_ADC_TSC_WAIT4INT);
	if (adc->pen_irq_masked) {
		adc->pen_irq_masked = false;
		enable_irq(adc->irq_adcts);
	}
}

/* One touch sample: converts X/Y in continuous auto-position mode, then
 * pushes an {x, y, z1, z2} sample to any attached IIO buffer. Returns
 * true while the pen is still down (caller keeps polling), false once
 * released (caller re-arms WAIT4INT instead).
 *
 * z1/z2: HAMCOP has no real pressure-sensing path (see this file's header
 * comment), so both channels always carry the same value -- 1 while
 * touching, 0 once released -- rather than two independent resistance
 * readings. Fed into resistive-adc-touch's Rt formula
 * (Rt = (z2-z1)*x_plate_ohms/16*x/z1/256), z2-z1 is then always exactly 0
 * while touching, so Rt collapses to 0 regardless of x_plate_ohms,
 * comfortably clearing its pressure_min gate and reporting a touch; while
 * released, z1 == 0 makes resistive-adc-touch skip the Rt computation
 * entirely ("if (likely(x && z1))") and report release instead. This
 * reproduces the 2007 driver's own 0/100 pen-up/pen-down proxy value
 * through the shape resistive-adc-touch expects, without a division by
 * zero or a meaningless negative Rt anywhere in the path.
 */
static bool hamcop_adc_sample_touch(struct hamcop_adc *adc)
{
	u16 buf[4]; /* x, y, z1, z2 -- ascending scan_index order */
	u16 data0, data1;
	bool down = false;

	/*
	 * ADCDAT0/1 bit 15 is defined by the datasheet only "at Waiting for
	 * Interrupt Mode", and ADCTSC note 1 requires XP_SEN=1 with PULL_UP=0
	 * (the XP pull-up the pen detector works through) while waiting.
	 * SAMPLE_TOUCH disables that pull-up and leaves XP undriven, so bit 15
	 * out of the conversion below carries no pen state. Read it here, with
	 * the block still parked in WAIT4INT, and re-park it after every sample.
	 */
	down = !(hamcop_adc_read(adc, HAMCOP_ADC_DATA0) & HAMCOP_ADC_DATA_PEN_UP);

	if (hamcop_adc_convert(adc, HAMCOP_ADC_TSC_SAMPLE_TOUCH,
			       HAMCOP_ADC_CONTROL_SEL_AIN3, &data0, &data1)) {
		dev_warn_ratelimited(adc->dev, "touch conversion timed out\n");
		/* Without a release here the input layer keeps BTN_TOUCH=1. */
		memset(buf, 0, sizeof(buf));
		iio_push_to_buffers(adc->indio_dev, buf);
		return false;
	}

	buf[0] = down ? data0 & HAMCOP_ADC_DATA_MASK : 0;
	buf[1] = down ? data1 & HAMCOP_ADC_DATA_MASK : 0;
	buf[2] = down ? 1 : 0;
	buf[3] = down ? 1 : 0;

	iio_push_to_buffers(adc->indio_dev, buf);

	return down;
}

static void hamcop_adc_poll_work(struct work_struct *work)
{
	struct hamcop_adc *adc = container_of(work, struct hamcop_adc, poll_work.work);

	if (hamcop_adc_sample_touch(adc)) {
		/*
		 * Parking in WAIT4INT re-asserts INT_WKU every interval while
		 * the pen is down; safe only because ADCTS stays masked for
		 * the whole chain.
		 */
		hamcop_adc_idle(adc, HAMCOP_ADC_TSC_WAIT4INT);
		queue_delayed_work(system_freezable_wq, &adc->poll_work,
				   HAMCOP_ADC_POLL_INTERVAL);
		return;
	}

	hamcop_adc_arm_pen_irq(adc);
}

static irqreturn_t hamcop_adc_pen_irq(int irq, void *data)
{
	struct hamcop_adc *adc = data;

	/*
	 * Unmasked, the WAIT4INT parking in poll_work would re-enter the
	 * demux drain loop of hamcop.c every interval. Mask here and unmask
	 * when the chain ends.
	 */
	if (!adc->pen_irq_masked) {
		adc->pen_irq_masked = true;
		disable_irq_nosync(adc->irq_adcts);
	}

	queue_delayed_work(system_freezable_wq, &adc->poll_work, 0);

	return IRQ_HANDLED;
}

static int hamcop_adc_buffer_postenable(struct iio_dev *indio_dev)
{
	struct hamcop_adc *adc = iio_priv(indio_dev);

	adc->pen_irq_masked = false;
	hamcop_adc_idle(adc, HAMCOP_ADC_TSC_WAIT4INT);
	enable_irq(adc->irq_adcts);

	return 0;
}

static int hamcop_adc_buffer_predisable(struct iio_dev *indio_dev)
{
	struct hamcop_adc *adc = iio_priv(indio_dev);

	disable_irq(adc->irq_adcts);
	cancel_delayed_work_sync(&adc->poll_work);
	/* A chain torn down mid-flight left its own disable_irq_nosync()
	 * outstanding; drop it so the depth is exactly one.
	 */
	if (adc->pen_irq_masked) {
		adc->pen_irq_masked = false;
		enable_irq(adc->irq_adcts);
	}
	hamcop_adc_idle(adc, HAMCOP_ADC_TSC_SUSPEND);

	return 0;
}

static const struct iio_buffer_setup_ops hamcop_adc_buffer_setup_ops = {
	.postenable	= hamcop_adc_buffer_postenable,
	.predisable	= hamcop_adc_buffer_predisable,
};

static int hamcop_adc_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct hamcop_adc *adc = iio_priv(indio_dev);
	u16 data0, data1;
	int ret;

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	IIO_DEV_ACQUIRE_DIRECT_MODE(indio_dev, claim);
	if (IIO_DEV_ACQUIRE_FAILED(claim))
		return -EBUSY;

	switch (chan->address) {
	case HAMCOP_ADC_X:
	case HAMCOP_ADC_Y:
	case HAMCOP_ADC_Z1:
	case HAMCOP_ADC_Z2:
		ret = hamcop_adc_convert(adc, HAMCOP_ADC_TSC_SAMPLE_TOUCH,
					 HAMCOP_ADC_CONTROL_SEL_AIN3, &data0, &data1);
		/*
		 * Restore WAIT4INT on every exit, including a timed-out
		 * conversion -- hamcop_adc_convert() leaves the TSC
		 * register exactly as it programmed it (SAMPLE_TOUCH) on a
		 * timeout, so skipping this here left the block sitting in
		 * a non-idle mode until the next successful read happened
		 * to reprogram it.
		 */
		hamcop_adc_idle(adc, HAMCOP_ADC_TSC_WAIT4INT);
		if (ret)
			break;

		switch (chan->address) {
		case HAMCOP_ADC_X:
			*val = data0 & HAMCOP_ADC_DATA_MASK;
			break;
		case HAMCOP_ADC_Y:
			*val = data1 & HAMCOP_ADC_DATA_MASK;
			break;
		default: /* Z1/Z2 -- see this file's header comment */
			*val = (data0 & HAMCOP_ADC_DATA_PEN_UP) ||
			       (data1 & HAMCOP_ADC_DATA_PEN_UP) ? 0 : 1;
			break;
		}
		ret = IIO_VAL_INT;
		break;
	case HAMCOP_ADC_AIN0 ... HAMCOP_ADC_AIN2: {
		unsigned int ain = chan->address - HAMCOP_ADC_AIN0;
		u16 sel = ain << HAMCOP_ADC_CONTROL_SEL_SHIFT;

		ret = hamcop_adc_convert(adc, HAMCOP_ADC_TSC_SAMPLE_AIN, sel,
					 &data0, &data1);
		/* Same timeout-path restore as the touch case above. */
		hamcop_adc_idle(adc, HAMCOP_ADC_TSC_WAIT4INT);
		if (ret)
			break;

		*val = data0 & HAMCOP_ADC_DATA_MASK;
		ret = IIO_VAL_INT;
		break;
	}
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*
 * .indexed is required: the sysfs name generator only folds .channel into
 * the attribute name when it is set, and several channels share a type.
 */
#define HAMCOP_ADC_CHAN(_addr, _type, _idx, _name) {			\
	.type = (_type),						\
	.channel = (_idx),						\
	.indexed = 1,							\
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

static const struct iio_chan_spec hamcop_adc_channels[] = {
	HAMCOP_ADC_CHAN(HAMCOP_ADC_X,    IIO_POSITIONRELATIVE, 0, "x"),
	HAMCOP_ADC_CHAN(HAMCOP_ADC_Y,    IIO_POSITIONRELATIVE, 1, "y"),
	HAMCOP_ADC_CHAN(HAMCOP_ADC_Z1,   IIO_PRESSURE,         0, "z1"),
	HAMCOP_ADC_CHAN(HAMCOP_ADC_Z2,   IIO_PRESSURE,         1, "z2"),
	HAMCOP_ADC_CHAN(HAMCOP_ADC_AIN0, IIO_VOLTAGE,          0, "ain0"),
	HAMCOP_ADC_CHAN(HAMCOP_ADC_AIN1, IIO_VOLTAGE,          1, "ain1"),
	HAMCOP_ADC_CHAN(HAMCOP_ADC_AIN2, IIO_VOLTAGE,          2, "ain2"),
};

static const struct iio_info hamcop_adc_info = {
	.read_raw = hamcop_adc_read_raw,
};

static int hamcop_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct hamcop_adc *adc;
	struct resource *mem;
	unsigned long rate, prescaler;
	int irq_adc, ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->dev = dev;
	adc->indio_dev = indio_dev;
	init_completion(&adc->done);
	INIT_DELAYED_WORK(&adc->poll_work, hamcop_adc_poll_work);

	/*
	 * platform_get_resource() + devm_ioremap(), not
	 * devm_platform_ioremap_resource() -- the latter reserves the
	 * region exclusively via devm_request_mem_region(), which always
	 * loses to drivers/mfd/hamcop.c's own devm_ioremap_resource() of
	 * the entire HAMCOP MMIO window with -EBUSY.
	 * HAMCOP_ADC_BASE (0x3800) is a sub-range of that same
	 * already-claimed window.
	 */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -ENODEV;
	adc->regs = devm_ioremap(dev, mem->start, resource_size(mem));
	if (!adc->regs)
		return -ENOMEM;

	irq_adc = platform_get_irq(pdev, 0);
	if (irq_adc < 0)
		return irq_adc;

	adc->irq_adcts = platform_get_irq(pdev, 1);
	if (adc->irq_adcts < 0)
		return adc->irq_adcts;

	adc->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(adc->clk))
		return dev_err_probe(dev, PTR_ERR(adc->clk), "failed to get clock\n");

	rate = clk_get_rate(adc->clk);
	if (!rate)
		return dev_err_probe(dev, -EINVAL, "clock rate is 0\n");

	prescaler = rate / HAMCOP_ADC_TARGET_FREQ;
	prescaler = clamp_t(unsigned long, prescaler, 1, HAMCOP_ADC_CONTROL_PRESCALER_MAX + 1) - 1;
	adc->control_base = HAMCOP_ADC_CONTROL_PRESCALER_ENABLE |
			     (prescaler << HAMCOP_ADC_CONTROL_PRESCALER_SHIFT);

	/* Safe, low-current idle state until a consumer's buffer starts
	 * polling (hamcop_adc_buffer_postenable() switches to WAIT4INT).
	 */
	hamcop_adc_write(adc, HAMCOP_ADC_CONTROL,
			 adc->control_base | HAMCOP_ADC_CONTROL_STANDBY);
	hamcop_adc_idle(adc, HAMCOP_ADC_TSC_SUSPEND);

	ret = devm_request_irq(dev, irq_adc, hamcop_adc_conv_irq, 0,
			       "hamcop-adc", adc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request ADC IRQ\n");

	/*
	 * NO_AUTOEN: the pen-detect line only matters while a consumer has
	 * the touchscreen buffer open (see hamcop_adc_buffer_postenable()/
	 * _predisable()) -- masked at request time keeps HAMCOP from waking
	 * this driver over an unopened touchscreen, PDA battery life.
	 */
	ret = devm_request_irq(dev, adc->irq_adcts, hamcop_adc_pen_irq,
			       IRQF_NO_AUTOEN, "hamcop-adc-pen", adc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request pen IRQ\n");

	indio_dev->name = "hamcop-adc";
	indio_dev->info = &hamcop_adc_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	indio_dev->setup_ops = &hamcop_adc_buffer_setup_ops;
	indio_dev->channels = hamcop_adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(hamcop_adc_channels);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id hamcop_adc_of_match[] = {
	{ .compatible = "samsung,hamcop-adc" },
	{ }
};
MODULE_DEVICE_TABLE(of, hamcop_adc_of_match);

static struct platform_driver hamcop_adc_driver = {
	.driver = {
		.name = "hamcop-adc",
		.of_match_table = hamcop_adc_of_match,
	},
	.probe	= hamcop_adc_probe,
};
module_platform_driver(hamcop_adc_driver);

MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("HAMCOP ADC and resistive touchscreen front-end (HP iPAQ h2200)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:hamcop-adc");
