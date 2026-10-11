// SPDX-License-Identifier: GPL-2.0-only
/*
 * IIO ADC driver for the ASIC2 companion ASIC's ADC (HP iPAQ h38xx/h39xx
 * family): light sensor, Imin and battery voltage (Vs/Mbat) as plain
 * single-ended channels, plus the two touchpanel channels (TP_X0/TP_Y1)
 * and the resistive-touchscreen plate-switching state machine that drives
 * them.
 *
 * The register layout (Multiplexer/ControlStatus/Data at 0x00/0x04/0x08)
 * matches include/linux/mfd/asic2.h. The driver maps its own copy of just
 * that window.
 *
 * mainline's generic drivers/input/touchscreen/resistive-adc-touch.c is
 * the touchscreen consumer. It is driven entirely through IIO's
 * push-callback-buffer API and never touches hardware itself, so this
 * driver is a triggerless IIO buffer producer for "x" and "y" rather
 * than a set of raw-readable channels.
 *
 * The ASIC2 has no touchscreen block that switches the plates itself, so
 * the four GPIO2 plate-drive bits are set by hand around every ADC
 * conversion. asic2_ts_sweep_work() runs the 12-conversion sequence,
 * each conversion preceded by its own plate switch, in a workqueue: the
 * conversions sleep waiting for the interrupt and cannot run from hard-IRQ
 * context.
 *
 * The plate GPIOs and the pen-contact GPIO are not requested here:
 * drivers/mfd/asic2.c requests them against its own gpio_chip and passes
 * them as the cell's platform data, so that all accesses to the GPIO2 data
 * register are serialized by the one lock that guards it.
 *
 * The conversion clock prescaler is 4, the value the manufacturer programs
 * for this ADC. The target conversion frequency and the per-step frequency
 * of the 4-bit prescaler field are not documented, and the duration of the
 * individual conversions in a touch sweep has not been measured.
 *
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/asic2.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/ratelimit.h>
#include <linux/workqueue.h>

#include <linux/iio/buffer.h>
#include <linux/iio/driver.h>
#include <linux/iio/iio.h>
#include <linux/iio/machine.h>

#define ASIC2_ADC_DATA_MASK		GENMASK(9, 0)	/* 10-bit conversion */

#define ASIC2_ADC_CONV_TIMEOUT_MS	50

/* At most one conversion-timeout warning per this interval. */
#define ASIC2_ADC_WARN_INTERVAL		(5 * HZ)

/*
 * A touch sweep that lost a conversion is dropped rather than reported, and
 * the next sweep of a pen that still reads down is delayed by an interval
 * that starts at MIN, doubles per consecutive drop up to MAX, and starts
 * over after a sweep that completed. With no delay a persistent
 * completion-interrupt loss would keep the sweep running back to back,
 * sleeping only for the conversion timeouts.
 */
#define ASIC2_TS_BACKOFF_MIN_MS		50
#define ASIC2_TS_BACKOFF_MAX_MS		500

/*
 * Time given to the plates to settle back into the pen-sensing
 * configuration, and to GPIO2_PEN_IRQ to read a stable value, between the
 * end of a sweep and trusting the live pen-state read.
 */
#define ASIC2_TS_SETTLE_MS		10

/*
 * Time between the pen interrupt and the first sweep: the contact still
 * bounces at that moment and the plates read as the rails.
 */
#define ASIC2_TS_PENDOWN_SETTLE_MS	10

/* An axis that reads 0 or full scale has no contact behind it. */
#define ASIC2_TS_RAIL_MAX		ASIC2_ADC_DATA_MASK

enum asic2_adc_channel {
	ASIC2_ADC_LIGHTSENSOR,
	ASIC2_ADC_IMIN,
	ASIC2_ADC_VS_MBAT,
	ASIC2_ADC_TP_X0,
	ASIC2_ADC_TP_Y1,
};

/*
 * One step of the touch sweep. "mask" is which of GPIO2's 4 plate-drive
 * bits (ASIC2_GPIO2_IN_{Y1_N,X0,Y0,X1_N}) are driven high for this step;
 * "mux" is the ADC mux value to convert immediately after the plate write.
 * The final switch back to the pen-sensing configuration does not convert
 * anything and is ASIC2_TS_PLATE_PEN_SENSE below rather than a table row.
 */
struct asic2_ts_step {
	u8 mask;
	u8 mux;
};

#define ASIC2_TS_PLATE_X_EXCITE		(ASIC2_GPIO2_IN_X1_N | ASIC2_GPIO2_IN_Y0)
#define ASIC2_TS_PLATE_Y_EXCITE		(ASIC2_GPIO2_IN_Y1_N | ASIC2_GPIO2_IN_X0)
#define ASIC2_TS_PLATE_CAL		(ASIC2_GPIO2_IN_Y1_N | ASIC2_GPIO2_IN_Y0)

/* Pen-sensing config: end of a sweep, and when the buffer is enabled */
#define ASIC2_TS_PLATE_PEN_SENSE \
	(ASIC2_GPIO2_IN_Y1_N | ASIC2_GPIO2_IN_X0 | ASIC2_GPIO2_IN_X1_N)
/* Off state when the buffer is disabled: PEN_SENSE without X0 */
#define ASIC2_TS_PLATE_OFF \
	(ASIC2_GPIO2_IN_Y1_N | ASIC2_GPIO2_IN_X1_N)

static const struct asic2_ts_step asic2_ts_sequence[] = {
	/* samples[0..3]: X excitation, repeated 4x. No pin state changes
	 * across these four steps, only elapsed conversion time does, which
	 * lets the plate settle. samples[3], the most settled, is reported.
	 */
	{ ASIC2_TS_PLATE_X_EXCITE, ASIC2_ADC_TP_X0 },
	{ ASIC2_TS_PLATE_X_EXCITE, ASIC2_ADC_TP_X0 },
	{ ASIC2_TS_PLATE_X_EXCITE, ASIC2_ADC_TP_X0 },
	{ ASIC2_TS_PLATE_X_EXCITE, ASIC2_ADC_TP_X0 },	/* [3] -- reported X */
	/* samples[4]: TP_Y1 read while still X-excited. Not reported;
	 * kept because it is part of the sequence the controller was
	 * designed around, though its purpose is unverified.
	 */
	{ ASIC2_TS_PLATE_X_EXCITE, ASIC2_ADC_TP_Y1 },
	/* samples[5..8]: Y excitation, repeated 4x. samples[8] is reported. */
	{ ASIC2_TS_PLATE_Y_EXCITE, ASIC2_ADC_TP_Y1 },
	{ ASIC2_TS_PLATE_Y_EXCITE, ASIC2_ADC_TP_Y1 },
	{ ASIC2_TS_PLATE_Y_EXCITE, ASIC2_ADC_TP_Y1 },
	{ ASIC2_TS_PLATE_Y_EXCITE, ASIC2_ADC_TP_Y1 },	/* [8] -- reported Y */
	/* samples[9]: the other cross-read, not reported. */
	{ ASIC2_TS_PLATE_Y_EXCITE, ASIC2_ADC_TP_X0 },
	/* samples[10]/[11]: a third plate mask, neither pure X nor Y
	 * excitation; calibration reads that are not reported, purpose
	 * unverified.
	 */
	{ ASIC2_TS_PLATE_CAL, ASIC2_ADC_TP_X0 },
	{ ASIC2_TS_PLATE_CAL, ASIC2_ADC_TP_Y1 },
};

#define ASIC2_TS_NR_STEPS		ARRAY_SIZE(asic2_ts_sequence)
#define ASIC2_TS_SAMPLE_X		3
#define ASIC2_TS_SAMPLE_Y		8

struct asic2_adc {
	struct device *dev;
	struct iio_dev *indio_dev;
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

	/* Touch plate/pen GPIOs, owned and locked by drivers/mfd/asic2.c */
	const struct asic2_adc_touch_pdata *touch_gpios;
	int irq_pen;
	struct delayed_work ts_work;

	/*
	 * Set by the pen interrupt handler right after disable_irq_nosync(),
	 * cleared by the pen-up branch of asic2_ts_sweep_work() right before
	 * enable_irq(). The buffer predisable callback consults it to
	 * rebalance the disable when it cancels ts_work mid-sweep. A plain
	 * bool is enough: predisable only reads it after
	 * cancel_delayed_work_sync() has guaranteed that ts_work is not
	 * running, and the interrupt handler runs on the single CPU of the
	 * PXA25x/PXA27x targets.
	 */
	bool ts_irq_disabled;

	/*
	 * Delay the next retry of a dropped sweep will use. Only touched by
	 * asic2_ts_sweep_work() and by postenable() while no work can be
	 * running.
	 */
	unsigned int ts_backoff_ms;
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

/*
 * Set the plate-drive bits. The four gpiod_set_value() calls are four
 * separate read-modify-write cycles of the GPIO2 data register, each
 * serialized against every other GPIO2 user by the gpio_chip's lock. A set
 * bit in mask drives the raw GPIO2 bit high and a clear bit drives it low:
 * there is no active-low inversion, the mask is the register
 * pattern the controller expects.
 */
static void asic2_adc_ts_set_plate(struct asic2_adc *adc, u8 mask)
{
	const struct asic2_adc_touch_pdata *tp = adc->touch_gpios;

	gpiod_set_value(tp->gpio_y1_n, !!(mask & ASIC2_GPIO2_IN_Y1_N));
	gpiod_set_value(tp->gpio_x0,   !!(mask & ASIC2_GPIO2_IN_X0));
	gpiod_set_value(tp->gpio_y0,   !!(mask & ASIC2_GPIO2_IN_Y0));
	gpiod_set_value(tp->gpio_x1_n, !!(mask & ASIC2_GPIO2_IN_X1_N));
}

/* Report (0, 0), which resistive-adc-touch treats as end of touch. */
static void asic2_ts_pen_up(struct asic2_adc *adc)
{
	u16 buf[2] = { };

	iio_push_to_buffers(adc->indio_dev, buf);

	adc->ts_irq_disabled = false;
	enable_irq(adc->irq_pen);
}

/*
 * If the pen reads down, run one full 12-conversion touch sweep
 * synchronously, then settle, poll the pen state and report. If the pen is
 * still down, schedule another sweep; if it was released, re-enable the pen
 * interrupt.
 *
 * A sweep in which a conversion timed out is dropped without a report: the
 * lost step leaves its samples[] slot unwritten, and a (0, 0) or half-real
 * pair pushed to the buffer would reach resistive-adc-touch as an end of
 * touch or as a jump to an axis edge. samples[] is zeroed so that no path
 * can push stack contents. The pen-up path is unaffected, so a released pen
 * still gets its release report and re-arms the pen IRQ.
 */
static void asic2_ts_sweep_work(struct work_struct *work)
{
	struct asic2_adc *adc = container_of(work, struct asic2_adc, ts_work.work);
	static DEFINE_RATELIMIT_STATE(step_rs, ASIC2_ADC_WARN_INTERVAL, 1);
	u16 samples[ASIC2_TS_NR_STEPS] = { };
	u16 buf[2]; /* x, y -- ascending scan_index order (TP_X0=3, TP_Y1=4) */
	unsigned int delay_ms = 0;
	bool dropped = false;
	unsigned int i;

	/*
	 * The plate switching of a sweep makes edges on GPIO2_PEN_IRQ while
	 * the pen interrupt is disabled. The disable is lazy, so the IRQ core
	 * keeps such an edge pending and replays it on the enable_irq() of
	 * the pen-up path. Sweeping on that replay would end in another
	 * enable_irq() and another replay, with no pen on the panel.
	 */
	if (gpiod_get_value(adc->touch_gpios->gpio_pen)) {
		asic2_ts_pen_up(adc);
		return;
	}

	for (i = 0; i < ASIC2_TS_NR_STEPS; i++) {
		asic2_adc_ts_set_plate(adc, asic2_ts_sequence[i].mask);
		if (asic2_adc_convert(adc, asic2_ts_sequence[i].mux, &samples[i])) {
			if (__ratelimit(&step_rs))
				dev_warn(adc->dev,
					 "touch sweep step %u timed out\n", i);
			/*
			 * A lost conversion means the completion path is
			 * broken, so the remaining steps would only cost
			 * another ASIC2_ADC_CONV_TIMEOUT_MS each.
			 */
			dropped = true;
			break;
		}
	}

	if (dropped) {
		delay_ms = adc->ts_backoff_ms;
		adc->ts_backoff_ms = min(2 * delay_ms, ASIC2_TS_BACKOFF_MAX_MS);
	} else {
		adc->ts_backoff_ms = ASIC2_TS_BACKOFF_MIN_MS;
	}

	/* Plates back to the pen-sensing config, then let them settle before
	 * the pen-state read below is trusted.
	 */
	asic2_adc_ts_set_plate(adc, ASIC2_TS_PLATE_PEN_SENSE);
	msleep(ASIC2_TS_SETTLE_MS);

	/* Poll the pen level rather than waiting for the edge interrupt: the
	 * raw bit is high when the pen is up.
	 */
	if (gpiod_get_value(adc->touch_gpios->gpio_pen)) {
		asic2_ts_pen_up(adc);
		return;
	}

	/* The pen IRQ stays disabled across the retry: ts_irq_disabled is
	 * still set, so predisable() rebalances it if this work is cancelled
	 * while it waits.
	 */
	if (dropped) {
		schedule_delayed_work(&adc->ts_work, msecs_to_jiffies(delay_ms));
		return;
	}

	/* A rail on either axis is a contact that has not settled, not a
	 * position: no report and no backoff, the next sweep follows at once.
	 */
	if (!samples[ASIC2_TS_SAMPLE_X] ||
	    samples[ASIC2_TS_SAMPLE_X] >= ASIC2_TS_RAIL_MAX ||
	    !samples[ASIC2_TS_SAMPLE_Y] ||
	    samples[ASIC2_TS_SAMPLE_Y] >= ASIC2_TS_RAIL_MAX) {
		schedule_delayed_work(&adc->ts_work, 0);
		return;
	}

	/* Pen still down: report the most settled sample of each axis and
	 * start another sweep.
	 */
	buf[0] = samples[ASIC2_TS_SAMPLE_X];
	buf[1] = samples[ASIC2_TS_SAMPLE_Y];
	iio_push_to_buffers(adc->indio_dev, buf);

	schedule_delayed_work(&adc->ts_work, 0);
}

/*
 * Falling edge on GPIO2_PEN_IRQ: the pen may have touched. Keep the interrupt
 * disabled for the whole active-sampling state (it is re-enabled by the
 * pen-up branch of asic2_ts_sweep_work()) and start the first sweep.
 */
static irqreturn_t asic2_adc_ts_pen_irq(int irq, void *data)
{
	struct asic2_adc *adc = data;

	disable_irq_nosync(adc->irq_pen);
	adc->ts_irq_disabled = true;
	schedule_delayed_work(&adc->ts_work,
			      msecs_to_jiffies(ASIC2_TS_PENDOWN_SETTLE_MS));

	return IRQ_HANDLED;
}

static int asic2_adc_ts_postenable(struct iio_dev *indio_dev)
{
	struct asic2_adc *adc = iio_priv(indio_dev);

	/* A retry delay grown in an earlier pen-down sequence must not carry over. */
	adc->ts_backoff_ms = ASIC2_TS_BACKOFF_MIN_MS;

	/* Plates to the pen-sensing config, then arm the falling-edge IRQ */
	asic2_adc_ts_set_plate(adc, ASIC2_TS_PLATE_PEN_SENSE);
	enable_irq(adc->irq_pen);

	return 0;
}

static int asic2_adc_ts_predisable(struct iio_dev *indio_dev)
{
	struct asic2_adc *adc = iio_priv(indio_dev);

	/*
	 * Before the cancel, not after: disable_irq() waits for a pen ISR
	 * that is already running, and no further one can queue ts_work or
	 * set ts_irq_disabled behind the cancel below.
	 */
	disable_irq(adc->irq_pen);

	cancel_delayed_work_sync(&adc->ts_work);

	/*
	 * If a sweep, or the delay before a dropped sweep's retry, was in
	 * flight, the disable_irq_nosync() of the pen handler was not
	 * rebalanced by the pen-up path of ts_work, which the cancel above
	 * prevented from running. Rebalance it, or the disable depth would
	 * leak and no later postenable() could undo it. The disable_irq()
	 * above stays.
	 */
	if (adc->ts_irq_disabled) {
		enable_irq(adc->irq_pen);
		adc->ts_irq_disabled = false;
	}

	/* Plates to the off state */
	asic2_adc_ts_set_plate(adc, ASIC2_TS_PLATE_OFF);

	return 0;
}

static const struct iio_buffer_setup_ops asic2_adc_ts_buffer_setup_ops = {
	.postenable	= asic2_adc_ts_postenable,
	.predisable	= asic2_adc_ts_predisable,
};

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

	/*
	 * TP_X0/TP_Y1 have no _raw attribute (see ASIC2_ADC_CHAN_BUFFER_ONLY),
	 * so only the plain single-ended channels reach here.
	 */
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

/*
 * Same as ASIC2_ADC_CHAN, but without IIO_CHAN_INFO_RAW: TP_X0/TP_Y1 are
 * buffer-only. A lone X or Y sample is not meaningful on its own, and a raw
 * read would have to drive the plate-excitation GPIOs, so any generic
 * consumer polling in_voltageN_raw on a board where the touch buffer is
 * never enabled would keep re-exciting and disturbing the plates.
 */
#define ASIC2_ADC_CHAN_BUFFER_ONLY(_idx, _name) {			\
	.type = IIO_VOLTAGE,						\
	.channel = (_idx),						\
	.indexed = 1,							\
	.scan_index = (_idx),						\
	.scan_type = {							\
		.sign = 'u',						\
		.realbits = 10,						\
		.storagebits = 16,					\
	},								\
	.datasheet_name = (_name),					\
}

static const struct iio_chan_spec asic2_adc_channels[] = {
	ASIC2_ADC_CHAN(ASIC2_ADC_LIGHTSENSOR, "lightsensor"),
	ASIC2_ADC_CHAN(ASIC2_ADC_IMIN,        "imin"),
	ASIC2_ADC_CHAN(ASIC2_ADC_VS_MBAT,     "vs_mbat"),
	ASIC2_ADC_CHAN_BUFFER_ONLY(ASIC2_ADC_TP_X0, "tp_x0"),
	ASIC2_ADC_CHAN_BUFFER_ONLY(ASIC2_ADC_TP_Y1, "tp_y1"),
};

/*
 * Legacy (non-devicetree) iio_map for "vs_mbat" -> "iio_hwmon": the generic
 * IIO-to-hwmon bridge has no firmware node here, so it looks its channels
 * up by its device name, which is the driver name since the cell has no
 * id. No scale is provided, so the hwmon in0_input file reports raw ADC
 * counts (0-1023).
 *
 * The touchscreen takes its channels from the "io-channels" property of
 * its devicetree node instead. Their order there, x then y, must match the
 * packed sample layout asic2_ts_sweep_work() pushes (buf[0]=x, buf[1]=y),
 * which follows the ascending scan_index of TP_X0 and TP_Y1.
 */
static const struct iio_map asic2_adc_iio_maps[] = {
	IIO_MAP("vs_mbat", "iio_hwmon", NULL),
	{ }
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
	adc->indio_dev = indio_dev;
	init_completion(&adc->done);
	ret = devm_mutex_init(dev, &adc->convert_lock);
	if (ret)
		return ret;
	INIT_DELAYED_WORK(&adc->ts_work, asic2_ts_sweep_work);

	adc->touch_gpios = dev_get_platdata(dev);
	if (!adc->touch_gpios)
		return dev_err_probe(dev, -EINVAL, "no touch GPIO platform_data\n");

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

	adc->irq_pen = platform_get_irq(pdev, 1);
	if (adc->irq_pen < 0)
		return adc->irq_pen;

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

	/*
	 * The pen line only matters while a consumer has the touch buffer
	 * open (asic2_adc_ts_postenable()/_predisable()), so request it
	 * masked. IRQF_TRIGGER_FALLING makes asic2_irq_set_type() in
	 * drivers/mfd/asic2.c program GPIO2's edge/type registers and switch
	 * this hwirq to handle_edge_irq.
	 */
	ret = devm_request_irq(dev, adc->irq_pen, asic2_adc_ts_pen_irq,
			       IRQF_TRIGGER_FALLING | IRQF_NO_AUTOEN,
			       "asic2-adc-pen", adc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request pen IRQ\n");

	indio_dev->name = "asic2-adc";
	indio_dev->info = &asic2_adc_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	indio_dev->setup_ops = &asic2_adc_ts_buffer_setup_ops;
	indio_dev->channels = asic2_adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(asic2_adc_channels);

	ret = devm_iio_map_array_register(dev, indio_dev, asic2_adc_iio_maps);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register iio_map\n");

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
MODULE_DESCRIPTION("ASIC2 companion ASIC ADC and resistive touchscreen (HP iPAQ h38xx/h39xx)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:asic2-adc");
