// SPDX-License-Identifier: GPL-2.0-only
/*
 * Synaptics NavPoint touchpad, an SPI target device.
 *
 * Copyright (C) 2012 Paul Parsons <lost.distance@yahoo.com>
 *
 * The module generates the clock and frame signals of its SPI link itself and
 * pushes packets of the Synaptics Modular Embedded Protocol to the host.
 */

#include <linux/completion.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/string.h>

/*
 * Module packet header: bits 2:0 are the number of bytes that follow, bits
 * 4:3 the control field and bits 7:5 the module address.
 */
#define NAVPOINT_HEADER_LENGTH(byte)	((byte) & 0x07)

#define NAVPOINT_PKT_ABSOLUTE		0x0e
#define NAVPOINT_PKT_HELLO		0x19

/*
 * The module does not implement the coordinate range query of the protocol,
 * so these are the limits seen in practice.
 */
#define NAVPOINT_X_MIN			1278
#define NAVPOINT_X_MAX			5340
#define NAVPOINT_Y_MIN			1572
#define NAVPOINT_Y_MAX			4396
#define NAVPOINT_PRESSURE_MAX		255

/* Aborting can miss a message that is only just being queued, so retry */
/*
 * Receive errors, overruns and flushes that find the pad mid-burst, are
 * transient on a free-running link. Only a long unbroken run of them is taken
 * to be a fault.
 */
#define NAVPOINT_MAX_ERRORS		100

#define NAVPOINT_STOP_STEP_MS		10
#define NAVPOINT_STOP_TRIES		100

/*
 * Words per receive transfer. A burst that fills the buffer completes the
 * transfer at once and the remainder is lost to the flush that starts the
 * next one, so this stays well above the 16 entry Rx FIFO. Bursts that are
 * shorter end on the receive timeout.
 */
#define NAVPOINT_RX_WORDS		32

struct navpoint {
	struct spi_device	*spi;
	struct input_dev	*input;
	struct regulator	*vdd;
	bool			powered;

	struct spi_message	msg;
	struct spi_transfer	xfer;
	u16			*rx;
	struct completion	done;
	bool			stopping;
	bool			in_flight;
	bool			wedged;
	unsigned int		errors;

	/* Packet parser state, kept across transfers */
	u8			data[1 + NAVPOINT_HEADER_LENGTH(0xff)];
	unsigned int		index;
	unsigned int		length;
};

static void navpoint_packet(struct navpoint *navpoint)
{
	const u8 *data = navpoint->data;
	int finger, gesture, x, y, z;

	switch (data[0]) {
	case NAVPOINT_PKT_ABSOLUTE:
		finger = data[1] & 0x01;
		gesture = data[1] & 0x02;
		x = ((data[2] & 0x1f) << 8) | data[3];
		y = ((data[4] & 0x1f) << 8) | data[5];
		z = data[6];

		input_report_key(navpoint->input, BTN_TOUCH, finger);
		input_report_key(navpoint->input, BTN_TOOL_FINGER, finger);
		input_report_key(navpoint->input, BTN_LEFT, gesture);
		if (finger) {
			input_report_abs(navpoint->input, ABS_X, x);
			input_report_abs(navpoint->input, ABS_Y, y);
		}
		input_report_abs(navpoint->input, ABS_PRESSURE, z);
		input_sync(navpoint->input);
		break;

	case NAVPOINT_PKT_HELLO:
		if ((data[1] & 0xf0) == 0x10) {
			dev_dbg(&navpoint->spi->dev, "hello packet\n");
			break;
		}
		fallthrough;
	default:
		dev_dbg_ratelimited(&navpoint->spi->dev,
				    "spurious packet: %*ph\n",
				    (int)navpoint->length, data);
		break;
	}
}

static void navpoint_feed(struct navpoint *navpoint, u8 byte)
{
	if (!navpoint->index) {
		/* Idle bytes between packets, including the pad byte */
		if (byte == 0x00 || byte == 0xff)
			return;

		navpoint->length = NAVPOINT_HEADER_LENGTH(byte) + 1;
	}

	navpoint->data[navpoint->index++] = byte;

	if (navpoint->index == navpoint->length) {
		navpoint_packet(navpoint);
		navpoint->index = 0;
	}
}

static void navpoint_complete(void *context);

static int navpoint_submit(struct navpoint *navpoint)
{
	int error;

	/*
	 * Rebuild the message every time: the controller shortens the transfer
	 * to the length of the burst that arrived.
	 */
	spi_message_init(&navpoint->msg);
	memset(&navpoint->xfer, 0, sizeof(navpoint->xfer));
	navpoint->xfer.rx_buf = navpoint->rx;
	navpoint->xfer.len = NAVPOINT_RX_WORDS * sizeof(*navpoint->rx);
	navpoint->xfer.bits_per_word = 16;
	spi_message_add_tail(&navpoint->xfer, &navpoint->msg);
	navpoint->msg.complete = navpoint_complete;
	navpoint->msg.context = navpoint;

	WRITE_ONCE(navpoint->in_flight, true);
	error = spi_async(navpoint->spi, &navpoint->msg);
	if (error)
		WRITE_ONCE(navpoint->in_flight, false);

	return error;
}

static void navpoint_complete(void *context)
{
	struct navpoint *navpoint = context;
	struct device *dev = &navpoint->spi->dev;
	unsigned int i;
	int error;

	WRITE_ONCE(navpoint->in_flight, false);

	if (navpoint->msg.status) {
		if (READ_ONCE(navpoint->stopping) ||
		    navpoint->msg.status == -ESHUTDOWN)
			goto out;

		dev_err_ratelimited(dev, "receive failed: %d\n",
				    navpoint->msg.status);
		if (++navpoint->errors >= NAVPOINT_MAX_ERRORS) {
			dev_err(dev, "giving up after %u receive errors\n",
				navpoint->errors);
			goto out;
		}
	} else {
		navpoint->errors = 0;

		/* The first byte on the wire is the high half of each word */
		for (i = 0; i < navpoint->xfer.len / sizeof(u16); i++) {
			navpoint_feed(navpoint, navpoint->rx[i] >> 8);
			navpoint_feed(navpoint, navpoint->rx[i] & 0xff);
		}
	}

	if (READ_ONCE(navpoint->stopping))
		goto out;

	error = navpoint_submit(navpoint);
	if (!error)
		return;

	dev_err(dev, "cannot queue receive: %d\n", error);
out:
	complete_all(&navpoint->done);
}

/*
 * Stop receiving. The abort fails when it lands between two messages, in
 * which case the next message is already queued and needs another abort.
 *
 * A message the controller still owns must never be reused or freed, so
 * when it does not come back the device is given up on instead.
 */
static void navpoint_stop_rx(struct navpoint *navpoint)
{
	unsigned long step = msecs_to_jiffies(NAVPOINT_STOP_STEP_MS);
	unsigned int i;

	if (navpoint->wedged)
		return;

	WRITE_ONCE(navpoint->stopping, true);

	for (i = 0; i < NAVPOINT_STOP_TRIES; i++) {
		spi_target_abort(navpoint->spi);
		if (wait_for_completion_timeout(&navpoint->done, step))
			return;
	}

	WRITE_ONCE(navpoint->wedged, true);
	dev_err(&navpoint->spi->dev,
		"receive did not stop, the device is unusable until reboot\n");
}

static void navpoint_stop(struct navpoint *navpoint)
{
	if (navpoint->powered) {
		regulator_disable(navpoint->vdd);
		navpoint->powered = false;
	}

	navpoint_stop_rx(navpoint);
}

/* The SPI side is armed before the module is powered, so no packet is missed */
static int navpoint_start(struct navpoint *navpoint)
{
	int error;

	if (navpoint->wedged || READ_ONCE(navpoint->in_flight))
		return -EBUSY;

	WRITE_ONCE(navpoint->stopping, false);
	navpoint->errors = 0;
	reinit_completion(&navpoint->done);
	navpoint->index = 0;

	error = navpoint_submit(navpoint);
	if (error)
		return error;

	error = regulator_enable(navpoint->vdd);
	if (error) {
		dev_err(&navpoint->spi->dev, "cannot enable vdd: %d\n", error);
		navpoint_stop_rx(navpoint);
		return error;
	}
	navpoint->powered = true;

	return 0;
}

static int navpoint_open(struct input_dev *input)
{
	return navpoint_start(input_get_drvdata(input));
}

static void navpoint_close(struct input_dev *input)
{
	navpoint_stop(input_get_drvdata(input));
}

/*
 * Registered first, so it runs after the input device is gone and has stopped
 * the receiver. The message and its buffer are leaked when the controller
 * never gave them back, because it would write to them later.
 */
static void navpoint_free(void *data)
{
	struct navpoint *navpoint = data;

	if (navpoint->wedged) {
		dev_err(&navpoint->spi->dev,
			"leaking the receive message still owned by the SPI controller\n");
		return;
	}

	kfree(navpoint->rx);
	kfree(navpoint);
}

static int navpoint_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct navpoint *navpoint;
	struct input_dev *input;
	int error;

	if (!spi_controller_is_target(spi->controller))
		return dev_err_probe(dev, -EINVAL,
				     "the parent is not an SPI target controller\n");

	navpoint = kzalloc_obj(*navpoint);
	if (!navpoint)
		return -ENOMEM;

	/* Separate allocation: the receive buffer must not share cache lines */
	navpoint->rx = kcalloc(NAVPOINT_RX_WORDS, sizeof(*navpoint->rx),
			       GFP_KERNEL);
	if (!navpoint->rx) {
		kfree(navpoint);
		return -ENOMEM;
	}

	navpoint->spi = spi;
	error = devm_add_action_or_reset(dev, navpoint_free, navpoint);
	if (error)
		return error;

	init_completion(&navpoint->done);

	navpoint->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(navpoint->vdd))
		return dev_err_probe(dev, PTR_ERR(navpoint->vdd),
				     "cannot get vdd supply\n");

	spi->mode = SPI_MODE_1 | SPI_NO_TX;
	spi->bits_per_word = 16;
	error = spi_setup(spi);
	if (error)
		return dev_err_probe(dev, error, "cannot set up SPI\n");

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	navpoint->input = input;

	input->name = "Synaptics NavPoint";
	input->phys = "navpoint/input0";
	input->id.bustype = BUS_SPI;
	input->open = navpoint_open;
	input->close = navpoint_close;

	__set_bit(EV_KEY, input->evbit);
	__set_bit(BTN_LEFT, input->keybit);
	__set_bit(BTN_TOUCH, input->keybit);
	__set_bit(BTN_TOOL_FINGER, input->keybit);

	__set_bit(EV_ABS, input->evbit);
	input_set_abs_params(input, ABS_X, NAVPOINT_X_MIN, NAVPOINT_X_MAX, 0, 0);
	input_set_abs_params(input, ABS_Y, NAVPOINT_Y_MIN, NAVPOINT_Y_MAX, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, NAVPOINT_PRESSURE_MAX, 0, 0);

	__set_bit(INPUT_PROP_POINTER, input->propbit);

	input_set_drvdata(input, navpoint);
	spi_set_drvdata(spi, navpoint);

	error = input_register_device(input);
	if (error)
		return dev_err_probe(dev, error, "cannot register input device\n");

	return 0;
}

static int navpoint_suspend(struct device *dev)
{
	struct navpoint *navpoint = dev_get_drvdata(dev);
	struct input_dev *input = navpoint->input;

	mutex_lock(&input->mutex);
	if (input_device_enabled(input))
		navpoint_stop(navpoint);
	mutex_unlock(&input->mutex);

	return 0;
}

static int navpoint_resume(struct device *dev)
{
	struct navpoint *navpoint = dev_get_drvdata(dev);
	struct input_dev *input = navpoint->input;
	int error = 0;

	mutex_lock(&input->mutex);
	if (input_device_enabled(input))
		error = navpoint_start(navpoint);
	mutex_unlock(&input->mutex);

	return error;
}

static DEFINE_SIMPLE_DEV_PM_OPS(navpoint_pm_ops, navpoint_suspend, navpoint_resume);

static const struct of_device_id navpoint_of_match[] = {
	{ .compatible = "synaptics,navpoint" },
	{ }
};
MODULE_DEVICE_TABLE(of, navpoint_of_match);

static const struct spi_device_id navpoint_id[] = {
	{ "navpoint" },
	{ }
};
MODULE_DEVICE_TABLE(spi, navpoint_id);

static struct spi_driver navpoint_driver = {
	.driver = {
		.name = "navpoint",
		.of_match_table = navpoint_of_match,
		.pm = pm_sleep_ptr(&navpoint_pm_ops),
	},
	.id_table = navpoint_id,
	.probe = navpoint_probe,
};
module_spi_driver(navpoint_driver);

MODULE_AUTHOR("Paul Parsons <lost.distance@yahoo.com>");
MODULE_DESCRIPTION("Synaptics NavPoint touchpad driver");
MODULE_LICENSE("GPL");
