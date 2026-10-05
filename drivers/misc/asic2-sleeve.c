// SPDX-License-Identifier: GPL-2.0-only
/*
 * ASIC2 backend for the generic iPAQ expansion-pack ("sleeve" or "option
 * jacket") bus, drivers/misc/ipaq-sleeve.c, on the HP/Compaq iPAQ
 * h38xx/h39xx family.
 *
 * Both SPI protocols of the connector are implemented, through the
 * spi_controller API on top of drivers/spi/spi-asic2.c:
 *
 *  - the passive 25LC040 ID EEPROM that every sleeve answers, on chip
 *    select 0, and
 *  - the checksummed command and reply protocol of the microcontroller
 *    inside a powered PCMCIA or extended-battery jacket, on chip select 1.
 *
 * The SPI driver holds the clocks of the SPI block and of the shared
 * crystal for its lifetime, so no clock handling is needed here. It also
 * selects the clock rate by chip select, and the hardware selector has no
 * established conversion to Hz, which makes ASIC2_SLEEVE_SPI_HZ an API
 * ceiling and not the source of the divider.
 *
 * The power and reset lines and the jacket presence line are GPIO
 * descriptors resolved from the "sleeve" child node of the ASIC2, see
 * Documentation/devicetree/bindings/mfd/htc,asic2.yaml. Jacket presence
 * is a bare SoC GPIO and not an ASIC2 line.
 *
 * This driver is the only owner of the power, nvram and presence lines,
 * and so of the jacket's power. The CF or PC Card socket of a jacket does
 * not request them. It is created on demand as a slot consumer of the bus
 * core, through its IPAQ_SLEEVE_EVENT_ADD and _REMOVE notifications and
 * ipaq_sleeve_features(), once the ID EEPROM has identified a jacket that
 * has a slot. That is a notification and not a second ipaq_sleeve_driver
 * because the driver core binds one driver per device, and a Navman GPS
 * and CF sleeve is already bound to its GPS driver.
 *
 * The four PCMCIA status lines of enum ipaq_sleeve_egpio (card detect and
 * interrupt of each socket) are not implemented here. On the h39xx they
 * are bare SoC GPIOs, owned by the PCMCIA socket driver and read through
 * the generic GPIO path of soc_common, so there is nothing for this
 * backend to arbitrate. On the h5xxx they are bits of a SAMCOP register
 * with no GPIO behind them, which is why the SAMCOP backend implements
 * them.
 *
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/ipaq-sleeve.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/spi/spi.h>
#include <linux/string.h>

/*
 * Compatible of the "spi" child node of the ASIC2. The SPI block is an MFD
 * cell without an identity of its own, so this node is what the controller
 * is looked up by.
 */
#define ASIC2_SLEEVE_SPI_COMPATIBLE	"htc,asic2-spi"

/* The SPI driver maps chip select 0 to CS0 and 1 to CONTROL.SEL_CS1. */
#define ASIC2_SLEEVE_CS_EEPROM		0
#define ASIC2_SLEEVE_CS_MCU		1

/* An API ceiling, not a divider input, see the header comment. */
#define ASIC2_SLEEVE_SPI_HZ		2000000

/*
 * Microchip 25LC040, the standard ID EEPROM part on every option pack. The
 * status register is polled until the write-in-progress bit clears before a
 * read is issued.
 */
#define ASIC2_SLEEVE_EEPROM_RDSR	0x05
#define ASIC2_SLEEVE_EEPROM_RDSR_WIP	BIT(0)
#define ASIC2_SLEEVE_EEPROM_READY_TRIES	20

/* Read command with address bit A8 folded into command bit 3. */
#define ASIC2_SLEEVE_EEPROM_READ(addr)	(0x03 | (((addr) & 0x100) >> 5))

/*
 * Checksummed command and reply protocol of the microcontroller inside a
 * powered PCMCIA or extended-battery jacket.
 *
 * Message shape, both directions: 0xA1 header, one command or reply byte,
 * then for a reply the number of payload bytes given by the low nibble of
 * the reply byte, then a checksum. The transmit checksum of a payload-free
 * command is the command byte itself, the receive checksum is the reply
 * byte plus every payload byte, with the header excluded.
 */
#define ASIC2_SLEEVE_MCU_HEADER		0xa1
#define ASIC2_SLEEVE_MCU_ID		0x10
#define ASIC2_SLEEVE_MCU_ID_RESULT	0x13	/* 3 bytes: x.xx */
#define ASIC2_SLEEVE_MCU_BATTERY	0x20
#define ASIC2_SLEEVE_MCU_BATTERY_RESULT	0x24	/* 4 bytes: chem pct flag voltage */
#define ASIC2_SLEEVE_MCU_EBAT_ON	0x30	/* no result */

#define ASIC2_SLEEVE_MCU_PAYLOAD_MAX	0x0f	/* low nibble of the reply byte */
#define ASIC2_SLEEVE_MCU_REPLY_MAX	(ASIC2_SLEEVE_MCU_PAYLOAD_MAX + 3)

/*
 * The microcontroller needs time to respond between the command and the
 * reply. A delay inside the same spi_message keeps chip select asserted
 * across the pause, which the protocol relies on.
 */
#define ASIC2_SLEEVE_MCU_REPLY_DELAY_US	10000

struct asic2_sleeve {
	struct device		*dev;
	struct spi_device	*eeprom;
	struct spi_device	*mcu;
	struct gpio_desc	*detect_gpio;
	struct gpio_desc	*nvram_gpio;
	struct gpio_desc	*power_gpio;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*opt_int_gpio;
	int			irq_detect;
	int			irq_opt_int;
	/* Serialises whole multi-message transactions, not single transfers:
	 * the EEPROM path is an RDSR poll followed by a separate read, which
	 * must not interleave with the jacket microcontroller's own
	 * command and reply exchange.
	 */
	struct mutex		lock;
};

/*
 * Wait for the ID EEPROM to report itself idle (WIP clear) before reading it.
 * spi_write_then_read() keeps chip select asserted across the command and
 * the status byte. A failed transfer is retried like a busy status, within
 * the same budget, and reported only if every attempt failed.
 */
static int asic2_sleeve_eeprom_ready(struct asic2_sleeve *sleeve)
{
	u8 cmd = ASIC2_SLEEVE_EEPROM_RDSR;
	unsigned int i;
	u8 status;
	int ret, last_ret = 0;

	for (i = 0; i < ASIC2_SLEEVE_EEPROM_READY_TRIES; i++) {
		ret = spi_write_then_read(sleeve->eeprom, &cmd, 1, &status, 1);
		if (ret) {
			last_ret = ret;
			continue;
		}

		if (!(status & ASIC2_SLEEVE_EEPROM_RDSR_WIP))
			return 0;

		last_ret = 0;
	}

	return last_ret ?: -ETIMEDOUT;
}

/*
 * ipaq_sleeve_ops::spi_read -- read @len bytes of the plugged-in
 * sleeve's ID EEPROM at @address. The callers are in
 * drivers/misc/ipaq-sleeve.c: ipaq_sleeve_insert() reads the 10 bytes from
 * the header byte through the device ID in one transaction, and
 * ipaq_sleeve_parse_record() reads the description in chunks and then the
 * fixed fields of the record behind it. Returns 0 on success.
 */
static int asic2_sleeve_spi_read(void *dev, unsigned short address,
				 unsigned char *data, unsigned short len)
{
	struct asic2_sleeve *sleeve = dev;
	u8 cmd[2];
	int ret;

	if (!data)
		return -EINVAL;

	cmd[0] = ASIC2_SLEEVE_EEPROM_READ(address);
	cmd[1] = address & 0xff;

	guard(mutex)(&sleeve->lock);

	ret = asic2_sleeve_eeprom_ready(sleeve);
	if (ret)
		return ret;

	return spi_write_then_read(sleeve->eeprom, cmd, sizeof(cmd), data, len);
}

/*
 * One command/reply exchange with the jacket microcontroller on CS1. @reply is the
 * expected reply byte, whose low nibble is the payload length; pass
 * @data == NULL for a command that produces no reply at all
 * (EBAT_ON is the only such command). Caller holds sleeve->lock.
 */
static int asic2_sleeve_mcu_cmd(struct asic2_sleeve *sleeve, u8 cmd, u8 reply,
				u8 *data)
{
	u8 rx[ASIC2_SLEEVE_MCU_REPLY_MAX];
	struct spi_transfer xfers[2] = { };
	struct spi_message msg;
	unsigned int payload, i;
	u8 checksum;
	u8 tx[3];
	int ret;

	tx[0] = ASIC2_SLEEVE_MCU_HEADER;
	tx[1] = cmd;
	tx[2] = cmd;			/* checksum of a one-byte payload */

	spi_message_init(&msg);

	xfers[0].tx_buf = tx;
	xfers[0].len = sizeof(tx);
	spi_message_add_tail(&xfers[0], &msg);

	if (!data) {
		/* No reply expected; chip select drops at message end. */
		return spi_sync(sleeve->mcu, &msg);
	}

	xfers[0].delay.value = ASIC2_SLEEVE_MCU_REPLY_DELAY_US;
	xfers[0].delay.unit = SPI_DELAY_UNIT_USECS;

	payload = reply & ASIC2_SLEEVE_MCU_PAYLOAD_MAX;

	/* header + reply byte + payload + checksum */
	xfers[1].rx_buf = rx;
	xfers[1].len = payload + 3;
	spi_message_add_tail(&xfers[1], &msg);

	ret = spi_sync(sleeve->mcu, &msg);
	if (ret)
		return ret;

	if (rx[0] != ASIC2_SLEEVE_MCU_HEADER || rx[1] != reply)
		return -EIO;

	checksum = rx[1];
	for (i = 0; i < payload; i++) {
		data[i] = rx[2 + i];
		checksum += rx[2 + i];
	}

	if (checksum != rx[2 + payload])
		return -EIO;

	return 0;
}

/*
 * ipaq_sleeve_ops::get_version -- the jacket microcontroller's own
 * firmware version, three bytes read as "x.xx".
 */
static int asic2_sleeve_get_version(void *dev, struct ipaq_sleeve_version *ver)
{
	struct asic2_sleeve *sleeve = dev;
	u8 data[3];
	int ret;

	if (!ver)
		return -EINVAL;

	guard(mutex)(&sleeve->lock);

	ret = asic2_sleeve_mcu_cmd(sleeve, ASIC2_SLEEVE_MCU_ID,
				   ASIC2_SLEEVE_MCU_ID_RESULT, data);
	if (ret)
		return ret;

	ver->d[0] = data[0];
	ver->d[1] = data[1];
	ver->d[2] = data[2];

	return 0;
}

/*
 * ipaq_sleeve_ops::read_battery -- an extended-battery jacket's own
 * battery telemetry. The reply carries four payload bytes (chemistry,
 * percent, flag, voltage) but the ops signature surfaces only the first
 * three.
 */
static int asic2_sleeve_read_battery(void *dev, unsigned char *chem,
				     unsigned char *percent, unsigned char *flag)
{
	struct asic2_sleeve *sleeve = dev;
	u8 data[4];
	int ret;

	guard(mutex)(&sleeve->lock);

	ret = asic2_sleeve_mcu_cmd(sleeve, ASIC2_SLEEVE_MCU_BATTERY,
				   ASIC2_SLEEVE_MCU_BATTERY_RESULT, data);
	if (ret)
		return ret;

	if (chem)
		*chem = data[0];
	if (percent)
		*percent = data[1];
	if (flag)
		*flag = data[2];

	return 0;
}

/*
 * ipaq_sleeve_ops::set_ebat -- tell an extended-battery jacket to start
 * supplying the host. There is no reply, so no checksum to verify.
 */
static int asic2_sleeve_set_ebat(void *dev)
{
	struct asic2_sleeve *sleeve = dev;

	guard(mutex)(&sleeve->lock);

	return asic2_sleeve_mcu_cmd(sleeve, ASIC2_SLEEVE_MCU_EBAT_ON, 0, NULL);
}

/*
 * ipaq_sleeve_ops::get_option_detect -- is a jacket physically clipped
 * on right now? The line is active-low and declared GPIO_ACTIVE_LOW in the
 * devicetree, so gpiod_get_value() == 1 means "jacket present".
 */
static int asic2_sleeve_get_option_detect(void *dev, int *result)
{
	struct asic2_sleeve *sleeve = dev;
	int val;

	val = gpiod_get_value_cansleep(sleeve->detect_gpio);
	if (val < 0)
		return val;

	if (result)
		*result = val;

	return 0;
}

/*
 * ipaq_sleeve_ops::control_egpio -- the lines of enum ipaq_sleeve_egpio
 * (include/linux/ipaq-sleeve.h), mapped onto the GPIO descriptors.
 *
 * IPAQ_EGPIO_CARD_RESET is deliberately not implemented. It is the
 * per-card PCMCIA reset, which belongs to the socket driver as the generic
 * reset line of soc_common, driven from the SS_RESET flag of the socket
 * state. The bus core only ever clears it, only when registering or
 * unregistering, and ignores the return value, so -EINVAL here is a no-op.
 */
static int asic2_sleeve_control_egpio(void *dev, int egpio, int state)
{
	struct asic2_sleeve *sleeve = dev;

	switch (egpio) {
	case IPAQ_EGPIO_OPT_NVRAM_ON:
		gpiod_set_value_cansleep(sleeve->nvram_gpio, state);
		break;
	case IPAQ_EGPIO_OPT_ON:
		gpiod_set_value_cansleep(sleeve->power_gpio, state);
		break;
	case IPAQ_EGPIO_OPT_RESET:
		gpiod_set_value_cansleep(sleeve->reset_gpio, state);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * ipaq_sleeve_ops::read_egpio -- no line of enum ipaq_sleeve_egpio is
 * readable on this backend. The four power and reset lines are GPIO
 * outputs, and the four PCMCIA status lines belong to the socket driver,
 * see the header comment. It is a stub instead of a NULL slot because the
 * bus core calls through the pointer without checking it.
 */
static int asic2_sleeve_read_egpio(void *dev, int nr)
{
	return -EINVAL;
}

/*
 * ipaq_sleeve_ops::egpio_irq -- IPAQ_EGPIO_OPT_INT only. The sleeve-device
 * interrupt of the connector is a bare SoC pin, separate from the PC Card
 * IRQ pin, so it is described as an ordinary GPIO and resolved here. The
 * four PCMCIA status lines belong to the socket driver. The line is
 * optional in the binding, so a board that does not describe it gets
 * -ENXIO, which a consumer can degrade on, instead of a failed probe.
 */
static int asic2_sleeve_egpio_irq(void *dev, int nr)
{
	struct asic2_sleeve *sleeve = dev;

	if (nr != IPAQ_EGPIO_OPT_INT)
		return -EINVAL;

	return sleeve->irq_opt_int;
}

static struct ipaq_sleeve_ops asic2_sleeve_ops = {
	.spi_read		= asic2_sleeve_spi_read,
	.get_option_detect	= asic2_sleeve_get_option_detect,
	.get_version		= asic2_sleeve_get_version,
	.set_ebat		= asic2_sleeve_set_ebat,
	.control_egpio		= asic2_sleeve_control_egpio,
	.read_egpio		= asic2_sleeve_read_egpio,
	.egpio_irq		= asic2_sleeve_egpio_irq,
	.read_battery		= asic2_sleeve_read_battery,
};

/*
 * Allocate one spi_device per chip select on the controller of the ASIC2 SPI
 * block. Neither device has a fixed identity to bind a devicetree child node
 * to, as both live on whichever jacket is plugged in, and the ID EEPROM is
 * what says which one that is, so they are allocated by hand at probe.
 *
 * Returns -EPROBE_DEFER if the SPI driver has not registered its controller
 * yet.
 */
static struct spi_device *asic2_sleeve_add_spi_device(struct spi_controller *ctlr,
						      u8 chip_select,
						      const char *modalias)
{
	struct spi_device *spi;
	int ret;

	spi = spi_alloc_device(ctlr);
	if (!spi)
		return ERR_PTR(-ENOMEM);

	spi_set_chipselect(spi, 0, chip_select);
	spi->cs_index_mask = BIT(0);
	spi->max_speed_hz = ASIC2_SLEEVE_SPI_HZ;
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	strscpy(spi->modalias, modalias, sizeof(spi->modalias));

	ret = spi_add_device(spi);
	if (ret) {
		spi_dev_put(spi);
		return ERR_PTR(ret);
	}

	return spi;
}

static int asic2_sleeve_add_spi_devices(struct asic2_sleeve *sleeve)
{
	struct spi_controller *ctlr;
	struct device_node *spi_np;

	spi_np = of_find_compatible_node(NULL, NULL, ASIC2_SLEEVE_SPI_COMPATIBLE);
	if (!spi_np)
		return dev_err_probe(sleeve->dev, -ENODEV,
				     "no %s devicetree node\n",
				     ASIC2_SLEEVE_SPI_COMPATIBLE);

	ctlr = of_find_spi_controller_by_node(spi_np);
	of_node_put(spi_np);
	if (!ctlr)
		return dev_err_probe(sleeve->dev, -EPROBE_DEFER,
				     "ASIC2 spi controller not registered yet\n");

	sleeve->eeprom = asic2_sleeve_add_spi_device(ctlr,
						     ASIC2_SLEEVE_CS_EEPROM,
						     "asic2-sleeve-eeprom");
	if (IS_ERR(sleeve->eeprom)) {
		put_device(&ctlr->dev);
		return dev_err_probe(sleeve->dev, PTR_ERR(sleeve->eeprom),
				     "failed to add ID EEPROM spi device\n");
	}

	sleeve->mcu = asic2_sleeve_add_spi_device(ctlr, ASIC2_SLEEVE_CS_MCU,
						  "asic2-sleeve-mcu");
	put_device(&ctlr->dev);
	if (IS_ERR(sleeve->mcu)) {
		spi_unregister_device(sleeve->eeprom);
		return dev_err_probe(sleeve->dev, PTR_ERR(sleeve->mcu),
				     "failed to add jacket controller spi device\n");
	}

	return 0;
}

static void asic2_sleeve_remove_spi_devices(struct asic2_sleeve *sleeve)
{
	spi_unregister_device(sleeve->mcu);
	spi_unregister_device(sleeve->eeprom);
}

static int asic2_sleeve_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct asic2_sleeve *sleeve;
	int ret;

	sleeve = devm_kzalloc(dev, sizeof(*sleeve), GFP_KERNEL);
	if (!sleeve)
		return -ENOMEM;

	sleeve->dev = dev;

	ret = devm_mutex_init(dev, &sleeve->lock);
	if (ret)
		return ret;

	/*
	 * All output rails start de-asserted: no rail may be live before the
	 * bus core has decided that a jacket is really there.
	 */
	sleeve->nvram_gpio = devm_gpiod_get(dev, "nvram", GPIOD_OUT_LOW);
	if (IS_ERR(sleeve->nvram_gpio))
		return dev_err_probe(dev, PTR_ERR(sleeve->nvram_gpio),
				     "failed to get nvram gpio\n");

	sleeve->power_gpio = devm_gpiod_get(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(sleeve->power_gpio))
		return dev_err_probe(dev, PTR_ERR(sleeve->power_gpio),
				     "failed to get power gpio\n");

	sleeve->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(sleeve->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(sleeve->reset_gpio),
				     "failed to get reset gpio\n");

	sleeve->detect_gpio = devm_gpiod_get(dev, "detect", GPIOD_IN);
	if (IS_ERR(sleeve->detect_gpio))
		return dev_err_probe(dev, PTR_ERR(sleeve->detect_gpio),
				     "failed to get detect gpio\n");

	/*
	 * Both jacket insertion and removal must reach the debounce and
	 * enumeration work of the bus core, so the interrupt is requested on
	 * both edges below.
	 */
	sleeve->irq_detect = gpiod_to_irq(sleeve->detect_gpio);
	if (sleeve->irq_detect < 0)
		return dev_err_probe(dev, sleeve->irq_detect,
				     "detect gpio has no irq\n");

	/*
	 * The sleeve-device interrupt (OPT_INT) of the connector is handed to
	 * whichever sleeve driver wants it through ipaq_sleeve_egpio_irq().
	 * Nothing after this point may fail because of it: a missing line, a
	 * line without an interrupt or a rejected trigger type only means
	 * that there is no OPT_INT source. -EPROBE_DEFER is the exception and
	 * is propagated, so that the probe is retried when the GPIO provider
	 * shows up.
	 */
	sleeve->opt_int_gpio = devm_gpiod_get_optional(dev, "opt-int", GPIOD_IN);
	if (IS_ERR(sleeve->opt_int_gpio)) {
		ret = PTR_ERR(sleeve->opt_int_gpio);
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_warn(dev,
			 "failed to get opt-int gpio (%d); continuing with no OPT_INT source\n",
			 ret);
		sleeve->opt_int_gpio = NULL;
	}

	sleeve->irq_opt_int = -ENXIO;
	if (sleeve->opt_int_gpio) {
		int opt_int_irq = gpiod_to_irq(sleeve->opt_int_gpio);

		if (opt_int_irq < 0) {
			dev_warn(dev,
				 "opt-int gpio has no irq (%d); continuing with no OPT_INT source\n",
				 opt_int_irq);
			goto opt_int_done;
		}

		/*
		 * An irq derived from a GPIO carries no trigger type until one
		 * is set, and a consumer only receives a bare irq number
		 * through ops->egpio_irq(), so it cannot know the polarity of
		 * the board's wiring. Use a rising edge. PXA GPIO interrupts
		 * are edge-only, so a level type is not available.
		 */
		ret = irq_set_irq_type(opt_int_irq, IRQ_TYPE_EDGE_RISING);
		if (ret) {
			dev_warn(dev,
				 "cannot set opt-int irq trigger (%d); continuing with no OPT_INT source\n",
				 ret);
			goto opt_int_done;
		}

		sleeve->irq_opt_int = opt_int_irq;
	}
opt_int_done:

	ret = asic2_sleeve_add_spi_devices(sleeve);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, sleeve);

	ret = ipaq_sleeve_register(&asic2_sleeve_ops, sleeve);
	if (ret) {
		asic2_sleeve_remove_spi_devices(sleeve);
		return dev_err_probe(dev, ret,
				     "failed to register sleeve bus backend\n");
	}

	/*
	 * Plain request_irq(), not devm_request_irq(): it must be freed
	 * *before* ipaq_sleeve_unregister() in .remove(), not after it via
	 * devm's own LIFO teardown, or a presence interrupt firing during
	 * that unregister's cancel_delayed_work_sync()/eject() could
	 * schedule new work against an already-unregistered backend.
	 */
	ret = request_irq(sleeve->irq_detect, ipaq_sleeve_presence_interrupt,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "asic2-sleeve-detect", NULL);
	if (ret) {
		ipaq_sleeve_unregister();
		asic2_sleeve_remove_spi_devices(sleeve);
		return dev_err_probe(dev, ret,
				     "failed to request option-detect irq\n");
	}

	return 0;
}

static void asic2_sleeve_remove(struct platform_device *pdev)
{
	struct asic2_sleeve *sleeve = platform_get_drvdata(pdev);

	free_irq(sleeve->irq_detect, NULL);
	ipaq_sleeve_unregister();
	asic2_sleeve_remove_spi_devices(sleeve);
}

/*
 * Presence changes while the system is suspended produce no edge this
 * driver sees: on h38xx sa1100_gpio_suspend() narrows GRER/GFER to wake
 * sources and clears GEDR, on h39xx pxa_gpio_suspend() clears GEDR
 * (drivers/gpio/gpio-sa1100.c, drivers/gpio/gpio-pxa.c), and neither
 * resume path replays one. Calling ipaq_sleeve_presence_interrupt() here
 * makes resume behave like one more edge arrived: the delayed work it
 * (re)arms samples the live detect level under ipaq_sleeve_lock, ejects a
 * jacket removed while suspended and enumerates one fitted into an empty
 * connector. It does not re-identify a jacket exchanged for another while
 * suspended -- both read present; see ipaq_sleeve_task_handler().
 */
static int asic2_sleeve_resume(struct device *dev)
{
	ipaq_sleeve_presence_interrupt(0, NULL);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(asic2_sleeve_pm_ops, NULL, asic2_sleeve_resume);

static struct platform_driver asic2_sleeve_driver = {
	.driver = {
		.name = "asic2-sleeve",
		.pm = pm_sleep_ptr(&asic2_sleeve_pm_ops),
	},
	.probe	= asic2_sleeve_probe,
	.remove	= asic2_sleeve_remove,
};
module_platform_driver(asic2_sleeve_driver);

MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("ASIC2 expansion-pack (sleeve) backend (HP iPAQ h38xx/h39xx)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:asic2-sleeve");
