// SPDX-License-Identifier: GPL-2.0-only
/*
 * HP iPAQ hx4700 CompactFlash socket board-ops.
 *
 * One CD/RDY pair, a plain active-high CF_RESET and a single 3.3 V rail
 * keyed off Vcc, with 3.3 V the only supported card voltage. Unlike the
 * h2200 socket, the card's address and data bus is buffered by the ASIC3
 * companion chip, so this file also programs that buffering.
 *
 * The lines are properties of the "pcmcia" devicetree node
 * ("hp,ipaq-hx4700-pcmcia"), which is disabled: the shared
 * pxa2xx_pcmcia_driver in pxa2xx_base.c is matched by name only and its
 * probe needs platform data made of C function pointers that no property
 * can express. hx4700_pcmcia_init() therefore looks the node up and
 * assigns it to the platform device's of_node before registering it,
 * which is enough for the gpiod requests below to resolve against it.
 *
 * The CF_3V3_ON line is not skt->gpio_bus_enable: that field follows
 * SS_OUTPUT_ENA, the PC Card output enable, which is not card power.
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/mfd/asic3.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include "soc_common.h"

/*
 * ASIC3 banks A and B, and most of C and D, buffer the CF socket. Bank C
 * bits 0-2 are the LED outputs, which leds-asic3 configures itself.
 */
static const u16 hx4700_cf_gpio_config[] = {
	/* Banks A and B in full: alt-function outputs, initial value 0. */
	ASIC3_CONFIG_GPIO(0, 1, 1, 0),  ASIC3_CONFIG_GPIO(1, 1, 1, 0),
	ASIC3_CONFIG_GPIO(2, 1, 1, 0),  ASIC3_CONFIG_GPIO(3, 1, 1, 0),
	ASIC3_CONFIG_GPIO(4, 1, 1, 0),  ASIC3_CONFIG_GPIO(5, 1, 1, 0),
	ASIC3_CONFIG_GPIO(6, 1, 1, 0),  ASIC3_CONFIG_GPIO(7, 1, 1, 0),
	ASIC3_CONFIG_GPIO(8, 1, 1, 0),  ASIC3_CONFIG_GPIO(9, 1, 1, 0),
	ASIC3_CONFIG_GPIO(10, 1, 1, 0), ASIC3_CONFIG_GPIO(11, 1, 1, 0),
	ASIC3_CONFIG_GPIO(12, 1, 1, 0), ASIC3_CONFIG_GPIO(13, 1, 1, 0),
	ASIC3_CONFIG_GPIO(14, 1, 1, 0), ASIC3_CONFIG_GPIO(15, 1, 1, 0),

	ASIC3_CONFIG_GPIO(16, 1, 1, 0), ASIC3_CONFIG_GPIO(17, 1, 1, 0),
	ASIC3_CONFIG_GPIO(18, 1, 1, 0), ASIC3_CONFIG_GPIO(19, 1, 1, 0),
	ASIC3_CONFIG_GPIO(20, 1, 1, 0), ASIC3_CONFIG_GPIO(21, 1, 1, 0),
	ASIC3_CONFIG_GPIO(22, 1, 1, 0), ASIC3_CONFIG_GPIO(23, 1, 1, 0),
	ASIC3_CONFIG_GPIO(24, 1, 1, 0), ASIC3_CONFIG_GPIO(25, 1, 1, 0),
	ASIC3_CONFIG_GPIO(26, 1, 1, 0), ASIC3_CONFIG_GPIO(27, 1, 1, 0),
	ASIC3_CONFIG_GPIO(28, 1, 1, 0), ASIC3_CONFIG_GPIO(29, 1, 1, 0),
	ASIC3_CONFIG_GPIO(30, 1, 1, 0), ASIC3_CONFIG_GPIO(31, 1, 1, 0),

	/* Bank C: the PC Card control signals. */
	ASIC3_GPIOC5_nCIOW,
	ASIC3_GPIOC6_nCIOR,
	ASIC3_GPIOC7_nPCE_1,
	ASIC3_GPIOC8_nPCE_2,
	ASIC3_GPIOC9_nPOE,
	ASIC3_GPIOC10_nPWE,
	ASIC3_GPIOC11_PSKTSEL,
	ASIC3_GPIOC12_nPREG,
	ASIC3_GPIOC13_nPWAIT,
	ASIC3_GPIOC14_nPIOIS16,
	ASIC3_GPIOC15_nPIOR,

	/* Bank D: the four CF-side inputs. */
	ASIC3_GPIOD4_CF_nCD,
	ASIC3_GPIOD11_nCIOIS16,
	ASIC3_GPIOD12_nCWAIT,
	ASIC3_GPIOD15_nPIOW,
};

/*
 * EXTCF_SELECT bits owned by this socket. CF1 is absent on this board but
 * the register is shared and keeps its sleep mode set. OWM_RESET holds the
 * 1-Wire master in reset, so it is cleared for every Vcc.
 */
#define HX4700_EXTCF_MASK	(ASIC3_EXTCF_OWM_RESET | \
				 ASIC3_EXTCF_CF1_SLEEP_MODE | \
				 ASIC3_EXTCF_CF0_PWAIT_EN | \
				 ASIC3_EXTCF_CF1_PWAIT_EN | \
				 ASIC3_EXTCF_CF0_BUF_EN | \
				 ASIC3_EXTCF_CF1_BUF_EN)
#define HX4700_EXTCF_BITS	(ASIC3_EXTCF_CF1_SLEEP_MODE | \
				 ASIC3_EXTCF_CF0_PWAIT_EN | \
				 ASIC3_EXTCF_CF0_BUF_EN)

struct hx4700_pcmcia_data {
	struct gpio_desc *power;
	struct asic3 *asic;
};

static void hx4700_pcmcia_put_asic3(void *pdev)
{
	put_device(&((struct platform_device *)pdev)->dev);
}

/*
 * The socket is not an MFD cell of ASIC3, so it reaches the chip through
 * the "hp,asic3" phandle and the driver data the ASIC3 driver stores. The
 * reference on the platform device is dropped when the socket goes away.
 */
static struct asic3 *hx4700_pcmcia_get_asic3(struct device *dev)
{
	struct device_node *np;
	struct platform_device *pdev;
	struct asic3 *asic;
	int ret;

	np = of_parse_phandle(dev->of_node, "hp,asic3", 0);
	if (!np) {
		dev_err(dev, "no hp,asic3 phandle\n");
		return ERR_PTR(-ENODEV);
	}

	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return ERR_PTR(dev_err_probe(dev, -EPROBE_DEFER,
					     "asic3 has no platform device yet\n"));

	asic = platform_get_drvdata(pdev);
	if (!asic) {
		put_device(&pdev->dev);
		return ERR_PTR(dev_err_probe(dev, -EPROBE_DEFER,
					     "asic3 has not finished probing\n"));
	}

	ret = devm_add_action_or_reset(dev, hx4700_pcmcia_put_asic3, pdev);
	if (ret)
		return ERR_PTR(ret);

	return asic;
}

static void hx4700_pcmcia_set_extcf(struct asic3 *asic)
{
	u32 sel;

	sel = asic3_read_register(asic, ASIC3_EXTCF_BASE + ASIC3_EXTCF_SELECT);
	sel = (sel & ~HX4700_EXTCF_MASK) | HX4700_EXTCF_BITS;
	asic3_write_register(asic, ASIC3_EXTCF_BASE + ASIC3_EXTCF_SELECT, sel);
}

/*
 * This runs once from .hw_init and does read-modify-write on registers the
 * ASIC3 GPIO chip also touches, without its lock. Every bit touched here
 * belongs to the socket: banks A and B have no other user, and the other
 * bits of banks C and D are outside the mask.
 */
static void hx4700_pcmcia_configure_asic3(struct asic3 *asic)
{
	unsigned int i;
	u32 sel;

	for (i = 0; i < ARRAY_SIZE(hx4700_cf_gpio_config); i++) {
		u16 config = hx4700_cf_gpio_config[i];
		unsigned int pin = ASIC3_CONFIG_GPIO_PIN(config);
		unsigned int alt = ASIC3_CONFIG_GPIO_ALT(config);
		unsigned int dir = ASIC3_CONFIG_GPIO_DIR(config);
		unsigned int init = ASIC3_CONFIG_GPIO_INIT(config);
		u32 base = ASIC3_BANK_TO_BASE(pin / ASIC3_GPIOS_PER_BANK);
		unsigned int bit = pin % ASIC3_GPIOS_PER_BANK;
		u32 val;

		/*
		 * Output level before direction, so a pin configured as an
		 * output never drives the stale level a previous owner left
		 * in ASIC3_GPIO_OUT.
		 */
		val = asic3_read_register(asic, base + ASIC3_GPIO_OUT);
		val = init ? (val | BIT(bit)) : (val & ~BIT(bit));
		asic3_write_register(asic, base + ASIC3_GPIO_OUT, val);

		val = asic3_read_register(asic, base + ASIC3_GPIO_DIRECTION);
		val = dir ? (val | BIT(bit)) : (val & ~BIT(bit));
		asic3_write_register(asic, base + ASIC3_GPIO_DIRECTION, val);

		val = asic3_read_register(asic, base + ASIC3_GPIO_ALT_FUNCTION);
		val = alt ? (val | BIT(bit)) : (val & ~BIT(bit));
		asic3_write_register(asic, base + ASIC3_GPIO_ALT_FUNCTION, val);
	}

	/*
	 * Put EXTCF_SELECT on the 1-Wire path. The 1-Wire master's reset
	 * pulse is left to the ASIC3 driver: pulsing it again at this point
	 * would wipe the clock divisor and interrupt enables of a ds1wm that
	 * has already probed.
	 */
	sel = asic3_read_register(asic, ASIC3_EXTCF_BASE + ASIC3_EXTCF_SELECT);
	sel &= ~(ASIC3_EXTCF_OWM_EN | ASIC3_EXTCF_OWM_SMB);
	sel |= ASIC3_EXTCF_OWM_EN;
	asic3_write_register(asic, ASIC3_EXTCF_BASE + ASIC3_EXTCF_SELECT, sel);

	hx4700_pcmcia_set_extcf(asic);
}

static int hx4700_pcmcia_hw_init(struct soc_pcmcia_socket *skt)
{
	struct device *dev = skt->socket.dev.parent;
	struct hx4700_pcmcia_data *data;
	int irq, ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->asic = hx4700_pcmcia_get_asic3(dev);
	if (IS_ERR(data->asic))
		return PTR_ERR(data->asic);

	/*
	 * soc_common derives the IRQs of CD and RDY from the gpiods and
	 * fills state->detect and state->ready from them.
	 */
	skt->stat[SOC_STAT_CD].name = "detect";
	skt->stat[SOC_STAT_RDY].name = "ready";
	ret = soc_pcmcia_request_gpiods(skt);
	if (ret)
		return ret;

	/*
	 * ASIC3 maps every hwirq level-low by default, which is the level
	 * card detect holds while a card is seated, and soc_pcmcia_hw_init()
	 * requests the IRQ without a trigger type. A card already in the
	 * slot at boot would keep the ASIC3 demultiplexer looping on a
	 * level it cannot clear until soc_pcmcia_hw_enable() sets both
	 * edges, so set the type before the IRQ is requested.
	 */
	irq = gpiod_to_irq(skt->stat[SOC_STAT_CD].desc);
	if (irq > 0)
		irq_set_irq_type(irq, IRQ_TYPE_EDGE_BOTH);

	/*
	 * Both outputs start de-asserted: the socket is unpowered until the
	 * PCMCIA core has a card to power.
	 */
	skt->gpio_reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(skt->gpio_reset))
		return PTR_ERR(skt->gpio_reset);

	data->power = devm_gpiod_get(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(data->power))
		return PTR_ERR(data->power);

	skt->driver_data = data;

	hx4700_pcmcia_configure_asic3(data->asic);

	return 0;
}

static int hx4700_pcmcia_configure_socket(struct soc_pcmcia_socket *skt,
					  const socket_state_t *state)
{
	struct hx4700_pcmcia_data *data = skt->driver_data;
	int on;

	switch (state->Vcc) {
	case 0:
		on = 0;
		break;
	case 33:
		on = 1;
		break;
	default:
		/* The socket has a single 3.3 V rail. */
		dev_err(skt->socket.dev.parent, "unsupported Vcc %u.%uV\n",
			state->Vcc / 10, state->Vcc % 10);
		return -EINVAL;
	}

	hx4700_pcmcia_set_extcf(data->asic);
	gpiod_set_value_cansleep(data->power, on);

	return 0;
}

static struct pcmcia_low_level hx4700_pcmcia_ops = {
	.owner			= THIS_MODULE,
	.first			= 0,
	.nr			= 1,
	.hw_init		= hx4700_pcmcia_hw_init,
	.socket_state		= soc_common_cf_socket_state,
	.configure_socket	= hx4700_pcmcia_configure_socket,
};

static void hx4700_pcmcia_release(struct device *dev)
{
	/* The device is static: there is nothing to free. */
}

static struct platform_device hx4700_pcmcia_device = {
	.name	= "pxa2xx-pcmcia",
	.id	= -1,
	.dev	= {
		.platform_data	= &hx4700_pcmcia_ops,
		.release	= hx4700_pcmcia_release,
	},
};

static int __init hx4700_pcmcia_init(void)
{
	struct device_node *np;
	int ret;

	if (!of_machine_is_compatible("hp,ipaq-hx4700"))
		return -ENODEV;

	np = of_find_compatible_node(NULL, NULL, "hp,ipaq-hx4700-pcmcia");
	if (!np)
		return -ENODEV;

	hx4700_pcmcia_device.dev.of_node = np;
	ret = platform_device_register(&hx4700_pcmcia_device);
	if (ret)
		of_node_put(np);

	return ret;
}
/*
 * ASIC3 and the EGPIO expander bind at device_initcall level. Registering
 * the socket earlier makes its first probe defer on their GPIOs while the
 * PCMCIA core is already live, which hangs a built-in kernel.
 */
late_initcall_sync(hx4700_pcmcia_init);

static void __exit hx4700_pcmcia_exit(void)
{
	struct device_node *np = hx4700_pcmcia_device.dev.of_node;

	platform_device_unregister(&hx4700_pcmcia_device);
	of_node_put(np);
}
module_exit(hx4700_pcmcia_exit);

MODULE_AUTHOR("Koen Kooi <koen@dominion.thruhere.net>");
MODULE_DESCRIPTION("HP iPAQ hx4700 PCMCIA/CompactFlash socket support");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pxa2xx-pcmcia");
