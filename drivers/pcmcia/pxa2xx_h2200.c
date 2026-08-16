// SPDX-License-Identifier: GPL-2.0-only
/*
 * HP iPAQ h2200 CompactFlash/PCMCIA socket board-ops.
 *
 * Translated from the old out-of-tree pxa2xx_h2200.c (Koen
 * Kooi, 2004), which was never mainlined -- soc_common.h's IRQ-array API
 * that file used (struct pcmcia_irqs, soc_pcmcia_request_irqs()) is long
 * gone, and pxa_gpio_mode()/GPLR/GPSR direct register access has been
 * replaced by gpiod. The socket's actual electrical behaviour is kept
 * faithful to that original: one CD/RDY pair, a plain-high CF_RESET, and
 * CF_POWER_EN/CF_ADD_EN_N/CF_BUFF_EN flipped together as a single
 * "socket powered" group keyed off Vcc, nothing fancier.
 *
 * Shaped after drivers/pcmcia/pxa2xx_sharpsl.c (self-contained
 * pcmcia_low_level ops + a "pxa2xx-pcmcia" platform_device registered
 * from this file's own gated initcall, not from the board file) and
 * drivers/pcmcia/sa1100_generic.c / sa1100_h3600.c (the modern
 * plain-GPIO CF socket idiom: named gpiod lookups into skt->stat[]/
 * skt->gpio_reset via soc_pcmcia_request_gpiods(), rather than sharpsl's
 * scoop-register-specific approach -- h2200 has no scoop chip, just raw
 * PXA GPIOs, so it fits the sa1100 shape more closely).
 *
 * The GPIO lines are the properties of the "pcmcia" devicetree node
 * ("hp,ipaq-h2200-pcmcia"), which is disabled: the shared
 * pxa2xx_pcmcia_driver in pxa2xx_base.c is matched by name only, and its
 * probe needs platform data made of C function pointers that no property
 * can express. h2200_pcmcia_init() therefore looks the node up and assigns
 * it to the platform device's of_node before registering it, which is
 * enough for the gpiod requests below to resolve against the node.
 *
 * CONFIG_PCMCIA_PXA2XX's "depends on ... && PXA_SHARPSL" needed relaxing
 * to admit MACH_H2200 -- see drivers/pcmcia/Kconfig.
 */

#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "soc_common.h"

struct h2200_pcmcia_data {
	struct gpio_desc *power_en;
	struct gpio_desc *add_en;
	struct gpio_desc *buff_en;
};

static int h2200_pcmcia_hw_init(struct soc_pcmcia_socket *skt)
{
	struct device *dev = skt->socket.dev.parent;
	struct h2200_pcmcia_data *data;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/*
	 * CD/RDY: soc_pcmcia_request_gpiods() looks these two up by con_id
	 * against the "pcmcia" devicetree node, then soc_common's own generic
	 * code derives their IRQs via gpiod_to_irq() and pre-fills
	 * state->detect/ready before .socket_state() ever runs -- see
	 * soc_common_pcmcia_skt_state().
	 */
	skt->stat[SOC_STAT_CD].name = "detect";
	skt->stat[SOC_STAT_RDY].name = "ready";
	ret = soc_pcmcia_request_gpiods(skt);
	if (ret)
		return ret;

	/*
	 * CF_RESET maps 1:1 onto soc_common's generic gpio_reset field --
	 * soc_common_pcmcia_config_skt() toggles it from state->flags &
	 * SS_RESET itself once .configure_socket() returns 0, so it needs
	 * no handling of its own below beyond the suspend-time assert in
	 * h2200_pcmcia_socket_suspend(). Requested asserted (held in
	 * reset) so the card stays quiescent until the PCMCIA core
	 * actually configures the socket.
	 */
	skt->gpio_reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(skt->gpio_reset))
		return PTR_ERR(skt->gpio_reset);

	/*
	 * CF_POWER_EN/CF_ADD_EN_N/CF_BUFF_EN: no separate address/buffer
	 * enable control point exists on this board's CF socket the way
	 * soc_common's generic gpio_bus_enable (keyed off SS_OUTPUT_ENA)
	 * assumes, so these stay a bespoke three-GPIO group toggled
	 * together in h2200_pcmcia_configure_socket() instead. Requested
	 * de-asserted (socket unpowered) as the safe initial state.
	 */
	data->power_en = devm_gpiod_get(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(data->power_en))
		return PTR_ERR(data->power_en);

	data->add_en = devm_gpiod_get(dev, "add-en", GPIOD_OUT_LOW);
	if (IS_ERR(data->add_en))
		return PTR_ERR(data->add_en);

	data->buff_en = devm_gpiod_get(dev, "buff-en", GPIOD_OUT_LOW);
	if (IS_ERR(data->buff_en))
		return PTR_ERR(data->buff_en);

	skt->driver_data = data;

	return 0;
}

static int h2200_pcmcia_configure_socket(struct soc_pcmcia_socket *skt,
					  const socket_state_t *state)
{
	struct h2200_pcmcia_data *data = skt->driver_data;
	int on;

	switch (state->Vcc) {
	case 0:
		on = 0;
		break;
	case 33:
	case 50:
		on = 1;
		break;
	default:
		dev_err(&skt->socket.dev, "unsupported Vcc %u.%uV\n",
			state->Vcc / 10, state->Vcc % 10);
		return -EINVAL;
	}

	/* Vpp, IREQ routing and speaker enable are silently ignored. */
	gpiod_set_value_cansleep(data->power_en, on);
	gpiod_set_value_cansleep(data->add_en, on);
	gpiod_set_value_cansleep(data->buff_en, on);

	return 0;
}

static void h2200_pcmcia_socket_suspend(struct soc_pcmcia_socket *skt)
{
	/* Hold the card in reset across suspend. */
	gpiod_set_value_cansleep(skt->gpio_reset, 1);
}

static struct pcmcia_low_level h2200_pcmcia_ops = {
	.owner			= THIS_MODULE,
	.first			= 0,
	.nr			= 1,
	.hw_init		= h2200_pcmcia_hw_init,
	/*
	 * CD/RDY are already read into *state by soc_common's generic
	 * gpiod path above; the CF 3.0 spec ties VS1 to ground on every
	 * real card, so soc_common_cf_socket_state()'s hard-coded vs_3v=1
	 * is all that's left to do -- see its own comment in soc_common.c.
	 */
	.socket_state		= soc_common_cf_socket_state,
	.configure_socket	= h2200_pcmcia_configure_socket,
	.socket_suspend		= h2200_pcmcia_socket_suspend,
};

static void h2200_pcmcia_release(struct device *dev)
{
	/* The device is static: there is nothing to free. */
}

static struct platform_device h2200_pcmcia_device = {
	.name	= "pxa2xx-pcmcia",
	.id	= -1,
	.dev	= {
		.platform_data	= &h2200_pcmcia_ops,
		.release	= h2200_pcmcia_release,
	},
};

static int __init h2200_pcmcia_init(void)
{
	struct device_node *np;
	int ret;

	if (!of_machine_is_compatible("hp,ipaq-h2200"))
		return -ENODEV;

	np = of_find_compatible_node(NULL, NULL, "hp,ipaq-h2200-pcmcia");
	if (!np)
		return -ENODEV;

	h2200_pcmcia_device.dev.of_node = np;
	ret = platform_device_register(&h2200_pcmcia_device);
	if (ret)
		of_node_put(np);

	return ret;
}
fs_initcall(h2200_pcmcia_init);

static void __exit h2200_pcmcia_exit(void)
{
	struct device_node *np = h2200_pcmcia_device.dev.of_node;

	platform_device_unregister(&h2200_pcmcia_device);
	of_node_put(np);
}
module_exit(h2200_pcmcia_exit);

MODULE_AUTHOR("Koen Kooi <koen@dominion.thruhere.net>");
MODULE_DESCRIPTION("HP iPAQ h2200 PCMCIA/CompactFlash socket support");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pxa2xx-pcmcia");
