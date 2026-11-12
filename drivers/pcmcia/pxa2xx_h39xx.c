// SPDX-License-Identifier: GPL-2.0-only
/*
 * HP iPAQ h39xx (h3950/h3955/h3970/h3975) CompactFlash/PCMCIA socket
 * board-ops.
 *
 * The board has no slot of its own: PCMCIA/CF access needs an expansion
 * pack ("option jacket" or "sleeve") clipped onto the base connector. The
 * pack's socket wiring passes straight through to the PXA's own PCMCIA/CF
 * controller, which pxa2xx_base.c already programs, so this file is
 * board-ops plus a lifecycle. The bus strobes and the ready inputs are
 * muxed by the board file's pin configuration.
 *
 * Card detect and the card interrupt of both sockets (PCMCIA_CD0_N/IRQ0_N
 * and PCMCIA_CD1_N/IRQ1_N) are plain PXA GPIOs, and the reset of the card
 * in socket 0 is a bit of the ASIC2 GPIO bank. They are properties of the
 * "pcmcia" devicetree node ("hp,ipaq-h3900-pcmcia"), which is disabled:
 * the shared pxa2xx_pcmcia_driver in pxa2xx_base.c is matched by name only
 * and its probe needs platform data made of C function pointers that no
 * property can express. The socket device is therefore registered here,
 * with the node as its firmware node, once the sleeve bus has enumerated a
 * pack. IRQ0_N doubles as the card's RDY/BSY line, as the two are one
 * signal of a PC Card.
 *
 * CD0_N does not read correctly until the pack is powered: before that it
 * floats high whether or not a card is seated. The sleeve bus core powers
 * the pack as part of enumerating it, strictly before the socket exists,
 * so the first look at card detect already sees the real state. A pack
 * whose ID EEPROM could not be read still gets the sockets, as the card
 * detect lines are what the vendor firmware goes by.
 *
 * Socket 1 is registered as well: the UART of the Navman GPS pack is
 * reached through its I/O window, and MECR:NOS has to be set for that
 * socket to be selected at all. No reset line is known for it.
 *
 * The pack's supply is shared by whatever is plugged in and is not
 * switched per card, so configure_socket() only validates Vcc. The pack
 * reset (OPT_RESET) resets the whole pack, not one card, and stays with
 * the bus core.
 */

#include <linux/cleanup.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/ipaq-sleeve.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "soc_common.h"

static int h39xx_pcmcia_hw_init(struct soc_pcmcia_socket *skt)
{
	struct device *dev = skt->socket.dev.parent;
	int ret;

	skt->stat[SOC_STAT_CD].name = skt->nr ? "detect1" : "detect";
	skt->stat[SOC_STAT_RDY].name = skt->nr ? "ready1" : "ready";
	ret = soc_pcmcia_request_gpiods(skt);
	if (ret) {
		dev_err(dev, "cannot request detect/ready GPIOs: %d\n", ret);
		return ret;
	}

	if (skt->nr)
		return 0;

	/*
	 * Requested de-asserted: holding the card in reset forces card
	 * detect to read "no card", and configure_socket(), the only place
	 * the PCMCIA core releases the line from, would never run.
	 */
	skt->gpio_reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(skt->gpio_reset)) {
		ret = PTR_ERR(skt->gpio_reset);
		dev_err(dev, "cannot get the reset GPIO: %d\n", ret);
		return ret;
	}

	return 0;
}

/*
 * Logs against skt->socket.dev.parent, never &skt->socket.dev.
 * soc_pcmcia_remove_one() calls pcmcia_unregister_socket() and only then
 * drives this op with dead_socket, by which time the socket device has
 * been unregistered and its name freed, so dev_name(&skt->socket.dev) would
 * read freed memory on every sleeve removal and module unload. The parent
 * is the platform device this driver was probed from and outlives all of
 * it.
 */
static int h39xx_pcmcia_configure_socket(struct soc_pcmcia_socket *skt,
					 const socket_state_t *state)
{
	struct device *dev = skt->socket.dev.parent;

	switch (state->Vcc) {
	case 0:
	case 33:
		break;
	default:
		/*
		 * Rejecting 5 V is the safe direction to be wrong in: a 5 V
		 * card fails to configure instead of being powered at a
		 * voltage the pack may not supply. No 5 V card was available
		 * to test the other way.
		 */
		dev_err(dev, "unsupported Vcc %u.%uV\n",
			state->Vcc / 10, state->Vcc % 10);
		return -EINVAL;
	}

	/* Vpp, IREQ routing and speaker enable are ignored. */
	return 0;
}

static struct pcmcia_low_level h39xx_pcmcia_ops = {
	.owner			= THIS_MODULE,
	.first			= 0,
	.nr			= 2,
	.hw_init		= h39xx_pcmcia_hw_init,
	.socket_state		= soc_common_cf_socket_state,
	.configure_socket	= h39xx_pcmcia_configure_socket,
};

static struct platform_device *h39xx_pcmcia_pdev;
static DEFINE_MUTEX(h39xx_pcmcia_lock);

/*
 * @data is the struct ipaq_sleeve_device just added or about to be torn
 * down.
 *
 * Factory WinCE gates the sockets on jacket presence and card detect, not
 * on an ID EEPROM read, so an unidentified but physically present pack
 * still gets its sockets. An identified pack without a CF or PC Card slot
 * does not.
 *
 * PLATFORM_DEVID_NONE is mandatory so that dev_name() is exactly
 * "pxa2xx-pcmcia": the MEMC clock is registered with that dev_id and
 * pxa2xx_base.c does devm_clk_get(&dev->dev, NULL), so any other name
 * means no clock and a failed probe.
 */
static int h39xx_pcmcia_sleeve_notify(struct notifier_block *nb,
				      unsigned long action, void *data)
{
	struct ipaq_sleeve_device *sdev = data;
	struct platform_device_info info = {
		.name		= "pxa2xx-pcmcia",
		.id		= PLATFORM_DEVID_NONE,
		.data		= &h39xx_pcmcia_ops,
		.size_data	= sizeof(h39xx_pcmcia_ops),
	};
	struct device_node *np;
	unsigned int features;

	guard(mutex)(&h39xx_pcmcia_lock);

	switch (action) {
	case IPAQ_SLEEVE_EVENT_ADD:
		if (h39xx_pcmcia_pdev)
			break;

		features = ipaq_sleeve_features(sdev);
		if (sdev->id_valid &&
		    !SLEEVE_GET_CF(features) && !SLEEVE_GET_PCMCIA(features)) {
			pr_info("pxa2xx_h39xx: sleeve %04x:%04x carries no CF/PC Card slot, no socket registered\n",
				sdev->vendor, sdev->device);
			break;
		}
		if (!sdev->id_valid)
			pr_info("pxa2xx_h39xx: sleeve identity unavailable, registering the sockets anyway\n");
		if (SLEEVE_GET_CF(features) + SLEEVE_GET_PCMCIA(features) > 2)
			pr_info("pxa2xx_h39xx: sleeve %04x:%04x reports %u slots; only sockets 0 and 1 are wired\n",
				sdev->vendor, sdev->device,
				SLEEVE_GET_CF(features) + SLEEVE_GET_PCMCIA(features));

		np = of_find_compatible_node(NULL, NULL, "hp,ipaq-h3900-pcmcia");
		if (!np) {
			pr_err("pxa2xx_h39xx: no hp,ipaq-h3900-pcmcia node\n");
			break;
		}
		info.fwnode = of_fwnode_handle(np);

		h39xx_pcmcia_pdev = platform_device_register_full(&info);
		of_node_put(np);
		if (IS_ERR(h39xx_pcmcia_pdev)) {
			pr_err("pxa2xx_h39xx: platform_device_register_full() = %ld\n",
			       PTR_ERR(h39xx_pcmcia_pdev));
			h39xx_pcmcia_pdev = NULL;
		}
		break;

	case IPAQ_SLEEVE_EVENT_REMOVE:
		if (!h39xx_pcmcia_pdev)
			break;

		platform_device_unregister(h39xx_pcmcia_pdev);
		h39xx_pcmcia_pdev = NULL;
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block h39xx_pcmcia_sleeve_nb = {
	.notifier_call = h39xx_pcmcia_sleeve_notify,
};

static int __init h39xx_pcmcia_init(void)
{
	if (!of_machine_is_compatible("hp,ipaq-h3900"))
		return -ENODEV;

	return ipaq_sleeve_register_notifier(&h39xx_pcmcia_sleeve_nb);
}
/*
 * ipaq_sleeve_register_notifier() replays the currently enumerated sleeve
 * the moment it registers, so the order against the sleeve backend does not
 * matter. late_initcall and not late_initcall_sync leaves the later slot to
 * drivers/misc/navman-gps-sleeve.c, which consumes the same ADD replay and
 * must find the sockets already registered before it probes inside the
 * window of socket 1.
 */
late_initcall(h39xx_pcmcia_init);

static void __exit h39xx_pcmcia_exit(void)
{
	ipaq_sleeve_unregister_notifier(&h39xx_pcmcia_sleeve_nb);

	guard(mutex)(&h39xx_pcmcia_lock);
	if (h39xx_pcmcia_pdev) {
		platform_device_unregister(h39xx_pcmcia_pdev);
		h39xx_pcmcia_pdev = NULL;
	}
}
module_exit(h39xx_pcmcia_exit);

MODULE_AUTHOR("Koen Kooi <koen@dominion.thruhere.net>");
MODULE_DESCRIPTION("HP iPAQ h39xx PCMCIA/CompactFlash socket support");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pxa2xx-pcmcia");
