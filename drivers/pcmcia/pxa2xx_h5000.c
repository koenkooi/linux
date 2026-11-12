// SPDX-License-Identifier: GPL-2.0-only
/*
 * HP iPAQ h5xxx (h5000/h5400/h5450/h5500/h5550) CompactFlash/PCMCIA
 * socket board-ops.
 *
 * Like h39xx, the board has no slot of its own: PCMCIA/CF access needs an
 * expansion pack ("sleeve") clipped onto the base connector. The pack's
 * socket wiring passes straight through to the PXA's own PCMCIA/CF
 * controller, which pxa2xx_base.c already programs, so this file is
 * board-ops plus a lifecycle. The bus strobes and the ready input are muxed
 * by the board devicetree's pin configuration.
 *
 * Card status is what sets this board apart from the GPIO-wired sockets
 * (pxa2xx_h2200.c) and is why this file requests no gpiod: card detect and
 * the card interrupt are bits of the SAMCOP expansion-pack status register
 * (SAMCOP_PCMCIA_EPS), and SAMCOP's gpio_chip covers the GPA-GPE banks
 * only. They are read through the ipaq-sleeve bus ops
 * (ipaq_sleeve_read_egpio() and ipaq_sleeve_egpio_irq(), backed by
 * drivers/misc/samcop-sleeve.c) instead. soc_common supports status that is
 * filled in by hand: soc_common_pcmcia_skt_state() reads a gpiod only when
 * skt->stat[i].desc is set and calls ops->socket_state() last and
 * unconditionally, and ops->hw_init() runs before soc_pcmcia_hw_init()
 * request_irq()s any non-zero skt->stat[i].irq.
 *
 * The "pcmcia" devicetree node ("hp,ipaq-h5000-pcmcia") is disabled: the
 * shared pxa2xx_pcmcia_driver in pxa2xx_base.c is matched by name only and
 * its probe needs platform data made of C function pointers that no
 * property can express. The socket device is therefore registered here,
 * with the node as its firmware node, once the sleeve bus has enumerated a
 * pack that carries a CF or PC Card slot. It cannot exist earlier: the
 * sleeve calls made below warn and fail with no registered backend.
 *
 * Only socket 0 (CD0_N/IRQ0_N) is wired. Adding socket 1 would also need
 * pxa2xx_configure_sockets(), which hardcodes
 * pxa_smemc_set_pcmcia_socket(1), to set MECR_NOS.
 *
 * CD0_N interrupts on one edge at a time. SAMCOP_PCMCIA_IM selects falling
 * only for it, both at its reset default and as the WinCE firmware leaves
 * it, and CD0_N is active low, so a static configuration interrupts on
 * insertion and never on removal. h5000_pcmcia_socket_state() therefore
 * re-arms the edge to the opposite of the state just read, as the vendor's
 * pcmcia.dll does; soc_common's poll timer covers the residual race of a
 * transition falling between the read and the re-arm.
 *
 * SAMCOP_PCMCIA_TDW (PCMCIA data width) is left alone. Its reset value of
 * 1 is what the vendor firmware keeps: the h5550 OAL writes it once at
 * power-on, unconditionally and with no relation to the card, and never
 * touches it again.
 *
 * No OPT_RESET pulse is issued here. It resets the whole pack, not one
 * card, so it belongs to the bus core's insert sequence, which holds the
 * line deasserted. The vendor's WinCE sleeve drivers do pulse it after a
 * good ID EEPROM read, for a width the EEPROM record supplies (when
 * non-zero), then wait a record-supplied stabilisation time.
 */

#include <linux/cleanup.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/ipaq-sleeve.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "soc_common.h"

static int h5000_pcmcia_hw_init(struct soc_pcmcia_socket *skt)
{
	struct device *dev = skt->socket.dev.parent;
	int irq;

	/* Used only as request_irq()'s devname; there is no gpiod. */
	skt->stat[SOC_STAT_CD].name = "detect";

	irq = ipaq_sleeve_egpio_irq(IPAQ_EGPIO_PCMCIA_CD0_N);
	if (irq <= 0) {
		dev_err(dev, "no CD0_N irq (%d)\n", irq);
		return irq < 0 ? irq : -ENXIO;
	}
	skt->stat[SOC_STAT_CD].irq = irq;

	/*
	 * The card's own interrupt line, handed to card drivers as
	 * socket.pci_irq. It must be set: soc_pcmcia_init_one() leaves it at
	 * NO_IRQ, which is (unsigned int)-1, and pcmcia_resource.c tests it
	 * with a plain "if (s->pci_irq)", so the default hands a card driver
	 * a garbage IRQ number instead of degrading to "no IRQ".
	 *
	 * IRQ0_N doubles as the card's RDY/BSY line, as the ready/busy and
	 * interrupt request pins of a PC Card are one signal.
	 *
	 * soc_common_pcmcia_config_skt() calls irq_set_irq_type() on
	 * socket.pci_irq as io_irq comes and goes. The SAMCOP EPS irq_chip
	 * has no .irq_set_type, which __irq_set_trigger() tolerates; the
	 * edge configuration of the whole EPS tier is programmed once by the
	 * backend from SAMCOP_PCMCIA_IM.
	 */
	irq = ipaq_sleeve_egpio_irq(IPAQ_EGPIO_PCMCIA_IRQ0);
	if (irq <= 0) {
		dev_err(dev, "no IRQ0 irq (%d)\n", irq);
		return irq < 0 ? irq : -ENXIO;
	}
	skt->socket.pci_irq = irq;

	return 0;
}

/*
 * No memset of *state: soc_common_pcmcia_skt_state() pre-fills
 * bvd1 = bvd2 = 1 ("battery good") and zeroing them would report
 * SS_BATDEAD on every poll.
 *
 * read_egpio() returns the raw active-low EPS bit, so CD0_N reading 0 is a
 * card present. IRQ0 is used uninverted for "ready".
 *
 * vs_3v is hardcoded like soc_common_cf_socket_state() does, as the CF 3.0
 * spec has cards tie VS1 to ground and the board wires no VS signals; that
 * helper cannot be reused here because detect and ready need filling too.
 */
static void h5000_pcmcia_socket_state(struct soc_pcmcia_socket *skt,
				      struct pcmcia_state *state)
{
	state->detect = ipaq_sleeve_read_egpio(IPAQ_EGPIO_PCMCIA_CD0_N) == 0;
	state->ready = ipaq_sleeve_read_egpio(IPAQ_EGPIO_PCMCIA_IRQ0) != 0;
	state->vs_3v = 1;

	/* A no-op on a backend without the optional edge re-arm op. */
	ipaq_sleeve_rearm_cd_edge(IPAQ_EGPIO_PCMCIA_CD0_N, state->detect);
}

/*
 * Logs against skt->socket.dev.parent, never &skt->socket.dev.
 * soc_pcmcia_remove_one() calls pcmcia_unregister_socket() and only then
 * drives this op with dead_socket, by which time the socket device has been
 * unregistered and its name freed, so dev_name(&skt->socket.dev) would read
 * freed memory on every sleeve removal and module unload. The parent is the
 * platform device this driver was probed from and outlives all of it.
 */
static int h5000_pcmcia_configure_socket(struct soc_pcmcia_socket *skt,
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

	/*
	 * Per-card reset. skt->gpio_reset stays NULL (the line is
	 * SAMCOP_PCMCIA_CC_RESET, a register bit reached through the sleeve
	 * ops), so soc_common's generic reset handling does nothing and
	 * SS_RESET is applied here.
	 *
	 * Pack power (OPT_ON), the NVRAM rail (OPT_NVRAM_ON) and the
	 * whole-pack OPT_RESET belong to drivers/misc/samcop-sleeve.c and the
	 * bus core. The pack rail is shared by whatever is plugged in and is
	 * not switched per socket_state_t request.
	 */
	ipaq_sleeve_write_egpio(IPAQ_EGPIO_CARD_RESET,
				!!(state->flags & SS_RESET));

	/* Vpp, IREQ routing and speaker enable are ignored. */
	return 0;
}

static struct pcmcia_low_level h5000_pcmcia_ops = {
	.owner			= THIS_MODULE,
	.first			= 0,
	.nr			= 1,
	.hw_init		= h5000_pcmcia_hw_init,
	.socket_state		= h5000_pcmcia_socket_state,
	.configure_socket	= h5000_pcmcia_configure_socket,
};

static struct platform_device *h5000_pcmcia_pdev;
static DEFINE_MUTEX(h5000_pcmcia_lock);

/*
 * @data is the struct ipaq_sleeve_device just added or about to be torn
 * down.
 *
 * PLATFORM_DEVID_NONE is mandatory so that dev_name() is exactly
 * "pxa2xx-pcmcia": the MEMC clock is registered with that dev_id and
 * pxa2xx_base.c does devm_clk_get(&dev->dev, NULL), so any other name
 * means no clock and a failed probe.
 */
static int h5000_pcmcia_sleeve_notify(struct notifier_block *nb,
				      unsigned long action, void *data)
{
	struct ipaq_sleeve_device *sdev = data;
	struct platform_device_info info = {
		.name		= "pxa2xx-pcmcia",
		.id		= PLATFORM_DEVID_NONE,
		.data		= &h5000_pcmcia_ops,
		.size_data	= sizeof(h5000_pcmcia_ops),
	};
	struct device_node *np;
	unsigned int features;

	guard(mutex)(&h5000_pcmcia_lock);

	switch (action) {
	case IPAQ_SLEEVE_EVENT_ADD:
		if (h5000_pcmcia_pdev)
			break;

		features = ipaq_sleeve_features(sdev);
		if (!SLEEVE_GET_CF(features) && !SLEEVE_GET_PCMCIA(features)) {
			pr_info("pxa2xx_h5000: sleeve %04x:%04x carries no CF/PC Card slot, no socket registered\n",
				sdev->vendor, sdev->device);
			break;
		}
		if (SLEEVE_GET_CF(features) > 1 || SLEEVE_GET_PCMCIA(features) > 1)
			pr_info("pxa2xx_h5000: sleeve %04x:%04x reports %u slots; only socket 0 (CD0_N/IRQ0) is wired\n",
				sdev->vendor, sdev->device,
				SLEEVE_GET_CF(features) + SLEEVE_GET_PCMCIA(features));

		np = of_find_compatible_node(NULL, NULL, "hp,ipaq-h5000-pcmcia");
		if (!np) {
			pr_err("pxa2xx_h5000: no hp,ipaq-h5000-pcmcia node\n");
			break;
		}
		info.fwnode = of_fwnode_handle(np);

		h5000_pcmcia_pdev = platform_device_register_full(&info);
		of_node_put(np);
		if (IS_ERR(h5000_pcmcia_pdev)) {
			pr_err("pxa2xx_h5000: platform_device_register_full() = %ld\n",
			       PTR_ERR(h5000_pcmcia_pdev));
			h5000_pcmcia_pdev = NULL;
		}
		break;

	case IPAQ_SLEEVE_EVENT_REMOVE:
		if (!h5000_pcmcia_pdev)
			break;

		platform_device_unregister(h5000_pcmcia_pdev);
		h5000_pcmcia_pdev = NULL;
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block h5000_pcmcia_sleeve_nb = {
	.notifier_call = h5000_pcmcia_sleeve_notify,
};

static int __init h5000_pcmcia_init(void)
{
	if (!of_machine_is_compatible("hp,ipaq-h5000"))
		return -ENODEV;

	return ipaq_sleeve_register_notifier(&h5000_pcmcia_sleeve_nb);
}
/*
 * ipaq_sleeve_register_notifier() replays the currently enumerated sleeve
 * the moment it registers, so the order against the sleeve backend does not
 * matter. late_initcall and not late_initcall_sync leaves the later slot to
 * drivers/misc/navman-gps-sleeve.c, which consumes the same ADD replay and
 * must find this socket already registered before it probes inside the
 * socket's window.
 */
late_initcall(h5000_pcmcia_init);

static void __exit h5000_pcmcia_exit(void)
{
	ipaq_sleeve_unregister_notifier(&h5000_pcmcia_sleeve_nb);

	guard(mutex)(&h5000_pcmcia_lock);
	if (h5000_pcmcia_pdev) {
		platform_device_unregister(h5000_pcmcia_pdev);
		h5000_pcmcia_pdev = NULL;
	}
}
module_exit(h5000_pcmcia_exit);

MODULE_AUTHOR("Koen Kooi <koen@dominion.thruhere.net>");
MODULE_DESCRIPTION("HP iPAQ h5xxx PCMCIA/CompactFlash socket support");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pxa2xx-pcmcia");
