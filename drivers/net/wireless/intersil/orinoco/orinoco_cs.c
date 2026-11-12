// SPDX-License-Identifier: GPL-2.0-only
/*
 *   orinoco_cs.c - PCMCIA glue for Hermes/HFA384x ("Orinoco") CF/PCMCIA
 *   802.11b cards.
 *
 *   probe() allocates the orinoco_private/wiphy pair, maps the I/O
 *   window, requests the card IRQ and hands off to orinoco_init() and
 *   orinoco_if_add().
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <pcmcia/cistpl.h>
#include <pcmcia/cisreg.h>
#include <pcmcia/ds.h>

#include "hermes.h"
#include "orinoco.h"

#define DRV_NAME "orinoco_cs"

/* PCMCIA specific device information (goes in the card field of
 * struct orinoco_private)
 */
struct orinoco_pccard {
	struct pcmcia_device	*p_dev;

	/* pcmcia_reset_card() bounces the socket through the PCMCIA
	 * layer's suspend/resume callbacks; this flag keeps
	 * orinoco_cs_suspend()/_resume() from stacking a full down/up
	 * cycle on top of a hard reset already in progress.
	 */
	unsigned long hard_reset_in_progress;
};

static int orinoco_cs_hard_reset(struct orinoco_private *priv)
{
	struct orinoco_pccard *card = priv->card;
	struct pcmcia_device *link = card->p_dev;
	int err;

	/* We need atomic ops here, because we're not holding the lock */
	set_bit(0, &card->hard_reset_in_progress);

	err = pcmcia_reset_card(link->socket);
	if (!err)
		msleep(100);

	clear_bit(0, &card->hard_reset_in_progress);

	return err;
}

/* The two card resets a genesis-mode (RAM Primary) download uses. Both
 * are COR/CCSR writes through the attribute window and neither reverts
 * anything but the card's own CPU state.
 */
static int orinoco_cs_cor_sreset(struct orinoco_private *priv)
{
	struct orinoco_pccard *card = priv->card;
	struct pcmcia_device *link = card->p_dev;
	u8 val;
	int err;

	err = pcmcia_read_config_byte(link, CISREG_COR, &val);
	if (err)
		return err;

	err = pcmcia_write_config_byte(link, CISREG_COR, val | COR_SOFT_RESET);
	if (err)
		return err;
	fsleep(2000);

	err = pcmcia_write_config_byte(link, CISREG_COR, val & ~COR_SOFT_RESET);
	if (err)
		return err;
	fsleep(2000);

	return 0;
}

static int orinoco_cs_genesis_reset(struct orinoco_private *priv, u8 hcr)
{
	struct orinoco_pccard *card = priv->card;
	struct pcmcia_device *link = card->p_dev;
	u8 old_cor;
	int err;

	err = pcmcia_read_config_byte(link, CISREG_COR, &old_cor);
	if (err)
		return err;

	err = pcmcia_write_config_byte(link, CISREG_COR, old_cor | COR_SOFT_RESET);
	if (err)
		return err;
	fsleep(10000);

	/* Set up genesis mode */
	err = pcmcia_write_config_byte(link, CISREG_CCSR, hcr);
	if (err)
		return err;
	fsleep(10000);

	err = pcmcia_write_config_byte(link, CISREG_COR, old_cor & ~COR_SOFT_RESET);
	if (err)
		return err;
	fsleep(10000);

	return 0;
}

static bool force_16bit = true;
module_param(force_16bit, bool, 0444);
MODULE_PARM_DESC(force_16bit,
		 "Request a 16-bit I/O data path explicitly (default) instead of IO_DATA_PATH_WIDTH_AUTO");

/**
 *	orinoco_cs_config_check	-	accept the first usable CIS config
 *	@link: pcmcia device
 *	@priv_data: unused, required by pcmcia_loop_config()'s callback shape
 *
 *	Hermes presents a fixed 16-bit register interface
 *	(hermes_struct_init() below passes HERMES_16BIT_REGSPACING), so a
 *	16-bit data path is what the chip needs. Whether to request that
 *	width explicitly or let IO_DATA_PATH_WIDTH_AUTO negotiate it from
 *	nIOIS16 is a socket question rather than a chip one: a host that
 *	drives 16-bit cycles without regard to nIOIS16 reads back 0xffff on
 *	sockets that expect the negotiation. force_16bit selects between the
 *	two.
 */
static int orinoco_cs_config_check(struct pcmcia_device *link, void *priv_data)
{
	if (link->config_index == 0)
		return -EINVAL;

	link->resource[0]->flags &= ~IO_DATA_PATH_WIDTH;
	link->resource[0]->flags |= force_16bit ? IO_DATA_PATH_WIDTH_16 :
						  IO_DATA_PATH_WIDTH_AUTO;

	return pcmcia_request_io(link);
}

/**
 *	orinoco_cs_probe	-	attach a Hermes PCMCIA interface
 *	@link: pcmcia device
 *
 *	Allocate the wiphy-backed private structure first, request the IRQ
 *	against core.c's orinoco_interrupt(), enable the device, then
 *	orinoco_init() + orinoco_if_add().
 */
static int orinoco_cs_probe(struct pcmcia_device *link)
{
	struct orinoco_private *priv;
	struct orinoco_pccard *card;
	void __iomem *io_addr;
	int ret;

	priv = alloc_orinocodev(sizeof(*card), &link->dev,
				orinoco_cs_hard_reset, NULL);
	if (!priv)
		return -ENOMEM;
	card = priv->card;
	priv->cor_sreset = orinoco_cs_cor_sreset;
	priv->genesis_reset = orinoco_cs_genesis_reset;

	/* Link both structures together */
	card->p_dev = link;
	link->priv = priv;

	link->config_flags |= CONF_ENABLE_IRQ | CONF_AUTO_SET_IO |
		CONF_AUTO_CHECK_VCC;

	ret = pcmcia_loop_config(link, orinoco_cs_config_check, NULL);
	if (ret) {
		dev_err(&link->dev,
			"orinoco_cs: no suitable CIS configuration found\n");
		goto failed;
	}

	if (!link->irq) {
		dev_err(&link->dev, "orinoco_cs: no IRQ assigned\n");
		ret = -ENODEV;
		goto failed;
	}

	io_addr = devm_ioport_map(&link->dev, link->resource[0]->start,
				  resource_size(link->resource[0]));
	if (!io_addr) {
		ret = -ENOMEM;
		goto failed;
	}

	/* The hermes structure must be valid before the IRQ is requested:
	 * orinoco_interrupt() dereferences priv->hw as its first hardware
	 * access, and the line is shared. Until orinoco_if_add() below,
	 * the handler sees priv->ndev == NULL, but never dereferences it
	 * before finding an enabled event -- and the chip's event mask
	 * stays clear until the interface is brought up.
	 */
	hermes_struct_init(&priv->hw, io_addr, HERMES_16BIT_REGSPACING);

	ret = pcmcia_request_irq(link, orinoco_interrupt);
	if (ret)
		goto failed;

	ret = pcmcia_enable_device(link);
	if (ret)
		goto failed;

	ret = orinoco_init(priv);
	if (ret) {
		dev_err(&link->dev, "orinoco_cs: orinoco_init() failed: %d\n",
			ret);
		goto failed;
	}

	/* Register an interface with the stack */
	ret = orinoco_if_add(priv, link->resource[0]->start, link->irq, NULL);
	if (ret) {
		dev_err(&link->dev, "orinoco_cs: orinoco_if_add() failed: %d\n",
			ret);
		goto failed_unregister;
	}

	dev_info(&link->dev, "orinoco_cs: %s: firmware %s, MAC %pM\n",
		 priv->ndev->name, priv_to_wiphy(priv)->fw_version,
		 priv_to_wiphy(priv)->perm_addr);

	return 0;

failed_unregister:
	/* orinoco_init() succeeded, so the wiphy is registered; only this
	 * path needs the matching unregister (free_orinocodev() only
	 * frees).
	 */
	wiphy_unregister(priv_to_wiphy(priv));
failed:
	/* Safe on every path: releases whatever of the I/O windows,
	 * configuration and IRQ was actually claimed. Freeing the IRQ here
	 * (dev_id is link->priv) is what makes the free_orinocodev() after
	 * it safe.
	 */
	pcmcia_disable_device(link);
	free_orinocodev(priv);
	return ret;
}

/**
 *	orinoco_cs_remove	-	unplug a Hermes PCMCIA interface
 *	@link: pcmcia device
 *
 *	Mark the hardware unavailable (any late orinoco_lock() taker bails
 *	out with -EBUSY), wait out the deferred work that is already past
 *	that guard, tear down the net_device, release the PCMCIA resources
 *	-- which also frees the IRQ, so no handler can run afterwards --
 *	then unregister and free the wiphy. The devm_ioport_map() mapping
 *	is released by devres as the device detaches, after this returns;
 *	nothing touches the I/O window past pcmcia_disable_device().
 */
static void orinoco_cs_remove(struct pcmcia_device *link)
{
	struct orinoco_private *priv = link->priv;
	unsigned long flags;

	dev_info(&link->dev, "orinoco_cs: remove\n");

	/* We're committed to taking the device away now, so mark the
	 * hardware as unavailable. This has to come before the net_device
	 * teardown, not after it: it is the only thing that stops the
	 * deferred work in core.c from running against a card that is on
	 * its way out.
	 */
	priv->hw.ops->lock_irqsave(&priv->lock, &flags);
	priv->hw_unavailable++;
	priv->hw.ops->unlock_irqrestore(&priv->lock, &flags);

	/* The flag above only turns back work that has not yet taken
	 * priv->lock. Work already past its orinoco_lock() latched
	 * priv->ndev on entry and goes on using that pointer -- across a
	 * 100 ms msleep() in the reset path -- and the scan-results work
	 * never consults hw_unavailable at all. orinoco_if_del() below
	 * free_netdev()s exactly that memory, so drain the whole set here
	 * first. free_orinocodev() issues the same calls again later,
	 * which is harmless.
	 */
	cancel_work_sync(&priv->reset_work);
	cancel_work_sync(&priv->link_work);
	cancel_work_sync(&priv->join_work);
	cancel_work_sync(&priv->process_scan);
	tasklet_kill(&priv->rx_tasklet);

	orinoco_if_del(priv);

	pcmcia_disable_device(link);

	wiphy_unregister(priv_to_wiphy(priv));
	free_orinocodev(priv);
}

static int orinoco_cs_suspend(struct pcmcia_device *link)
{
	struct orinoco_private *priv = link->priv;
	struct orinoco_pccard *card = priv->card;

	/* A hard reset bounces the socket through suspend/resume; doubling
	 * it with a full down/up cycle is both pointless and racy against
	 * the reset work that triggered it (see struct orinoco_pccard).
	 */
	if (!test_bit(0, &card->hard_reset_in_progress))
		orinoco_down(priv);

	return 0;
}

static int orinoco_cs_resume(struct pcmcia_device *link)
{
	struct orinoco_private *priv = link->priv;
	struct orinoco_pccard *card = priv->card;
	int err = 0;

	if (!test_bit(0, &card->hard_reset_in_progress))
		err = orinoco_up(priv);

	return err;
}

static const struct pcmcia_device_id orinoco_cs_ids[] = {
	PCMCIA_DEVICE_MANF_CARD(0x0101, 0x0777), /* 3Com AirConnect PCI 777A */
	PCMCIA_DEVICE_MANF_CARD(0x016b, 0x0001), /* Ericsson WLAN Card C11 */
	/* Nortel Networks eMobility 802.11 Wireless Adapter */
	PCMCIA_DEVICE_MANF_CARD(0x01eb, 0x080a),
	PCMCIA_DEVICE_MANF_CARD(0x0261, 0x0002), /* AirWay 802.11 Adapter (PCMCIA) */
	PCMCIA_DEVICE_MANF_CARD(0x0268, 0x0001), /* ARtem Onair */
	PCMCIA_DEVICE_MANF_CARD(0x0268, 0x0003), /* ARtem Onair Comcard 11 */
	PCMCIA_DEVICE_MANF_CARD(0x026f, 0x0305), /* Buffalo WLI-PCM-S11 */
	PCMCIA_DEVICE_MANF_CARD(0x02aa, 0x0002), /* ASUS SpaceLink WL-100 */
	PCMCIA_DEVICE_MANF_CARD(0x02ac, 0x0002), /* SpeedStream SS1021 Wireless Adapter */
	PCMCIA_DEVICE_MANF_CARD(0x02ac, 0x3021), /* SpeedStream Wireless Adapter */
	PCMCIA_DEVICE_MANF_CARD(0x14ea, 0xb001), /* PLANEX RoadLannerWave GW-NS11H */
	PCMCIA_DEVICE_PROD_ID12("3Com", "3CRWE737A AirConnect Wireless LAN PC Card",
				0x41240e5b, 0x56010af3),
	PCMCIA_DEVICE_PROD_ID12("Allied Telesyn", "AT-WCL452 Wireless PCMCIA Radio",
				0x5cd01705, 0x4271660f),
	PCMCIA_DEVICE_PROD_ID12("ASUS", "802_11B_CF_CARD_25", 0x78fc06ee, 0x45a50c1e),
	PCMCIA_DEVICE_PROD_ID12("ASUS", "802_11b_PC_CARD_25", 0x78fc06ee, 0xdb9aa842),
	PCMCIA_DEVICE_PROD_ID12("Avaya Communication", "Avaya Wireless PC Card",
				0xd8a43b78, 0x0d341169),
	PCMCIA_DEVICE_PROD_ID12("BENQ", "AWL100 PCMCIA ADAPTER", 0x35dadc74, 0x01f7fedb),
	PCMCIA_DEVICE_PROD_ID12("Cabletron", "RoamAbout 802.11 DS", 0x32d445f5, 0xedeffd90),
	PCMCIA_DEVICE_PROD_ID12("D-Link Corporation", "D-Link DWL-650H 11Mbps WLAN Adapter",
				0xef544d24, 0xcd8ea916),
	PCMCIA_DEVICE_PROD_ID12("ELSA", "AirLancer MC-11", 0x4507a33a, 0xef54f0e3),
	PCMCIA_DEVICE_PROD_ID12("HyperLink", "Wireless PC Card 11Mbps", 0x56cc3f1a, 0x0bcf220c),
	PCMCIA_DEVICE_PROD_ID12("Intel", "PRO/Wireless 2011 LAN PC Card", 0x816cc815, 0x07f58077),
	PCMCIA_DEVICE_PROD_ID12("LeArtery", "SYNCBYAIR 11Mbps Wireless LAN PC Card",
				0x7e3b326a, 0x49893e92),
	PCMCIA_DEVICE_PROD_ID12("Lucent Technologies", "WaveLAN/IEEE", 0x23eb9949, 0xc562e72a),
	PCMCIA_DEVICE_PROD_ID12("MELCO", "WLI-PCM-L11", 0x481e0094, 0x7360e410),
	PCMCIA_DEVICE_PROD_ID12("MELCO", "WLI-PCM-L11G", 0x481e0094, 0xf57ca4b3),
	PCMCIA_DEVICE_PROD_ID12("NCR", "WaveLAN/IEEE", 0x24358cd4, 0xc562e72a),
	PCMCIA_DEVICE_PROD_ID12("Nortel Networks", "emobility 802.11 Wireless LAN PC Card",
				0x2d617ea0, 0x88cd5767),
	PCMCIA_DEVICE_PROD_ID12("OTC", "Wireless AirEZY 2411-PCC WLAN Card",
				0x4ac44287, 0x235a6bed),
	PCMCIA_DEVICE_PROD_ID12("PROXIM", "LAN PC CARD HARMONY 80211B", 0xc6536a5e, 0x090c3cd9),
	PCMCIA_DEVICE_PROD_ID12("PROXIM", "LAN PCI CARD HARMONY 80211B", 0xc6536a5e, 0x9f494e26),
	PCMCIA_DEVICE_PROD_ID12("SAMSUNG", "11Mbps WLAN Card", 0x43d74cb4, 0x579bd91b),
	PCMCIA_DEVICE_PROD_ID12("Symbol Technologies", "LA4111 Spectrum24 Wireless LAN PC Card",
				0x3f02b4d6, 0x3663cb0e),
	/* Lucent Orinoco */
	PCMCIA_DEVICE_MANF_CARD_PROD_ID3(0x0156, 0x0002, "Version 01.01", 0xd27deb1a),
#ifdef CONFIG_HERMES_PRISM
	/* Only entries that certainly identify Prism chipset */
	PCMCIA_DEVICE_MANF_CARD(0x000b, 0x7100), /* SonicWALL Long Range Wireless Card */
	PCMCIA_DEVICE_MANF_CARD(0x000b, 0x7300), /* Sohoware NCP110, Philips 802.11b */
	PCMCIA_DEVICE_MANF_CARD(0x0089, 0x0002), /* AnyPoint(TM) Wireless II PC Card */
	PCMCIA_DEVICE_MANF_CARD(0x0126, 0x8000), /* PROXIM RangeLAN-DS/LAN PC CARD */
	PCMCIA_DEVICE_MANF_CARD(0x0138, 0x0002), /* Compaq WL100 11 Mbps Wireless Adapter */
	PCMCIA_DEVICE_MANF_CARD(0x01ff, 0x0008), /* Intermec MobileLAN 11Mbps 802.11b WLAN Card */
	PCMCIA_DEVICE_MANF_CARD(0x0250, 0x0002), /* Samsung SWL2000-N 11Mb/s WLAN Card */
	PCMCIA_DEVICE_MANF_CARD(0x0274, 0x1612), /* Linksys WPC11 Version 2.5 */
	PCMCIA_DEVICE_MANF_CARD(0x0274, 0x1613), /* Linksys WPC11 Version 3 */
	PCMCIA_DEVICE_MANF_CARD(0x028a, 0x0002), /* Compaq HNW-100 11 Mbps Wireless Adapter */
	PCMCIA_DEVICE_MANF_CARD(0x028a, 0x0673), /* Linksys WCF12 Wireless CompactFlash Card */
	PCMCIA_DEVICE_MANF_CARD(0x50c2, 0x7300), /* Airvast WN-100 */
	PCMCIA_DEVICE_MANF_CARD(0x9005, 0x0021), /* Adaptec Ultra Wireless ANW-8030 */
	PCMCIA_DEVICE_MANF_CARD(0xc001, 0x0008), /* CONTEC FLEXSCAN/FX-DDS110-PCC */
	PCMCIA_DEVICE_MANF_CARD(0xc250, 0x0002), /* Conceptronic CON11Cpro, EMTAC A2424i */
	PCMCIA_DEVICE_MANF_CARD(0xd601, 0x0002), /* Safeway 802.11b, ZCOMAX AirRunner/XI-300 */
	PCMCIA_DEVICE_MANF_CARD(0xd601, 0x0005), /* D-Link DCF660, Sandisk Connect SDWCFB-000 */
	PCMCIA_DEVICE_PROD_ID123("Instant Wireless ", " Network PC CARD", "Version 01.02",
				 0x11d901af, 0x6e9bd926, 0x4b74baa0),
	PCMCIA_DEVICE_PROD_ID12("ACTIONTEC", "PRISM Wireless LAN PC Card", 0x393089da, 0xa71e69d5),
	PCMCIA_DEVICE_PROD_ID12("Addtron", "AWP-100 Wireless PCMCIA", 0xe6ec52ce, 0x08649af2),
	PCMCIA_DEVICE_PROD_ID12("BUFFALO", "WLI-CF-S11G", 0x2decece3, 0x82067c18),
	PCMCIA_DEVICE_PROD_ID12("BUFFALO", "WLI-PCM-L11G", 0x2decece3, 0xf57ca4b3),
	PCMCIA_DEVICE_PROD_ID12("Compaq", "WL200_11Mbps_Wireless_PCI_Card", 0x54f7c49c, 0x15a75e5b),
	PCMCIA_DEVICE_PROD_ID12("corega K.K.", "Wireless LAN PCC-11", 0x5261440f, 0xa6405584),
	PCMCIA_DEVICE_PROD_ID12("corega K.K.", "Wireless LAN PCCA-11", 0x5261440f, 0xdf6115f9),
	PCMCIA_DEVICE_PROD_ID12("corega_K.K.", "Wireless_LAN_PCCB-11", 0x29e33311, 0xee7a27ae),
	PCMCIA_DEVICE_PROD_ID12("Digital Data Communications", "WPC-0100", 0xfdd73470, 0xe0b6f146),
	PCMCIA_DEVICE_PROD_ID12("D", "Link DRC-650 11Mbps WLAN Card", 0x71b18589, 0xf144e3ac),
	PCMCIA_DEVICE_PROD_ID12("D", "Link DWL-650 11Mbps WLAN Card", 0x71b18589, 0xb6f1b0ab),
	PCMCIA_DEVICE_PROD_ID12(" ", "IEEE 802.11 Wireless LAN/PC Card", 0x3b6e20c8, 0xefccafe9),
	PCMCIA_DEVICE_PROD_ID12("INTERSIL", "HFA384x/IEEE", 0x74c5e40d, 0xdb472a18),
	PCMCIA_DEVICE_PROD_ID12("INTERSIL", "I-GATE 11M PC Card / PC Card plus",
				0x74c5e40d, 0x8304ff77),
	PCMCIA_DEVICE_PROD_ID12("Intersil", "PRISM 2_5 PCMCIA ADAPTER", 0x4b801a17, 0x6345a0bf),
	PCMCIA_DEVICE_PROD_ID12("Linksys", "Wireless CompactFlash Card", 0x0733cc81, 0x0c52f395),
	PCMCIA_DEVICE_PROD_ID12("Microsoft", "Wireless Notebook Adapter MN-520",
				0x5961bf85, 0x6eec8c01),
	PCMCIA_DEVICE_PROD_ID12("NETGEAR MA401RA Wireless PC", "Card", 0x0306467f, 0x9762e8f1),
	PCMCIA_DEVICE_PROD_ID12("NETGEAR MA401 Wireless PC", "Card", 0xa37434e9, 0x9762e8f1),
	PCMCIA_DEVICE_PROD_ID12("OEM", "PRISM2 IEEE 802.11 PC-Card", 0xfea54c90, 0x48f2bdd6),
	PCMCIA_DEVICE_PROD_ID12("PLANEX", "GeoWave/GW-CF110", 0x209f40ab, 0xd9715264),
	PCMCIA_DEVICE_PROD_ID12("PLANEX", "GeoWave/GW-NS110", 0x209f40ab, 0x46263178),
	PCMCIA_DEVICE_PROD_ID12("SMC", "SMC2532W-B EliteConnect Wireless Adapter",
				0xc4f8b18b, 0x196bd757),
	PCMCIA_DEVICE_PROD_ID12("SMC", "SMC2632W", 0xc4f8b18b, 0x474a1f2a),
	PCMCIA_DEVICE_PROD_ID12("ZoomAir 11Mbps High", "Rate wireless Networking",
				0x273fe3db, 0x32a1eaee),
	PCMCIA_DEVICE_PROD_ID3("HFA3863", 0x355cb092),
	PCMCIA_DEVICE_PROD_ID3("ISL37100P", 0x630d52b2),
	PCMCIA_DEVICE_PROD_ID3("ISL37101P-10", 0xdd97a26b),
	PCMCIA_DEVICE_PROD_ID3("ISL37300P", 0xc9049a39),

	/* This may be Agere or Intersil Firmware */
	PCMCIA_DEVICE_MANF_CARD(0x0156, 0x0002),
#endif
	PCMCIA_DEVICE_NULL,
};
MODULE_DEVICE_TABLE(pcmcia, orinoco_cs_ids);

static struct pcmcia_driver orinoco_cs_driver = {
	.owner		= THIS_MODULE,
	.name		= DRV_NAME,
	.id_table	= orinoco_cs_ids,
	.probe		= orinoco_cs_probe,
	.remove		= orinoco_cs_remove,
	.suspend	= orinoco_cs_suspend,
	.resume		= orinoco_cs_resume,
};
module_pcmcia_driver(orinoco_cs_driver);

MODULE_AUTHOR("David Gibson <hermes@gibson.dropbear.id.au>");
MODULE_DESCRIPTION("Driver for PCMCIA Lucent Orinoco, Prism II based and similar wireless cards");
MODULE_LICENSE("Dual MPL/GPL");
