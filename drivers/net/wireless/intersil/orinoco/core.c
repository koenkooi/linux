// SPDX-License-Identifier: GPL-2.0-only
/* core.c - driver core for Hermes/Prism 2 ("Orinoco") 802.11b devices.
 *
 * The bus-independent core: struct orinoco_private and wiphy lifecycle,
 * card initialisation, the net_device, the commit and reset paths, the
 * interrupt handler, the PIO transmit and receive paths, and the work
 * that turns link status changes into cfg80211 connect, roam and
 * disconnect events. The cfg80211 ops live in cfg.c.
 *
 * Based on the orinoco driver's main.c, whose copyright notice follows:
 *
 * A driver for Hermes or Prism 2 chipset based PCMCIA wireless
 * adaptors, with Lucent/Agere, Intersil or Symbol firmware.
 *
 * (C) Copyright David Gibson, IBM Corporation 2001-2003.
 * Copyright (C) 2000 David Gibson, Linuxcare Australia.
 *	With some help from :
 * Copyright (C) 2001 Jean Tourrilhes, HP Labs
 * Copyright (C) 2001 Benjamin Herrenschmidt
 *
 * Based on dummy_cs.c 1.27 2000/06/12 21:27:25
 *
 * Portions based on wvlan_cs.c 1.0.6, Copyright Andreas Neuhaus <andy
 * AT fasta.fh-dortmund.de>
 *
 * The contents of this file are subject to the Mozilla Public License
 * Version 1.1 (the "License"); you may not use this file except in
 * compliance with the License. You may obtain a copy of the License
 * at http://www.mozilla.org/MPL/
 *
 * Software distributed under the License is distributed on an "AS IS"
 * basis, WITHOUT WARRANTY OF ANY KIND, either express or implied. See
 * the License for the specific language governing rights and
 * limitations under the License.
 *
 * The initial developer of the original code is David A. Hinds
 * <dahinds AT users.sourceforge.net>.  Portions created by David
 * A. Hinds are Copyright (C) 1999 David A. Hinds.  All Rights
 * Reserved.
 *
 * Alternatively, the contents of this file may be used under the
 * terms of the GNU General Public License version 2 (the "GPL"), in
 * which case the provisions of the GPL are applicable instead of the
 * above.  If you wish to allow the use of your version of this file
 * only under the terms of the GPL and not to allow others to use your
 * version of this file under the MPL, indicate your decision by
 * deleting the provisions above and replace them with the notice and
 * other provisions required by the GPL.  If you do not delete the
 * provisions above, a recipient may use your version of this file
 * under either the MPL or the GPL.
 */

#define DRIVER_NAME "orinoco"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/ieee80211.h>
#include <linux/unaligned.h>
#include <net/cfg80211.h>

#include "hermes_rid.h"
#include "hw.h"
#include "scan.h"
#include "cfg.h"
#include "ccmp.h"
#include "main.h"
#include "fw_prism.h"

#include "orinoco.h"

/********************************************************************/
/* Module information                                               */
/********************************************************************/

MODULE_AUTHOR("Pavel Roskin <proski@gnu.org> & David Gibson <hermes@gibson.dropbear.id.au>");
MODULE_DESCRIPTION("Driver for Lucent Orinoco, Prism II based and similar wireless cards");
MODULE_LICENSE("Dual MPL/GPL");

static bool suppress_linkstatus; /* = 0 */
module_param(suppress_linkstatus, bool, 0644);
MODULE_PARM_DESC(suppress_linkstatus, "Don't log link status changes");

static int ignore_disconnect; /* = 0 */
module_param(ignore_disconnect, int, 0644);
MODULE_PARM_DESC(ignore_disconnect,
		 "Don't report lost link to the network layer");

/********************************************************************/
/* Internal constants                                               */
/********************************************************************/

/* 802.2 LLC/SNAP header used for Ethernet encapsulation over 802.11 */
static const u8 encaps_hdr[] = {0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00};
#define ENCAPS_OVERHEAD		(sizeof(encaps_hdr) + 2)

#define ORINOCO_MIN_MTU		256
#define ORINOCO_MAX_MTU		(IEEE80211_MAX_DATA_LEN - ENCAPS_OVERHEAD)

#define MAX_IRQLOOPS_PER_IRQ	10
/* Based on a guestimate of how many events the device could legitimately
 * generate
 */
#define MAX_IRQLOOPS_PER_JIFFY	(20000 / HZ)

#define DUMMY_FID		0xFFFF

#define ORINOCO_INTEN		(HERMES_EV_RX | HERMES_EV_ALLOC \
				 | HERMES_EV_TX | HERMES_EV_TXEXC \
				 | HERMES_EV_WTERR | HERMES_EV_INFO \
				 | HERMES_EV_INFDROP)

/* Upper bound on a single scan-results info frame */
#define ORINOCO_MAX_SCAN_LEN	4096

/********************************************************************/
/* Data types                                                       */
/********************************************************************/

/* Beginning of the Tx descriptor, used in TxExc handling */
struct hermes_txexc_data {
	struct hermes_tx_descriptor desc;
	__le16 frame_ctl;
	__le16 duration_id;
	u8 addr1[ETH_ALEN];
} __packed;

/* Rx frame header except compatibility 802.3 header */
struct hermes_rx_descriptor {
	/* Control */
	__le16 status;
	__le32 time;
	u8 silence;
	u8 signal;
	u8 rate;
	u8 rxflow;
	__le32 reserved;

	/* 802.11 header */
	__le16 frame_ctl;
	__le16 duration_id;
	u8 addr1[ETH_ALEN];
	u8 addr2[ETH_ALEN];
	u8 addr3[ETH_ALEN];
	__le16 seq_ctl;
	u8 addr4[ETH_ALEN];

	/* Data length */
	__le16 data_len;
} __packed;

struct orinoco_rx_data {
	struct hermes_rx_descriptor *desc;
	struct sk_buff *skb;
	struct list_head list;
};

struct orinoco_scan_data {
	void *buf;
	size_t len;
	int type;
	struct list_head list;
};

/********************************************************************/
/* Function prototypes                                              */
/********************************************************************/

static int __orinoco_set_multicast_list(struct net_device *dev);
static int __orinoco_up(struct orinoco_private *priv);
static int __orinoco_down(struct orinoco_private *priv);
static int __orinoco_commit(struct orinoco_private *priv);

/********************************************************************/
/* Internal helper functions                                        */
/********************************************************************/

void set_port_type(struct orinoco_private *priv)
{
	switch (priv->iw_mode) {
	case NL80211_IFTYPE_STATION:
		priv->port_type = 1;
		priv->createibss = 0;
		break;
	case NL80211_IFTYPE_ADHOC:
		if (priv->prefer_port3) {
			priv->port_type = 3;
			priv->createibss = 0;
		} else {
			priv->port_type = priv->ibss_port;
			priv->createibss = 1;
		}
		break;
	default:
		dev_err(priv->dev, "Invalid priv->iw_mode in %s()\n", __func__);
	}
}

/********************************************************************/
/* Device methods                                                   */
/********************************************************************/

static int orinoco_open(struct net_device *dev)
{
	struct orinoco_private *priv = ndev_priv(dev);
	unsigned long flags;
	int err;

	if (orinoco_lock(priv, &flags) != 0)
		return -EBUSY;

	err = __orinoco_up(priv);

	if (!err)
		priv->open = 1;

	orinoco_unlock(priv, &flags);

	return err;
}

static int orinoco_stop(struct net_device *dev)
{
	struct orinoco_private *priv = ndev_priv(dev);
	int err = 0;

	/* We mustn't use orinoco_lock() here, because we need to be
	 * able to close the interface even if hw_unavailable is set
	 * (e.g. as we're released after a PC Card removal)
	 */
	orinoco_lock_irq(priv);

	priv->open = 0;

	err = __orinoco_down(priv);

	orinoco_unlock_irq(priv);

	return err;
}

static void orinoco_set_multicast_list(struct net_device *dev)
{
	struct orinoco_private *priv = ndev_priv(dev);
	unsigned long flags;

	if (orinoco_lock(priv, &flags) != 0) {
		netdev_dbg(dev, "%s() called when hw_unavailable\n", __func__);
		return;
	}

	__orinoco_set_multicast_list(dev);
	orinoco_unlock(priv, &flags);
}

static int orinoco_change_mtu(struct net_device *dev, int new_mtu)
{
	struct orinoco_private *priv = ndev_priv(dev);

	/* MTU + encapsulation + header length */
	if ((new_mtu + ENCAPS_OVERHEAD + sizeof(struct ieee80211_hdr)) >
	     (priv->nicbuf_size - ETH_HLEN))
		return -EINVAL;

	WRITE_ONCE(dev->mtu, new_mtu);

	return 0;
}

/********************************************************************/
/* Tx path                                                          */
/********************************************************************/

/* Add encapsulation and adjust the TX buffer for a data frame.
 *
 * On entry, the frame is an 802.3 (Ethernet) frame. Ethernet-II frames
 * (h_proto > 1500) are re-encapsulated as 802.3 + 802.2 LLC/SNAP for the
 * firmware; genuine 802.3 frames are passed through untouched:
 *
 * 803.3 header (14 bytes)   803.3 header (14 bytes)
 *  dst[6]                    dst[6]
 *  src[6]                    src[6]
 *  type[2] -> len[2]         len[2] <- unchanged
 * --------                  802.2 header (8 bytes)
 * Payload                    encaps[6]
 * ...                        type[2]
 *                           --------
 *                           Payload
 *
 * Returns 0 on success, -ENOMEM on error.
 */
static int orinoco_process_xmit_skb(struct sk_buff *skb,
				    struct net_device *dev)
{
	struct ethhdr *eh = (struct ethhdr *)skb->data;

	/* Encapsulate Ethernet-II frames */
	if (ntohs(eh->h_proto) > ETH_DATA_LEN) { /* Ethernet-II frame */
		struct header_struct {
			struct ethhdr eth;	/* 802.3 header */
			u8 encap[6];		/* 802.2 header */
		} __packed hdr;
		int len = skb->len + sizeof(encaps_hdr) - (2 * ETH_ALEN);

		if (skb_headroom(skb) < ENCAPS_OVERHEAD) {
			if (net_ratelimit())
				netdev_err(dev,
					   "Not enough headroom for 802.2 headers %d\n",
					   skb_headroom(skb));
			return -ENOMEM;
		}

		/* Fill in new header */
		memcpy(&hdr.eth, eh, 2 * ETH_ALEN);
		hdr.eth.h_proto = htons(len);
		memcpy(hdr.encap, encaps_hdr, sizeof(encaps_hdr));

		/* Make room for the new header, and copy it in */
		eh = skb_push(skb, ENCAPS_OVERHEAD);
		memcpy(eh, &hdr, sizeof(hdr));
	}

	return 0;
}

/* Take every WPA2 key out of the table. The caller holds priv->lock and
 * frees the returned keys after dropping it: freeing a transform can
 * sleep.
 */
void orinoco_wpa_detach_keys(struct orinoco_private *priv,
			     struct orinoco_ccmp_key *old[ORINOCO_WPA_NKEYS])
{
	int i;

	old[0] = priv->ptk;
	priv->ptk = NULL;
	for (i = 0; i < ARRAY_SIZE(priv->gtk); i++) {
		old[i + 1] = priv->gtk[i];
		priv->gtk[i] = NULL;
	}
}

void orinoco_wpa_free_keys(struct orinoco_ccmp_key *old[ORINOCO_WPA_NKEYS])
{
	int i;

	for (i = 0; i < ORINOCO_WPA_NKEYS; i++)
		orinoco_ccmp_key_free(old[i]);
}

/* Shared by .disconnect and the reset path so that both leave the same
 * idle state behind: the next commit programs the card for no network
 * (an empty DESIREDSSID, no pinned BSSID, no privacy, no RSN element),
 * and only a fresh .connect brings the configuration back. The caller
 * holds priv->lock and frees @old after dropping it.
 */
void __orinoco_forget_connection(struct orinoco_private *priv,
				 struct orinoco_ccmp_key *old[ORINOCO_WPA_NKEYS])
{
	memset(priv->desired_essid, 0, sizeof(priv->desired_essid));
	priv->bssid_fixed = 0;
	eth_zero_addr(priv->desired_bssid);
	priv->encode_alg = ORINOCO_ALG_NONE;
	priv->wpa_mode = 0;
	priv->wpa_ie_len = 0;
	orinoco_wpa_detach_keys(priv, old);
}

static void orinoco_wpa_wipe_keys(struct orinoco_private *priv)
{
	struct orinoco_ccmp_key *old[ORINOCO_WPA_NKEYS];

	orinoco_lock_irq(priv);
	orinoco_wpa_detach_keys(priv, old);
	orinoco_unlock_irq(priv);
	orinoco_wpa_free_keys(old);
}

/* True for an LLC/SNAP frame whose ethertype is EAPOL, @p pointing at the
 * LLC header. EAPOL is the one kind of data frame that is neither
 * encrypted nor required to be before the pairwise key exists.
 */
static bool orinoco_is_eapol(const u8 *p, int len)
{
	return len >= ENCAPS_OVERHEAD && !memcmp(p, encaps_hdr, sizeof(encaps_hdr)) &&
	       p[6] == 0x88 && p[7] == 0x8e;
}

/* Transmit path of a WPA2 connection. Prism firmware does not encrypt when
 * told the host will (CNFWEPFLAGS HOSTENCRYPT) and, with
 * HERMES_TXCTRL_802_11, sends the 802.11 header and body it is given as
 * they are, adding only addr2's transmitter slot handling, the sequence
 * number and the PHY framing. So the whole data frame is built here: the
 * ToDS header, then for everything but EAPOL the CCMP header, ciphertext
 * and MIC. skb holds an 802.3 header followed by the LLC/SNAP body, as
 * orinoco_process_xmit_skb() leaves it. Returns -ENOKEY for a frame that
 * must be protected before a pairwise key exists (dropped by the caller)
 * and -ENOLINK when the BSSID is not known yet.
 */
static int orinoco_wpa_write_frame(struct orinoco_private *priv,
				   struct sk_buff *skb, u16 txfid,
				   int tx_control)
{
	const struct ethhdr *eh = (const struct ethhdr *)skb->data;
	const u8 *body = skb->data + ETH_HLEN;
	size_t body_len = skb->len - ETH_HLEN;
	bool protect = !orinoco_is_eapol(body, body_len);
	size_t wire_len = body_len;
	struct orinoco_ccmp_hdr h;
	__le16 fc;
	u8 *buf;
	int err;

	if (!priv->associated)
		return -ENOLINK;
	if (protect && !priv->ptk)
		return -ENOKEY;
	if (protect)
		wire_len += ORINOCO_CCMP_HDR_LEN + ORINOCO_CCMP_MIC_LEN;

	fc = cpu_to_le16(IEEE80211_FTYPE_DATA | IEEE80211_STYPE_DATA |
			 IEEE80211_FCTL_TODS |
			 (protect ? IEEE80211_FCTL_PROTECTED : 0));

	if (HERMES_802_2_OFFSET + wire_len > priv->nicbuf_size)
		return -EMSGSIZE;

	buf = kzalloc(HERMES_802_2_OFFSET + wire_len, GFP_ATOMIC);
	if (!buf)
		return -ENOMEM;

	/* Descriptor tx_control (offset 12), then the 802.11 header region
	 * (frame control, duration, addr1-3, sequence control, addr4,
	 * data length), then the pseudo 802.3 header the firmware wants
	 * (addresses unused, length big endian), then the body.
	 */
	put_unaligned_le16(tx_control | HERMES_TXCTRL_802_11, buf + 12);
	put_unaligned(fc, (__le16 *)(buf + HERMES_802_11_OFFSET));
	memcpy(buf + HERMES_802_11_OFFSET + 4, priv->assoc_bssid, ETH_ALEN);
	memcpy(buf + HERMES_802_11_OFFSET + 10, priv->ndev->dev_addr, ETH_ALEN);
	memcpy(buf + HERMES_802_11_OFFSET + 16, eh->h_dest, ETH_ALEN);
	put_unaligned_le16(wire_len, buf + HERMES_802_11_OFFSET + 30);
	put_unaligned_be16(wire_len, buf + HERMES_802_3_OFFSET + 12);

	if (protect) {
		h.frame_control = fc;
		h.seq_ctrl = 0;
		h.addr1 = priv->assoc_bssid;
		h.addr2 = priv->ndev->dev_addr;
		h.addr3 = eh->h_dest;
		err = orinoco_ccmp_encrypt(priv->ptk, 0, &h, body, body_len,
					   buf + HERMES_802_2_OFFSET);
		if (err) {
			kfree(buf);
			return err;
		}
	} else {
		memcpy(buf + HERMES_802_2_OFFSET, body, body_len);
	}

	err = priv->hw.ops->bap_pwrite(&priv->hw, USER_BAP, buf,
				       HERMES_802_2_OFFSET + wire_len, txfid, 0);
	kfree(buf);

	return err;
}

static netdev_tx_t orinoco_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct orinoco_private *priv = ndev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct hermes *hw = &priv->hw;
	int err = 0;
	u16 txfid = priv->txfid;
	int tx_control;
	unsigned long flags;

	if (!netif_running(dev)) {
		netdev_err(dev, "Tx on stopped device!\n");
		return NETDEV_TX_BUSY;
	}

	if (netif_queue_stopped(dev)) {
		netdev_dbg(dev, "Tx while transmitter busy!\n");
		return NETDEV_TX_BUSY;
	}

	if (orinoco_lock(priv, &flags) != 0) {
		netdev_err(dev, "%s() called while hw_unavailable\n", __func__);
		return NETDEV_TX_BUSY;
	}

	if (!netif_carrier_ok(dev)) {
		/* Oops, the firmware hasn't established a connection,
		 * silently drop the packet (this seems to be the
		 * safest approach).
		 */
		goto drop;
	}

	/* Check packet length */
	if (skb->len < ETH_HLEN)
		goto drop;

	tx_control = HERMES_TXCTRL_TX_OK | HERMES_TXCTRL_TX_EX;

	err = orinoco_process_xmit_skb(skb, dev);
	if (err)
		goto drop;

	if (priv->wpa_mode) {
		err = orinoco_wpa_write_frame(priv, skb, txfid, tx_control);
		if (err == -ENOKEY || err == -ENOLINK || err == -EOVERFLOW ||
		    err == -EMSGSIZE)
			goto drop;
		if (err) {
			if (net_ratelimit())
				netdev_err(dev, "Error %d writing WPA frame to BAP\n",
					   err);
			goto busy;
		}
		goto send;
	}

	if (priv->has_alt_txcntl) {
		/* WPA enabled firmwares have tx_cntl at the end of
		 * the 802.11 header.  So write zeroed descriptor and
		 * 802.11 header at the same time
		 */
		char desc[HERMES_802_3_OFFSET];
		__le16 *txcntl = (__le16 *)&desc[HERMES_TXCNTL2_OFFSET];

		memset(&desc, 0, sizeof(desc));

		*txcntl = cpu_to_le16(tx_control);
		err = hw->ops->bap_pwrite(hw, USER_BAP, &desc, sizeof(desc),
					  txfid, 0);
		if (err) {
			if (net_ratelimit())
				netdev_err(dev,
					   "Error %d writing Tx descriptor to BAP\n",
					   err);
			goto busy;
		}
	} else {
		struct hermes_tx_descriptor desc;

		memset(&desc, 0, sizeof(desc));

		desc.tx_control = cpu_to_le16(tx_control);
		err = hw->ops->bap_pwrite(hw, USER_BAP, &desc, sizeof(desc),
					  txfid, 0);
		if (err) {
			if (net_ratelimit())
				netdev_err(dev,
					   "Error %d writing Tx descriptor to BAP\n",
					   err);
			goto busy;
		}

		/* Clear the 802.11 header and data length fields - some
		 * firmwares (e.g. Lucent/Agere 8.xx) appear to get confused
		 * if this isn't done.
		 */
		hermes_clear_words(hw, HERMES_DATA0,
				   HERMES_802_3_OFFSET - HERMES_802_11_OFFSET);
	}

	err = hw->ops->bap_pwrite(hw, USER_BAP, skb->data, skb->len,
				  txfid, HERMES_802_3_OFFSET);
	if (err) {
		netdev_err(dev, "Error %d writing packet to BAP\n", err);
		goto busy;
	}

send:
	/* Finally, we actually initiate the send */
	netif_stop_queue(dev);

	err = hw->ops->cmd_wait(hw, HERMES_CMD_TX | HERMES_CMD_RECL,
				txfid, NULL);
	if (err) {
		netif_start_queue(dev);
		if (net_ratelimit())
			netdev_err(dev, "Error %d transmitting packet\n", err);
		goto busy;
	}

	stats->tx_bytes += HERMES_802_3_OFFSET + skb->len;
	goto ok;

 drop:
	stats->tx_errors++;
	stats->tx_dropped++;

 ok:
	orinoco_unlock(priv, &flags);
	dev_kfree_skb(skb);
	return NETDEV_TX_OK;

 busy:
	if (err == -EIO)
		schedule_work(&priv->reset_work);
	orinoco_unlock(priv, &flags);
	return NETDEV_TX_BUSY;
}

static void orinoco_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
	struct orinoco_private *priv = ndev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct hermes *hw = &priv->hw;

	netdev_warn(dev, "Tx timeout! ALLOCFID=%04x, TXCOMPLFID=%04x, EVSTAT=%04x\n",
		    hermes_read_regn(hw, ALLOCFID),
		    hermes_read_regn(hw, TXCOMPLFID),
		    hermes_read_regn(hw, EVSTAT));

	stats->tx_errors++;

	schedule_work(&priv->reset_work);
}

/********************************************************************/
/* Interrupt handler TX/ALLOC event bodies                          */
/********************************************************************/

static void __orinoco_ev_alloc(struct net_device *dev, struct hermes *hw)
{
	struct orinoco_private *priv = ndev_priv(dev);
	u16 fid = hermes_read_regn(hw, ALLOCFID);

	if (fid != priv->txfid) {
		if (fid != DUMMY_FID)
			netdev_warn(dev, "Allocate event on unexpected fid (%04X)\n",
				    fid);
		return;
	}

	hermes_write_regn(hw, ALLOCFID, DUMMY_FID);
}

static void __orinoco_ev_tx(struct net_device *dev, struct hermes *hw)
{
	dev->stats.tx_packets++;

	netif_wake_queue(dev);

	hermes_write_regn(hw, TXCOMPLFID, DUMMY_FID);
}

static void __orinoco_ev_txexc(struct net_device *dev, struct hermes *hw)
{
	struct net_device_stats *stats = &dev->stats;
	u16 fid = hermes_read_regn(hw, TXCOMPLFID);
	u16 status;
	struct hermes_txexc_data hdr;
	int err = 0;

	if (fid == DUMMY_FID)
		return; /* Nothing's really happened */

	/* Read part of the frame header - we need status and addr1 */
	err = hw->ops->bap_pread(hw, IRQ_BAP, &hdr,
				 sizeof(struct hermes_txexc_data),
				 fid, 0);

	hermes_write_regn(hw, TXCOMPLFID, DUMMY_FID);
	stats->tx_errors++;

	if (err) {
		netdev_warn(dev, "Unable to read descriptor on Tx error (FID=%04X error %d)\n",
			    fid, err);
		return;
	}

	/* Connection loss is reported by the firmware through
	 * HERMES_INQ_LINKSTATUS; a failed frame is only logged.
	 */
	status = le16_to_cpu(hdr.desc.status);
	if (status & (HERMES_TXSTAT_RETRYERR | HERMES_TXSTAT_AGEDERR))
		netdev_dbg(dev, "Tx retry/lifetime exceeded towards %pM (FID=%04X status %04X)\n",
			   hdr.addr1, fid, status);
	else
		netdev_dbg(dev, "Tx error (FID=%04X status %04X)\n",
			   fid, status);

	netif_wake_queue(dev);
}

/********************************************************************/
/* Rx path (data frames)                                            */
/********************************************************************/

/* Does the frame have a SNAP header indicating it should be
 * de-encapsulated to Ethernet-II?
 */
static inline int is_ethersnap(void *_hdr)
{
	u8 *hdr = _hdr;

	/* We de-encapsulate all packets which, a) have SNAP headers
	 * (i.e. SSAP=DSAP=0xaa and CTRL=0x3 in the 802.2 LLC header
	 * and where b) the OUI of the SNAP header is 00:00:00 or
	 * 00:00:f8 - we need both because different APs appear to use
	 * different OUIs for some reason
	 */
	return (memcmp(hdr, &encaps_hdr, 5) == 0) &&
		((hdr[5] == 0x00) || (hdr[5] == 0xf8));
}

/* Hard-IRQ half of the RX path: read the descriptor and frame body off
 * the card through the BAP, then queue for the tasklet.
 */
static void __orinoco_ev_rx(struct net_device *dev, struct hermes *hw)
{
	struct orinoco_private *priv = ndev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct sk_buff *skb = NULL;
	u16 rxfid, status;
	int length;
	struct hermes_rx_descriptor *desc;
	struct orinoco_rx_data *rx_data;
	int err;

	desc = kmalloc_obj(*desc, GFP_ATOMIC);
	if (!desc)
		goto update_stats;

	rxfid = hermes_read_regn(hw, RXFID);

	err = hw->ops->bap_pread(hw, IRQ_BAP, desc, sizeof(*desc),
				 rxfid, 0);
	if (err) {
		netdev_err(dev, "error %d reading Rx descriptor. Frame dropped.\n",
			   err);
		goto update_stats;
	}

	status = le16_to_cpu(desc->status);

	if (status & HERMES_RXSTAT_BADCRC) {
		netdev_dbg(dev, "Bad CRC on Rx. Frame dropped.\n");
		stats->rx_crc_errors++;
		goto update_stats;
	}

	/* With host decryption the firmware cannot decrypt anything and
	 * flags every protected frame; the host's own check decides.
	 */
	if ((status & HERMES_RXSTAT_UNDECRYPTABLE) && !priv->wpa_mode) {
		netdev_dbg(dev, "Undecryptable frame on Rx. Frame dropped.\n");
		goto update_stats;
	}

	/* TKIP is not supported, so a Michael MIC cannot be verified */
	if ((status & HERMES_RXSTAT_MIC) && !priv->wpa_mode) {
		if (net_ratelimit())
			netdev_warn(dev, "MIC-carrying frame on Rx without TKIP support. Frame dropped.\n");
		goto update_stats;
	}

	/* Host decryption makes the firmware hand up every frame it hears,
	 * beacons and probe responses included, a few dozen a second; only
	 * data frames are of use and the body read below is the expensive
	 * part over this bus.
	 */
	if (priv->wpa_mode &&
	    (le16_to_cpu(desc->frame_ctl) & IEEE80211_FCTL_FTYPE) !=
	    IEEE80211_FTYPE_DATA)
		goto out;

	length = le16_to_cpu(desc->data_len);

	/* Sanity checks */
	if (length < 3) { /* No for even an 802.2 LLC header */
		/* At least on Symbol firmware with PCF we get quite a
		 * lot of these legitimately - Poll frames with no
		 * data.
		 */
		goto out;
	}
	if (length > IEEE80211_MAX_DATA_LEN + ORINOCO_CCMP_HDR_LEN +
	    ORINOCO_CCMP_MIC_LEN) {
		netdev_warn(dev, "Oversized frame received (%d bytes)\n",
			    length);
		stats->rx_length_errors++;
		goto update_stats;
	}

	/* We need space for the packet data itself, plus an ethernet
	 * header, plus 2 bytes so we can align the IP header on a
	 * 32bit boundary, plus 1 byte so we can read in odd length
	 * packets from the card, which has an IO granularity of 16
	 * bits
	 */
	skb = dev_alloc_skb(length + ETH_HLEN + 2 + 1);
	if (!skb)
		goto update_stats;

	/* We'll prepend the header, so reserve space for it.  The worst
	 * case is no decapsulation, when 802.3 header is prepended and
	 * nothing is removed.  2 is for aligning the IP header.
	 */
	skb_reserve(skb, ETH_HLEN + 2);

	err = hw->ops->bap_pread(hw, IRQ_BAP, skb_put(skb, length),
				 ALIGN(length, 2), rxfid,
				 HERMES_802_2_OFFSET);
	if (err) {
		netdev_err(dev, "error %d reading frame. Frame dropped.\n",
			   err);
		goto drop;
	}

	/* Add desc and skb to rx queue */
	rx_data = kzalloc(sizeof(*rx_data), GFP_ATOMIC);
	if (!rx_data)
		goto drop;

	rx_data->desc = desc;
	rx_data->skb = skb;
	list_add_tail(&rx_data->list, &priv->rx_list);
	tasklet_schedule(&priv->rx_tasklet);

	return;

drop:
	dev_kfree_skb_any(skb);
update_stats:
	stats->rx_errors++;
	stats->rx_dropped++;
out:
	kfree(desc);
}

/* Receive side of a WPA2 connection: with HOSTDECRYPT the firmware hands up
 * the 802.11 body untouched, so a protected data frame is decrypted and
 * verified here, in place, before the usual de-encapsulation. Management
 * frames (the firmware also delivers the association response) and QoS
 * frames are dropped. Only EAPOL may arrive in the clear. Returns 0 with
 * desc->data_len and skb trimmed to the plain LLC/SNAP body, or a negative
 * errno for a frame to drop.
 */
static int orinoco_wpa_rx(struct orinoco_private *priv,
			  struct hermes_rx_descriptor *desc,
			  struct sk_buff *skb)
{
	u16 fc = le16_to_cpu(desc->frame_ctl);
	int length = le16_to_cpu(desc->data_len);
	struct orinoco_ccmp_key *key;
	struct orinoco_ccmp_hdr h;
	u8 *plain;
	int err;

	if ((fc & IEEE80211_FCTL_FTYPE) != IEEE80211_FTYPE_DATA ||
	    (fc & IEEE80211_STYPE_QOS_DATA))
		return -EINVAL;

	if (!(fc & IEEE80211_FCTL_PROTECTED))
		return orinoco_is_eapol(skb->data, length) ? 0 : -EPERM;

	/* Only the associated AP's frames are worth a CCM attempt */
	if (!ether_addr_equal(desc->addr2, priv->assoc_bssid))
		return -EINVAL;

	if (length < ORINOCO_CCMP_HDR_LEN + ORINOCO_CCMP_MIC_LEN + 1)
		return -EINVAL;

	if (is_multicast_ether_addr(desc->addr1))
		key = priv->gtk[orinoco_ccmp_hdr_key_id(skb->data)];
	else
		key = priv->ptk;
	if (!key)
		return -ENOKEY;

	h.frame_control = desc->frame_ctl;
	h.seq_ctrl = desc->seq_ctl;
	h.addr1 = desc->addr1;
	h.addr2 = desc->addr2;
	h.addr3 = desc->addr3;

	length -= ORINOCO_CCMP_HDR_LEN + ORINOCO_CCMP_MIC_LEN;
	plain = kmalloc(length + ORINOCO_CCMP_MIC_LEN, GFP_ATOMIC);
	if (!plain)
		return -ENOMEM;

	err = orinoco_ccmp_decrypt(key, &h, skb->data,
				   le16_to_cpu(desc->data_len), plain);
	if (!err) {
		memcpy(skb->data, plain, length);
		skb_trim(skb, length);
		desc->data_len = cpu_to_le16(length);
	}
	kfree(plain);

	return err;
}

/* BH half of the RX path: de-encapsulate and hand the frame to the
 * stack. Split from the hard-IRQ half so the time spent with the shared
 * ready/IRQ GPIO's interrupt blocked stays bounded.
 */
static void orinoco_rx(struct net_device *dev,
		       struct hermes_rx_descriptor *desc,
		       struct sk_buff *skb)
{
	struct orinoco_private *priv = ndev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	u16 status, fc;
	int length;
	int err;
	struct ethhdr *hdr;

	err = priv->wpa_mode ? orinoco_wpa_rx(priv, desc, skb) : 0;
	if (err) {
		if (net_ratelimit())
			netdev_info(dev, "WPA: dropped Rx frame, fc %04x len %d: %d\n",
				    le16_to_cpu(desc->frame_ctl),
				    le16_to_cpu(desc->data_len), err);
		stats->rx_errors++;
		stats->rx_dropped++;
		dev_kfree_skb(skb);
		return;
	}

	status = le16_to_cpu(desc->status);
	length = le16_to_cpu(desc->data_len);
	fc = le16_to_cpu(desc->frame_ctl);

	/* Handle decapsulation
	 * In most cases, the firmware tell us about SNAP frames.
	 * For some reason, the SNAP frames sent by LinkSys APs
	 * are not properly recognised by most firmwares.
	 * So, check ourselves
	 */
	if (length >= ENCAPS_OVERHEAD &&
	    (((status & HERMES_RXSTAT_MSGTYPE) == HERMES_RXSTAT_1042) ||
	     ((status & HERMES_RXSTAT_MSGTYPE) == HERMES_RXSTAT_TUNNEL) ||
	     is_ethersnap(skb->data))) {
		/* These indicate a SNAP within 802.2 LLC within
		 * 802.11 frame which we'll need to de-encapsulate to
		 * the original EthernetII frame.
		 */
		hdr = skb_push(skb, ETH_HLEN - ENCAPS_OVERHEAD);
	} else {
		/* 802.3 frame - prepend 802.3 header as is */
		hdr = skb_push(skb, ETH_HLEN);
		hdr->h_proto = htons(length);
	}
	memcpy(hdr->h_dest, desc->addr1, ETH_ALEN);
	if (fc & IEEE80211_FCTL_FROMDS)
		memcpy(hdr->h_source, desc->addr3, ETH_ALEN);
	else
		memcpy(hdr->h_source, desc->addr2, ETH_ALEN);

	skb->protocol = eth_type_trans(skb, dev);
	skb->ip_summed = CHECKSUM_NONE;
	if (fc & IEEE80211_FCTL_TODS)
		skb->pkt_type = PACKET_OTHERHOST;

	/* Pass the packet to the networking stack */
	netif_rx(skb);
	stats->rx_packets++;
	stats->rx_bytes += length;
}

static void orinoco_rx_isr_tasklet(struct tasklet_struct *t)
{
	struct orinoco_private *priv = from_tasklet(priv, t, rx_tasklet);
	struct orinoco_rx_data *rx_data, *temp;
	struct hermes_rx_descriptor *desc;
	struct net_device *dev;
	struct sk_buff *skb;
	unsigned long flags;

	/* orinoco_rx requires the driver lock, and we also need to
	 * protect priv->rx_list, so just hold the lock over the
	 * lot.
	 *
	 * If orinoco_lock fails, we've unplugged the card. In this
	 * case just abort.
	 */
	if (orinoco_lock(priv, &flags) != 0)
		return;

	dev = priv->ndev;

	/* extract desc and skb from queue */
	list_for_each_entry_safe(rx_data, temp, &priv->rx_list, list) {
		desc = rx_data->desc;
		skb = rx_data->skb;
		list_del(&rx_data->list);
		kfree(rx_data);

		orinoco_rx(dev, desc, skb);

		kfree(desc);
	}

	orinoco_unlock(priv, &flags);
}

/********************************************************************/
/* Info frame handling (scan results, link status)                  */
/********************************************************************/

/* Linkstatus values that mean "the firmware holds an association" */
static bool orinoco_linkstatus_connected(u16 status)
{
	return (status == HERMES_LINKSTATUS_CONNECTED) ||
		(status == HERMES_LINKSTATUS_AP_CHANGE) ||
		(status == HERMES_LINKSTATUS_AP_IN_RANGE);
}

static void print_linkstatus(struct net_device *dev, u16 status)
{
	char *s;

	if (suppress_linkstatus)
		return;

	switch (status) {
	case HERMES_LINKSTATUS_NOT_CONNECTED:
		s = "Not Connected";
		break;
	case HERMES_LINKSTATUS_CONNECTED:
		s = "Connected";
		break;
	case HERMES_LINKSTATUS_DISCONNECTED:
		s = "Disconnected";
		break;
	case HERMES_LINKSTATUS_AP_CHANGE:
		s = "AP Changed";
		break;
	case HERMES_LINKSTATUS_AP_OUT_OF_RANGE:
		s = "AP Out of Range";
		break;
	case HERMES_LINKSTATUS_AP_IN_RANGE:
		s = "AP In Range";
		break;
	case HERMES_LINKSTATUS_ASSOC_FAILED:
		s = "Association Failed";
		break;
	default:
		s = "UNKNOWN";
	}

	netdev_dbg(dev, "New link status: %s (%04x)\n", s, status);
}

/* Scan results are processed from a workqueue: the info-frame interrupt
 * queues the raw buffer here, and orinoco_process_scan_results() feeds it
 * to scan.c's cfg80211_inform_bss*()/cfg80211_scan_done() calls in
 * process context.
 */
static void qbuf_scan(struct orinoco_private *priv, void *buf,
		      int len, int type)
{
	struct orinoco_scan_data *sd;
	unsigned long flags;

	sd = kmalloc_obj(*sd, GFP_ATOMIC);
	if (!sd)
		return;

	sd->buf = buf;
	sd->len = len;
	sd->type = type;

	spin_lock_irqsave(&priv->scan_lock, flags);
	list_add_tail(&sd->list, &priv->scan_list);
	spin_unlock_irqrestore(&priv->scan_lock, flags);

	schedule_work(&priv->process_scan);
}

static void qabort_scan(struct orinoco_private *priv)
{
	struct orinoco_scan_data *sd;
	unsigned long flags;

	sd = kmalloc_obj(*sd, GFP_ATOMIC);
	if (!sd)
		return;

	sd->buf = NULL;
	sd->len = -1; /* Abort */
	sd->type = 0; /* No info frame; len < 0 means type is never consulted */

	spin_lock_irqsave(&priv->scan_lock, flags);
	list_add_tail(&sd->list, &priv->scan_list);
	spin_unlock_irqrestore(&priv->scan_lock, flags);

	schedule_work(&priv->process_scan);
}

static void orinoco_process_scan_results(struct work_struct *work)
{
	struct orinoco_private *priv =
		container_of(work, struct orinoco_private, process_scan);
	struct orinoco_scan_data *sd, *temp;
	unsigned long flags;
	void *buf;
	int len;
	int type;

	spin_lock_irqsave(&priv->scan_lock, flags);
	list_for_each_entry_safe(sd, temp, &priv->scan_list, list) {
		buf = sd->buf;
		len = sd->len;
		type = sd->type;

		list_del(&sd->list);
		spin_unlock_irqrestore(&priv->scan_lock, flags);
		kfree(sd);

		if (len > 0) {
			if (type == HERMES_INQ_CHANNELINFO)
				orinoco_add_extscan_result(priv, buf, len);
			else
				orinoco_add_hostscan_results(priv, buf, len);

			kfree(buf);
		} else {
			/* Either abort or complete the scan */
			orinoco_scan_done(priv, (len < 0));
		}

		spin_lock_irqsave(&priv->scan_lock, flags);
	}
	spin_unlock_irqrestore(&priv->scan_lock, flags);
}

static void __orinoco_ev_info(struct net_device *dev, struct hermes *hw)
{
	struct orinoco_private *priv = ndev_priv(dev);
	u16 infofid;
	struct {
		__le16 len;
		__le16 type;
	} __packed info;
	int len, type;
	int err;

	/* This is an answer to an INQUIRE command that we did earlier,
	 * or an information "event" generated by the card
	 * The controller return to us a pseudo frame containing
	 * the information in question - Jean II
	 */
	infofid = hermes_read_regn(hw, INFOFID);

	/* Read the info frame header - don't try too hard */
	err = hw->ops->bap_pread(hw, IRQ_BAP, &info, sizeof(info),
				 infofid, 0);
	if (err) {
		netdev_err(dev, "error %d reading info frame. Frame dropped.\n",
			   err);
		return;
	}

	len = HERMES_RECLEN_TO_BYTES(le16_to_cpu(info.len));
	type = le16_to_cpu(info.type);

	switch (type) {
	case HERMES_INQ_TALLIES:
		break;
	case HERMES_INQ_LINKSTATUS: {
		struct hermes_linkstatus linkstatus;
		u16 newstatus;
		int connected;

		if (len != sizeof(linkstatus)) {
			netdev_warn(dev, "Unexpected size for linkstatus frame (%d bytes)\n",
				    len);
			break;
		}

		err = hw->ops->bap_pread(hw, IRQ_BAP, &linkstatus, len,
					 infofid, sizeof(info));
		if (err)
			break;
		newstatus = le16_to_cpu(linkstatus.linkstatus);

		/* Symbol firmware uses "out of range" to signal that
		 * the hostscan frame can be requested.
		 */
		if (newstatus == HERMES_LINKSTATUS_AP_OUT_OF_RANGE &&
		    priv->firmware_type == FIRMWARE_TYPE_SYMBOL &&
		    priv->has_hostscan && READ_ONCE(priv->scan_request)) {
			hermes_inquire(hw, HERMES_INQ_HOSTSCAN_SYMBOL);
			break;
		}

		connected = orinoco_linkstatus_connected(newstatus);

		if (connected) {
			/* For a cfg80211-level STA the data path opens
			 * only for an association userspace asked for: an
			 * idle firmware associates to "any" at radio
			 * level (empty DESIREDSSID -- see cfg.c's
			 * .disconnect), and that must stay invisible.
			 */
			if (priv->iw_mode != NL80211_IFTYPE_STATION ||
			    priv->connect_pending || priv->associated)
				netif_carrier_on(dev);
			else
				netdev_dbg(dev, "Firmware associated with no cfg80211 connection; carrier left off\n");
		} else if (!ignore_disconnect) {
			netif_carrier_off(dev);
		}

		if (newstatus != priv->last_linkstatus) {
			priv->last_linkstatus = newstatus;
			print_linkstatus(dev, newstatus);
			/* Reported to cfg80211 from process context */
			if (priv->iw_mode == NL80211_IFTYPE_STATION)
				schedule_work(&priv->link_work);
		}
	}
	break;
	case HERMES_INQ_SCAN:
		/* Intersil fixed-BSSID join: with manual roaming selected
		 * (CNFROAMINGMODE 2, see __orinoco_hw_set_wap()) the
		 * firmware scans but never joins by itself; a scan event
		 * with no cfg80211 scan pending is its cue that results are
		 * ready for orinoco_join_ap() to pick the BSSID from.
		 */
		if (!READ_ONCE(priv->scan_request) && priv->bssid_fixed &&
		    priv->firmware_type == FIRMWARE_TYPE_INTERSIL) {
			schedule_work(&priv->join_work);
			break;
		}
		fallthrough;
	case HERMES_INQ_HOSTSCAN:
	case HERMES_INQ_HOSTSCAN_SYMBOL: {
		/* Result of a scanning. Contains information about
		 * cells in the vicinity - Jean II
		 */
		unsigned char *buf;

		/* Sanity check */
		if (len < 0 || len > ORINOCO_MAX_SCAN_LEN) {
			netdev_warn(dev, "Scan results too large (%d bytes)\n",
				    len);
			qabort_scan(priv);
			break;
		}

		/* Allocate buffer for results */
		buf = kmalloc(len, GFP_ATOMIC);
		if (!buf) {
			/* No memory, so can't printk()... */
			qabort_scan(priv);
			break;
		}

		/* Read scan data */
		err = hw->ops->bap_pread(hw, IRQ_BAP, (void *)buf, len,
					 infofid, sizeof(info));
		if (err) {
			kfree(buf);
			qabort_scan(priv);
			break;
		}

		qbuf_scan(priv, buf, len, type);
	}
	break;
	case HERMES_INQ_CHANNELINFO:
	{
		struct agere_ext_scan_info *bss;

		if (!READ_ONCE(priv->scan_request)) {
			netdev_dbg(dev, "Got chaninfo without scan, len=%d\n",
				   len);
			break;
		}

		/* An empty result indicates that the scan is complete */
		if (len == 0) {
			qbuf_scan(priv, NULL, len, type);
			break;
		}

		/* Sanity check */
		else if (len < (int)(offsetof(struct agere_ext_scan_info,
					      data) + 2)) {
			/* Drop this result now so we don't have to
			 * keep checking later
			 */
			netdev_warn(dev, "Ext scan results too short (%d bytes)\n",
				    len);
			break;
		}

		bss = kmalloc(len, GFP_ATOMIC);
		if (!bss)
			break;

		/* Read scan data */
		err = hw->ops->bap_pread(hw, IRQ_BAP, (void *)bss, len,
					 infofid, sizeof(info));
		if (err)
			kfree(bss);
		else
			qbuf_scan(priv, bss, len, type);

		break;
	}
	case HERMES_INQ_SEC_STAT_AGERE:
		/* Security status (Agere specific) */
		/* Ignore this frame for now */
		if (priv->firmware_type == FIRMWARE_TYPE_AGERE)
			break;
		fallthrough;
	default:
		netdev_dbg(dev, "Unknown information frame received: type 0x%04x, length %d\n",
			   type, len);
		/* We don't actually do anything about it */
		break;
	}
}

static void __orinoco_ev_infdrop(struct net_device *dev, struct hermes *hw)
{
	if (net_ratelimit())
		netdev_dbg(dev, "Information frame lost.\n");
}

/********************************************************************/
/* Connect machinery                                                */
/********************************************************************/

/* Intersil fixed-BSSID join: scheduled from the HERMES_INQ_SCAN event
 * when manual roaming is armed, it looks the requested BSSID up in the
 * firmware's scan-results table to learn its channel and issues the join
 * request.
 */
static void orinoco_join_ap(struct work_struct *work)
{
	struct orinoco_private *priv =
		container_of(work, struct orinoco_private, join_work);
	struct hermes *hw = &priv->hw;
	struct net_device *dev;
	int err;
	unsigned long flags;
	struct join_req {
		u8 bssid[ETH_ALEN];
		__le16 channel;
	} __packed req;
	const int atom_len = offsetof(struct prism2_scan_apinfo, atim);
	struct prism2_scan_apinfo *atom = NULL;
	int offset = 4;
	int found = 0;
	u8 *buf;
	u16 len;

	/* Allocate buffer for scan results */
	buf = kmalloc(ORINOCO_MAX_SCAN_LEN, GFP_KERNEL);
	if (!buf)
		return;

	if (orinoco_lock(priv, &flags) != 0)
		goto fail_lock;

	dev = priv->ndev;

	/* Sanity checks in case user changed something in the meantime */
	if (!priv->bssid_fixed)
		goto out;

	if (strlen(priv->desired_essid) == 0)
		goto out;

	/* Read scan results from the firmware */
	err = hw->ops->read_ltv(hw, USER_BAP,
				HERMES_RID_SCANRESULTSTABLE,
				ORINOCO_MAX_SCAN_LEN, &len, buf);
	if (err) {
		netdev_err(dev, "Cannot read scan results\n");
		goto out;
	}

	/* read_ltv reports the card's record length even when it truncated */
	len = min_t(u16, HERMES_RECLEN_TO_BYTES(len), ORINOCO_MAX_SCAN_LEN);

	/* Go through the scan results looking for the channel of the AP
	 * we were requested to join
	 */
	for (; offset + atom_len <= len; offset += atom_len) {
		atom = (struct prism2_scan_apinfo *)(buf + offset);
		if (memcmp(&atom->bssid, priv->desired_bssid, ETH_ALEN) == 0) {
			found = 1;
			break;
		}
	}

	if (!found) {
		netdev_dbg(dev, "Requested AP not found in scan results\n");
		goto out;
	}

	memcpy(req.bssid, priv->desired_bssid, ETH_ALEN);
	req.channel = atom->channel;	/* both are little-endian */
	err = HERMES_WRITE_RECORD(hw, USER_BAP, HERMES_RID_CNFJOINREQUEST,
				  &req);
	if (err)
		netdev_err(dev, "Error issuing join request\n");

 out:
	orinoco_unlock(priv, &flags);

 fail_lock:
	kfree(buf);
}

/* cfg80211_connect_result() and cfg80211_roamed() WARN and abandon the
 * event when the reported BSS is not in cfg80211's scan cache
 * (net/wireless/sme.c looks it up and trips WARN_ON(bss_not_found)).
 * iwd/wpa_supplicant scan before connecting so the cache is warm, but a
 * bare `iw connect` straight after card insertion is not covered; the
 * firmware associates fine regardless, since it does its own scanning.
 * So before reporting, make sure cfg80211 has seen the BSS, informing a
 * minimal entry from what the firmware can tell us (BSSID, channel,
 * SSID, live signal level) when it has not. Returns false if the BSS
 * cannot be made visible (channel unknown), in which case a success
 * report would be lost inside cfg80211.
 */
static bool orinoco_ensure_bss_cached(struct orinoco_private *priv,
				      const u8 *bssid, const char *ssid,
				      u16 channel, bool privacy, s32 signal_mbm)
{
	struct wiphy *wiphy = priv_to_wiphy(priv);
	struct ieee80211_channel *chan = NULL;
	struct cfg80211_bss *bss;
	size_t ssid_len = strlen(ssid);
	u8 ie[2 + IEEE80211_MAX_SSID_LEN + 22];
	size_t ie_len;

	bss = cfg80211_get_bss(wiphy, NULL, bssid, ssid, ssid_len,
			       IEEE80211_BSS_TYPE_ESS, IEEE80211_PRIVACY_ANY);
	if (bss) {
		cfg80211_put_bss(wiphy, bss);
		return true;
	}

	if (channel >= 1 && channel <= NUM_CHANNELS)
		chan = ieee80211_get_channel(wiphy,
					     ieee80211_channel_to_frequency(channel,
									    NL80211_BAND_2GHZ));
	if (!chan)
		return false;

	ie[0] = WLAN_EID_SSID;
	ie[1] = ssid_len;
	memcpy(&ie[2], ssid, ssid_len);
	ie_len = ssid_len + 2;
	if (privacy && priv->wpa_mode)
		ie_len += orinoco_assumed_rsn_ie(ie + ie_len);

	/* Beacon interval and timestamp are not retrievable over any
	 * RID; 100 TU is the near-universal default.
	 */
	bss = cfg80211_inform_bss(wiphy, chan, CFG80211_BSS_FTYPE_UNKNOWN,
				  bssid, 0,
				  WLAN_CAPABILITY_ESS |
				  (privacy ? WLAN_CAPABILITY_PRIVACY : 0),
				  100, ie, ie_len, signal_mbm,
				  GFP_KERNEL);
	if (!bss)
		return false;
	cfg80211_put_bss(wiphy, bss);
	return true;
}

/* Process-context half of the HERMES_INQ_LINKSTATUS handling:
 * translate the firmware's association-state transitions into
 * cfg80211_connect_result(), cfg80211_roamed() and
 * cfg80211_disconnected().
 *
 * Driver state is examined and updated under orinoco_lock(); the
 * cfg80211 calls are made after dropping it (they allocate and take
 * their own locks). All three are queue-and-dispatch event APIs, so
 * ordering towards userspace is cfg80211's own.
 */
static void orinoco_process_linkstatus(struct work_struct *work)
{
	struct orinoco_private *priv =
		container_of(work, struct orinoco_private, link_work);
	struct net_device *dev;
	enum {
		LINK_NONE, LINK_CONNECTED, LINK_ROAMED, LINK_ASSOC_FAILED,
		LINK_LOST
	} action = LINK_NONE;
	u8 bssid[ETH_ALEN];
	char ssid[IEEE80211_MAX_SSID_LEN + 1];
	u8 req_ie[ORINOCO_MAX_WPA_IE];
	u8 req_ie_len;
	bool privacy;
	s32 signal_mbm = 0;
	s32 signal_dbm;
	u16 channel = 0;
	u16 status;
	unsigned long flags;
	int err;

	if (orinoco_lock(priv, &flags) != 0)
		return;

	dev = priv->ndev;

	if (priv->iw_mode != NL80211_IFTYPE_STATION)
		goto out_unlock;

	status = priv->last_linkstatus;

	if (orinoco_linkstatus_connected(status)) {
		err = orinoco_hw_get_current_bssid(priv, bssid);
		if (err) {
			netdev_err(dev, "Error %d reading BSSID of new association\n",
				   err);
			goto out_unlock;
		}
		if (hermes_read_wordrec(&priv->hw, USER_BAP,
					HERMES_RID_CURRENTCHANNEL, &channel))
			channel = 0;	/* unknown */
		if (!orinoco_hw_get_signal_dbm(priv, &signal_dbm))
			signal_mbm = signal_dbm * 100;

		if (priv->connect_pending) {
			priv->connect_pending = 0;
			priv->associated = 1;
			memcpy(priv->assoc_bssid, bssid, ETH_ALEN);
			action = LINK_CONNECTED;

			/* Roaming mode 3 (disabled), but only now: set before
			 * the join it stops the firmware from associating at
			 * all. While associated the other modes keep scanning
			 * for other access points, taking the radio off
			 * channel for tens of milliseconds at a time, and a
			 * moved association would need a new handshake that
			 * nothing starts: the supplicant does the roaming.
			 */
			if (priv->wpa_mode &&
			    priv->firmware_type == FIRMWARE_TYPE_INTERSIL)
				hermes_write_wordrec(&priv->hw, USER_BAP,
						     HERMES_RID_CNFROAMINGMODE, 3);
		} else if (priv->associated &&
			   !ether_addr_equal(priv->assoc_bssid, bssid)) {
			memcpy(priv->assoc_bssid, bssid, ETH_ALEN);
			action = LINK_ROAMED;
		}
		/* Remaining connected cases, deliberately silent:
		 * associated with an unchanged BSSID means the firmware
		 * re-established the same association (e.g. after the
		 * reset path) and cfg80211's view is already right;
		 * neither associated nor pending means the firmware
		 * associated at radio level while cfg80211 is idle --
		 * not reported, and the carrier gate in
		 * __orinoco_ev_info() keeps the data path shut.
		 */
	} else {
		if (priv->connect_pending &&
		    status == HERMES_LINKSTATUS_ASSOC_FAILED) {
			priv->connect_pending = 0;
			action = LINK_ASSOC_FAILED;
		} else if (priv->associated && !ignore_disconnect) {
			priv->associated = 0;
			action = LINK_LOST;
		}
		/* A not-yet-connected status while a connect is pending
		 * is a transient (the firmware scans, then associates,
		 * reporting NOT_CONNECTED/DISCONNECTED along the way);
		 * only ASSOC_FAILED is a verdict. Userspace owns the
		 * connect timeout and aborts through .disconnect.
		 */
	}

	memcpy(ssid, priv->desired_essid, sizeof(ssid));
	privacy = (priv->encode_alg != ORINOCO_ALG_NONE) || priv->wpa_mode;
	req_ie_len = priv->wpa_mode ? priv->wpa_ie_len : 0;
	memcpy(req_ie, priv->wpa_ie, req_ie_len);
	orinoco_unlock(priv, &flags);

	switch (action) {
	case LINK_CONNECTED:
		if (!orinoco_ensure_bss_cached(priv, bssid, ssid, channel,
					       privacy, signal_mbm)) {
			/* Reporting success anyway would trip cfg80211's
			 * WARN and lose the event; fail the connect
			 * cleanly instead so userspace retries (with a
			 * scan, which fills the cache). Unwind the
			 * associated bit set above so driver and
			 * cfg80211 agree again.
			 */
			netdev_warn(dev, "Cannot inform cfg80211 of the joined BSS (channel unknown); failing connect\n");
			if (orinoco_lock(priv, &flags) == 0) {
				priv->associated = 0;
				orinoco_unlock(priv, &flags);
			}
			cfg80211_connect_result(dev, NULL, NULL, 0, NULL, 0,
						WLAN_STATUS_UNSPECIFIED_FAILURE,
						GFP_KERNEL);
			break;
		}
		cfg80211_connect_result(dev, bssid, req_ie, req_ie_len, NULL, 0,
					WLAN_STATUS_SUCCESS, GFP_KERNEL);
		break;

	case LINK_ROAMED: {
		struct cfg80211_roam_info info = {};

		if (!orinoco_ensure_bss_cached(priv, bssid, ssid, channel,
					       privacy, signal_mbm)) {
			/* Same WARN-and-drop inside cfg80211_roamed();
			 * with no way to describe the new BSS, the only
			 * honest report is that the old association is
			 * gone -- so the driver's associated bit must
			 * drop with it.
			 */
			netdev_warn(dev, "Roamed to a BSS cfg80211 cannot be told about; reporting disconnect\n");
			if (orinoco_lock(priv, &flags) == 0) {
				priv->associated = 0;
				orinoco_unlock(priv, &flags);
			}
			cfg80211_disconnected(dev, 0, NULL, 0, false,
					      GFP_KERNEL);
			break;
		}
		info.links[0].bssid = bssid;
		if (channel >= 1 && channel <= NUM_CHANNELS) {
			int freq = ieee80211_channel_to_frequency(channel, NL80211_BAND_2GHZ);

			info.links[0].channel = ieee80211_get_channel(priv_to_wiphy(priv), freq);
		}
		cfg80211_roamed(dev, &info, GFP_KERNEL);
		break;
	}

	case LINK_ASSOC_FAILED:
		orinoco_wpa_wipe_keys(priv);
		cfg80211_connect_result(dev, NULL, NULL, 0, NULL, 0,
					WLAN_STATUS_UNSPECIFIED_FAILURE,
					GFP_KERNEL);
		break;

	case LINK_LOST:
		/* The firmware eats the deauth/disassoc frame, so
		 * neither the AP's reason code nor whether one existed
		 * at all is recoverable: reason 0 (unknown), not
		 * locally generated.
		 */
		orinoco_wpa_wipe_keys(priv);
		cfg80211_disconnected(dev, 0, NULL, 0, false, GFP_KERNEL);
		break;

	case LINK_NONE:
		break;
	}

	return;

 out_unlock:
	orinoco_unlock(priv, &flags);
}

/********************************************************************/
/* Internal hardware control routines                               */
/********************************************************************/

static int __orinoco_up(struct orinoco_private *priv)
{
	struct net_device *dev = priv->ndev;
	struct hermes *hw = &priv->hw;
	int err;

	netif_carrier_off(dev); /* just to make sure */

	err = __orinoco_commit(priv);
	if (err) {
		netdev_err(dev, "Error %d configuring card\n", err);
		return err;
	}

	/* Fire things up again */
	hermes_set_irqmask(hw, ORINOCO_INTEN);
	err = hermes_enable_port(hw, 0);
	if (err) {
		netdev_err(dev, "Error %d enabling MAC port\n", err);
		return err;
	}

	netif_start_queue(dev);

	return 0;
}

static int __orinoco_down(struct orinoco_private *priv)
{
	struct net_device *dev = priv->ndev;
	struct hermes *hw = &priv->hw;
	int err;

	netif_stop_queue(dev);

	if (!priv->hw_unavailable) {
		if (!priv->broken_disableport) {
			err = hermes_disable_port(hw, 0);
			if (err) {
				/* Some firmwares (e.g. Intersil 1.3.x) seem
				 * to have problems disabling the port, oh
				 * well, too bad.
				 */
				netdev_warn(dev, "Error %d disabling MAC port\n",
					    err);
				priv->broken_disableport = 1;
			}
		}
		hermes_set_irqmask(hw, 0);
		hermes_write_regn(hw, EVACK, 0xffff);
	}

	orinoco_scan_done(priv, true);

	/* firmware will have to reassociate */
	netif_carrier_off(dev);
	priv->last_linkstatus = 0xffff;

	return 0;
}

static int orinoco_reinit_firmware(struct orinoco_private *priv)
{
	struct hermes *hw = &priv->hw;
	int err;

	err = hw->ops->init(hw);
	if (!err)
		err = orinoco_hw_allocate_fid(priv);

	return err;
}

static int
__orinoco_set_multicast_list(struct net_device *dev)
{
	struct orinoco_private *priv = ndev_priv(dev);
	int err = 0;
	int promisc, mc_count;

	/* The Hermes doesn't seem to have an allmulti mode, so we go
	 * into promiscuous mode and let the upper levels deal.
	 */
	if ((dev->flags & IFF_PROMISC) || (dev->flags & IFF_ALLMULTI) ||
	    (netdev_mc_count(dev) > HERMES_MAX_MULTICAST)) {
		promisc = 1;
		mc_count = 0;
	} else {
		promisc = 0;
		mc_count = netdev_mc_count(dev);
	}

	err = __orinoco_hw_set_multicast_list(priv, dev, mc_count, promisc);

	return err;
}

/* This must be called from user context, without locks held - use
 * schedule_work()
 */
void orinoco_reset(struct work_struct *work)
{
	struct orinoco_private *priv =
		container_of(work, struct orinoco_private, reset_work);
	struct orinoco_ccmp_key *old_keys[ORINOCO_WPA_NKEYS] = {};
	struct hermes *hw = &priv->hw;
	struct net_device *dev;
	bool drop_wpa;
	int err;
	unsigned long flags;

	if (orinoco_lock(priv, &flags) != 0)
		/* When the hardware becomes available again, whatever
		 * detects that is responsible for re-initializing
		 * it. So no need for anything further
		 */
		return;

	dev = priv->ndev;

	netif_stop_queue(dev);

	/* Shut off interrupts.  Depending on what state the hardware
	 * is in, this might not work, but we'll try anyway
	 */
	hermes_set_irqmask(hw, 0);
	hermes_write_regn(hw, EVACK, 0xffff);

	priv->hw_unavailable++;
	priv->last_linkstatus = 0xffff; /* firmware will have to reassociate */
	netif_carrier_off(dev);

	/* A WPA2 association cannot survive the reset silently: the
	 * firmware would rejoin with roaming mode 1 or 2 from the commit
	 * (mode 3 is written only when a pending connect completes) and
	 * the host would keep the old keys and packet numbers with no
	 * handshake to replace them. End it as .disconnect would, so the
	 * supplicant reconnects with a new 4-way handshake. WEP and open
	 * associations keep the silent rejoin: their keys are programmed
	 * again by the commit and carry no per-association state.
	 * associated stays set until the report below, so that a
	 * .disconnect arriving meanwhile reports the loss itself and
	 * this path then does not.
	 */
	drop_wpa = priv->associated && priv->wpa_mode;
	if (drop_wpa)
		__orinoco_forget_connection(priv, old_keys);

	orinoco_unlock(priv, &flags);
	orinoco_wpa_free_keys(old_keys);

	/* Scanning support: Notify scan cancellation */
	orinoco_scan_done(priv, true);

	/* A downloaded firmware lives in the card's RAM and a card reset
	 * reverts it to the flash image, which cannot do what the
	 * connection needs; INIT alone restarts the running one.
	 */
	if (priv->hard_reset && !priv->fw_ram) {
		err = (*priv->hard_reset)(priv);
		if (err) {
			netdev_err(dev, "%s: Error %d performing hard reset\n",
				   __func__, err);
			goto disable;
		}
	}

	err = orinoco_reinit_firmware(priv);
	if (err) {
		netdev_err(dev, "%s: Error %d re-initializing firmware\n",
			   __func__, err);
		goto disable;
	}

	/* This has to be called from user context */
	orinoco_lock_irq(priv);

	priv->hw_unavailable--;

	/* Cleared in the same section that lets link_work run again, so
	 * it never sees the stale association.
	 */
	drop_wpa = drop_wpa && priv->associated;
	if (drop_wpa)
		priv->associated = 0;

	/* priv->open or priv->hw_unavailable might have changed while
	 * we dropped the lock
	 */
	if (priv->open && !priv->hw_unavailable) {
		err = __orinoco_up(priv);
		if (err) {
			netdev_err(dev, "%s: Error %d reenabling card\n",
				   __func__, err);
		} else {
			netif_trans_update(dev);
		}
	}

	orinoco_unlock_irq(priv);

	/* Reported only once the card is back, so that the reconnect it
	 * triggers does not find hw_unavailable and fail with -EBUSY.
	 */
	if (drop_wpa) {
		netdev_info(dev, "Reset ended the WPA2 association\n");
		cfg80211_disconnected(dev, WLAN_REASON_UNSPECIFIED, NULL, 0,
				      true, GFP_KERNEL);
	}

	return;
 disable:
	hermes_set_irqmask(hw, 0);
	netif_device_detach(dev);
	netdev_err(dev, "Device has been disabled!\n");

	orinoco_lock_irq(priv);
	drop_wpa = drop_wpa && priv->associated;
	if (drop_wpa)
		priv->associated = 0;
	orinoco_unlock_irq(priv);
	if (drop_wpa)
		cfg80211_disconnected(dev, WLAN_REASON_UNSPECIFIED, NULL, 0,
				      true, GFP_KERNEL);
}

static int __orinoco_commit(struct orinoco_private *priv)
{
	struct net_device *dev = priv->ndev;
	int err = 0;

	err = orinoco_hw_program_rids(priv);

	/* FIXME: what about netif_tx_lock */
	(void)__orinoco_set_multicast_list(dev);

	return err;
}

/* Ensures configuration changes are applied. May result in a reset.
 * The caller should hold priv->lock
 */
int orinoco_commit(struct orinoco_private *priv)
{
	struct net_device *dev = priv->ndev;
	struct hermes *hw = &priv->hw;
	int err;

	if (priv->broken_disableport) {
		schedule_work(&priv->reset_work);
		return 0;
	}

	err = hermes_disable_port(hw, 0);
	if (err) {
		netdev_warn(dev, "Unable to disable port while reconfiguring card\n");
		priv->broken_disableport = 1;
		goto out;
	}

	err = __orinoco_commit(priv);
	if (err) {
		netdev_warn(dev, "Unable to reconfigure card\n");
		goto out;
	}

	err = hermes_enable_port(hw, 0);
	if (err) {
		netdev_warn(dev, "Unable to enable port while reconfiguring card\n");
		goto out;
	}

 out:
	if (err) {
		netdev_warn(dev, "Resetting instead...\n");
		schedule_work(&priv->reset_work);
		err = 0;
	}
	return err;
}

/********************************************************************/
/* Interrupt handler                                                */
/********************************************************************/

static void __orinoco_ev_tick(struct net_device *dev, struct hermes *hw)
{
	netdev_dbg(dev, "TICK\n");
}

static void __orinoco_ev_wterr(struct net_device *dev, struct hermes *hw)
{
	/* This seems to happen a fair bit under load, but ignoring it
	 * seems to work fine...
	 */
	netdev_dbg(dev, "MAC controller error (WTERR). Ignoring.\n");
}

irqreturn_t orinoco_interrupt(int irq, void *dev_id)
{
	struct orinoco_private *priv = dev_id;
	struct hermes *hw = &priv->hw;
	struct net_device *dev;
	int count = MAX_IRQLOOPS_PER_IRQ;
	u16 evstat, events;
	/* These are used to detect a runaway interrupt situation.
	 *
	 * If we get more than MAX_IRQLOOPS_PER_JIFFY iterations in a jiffy,
	 * we panic and shut down the hardware
	 */
	/* jiffies value the last time we were called */
	static unsigned long last_irq_jiffy; /* = 0 */
	static int loops_this_jiffy; /* = 0 */
	unsigned long flags;

	if (orinoco_lock(priv, &flags) != 0) {
		/* If hw is unavailable - we don't know if the irq was
		 * for us or not
		 */
		return IRQ_HANDLED;
	}

	dev = priv->ndev;

	evstat = hermes_read_regn(hw, EVSTAT);
	events = evstat & hw->inten;
	if (!events) {
		orinoco_unlock(priv, &flags);
		return IRQ_NONE;
	}

	if (time_after(jiffies, last_irq_jiffy))
		loops_this_jiffy = 0;
	last_irq_jiffy = jiffies;

	while (events && count--) {
		if (++loops_this_jiffy > MAX_IRQLOOPS_PER_JIFFY) {
			netdev_warn(dev, "IRQ handler is looping too much! Resetting.\n");
			/* Disable interrupts for now */
			hermes_set_irqmask(hw, 0);
			schedule_work(&priv->reset_work);
			break;
		}

		/* Check the card hasn't been removed */
		if (!hermes_present(hw)) {
			netdev_dbg(dev, "%s(): card removed\n", __func__);
			break;
		}

		if (events & HERMES_EV_TICK)
			__orinoco_ev_tick(dev, hw);
		if (events & HERMES_EV_WTERR)
			__orinoco_ev_wterr(dev, hw);
		if (events & HERMES_EV_INFDROP)
			__orinoco_ev_infdrop(dev, hw);
		if (events & HERMES_EV_INFO)
			__orinoco_ev_info(dev, hw);
		if (events & HERMES_EV_RX)
			__orinoco_ev_rx(dev, hw);
		if (events & HERMES_EV_TXEXC)
			__orinoco_ev_txexc(dev, hw);
		if (events & HERMES_EV_TX)
			__orinoco_ev_tx(dev, hw);
		if (events & HERMES_EV_ALLOC)
			__orinoco_ev_alloc(dev, hw);

		hermes_write_regn(hw, EVACK, evstat);

		evstat = hermes_read_regn(hw, EVSTAT);
		events = evstat & hw->inten;
	}

	/* On sockets that deliver the card's level-triggered IREQ# as an
	 * edge-triggered interrupt, leaving here with events still pending
	 * means no new edge arrives until another event or the watchdog's
	 * reset acknowledges them.
	 */
	if (events && count < 0 && net_ratelimit())
		netdev_warn(dev, "IRQ loop budget exhausted with events %04x still pending; relying on next edge or watchdog\n",
			    events);

	orinoco_unlock(priv, &flags);
	return IRQ_HANDLED;
}
EXPORT_SYMBOL(orinoco_interrupt);

/********************************************************************/
/* Initialization                                                   */
/********************************************************************/

int orinoco_init(struct orinoco_private *priv)
{
	struct device *dev = priv->dev;
	struct wiphy *wiphy = priv_to_wiphy(priv);
	struct hermes *hw = &priv->hw;
	int err = 0;

	/* No need to lock, the hw_unavailable flag is already set in
	 * alloc_orinocodev()
	 */
	priv->nicbuf_size = IEEE80211_MAX_FRAME_LEN + ETH_HLEN;

	/* Initialize the firmware */
	err = hw->ops->init(hw);
	if (err != 0) {
		dev_err(dev, "Failed to initialize firmware (err = %d)\n",
			err);
		goto out;
	}

	err = determine_fw_capabilities(priv, wiphy->fw_version,
					sizeof(wiphy->fw_version),
					&wiphy->hw_version);
	if (err != 0) {
		dev_err(dev, "Incompatible firmware, aborting\n");
		goto out;
	}

	if (priv->do_fw_download)
		dev_info(dev, "Firmware download wanted by firmware type but not supported\n");

	if (priv->has_port3)
		dev_info(dev, "Ad-hoc demo mode supported\n");
	if (priv->has_ibss)
		dev_info(dev, "IEEE standard IBSS ad-hoc mode supported\n");
	if (priv->has_wep)
		dev_info(dev, "WEP supported, %s-bit key\n",
			 priv->has_big_wep ? "104" : "40");
	if (priv->has_wpa)
		dev_info(dev, "WPA-PSK supported\n");

	err = orinoco_hw_read_card_settings(priv, wiphy->perm_addr);
	if (err)
		goto out;

	err = orinoco_hw_allocate_fid(priv);
	if (err) {
		dev_err(dev, "Failed to allocate NIC buffer!\n");
		goto out;
	}

	/* Set up the default configuration */
	priv->iw_mode = NL80211_IFTYPE_STATION;
	/* By default use IEEE/IBSS ad-hoc mode if we have it */
	priv->prefer_port3 = priv->has_port3 && (!priv->has_ibss);
	set_port_type(priv);
	priv->channel = 0; /* use firmware default */

	priv->promiscuous = 0;

	/* The wiphy capabilities depend on the RIDs read above */
	if (orinoco_wiphy_register(wiphy)) {
		err = -ENODEV;
		goto out;
	}

	/* Make the hardware available, as long as it hasn't been
	 * removed elsewhere (e.g. by PCMCIA hot unplug)
	 */
	orinoco_lock_irq(priv);
	priv->hw_unavailable--;
	orinoco_unlock_irq(priv);

	dev_dbg(dev, "Ready\n");

 out:
	return err;
}
EXPORT_SYMBOL(orinoco_init);

static const struct net_device_ops orinoco_netdev_ops = {
	.ndo_open		= orinoco_open,
	.ndo_stop		= orinoco_stop,
	.ndo_start_xmit		= orinoco_xmit,
	.ndo_set_rx_mode	= orinoco_set_multicast_list,
	.ndo_change_mtu		= orinoco_change_mtu,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_tx_timeout		= orinoco_tx_timeout,
};

/* Reports wiphy->fw_version, which ethtool's generic fallback does not */
static const struct ethtool_ops orinoco_ethtool_ops = {
	.get_drvinfo		= cfg80211_get_drvinfo,
};

/* Allocate private data.
 *
 * This driver has a number of structures associated with it
 *  netdev - Net device structure for each network interface
 *  wiphy - structure associated with wireless phy
 *  wireless_dev (wdev) - structure for each wireless interface
 *  hw - structure for hermes chip info
 *  card - card specific structure for use by the card driver
 *         (orinoco_cs)
 *  priv - orinoco private data
 *  device - generic linux device structure
 *
 *  +---------+    +---------+
 *  |  wiphy  |    | netdev  |
 *  | +-------+    | +-------+
 *  | | priv  |    | | wdev  |
 *  | | +-----+    +-+-------+
 *  | | | hw  |
 *  | +-+-----+
 *  | | card  |
 *  +-+-------+
 *
 * priv has a link to netdev and device
 * wdev has a link to wiphy
 *
 * No firmware access happens here: the wiphy capabilities are filled in
 * only after orinoco_init()'s RID reads succeed.
 */
struct orinoco_private
*alloc_orinocodev(int sizeof_card,
		  struct device *device,
		  int (*hard_reset)(struct orinoco_private *),
		  int (*stop_fw)(struct orinoco_private *, int))
{
	struct orinoco_private *priv;
	struct wiphy *wiphy;

	/* allocate wiphy
	 * NOTE: We only support a single virtual interface
	 */
	wiphy = wiphy_new(&orinoco_cfg_ops,
			  sizeof(struct orinoco_private) + sizeof_card);
	if (!wiphy)
		return NULL;

	priv = wiphy_priv(wiphy);
	priv->dev = device;

	if (sizeof_card)
		priv->card = (void *)((unsigned long)priv
				      + sizeof(struct orinoco_private));
	else
		priv->card = NULL;

	orinoco_wiphy_init(wiphy);

	/* Set up default callbacks */
	priv->hard_reset = hard_reset;
	priv->stop_fw = stop_fw;

	spin_lock_init(&priv->lock);
	mutex_init(&priv->prism_fw_lock);
	priv->open = 0;
	/* orinoco_init() must clear this before anything else touches the
	 * hardware
	 */
	priv->hw_unavailable = 1;
	INIT_WORK(&priv->reset_work, orinoco_reset);
	INIT_WORK(&priv->link_work, orinoco_process_linkstatus);
	INIT_WORK(&priv->join_work, orinoco_join_ap);

	INIT_LIST_HEAD(&priv->rx_list);
	tasklet_setup(&priv->rx_tasklet, orinoco_rx_isr_tasklet);

	spin_lock_init(&priv->scan_lock);
	INIT_LIST_HEAD(&priv->scan_list);
	INIT_WORK(&priv->process_scan, orinoco_process_scan_results);

	priv->last_linkstatus = 0xffff;

	return priv;
}
EXPORT_SYMBOL(alloc_orinocodev);

/* We can only support a single interface. We provide a separate
 * function to set it up to distinguish between hardware
 * initialisation and interface setup.
 *
 * The base_addr and irq parameters are passed on to netdev for use
 * with SIOCGIFMAP.
 */
int orinoco_if_add(struct orinoco_private *priv,
		   unsigned long base_addr,
		   unsigned int irq,
		   const struct net_device_ops *ops)
{
	struct wiphy *wiphy = priv_to_wiphy(priv);
	struct wireless_dev *wdev;
	struct net_device *dev;
	int ret;

	dev = alloc_netdev(sizeof(struct wireless_dev), "wlan%d",
			   NET_NAME_ENUM, ether_setup);

	if (!dev)
		return -ENOMEM;

	/* Initialise wireless_dev */
	wdev = netdev_priv(dev);
	wdev->wiphy = wiphy;
	wdev->iftype = NL80211_IFTYPE_STATION;

	/* Setup / override net_device fields */
	dev->ieee80211_ptr = wdev;
	dev->watchdog_timeo = HZ; /* 1 second timeout */

	/* Default to standard ops if not set */
	if (ops)
		dev->netdev_ops = ops;
	else
		dev->netdev_ops = &orinoco_netdev_ops;

	dev->ethtool_ops = &orinoco_ethtool_ops;

	/* we use the default eth_mac_addr for setting the MAC addr */

	/* Reserve space in skb for the SNAP header */
	dev->needed_headroom = ENCAPS_OVERHEAD;

	netif_carrier_off(dev);

	eth_hw_addr_set(dev, wiphy->perm_addr);

	dev->base_addr = base_addr;
	dev->irq = irq;

	dev->min_mtu = ORINOCO_MIN_MTU;
	dev->max_mtu = ORINOCO_MAX_MTU;

	/* Must run before register_netdev() creates the kobject */
	orinoco_prism_fw_sysfs_init(dev);

	SET_NETDEV_DEV(dev, priv->dev);
	ret = register_netdev(dev);
	if (ret)
		goto fail;

	priv->ndev = dev;

	/* Report what we've done */
	dev_dbg(priv->dev, "Registered interface %s.\n", dev->name);

	return 0;

 fail:
	free_netdev(dev);
	return ret;
}
EXPORT_SYMBOL(orinoco_if_add);

void orinoco_if_del(struct orinoco_private *priv)
{
	struct net_device *dev = priv->ndev;

	unregister_netdev(dev);
	priv->ndev = NULL;
	free_netdev(dev);
}
EXPORT_SYMBOL(orinoco_if_del);

void free_orinocodev(struct orinoco_private *priv)
{
	struct wiphy *wiphy = priv_to_wiphy(priv);
	struct orinoco_rx_data *rx_data, *temp;
	struct orinoco_scan_data *sd, *sdtemp;

	/* A queued reset would touch freed memory once wiphy_free() runs;
	 * by this point hw_unavailable is guaranteed non-zero (the caller
	 * has detached the hardware), so a run that started earlier bails
	 * out at its orinoco_lock(). Same for the other works.
	 */
	cancel_work_sync(&priv->reset_work);
	cancel_work_sync(&priv->link_work);
	cancel_work_sync(&priv->join_work);
	mutex_destroy(&priv->prism_fw_lock);

	/* If the tasklet is scheduled when we call tasklet_kill it
	 * will run one final time. However the tasklet will only
	 * drain priv->rx_list if the hw is still available.
	 */
	tasklet_kill(&priv->rx_tasklet);

	/* Explicitly drain priv->rx_list */
	list_for_each_entry_safe(rx_data, temp, &priv->rx_list, list) {
		list_del(&rx_data->list);

		dev_kfree_skb(rx_data->skb);
		kfree(rx_data->desc);
		kfree(rx_data);
	}

	orinoco_wpa_wipe_keys(priv);

	cancel_work_sync(&priv->process_scan);
	/* Explicitly drain priv->scan_list */
	list_for_each_entry_safe(sd, sdtemp, &priv->scan_list, list) {
		list_del(&sd->list);

		if (sd->len > 0)
			kfree(sd->buf);
		kfree(sd);
	}

	wiphy_free(wiphy);
}
EXPORT_SYMBOL(free_orinocodev);

/* Undo one orinoco_down(). This is the counterpart of a completed
 * orinoco_down(), so it must undo it on every return path: the caller
 * has no other handle on priv->hw_unavailable, and nothing in the
 * PCMCIA core retries a failed .resume. Leaving the count raised after a
 * failed firmware re-init would strand the card -- every subsequent
 * orinoco_lock() taker (TX, the cfg80211 ops, the interrupt handler, all
 * four work items) would bail out with -EBUSY forever, with no path back
 * short of an unplug. So the attach and the decrement happen even when
 * the re-init failed, which leaves ifdown/ifup and the tx-timeout reset
 * work able to try again; fw_prism.c's recover: path relies on that.
 *
 * Returns the first error seen, for the caller's log. A non-zero return
 * from the PCMCIA .resume callback is informational -- pcmcia_bus_resume_
 * callback() discards runtime_resume()'s result -- so reporting the
 * failure cannot itself make the socket behave differently.
 */
int orinoco_up(struct orinoco_private *priv)
{
	struct net_device *dev = priv->ndev;
	unsigned long flags;
	int err, ret;

	priv->hw.ops->lock_irqsave(&priv->lock, &flags);

	ret = orinoco_reinit_firmware(priv);
	if (ret)
		netdev_err(dev, "Error %d re-initializing firmware\n", ret);

	netif_device_attach(dev);
	priv->hw_unavailable--;

	if (!ret && priv->open && !priv->hw_unavailable) {
		err = __orinoco_up(priv);
		if (err) {
			netdev_err(dev, "Error %d restarting card\n", err);
			ret = err;
		}
	}

	priv->hw.ops->unlock_irqrestore(&priv->lock, &flags);

	return ret;
}
EXPORT_SYMBOL(orinoco_up);

void orinoco_down(struct orinoco_private *priv)
{
	struct net_device *dev = priv->ndev;
	unsigned long flags;
	int err;

	priv->hw.ops->lock_irqsave(&priv->lock, &flags);
	err = __orinoco_down(priv);
	if (err)
		netdev_warn(dev, "Error %d downing interface\n", err);

	netif_device_detach(dev);
	priv->hw_unavailable++;
	priv->hw.ops->unlock_irqrestore(&priv->lock, &flags);
}
EXPORT_SYMBOL(orinoco_down);
