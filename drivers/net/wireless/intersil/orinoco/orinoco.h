/* SPDX-License-Identifier: GPL-2.0-only */
/* orinoco.h
 *
 * Common definitions to all pieces of the various orinoco drivers
 */

#ifndef _ORINOCO_H
#define _ORINOCO_H

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/minmax.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/ieee80211.h>
#include <net/cfg80211.h>

#include "hermes.h"

/* WEP key sizes in bytes: 40-bit and 104-bit keys. The RID buffer
 * reserves ORINOCO_MAX_KEY_SIZE bytes per key, one byte of padding
 * beyond the largest real key.
 */
#define SMALL_KEY_SIZE		5
#define LARGE_KEY_SIZE		13
#define ORINOCO_MAX_KEY_SIZE	14
#define ORINOCO_MAX_KEYS	4
/* Longest RSN element accepted for the association request */
#define ORINOCO_MAX_WPA_IE	64
#define ORINOCO_WPA_NKEYS	5	/* pairwise key plus four group slots */

struct orinoco_ccmp_key;

/* On-wire layout of one key inside the Agere HERMES_RID_CNFWEPKEYS_AGERE
 * record (written as an array of ORINOCO_MAX_KEYS of these).
 */
struct orinoco_key {
	__le16 len;	/* always stored as little-endian */
	char data[ORINOCO_MAX_KEY_SIZE];
} __packed;

enum orinoco_alg {
	ORINOCO_ALG_NONE,
	ORINOCO_ALG_WEP,
};

/* Firmware signal-level units -> dBm/mBm, zero point 0x95 */
#define ORINOCO_ZERO_DBM_OFFSET		0x95
#define ORINOCO_MAX_SIGNAL_LEVEL	0x8A
#define ORINOCO_MIN_SIGNAL_LEVEL	0x2F

static inline s32 orinoco_signal_to_dbm(s32 level)
{
	return clamp_t(s32, level, ORINOCO_MIN_SIGNAL_LEVEL,
		       ORINOCO_MAX_SIGNAL_LEVEL) - ORINOCO_ZERO_DBM_OFFSET;
}

static inline s32 orinoco_signal_to_mbm(s32 level)
{
	return orinoco_signal_to_dbm(level) * 100;
}

enum fwtype {
	FIRMWARE_TYPE_AGERE,
	FIRMWARE_TYPE_INTERSIL,
	FIRMWARE_TYPE_SYMBOL
};

struct orinoco_private {
	void *card;	/* Pointer to card dependent structure */
	struct device *dev;
	int (*hard_reset)(struct orinoco_private *priv);
	/* Optional bus-glue hooks for genesis-mode (RAM Primary) downloads:
	 * a card soft reset through COR, and a COR soft reset held across a
	 * write of @hcr to the CCSR register. NULL where the bus has no such
	 * registers (fw_prism.c then refuses a Primary download).
	 */
	int (*cor_sreset)(struct orinoco_private *priv);
	int (*genesis_reset)(struct orinoco_private *priv, u8 hcr);

	/* Filled by orinoco_wiphy_register() once the RIDs are read */
	struct ieee80211_supported_band band;
	struct ieee80211_channel channels[14];
	u32 cipher_suites[4];

	/* Synchronisation stuff */
	spinlock_t lock;
	int hw_unavailable;
	struct work_struct reset_work;

	/* Serializes fw_prism.c's operator-triggered volatile download
	 * against itself; a second concurrent trigger fails with -EBUSY
	 * rather than racing two AUX-port programming sequences on a bus
	 * with no error timeout.
	 */
	struct mutex prism_fw_lock;

	/* driver state */
	int open;
	u16 last_linkstatus;

	/* cfg80211 connect state machine, all under priv->lock.
	 * connect_pending is set by cfg.c's .connect once the RIDs are
	 * committed and cleared when core.c's link_work reports the
	 * outcome through cfg80211_connect_result(); associated tracks
	 * the association as reported to cfg80211, and assoc_bssid the
	 * BSSID it was reported with (roam detection, .get_station's
	 * peer check). link_work is the process-context half of the
	 * HERMES_INQ_LINKSTATUS handling; join_work is the Intersil
	 * fixed-BSSID join (manual roaming).
	 */
	unsigned int connect_pending:1;
	unsigned int associated:1;
	u8 assoc_bssid[ETH_ALEN];
	struct work_struct link_work;
	struct work_struct join_work;

	/* Net device stuff */
	struct net_device *ndev;

	/* Hardware control variables */
	struct hermes hw;
	u16 txfid;

	/* Frames are pulled off the card's BAP in hard-IRQ context and
	 * queued here; the tasklet de-encapsulates them.
	 */
	struct tasklet_struct rx_tasklet;
	struct list_head rx_list;

	/* Capabilities of the hardware/firmware */
	enum fwtype firmware_type;
	int ibss_port;
	int nicbuf_size;
	u16 channel_mask;

	/* Live Primary-firmware identity variant and PRI-compat supplier
	 * range, from HERMES_RID_PRIID/HERMES_RID_PRISUPRANGE (hw.c's
	 * determine_fw_capabilities()). Consumed by fw_prism.c's
	 * validate_identity() before a volatile Prism firmware download.
	 * has_pri_fw_id/has_pri_range (below) say whether these are valid --
	 * either RID can fail to read on some cards.
	 */
	u16 pri_fw_variant;
	u16 pri_range_bottom, pri_range_top;
	/* HERMES_RID_NICID id, e.g. 0x800c; fw_prism.c refuses a firmware
	 * image whose own NICID list does not name it.
	 */
	u16 hw_nicid;

	/* Boolean capabilities, set by determine_fw_capabilities() */
	unsigned int has_ibss:1;
	unsigned int has_port3:1;
	unsigned int has_wep:1;
	unsigned int has_big_wep:1;
	unsigned int has_mwo:1;
	unsigned int has_pm:1;
	unsigned int has_preamble:1;
	unsigned int has_sensitivity:1;
	unsigned int has_hostscan:1;
	unsigned int has_alt_txcntl:1;
	unsigned int has_ext_scan:1;
	unsigned int has_wpa:1;
	/* Prism firmware >= 1.3.1 exposes a dedicated dBm comms-quality RID
	 * (and reports scan levels pre-scaled to dBm) instead of the
	 * Agere-scale raw level the shared orinoco_signal_to_*() clamp
	 * expects; only ever set for FIRMWARE_TYPE_INTERSIL.
	 */
	unsigned int has_dbm:1;
	/* A volatile firmware download is running: a card reset would throw
	 * it away, so the reset path reinitialises with INIT only.
	 */
	unsigned int fw_ram:1;
	/* The current connection is WPA2-PSK with host-side CCMP (Prism): the
	 * firmware sends and receives the cipher untouched (HOSTENCRYPT/
	 * HOSTDECRYPT), EAPOL goes in the clear, the RSN element rides in
	 * wpa_ie. wpa_rids_set records that the two 1.7.0 RIDs were written,
	 * so a later non-WPA commit clears them.
	 */
	unsigned int wpa_mode:1;
	unsigned int wpa_rids_set:1;
	unsigned int do_fw_download:1;
	unsigned int broken_disableport:1;
	unsigned int broken_monitor:1;
	unsigned int prefer_port3:1;
	unsigned int has_pri_fw_id:1;
	unsigned int has_pri_range:1;
	unsigned int has_hw_nicid:1;

	/* Configuration parameters */
	enum nl80211_iftype iw_mode;
	int bitratemode;
	char nick[IEEE80211_MAX_SSID_LEN + 1];
	char desired_essid[IEEE80211_MAX_SSID_LEN + 1];
	char desired_bssid[ETH_ALEN];
	int bssid_fixed;

	/* WEP state. keys[] holds the raw key material cfg80211 handed
	 * over via .add_key or the .connect parameters; tx_key is the
	 * default (transmit) key index. Fixed buffers because the
	 * fast-key-change path runs under a spinlock.
	 */
	enum orinoco_alg encode_alg;
	u16 wep_restrict, tx_key;
	struct {
		u8 data[ORINOCO_MAX_KEY_SIZE];
		u8 len;		/* 0 == no key at this index */
	} keys[ORINOCO_MAX_KEYS];

	/* WPA2 host-crypto state, protected by priv->lock. Keys are replaced
	 * and freed outside it (see orinoco_wpa_detach_keys()).
	 */
	u8 wpa_ie[ORINOCO_MAX_WPA_IE];
	u8 wpa_ie_len;
	struct orinoco_ccmp_key *ptk;
	struct orinoco_ccmp_key *gtk[4];

	u16 frag_thresh, mwo_robust;
	u16 channel;
	u16 ap_density, rts_thresh;
	u16 pm_on, pm_mcast, pm_period, pm_timeout;
	u16 preamble;
	u16 short_retry_limit, long_retry_limit;
	u16 retry_lifetime;

	/* Configuration dependent variables */
	int port_type, createibss;
	int promiscuous, mc_count;

	/* Scanning support */
	struct cfg80211_scan_request *scan_request;
	struct work_struct process_scan;
	struct list_head scan_list;
	spinlock_t scan_lock; /* protects the scan list */

	/* Symbol RAM-firmware reset hook; no bus glue sets it yet */
	int (*stop_fw)(struct orinoco_private *priv, int idle);
};

/********************************************************************/
/* Exported prototypes                                              */
/********************************************************************/

struct orinoco_private *alloc_orinocodev(int sizeof_card, struct device *device,
					 int (*hard_reset)(struct orinoco_private *),
					 int (*stop_fw)(struct orinoco_private *, int));
void free_orinocodev(struct orinoco_private *priv);
int orinoco_init(struct orinoco_private *priv);
int orinoco_if_add(struct orinoco_private *priv, unsigned long base_addr,
		   unsigned int irq, const struct net_device_ops *ops);
void orinoco_if_del(struct orinoco_private *priv);
int orinoco_up(struct orinoco_private *priv);
void orinoco_down(struct orinoco_private *priv);
irqreturn_t orinoco_interrupt(int irq, void *dev_id);

/********************************************************************/
/* Locking and synchronization functions                            */
/********************************************************************/

static inline int orinoco_lock(struct orinoco_private *priv,
			       unsigned long *flags)
{
	priv->hw.ops->lock_irqsave(&priv->lock, flags);
	if (priv->hw_unavailable) {
		priv->hw.ops->unlock_irqrestore(&priv->lock, flags);
		return -EBUSY;
	}
	return 0;
}

static inline void orinoco_unlock(struct orinoco_private *priv,
				  unsigned long *flags)
{
	priv->hw.ops->unlock_irqrestore(&priv->lock, flags);
}

static inline void orinoco_lock_irq(struct orinoco_private *priv)
{
	priv->hw.ops->lock_irq(&priv->lock);
}

static inline void orinoco_unlock_irq(struct orinoco_private *priv)
{
	priv->hw.ops->unlock_irq(&priv->lock);
}

/*** Navigate from net_device to orinoco_private ***/
static inline struct orinoco_private *ndev_priv(struct net_device *dev)
{
	struct wireless_dev *wdev = netdev_priv(dev);

	return wdev_priv(wdev);
}

/* WPA2 key table: detach everything under priv->lock, free outside it. */
void orinoco_wpa_detach_keys(struct orinoco_private *priv,
			     struct orinoco_ccmp_key *old[ORINOCO_WPA_NKEYS]);
void orinoco_wpa_free_keys(struct orinoco_ccmp_key *old[ORINOCO_WPA_NKEYS]);
void __orinoco_forget_connection(struct orinoco_private *priv,
				 struct orinoco_ccmp_key *old[ORINOCO_WPA_NKEYS]);

#endif /* _ORINOCO_H */
