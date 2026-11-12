// SPDX-License-Identifier: GPL-2.0-only
/* cfg80211 support
 *
 * See copyright notice in core.c.
 */
#include <linux/atomic.h>
#include <linux/etherdevice.h>
#include <linux/ieee80211.h>
#include <net/cfg80211.h>
#include "ccmp.h"
#include "hw.h"
#include "main.h"
#include "orinoco.h"

#include "cfg.h"

/* Supported bitrates. Must agree with hw.c */
static struct ieee80211_rate orinoco_rates[] = {
	{ .bitrate = 10 },
	{ .bitrate = 20 },
	{ .bitrate = 55 },
	{ .bitrate = 110 },
};

static const void * const orinoco_wiphy_privid = &orinoco_wiphy_privid;

/* Called after orinoco_private is allocated. */
void orinoco_wiphy_init(struct wiphy *wiphy)
{
	struct orinoco_private *priv = wiphy_priv(wiphy);

	wiphy->privid = orinoco_wiphy_privid;

	set_wiphy_dev(wiphy, priv->dev);
}

/* Called after firmware is initialised: the band, channel and cipher
 * capabilities come from the RIDs read by determine_fw_capabilities() and
 * orinoco_hw_read_card_settings().
 */
int orinoco_wiphy_register(struct wiphy *wiphy)
{
	struct orinoco_private *priv = wiphy_priv(wiphy);
	int i, channels = 0;

	/* 1 for every firmware type: an active scan request carries one
	 * wildcard SSID entry, which nl80211 rejects against
	 * max_scan_ssids == 0.
	 */
	wiphy->max_scan_ssids = 1;

	wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);

	/* TODO: should we set if we only have demo ad-hoc?
	 *       (priv->has_port3)
	 */
	if (priv->has_ibss)
		wiphy->interface_modes |= BIT(NL80211_IFTYPE_ADHOC);

	priv->band.bitrates = orinoco_rates;
	priv->band.n_bitrates = ARRAY_SIZE(orinoco_rates);

	/* Only support channels allowed by the card EEPROM */
	for (i = 0; i < NUM_CHANNELS; i++) {
		if (priv->channel_mask & (1 << i)) {
			priv->channels[i].center_freq =
				ieee80211_channel_to_frequency(i + 1,
							       NL80211_BAND_2GHZ);
			channels++;
		}
	}
	priv->band.channels = priv->channels;
	priv->band.n_channels = channels;

	wiphy->bands[NL80211_BAND_2GHZ] = &priv->band;
	wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;

	/* TKIP is not advertised: nothing keys it. CCMP is advertised for
	 * Prism firmware only, where the host does the cipher (ccmp.c);
	 * .connect still refuses it unless the firmware is 1.7.0 or later,
	 * which may only be the case after a Station firmware download.
	 */
	i = 0;
	if (priv->has_wep) {
		priv->cipher_suites[i] = WLAN_CIPHER_SUITE_WEP40;
		i++;

		if (priv->has_big_wep) {
			priv->cipher_suites[i] = WLAN_CIPHER_SUITE_WEP104;
			i++;
		}
	}
	if (priv->firmware_type == FIRMWARE_TYPE_INTERSIL) {
		priv->cipher_suites[i] = WLAN_CIPHER_SUITE_CCMP;
		i++;
	}
	wiphy->cipher_suites = priv->cipher_suites;
	wiphy->n_cipher_suites = i;

	wiphy->rts_threshold = priv->rts_thresh;
	if (!priv->has_mwo)
		wiphy->frag_threshold = priv->frag_thresh + 1;
	wiphy->retry_short = priv->short_retry_limit;
	wiphy->retry_long = priv->long_retry_limit;

	return wiphy_register(wiphy);
}

static int orinoco_change_vif(struct wiphy *wiphy, struct net_device *dev,
			      enum nl80211_iftype type,
			      struct vif_params *params)
{
	struct orinoco_private *priv = wiphy_priv(wiphy);
	int err = 0;
	unsigned long lock;

	if (orinoco_lock(priv, &lock) != 0)
		return -EBUSY;

	switch (type) {
	case NL80211_IFTYPE_ADHOC:
		if (!priv->has_ibss && !priv->has_port3)
			err = -EINVAL;
		break;

	case NL80211_IFTYPE_STATION:
		break;

	default:
		err = -EINVAL;
	}

	if (!err) {
		priv->iw_mode = type;
		set_port_type(priv);
		err = orinoco_commit(priv);
	}

	orinoco_unlock(priv, &lock);

	return err;
}

static int orinoco_scan(struct wiphy *wiphy,
			struct cfg80211_scan_request *request)
{
	struct orinoco_private *priv = wiphy_priv(wiphy);
	struct cfg80211_scan_request *pending;
	int err;

	if (!request)
		return -EINVAL;

	/* Install the request atomically. scan.c's completion path claims
	 * this pointer with xchg() from workqueue context, so a test
	 * followed by a separate store could drop a completion on the
	 * floor -- the work clears the slot between the two, and the store
	 * then reinstalls a request nothing will ever complete, leaving
	 * userspace to wait out cfg80211's own 15 s scan timeout.
	 *
	 * A repeat of the request already installed still goes on to
	 * re-trigger the hardware scan.
	 */
	pending = cmpxchg(&priv->scan_request, NULL, request);
	if (pending && pending != request)
		return -EBUSY;

	/* A passive scan has n_ssids == 0 and no ssids array */
	err = orinoco_hw_trigger_scan(priv, request->n_ssids > 0 ?
					    &request->ssids[0] : NULL);
	/* On error we aren't processing the request. Release only our own
	 * request: a plain store here would clear a successor installed
	 * after the completion path already claimed this one.
	 */
	if (err)
		cmpxchg(&priv->scan_request, request, NULL);

	return err;
}

static int orinoco_set_wiphy_params(struct wiphy *wiphy, int radio_idx,
				    u32 changed)
{
	struct orinoco_private *priv = wiphy_priv(wiphy);
	int frag_value = -1;
	int rts_value = -1;
	int err = 0;

	if (changed & WIPHY_PARAM_RETRY_SHORT) {
		/* Setting short retry not supported */
		err = -EINVAL;
	}

	if (changed & WIPHY_PARAM_RETRY_LONG) {
		/* Setting long retry not supported */
		err = -EINVAL;
	}

	if (changed & WIPHY_PARAM_FRAG_THRESHOLD) {
		/* Set fragmentation */
		if (priv->has_mwo) {
			if (wiphy->frag_threshold == -1) {
				frag_value = 0;
			} else {
				wiphy_warn(wiphy,
					   "Fixed fragmentation is not supported on this firmware. Using MWO robust instead.\n");
				frag_value = 1;
			}
		} else {
			if (wiphy->frag_threshold == -1)
				frag_value = 2346;
			else if ((wiphy->frag_threshold < 257) ||
				 (wiphy->frag_threshold > 2347))
				err = -EINVAL;
			else
				/* cfg80211 value is 257-2347 (odd only)
				 * orinoco rid has range 256-2346 (even only)
				 */
				frag_value = wiphy->frag_threshold & ~0x1;
		}
	}

	if (changed & WIPHY_PARAM_RTS_THRESHOLD) {
		/* Set RTS.
		 *
		 * Prism documentation suggests default of 2432,
		 * and a range of 0-3000.
		 *
		 * Current implementation uses 2347 as the default and
		 * the upper limit.
		 */

		if (wiphy->rts_threshold == -1)
			rts_value = 2347;
		else if (wiphy->rts_threshold > 2347)
			err = -EINVAL;
		else
			rts_value = wiphy->rts_threshold;
	}

	if (!err) {
		unsigned long flags;

		if (orinoco_lock(priv, &flags) != 0)
			return -EBUSY;

		if (frag_value >= 0) {
			if (priv->has_mwo)
				priv->mwo_robust = frag_value;
			else
				priv->frag_thresh = frag_value;
		}
		if (rts_value >= 0)
			priv->rts_thresh = rts_value;

		/* A wiphy-level op can legitimately arrive in the window
		 * after wiphy_register() (end of orinoco_init()) and
		 * before the net_device exists (orinoco_if_add()); the
		 * commit path needs the net_device, so just store the
		 * values -- they are programmed by the first commit /
		 * __orinoco_up() anyway.
		 */
		if (priv->ndev)
			err = orinoco_commit(priv);

		orinoco_unlock(priv, &flags);
	}

	return err;
}

/* Store WEP key material into priv->keys[]. priv->lock held. */
static int orinoco_store_key(struct orinoco_private *priv, u8 idx,
			     const u8 *key, size_t key_len)
{
	if (key_len > LARGE_KEY_SIZE)
		return -E2BIG;
	if (key_len > SMALL_KEY_SIZE && !priv->has_big_wep)
		return -E2BIG;

	memcpy(priv->keys[idx].data, key, key_len);
	memset(priv->keys[idx].data + key_len, 0,
	       ORINOCO_MAX_KEY_SIZE - key_len);
	priv->keys[idx].len = key_len;

	return 0;
}

/* CCMP key for the WPA2 connection: the host does the cipher, so the key
 * never goes to the card. The AEAD transform is allocated and keyed before
 * the lock is taken (it can sleep) and the replaced one freed after it.
 */
static int orinoco_add_ccmp_key(struct orinoco_private *priv, u8 key_index,
				bool pairwise, struct key_params *params)
{
	struct orinoco_ccmp_key *key, *old;
	unsigned long flags;

	if (priv->firmware_type != FIRMWARE_TYPE_INTERSIL)
		return -EOPNOTSUPP;
	if (params->key_len != 16 || key_index > 3)
		return -EINVAL;

	key = orinoco_ccmp_key_new(params->key, params->key_len,
				   params->seq_len ? params->seq : NULL,
				   params->seq_len);
	if (IS_ERR(key))
		return PTR_ERR(key);

	if (orinoco_lock(priv, &flags) != 0) {
		orinoco_ccmp_key_free(key);
		return -EBUSY;
	}

	if (!priv->wpa_mode) {
		orinoco_unlock(priv, &flags);
		orinoco_ccmp_key_free(key);
		return -EINVAL;
	}

	if (pairwise) {
		old = priv->ptk;
		priv->ptk = key;
	} else {
		old = priv->gtk[key_index];
		priv->gtk[key_index] = key;
	}

	orinoco_unlock(priv, &flags);
	orinoco_ccmp_key_free(old);

	return 0;
}

static int orinoco_add_key(struct wiphy *wiphy, struct wireless_dev *wdev,
			   int link_id, u8 key_index, bool pairwise,
			   const u8 *mac_addr, struct key_params *params)
{
	struct orinoco_private *priv = wiphy_priv(wiphy);
	unsigned long flags;
	int err = 0;

	if (params->cipher == WLAN_CIPHER_SUITE_CCMP)
		return orinoco_add_ccmp_key(priv, key_index, pairwise, params);

	if (!priv->has_wep)
		return -EOPNOTSUPP;
	if (key_index >= ORINOCO_MAX_KEYS)
		return -EINVAL;
	/* Hermes WEP is default-key based: no pairwise keys */
	if (pairwise)
		return -EOPNOTSUPP;

	switch (params->cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
		break;
	case WLAN_CIPHER_SUITE_WEP104:
		if (!priv->has_big_wep)
			return -EOPNOTSUPP;
		break;
	default:
		return -EOPNOTSUPP;
	}

	if (orinoco_lock(priv, &flags) != 0)
		return -EBUSY;

	err = orinoco_store_key(priv, key_index, params->key,
				params->key_len);
	if (err)
		goto out;

	/* Fast key change on a live WEP association, without
	 * reassociating (needed for 802.1x-style re-keying). With WEP not
	 * enabled, the stored key just waits for the next .connect's
	 * commit.
	 */
	if (priv->encode_alg == ORINOCO_ALG_WEP &&
	    priv->ndev && netif_carrier_ok(priv->ndev))
		err = __orinoco_hw_setup_wepkeys(priv);

 out:
	orinoco_unlock(priv, &flags);
	return err;
}

static int orinoco_del_key(struct wiphy *wiphy, struct wireless_dev *wdev,
			   int link_id, u8 key_index, bool pairwise,
			   const u8 *mac_addr)
{
	struct orinoco_private *priv = wiphy_priv(wiphy);
	struct orinoco_ccmp_key *old = NULL;
	unsigned long flags;

	if (key_index >= ORINOCO_MAX_KEYS)
		return -EINVAL;

	if (orinoco_lock(priv, &flags) != 0)
		return -EBUSY;

	if (priv->wpa_mode) {
		if (pairwise) {
			old = priv->ptk;
			priv->ptk = NULL;
		} else {
			old = priv->gtk[key_index];
			priv->gtk[key_index] = NULL;
		}
		orinoco_unlock(priv, &flags);
		orinoco_ccmp_key_free(old);
		return 0;
	}

	/* Driver state only: the firmware keeps its copy until the next
	 * __orinoco_hw_setup_wepkeys()/commit rewrites all four key
	 * RIDs (which zero-length entries clear). Deleting keys out of
	 * a live association is not a flow WEP userspace performs
	 * except on teardown, where .disconnect's commit follows
	 * anyway.
	 */
	memset(priv->keys[key_index].data, 0,
	       sizeof(priv->keys[key_index].data));
	priv->keys[key_index].len = 0;

	orinoco_unlock(priv, &flags);
	return 0;
}

static int orinoco_set_default_key(struct wiphy *wiphy,
				   struct net_device *netdev, int link_id,
				   u8 key_index, bool unicast, bool multicast)
{
	struct orinoco_private *priv = wiphy_priv(wiphy);
	unsigned long flags;
	int err = 0;

	if (key_index >= ORINOCO_MAX_KEYS)
		return -EINVAL;

	/* A station transmits with the pairwise key; group keys only receive */
	if (priv->wpa_mode)
		return 0;

	if (orinoco_lock(priv, &flags) != 0)
		return -EBUSY;

	priv->tx_key = key_index;

	/* Same fast-change rule as .add_key */
	if (priv->encode_alg == ORINOCO_ALG_WEP &&
	    priv->ndev && netif_carrier_ok(priv->ndev))
		err = __orinoco_hw_setup_wepkeys(priv);

	orinoco_unlock(priv, &flags);
	return err;
}

/* FullMAC connect: program the RIDs (SSID, WEP, auth mode, optionally
 * a pinned BSSID) and bounce the MAC port; the firmware scans and
 * associates on its own, and the eventual HERMES_INQ_LINKSTATUS event
 * completes the request -- core.c's link_work reports
 * cfg80211_connect_result().
 */
static int orinoco_connect(struct wiphy *wiphy, struct net_device *dev,
			   struct cfg80211_connect_params *sme)
{
	struct orinoco_private *priv = wiphy_priv(wiphy);
	struct orinoco_ccmp_key *old_keys[ORINOCO_WPA_NKEYS] = {};
	unsigned long flags;
	bool wep_used;
	bool wpa = false;
	u16 restricted;
	int err;
	int i;

	if (!sme->ssid_len || sme->ssid_len > IEEE80211_MAX_SSID_LEN)
		return -EINVAL;

	/* The one WPA flavour supported is WPA2-PSK with CCMP, on Prism
	 * firmware that can carry the RSN element (1.7.0 or later) and with
	 * the cipher done by the host. Anything else that asks for WPA, an
	 * AKM suite or extra association-request elements is refused here,
	 * which is what makes userspace's failure legible ("not supported"
	 * at connect time, not a hanging association).
	 */
	if (sme->mfp != NL80211_MFP_NO)
		return -EOPNOTSUPP;

	if (sme->crypto.wpa_versions || sme->crypto.n_akm_suites ||
	    sme->ie_len) {
		if (priv->firmware_type != FIRMWARE_TYPE_INTERSIL ||
		    !priv->has_wpa)
			return -EOPNOTSUPP;
		if (sme->crypto.wpa_versions != NL80211_WPA_VERSION_2 ||
		    sme->crypto.n_akm_suites != 1 ||
		    sme->crypto.akm_suites[0] != WLAN_AKM_SUITE_PSK ||
		    sme->crypto.cipher_group != WLAN_CIPHER_SUITE_CCMP ||
		    sme->crypto.n_ciphers_pairwise != 1 ||
		    sme->crypto.ciphers_pairwise[0] != WLAN_CIPHER_SUITE_CCMP)
			return -EOPNOTSUPP;
		if (!sme->ie || !sme->ie_len ||
		    sme->ie_len > ORINOCO_MAX_WPA_IE)
			return -EINVAL;
		wpa = true;
	}

	if (!wpa) {
		switch (sme->crypto.cipher_group) {
		case 0:	/* none specified */
		case WLAN_CIPHER_SUITE_WEP40:
		case WLAN_CIPHER_SUITE_WEP104:
			break;
		default:
			return -EOPNOTSUPP;
		}
		for (i = 0; i < sme->crypto.n_ciphers_pairwise; i++) {
			switch (sme->crypto.ciphers_pairwise[i]) {
			case WLAN_CIPHER_SUITE_WEP40:
			case WLAN_CIPHER_SUITE_WEP104:
				break;
			default:
				return -EOPNOTSUPP;
			}
		}
	}

	switch (sme->auth_type) {
	case NL80211_AUTHTYPE_OPEN_SYSTEM:
	case NL80211_AUTHTYPE_AUTOMATIC:
		/* The firmware has no shared-key fallback try-both mode,
		 * so AUTOMATIC means open system here (the sane WEP
		 * default anyway -- shared-key auth leaks keystream).
		 */
		restricted = 0;
		break;
	case NL80211_AUTHTYPE_SHARED_KEY:
		restricted = 1;
		break;
	default:
		return -EOPNOTSUPP;
	}

	wep_used = !wpa && (sme->key_len || sme->privacy);
	if (wep_used && !priv->has_wep)
		return -EOPNOTSUPP;
	if (sme->key_len && sme->key_idx >= ORINOCO_MAX_KEYS)
		return -EINVAL;
	if (restricted && !wep_used)
		return -EINVAL;

	if (orinoco_lock(priv, &flags) != 0)
		return -EBUSY;

	if (priv->iw_mode != NL80211_IFTYPE_STATION) {
		err = -EOPNOTSUPP;
		goto out;
	}

	/* A new connection starts with no keys of the old one */
	orinoco_wpa_detach_keys(priv, old_keys);
	priv->wpa_mode = wpa;
	if (wpa) {
		memcpy(priv->wpa_ie, sme->ie, sme->ie_len);
		priv->wpa_ie_len = sme->ie_len;
	} else {
		priv->wpa_ie_len = 0;
	}

	if (sme->key_len) {
		err = orinoco_store_key(priv, sme->key_idx, sme->key,
					sme->key_len);
		if (err)
			goto out;
		priv->tx_key = sme->key_idx;
	} else if (wep_used && priv->keys[priv->tx_key].len == 0) {
		/* privacy requested, but no key material -- neither in
		 * this request nor stored by an earlier .add_key
		 */
		err = -EINVAL;
		goto out;
	}

	priv->encode_alg = wep_used ? ORINOCO_ALG_WEP : ORINOCO_ALG_NONE;
	priv->wep_restrict = restricted;

	/* An SSID with embedded NULs cannot survive the strlen()-based
	 * RID programming.
	 */
	memset(priv->desired_essid, 0, sizeof(priv->desired_essid));
	memcpy(priv->desired_essid, sme->ssid, sme->ssid_len);

	/* BSSID: only Intersil/Symbol firmware can pin one; Agere has
	 * no manual-roaming RID, so there it is treated as a hint and
	 * the SSID still selects the network. On Intersil the pin works through CNFROAMINGMODE 2
	 * (__orinoco_hw_set_wap()) plus the join request core.c's
	 * join_work issues when the firmware's scan results arrive.
	 */
	if (sme->bssid && !is_zero_ether_addr(sme->bssid) &&
	    !is_broadcast_ether_addr(sme->bssid) &&
	    priv->firmware_type != FIRMWARE_TYPE_AGERE) {
		priv->bssid_fixed = 1;
		memcpy(priv->desired_bssid, sme->bssid, ETH_ALEN);
	} else {
		if (sme->bssid && priv->firmware_type == FIRMWARE_TYPE_AGERE)
			netdev_dbg(dev, "Agere firmware cannot pin a BSSID; treating it as a hint\n");
		priv->bssid_fixed = 0;
		eth_zero_addr(priv->desired_bssid);
	}

	/* sme->channel is likewise only a hint: for a STATION the
	 * firmware picks the channel from its own scan
	 * (orinoco_hw_program_rids() writes CNFOWNCHANNEL only for
	 * IBSS).
	 */

	priv->connect_pending = 1;
	priv->associated = 0;

	err = orinoco_commit(priv);
	if (err)
		priv->connect_pending = 0;

 out:
	orinoco_unlock(priv, &flags);
	orinoco_wpa_free_keys(old_keys);
	return err;
}

static int orinoco_disconnect(struct wiphy *wiphy, struct net_device *dev,
			      u16 reason_code)
{
	struct orinoco_private *priv = wiphy_priv(wiphy);
	struct orinoco_ccmp_key *old_keys[ORINOCO_WPA_NKEYS] = {};
	bool was_associated;
	int err = 0;

	/* Must run even with the hardware unavailable (PCMCIA pull
	 * mid-association): cfg80211's teardown path calls this and the
	 * driver state has to unwind regardless, so take the plain lock
	 * and skip the RID work when the hardware is gone.
	 */
	orinoco_lock_irq(priv);

	was_associated = priv->associated;
	priv->connect_pending = 0;
	priv->associated = 0;

	/* An empty DESIREDSSID means "any" to this firmware generation
	 * -- there is no true idle mode. The commit below bounces the
	 * MAC port, which drops the current association; if the radio
	 * later joins some open AP on its own, core.c's STATION carrier
	 * gate keeps the data path shut and nothing is reported to
	 * cfg80211.
	 */
	__orinoco_forget_connection(priv, old_keys);

	if (!priv->hw_unavailable && netif_running(dev)) {
		/* Tell the AP first where the firmware offers it
		 * (Agere-WPA only).
		 * Best-effort: -EOPNOTSUPP everywhere else, and the
		 * port bounce below is the disconnect either way.
		 */
		if (was_associated)
			(void)orinoco_hw_disassociate(priv,
						       priv->assoc_bssid,
						       reason_code);
		err = orinoco_commit(priv);
	}

	orinoco_unlock_irq(priv);
	orinoco_wpa_free_keys(old_keys);

	/* cfg80211 clears a pending (never-completed) connect attempt
	 * itself when .disconnect returns; only an established
	 * association needs the explicit event.
	 */
	if (was_associated)
		cfg80211_disconnected(dev, reason_code, NULL, 0, true,
				      GFP_KERNEL);

	return err;
}

/* STATION mode only: the only peer the firmware knows anything about is
 * the current AP.
 */
static int orinoco_get_station(struct wiphy *wiphy, struct wireless_dev *wdev,
			       const u8 *mac, struct station_info *sinfo)
{
	struct orinoco_private *priv = wiphy_priv(wiphy);
	struct net_device *dev = priv->ndev;
	unsigned long flags;
	s32 signal_dbm;
	int bitrate;

	if (priv->iw_mode != NL80211_IFTYPE_STATION)
		return -EOPNOTSUPP;

	if (orinoco_lock(priv, &flags) != 0)
		return -EBUSY;

	if (!priv->associated || !ether_addr_equal(mac, priv->assoc_bssid)) {
		orinoco_unlock(priv, &flags);
		return -ENOENT;
	}

	if (!orinoco_hw_get_signal_dbm(priv, &signal_dbm)) {
		sinfo->signal = signal_dbm;
		sinfo->filled |= BIT_ULL(NL80211_STA_INFO_SIGNAL);
	}

	if (!orinoco_hw_get_act_bitrate(priv, &bitrate)) {
		/* txrate.legacy is in units of 100 kbit/s */
		sinfo->txrate.legacy = bitrate / 100000;
		sinfo->filled |= BIT_ULL(NL80211_STA_INFO_TX_BITRATE);
	}

	orinoco_unlock(priv, &flags);

	sinfo->filled |= BIT_ULL(NL80211_STA_INFO_TX_BYTES) |
			 BIT_ULL(NL80211_STA_INFO_TX_PACKETS) |
			 BIT_ULL(NL80211_STA_INFO_RX_BYTES) |
			 BIT_ULL(NL80211_STA_INFO_RX_PACKETS) |
			 BIT_ULL(NL80211_STA_INFO_TX_FAILED);
	sinfo->tx_bytes = dev->stats.tx_bytes;
	sinfo->tx_packets = dev->stats.tx_packets;
	sinfo->rx_bytes = dev->stats.rx_bytes;
	sinfo->rx_packets = dev->stats.rx_packets;
	sinfo->tx_failed = dev->stats.tx_errors;

	return 0;
}

const struct cfg80211_ops orinoco_cfg_ops = {
	.change_virtual_intf = orinoco_change_vif,
	.scan = orinoco_scan,
	.set_wiphy_params = orinoco_set_wiphy_params,
	.connect = orinoco_connect,
	.disconnect = orinoco_disconnect,
	.add_key = orinoco_add_key,
	.del_key = orinoco_del_key,
	.set_default_key = orinoco_set_default_key,
	.get_station = orinoco_get_station,
};
