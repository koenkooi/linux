// SPDX-License-Identifier: GPL-2.0-only
/* Helpers for managing scan queues
 *
 * See copyright notice in core.c.
 */

#include <linux/atomic.h>
#include <linux/gfp.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/ieee80211.h>
#include <net/cfg80211.h>

#include "hermes.h"
#include "orinoco.h"
#include "main.h"

#include "scan.h"

/* The shared orinoco_signal_to_mbm() clamp (orinoco.h) is tuned for Agere's
 * 0x2f..0x8a level scale and pins every Prism result at its ceiling,
 * -11 dBm. Prism has two encodings of its own: a HOSTSCAN atom carries
 * the level already as (256 + dBm) truncated to a byte, while a legacy
 * INQ_SCAN atom (pre-1.3.1 firmware) carries a raw 27..154 level, mapped
 * as x*100/255 - 100.
 */
static s32 prism_level_to_mbm(u16 level, bool hostscan_dbm)
{
	s32 dbm;

	if (hostscan_dbm)
		dbm = (s8)level;
	else
		dbm = (s32)(level & 0xff) * 100 / 255 - 100;

	return dbm * 100;
}

/* Prism scan results carry no information elements: the firmware reports
 * the capability field, rates and SSID, and nothing that says how an
 * encrypted BSS is secured. Userspace that classifies networks from their
 * elements (iwd) would call every one of them WEP and never try it, so an
 * encrypted BSS is described as WPA2-PSK with CCMP, the only encrypted
 * network this driver can join. A BSS that is really WEP or TKIP will
 * fail at association instead.
 *
 * The capabilities word matters: the supplicant compares this element with
 * the one in the access point's handshake message 3 and aborts the
 * handshake on any difference. 0x000c (16 PTKSA replay counters) is what
 * hostapd-based access points advertise.
 */
size_t orinoco_assumed_rsn_ie(u8 *buf)
{
	static const u8 rsn[] = {
		WLAN_EID_RSN, 20, 0x01, 0x00,
		0x00, 0x0f, 0xac, 0x04,			/* group: CCMP */
		0x01, 0x00, 0x00, 0x0f, 0xac, 0x04,	/* pairwise: CCMP */
		0x01, 0x00, 0x00, 0x0f, 0xac, 0x02,	/* AKM: PSK */
		0x0c, 0x00,				/* capabilities: 16 replay counters */
	};

	memcpy(buf, rsn, sizeof(rsn));
	return sizeof(rsn);
}

static int symbol_build_supp_rates(u8 *buf, const __le16 *rates)
{
	int i;
	u8 rate;

	buf[0] = WLAN_EID_SUPP_RATES;
	for (i = 0; i < 5; i++) {
		rate = le16_to_cpu(rates[i]);
		/* NULL terminated */
		if (rate == 0x0)
			break;
		buf[i + 2] = rate;
	}
	buf[1] = i;

	return i + 2;
}

static int prism_build_supp_rates(u8 *buf, const u8 *rates)
{
	int i, n;

	buf[0] = WLAN_EID_SUPP_RATES;
	for (i = 0; i < 8; i++) {
		/* NULL terminated */
		if (rates[i] == 0x0)
			break;
		buf[i + 2] = rates[i];
	}
	buf[1] = i;

	if (i < 8 || rates[8] == 0x0)
		return i + 2;

	/* We might still have another 2 rates, which need to go in
	 * extended supported rates. That is a second element with its own
	 * two-byte header, so its payload starts at buf[12] -- buf[10] and
	 * buf[11] are the EID and length bytes, not rate slots.
	 */
	n = 0;
	buf[10] = WLAN_EID_EXT_SUPP_RATES;
	while (n < 2 && rates[8 + n] != 0x0) {
		/* NULL terminated */
		buf[12 + n] = rates[8 + n];
		n++;
	}
	buf[11] = n;

	return 12 + n;
}

static void orinoco_add_hostscan_result(struct orinoco_private *priv,
					const union hermes_scan_info *bss,
					bool prism_hostscan_dbm)
{
	struct wiphy *wiphy = priv_to_wiphy(priv);
	struct ieee80211_channel *channel;
	struct cfg80211_bss *cbss;
	u8 *ie;
	/* SSID element, then the rates elements the *_build_supp_rates()
	 * helpers below append: a full 8-rate supported-rates element plus,
	 * on Prism, a 2-rate extended-supported-rates element.
	 */
	u8 ie_buf[2 + IEEE80211_MAX_SSID_LEN + 2 + 8 + 2 + 2 + 22];
	u64 timestamp;
	s32 signal;
	u16 capability;
	u16 beacon_interval;
	int ie_len;
	int freq;
	int len;

	len = le16_to_cpu(bss->a.essid_len);
	if (len > sizeof(bss->a.essid))
		len = sizeof(bss->a.essid);

	/* Reconstruct SSID and bitrate IEs to pass up */
	ie_buf[0] = WLAN_EID_SSID;
	ie_buf[1] = len;
	memcpy(&ie_buf[2], bss->a.essid, len);

	ie = ie_buf + len + 2;
	ie_len = ie_buf[1] + 2;
	switch (priv->firmware_type) {
	case FIRMWARE_TYPE_SYMBOL:
		ie_len += symbol_build_supp_rates(ie, bss->s.rates);
		break;

	case FIRMWARE_TYPE_INTERSIL:
		ie_len += prism_build_supp_rates(ie, bss->p.rates);
		break;

	case FIRMWARE_TYPE_AGERE:
	default:
		break;
	}

	freq = ieee80211_channel_to_frequency(le16_to_cpu(bss->a.channel),
					      NL80211_BAND_2GHZ);
	channel = ieee80211_get_channel(wiphy, freq);
	if (!channel) {
		dev_dbg(priv->dev, "Invalid channel designation %04X(%04X)\n",
			bss->a.channel, freq);
		return;	/* Then ignore it for now */
	}
	timestamp = 0;
	capability = le16_to_cpu(bss->a.capabilities);
	if (priv->firmware_type == FIRMWARE_TYPE_INTERSIL && priv->has_wpa &&
	    (capability & WLAN_CAPABILITY_PRIVACY))
		ie_len += orinoco_assumed_rsn_ie(ie_buf + ie_len);
	beacon_interval = le16_to_cpu(bss->a.beacon_interv);
	if (priv->firmware_type == FIRMWARE_TYPE_INTERSIL)
		signal = prism_level_to_mbm(le16_to_cpu(bss->a.level),
					    prism_hostscan_dbm);
	else
		signal = orinoco_signal_to_mbm(le16_to_cpu(bss->a.level));

	cbss = cfg80211_inform_bss(wiphy, channel, CFG80211_BSS_FTYPE_UNKNOWN,
				   bss->a.bssid, timestamp, capability,
				   beacon_interval, ie_buf, ie_len, signal,
				   GFP_KERNEL);
	cfg80211_put_bss(wiphy, cbss);
}

void orinoco_add_extscan_result(struct orinoco_private *priv,
				struct agere_ext_scan_info *bss,
				size_t len)
{
	struct wiphy *wiphy = priv_to_wiphy(priv);
	struct ieee80211_channel *channel;
	struct cfg80211_bss *cbss;
	struct ieee80211_mgmt *mgmt;
	const u8 *ie;
	size_t ie_len, mgmt_len;
	s32 signal;
	int chan, freq;

	ie_len = len - sizeof(*bss);
	ie = cfg80211_find_ie(WLAN_EID_DS_PARAMS, bss->data, ie_len);
	chan = ie ? ie[2] : 0;
	freq = ieee80211_channel_to_frequency(chan, NL80211_BAND_2GHZ);
	channel = ieee80211_get_channel(wiphy, freq);
	if (!channel) {
		dev_dbg(priv->dev, "Invalid ext-scan channel %d(%04X)\n",
			chan, freq);
		return;
	}

	signal = orinoco_signal_to_mbm(bss->level);

	/* The firmware handed us a captured management frame in pieces:
	 * 802.11 header fields, the fixed beacon/probe-response part
	 * (timestamp, beacon interval, capabilities) and the IEs.
	 * Reassemble it and let cfg80211 parse the real frame.
	 */
	mgmt_len = offsetof(struct ieee80211_mgmt, u.beacon.variable) + ie_len;
	mgmt = kzalloc(mgmt_len, GFP_KERNEL);
	if (!mgmt)
		return;

	mgmt->frame_control = bss->frame_control;
	mgmt->duration = bss->dur_id;
	memcpy(mgmt->da, bss->addr1, ETH_ALEN);
	memcpy(mgmt->sa, bss->addr2, ETH_ALEN);
	memcpy(mgmt->bssid, bss->bssid, ETH_ALEN);
	mgmt->seq_ctrl = bss->sequence;
	mgmt->u.beacon.timestamp = bss->timestamp;
	mgmt->u.beacon.beacon_int = bss->beacon_interval;
	mgmt->u.beacon.capab_info = bss->capabilities;
	memcpy(mgmt->u.beacon.variable, bss->data, ie_len);

	if (ieee80211_is_beacon(mgmt->frame_control) ||
	    ieee80211_is_probe_resp(mgmt->frame_control)) {
		cbss = cfg80211_inform_bss_frame(wiphy, channel, mgmt,
						 mgmt_len, signal, GFP_KERNEL);
	} else {
		/* The frame control word comes off the air and may not be
		 * a beacon or probe response cfg80211 can take whole.
		 */
		cbss = cfg80211_inform_bss(wiphy, channel,
					   CFG80211_BSS_FTYPE_UNKNOWN,
					   bss->bssid,
					   le64_to_cpu(bss->timestamp),
					   le16_to_cpu(bss->capabilities),
					   le16_to_cpu(bss->beacon_interval),
					   bss->data, ie_len, signal,
					   GFP_KERNEL);
	}
	cfg80211_put_bss(wiphy, cbss);
	kfree(mgmt);
}

void orinoco_add_hostscan_results(struct orinoco_private *priv,
				  unsigned char *buf,
				  size_t len)
{
	int offset;		/* In the scan data */
	size_t atom_len;
	bool abort = false;
	bool prism_hostscan_dbm = priv->has_hostscan;

	switch (priv->firmware_type) {
	case FIRMWARE_TYPE_AGERE:
		atom_len = sizeof(struct agere_scan_apinfo);
		offset = 0;
		break;

	case FIRMWARE_TYPE_SYMBOL:
		/* Lack of documentation necessitates this hack.
		 * Different firmwares have 68 or 76 byte long atoms.
		 * We try modulo first.  If the length divides by both,
		 * we check what would be the channel in the second
		 * frame for a 68-byte atom.  76-byte atoms have 0 there.
		 * Valid channel cannot be 0.
		 */
		if (len % 76)
			atom_len = 68;
		else if (len % 68)
			atom_len = 76;
		else if (len >= 1292 && buf[68] == 0)
			atom_len = 76;
		else
			atom_len = 68;
		offset = 0;
		break;

	case FIRMWARE_TYPE_INTERSIL:
		offset = 4;
		if (priv->has_hostscan) {
			atom_len = le16_to_cpup((__le16 *)buf);
			/* Sanity check for atom_len */
			if (atom_len < sizeof(struct prism2_scan_apinfo)) {
				dev_err(priv->dev,
					"Invalid atom_len in scan data: %zu\n",
					atom_len);
				abort = true;
				goto scan_abort;
			}
		} else {
			atom_len = offsetof(struct prism2_scan_apinfo, atim);
		}
		break;

	default:
		abort = true;
		goto scan_abort;
	}

	/* Check that we got an whole number of atoms */
	if ((len - offset) % atom_len) {
		dev_err(priv->dev,
			"Unexpected scan data length %zu, atom_len %zu, offset %d\n",
			len, atom_len, offset);
		abort = true;
		goto scan_abort;
	}

	/* Process the entries one by one */
	for (; offset + atom_len <= len; offset += atom_len) {
		union hermes_scan_info *atom;

		atom = (union hermes_scan_info *)(buf + offset);

		orinoco_add_hostscan_result(priv, atom, prism_hostscan_dbm);
	}

 scan_abort:
	orinoco_scan_done(priv, abort);
}

/* Report the pending scan complete, at most once per request.
 *
 * priv->scan_request is written from three contexts with no lock in
 * common: cfg.c's .scan under cfg80211's wiphy lock, this function from
 * workqueue context (both the process_scan work and core.c's reset work
 * reach it), and core.c's hardirq readers. A test followed by a separate
 * clear therefore lets two callers observe the same non-NULL pointer and
 * hand it to cfg80211_scan_done() twice -- the second on a request
 * cfg80211 has already completed and released. cfg80211_scan_done() can
 * sleep, so the window between the test and the clear is wide even on
 * a single CPU.
 *
 * Claiming the pointer with xchg() makes exactly one caller the winner,
 * which removes the double completion structurally rather than by
 * narrowing the window.
 */
void orinoco_scan_done(struct orinoco_private *priv, bool abort)
{
	struct cfg80211_scan_request *request;
	struct cfg80211_scan_info info = {
		.aborted = abort,
	};

	request = xchg(&priv->scan_request, NULL);
	if (request)
		cfg80211_scan_done(request, &info);
}
