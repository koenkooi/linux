/* SPDX-License-Identifier: GPL-2.0-only */
/* hw.h
 *
 * Encapsulate basic setting changes and retrieval on Hermes hardware
 */
#ifndef _ORINOCO_HW_H_
#define _ORINOCO_HW_H_

#include <linux/types.h>

/* Hardware BAPs */
#define USER_BAP 0
#define IRQ_BAP  1

/* Number of supported channels */
#define NUM_CHANNELS 14

/* Forward declarations */
struct orinoco_private;
struct net_device;
struct cfg80211_ssid;

int determine_fw_capabilities(struct orinoco_private *priv, char *fw_name,
			      size_t fw_name_len, u32 *hw_ver);
int orinoco_hw_read_card_settings(struct orinoco_private *priv, u8 *dev_addr);
int orinoco_hw_allocate_fid(struct orinoco_private *priv);
int orinoco_hw_program_rids(struct orinoco_private *priv);
int __orinoco_hw_setup_wepkeys(struct orinoco_private *priv);
int __orinoco_hw_setup_enc(struct orinoco_private *priv);
int __orinoco_hw_set_bitrate(struct orinoco_private *priv);
int orinoco_hw_get_act_bitrate(struct orinoco_private *priv, int *bitrate);
int __orinoco_hw_set_wap(struct orinoco_private *priv);
int __orinoco_hw_set_multicast_list(struct orinoco_private *priv,
				    struct net_device *dev,
				    int mc_count, int promisc);
int orinoco_hw_trigger_scan(struct orinoco_private *priv,
			    const struct cfg80211_ssid *ssid);
int orinoco_hw_disassociate(struct orinoco_private *priv,
			    u8 *addr, u16 reason_code);
int orinoco_hw_get_current_bssid(struct orinoco_private *priv,
				 u8 *addr);
int orinoco_hw_get_signal_dbm(struct orinoco_private *priv, s32 *signal_dbm);

#endif /* _ORINOCO_HW_H_ */
