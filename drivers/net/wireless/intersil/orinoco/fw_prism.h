/* SPDX-License-Identifier: GPL-2.0-only */
/* fw_prism.h -- operator-triggered volatile Intersil/Prism Secondary
 * (Station) firmware download.
 */
#ifndef _ORINOCO_FW_PRISM_H_
#define _ORINOCO_FW_PRISM_H_

struct net_device;

#if IS_ENABLED(CONFIG_HERMES_PRISM_FWDL)
/* Must be called before register_netdev(): net_device.sysfs_groups[] is
 * only consumed at kobject registration time.
 */
void orinoco_prism_fw_sysfs_init(struct net_device *dev);
#else
static inline void orinoco_prism_fw_sysfs_init(struct net_device *dev) {}
#endif

#endif /* _ORINOCO_FW_PRISM_H_ */
