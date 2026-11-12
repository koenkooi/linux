/* SPDX-License-Identifier: GPL-2.0-only */
/* Helpers for managing scan queues
 *
 * See copyright notice in core.c.
 */
#ifndef _ORINOCO_SCAN_H_
#define _ORINOCO_SCAN_H_

/* Forward declarations */
struct orinoco_private;
struct agere_ext_scan_info;

/* Add scan results */
void orinoco_add_extscan_result(struct orinoco_private *priv,
				struct agere_ext_scan_info *atom,
				size_t len);
void orinoco_add_hostscan_results(struct orinoco_private *dev,
				  unsigned char *buf,
				  size_t len);
void orinoco_scan_done(struct orinoco_private *priv, bool abort);

/* Append the security element assumed for an encrypted BSS on Prism
 * firmware that can do WPA2; returns the number of bytes written.
 */
size_t orinoco_assumed_rsn_ie(u8 *buf);

#endif /* _ORINOCO_SCAN_H_ */
