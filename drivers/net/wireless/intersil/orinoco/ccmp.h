/* SPDX-License-Identifier: GPL-2.0-only */
/* ccmp.h -- host-side CCMP (WPA2) for Prism firmware.
 *
 * Prism firmware does no WPA crypto: with CNFWEPFLAGS HOSTENCRYPT and
 * HOSTDECRYPT it transmits a host-built protected frame verbatim and hands
 * a received one up undecrypted. These helpers do the CCM work on the 802.11
 * body (CCMP header, ciphertext, MIC) the way mac80211 does.
 */
#ifndef _ORINOCO_CCMP_H_
#define _ORINOCO_CCMP_H_

#include <linux/types.h>
#include <linux/if_ether.h>

#define ORINOCO_CCMP_HDR_LEN	8
#define ORINOCO_CCMP_MIC_LEN	8
#define ORINOCO_CCMP_PN_LEN	6

struct orinoco_ccmp_key;

/* The 802.11 header fields CCM authenticates and derives its nonce from. */
struct orinoco_ccmp_hdr {
	__le16 frame_control;
	__le16 seq_ctrl;
	const u8 *addr1;
	const u8 *addr2;
	const u8 *addr3;
};

/* @rx_seq is the key's initial receive sequence counter as cfg80211 passes
 * it (six bytes, little endian), or NULL for zero. Process context.
 */
struct orinoco_ccmp_key *orinoco_ccmp_key_new(const u8 *key, size_t key_len,
					      const u8 *rx_seq,
					      size_t rx_seq_len);
void orinoco_ccmp_key_free(struct orinoco_ccmp_key *key);

/* Encrypt @len bytes of @body into @out, which must hold len + 16 bytes:
 * the CCMP header for @key_id, the ciphertext, the MIC. Advances the
 * transmit packet number. Safe in atomic context.
 */
int orinoco_ccmp_encrypt(struct orinoco_ccmp_key *key, u8 key_id,
			 const struct orinoco_ccmp_hdr *hdr,
			 const u8 *body, size_t len, u8 *out);

/* Decrypt and verify @in, which is CCMP header + ciphertext + MIC
 * (@len bytes, at least 17), into @out (len - 16 bytes). Rejects a replayed
 * packet number with -EALREADY (-EINVAL is a malformed header, -EBADMSG a
 * MIC failure) and only advances the receive counter after the MIC checks
 * out. Safe in atomic context.
 */
int orinoco_ccmp_decrypt(struct orinoco_ccmp_key *key,
			 const struct orinoco_ccmp_hdr *hdr,
			 const u8 *in, size_t len, u8 *out);

/* Key id carried in the CCMP header of a received body. */
static inline u8 orinoco_ccmp_hdr_key_id(const u8 *body)
{
	return body[3] >> 6;
}

#endif /* _ORINOCO_CCMP_H_ */
