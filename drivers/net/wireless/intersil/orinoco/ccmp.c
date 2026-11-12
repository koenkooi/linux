// SPDX-License-Identifier: GPL-2.0-only
/*
 * ccmp.c -- host-side CCMP for Prism firmware, see ccmp.h.
 *
 * The AAD and nonce construction follows mac80211 (net/mac80211/wpa.c,
 * ccmp_gcmp_aad() and ccmp_special_blocks()) for a non-QoS three-address
 * data frame, which is all a station of this firmware ever exchanges; the
 * AEAD request handling follows mac80211's aead_api.c. AES comes from the
 * crypto library, which uses the ARM scalar implementation on this
 * architecture; that is fast enough for an 11 Mbit/s radio.
 */

#include <crypto/aead.h>
#include <linux/err.h>
#include <linux/ieee80211.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "ccmp.h"

#define CCMP_AAD_LEN	22	/* FC, A1, A2, A3, masked SC */
#define CCMP_IV_LEN	16

struct orinoco_ccmp_key {
	struct crypto_aead *tfm;
	u8 tx_pn[ORINOCO_CCMP_PN_LEN];	/* [0] is the most significant byte */
	u8 rx_pn[ORINOCO_CCMP_PN_LEN];
};

struct orinoco_ccmp_key *orinoco_ccmp_key_new(const u8 *key, size_t key_len,
					      const u8 *rx_seq,
					      size_t rx_seq_len)
{
	struct orinoco_ccmp_key *k;
	int err, i;

	if (key_len != 16)
		return ERR_PTR(-EINVAL);
	if (rx_seq && rx_seq_len != ORINOCO_CCMP_PN_LEN)
		return ERR_PTR(-EINVAL);

	k = kzalloc_obj(*k, GFP_KERNEL);
	if (!k)
		return ERR_PTR(-ENOMEM);

	k->tfm = crypto_alloc_aead("ccm(aes)", 0, CRYPTO_ALG_ASYNC);
	if (IS_ERR(k->tfm)) {
		err = PTR_ERR(k->tfm);
		kfree(k);
		return ERR_PTR(err);
	}

	err = crypto_aead_setkey(k->tfm, key, key_len);
	if (!err)
		err = crypto_aead_setauthsize(k->tfm, ORINOCO_CCMP_MIC_LEN);
	if (err) {
		crypto_free_aead(k->tfm);
		kfree(k);
		return ERR_PTR(err);
	}

	/* cfg80211 hands the counter little endian; compare it big endian */
	if (rx_seq)
		for (i = 0; i < ORINOCO_CCMP_PN_LEN; i++)
			k->rx_pn[i] = rx_seq[ORINOCO_CCMP_PN_LEN - 1 - i];

	return k;
}

void orinoco_ccmp_key_free(struct orinoco_ccmp_key *key)
{
	if (IS_ERR_OR_NULL(key))
		return;

	crypto_free_aead(key->tfm);
	kfree_sensitive(key);
}

static void ccmp_aad(u8 *aad, const struct orinoco_ccmp_hdr *hdr)
{
	__le16 fc = hdr->frame_control;

	/* Retry, power management and more-data change on retransmission and
	 * cannot be authenticated; for data frames the subtype bits other
	 * than QoS are masked too, and Protected is always covered.
	 */
	fc &= ~cpu_to_le16(IEEE80211_FCTL_RETRY | IEEE80211_FCTL_PM |
			   IEEE80211_FCTL_MOREDATA | 0x0070);
	fc |= cpu_to_le16(IEEE80211_FCTL_PROTECTED);

	put_unaligned(fc, (__le16 *)&aad[0]);
	memcpy(&aad[2], hdr->addr1, ETH_ALEN);
	memcpy(&aad[8], hdr->addr2, ETH_ALEN);
	memcpy(&aad[14], hdr->addr3, ETH_ALEN);
	aad[20] = *((const u8 *)&hdr->seq_ctrl) & 0x0f;	/* fragment number only */
	aad[21] = 0;
}

static void ccmp_iv(u8 *iv, const struct orinoco_ccmp_hdr *hdr, const u8 *pn)
{
	iv[0] = 1;		/* CCM flags for a two-byte length field */
	iv[1] = 0;		/* TID of a non-QoS frame */
	memcpy(&iv[2], hdr->addr2, ETH_ALEN);
	memcpy(&iv[8], pn, ORINOCO_CCMP_PN_LEN);
	iv[14] = 0;
	iv[15] = 0;
}

static void ccmp_pn2hdr(u8 *hdr, const u8 *pn, int key_id)
{
	hdr[0] = pn[5];
	hdr[1] = pn[4];
	hdr[2] = 0;
	hdr[3] = 0x20 | (key_id << 6);
	hdr[4] = pn[3];
	hdr[5] = pn[2];
	hdr[6] = pn[1];
	hdr[7] = pn[0];
}

static void ccmp_hdr2pn(u8 *pn, const u8 *hdr)
{
	pn[0] = hdr[7];
	pn[1] = hdr[6];
	pn[2] = hdr[5];
	pn[3] = hdr[4];
	pn[4] = hdr[1];
	pn[5] = hdr[0];
}

/* One request with its AAD behind it, as mac80211 does, so the scatterlist
 * never points at the stack.
 */
static struct aead_request *ccmp_req(struct orinoco_ccmp_key *key,
				     const struct orinoco_ccmp_hdr *hdr)
{
	struct aead_request *req;
	size_t reqsize = sizeof(*req) + crypto_aead_reqsize(key->tfm);

	req = kzalloc(reqsize + CCMP_AAD_LEN, GFP_ATOMIC);
	if (!req)
		return NULL;

	ccmp_aad((u8 *)req + reqsize, hdr);
	return req;
}

int orinoco_ccmp_encrypt(struct orinoco_ccmp_key *key, u8 key_id,
			 const struct orinoco_ccmp_hdr *hdr,
			 const u8 *body, size_t len, u8 *out)
{
	struct scatterlist sg[3];
	struct aead_request *req;
	u8 iv[CCMP_IV_LEN];
	u8 *data = out + ORINOCO_CCMP_HDR_LEN;
	u8 *mic = data + len;
	u8 *aad;
	int i, err;

	req = ccmp_req(key, hdr);
	if (!req)
		return -ENOMEM;
	aad = (u8 *)req + sizeof(*req) + crypto_aead_reqsize(key->tfm);

	/* The packet number is incremented before use, and a number is
	 * never reused: running it out means the key must be replaced.
	 */
	for (i = ORINOCO_CCMP_PN_LEN - 1; i >= 0; i--)
		if (++key->tx_pn[i])
			break;
	if (i < 0) {
		kfree_sensitive(req);
		return -EOVERFLOW;
	}

	ccmp_pn2hdr(out, key->tx_pn, key_id);
	memcpy(data, body, len);
	ccmp_iv(iv, hdr, key->tx_pn);

	sg_init_table(sg, 3);
	sg_set_buf(&sg[0], aad, CCMP_AAD_LEN);
	sg_set_buf(&sg[1], data, len);
	sg_set_buf(&sg[2], mic, ORINOCO_CCMP_MIC_LEN);

	aead_request_set_tfm(req, key->tfm);
	aead_request_set_callback(req, 0, NULL, NULL);
	aead_request_set_ad(req, CCMP_AAD_LEN);
	aead_request_set_crypt(req, sg, sg, len, iv);
	err = crypto_aead_encrypt(req);

	kfree_sensitive(req);
	return err;
}

int orinoco_ccmp_decrypt(struct orinoco_ccmp_key *key,
			 const struct orinoco_ccmp_hdr *hdr,
			 const u8 *in, size_t len, u8 *out)
{
	struct scatterlist sg[3];
	struct aead_request *req;
	u8 pn[ORINOCO_CCMP_PN_LEN];
	u8 iv[CCMP_IV_LEN];
	size_t data_len;
	u8 *aad;
	int err;

	if (len < ORINOCO_CCMP_HDR_LEN + ORINOCO_CCMP_MIC_LEN + 1)
		return -EINVAL;
	/* Reserved byte clear and the extended IV flag set */
	if (in[2] != 0 || !(in[3] & 0x20))
		return -EINVAL;

	ccmp_hdr2pn(pn, in);
	if (memcmp(pn, key->rx_pn, ORINOCO_CCMP_PN_LEN) <= 0)
		return -EALREADY;	/* replay */

	data_len = len - ORINOCO_CCMP_HDR_LEN - ORINOCO_CCMP_MIC_LEN;

	req = ccmp_req(key, hdr);
	if (!req)
		return -ENOMEM;
	aad = (u8 *)req + sizeof(*req) + crypto_aead_reqsize(key->tfm);

	memcpy(out, in + ORINOCO_CCMP_HDR_LEN, data_len + ORINOCO_CCMP_MIC_LEN);
	ccmp_iv(iv, hdr, pn);

	sg_init_table(sg, 3);
	sg_set_buf(&sg[0], aad, CCMP_AAD_LEN);
	sg_set_buf(&sg[1], out, data_len);
	sg_set_buf(&sg[2], out + data_len, ORINOCO_CCMP_MIC_LEN);

	aead_request_set_tfm(req, key->tfm);
	aead_request_set_callback(req, 0, NULL, NULL);
	aead_request_set_ad(req, CCMP_AAD_LEN);
	aead_request_set_crypt(req, sg, sg, data_len + ORINOCO_CCMP_MIC_LEN, iv);
	err = crypto_aead_decrypt(req);

	kfree_sensitive(req);
	if (!err)
		memcpy(key->rx_pn, pn, ORINOCO_CCMP_PN_LEN);

	return err;
}
