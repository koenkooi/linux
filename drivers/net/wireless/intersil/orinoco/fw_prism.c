// SPDX-License-Identifier: GPL-2.0-only
/* fw_prism.c -- operator-triggered volatile (RAM-only) download of an
 * Intersil Prism Secondary/Station firmware image, gated on
 * FIRMWARE_TYPE_INTERSIL and triggered by a write to
 * /sys/class/net/wlanN/prism_download. Never runs automatically at
 * probe time.
 *
 * Every hardware write in this file goes through hw->ops->init(),
 * ->program_init(), ->program() and ->program_end(), and
 * hermesi_program_init() hardcodes HERMES_PROGRAM_ENABLE_VOLATILE: the
 * card's flash is never programmed.
 *
 * Before the download, the image's PDR plug records are filled from the
 * card's own Production Data Area (read over the AUX port and CRC-checked)
 * and the CRC signature records are stamped, without which the Primary
 * firmware does not accept the Secondary. A plug whose target lies in no
 * image chunk is skipped with a warning rather than failing the
 * download, and a PDA record with no matching plug is the normal case:
 * apart from the MAC address, the serial number and the TX calibration
 * records the image carries usable defaults.
 */

#include <linux/crc16.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/hex.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/unaligned.h>
#include <net/cfg80211.h>

#include "hermes.h"
#include "hw.h"
#include "orinoco.h"
#include "fw_prism.h"

#define PRISM_FW_NAME	"intersil/prism_sta_fw.hex"
#define PRISM_FW_PRI_NAME	"intersil/prism_pri_fw.hex"
MODULE_FIRMWARE(PRISM_FW_NAME);
MODULE_FIRMWARE(PRISM_FW_PRI_NAME);

/* S-record sentinel addresses inside the S3 stream. The program-mode
 * entry point is taken from the S7 termination record's address field,
 * not from a START sentinel.
 */
#define PRISM_FW_ADDR_PLUG	0xff000000u
#define PRISM_FW_ADDR_CRC	0xff100000u
#define PRISM_FW_ADDR_INFO	0xff200000u
#define PRISM_FW_ADDR_START	0xff400000u
#define PRISM_FW_ADDR_SENTINEL_TOPBYTE	0xffu

/* PDR "compatibility range" supplier id -- matches hw.c's ranges[] table
 * (HERMES_RID_PRISUPRANGE == the "PRI" role, id 3).
 */
#define PRISM_FW_SUPPLIER_ID_PRI	3

/* S3ADDR_INFO record codes and component ids, as laid out in the images:
 * {words, code, ...} with words counting everything after itself.
 *   code 1 (component):  id, variant, major -- id 0x15 Primary, 0x1f Station
 *   code 2 (compat):     role, id, variant, bottom, top
 *                        role 0 = this image supplies that interface range,
 *                        role 1 = this image needs it (id 3 = Primary)
 *   code 4 (NICID):      nicid, bottom, top -- one record per card model
 */
#define PRISM_FW_INFO_COMPONENT		1
#define PRISM_FW_INFO_COMPAT		2
#define PRISM_FW_INFO_NICID		4
#define PRISM_FW_COMPONENT_PRI		0x0015
#define PRISM_FW_COMPONENT_STA		0x001f
#define PRISM_FW_MAX_NICID		48

/* The card's own Production Data Record area, read over the AUX port: a
 * flat little-endian 16-bit word stream, one record per {len, code,
 * (len-1) data words}, len counting the code word plus data but not
 * itself, terminated by a {len=2, code=0x0000} record. A CRC-16/ARC
 * (crc16()) over every byte up to and including the terminator's one data
 * word (the stored CRC) must come out zero.
 */
#define PRISM_FW_PDA_LEN_MAX		1024u	/* HFA384x_PDA_LEN_MAX, bytes */
#define PRISM_FW_PDA_RECS_MAX		200u	/* HFA384x_PDA_RECS_MAX */
#define PRISM_FW_PDA_END_CODE		0x0000u	/* HFA384x_PDR_END_OF_PDA */

/* hermes_read_pda() prepends a synthetic 2-word {len, 0x0800} header to
 * the data it reads, so the PDA proper starts two words in and the buffer
 * must be sized for that header plus the maximum.
 */
#define PRISM_FW_PDA_BUF_WORDS		((PRISM_FW_PDA_LEN_MAX + 4) / 2)
#define PRISM_FW_PDA_REAL_OFFSET_WORDS	2

/* Candidate PDA base addresses, tried in this order until one reads back
 * a structurally valid, CRC-16-clean PDA.
 */
static const u32 prism_fw_pda_addrs[] = {
	0x7f0000u,	/* every card except HFA3841-based ones */
	0x3f0000u,	/* HFA3841 */
	0x390000u,	/* older cards */
	0x7f0002u,	/* Intel PRO/Wireless 2011B (PCI) */
};

struct prism_fw_pda {
	__le16 buf[PRISM_FW_PDA_BUF_WORDS];
	/* word offset of the {len=2,code=0} terminator */
	unsigned int end_word;
	u32 addr;			/* which candidate address validated */
};

struct srec_line {
	char type;		/* '3' or '7'; anything else is rejected */
	u32  addr;
	u8   data[255];		/* max representable by an 8-bit byte count */
	u8   data_len;
};

/* Decode exactly @n hex digits at @s. -EINVAL on any non-hex byte. */
static int hex2val(const char *s, unsigned int n, u32 *val)
{
	u32 v = 0;
	unsigned int i;
	int c;

	for (i = 0; i < n; i++) {
		c = hex_to_bin(s[i]);
		if (c < 0)
			return -EINVAL;
		v = (v << 4) | (u32)c;
	}
	*val = v;
	return 0;
}

/* Parse one already-isolated line (no trailing CR/LF). Only S3 (32-bit
 * load-data) and S7 (32-bit start/termination) records are accepted;
 * anything else is a hard parse failure.
 *
 * Standard Motorola S-record checksum: sum(byte_count, address bytes,
 * data bytes, checksum byte) & 0xff must equal 0xff.
 */
static int srec_parse_line(const char *line, size_t linelen,
			   struct srec_line *out)
{
	u32 byte_count, addr, sum;
	unsigned int pos, i;
	u8 bc, checksum;

	if (linelen < 4 || line[0] != 'S')
		return -EINVAL;
	if (line[1] != '3' && line[1] != '7')
		return -EINVAL;
	out->type = line[1];

	if (hex2val(&line[2], 2, &byte_count))
		return -EINVAL;
	bc = (u8)byte_count;

	/* addr(4) + checksum(1) is the minimum for either accepted type */
	if (bc < 5)
		return -EINVAL;

	if (linelen != (size_t)(4 + (unsigned int)bc * 2))
		return -EINVAL;

	if (hex2val(&line[4], 8, &addr))
		return -EINVAL;
	out->addr = addr;

	out->data_len = bc - 4 - 1;
	if (out->data_len > sizeof(out->data))
		return -EINVAL;

	pos = 4 + 8;
	for (i = 0; i < out->data_len; i++, pos += 2) {
		u32 b;

		if (hex2val(&line[pos], 2, &b))
			return -EINVAL;
		out->data[i] = (u8)b;
	}

	{
		u32 cksum;

		if (hex2val(&line[pos], 2, &cksum))
			return -EINVAL;
		checksum = (u8)cksum;
	}

	sum = bc;
	sum += (addr >> 24) & 0xff;
	sum += (addr >> 16) & 0xff;
	sum += (addr >> 8) & 0xff;
	sum += addr & 0xff;
	for (i = 0; i < out->data_len; i++)
		sum += out->data[i];
	sum += checksum;

	if ((sum & 0xff) != 0xff)
		return -EINVAL;

	return 0;
}

/* Return the next non-empty line in [p, end), stripping a trailing \r.
 * NULL when no more lines remain.
 */
static const char *next_line(const char *p, const char *end, size_t *len)
{
	const char *nl;

	while (p < end && (*p == '\r' || *p == '\n'))
		p++;
	if (p >= end)
		return NULL;

	nl = memchr(p, '\n', (size_t)(end - p));
	*len = nl ? (size_t)(nl - p) : (size_t)(end - p);
	if (*len && p[*len - 1] == '\r')
		(*len)--;

	return p;
}

struct prism_fw_chunk {
	u32 addr;
	u32 len;
	/* Points into img->blob, except for a synthetic 2-byte CRC
	 * lead-in chunk, which points into img->crc_stamp_buf.
	 */
	const u8 *data;
};

/* One S3ADDR_PLUG record's 12-byte data field, three little-endian u32:
 * itemcode identifies a PDR to fetch from the card's own PDA (0xffffffff
 * means "not a PDR -- write this image's own filename instead"), addr is
 * the target byte address inside the load image (not the card), len is
 * the length in bytes.
 */
struct prism_fw_plug {
	u32 itemcode;
	u32 addr;
	u32 len;
};

/* One S3ADDR_CRC record's 12-byte data field: addr is the byte address,
 * in the load image, immediately after where the two-byte 0xC0DE
 * signature stamp goes (the stamp itself lands at addr-2); len is unused;
 * dowrite gates whether this record is acted on at all.
 */
struct prism_fw_crc {
	u32 addr;
	u32 len;
	bool dowrite;
};

struct prism_fw_image {
	struct prism_fw_chunk *chunks;
	unsigned int n_chunks;
	unsigned int n_chunks_alloc;
	u8 *blob;			/* backing store, in file order */
	size_t blob_len;
	u32 entry_point;		/* from the file's one S7 record */
	bool have_entry_point;
	unsigned int n_plug, n_crc, n_info;
	struct prism_fw_plug *plugs;	/* n_plug entries */
	struct prism_fw_crc *crcs;	/* n_crc entries */
	u8 *crc_stamp_buf;		/* 2 * n_crc bytes */
	unsigned int n_crc_stamps_used;
	u16 component_id;		/* INFO code 1 */
	bool have_component;
	u16 req_pri_bottom, req_pri_top;	/* INFO code 2, role 1, id 3 */
	bool have_req_pri;
	u16 sup_pri_bottom, sup_pri_top;	/* INFO code 2, role 0, id 3 */
	bool have_sup_pri;
	u16 nicid[PRISM_FW_MAX_NICID];	/* INFO code 4 */
	unsigned int n_nicid;
};

static void prism_fw_free(struct prism_fw_image *img)
{
	kvfree(img->chunks);
	kvfree(img->blob);
	kfree(img->plugs);
	kfree(img->crcs);
	kfree(img->crc_stamp_buf);
	memset(img, 0, sizeof(*img));
}

/* Decode one S3ADDR_INFO record. The payload is a little-endian u16 word
 * stream {words, code, fields...} (see the PRISM_FW_INFO_* comment above);
 * records that are too short or of an unknown code are ignored.
 */
static void prism_fw_parse_info(struct prism_fw_image *img,
				const u8 *data, u8 len)
{
	u16 code, role, id, bottom, top;

	if (len < 8)
		return;

	code = get_unaligned_le16(&data[2]);

	switch (code) {
	case PRISM_FW_INFO_COMPONENT:
		if (!img->have_component) {
			img->component_id = get_unaligned_le16(&data[4]);
			img->have_component = true;
		}
		break;
	case PRISM_FW_INFO_COMPAT:
		if (len < 14)
			return;
		role = get_unaligned_le16(&data[4]);
		id = get_unaligned_le16(&data[6]);
		bottom = get_unaligned_le16(&data[10]);
		top = get_unaligned_le16(&data[12]);
		if (id != PRISM_FW_SUPPLIER_ID_PRI)
			return;
		if (role == 1) {
			img->req_pri_bottom = bottom;
			img->req_pri_top = top;
			img->have_req_pri = true;
		} else if (role == 0) {
			img->sup_pri_bottom = bottom;
			img->sup_pri_top = top;
			img->have_sup_pri = true;
		}
		break;
	case PRISM_FW_INFO_NICID:
		if (img->n_nicid < PRISM_FW_MAX_NICID)
			img->nicid[img->n_nicid++] = get_unaligned_le16(&data[4]);
		break;
	}
}

static bool prism_fw_is_sentinel(u32 addr)
{
	return (addr >> 24) == PRISM_FW_ADDR_SENTINEL_TOPBYTE;
}

/* Two passes over the same buffer: pass 1 validates every line and
 * counts load records (to size one exact allocation); pass 2 fills it in.
 */
static int prism_fw_parse(const struct firmware *fw, struct prism_fw_image *img)
{
	const char *base = (const char *)fw->data;
	const char *end = base + fw->size;
	const char *p, *line;
	size_t len, total_bytes = 0, off = 0;
	unsigned int n_s3 = 0;
	unsigned int n_s7 = 0;
	unsigned int n_plug_seen = 0, n_crc_seen = 0;
	int err;

	memset(img, 0, sizeof(*img));

	p = base;
	while ((line = next_line(p, end, &len)) != NULL) {
		struct srec_line sl;

		err = srec_parse_line(line, len, &sl);
		if (err) {
			pr_err("prism_download: malformed/corrupt S-record at file offset %td\n",
			       line - base);
			return err;
		}

		if (sl.type == '7') {
			/* The start-address record is not always last */
			n_s7++;
			img->entry_point = sl.addr;
			img->have_entry_point = true;
		} else if (prism_fw_is_sentinel(sl.addr)) {
			switch (sl.addr) {
			case PRISM_FW_ADDR_PLUG:
				img->n_plug++;
				break;
			case PRISM_FW_ADDR_CRC:
				img->n_crc++;
				break;
			case PRISM_FW_ADDR_INFO:
				img->n_info++;
				break;
			case PRISM_FW_ADDR_START:
				break;
			default:
				pr_warn("prism_download: unrecognised sentinel address 0x%08x, ignoring\n",
					sl.addr);
			}
		} else {
			n_s3++;
			total_bytes += sl.data_len;
		}

		p = line + len;
	}

	if (n_s7 != 1) {
		pr_err("prism_download: expected exactly one S7 start-address record, found %u\n",
		       n_s7);
		return -EINVAL;
	}
	if (!n_s3) {
		pr_err("prism_download: no S3 load records found\n");
		return -EINVAL;
	}

	/* One spare chunk per CRC record, for a synthetic 2-byte lead-in */
	img->n_chunks_alloc = n_s3 + img->n_crc;
	img->chunks = kvcalloc(img->n_chunks_alloc, sizeof(*img->chunks), GFP_KERNEL);
	if (!img->chunks)
		return -ENOMEM;
	img->blob = kvmalloc(total_bytes, GFP_KERNEL);
	if (!img->blob) {
		kvfree(img->chunks);
		img->chunks = NULL;
		return -ENOMEM;
	}
	img->blob_len = total_bytes;

	if (img->n_plug) {
		img->plugs = kcalloc(img->n_plug, sizeof(*img->plugs), GFP_KERNEL);
		if (!img->plugs) {
			prism_fw_free(img);
			return -ENOMEM;
		}
	}
	if (img->n_crc) {
		img->crcs = kcalloc(img->n_crc, sizeof(*img->crcs), GFP_KERNEL);
		if (!img->crcs) {
			prism_fw_free(img);
			return -ENOMEM;
		}
		img->crc_stamp_buf = kcalloc(img->n_crc, 2, GFP_KERNEL);
		if (!img->crc_stamp_buf) {
			prism_fw_free(img);
			return -ENOMEM;
		}
	}

	p = base;
	while ((line = next_line(p, end, &len)) != NULL) {
		struct srec_line sl;

		err = srec_parse_line(line, len, &sl);
		if (err) {
			prism_fw_free(img);
			return err;
		}

		if (sl.type == '7') {
			p = line + len;
			continue;
		}

		if (prism_fw_is_sentinel(sl.addr)) {
			switch (sl.addr) {
			case PRISM_FW_ADDR_INFO:
				prism_fw_parse_info(img, sl.data, sl.data_len);
				break;
			case PRISM_FW_ADDR_PLUG:
				if (sl.data_len != 12) {
					pr_err("prism_download: PDA-plug record with unexpected length %u (want 12), aborting\n",
					       sl.data_len);
					prism_fw_free(img);
					return -EINVAL;
				}
				img->plugs[n_plug_seen].itemcode = get_unaligned_le32(&sl.data[0]);
				img->plugs[n_plug_seen].addr = get_unaligned_le32(&sl.data[4]);
				img->plugs[n_plug_seen].len = get_unaligned_le32(&sl.data[8]);
				n_plug_seen++;
				break;
			case PRISM_FW_ADDR_CRC:
				if (sl.data_len != 12) {
					pr_err("prism_download: CRC record with unexpected length %u (want 12), aborting\n",
					       sl.data_len);
					prism_fw_free(img);
					return -EINVAL;
				}
				img->crcs[n_crc_seen].addr = get_unaligned_le32(&sl.data[0]);
				img->crcs[n_crc_seen].len = get_unaligned_le32(&sl.data[4]);
				img->crcs[n_crc_seen].dowrite =
					get_unaligned_le32(&sl.data[8]) != 0;
				n_crc_seen++;
				break;
			default:
				break;
			}
		} else {
			struct prism_fw_chunk *c = &img->chunks[img->n_chunks];

			memcpy(img->blob + off, sl.data, sl.data_len);
			c->addr = sl.addr;
			c->len = sl.data_len;
			c->data = img->blob + off;
			off += sl.data_len;
			img->n_chunks++;
		}

		p = line + len;
	}

	return 0;
}

/* Structure-walk one already-read PDA (@real points past
 * hermes_read_pda()'s synthetic 2-word header) and, if it terminates
 * cleanly within @max_words and its CRC-16/ARC comes back zero, report
 * where the {len=2,code=0} terminator record starts.
 */
static int prism_fw_pda_validate(const __le16 *real, unsigned int max_words,
				 unsigned int *out_end_word)
{
	unsigned int pos = 0, nrec = 0;
	u16 rlen, code;

	while (pos + 1 < max_words) {
		rlen = le16_to_cpu(real[pos]);
		code = le16_to_cpu(real[pos + 1]);

		if (rlen == 2 && code == PRISM_FW_PDA_END_CODE)
			goto found;

		if (rlen == 0 || pos + rlen + 1 > max_words)
			return -EINVAL;

		pos += rlen + 1;
		if (++nrec > PRISM_FW_PDA_RECS_MAX)
			return -EINVAL;
	}
	return -EINVAL;

found:
	/* The terminator's len=2 covers its code word plus one data word,
	 * the stored CRC-16; including it makes a valid PDA's crc16() zero.
	 */
	if (pos + 3 > max_words)
		return -EINVAL;
	if (crc16(0, (const u8 *)real, (pos + 3) * 2) != 0)
		return -EINVAL;

	*out_end_word = pos;
	return 0;
}

/* Probe the candidate PDA base addresses in order, accepting the first
 * that reads back a structurally valid, CRC-16-clean PDA. Read-only.
 */
static int prism_fw_read_pda(struct hermes *hw, struct device *dev,
			     struct prism_fw_pda *pda)
{
	unsigned int i;
	int err;

	if (!hw->ops->read_pda) {
		dev_err(dev, "prism_download: no read_pda hermes op on this bus, cannot plug\n");
		return -EOPNOTSUPP;
	}

	for (i = 0; i < ARRAY_SIZE(prism_fw_pda_addrs); i++) {
		unsigned int end_word;

		memset(pda->buf, 0, sizeof(pda->buf));
		err = hw->ops->read_pda(hw, pda->buf, prism_fw_pda_addrs[i],
					 sizeof(pda->buf));
		if (err) {
			dev_info(dev,
				 "prism_download: PDA read at 0x%06x failed: %d\n",
				 prism_fw_pda_addrs[i], err);
			continue;
		}

		err = prism_fw_pda_validate(&pda->buf[PRISM_FW_PDA_REAL_OFFSET_WORDS],
					    PRISM_FW_PDA_BUF_WORDS -
					    PRISM_FW_PDA_REAL_OFFSET_WORDS,
					    &end_word);
		if (err) {
			dev_info(dev,
				 "prism_download: PDA read at 0x%06x did not validate (bad structure or CRC-16)\n",
				 prism_fw_pda_addrs[i]);
			continue;
		}

		pda->addr = prism_fw_pda_addrs[i];
		pda->end_word = end_word;
		dev_info(dev,
			 "prism_download: PDA found at 0x%06x (%u words, CRC-16 clean)\n",
			 pda->addr, end_word);
		return 0;
	}

	dev_err(dev,
		"prism_download: no candidate PDA base address (0x7f0000/0x3f0000/0x390000/0x7f0002) validated -- cannot safely plug\n");
	return -ENODEV;
}

/* Linear search: the PDA has at most PRISM_FW_PDA_RECS_MAX records.
 * @out_len is in bytes.
 */
static bool prism_fw_pda_lookup(const struct prism_fw_pda *pda, u32 itemcode,
				const u8 **out_data, unsigned int *out_len)
{
	const __le16 *real = &pda->buf[PRISM_FW_PDA_REAL_OFFSET_WORDS];
	unsigned int pos = 0;
	u16 rlen, code;

	while (pos < pda->end_word) {
		rlen = le16_to_cpu(real[pos]);
		code = le16_to_cpu(real[pos + 1]);

		if (code == itemcode) {
			*out_data = (const u8 *)&real[pos + 2];
			*out_len = (unsigned int)(rlen - 1) * 2;
			return true;
		}

		pos += rlen + 1;
	}

	return false;
}

/* Merge chunks that are contiguous both in card address and in img->blob.
 * Run before any plug is applied: prism_fw_find_mutable_span_in_blob()
 * needs a single chunk to contain a plug's whole target span, and plug
 * targets routinely cross S3 line boundaries.
 */
static void prism_fw_coalesce_chunks(struct prism_fw_image *img)
{
	unsigned int i, n = 0;

	for (i = 0; i < img->n_chunks; i++) {
		struct prism_fw_chunk *c = &img->chunks[i];

		if (n && img->chunks[n - 1].addr + img->chunks[n - 1].len == c->addr &&
		    img->chunks[n - 1].data + img->chunks[n - 1].len == c->data) {
			img->chunks[n - 1].len += c->len;
			continue;
		}

		if (n != i)
			img->chunks[n] = *c;
		n++;
	}

	img->n_chunks = n;
}

/* Only valid before prism_fw_apply_crc() runs: until then every chunk
 * points into img->blob, which the offset arithmetic below relies on.
 */
static u8 *prism_fw_find_mutable_span_in_blob(struct prism_fw_image *img,
					      u32 addr, u32 len)
{
	unsigned int i;

	for (i = 0; i < img->n_chunks; i++) {
		struct prism_fw_chunk *c = &img->chunks[i];
		u64 cend = (u64)c->addr + c->len;

		if (addr >= c->addr && (u64)addr + len <= cend) {
			size_t blob_off = (size_t)(c->data - img->blob);

			return img->blob + blob_off + (addr - c->addr);
		}
	}
	return NULL;
}

/* Fill the image's plug records from the PDA. A PDR with no plug
 * record, or a plug record with no matching PDR, leaves the image's own
 * default in place.
 */
static void prism_fw_apply_plugs(struct device *dev, struct prism_fw_image *img,
				 const struct prism_fw_pda *pda)
{
	unsigned int i;

	for (i = 0; i < img->n_plug; i++) {
		struct prism_fw_plug *pl = &img->plugs[i];
		const u8 *pda_data = NULL;
		unsigned int pda_len = 0;
		u8 *dest;

		dest = prism_fw_find_mutable_span_in_blob(img, pl->addr, pl->len);
		if (!dest) {
			dev_warn(dev,
				 "prism_download: no image chunk contains plug 0x%08x @ 0x%08x len=%u, skipping\n",
				 pl->itemcode, pl->addr, pl->len);
			continue;
		}

		if (pl->itemcode == 0xffffffffu) {
			/* Not a PDR: write the image's filename, ASCIIZ,
			 * truncated to len bytes
			 */
			static const char fwname[] = "prism_sta_fw.hex";

			memset(dest, 0, pl->len);
			strscpy((char *)dest, fwname, pl->len);
			dev_info(dev, "prism_download: plugged filename @ 0x%08x len=%u\n",
				 pl->addr, pl->len);
			continue;
		}

		if (!prism_fw_pda_lookup(pda, pl->itemcode, &pda_data, &pda_len)) {
			dev_warn(dev,
				 "prism_download: no PDR 0x%04x on this card's PDA, leaving image default @ 0x%08x\n",
				 pl->itemcode, pl->addr);
			continue;
		}

		/* The PDR and the plug must have exactly the same length */
		if (pda_len != pl->len) {
			dev_warn(dev,
				 "prism_download: PDR 0x%04x length mismatch (card has %u bytes, image wants %u), leaving image default @ 0x%08x\n",
				 pl->itemcode, pda_len, pl->len, pl->addr);
			continue;
		}

		memcpy(dest, pda_data, pl->len);
		dev_info(dev,
			 "prism_download: plugged PDR 0x%04x @ 0x%08x len=%u from the card's own PDA\n",
			 pl->itemcode, pl->addr, pl->len);
	}
}

/* Stamp the 0xC0DE signature the Primary firmware checks. Unlike a plug,
 * a CRC record with nowhere to go aborts the download. Must run after
 * prism_fw_apply_plugs(), see prism_fw_find_mutable_span_in_blob().
 */
static int prism_fw_apply_crc(struct device *dev, struct prism_fw_image *img)
{
	unsigned int i;

	for (i = 0; i < img->n_crc; i++) {
		struct prism_fw_crc *cr = &img->crcs[i];
		unsigned int c;
		bool done = false;

		if (!cr->dowrite)
			continue;

		/* An existing chunk has two bytes of headroom before cr->addr */
		for (c = 0; c < img->n_chunks; c++) {
			struct prism_fw_chunk *ch = &img->chunks[c];
			u64 cend = (u64)ch->addr + ch->len;

			if (cr->addr - 2 >= ch->addr && cr->addr < cend) {
				size_t blob_off = (size_t)(ch->data - img->blob);
				u8 *dest = img->blob + blob_off +
					   (cr->addr - 2 - ch->addr);

				dest[0] = 0xde;
				dest[1] = 0xc0;
				dev_info(dev,
					 "prism_download: CRC signature stamped @ 0x%08x (existing chunk headroom)\n",
					 cr->addr - 2);
				done = true;
				break;
			}
		}
		if (done)
			continue;

		/* A chunk starts exactly at cr->addr: add a 2-byte lead-in
		 * chunk in front of it.
		 */
		for (c = 0; c < img->n_chunks; c++) {
			struct prism_fw_chunk *ch = &img->chunks[c];

			if (ch->addr == cr->addr) {
				struct prism_fw_chunk *stamp;
				u8 *sbuf;

				if (img->n_chunks >= img->n_chunks_alloc) {
					dev_err(dev,
						"prism_download: no spare chunk slot for CRC stamp @ 0x%08x, aborting\n",
						cr->addr - 2);
					return -EINVAL;
				}

				sbuf = &img->crc_stamp_buf[img->n_crc_stamps_used * 2];
				img->n_crc_stamps_used++;
				sbuf[0] = 0xde;
				sbuf[1] = 0xc0;

				stamp = &img->chunks[img->n_chunks++];
				stamp->addr = cr->addr - 2;
				stamp->len = 2;
				stamp->data = sbuf;

				dev_info(dev,
					 "prism_download: CRC signature stamped @ 0x%08x (new lead-in chunk)\n",
					 cr->addr - 2);
				done = true;
				break;
			}
		}
		if (!done) {
			dev_err(dev,
				"prism_download: failed to find or create a chunk for CRC record @ 0x%08x, aborting\n",
				cr->addr);
			return -EINVAL;
		}
	}

	return 0;
}

struct prism_fw_run {
	u32 addr;
	u32 len;
	const u8 *data;
};

/* Merge chunks that are contiguous in both address and data into fewer,
 * larger hw->ops->program() calls. @runs must hold at least img->n_chunks
 * entries.
 */
static unsigned int prism_fw_coalesce(const struct prism_fw_image *img,
				      struct prism_fw_run *runs)
{
	unsigned int i, n = 0;

	for (i = 0; i < img->n_chunks; i++) {
		const struct prism_fw_chunk *c = &img->chunks[i];

		if (n && runs[n - 1].addr + runs[n - 1].len == c->addr &&
		    runs[n - 1].data + runs[n - 1].len == c->data) {
			runs[n - 1].len += c->len;
			continue;
		}

		runs[n].addr = c->addr;
		runs[n].len = c->len;
		runs[n].data = c->data;
		n++;
	}

	return n;
}

static int prism_fw_program(struct hermes *hw, const struct prism_fw_run *runs,
			    unsigned int n_runs)
{
	unsigned int i;
	int err;

	for (i = 0; i < n_runs; i++) {
		/* PIO without acknowledgment: a bad write only shows up
		 * later, in program_end() or the post-download reinit.
		 */
		err = hw->ops->program(hw, (const char *)runs[i].data,
				       runs[i].addr, runs[i].len);
		if (err)
			return err;
	}
	return 0;
}

/* Checks that hold for any image, primary or station, before the card is
 * touched: the card is an Intersil one, the file has its start-address
 * record, it is the kind of image the caller expects (INFO component id)
 * and, if it carries a NICID list, that list names this card's hardware
 * identity. All of these refuse on a positive mismatch only; an image
 * without the record is let through with a warning, since the operator
 * has already chosen to write to prism_download.
 */
static bool prism_fw_check_image(struct orinoco_private *priv,
				 const struct prism_fw_image *img,
				 u16 expect_component, const char *what)
{
	struct device *dev = priv->dev;
	unsigned int i;

	if (priv->firmware_type != FIRMWARE_TYPE_INTERSIL) {
		dev_err(dev, "prism_download: not an Intersil/Prism card, refusing\n");
		return false;
	}

	if (!img->have_entry_point) {
		dev_err(dev, "prism_download: %s image has no S7 start-address record, refusing\n",
			what);
		return false;
	}

	if (!img->have_component) {
		dev_warn(dev, "prism_download: %s image carries no component record; cannot confirm what kind of image it is\n",
			 what);
	} else if (img->component_id != expect_component) {
		dev_err(dev, "prism_download: %s image declares component 0x%04x, expected 0x%04x, refusing\n",
			what, img->component_id, expect_component);
		return false;
	}

	if (img->n_nicid && priv->has_hw_nicid) {
		for (i = 0; i < img->n_nicid; i++)
			if (img->nicid[i] == priv->hw_nicid)
				return true;
		dev_err(dev, "prism_download: %s image does not list this card's hardware identity 0x%04x, refusing\n",
			what, priv->hw_nicid);
		return false;
	}

	return true;
}

enum prism_pri_check {
	PRISM_PRI_OK,
	PRISM_PRI_NEED_NEWER,
};

/* Does the Primary firmware now running on the card supply the PRI
 * interface range the Station image needs? The two ranges must overlap;
 * a Station image started on a Primary that does not supply its range
 * leaves the card unresponsive. Unknown on either side is a warning, not
 * a refusal.
 */
static enum prism_pri_check prism_fw_check_station_pri(struct orinoco_private *priv,
						       const struct prism_fw_image *sta)
{
	struct device *dev = priv->dev;

	if (!sta->have_req_pri) {
		dev_warn(dev, "prism_download: station image declares no PRI interface requirement; proceeding without a compatibility check\n");
		return PRISM_PRI_OK;
	}
	if (!priv->has_pri_range) {
		dev_warn(dev, "prism_download: the card's PRI supplier range could not be read; proceeding without a compatibility check\n");
		return PRISM_PRI_OK;
	}

	if (sta->req_pri_bottom > priv->pri_range_top ||
	    sta->req_pri_top < priv->pri_range_bottom) {
		dev_info(dev, "prism_download: station image needs PRI interface [%u,%u], the card's Primary supplies [%u,%u]\n",
			 sta->req_pri_bottom, sta->req_pri_top,
			 priv->pri_range_bottom, priv->pri_range_top);
		return PRISM_PRI_NEED_NEWER;
	}

	dev_info(dev, "prism_download: card's Primary supplies PRI [%u,%u], station image needs [%u,%u]: compatible\n",
		 priv->pri_range_bottom, priv->pri_range_top,
		 sta->req_pri_bottom, sta->req_pri_top);
	return PRISM_PRI_OK;
}

/* A RAM Primary is only worth starting if it supplies what the Station
 * image needs, and it must say what it supplies.
 */
static bool prism_fw_check_primary_supplies(struct device *dev,
					    const struct prism_fw_image *pri,
					     const struct prism_fw_image *sta)
{
	if (!pri->have_sup_pri) {
		dev_err(dev, "prism_download: primary image declares no PRI supplier range, refusing\n");
		return false;
	}

	if (pri->sup_pri_bottom > sta->req_pri_top ||
	    pri->sup_pri_top < sta->req_pri_bottom) {
		dev_err(dev, "prism_download: primary image supplies PRI [%u,%u] but the station image needs [%u,%u], refusing\n",
			pri->sup_pri_bottom, pri->sup_pri_top,
			sta->req_pri_bottom, sta->req_pri_top);
		return false;
	}

	dev_info(dev, "prism_download: primary image supplies PRI [%u,%u], station image needs [%u,%u]: compatible\n",
		 pri->sup_pri_bottom, pri->sup_pri_top,
		 sta->req_pri_bottom, sta->req_pri_top);
	return true;
}

/* Read, parse and coalesce one image. Nothing here touches the card. */
static int prism_fw_load(struct device *dev, const char *name,
			 struct prism_fw_image *img)
{
	const struct firmware *fw;
	int err;

	dev_info(dev, "prism_download: requesting firmware \"%s\"\n", name);
	err = request_firmware(&fw, name, dev);
	if (err) {
		dev_err(dev, "prism_download: request_firmware(\"%s\") failed: %d\n",
			name, err);
		return err;
	}

	err = prism_fw_parse(fw, img);
	release_firmware(fw);
	if (err) {
		dev_err(dev, "prism_download: failed to parse %s: %d\n", name, err);
		return err;
	}

	dev_info(dev,
		 "prism_download: parsed %u load record(s) (%zu bytes), %u PDA-plug, %u CRC, %u INFO sentinel(s), entry point 0x%08x\n",
		 img->n_chunks, img->blob_len, img->n_plug, img->n_crc,
		 img->n_info, img->entry_point);

	/* Must run before any plug is applied */
	prism_fw_coalesce_chunks(img);

	return 0;
}

/* Everything short of touching program mode: reinit, PDA plug, CRC stamp,
 * run-building. Allocates *runs (kvfree() by the caller). The caller has
 * quiesced the interface (orinoco_down()) and re-arms it afterwards
 * (orinoco_up()).
 */
static int prism_fw_prepare_image(struct orinoco_private *priv,
				  struct prism_fw_image *img, const char *what,
				  struct prism_fw_run **runs_out,
				  unsigned int *n_runs_out)
{
	struct device *dev = priv->dev;
	struct hermes *hw = &priv->hw;
	struct prism_fw_run *runs;
	unsigned int n_runs;
	int err;

	/* A full reinit immediately before program-mode entry:
	 * hermesi_program_init() only issues a bare INIT command.
	 */
	dev_info(dev, "prism_download: reinitialising hardware before %s program mode\n",
		 what);
	err = hw->ops->init(hw);
	if (err) {
		dev_err(dev, "prism_download: pre-download reinit failed: %d\n", err);
		return err;
	}

	/* The PDA is read after init() and before program mode. An image
	 * with plug records and no readable, CRC-16-valid PDA to plug from
	 * is refused rather than downloaded miscalibrated.
	 */
	if (img->n_plug) {
		struct prism_fw_pda *pda;

		pda = kzalloc_obj(*pda, GFP_KERNEL);
		if (!pda)
			return -ENOMEM;

		err = prism_fw_read_pda(hw, dev, pda);
		if (err) {
			dev_err(dev,
				"prism_download: image declares %u PDA-plug record(s) and no valid PDA could be read off this card -- refusing to risk a mis-personalized download\n",
				img->n_plug);
			kfree(pda);
			return err;
		}

		prism_fw_apply_plugs(dev, img, pda);
		kfree(pda);
	}

	/* May append lead-in chunks, so it precedes run building */
	if (img->n_crc) {
		err = prism_fw_apply_crc(dev, img);
		if (err)
			return err;
	}

	runs = kvcalloc(img->n_chunks, sizeof(*runs), GFP_KERNEL);
	if (!runs)
		return -ENOMEM;
	n_runs = prism_fw_coalesce(img, runs);
	dev_info(dev, "prism_download: coalesced into %u AUX-port run(s)\n", n_runs);

	*runs_out = runs;
	*n_runs_out = n_runs;
	return 0;
}

/* Put one parsed image on the card and start it through the volatile
 * program-mode command: prepare, program, jump, reinit. This is the path
 * for a Station image; a RAM Primary goes through genesis mode instead.
 */
static int prism_fw_run_image(struct orinoco_private *priv,
			      struct prism_fw_image *img, const char *what)
{
	struct device *dev = priv->dev;
	struct hermes *hw = &priv->hw;
	struct prism_fw_run *runs = NULL;
	unsigned int n_runs;
	int err, err2;

	err = prism_fw_prepare_image(priv, img, what, &runs, &n_runs);
	if (err)
		return err;

	dev_info(dev, "prism_download: entering program mode, entry point 0x%08x\n",
		 img->entry_point);
	err = hw->ops->program_init(hw, img->entry_point);
	if (err) {
		dev_err(dev, "prism_download: program_init failed: %d\n", err);
		goto out;
	}

	dev_info(dev, "Downloading Intersil %s firmware (%zu bytes)\n",
		 what, img->blob_len);
	err = prism_fw_program(hw, runs, n_runs);
	if (err)
		dev_err(dev, "prism_download: program() failed: %d\n", err);

	dev_info(dev, "prism_download: leaving program mode\n");
	err2 = hw->ops->program_end(hw);
	if (err2)
		dev_err(dev,
			"prism_download: program_end returned %d (may be a benign post-jump completion timeout)\n",
			err2);
	if (!err)
		err = err2;
	if (err)
		goto out;

	/* A plain HERMES_CMD_INIT initialises whichever firmware is active
	 * and does not revert the RAM image to the flash one; only a
	 * hardware-level reset does that.
	 */
	err = hw->ops->init(hw);
	if (err)
		dev_err(dev, "prism_download: post-download reinit failed: %d\n", err);

out:
	kvfree(runs);
	return err;
}

/* Load a RAM Primary in genesis mode. A Primary is not started with the
 * volatile program-mode command: the card's CPU is halted by writing a
 * hardware-control value (HCR) into the CCSR attribute register while COR
 * holds the card in soft reset, the SRAM is written through the AUX port,
 * and the card is let go again. The HCR
 * value depends on the SRAM width (0x1f for x8, 0x0f for x16); a pattern
 * written to AUX address 0x7e0038 and read back proves which one works.
 * Afterwards the card runs only the Primary, so the Station image must be
 * downloaded next and the caller does not re-read the Station identity.
 */
static int prism_fw_genesis_probe(struct orinoco_private *priv, u8 hcr)
{
	static const u8 initseq[4] = { 0x00, 0xe1, 0xa1, 0xff };
	struct hermes *hw = &priv->hw;
	u8 readbuf[4];

	priv->cor_sreset(priv);
	hw->ops->program(hw, (const char *)initseq, 0x7e0038, sizeof(initseq));
	priv->genesis_reset(priv, hcr);

	/* Readback test: read, write, read again */
	hw->ops->aux_read(hw, 0x7e0038, readbuf, sizeof(readbuf));
	hw->ops->program(hw, (const char *)initseq, 0x7e0038, sizeof(initseq));
	hw->ops->aux_read(hw, 0x7e0038, readbuf, sizeof(readbuf));

	if (!memcmp(initseq, readbuf, sizeof(initseq)))
		return 0;

	dev_info(priv->dev, "prism_download: genesis readback failed with HCR 0x%02x (read %4ph)\n",
		 hcr, readbuf);
	return -EIO;
}

static int prism_fw_run_primary_genesis(struct orinoco_private *priv,
					struct prism_fw_image *img)
{
	struct device *dev = priv->dev;
	struct hermes *hw = &priv->hw;
	struct prism_fw_run *runs = NULL;
	unsigned int n_runs, i;
	bool ram16 = false;
	int err;

	if (!priv->cor_sreset || !priv->genesis_reset || !hw->ops->aux_enable ||
	    !hw->ops->aux_read) {
		dev_err(dev, "prism_download: this bus cannot load a Primary in genesis mode\n");
		return -EOPNOTSUPP;
	}

	err = prism_fw_prepare_image(priv, img, "primary", &runs, &n_runs);
	if (err)
		return err;

	err = hw->ops->aux_enable(hw, 1);
	if (err) {
		dev_err(dev, "prism_download: cannot open the AUX port: %d\n", err);
		goto out;
	}

	if (prism_fw_genesis_probe(priv, 0x1f) == 0) {
		dev_info(dev, "prism_download: genesis mode with x8 SRAM (HCR 0x1f)\n");
	} else if (prism_fw_genesis_probe(priv, 0x0f) == 0) {
		ram16 = true;
		dev_info(dev, "prism_download: genesis mode with x16 SRAM (HCR 0x0f)\n");
	} else {
		dev_err(dev, "prism_download: could not initiate genesis mode\n");
		priv->genesis_reset(priv, 0x17);
		hw->ops->aux_enable(hw, 0);
		err = -EIO;
		goto out;
	}

	dev_info(dev, "Downloading Intersil primary firmware (%zu bytes) in genesis mode\n",
		 img->blob_len);
	for (i = 0; i < n_runs; i++) {
		err = hw->ops->program(hw, (const char *)runs[i].data,
				       runs[i].addr, runs[i].len);
		if (err)
			break;
	}

	dev_info(dev, "prism_download: leaving genesis mode\n");
	priv->genesis_reset(priv, ram16 ? 0x07 : 0x17);
	hw->ops->aux_enable(hw, 0);
	if (err)
		goto out;

	fsleep(5000);

	/* The INIT must not raise a command-completion event the driver
	 * would take for a stray interrupt.
	 */
	hermes_set_irqmask(hw, 0);
	err = hw->ops->init(hw);
	if (err)
		dev_err(dev, "prism_download: reinit after genesis mode failed: %d\n", err);
	else
		dev_info(dev, "prism_download: card initialised, running the Primary only\n");

out:
	kvfree(runs);
	return err;
}

/* One trigger, one or two images. The Station image says which PRI
 * interface range it needs; if the Primary the card is running does not
 * supply it, a RAM Primary that does is downloaded first
 * (PRISM_FW_PRI_NAME) and the Station image after it. A Station image the
 * card's Primary already satisfies is downloaded alone, as before. An
 * incompatible pairing with no Primary available is refused before the
 * card is touched.
 */
static int orinoco_download_intersil(struct orinoco_private *priv)
{
	struct device *dev = priv->dev;
	struct wiphy *wiphy = priv_to_wiphy(priv);
	struct prism_fw_image sta = {}, pri = {};
	bool have_pri = false;
	bool had_wpa;
	int err;

	if (priv->firmware_type != FIRMWARE_TYPE_INTERSIL)
		return -EOPNOTSUPP;

	if (!mutex_trylock(&priv->prism_fw_lock))
		return -EBUSY;

	err = prism_fw_load(dev, PRISM_FW_NAME, &sta);
	if (err)
		goto out_unlock;

	if (!prism_fw_check_image(priv, &sta, PRISM_FW_COMPONENT_STA, "station")) {
		err = -EINVAL;
		goto out_free;
	}

	if (prism_fw_check_station_pri(priv, &sta) == PRISM_PRI_NEED_NEWER) {
		dev_info(dev, "prism_download: a RAM Primary is needed first; requesting \"%s\"\n",
			 PRISM_FW_PRI_NAME);
		err = prism_fw_load(dev, PRISM_FW_PRI_NAME, &pri);
		if (err) {
			dev_err(dev, "prism_download: refusing: the card's Primary supplies PRI [%u,%u] and the station image needs [%u,%u]; provide a RAM Primary that supplies it as /lib/firmware/%s\n",
				priv->pri_range_bottom, priv->pri_range_top,
				sta.req_pri_bottom, sta.req_pri_top,
				PRISM_FW_PRI_NAME);
			goto out_free;
		}
		have_pri = true;

		if (!prism_fw_check_image(priv, &pri, PRISM_FW_COMPONENT_PRI, "primary") ||
		    !prism_fw_check_primary_supplies(dev, &pri, &sta)) {
			err = -EINVAL;
			goto out_free;
		}
	}

	had_wpa = priv->has_wpa;

	/* Everything below touches live hardware. orinoco_down() quiesces
	 * the interface exactly like PCMCIA suspend does: stops the queue,
	 * disables the MAC port, masks interrupts, and increments
	 * hw_unavailable so every other entry point (IRQ handler,
	 * orinoco_xmit, the cfg80211 ops) bails out with -EBUSY via
	 * orinoco_lock() instead of racing this sequence.
	 */
	dev_info(dev, "prism_download: quiescing interface before firmware download\n");
	orinoco_down(priv);

	if (have_pri) {
		err = prism_fw_run_primary_genesis(priv, &pri);
		if (err)
			goto recover;
	}

	err = prism_fw_run_image(priv, &sta, "station");
	if (err)
		goto recover;
	priv->fw_ram = 1;

	/* Re-probe identity/capabilities while the port is still disabled
	 * and hw_unavailable is still elevated -- this MUST happen before
	 * orinoco_up() re-arms the port, or these RID reads over USER_BAP
	 * would race live orinoco_xmit()/orinoco_hw_program_rids() traffic
	 * on the same BAP.
	 */
	err = determine_fw_capabilities(priv, wiphy->fw_version,
					sizeof(wiphy->fw_version), &wiphy->hw_version);
	if (err)
		dev_err(dev, "prism_download: post-download identity re-read failed: %d\n",
			err);
	else if (!had_wpa && priv->has_wpa)
		dev_info(dev, "WPA-PSK supported\n");

recover:
	/* orinoco_up() re-arms hw_unavailable -- and, if the interface was
	 * open, the port/queue -- unconditionally, success or failure,
	 * exactly like orinoco_reset()'s own disable/reinit pattern.
	 */
	orinoco_up(priv);

out_free:
	prism_fw_free(&sta);
	prism_fw_free(&pri);
out_unlock:
	mutex_unlock(&priv->prism_fw_lock);
	return err;
}

static ssize_t prism_download_store(struct device *dev,
				    struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct net_device *ndev = to_net_dev(dev);
	struct orinoco_private *priv = ndev_priv(ndev);
	int err;

	/* No parameters: which image to try is chosen by which file the
	 * operator has copied/symlinked to PRISM_FW_NAME under
	 * /lib/firmware, not by what is written here. This attribute is a
	 * bare trigger.
	 */
	if (!sysfs_streq(buf, "1"))
		return -EINVAL;

	err = orinoco_download_intersil(priv);

	return err ? err : count;
}
static DEVICE_ATTR_WO(prism_download);

static struct attribute *orinoco_prism_fw_attrs[] = {
	&dev_attr_prism_download.attr,
	NULL,
};

static const struct attribute_group orinoco_prism_fw_attr_group = {
	.attrs = orinoco_prism_fw_attrs,
};

void orinoco_prism_fw_sysfs_init(struct net_device *dev)
{
	dev->sysfs_groups[0] = &orinoco_prism_fw_attr_group;
}
