// SPDX-License-Identifier: GPL-2.0-only
/*
 * JPEG framing helpers for the HP Photosmart Mobile / Veo SDIO camera.
 *
 * Kept free of kernel APIs other than <linux/types.h> and <linux/string.h> so
 * that the header builder and the validator can be checked against stored
 * captures outside the kernel with the very same source.
 */

#include <linux/string.h>
#include <linux/types.h>

#include "veo-sdiocam-jpeg.h"

/* Reads whose start went missing are common, so damage is the normal case. */
#define VEO_WALK_TRUNCATED	(-1)
#define VEO_WALK_INVALID	(-2)

/* Stray 0x00 of the factory header (offset 155) is left out here. */
static const u8 veo_sos[] = {
	0xff, 0xda, 0x00, 0x0c, 0x03, 0x01, 0x00, 0x02, 0x11, 0x03, 0x11, 0x00,
	0x3f, 0x00,
};

size_t veo_jpeg_build_header(u8 *buf, size_t size, u16 width, u16 height,
			     u8 sampling, unsigned int dqt_idx, bool with_dht)
{
	size_t need = VEO_JPEG_HDR_NO_DHT + (with_dht ? VEO_JPEG_DHT_LEN : 0);
	u8 *p = buf;

	if (!buf || size < need || dqt_idx >= VEO_QUALITY_TABLES ||
	    !width || !height ||
	    (sampling != VEO_JPEG_SAMPLING_444 &&
	     sampling != VEO_JPEG_SAMPLING_420))
		return 0;

	*p++ = 0xff;
	*p++ = 0xd8;

	*p++ = 0xff;
	*p++ = 0xdb;
	*p++ = 0x00;
	*p++ = 2 + VEO_JPEG_DQT_BODY;
	memcpy(p, veo_dqt[dqt_idx], VEO_JPEG_DQT_BODY);
	p += VEO_JPEG_DQT_BODY;

	*p++ = 0xff;
	*p++ = 0xc0;
	*p++ = 0x00;
	*p++ = 17;
	*p++ = 8;
	*p++ = height >> 8;
	*p++ = height & 0xff;
	*p++ = width >> 8;
	*p++ = width & 0xff;
	*p++ = 3;
	*p++ = 1;
	*p++ = sampling;
	*p++ = 0;
	*p++ = 2;
	*p++ = 0x11;
	*p++ = 1;
	*p++ = 3;
	*p++ = 0x11;
	*p++ = 1;

	if (with_dht) {
		*p++ = 0xff;
		*p++ = 0xc4;
		*p++ = (2 + VEO_JPEG_DHT_BODY) >> 8;
		*p++ = (2 + VEO_JPEG_DHT_BODY) & 0xff;
		memcpy(p, veo_dht, VEO_JPEG_DHT_BODY);
		p += VEO_JPEG_DHT_BODY;
	}

	memcpy(p, veo_sos, sizeof(veo_sos));
	p += sizeof(veo_sos);

	return p - buf;
}

struct veo_huff {
	s32 maxcode[17];
	s32 mincode[17];
	u16 valptr[17];
	const u8 *vals;
};

struct veo_bits {
	const u8 *p;
	u32 pos;
	u32 end;
	u32 acc;
	u32 n;
};

/* veo_dht holds DC luma, AC luma, DC chroma, AC chroma, in that order. */
static void veo_huff_init(struct veo_huff tab[4])
{
	const u8 *p = veo_dht;
	unsigned int t, l;

	for (t = 0; t < 4; t++) {
		const u8 *counts = p + 1;
		s32 code = 0;
		u16 k = 0;

		for (l = 1; l <= 16; l++) {
			u8 c = counts[l - 1];

			tab[t].valptr[l] = k;
			tab[t].mincode[l] = code;
			tab[t].maxcode[l] = c ? code + c - 1 : -1;
			code = (code + c) << 1;
			k += c;
		}
		tab[t].vals = p + 17;
		p += 17 + k;
	}
}

/* Byte stuffing is undone here; the caller sets @end at the first marker. */
static int veo_bits_get(struct veo_bits *b, unsigned int k, u32 *val)
{
	while (b->n < k) {
		if (b->pos >= b->end)
			return VEO_WALK_TRUNCATED;
		b->acc = (b->acc << 8) | b->p[b->pos];
		b->pos += b->p[b->pos] == 0xff ? 2 : 1;
		b->n += 8;
	}
	*val = (b->acc >> (b->n - k)) & ((1u << k) - 1);
	b->n -= k;
	return 0;
}

static int veo_huff_decode(struct veo_bits *b, const struct veo_huff *h,
			   u8 *sym)
{
	s32 code = 0;
	unsigned int l;
	u32 bit;

	for (l = 1; l <= 16; l++) {
		if (veo_bits_get(b, 1, &bit))
			return VEO_WALK_TRUNCATED;
		code = (code << 1) | bit;
		if (code <= h->maxcode[l]) {
			*sym = h->vals[h->valptr[l] + code - h->mincode[l]];
			return 0;
		}
	}
	return VEO_WALK_INVALID;
}

static int veo_walk_block(struct veo_bits *b, const struct veo_huff *dc,
			  const struct veo_huff *ac)
{
	unsigned int k = 1;
	u8 s, r;
	u32 v;
	int err;

	err = veo_huff_decode(b, dc, &s);
	if (err)
		return err;
	if (s > 11)
		return VEO_WALK_INVALID;
	if (s && veo_bits_get(b, s, &v))
		return VEO_WALK_TRUNCATED;

	while (k < 64) {
		err = veo_huff_decode(b, ac, &s);
		if (err)
			return err;
		r = s >> 4;
		s &= 15;
		if (!s) {
			if (r != 15)
				return 0;
			k += 16;
			continue;
		}
		k += r;
		if (k > 63)
			return VEO_WALK_INVALID;
		if (veo_bits_get(b, s, &v))
			return VEO_WALK_TRUNCATED;
		k++;
	}
	return k > 64 ? VEO_WALK_INVALID : 0;
}

/*
 * Walk the MCUs of @p[0..@end) and report whether the walk ends exactly at
 * @end with only 1-bits of padding left over. An MCU is at least 14 bits, so
 * fewer than 8 bits left can only be padding.
 */
static bool veo_walk(const u8 *p, u32 end, bool c420, u32 *mcus)
{
	struct veo_huff tab[4];
	struct veo_bits b = { .p = p, .end = end };
	unsigned int nluma = c420 ? 4 : 1;
	unsigned int i;
	u32 count = 0;

	veo_huff_init(tab);

	for (;;) {
		if (b.pos >= b.end) {
			u32 mask = (1u << b.n) - 1;

			*mcus = count;
			return (b.acc & mask) == mask;
		}
		for (i = 0; i < nluma; i++)
			if (veo_walk_block(&b, &tab[0], &tab[1]))
				goto stop;
		if (veo_walk_block(&b, &tab[2], &tab[3]) ||
		    veo_walk_block(&b, &tab[2], &tab[3]))
			goto stop;
		count++;
	}
stop:
	*mcus = count;
	return false;
}

static enum veo_jpeg_status veo_check_at(const u8 *buf, size_t avail,
					 u32 start, u32 len_reg, bool c420,
					 struct veo_jpeg_verdict *v)
{
	u32 limit = start + len_reg;
	u32 i, eoi;
	bool clean;

	v->start = start;
	v->eoi_off = -1;
	v->short_by = 0;
	v->mcus = 0;
	v->scan_len = 0;

	if (limit > avail)
		limit = avail;

	for (i = start; i + 1 < limit; i++) {
		if (buf[i] != 0xff)
			continue;
		if (buf[i + 1] == 0x00) {
			i++;
			continue;
		}
		break;
	}
	if (i + 1 >= limit) {
		v->status = VEO_JPEG_NOEOI;
		return v->status;
	}

	eoi = i - start;
	v->eoi_off = eoi;
	if (buf[i + 1] != 0xd9) {
		v->status = VEO_JPEG_BAD_MARKER;
		return v->status;
	}
	v->scan_len = eoi + 2;

	clean = veo_walk(buf + start, eoi, c420, &v->mcus);

	if (eoi + 2 != len_reg && eoi + 3 != len_reg) {
		v->short_by = len_reg - 2 - eoi;
		v->status = VEO_JPEG_SHORT;
	} else if (!clean || v->mcus != v->expected) {
		v->status = VEO_JPEG_MCU_MISMATCH;
	} else {
		v->status = VEO_JPEG_OK;
	}
	return v->status;
}

/*
 * The data loss seen on hardware removes bytes without any bus error, so a
 * frame is accepted only when its EOI sits where the frame length register
 * puts it and a Huffman walk of the whole scan yields exactly the MCU count
 * of the frame size. With @preroll the read started @preroll bytes before
 * the frame; a start is then derived from each EOI candidate and accepted
 * only through the same full check, never by guessing.
 */
enum veo_jpeg_status veo_jpeg_check(const u8 *buf, size_t avail, u32 len_reg,
				    u16 width, u16 height, u8 sampling,
				    u32 preroll,
				    struct veo_jpeg_verdict *verdict)
{
	struct veo_jpeg_verdict t;
	bool c420 = sampling == VEO_JPEG_SAMPLING_420;
	u32 e, lo, hi, extra;
	u32 expected;

	if (!verdict)
		return VEO_JPEG_BAD_ARGS;
	memset(verdict, 0, sizeof(*verdict));
	verdict->eoi_off = -1;
	if (!buf || !width || !height || len_reg < 2 ||
	    (sampling != VEO_JPEG_SAMPLING_444 && !c420)) {
		verdict->status = VEO_JPEG_BAD_ARGS;
		return verdict->status;
	}

	if (c420)
		expected = ((width + 15) / 16) * ((height + 15) / 16);
	else
		expected = ((width + 7) / 8) * ((height + 7) / 8);
	verdict->expected = expected;

	if (!preroll)
		return veo_check_at(buf, avail, 0, len_reg, c420, verdict);

	lo = preroll > 14 ? preroll - 14 : 0;
	hi = preroll + len_reg;
	for (e = lo; e < hi && e + 1 < avail; e++) {
		if (buf[e] != 0xff || buf[e + 1] != 0xd9)
			continue;
		for (extra = 2; extra <= 3; extra++) {
			if (e + extra < len_reg)
				continue;
			t.expected = expected;
			if (veo_check_at(buf, avail, e + extra - len_reg,
					 len_reg, c420, &t) == VEO_JPEG_OK) {
				*verdict = t;
				return VEO_JPEG_OK;
			}
		}
	}

	return veo_check_at(buf, avail, preroll < avail ? preroll : 0,
			    len_reg, c420, verdict);
}
