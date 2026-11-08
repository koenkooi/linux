// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the veo-sdiocam JPEG framing helpers.
 *
 * The two scans below are synthetic: random-pixel 16x16 (4:4:4) and 32x16
 * (4:2:0) images encoded as baseline JPEG by Pillow with the bridge's
 * preview quantiser table (veo_dqt[2]) and the standard Annex K Huffman
 * tables (optimize=False), keeping the bytes after the SOS header up to and
 * including EOI. The 4:4:4 scan was picked to contain byte stuffing.
 */

#include <kunit/test.h>
#include <linux/math.h>
#include <linux/string.h>

#include "veo-sdiocam-jpeg.h"
#include "veo-sdiocam-tables.h"

/* 16x16 4:4:4, 216 bytes */
static const u8 veo_test_scan_444[] = {
	0xa7, 0x3d, 0xc4, 0x93, 0x28, 0x4c, 0x8c, 0x36, 0x11, 0x93, 0x82, 0xb8,
	0xff, 0x00, 0x66, 0x95, 0x1c, 0xbd, 0x3a, 0xd6, 0xa9, 0x07, 0x53, 0x93,
	0xed, 0xda, 0xd3, 0xb5, 0xac, 0xae, 0xec, 0xdf, 0x99, 0xd2, 0xe7, 0x4d,
	0x54, 0x70, 0x82, 0x8f, 0x32, 0x8c, 0x5b, 0xb4, 0x53, 0xd5, 0xb7, 0xa5,
	0xad, 0xf3, 0xd8, 0xbb, 0x0d, 0xf2, 0x6e, 0x28, 0x02, 0xb1, 0x8e, 0x31,
	0xb4, 0x81, 0xc9, 0x07, 0x8e, 0xf9, 0xe7, 0x34, 0xab, 0x7b, 0x7a, 0x91,
	0x75, 0x27, 0x3a, 0x94, 0xa3, 0xcc, 0xd4, 0xf9, 0xdb, 0x8a, 0xf7, 0x52,
	0xbb, 0xdd, 0x68, 0xf6, 0x39, 0xb1, 0xb8, 0x6a, 0x98, 0x7a, 0x92, 0xe7,
	0x8c, 0xa5, 0x75, 0x26, 0xe2, 0xfe, 0x14, 0xd4, 0x9c, 0xaf, 0x67, 0xf3,
	0xb9, 0x66, 0xf2, 0xe1, 0x9d, 0x94, 0x80, 0x36, 0x80, 0x48, 0x3d, 0x70,
	0x3a, 0x9f, 0xce, 0xba, 0x3d, 0x9c, 0x6a, 0x49, 0x3a, 0x76, 0xa9, 0x51,
	0x49, 0x73, 0x5a, 0xf1, 0xb4, 0xb6, 0x8d, 0xed, 0xb9, 0x8c, 0x69, 0x42,
	0x94, 0xea, 0x53, 0xf6, 0x93, 0x4a, 0xee, 0x29, 0xa9, 0x3b, 0x29, 0x2d,
	0x35, 0xe8, 0xb5, 0x46, 0x48, 0x99, 0x1e, 0x35, 0x4f, 0x2d, 0x82, 0xc6,
	0x37, 0x64, 0x71, 0x8e, 0xe4, 0xf3, 0xf8, 0x7d, 0x6b, 0x0a, 0xad, 0xca,
	0x72, 0x75, 0x62, 0xb9, 0xa6, 0xe4, 0xe2, 0xd4, 0x6e, 0xed, 0x2d, 0xaf,
	0x6f, 0x4b, 0x6e, 0x6b, 0x1a, 0xb2, 0xab, 0x83, 0xa9, 0x17, 0x1d, 0x27,
	0x78, 0xc6, 0xef, 0x4d, 0x2f, 0x76, 0x9f, 0x6f, 0xf3, 0x3f, 0xff, 0xd9,
};

/* 32x16 4:2:0, 181 bytes */
static const u8 veo_test_scan_420[] = {
	0x92, 0xe2, 0xd8, 0x10, 0x51, 0x9d, 0x8a, 0x01, 0x86, 0x53, 0xd3, 0x6f,
	0xf2, 0xc7, 0x3d, 0xaa, 0xab, 0x3b, 0x4e, 0x17, 0x6b, 0x29, 0x20, 0x63,
	0xe5, 0x1d, 0x33, 0xcf, 0x5f, 0xa1, 0xff, 0x00, 0xeb, 0xd5, 0xa8, 0x2f,
	0x51, 0x4c, 0x6e, 0xcd, 0xb8, 0x46, 0x99, 0xe7, 0xef, 0x0c, 0x75, 0xe7,
	0x23, 0x8c, 0xd5, 0x09, 0xc4, 0x52, 0xc8, 0xcb, 0xb5, 0x50, 0x44, 0x54,
	0xab, 0xa8, 0x0b, 0xc6, 0x2a, 0xa3, 0x18, 0xa7, 0xee, 0x7b, 0xbd, 0x5e,
	0x9d, 0x9e, 0xbe, 0x45, 0xfb, 0x0a, 0xaa, 0xf0, 0xe4, 0xbe, 0x96, 0x7d,
	0xd6, 0xbf, 0xe6, 0x4d, 0x75, 0x13, 0x66, 0x31, 0xe7, 0x34, 0x9b, 0xb3,
	0xf3, 0x01, 0x8e, 0x00, 0xcf, 0x27, 0xf0, 0xef, 0x45, 0xb6, 0xa3, 0x25,
	0xb8, 0xca, 0xb6, 0x06, 0xc1, 0xf2, 0xe0, 0xfc, 0xa0, 0x7b, 0xf3, 0xda,
	0x88, 0x1e, 0x46, 0x50, 0x21, 0x91, 0xd9, 0x98, 0x81, 0xbd, 0x4f, 0xbe,
	0x70, 0x07, 0x4f, 0xd6, 0xb4, 0xd1, 0xdc, 0x24, 0xad, 0x2b, 0x95, 0x45,
	0x6f, 0x95, 0xb0, 0x30, 0xb8, 0x19, 0xcf, 0xf9, 0xf6, 0xac, 0x67, 0x52,
	0x33, 0x52, 0xa5, 0xca, 0xa5, 0x1b, 0xfe, 0x6d, 0x79, 0x19, 0x57, 0xab,
	0x34, 0xd2, 0x93, 0xbd, 0xef, 0x2f, 0x7b, 0x47, 0xdb, 0x4e, 0xa7, 0xff,
	0xd9,
};

static const u8 *veo_find_stuffing(const u8 *p, size_t len)
{
	size_t i;

	for (i = 0; i + 1 < len; i++)
		if (p[i] == 0xff && p[i + 1] == 0x00)
			return p + i;
	return NULL;
}

static void veo_test_header_layout(struct kunit *test)
{
	u8 hdr[VEO_JPEG_HDR_MAX];
	size_t n;

	n = veo_jpeg_build_header(hdr, sizeof(hdr), 160, 120,
				  VEO_JPEG_SAMPLING_444, 2, true);
	KUNIT_ASSERT_EQ(test, n, (size_t)VEO_JPEG_HDR_MAX);
	KUNIT_EXPECT_EQ(test, hdr[0], 0xff);
	KUNIT_EXPECT_EQ(test, hdr[1], 0xd8);
	KUNIT_EXPECT_EQ(test, hdr[2], 0xff);
	KUNIT_EXPECT_EQ(test, hdr[3], 0xdb);
	KUNIT_EXPECT_EQ(test, hdr[5], 0x84);
	KUNIT_EXPECT_EQ(test, hdr[136], 0xff);
	KUNIT_EXPECT_EQ(test, hdr[137], 0xc0);
	KUNIT_EXPECT_EQ(test, hdr[141] << 8 | hdr[142], 120);
	KUNIT_EXPECT_EQ(test, hdr[143] << 8 | hdr[144], 160);
	KUNIT_EXPECT_EQ(test, hdr[147], VEO_JPEG_SAMPLING_444);
	KUNIT_EXPECT_EQ(test, hdr[155], 0xff);
	KUNIT_EXPECT_EQ(test, hdr[156], 0xc4);
	KUNIT_EXPECT_EQ(test, hdr[157] << 8 | hdr[158], 2 + VEO_JPEG_DHT_BODY);
	KUNIT_EXPECT_EQ(test, hdr[575], 0xff);
	KUNIT_EXPECT_EQ(test, hdr[576], 0xda);

	n = veo_jpeg_build_header(hdr, sizeof(hdr), 320, 240,
				  VEO_JPEG_SAMPLING_420, 1, false);
	KUNIT_ASSERT_EQ(test, n, (size_t)VEO_JPEG_HDR_NO_DHT);
	KUNIT_EXPECT_EQ(test, hdr[141] << 8 | hdr[142], 240);
	KUNIT_EXPECT_EQ(test, hdr[143] << 8 | hdr[144], 320);
	KUNIT_EXPECT_EQ(test, hdr[147], VEO_JPEG_SAMPLING_420);
	KUNIT_EXPECT_EQ(test, hdr[155], 0xff);
	KUNIT_EXPECT_EQ(test, hdr[156], 0xda);
	KUNIT_EXPECT_EQ(test, memcmp(hdr + 6, veo_dqt[1], VEO_JPEG_DQT_BODY), 0);

	KUNIT_EXPECT_EQ(test, veo_jpeg_build_header(hdr, VEO_JPEG_HDR_MAX - 1,
						    160, 120, 0x11, 2, true),
			(size_t)0);
	KUNIT_EXPECT_EQ(test, veo_jpeg_build_header(hdr, sizeof(hdr), 160, 120,
						    0x21, 2, true), (size_t)0);
	KUNIT_EXPECT_EQ(test, veo_jpeg_build_header(hdr, sizeof(hdr), 160, 120,
						    0x11, 3, true), (size_t)0);
}

/*
 * The preview DQT (index 2) is the one in the factory 160x120 header: DC 7,
 * then 7 7 13 13 13 17 16 16 17 32 in luma, and 255 for the highest
 * frequencies of both tables.
 */
static void veo_test_dqt_preview(struct kunit *test)
{
	static const u8 luma_head[] = { 0x00, 7, 7, 7, 13, 13, 13, 17, 16, 16, 17, 32 };
	const u8 *d = veo_dqt[2];

	KUNIT_EXPECT_EQ(test, memcmp(d, luma_head, sizeof(luma_head)), 0);
	KUNIT_EXPECT_EQ(test, d[64], 0xff);
	KUNIT_EXPECT_EQ(test, d[65], 0x01);
	KUNIT_EXPECT_EQ(test, d[129], 0xff);
}

/* Each chip byte (m << 3) | e encodes 64 / q ~= m / 2^e, within 3.1 %. */
static void veo_test_chip_reciprocal(struct kunit *test)
{
	unsigned int t, k;

	for (t = 0; t < VEO_QUALITY_TABLES; t++) {
		for (k = 0; k < VEO_CHIP_QUANT_LEN; k++) {
			u8 c = veo_chip_quant[t][k];
			unsigned int q = veo_dqt[t][k < 64 ? 1 + k : 2 + k];
			unsigned int m = c >> 3, e = c & 7;
			int got = m ? (64 << e) * 1000 / m : 0;
			int want = q * 1000;

			KUNIT_EXPECT_NE(test, m, 0U);
			KUNIT_EXPECT_LE_MSG(test, abs(got - want), want * 31 / 1000,
					    "table %u entry %u", t, k);
		}
	}
}

static void veo_test_scan_ok(struct kunit *test)
{
	struct veo_jpeg_verdict v;

	KUNIT_EXPECT_NOT_NULL(test, veo_find_stuffing(veo_test_scan_444,
						      sizeof(veo_test_scan_444)));
	veo_jpeg_check(veo_test_scan_444, sizeof(veo_test_scan_444),
		       sizeof(veo_test_scan_444), 16, 16, 0x11, 0, &v);
	KUNIT_EXPECT_EQ(test, v.status, VEO_JPEG_OK);
	KUNIT_EXPECT_EQ(test, v.mcus, 4U);
	KUNIT_EXPECT_EQ(test, v.start, 0U);
	KUNIT_EXPECT_EQ(test, v.scan_len, (u32)sizeof(veo_test_scan_444));

	veo_jpeg_check(veo_test_scan_420, sizeof(veo_test_scan_420),
		       sizeof(veo_test_scan_420), 32, 16, 0x22, 0, &v);
	KUNIT_EXPECT_EQ(test, v.status, VEO_JPEG_OK);
	KUNIT_EXPECT_EQ(test, v.mcus, 2U);

	/* Wrong sampling: the walk no longer fits the frame. */
	veo_jpeg_check(veo_test_scan_420, sizeof(veo_test_scan_420),
		       sizeof(veo_test_scan_420), 32, 16, 0x11, 0, &v);
	KUNIT_EXPECT_EQ(test, v.status, VEO_JPEG_MCU_MISMATCH);

	/* Wrong size: same. */
	veo_jpeg_check(veo_test_scan_444, sizeof(veo_test_scan_444),
		       sizeof(veo_test_scan_444), 16, 24, 0x11, 0, &v);
	KUNIT_EXPECT_EQ(test, v.status, VEO_JPEG_MCU_MISMATCH);
}

/* The h2200 failure: the first 14 bytes missing, 14 bytes of tail read. */
static void veo_test_head_loss(struct kunit *test)
{
	size_t len = sizeof(veo_test_scan_444);
	struct veo_jpeg_verdict v;
	u8 *buf = kunit_kzalloc(test, len, GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, buf);
	memcpy(buf, veo_test_scan_444 + 14, len - 14);
	veo_jpeg_check(buf, len, len, 16, 16, 0x11, 0, &v);
	KUNIT_EXPECT_EQ(test, v.status, VEO_JPEG_SHORT);
	KUNIT_EXPECT_EQ(test, v.short_by, 14);
	KUNIT_EXPECT_EQ(test, v.eoi_off, (s32)(len - 16));
}

static void veo_test_markers(struct kunit *test)
{
	size_t len = sizeof(veo_test_scan_444);
	struct veo_jpeg_verdict v;
	u8 *buf = kunit_kmalloc(test, len, GFP_KERNEL);
	u8 *stuff;

	KUNIT_ASSERT_NOT_NULL(test, buf);

	/* A restart marker where the stuffed 0xff was. */
	memcpy(buf, veo_test_scan_444, len);
	stuff = (u8 *)veo_find_stuffing(buf, len);
	KUNIT_ASSERT_NOT_NULL(test, stuff);
	stuff[1] = 0xd0;
	veo_jpeg_check(buf, len, len, 16, 16, 0x11, 0, &v);
	KUNIT_EXPECT_EQ(test, v.status, VEO_JPEG_BAD_MARKER);

	/* No EOI within the register length. */
	memcpy(buf, veo_test_scan_444, len);
	veo_jpeg_check(buf, len - 2, len, 16, 16, 0x11, 0, &v);
	KUNIT_EXPECT_EQ(test, v.status, VEO_JPEG_NOEOI);

	/* One byte past the EOI is still in place (the factory's len - 3). */
	veo_jpeg_check(veo_test_scan_444, len, len + 1, 16, 16, 0x11, 0, &v);
	KUNIT_EXPECT_EQ(test, v.status, VEO_JPEG_OK);

	/* Two bytes past it are not. */
	veo_jpeg_check(veo_test_scan_444, len, len + 2, 16, 16, 0x11, 0, &v);
	KUNIT_EXPECT_EQ(test, v.status, VEO_JPEG_SHORT);
	KUNIT_EXPECT_EQ(test, v.short_by, 2);
}

/*
 * A 32-bit word cut from the middle of this scan still walks to exactly
 * the right MCU count, so only the EOI position betrays it. Fill bytes
 * after the moved EOI must not make the frame acceptable.
 */
static void veo_test_word_cut_fill(struct kunit *test)
{
	size_t len = sizeof(veo_test_scan_444);
	struct veo_jpeg_verdict v;
	u8 *buf = kunit_kmalloc(test, len, GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, buf);
	memcpy(buf, veo_test_scan_444, 100);
	memcpy(buf + 100, veo_test_scan_444 + 104, len - 104);
	memset(buf + len - 4, 0xff, 4);
	veo_jpeg_check(buf, len, len, 16, 16, 0x11, 0, &v);
	KUNIT_EXPECT_EQ(test, v.status, VEO_JPEG_SHORT);
	KUNIT_EXPECT_EQ(test, v.short_by, 4);
	KUNIT_EXPECT_EQ(test, v.mcus, 4U);
}

/* Read started 64 bytes early and lost 14: the start is found and checked. */
static void veo_test_preroll(struct kunit *test)
{
	size_t len = sizeof(veo_test_scan_444), total = 64 + len + 64;
	struct veo_jpeg_verdict v;
	u8 *raw = kunit_kzalloc(test, total, GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, raw);
	/* Junk with a false EOI, which must not be taken. */
	memset(raw, 0x5a, 64);
	raw[40] = 0xff;
	raw[41] = 0xd9;
	memcpy(raw + 64, veo_test_scan_444, len);

	veo_jpeg_check(raw + 14, total - 14, len, 16, 16, 0x11, 64, &v);
	KUNIT_EXPECT_EQ(test, v.status, VEO_JPEG_OK);
	KUNIT_EXPECT_EQ(test, v.start, 50U);
	KUNIT_EXPECT_EQ(test, v.scan_len, (u32)len);

	/* Without the preroll model the same read is not accepted. */
	veo_jpeg_check(raw + 14, total - 14, len, 16, 16, 0x11, 0, &v);
	KUNIT_EXPECT_NE(test, v.status, VEO_JPEG_OK);

	/* A second, mid-scan hole is not papered over by the search. */
	memmove(raw + 64 + 100, raw + 64 + 114, len - 114);
	veo_jpeg_check(raw + 14, total - 14, len, 16, 16, 0x11, 64, &v);
	KUNIT_EXPECT_NE(test, v.status, VEO_JPEG_OK);
}

static struct kunit_case veo_jpeg_cases[] = {
	KUNIT_CASE(veo_test_header_layout),
	KUNIT_CASE(veo_test_dqt_preview),
	KUNIT_CASE(veo_test_chip_reciprocal),
	KUNIT_CASE(veo_test_scan_ok),
	KUNIT_CASE(veo_test_head_loss),
	KUNIT_CASE(veo_test_markers),
	KUNIT_CASE(veo_test_word_cut_fill),
	KUNIT_CASE(veo_test_preroll),
	{}
};

static struct kunit_suite veo_jpeg_suite = {
	.name = "veo-sdiocam-jpeg",
	.test_cases = veo_jpeg_cases,
};

kunit_test_suite(veo_jpeg_suite);
