/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * JPEG framing helpers for the HP Photosmart Mobile / Veo SDIO camera.
 *
 * The bridge returns bare baseline scan data coded with the ITU-T T.81
 * Annex K Huffman tables. These helpers build the headers that turn a scan
 * into a standalone JPEG and check that a scan read from the card is
 * complete. They use no kernel API beyond types and memcpy so that the same
 * source can be exercised outside the kernel.
 */
#ifndef _VEO_SDIOCAM_JPEG_H
#define _VEO_SDIOCAM_JPEG_H

#include <linux/types.h>

#define VEO_QUALITY_TABLES	3
#define VEO_JPEG_DQT_BODY	130	/* two 8-bit tables with their Pq/Tq bytes */
#define VEO_JPEG_DHT_BODY	416	/* the four Annex K tables */
#define VEO_JPEG_DHT_LEN	(4 + VEO_JPEG_DHT_BODY)
#define VEO_JPEG_HDR_NO_DHT	169	/* SOI, DQT, SOF0, SOS */
#define VEO_JPEG_HDR_MAX	(VEO_JPEG_HDR_NO_DHT + VEO_JPEG_DHT_LEN)

#define VEO_JPEG_SAMPLING_444	0x11
#define VEO_JPEG_SAMPLING_420	0x22

extern const u8 veo_dqt[VEO_QUALITY_TABLES][VEO_JPEG_DQT_BODY];
extern const u8 veo_dht[VEO_JPEG_DHT_BODY];

enum veo_jpeg_status {
	VEO_JPEG_OK,
	VEO_JPEG_SHORT,		/* EOI earlier than the register length says */
	VEO_JPEG_NOEOI,		/* no marker within the register length */
	VEO_JPEG_MCU_MISMATCH,	/* EOI in place, MCU walk does not fit */
	VEO_JPEG_BAD_MARKER,	/* a marker other than EOI in the scan */
	VEO_JPEG_BAD_ARGS,
};

/**
 * struct veo_jpeg_verdict - result of veo_jpeg_check()
 * @status:	verdict
 * @start:	offset of the scan in the buffer
 * @scan_len:	bytes from @start up to and including the EOI marker
 * @eoi_off:	offset of the EOI marker from @start, or -1
 * @short_by:	(len_reg - 2) - @eoi_off when @status is VEO_JPEG_SHORT
 * @mcus:	MCUs the Huffman walk decoded before the EOI
 * @expected:	MCUs the frame size calls for
 */
struct veo_jpeg_verdict {
	enum veo_jpeg_status status;
	u32 start;
	u32 scan_len;
	s32 eoi_off;
	s32 short_by;
	u32 mcus;
	u32 expected;
};

size_t veo_jpeg_build_header(u8 *buf, size_t size, u16 width, u16 height,
			     u8 sampling, unsigned int dqt_idx, bool with_dht);

enum veo_jpeg_status veo_jpeg_check(const u8 *buf, size_t avail, u32 len_reg,
				    u16 width, u16 height, u8 sampling,
				    u32 preroll,
				    struct veo_jpeg_verdict *verdict);

#endif /* _VEO_SDIOCAM_JPEG_H */
