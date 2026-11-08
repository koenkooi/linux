/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Constant tables of the HP Photosmart Mobile / Veo SDIO camera bridge.
 * Kept free of kernel APIs other than <linux/types.h> so the JPEG helpers
 * and these tables build outside the kernel too.
 */
#ifndef _VEO_SDIOCAM_TABLES_H
#define _VEO_SDIOCAM_TABLES_H

#include <linux/types.h>

#include "veo-sdiocam-jpeg.h"

#define VEO_CHIP_QUANT_LEN	128
#define VEO_SENSOR_INIT_LEN	99
#define VEO_WB_PRESETS		4
#define VEO_BAND_LIMITS		14
#define VEO_IND_TABLE_LEN	17

struct veo_pair {
	u8 reg;
	u8 val;
};

/* Sensor readout classes, selected by sensor registers 0x52 and 0x00. */
enum veo_class {
	VEO_CLASS_FULL,
	VEO_CLASS_HALF,
	VEO_CLASS_QUARTER,
};

#define VEO_MODE_LEVELS		(1 << 0)	/* still levels calibration */
#define VEO_MODE_CMD28		(1 << 1)	/* engine command 0x28 */
#define VEO_MODE_CMD2C		(1 << 2)	/* engine command 0x2c */
#define VEO_MODE_EFFECT		(1 << 3)	/* colour effects possible */
#define VEO_MODE_EXPERIMENTAL	(1 << 4)	/* still chain, never run */

/**
 * struct veo_mode - one capture mode of the bridge
 * @id:		mode number of the factory driver
 * @class:	sensor readout class (enum veo_class)
 * @x:		window start X, registers 0x02/0x03
 * @w:		window width, registers 0x04/0x05
 * @y:		window start Y, registers 0x06/0x07
 * @h:		window height, registers 0x08/0x09
 * @scaler_w:	scaler input width, registers 0x5a/0x5b
 * @jin_w16:	JPEG input width in 16-pixel units minus one, register 0x75
 * @jin_h16:	JPEG input height in 16-pixel units minus one, register 0x76
 * @isp:	registers 0x70-0x73
 * @out_w:	JPEG output width, registers 0xc8/0xc9
 * @out_h:	JPEG output height, registers 0xca/0xcb
 * @fmt4d:	JPEG format code, register 0x4d
 * @sampling:	luma sampling factors of the scan
 * @stats_avg:	statistics command with average metering, 0 if none
 * @stats_cw:	statistics command with centre-weighted metering
 * @stats_flag:	statistics command flag byte
 * @stats_len:	raw frame length the statistics command reads
 * @flags:	VEO_MODE_*
 */
struct veo_mode {
	u8 id;
	u8 class;
	u16 x, w, y, h;
	u16 scaler_w;
	u8 jin_w16, jin_h16;
	u8 isp[4];
	u16 out_w, out_h;
	u8 fmt4d;
	u8 sampling;
	u8 stats_avg, stats_cw, stats_flag;
	u32 stats_len;
	u8 flags;
};

extern const u8 veo_chip_quant[VEO_QUALITY_TABLES][VEO_CHIP_QUANT_LEN];
extern const struct veo_pair veo_sensor_init[VEO_SENSOR_INIT_LEN];
extern const u16 veo_wb_presets[VEO_WB_PRESETS][4];
extern const u16 veo_band_limits[VEO_BAND_LIMITS];
extern const u8 veo_ind_table[VEO_IND_TABLE_LEN];
extern const struct veo_pair veo_ind_script1[10];
extern const struct veo_pair veo_ind_script3[29];
extern const struct veo_mode veo_modes[];
extern const unsigned int veo_num_modes;

const struct veo_mode *veo_find_mode(u8 id);

#endif /* _VEO_SDIOCAM_TABLES_H */
