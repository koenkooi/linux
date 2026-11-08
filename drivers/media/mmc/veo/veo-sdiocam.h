/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * HP Photosmart Mobile / Veo SDIO camera driver: shared definitions.
 */
#ifndef _VEO_SDIOCAM_H
#define _VEO_SDIOCAM_H

#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/videobuf2-v4l2.h>

#include "veo-sdiocam-jpeg.h"
#include "veo-sdiocam-tables.h"

struct dentry;
struct sdio_func;

#define VEO_DRIVER_NAME		"veo_sdiocam"
#define VEO_FW_NAME		"veo/hpcamera-init.bin"
#define VEO_FW_LEN		1620

#define VEO_PREROLL_MAX		256
#define VEO_BLKSZ		64
#define VEO_CHUNK		SZ_16K

/* ASIC memory: two ping-pong sets of raw and JPEG buffers. */
#define VEO_RAW_A		0x001000
#define VEO_JPEG_A		0x145000
#define VEO_RAW_B		0x1a0000
#define VEO_JPEG_B		0x1eb000
#define VEO_STATS_BUF		0x1ff000

#define VEO_EXPOSURE_MIN	1
#define VEO_EXPOSURE_MAX	0xcc6
#define VEO_EXPOSURE_DEF	0xe6
#define VEO_EXPOSURE_REG_MAX	0x890
#define VEO_GAIN_MIN		0x40
#define VEO_GAIN_MAX		0x3ff
#define VEO_GAIN_DEF		0x140

/*
 * Steps of veo_hw_still_frame(), then the per-still setup and the stop
 * before the read, to name the one that failed.
 */
enum veo_still_step {
	VEO_STILL_NONE,
	VEO_STILL_SETUP,
	VEO_STILL_CAPTURE1,
	VEO_STILL_CAPTURE2,
	VEO_STILL_CAPTURE3,
	VEO_STILL_LEVELS,
	VEO_STILL_CMD28,
	VEO_STILL_CMD2C,
	VEO_STILL_JPEG,
	VEO_STILL_RESETUP,
	VEO_STILL_STOP,
	VEO_STILL_STEPS
};

/* Values the controls ask for; applied when the card is initialised. */
struct veo_hwcfg {
	u16 exposure;
	u16 red;
	u16 blue;
	bool ae;
	bool awb;
	u8 metering;
	u8 quant_idx;
};

enum veo_state {
	VEO_ST_ARM,
	VEO_ST_WAIT,
	VEO_ST_RESTART,
};

struct veo_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

struct veo_size;

struct veo_dev {
	struct sdio_func *func;
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct vb2_queue queue;
	struct v4l2_ctrl_handler hdl;

	/* serialises the video node and the vb2 queue */
	struct mutex lock;
	/* serialises every access to the card and the hardware state below */
	struct mutex hw_lock;
	/* protects @bufs, @kick, @stats and the AE/AWB snapshot */
	spinlock_t slock;
	struct list_head bufs;
	bool kick;

	struct workqueue_struct *wq;
	struct delayed_work work;

	/* format, under @lock */
	u32 pixelformat;
	const struct veo_size *size;
	struct v4l2_fract timeperframe;

	/* stream state, under @hw_lock */
	const struct veo_mode *mode;
	u8 hdr[VEO_JPEG_HDR_MAX];
	unsigned int hdr_len;
	u8 *bounce;
	u8 *chunk;
	size_t bounce_len;
	u32 sizeimage;
	u32 scan_max;
	u32 preroll;
	enum veo_state state;
	unsigned int armed_set;
	u64 t_arm;
	u64 t_last_done;
	u64 interval_ns;
	unsigned int failed_restarts;
	unsigned int frame_count;
	bool stats_due;
	bool still_ready;	/* still setup done, no still taken since */
	u32 sequence;
	bool streaming;
	bool stopping;
	bool was_streaming;
	bool initialised;
	bool bayer;
	bool cccr_fallback;
	unsigned int cccr_misses;
	u8 irq5e;
	u8 irqa7;
	u8 *fw;
	struct veo_hwcfg cfg;

	/* exposure and gains in effect, under @slock */
	u16 cur_exposure;
	u16 cur_red;
	u16 cur_blue;
	int ae_last_dir;

	struct v4l2_ctrl *ctrl_exposure_auto;
	struct v4l2_ctrl *ctrl_exposure;
	struct v4l2_ctrl *ctrl_awb;
	struct v4l2_ctrl *ctrl_red;
	struct v4l2_ctrl *ctrl_blue;
	struct v4l2_ctrl *ctrl_wb_preset;
	bool wb_sync;
	struct v4l2_ctrl *ctrl_zoom;
	struct v4l2_ctrl *ctrl_quality;
};

/* veo-sdiocam-core.c, all called with @hw_lock held and the host released */
int veo_hw_link_check(struct veo_dev *dev, u8 *strap);
int veo_hw_init(struct veo_dev *dev);
int veo_hw_mode_start(struct veo_dev *dev, const struct veo_mode *mode);
int veo_hw_arm(struct veo_dev *dev, unsigned int set);
int veo_hw_still_setup(struct veo_dev *dev, const struct veo_mode *mode);
int veo_hw_still_frame(struct veo_dev *dev, const struct veo_mode *mode);
int veo_hw_still_done(struct veo_dev *dev);
int veo_hw_poll_done(struct veo_dev *dev, bool *done);
int veo_hw_frame_len(struct veo_dev *dev, u32 *len);
int veo_hw_read_frame(struct veo_dev *dev, u32 src, u32 len, size_t xfer);
int veo_hw_stop(struct veo_dev *dev);
int veo_hw_standby(struct veo_dev *dev);
int veo_hw_set_exposure(struct veo_dev *dev, const struct veo_mode *mode,
			u16 exposure);
int veo_hw_set_gains(struct veo_dev *dev, u16 red, u16 blue);
int veo_hw_stats(struct veo_dev *dev, u8 cmd, u32 raw, u32 len, u8 flag,
		 u8 st[3]);
int veo_hw_awb_stats(struct veo_dev *dev, u8 st[3]);
int veo_hw_read_strap(struct veo_dev *dev, u8 *strap);
int veo_hw_reorient(struct veo_dev *dev, bool flipped, u16 red, u16 blue);
u32 veo_jpeg_base(unsigned int set);
u32 veo_raw_base(unsigned int set);

/* veo-sdiocam-ctrl.c */
int veo_ctrl_init(struct veo_dev *dev);
void veo_ctrl_stream(struct veo_dev *dev, bool on);
int veo_ctrl_3a(struct veo_dev *dev, unsigned int set);

#endif /* _VEO_SDIOCAM_H */
