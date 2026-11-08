// SPDX-License-Identifier: GPL-2.0-only
/*
 * HP Photosmart Mobile / Veo SDIO camera: controls and the AE/AWB loops.
 *
 * Control values are cached in dev->cfg and written to the card only while
 * it is initialised; the 3 s initialisation applies the cached values.
 *
 * AE and AWB follow the factory driver's loops, in integer arithmetic. The
 * bridge computes three statistics bytes over a 40 x 30 cell grid of the
 * raw frame; AE servos exposure towards luma 128, AWB servos the red and
 * blue gains against green while exposure is settled.
 */

#include <linux/bitops.h>
#include <linux/minmax.h>
#include <linux/mutex.h>
#include <media/v4l2-ctrls.h>

#include "veo-sdiocam.h"

/* AWB ratios in units of 1/1024, given in thousandths; gains are 8.8. */
#define VEO_Q(milli)		((milli) * 1024 / 1000)
#define VEO_AWB_STEP_BIG	VEO_Q(200)
#define VEO_AWB_STEP_SMALL	VEO_Q(10)

/* Index of the factory presets: {blue, green, green, red}. */
static const u8 veo_wb_menu_preset[] = {
	[V4L2_WHITE_BALANCE_FLUORESCENT] = 1,
	[V4L2_WHITE_BALANCE_INCANDESCENT] = 2,
	[V4L2_WHITE_BALANCE_DAYLIGHT] = 3,
};

static u16 veo_class_exposure_max(u8 class)
{
	switch (class) {
	case VEO_CLASS_FULL:
		return VEO_EXPOSURE_MAX;
	case VEO_CLASS_HALF:
		return VEO_EXPOSURE_REG_MAX;
	default:
		return VEO_EXPOSURE_REG_MAX / 2;
	}
}

/* Steps that grow with exposure, used only far from the target. */
static int veo_ae_step_far(unsigned int e, int d)
{
	if (e < 171)
		return d / 2;
	if (e < 251)
		return (d / 3) * 2;
	if (e <= 350)
		return d;
	if (e < 501)
		return (d / 2) * 3;
	if (e < 751)
		return d * 2;
	if (e < 1001)
		return (d / 2) * 5;
	if (e < 1401)
		return d * 3;
	return (d / 2) * 7;
}

static int veo_ae_step_near(unsigned int e, int d)
{
	if (e < 16)
		return 1;
	if (e < 26)
		return 2;
	if (e < 36)
		return d >= 80 ? d >> 4 : 4;
	return max(d / 12, 6);
}

static void veo_set_cur(struct veo_dev *dev, u16 *field, u16 val)
{
	spin_lock_irq(&dev->slock);
	*field = val;
	spin_unlock_irq(&dev->slock);
}

static int veo_awb_update(struct veo_dev *dev)
{
	int a = dev->cur_red << 2, c = dev->cur_blue << 2;
	bool hot = a > VEO_Q(1300);
	int bias = hot ? VEO_Q(20) : VEO_Q(10);
	u16 red, blue;
	u8 st[3];
	int r, err;

	err = veo_hw_awb_stats(dev, st);
	if (err || !st[1])
		return err;

	r = st[2] * 1024 / st[1] - bias;
	if (r > VEO_Q(1200))
		a -= VEO_AWB_STEP_BIG;
	else if (r > VEO_Q(1030))
		a -= (r - 1024) * 4 / 5;
	else if (r > VEO_Q(1005))
		a -= VEO_AWB_STEP_SMALL;
	else if (r < VEO_Q(800))
		a += VEO_AWB_STEP_BIG;
	else if (r < VEO_Q(980))
		a += (1024 - r) * 4 / 5;
	else if (r < VEO_Q(995))
		a += VEO_AWB_STEP_SMALL;
	a = min(a, VEO_Q(1500));

	/* The factory adds the bias here where it subtracts it above. */
	r = st[0] * 1024 / st[1] + bias;
	if (r > VEO_Q(1200))
		c -= VEO_AWB_STEP_BIG;
	else if (r > VEO_Q(1020))
		c -= r - 1024;
	else if (r > VEO_Q(1005))
		c -= VEO_AWB_STEP_SMALL;
	else if (r < VEO_Q(800))
		c += VEO_AWB_STEP_BIG;
	else if (r < VEO_Q(980))
		c += 1024 - r;
	else if (r < VEO_Q(995))
		c += VEO_AWB_STEP_SMALL;
	if (hot)
		c = clamp(c, VEO_Q(1250), VEO_Q(1400));

	red = clamp(a >> 2, VEO_GAIN_MIN, VEO_GAIN_MAX);
	blue = clamp(c >> 2, VEO_GAIN_MIN, VEO_GAIN_MAX);
	if (red == dev->cur_red && blue == dev->cur_blue)
		return 0;
	err = veo_hw_set_gains(dev, red, blue);
	if (!err) {
		spin_lock_irq(&dev->slock);
		dev->cur_red = red;
		dev->cur_blue = blue;
		spin_unlock_irq(&dev->slock);
	}
	return err;
}

/*
 * The factory AE with one change: exposure is clamped at what the current
 * readout class can actually use, so it does not wind up beyond the
 * register limit in the dark and then take many passes to come back.
 */
static int veo_ae_update(struct veo_dev *dev, u8 r, u8 g, u8 b)
{
	unsigned int e = dev->cur_exposure;
	unsigned int emax = veo_class_exposure_max(dev->mode->class);
	int y, d, step;

	if (!g)
		return 0;
	y = (299 * r + 587 * g + 114 * b) / 1000;

	if (y < 123) {
		d = 128 - y;
		if (y > 50 && e <= 60)
			step = veo_ae_step_near(e, d);
		else if (e < 101 || y > 50)
			step = (d / 5) * 2;
		else
			step = veo_ae_step_far(e, d);
		step = min(step, 200);
		dev->ae_last_dir = 1;
		e = min(e + step, emax);
	} else if (y <= 133) {
		dev->ae_last_dir = 0;
		if (dev->cfg.awb) {
			int err = veo_awb_update(dev);

			if (err)
				return err;
		}
		if (y > 127 || e > 80)
			return 0;
		e = min(e + 1, emax);
	} else {
		d = y - 128;
		if (e <= 60)
			step = veo_ae_step_near(e, d);
		else if (e < 101 || d < 60)
			step = (d / 5) * 2;
		else
			step = veo_ae_step_far(e, d);
		if (dev->ae_last_dir > 0)
			step /= 2;
		step = min(step, 200);
		dev->ae_last_dir = -1;
		e = e > step ? e - step : VEO_EXPOSURE_MIN;
		e = min(e, emax);
	}

	if (e == dev->cur_exposure)
		return 0;
	veo_set_cur(dev, &dev->cur_exposure, e);
	return veo_hw_set_exposure(dev, dev->mode, e);
}

/*
 * One statistics pass over the raw frame of @set, the set just read; the
 * work item runs it in the idle time before the next capture. Called with
 * hw_lock held.
 */
int veo_ctrl_3a(struct veo_dev *dev, unsigned int set)
{
	const struct veo_mode *mode = dev->mode;
	u8 st[3], strap, cmd;
	int err;

	if (!mode->stats_avg || (!dev->cfg.ae && !dev->cfg.awb))
		return 0;

	/* The strap flips the sensor readout; the factory re-reads it here. */
	err = veo_hw_read_strap(dev, &strap);
	if (err)
		return err;
	if ((strap & 1) != dev->bayer) {
		err = veo_hw_reorient(dev, strap & 1, dev->cur_red,
				      dev->cur_blue);
		if (!err)
			err = veo_hw_mode_start(dev, mode);
		return err;
	}

	cmd = dev->cfg.metering == V4L2_EXPOSURE_METERING_CENTER_WEIGHTED ?
	      mode->stats_cw : mode->stats_avg;
	err = veo_hw_stats(dev, cmd, veo_raw_base(set), mode->stats_len,
			   mode->stats_flag, st);
	if (err)
		return err;

	if (dev->cfg.ae)
		return veo_ae_update(dev, st[2], st[1], st[0]);
	return veo_awb_update(dev);
}

static int veo_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct veo_dev *dev = container_of(ctrl->handler, struct veo_dev, hdl);

	spin_lock_irq(&dev->slock);
	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE_AUTO:
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		dev->ctrl_exposure->val = dev->cur_exposure;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
	case V4L2_CID_RED_BALANCE:
	case V4L2_CID_BLUE_BALANCE:
		dev->ctrl_red->val = dev->cur_red;
		dev->ctrl_blue->val = dev->cur_blue;
		break;
	}
	spin_unlock_irq(&dev->slock);
	return 0;
}

/*
 * Runs with the handler lock held, before hw_lock is taken. @wb_sync marks
 * a change one of the two white balance paths is making on behalf of the
 * other, so neither echoes it back.
 */
static int veo_s_wb_preset(struct veo_dev *dev, s32 val)
{
	const u16 *p;
	int err;

	if (dev->wb_sync)
		return 0;

	switch (val) {
	case V4L2_WHITE_BALANCE_AUTO:
	case V4L2_WHITE_BALANCE_MANUAL:
		p = NULL;
		break;
	default:
		if (val >= ARRAY_SIZE(veo_wb_menu_preset) ||
		    !veo_wb_menu_preset[val])
			return -EINVAL;
		p = veo_wb_presets[veo_wb_menu_preset[val]];
		break;
	}

	dev->wb_sync = true;
	err = __v4l2_ctrl_s_ctrl(dev->ctrl_awb,
				 val == V4L2_WHITE_BALANCE_AUTO);
	if (!err && p)
		err = __v4l2_ctrl_s_ctrl(dev->ctrl_red, p[3]);
	if (!err && p)
		err = __v4l2_ctrl_s_ctrl(dev->ctrl_blue, p[0]);
	dev->wb_sync = false;
	return err;
}

/*
 * AWB or a gain changed through its own control: the preset menu would
 * otherwise keep showing a preset the hardware is no longer in.
 */
static void veo_sync_wb_preset(struct veo_dev *dev)
{
	if (dev->wb_sync)
		return;

	dev->wb_sync = true;
	__v4l2_ctrl_s_ctrl(dev->ctrl_wb_preset,
			   dev->cfg.awb ? V4L2_WHITE_BALANCE_AUTO :
					  V4L2_WHITE_BALANCE_MANUAL);
	dev->wb_sync = false;
}

static int veo_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct veo_dev *dev = container_of(ctrl->handler, struct veo_dev, hdl);
	bool live;
	int err = 0;

	if (ctrl->id == V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE)
		return veo_s_wb_preset(dev, ctrl->val);

	mutex_lock(&dev->hw_lock);
	live = dev->initialised && dev->streaming && dev->mode;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE_AUTO:
		dev->cfg.ae = ctrl->val == V4L2_EXPOSURE_AUTO;
		if (dev->cfg.ae)
			break;
		dev->cfg.exposure = dev->ctrl_exposure->val;
		veo_set_cur(dev, &dev->cur_exposure, dev->cfg.exposure);
		if (live)
			err = veo_hw_set_exposure(dev, dev->mode,
						  dev->cfg.exposure);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev->cfg.awb = ctrl->val;
		if (dev->cfg.awb)
			break;
		dev->cfg.red = dev->ctrl_red->val;
		dev->cfg.blue = dev->ctrl_blue->val;
		spin_lock_irq(&dev->slock);
		dev->cur_red = dev->cfg.red;
		dev->cur_blue = dev->cfg.blue;
		spin_unlock_irq(&dev->slock);
		if (dev->initialised)
			err = veo_hw_set_gains(dev, dev->cfg.red, dev->cfg.blue);
		break;
	case V4L2_CID_EXPOSURE_METERING:
		dev->cfg.metering = ctrl->val;
		break;
	case V4L2_CID_ZOOM_ABSOLUTE:
		/* Read at STREAMON; the control is grabbed while streaming. */
		break;
	case V4L2_CID_JPEG_COMPRESSION_QUALITY:
		/*
		 * Three chip tables, coarse to fine 2, 0, 1; each pairs with the
		 * DQT of the same index, so both are chosen at STREAMON.
		 */
		dev->cfg.quant_idx = ctrl->val >= 88 ? 1 : ctrl->val >= 70 ? 0 : 2;
		break;
	default:
		err = -EINVAL;
		break;
	}

	mutex_unlock(&dev->hw_lock);

	if (!err && ctrl->id == V4L2_CID_AUTO_WHITE_BALANCE)
		veo_sync_wb_preset(dev);
	return err;
}

static const struct v4l2_ctrl_ops veo_ctrl_ops = {
	.g_volatile_ctrl = veo_g_volatile_ctrl,
	.s_ctrl = veo_s_ctrl,
};

int veo_ctrl_init(struct veo_dev *dev)
{
	struct v4l2_ctrl_handler *hdl = &dev->hdl;

	v4l2_ctrl_handler_init(hdl, 10);

	dev->ctrl_exposure_auto =
		v4l2_ctrl_new_std_menu(hdl, &veo_ctrl_ops,
				       V4L2_CID_EXPOSURE_AUTO,
				       V4L2_EXPOSURE_MANUAL,
				       ~(BIT(V4L2_EXPOSURE_AUTO) |
					 BIT(V4L2_EXPOSURE_MANUAL)),
				       V4L2_EXPOSURE_AUTO);
	dev->ctrl_exposure = v4l2_ctrl_new_std(hdl, &veo_ctrl_ops,
					       V4L2_CID_EXPOSURE_ABSOLUTE,
					       VEO_EXPOSURE_MIN,
					       VEO_EXPOSURE_MAX, 1,
					       VEO_EXPOSURE_DEF);
	v4l2_ctrl_new_std_menu(hdl, &veo_ctrl_ops, V4L2_CID_EXPOSURE_METERING,
			       V4L2_EXPOSURE_METERING_CENTER_WEIGHTED, 0,
			       V4L2_EXPOSURE_METERING_AVERAGE);

	dev->ctrl_awb = v4l2_ctrl_new_std(hdl, &veo_ctrl_ops,
					  V4L2_CID_AUTO_WHITE_BALANCE,
					  0, 1, 1, 1);
	dev->ctrl_red = v4l2_ctrl_new_std(hdl, &veo_ctrl_ops,
					  V4L2_CID_RED_BALANCE, VEO_GAIN_MIN,
					  VEO_GAIN_MAX, 1, VEO_GAIN_DEF);
	dev->ctrl_blue = v4l2_ctrl_new_std(hdl, &veo_ctrl_ops,
					   V4L2_CID_BLUE_BALANCE, VEO_GAIN_MIN,
					   VEO_GAIN_MAX, 1, VEO_GAIN_DEF);
	dev->ctrl_wb_preset =
		v4l2_ctrl_new_std_menu(hdl, &veo_ctrl_ops,
				       V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,
				       V4L2_WHITE_BALANCE_DAYLIGHT,
				       BIT(V4L2_WHITE_BALANCE_FLUORESCENT_H) |
				       BIT(V4L2_WHITE_BALANCE_HORIZON),
				       V4L2_WHITE_BALANCE_AUTO);

	dev->ctrl_zoom = v4l2_ctrl_new_std(hdl, &veo_ctrl_ops,
					   V4L2_CID_ZOOM_ABSOLUTE, 0, 2, 1, 0);
	dev->ctrl_quality = v4l2_ctrl_new_std(hdl, &veo_ctrl_ops,
					      V4L2_CID_JPEG_COMPRESSION_QUALITY,
					      1, 100, 1, 50);

	if (hdl->error)
		return hdl->error;

	v4l2_ctrl_auto_cluster(2, &dev->ctrl_exposure_auto,
			       V4L2_EXPOSURE_MANUAL, true);
	v4l2_ctrl_auto_cluster(3, &dev->ctrl_awb, 0, true);

	return v4l2_ctrl_handler_setup(hdl);
}

void veo_ctrl_stream(struct veo_dev *dev, bool on)
{
	v4l2_ctrl_grab(dev->ctrl_zoom, on);
	v4l2_ctrl_grab(dev->ctrl_quality, on);
}
