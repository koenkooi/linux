// SPDX-License-Identifier: GPL-2.0-only
/*
 * HP Photosmart Mobile / Veo SDIO camera: V4L2 capture driver.
 *
 * The card is an SDIO function in front of a camera ASIC that captures a
 * Bayer frame into its own memory and compresses it into baseline JPEG scan
 * data. The host starts every frame, polls for the JPEG-done flag and reads
 * the scan out of a data FIFO; this driver adds the JPEG header and checks
 * every scan before it is handed to userspace.
 */

#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-vmalloc.h>

#include "veo-sdiocam.h"

#define VEO_NO_MODE		0xff
#define VEO_STALL_NS		(600 * NSEC_PER_MSEC)
#define VEO_PREVIEW_TIMEOUT_NS	(200 * NSEC_PER_MSEC)
#define VEO_STILL_TIMEOUT_NS	(2000ULL * NSEC_PER_MSEC)
#define VEO_MAX_RESTARTS	3
#define VEO_NO_REQUEUE		ULONG_MAX

static unsigned int preroll;
module_param(preroll, uint, 0644);
MODULE_PARM_DESC(preroll,
		 "start the FIFO read this many bytes (even, at most 256) before the frame; experimental, a frame is accepted only if it then validates in full");

static bool deliver_damaged;
module_param(deliver_damaged, bool, 0644);
MODULE_PARM_DESC(deliver_damaged,
		 "hand frames that fail validation to userspace with V4L2_BUF_FLAG_ERROR instead of dropping them (diagnostics)");

static bool experimental_modes;
module_param(experimental_modes, bool, 0444);
MODULE_PARM_DESC(experimental_modes,
		 "also offer 640x480, 1280x960 and 1280x1024 through the factory still chain; untested on hardware; frames larger than the host's per-command limit are read in several transfers");

/*
 * The factory driver takes every still as a one-shot that starts with the
 * whole still setup and stops the pipeline before the read. Streaming
 * stills that skip both steps lose whole words from the head of the JPEG
 * in ASIC memory from the second frame on, so both follow the factory by
 * default; turning one off shows which of the two the camera needs.
 */
static bool still_resetup = true;
module_param(still_resetup, bool, 0644);
MODULE_PARM_DESC(still_resetup,
		 "repeat the full still setup before every still, as the factory driver does (default Y; N only at stream start, diagnostics)");

static bool still_stop = true;
module_param(still_stop, bool, 0644);
MODULE_PARM_DESC(still_stop,
		 "stop the pipeline after a still's JPEG run and before its read, as the factory driver does (default Y; N for diagnostics)");

static const struct v4l2_fract veo_preview_intervals[] = {
	{ 1, 10 }, { 1, 5 }, { 1, 2 }, { 1, 1 },
};

static const struct v4l2_fract veo_still_intervals[] = {
	{ 1, 2 }, { 1, 1 },
};

/**
 * struct veo_size - one frame size offered to userspace
 * @width:	width in pixels
 * @height:	height in pixels
 * @modes:	bridge mode for zoom steps 0, 1 and 2, VEO_NO_MODE if none
 * @sizeimage:	buffer size, header included
 * @intervals:	frame intervals offered
 * @n_intervals: number of @intervals
 */
struct veo_size {
	u16 width;
	u16 height;
	u8 modes[3];
	u32 sizeimage;
	const struct v4l2_fract *intervals;
	unsigned int n_intervals;
};

static const struct veo_size veo_sizes[] = {
	{
		.width = 160, .height = 120,
		.modes = { 0x06, 0x14, 0x15 },
		.sizeimage = SZ_32K,
		.intervals = veo_preview_intervals,
		.n_intervals = ARRAY_SIZE(veo_preview_intervals),
	}, {
		/* 76210 bytes is the factory application's frame limit. */
		.width = 320, .height = 240,
		.modes = { 0x04, 0x09, 0x11 },
		.sizeimage = SZ_64K + SZ_16K,
		.intervals = veo_preview_intervals,
		.n_intervals = ARRAY_SIZE(veo_preview_intervals),
	},
	/* Only with experimental_modes, and always last. */
	{
		.width = 640, .height = 480,
		.modes = { 0x02, 0x07, VEO_NO_MODE },
		.sizeimage = SZ_128K,
		.intervals = veo_still_intervals,
		.n_intervals = ARRAY_SIZE(veo_still_intervals),
	}, {
		/* The ASIC has 0xba000 bytes of room for the JPEG. */
		.width = 1280, .height = 960,
		.modes = { 0x01, VEO_NO_MODE, VEO_NO_MODE },
		.sizeimage = SZ_512K,
		.intervals = veo_still_intervals,
		.n_intervals = ARRAY_SIZE(veo_still_intervals),
	}, {
		.width = 1280, .height = 1024,
		.modes = { 0x00, VEO_NO_MODE, VEO_NO_MODE },
		.sizeimage = SZ_512K,
		.intervals = veo_still_intervals,
		.n_intervals = ARRAY_SIZE(veo_still_intervals),
	},
};

#define VEO_PREVIEW_SIZES	2

static const u32 veo_pixfmts[] = {
	V4L2_PIX_FMT_JPEG,	/* standalone: with the Annex K DHT */
	V4L2_PIX_FMT_MJPEG,	/* the same without the DHT */
};

static unsigned int veo_num_sizes(void)
{
	return experimental_modes ? ARRAY_SIZE(veo_sizes) : VEO_PREVIEW_SIZES;
}

static bool veo_still(const struct veo_mode *mode)
{
	return mode->flags & VEO_MODE_EXPERIMENTAL;
}

static int veo_start_mode(struct veo_dev *dev)
{
	int err;

	if (veo_still(dev->mode)) {
		dev->state = VEO_ST_ARM;
		err = veo_hw_still_setup(dev, dev->mode);
		dev->still_ready = !err;
		return err;
	}
	dev->state = VEO_ST_WAIT;
	return veo_hw_mode_start(dev, dev->mode);
}

static const struct veo_mode *veo_size_mode(const struct veo_size *size,
					    unsigned int zoom)
{
	while (zoom && size->modes[zoom] == VEO_NO_MODE)
		zoom--;
	return veo_find_mode(size->modes[zoom]);
}

static inline struct veo_buf *to_veo_buf(struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct veo_buf, vb);
}

/* Streaming work: arm, wait for JPEG done, read, validate, deliver. */

static struct veo_buf *veo_next_buf(struct veo_dev *dev)
{
	struct veo_buf *buf;

	spin_lock_irq(&dev->slock);
	buf = list_first_entry_or_null(&dev->bufs, struct veo_buf, list);
	if (buf)
		list_del(&buf->list);
	spin_unlock_irq(&dev->slock);
	return buf;
}

static void veo_deliver(struct veo_dev *dev, struct veo_buf *buf,
			const u8 *scan, u32 scan_len, u64 ts,
			enum vb2_buffer_state state)
{
	struct vb2_buffer *vb = &buf->vb.vb2_buf;
	u8 *vaddr = vb2_plane_vaddr(vb, 0);
	size_t room = vb2_plane_size(vb, 0);

	if (dev->hdr_len + scan_len > room) {
		scan_len = room > dev->hdr_len ? room - dev->hdr_len : 0;
		state = VB2_BUF_STATE_ERROR;
	}
	memcpy(vaddr, dev->hdr, dev->hdr_len);
	memcpy(vaddr + dev->hdr_len, scan, scan_len);
	vb2_set_plane_payload(vb, 0, dev->hdr_len + scan_len);
	buf->vb.field = V4L2_FIELD_NONE;
	buf->vb.sequence = dev->sequence;
	vb->timestamp = ts;
	vb2_buffer_done(vb, state);
}

static void veo_still_failed(struct veo_dev *dev, enum veo_still_step step,
			     int err)
{
	dev_dbg(&dev->func->dev, "still step %d failed (%d)\n", step, err);
}

/* Called after JPEG done for the armed set. */
static void veo_read_and_deliver(struct veo_dev *dev)
{
	const struct veo_mode *mode = dev->mode;
	struct veo_jpeg_verdict v;
	struct veo_buf *buf;
	u32 len, xfer;
	u64 ts;
	int err;

	err = veo_hw_frame_len(dev, &len);
	if (err) {
		dev_warn_ratelimited(&dev->func->dev,
				     "frame length read failed (%d)\n", err);
		goto out;
	}
	if (len < VEO_BLKSZ || len > dev->scan_max)
		goto out;
	if (veo_still(mode) && READ_ONCE(still_stop)) {
		err = veo_hw_still_done(dev);
		if (err) {
			veo_still_failed(dev, VEO_STILL_STOP, err);
			goto out;
		}
	}

	xfer = ALIGN(len + dev->preroll, VEO_BLKSZ);
	err = veo_hw_read_frame(dev, veo_jpeg_base(dev->armed_set) - dev->preroll,
				len + dev->preroll, xfer);
	ts = ktime_get_ns();
	if (err) {
		dev_warn_ratelimited(&dev->func->dev,
				     "frame read of %u bytes failed (%d)\n",
				     xfer, err);
		goto out;
	}

	veo_jpeg_check(dev->bounce, xfer, len, mode->out_w, mode->out_h,
		       mode->sampling, dev->preroll, &v);

	if (v.status != VEO_JPEG_OK && !READ_ONCE(deliver_damaged))
		goto out;

	buf = veo_next_buf(dev);
	if (!buf)
		goto out;
	if (v.status == VEO_JPEG_OK)
		veo_deliver(dev, buf, dev->bounce + v.start, v.scan_len, ts,
			    VB2_BUF_STATE_DONE);
	else
		veo_deliver(dev, buf, dev->bounce, len, ts,
			    VB2_BUF_STATE_ERROR);
out:
	/* Counted per captured frame, so a gap in the sequence is a drop. */
	dev->sequence++;
}

static bool veo_have_buf(struct veo_dev *dev)
{
	bool have;

	spin_lock_irq(&dev->slock);
	have = !list_empty(&dev->bufs);
	if (!have)
		dev->kick = true;
	spin_unlock_irq(&dev->slock);
	return have;
}

static unsigned long veo_restart(struct veo_dev *dev)
{
	int err;

	if (dev->was_streaming) {
		/* A resume, not a stall. */
		dev->was_streaming = false;
	} else {
		if (++dev->failed_restarts > VEO_MAX_RESTARTS) {
			dev_err(&dev->func->dev,
				"no frame after %u restarts, giving up\n",
				VEO_MAX_RESTARTS);
			vb2_queue_error(&dev->queue);
			return VEO_NO_REQUEUE;
		}
	}

	err = veo_hw_init(dev);
	if (!err)
		err = veo_start_mode(dev);
	dev->t_last_done = ktime_get_ns();
	if (err) {
		dev->state = VEO_ST_RESTART;
		return msecs_to_jiffies(100);
	}
	return 0;
}

static unsigned long veo_cycle(struct veo_dev *dev)
{
	u64 now = ktime_get_ns(), next, timeout, stall;
	bool done = false;
	int err;

	if (!dev->streaming)
		return VEO_NO_REQUEUE;

	timeout = veo_still(dev->mode) ? VEO_STILL_TIMEOUT_NS :
					 VEO_PREVIEW_TIMEOUT_NS;
	/* A still takes several captures before its JPEG run. */
	stall = veo_still(dev->mode) ? 4 * VEO_STILL_TIMEOUT_NS : VEO_STALL_NS;

	switch (dev->state) {
	case VEO_ST_RESTART:
		return veo_restart(dev);
	case VEO_ST_ARM:
		/* Back-pressure: never capture ahead of the consumer. */
		if (!veo_have_buf(dev))
			return VEO_NO_REQUEUE;
		next = dev->t_arm + dev->interval_ns;
		if (time_before64(now, next))
			return max_t(unsigned long, 1,
				     nsecs_to_jiffies(next - now));
		/* An idle spell is not a stall. */
		if (now - dev->t_arm > stall)
			dev->t_last_done = now;
		/* AE/AWB statistics in the gap, on the frame just read. */
		if (dev->stats_due) {
			dev->stats_due = false;
			if (veo_ctrl_3a(dev, dev->armed_set ^ 1))
				dev_dbg_ratelimited(&dev->func->dev,
						    "AE/AWB statistics read failed\n");
		}
		if (veo_still(dev->mode)) {
			err = 0;
			/*
			 * Skip the setup only when STREAMON or a restart has
			 * just run it: a still that failed or never reported
			 * done, the first one included, leaves the pipeline
			 * mid-still, and a restart has already set it up.
			 */
			if (!dev->still_ready && READ_ONCE(still_resetup)) {
				err = veo_hw_still_setup(dev, dev->mode);
				if (err)
					veo_still_failed(dev, VEO_STILL_RESETUP,
							 err);
			}
			dev->still_ready = false;
			if (!err)
				err = veo_hw_still_frame(dev, dev->mode);
		} else {
			err = veo_hw_arm(dev, dev->armed_set);
		}
		if (err) {
			dev_warn_ratelimited(&dev->func->dev,
					     "frame start failed (%d)\n", err);
			if (ktime_get_ns() - dev->t_last_done > stall)
				dev->state = VEO_ST_RESTART;
			return msecs_to_jiffies(10);
		}
		dev->state = VEO_ST_WAIT;
		fallthrough;
	case VEO_ST_WAIT:
		for (;;) {
			err = veo_hw_poll_done(dev, &done);
			if (err || done)
				break;
			if (ktime_get_ns() - dev->t_arm > timeout)
				break;
			if (READ_ONCE(dev->stopping))
				return VEO_NO_REQUEUE;
			mutex_unlock(&dev->hw_lock);
			usleep_range(2000, 3000);
			mutex_lock(&dev->hw_lock);
			if (!dev->streaming)
				return VEO_NO_REQUEUE;
		}
		now = ktime_get_ns();
		if (err || !done) {
			/* Re-arm the set that did not complete. */
			dev->state = now - dev->t_last_done > stall ?
				     VEO_ST_RESTART : VEO_ST_ARM;
			return 0;
		}
		dev->t_last_done = now;
		dev->failed_restarts = 0;
		veo_read_and_deliver(dev);
		/* Stills always use set A; preview ping-pongs. */
		if (!veo_still(dev->mode))
			dev->armed_set ^= 1;
		dev->stats_due = !(++dev->frame_count % 3);
		dev->state = VEO_ST_ARM;
		return 0;
	}
	return VEO_NO_REQUEUE;
}

static void veo_work(struct work_struct *work)
{
	struct veo_dev *dev = container_of(to_delayed_work(work),
					   struct veo_dev, work);
	unsigned long delay;

	mutex_lock(&dev->hw_lock);
	delay = veo_cycle(dev);
	mutex_unlock(&dev->hw_lock);

	if (delay != VEO_NO_REQUEUE && !READ_ONCE(dev->stopping))
		queue_delayed_work(dev->wq, &dev->work, delay);
}

/* videobuf2 */

static int veo_queue_setup(struct vb2_queue *q, unsigned int *nbuffers,
			   unsigned int *nplanes, unsigned int sizes[],
			   struct device *alloc_devs[])
{
	struct veo_dev *dev = vb2_get_drv_priv(q);
	u32 size = dev->size->sizeimage;

	if (*nplanes)
		return *nplanes != 1 || sizes[0] < size ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = size;
	return 0;
}

static int veo_buf_prepare(struct vb2_buffer *vb)
{
	struct veo_dev *dev = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < dev->size->sizeimage)
		return -EINVAL;
	return 0;
}

static void veo_buf_queue(struct vb2_buffer *vb)
{
	struct veo_dev *dev = vb2_get_drv_priv(vb->vb2_queue);
	struct veo_buf *buf = to_veo_buf(to_vb2_v4l2_buffer(vb));
	unsigned long flags;
	bool kick;

	spin_lock_irqsave(&dev->slock, flags);
	list_add_tail(&buf->list, &dev->bufs);
	kick = dev->kick && !READ_ONCE(dev->stopping);
	dev->kick = false;
	spin_unlock_irqrestore(&dev->slock, flags);

	if (kick)
		queue_delayed_work(dev->wq, &dev->work, 0);
}

static void veo_return_all(struct veo_dev *dev, enum vb2_buffer_state state)
{
	struct veo_buf *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&dev->slock, flags);
	list_for_each_entry_safe(buf, tmp, &dev->bufs, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	dev->kick = false;
	spin_unlock_irqrestore(&dev->slock, flags);
}

static int veo_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct veo_dev *dev = vb2_get_drv_priv(q);
	const struct veo_size *size = dev->size;
	const struct veo_mode *mode;
	size_t bounce_len;
	int err;

	mode = veo_size_mode(size, v4l2_ctrl_g_ctrl(dev->ctrl_zoom));
	if (!mode) {
		err = -EINVAL;
		goto err_bufs;
	}

	/*
	 * vmalloc'ed planes cannot be handed to the SDIO core, which maps the
	 * buffer with a single scatterlist entry, so the FIFO is read piecewise
	 * through a small linear chunk into a frame-sized bounce buffer, which
	 * may be vmalloc'ed: a contiguous frame-sized allocation fails on a
	 * fragmented board for the still sizes.
	 */
	bounce_len = ALIGN(size->sizeimage - VEO_JPEG_HDR_MAX + VEO_PREROLL_MAX,
			   VEO_BLKSZ);

	mutex_lock(&dev->hw_lock);
	dev->chunk = kmalloc(VEO_CHUNK, GFP_KERNEL);
	dev->bounce = kvmalloc(bounce_len, GFP_KERNEL);
	if (!dev->bounce || !dev->chunk) {
		kvfree(dev->bounce);
		kfree(dev->chunk);
		dev->bounce = NULL;
		dev->chunk = NULL;
		err = -ENOMEM;
		goto err_unlock;
	}
	dev->bounce_len = bounce_len;
	dev->sizeimage = size->sizeimage;
	dev->scan_max = size->sizeimage - VEO_JPEG_HDR_MAX;
	dev->preroll = min(READ_ONCE(preroll), VEO_PREROLL_MAX) & ~1u;
	dev->hdr_len = veo_jpeg_build_header(dev->hdr, sizeof(dev->hdr),
					     mode->out_w, mode->out_h,
					     mode->sampling,
					     dev->cfg.quant_idx,
					     dev->pixelformat ==
					     V4L2_PIX_FMT_JPEG);
	if (!dev->hdr_len) {
		err = -EINVAL;
		goto err_free;
	}

	dev->mode = mode;
	dev->sequence = 0;
	dev->frame_count = 0;
	dev->failed_restarts = 0;
	dev->cccr_misses = 0;
	dev->was_streaming = false;
	dev->interval_ns = div_u64((u64)dev->timeperframe.numerator *
				   NSEC_PER_SEC,
				   dev->timeperframe.denominator);

	if (!dev->initialised) {
		err = veo_hw_init(dev);
		if (err)
			goto err_free;
	}
	/* AE starts from where it settled last time. */
	if (!dev->cfg.ae)
		dev->cur_exposure = dev->cfg.exposure;
	dev->stats_due = false;
	err = veo_start_mode(dev);
	if (err)
		goto err_free;

	dev->t_last_done = ktime_get_ns();
	dev->streaming = true;
	WRITE_ONCE(dev->stopping, false);
	mutex_unlock(&dev->hw_lock);

	veo_ctrl_stream(dev, true);
	queue_delayed_work(dev->wq, &dev->work, 0);
	return 0;

err_free:
	kvfree(dev->bounce);
	kfree(dev->chunk);
	dev->bounce = NULL;
	dev->chunk = NULL;
err_unlock:
	mutex_unlock(&dev->hw_lock);
err_bufs:
	veo_return_all(dev, VB2_BUF_STATE_QUEUED);
	return err;
}

static void veo_stop_streaming(struct vb2_queue *q)
{
	struct veo_dev *dev = vb2_get_drv_priv(q);

	WRITE_ONCE(dev->stopping, true);
	cancel_delayed_work_sync(&dev->work);

	mutex_lock(&dev->hw_lock);
	if (dev->streaming && dev->initialised)
		veo_hw_stop(dev);
	dev->streaming = false;
	kvfree(dev->bounce);
	kfree(dev->chunk);
	dev->bounce = NULL;
	dev->chunk = NULL;
	mutex_unlock(&dev->hw_lock);

	veo_return_all(dev, VB2_BUF_STATE_ERROR);
	veo_ctrl_stream(dev, false);
}

static const struct vb2_ops veo_vb2_ops = {
	.queue_setup = veo_queue_setup,
	.buf_prepare = veo_buf_prepare,
	.buf_queue = veo_buf_queue,
	.start_streaming = veo_start_streaming,
	.stop_streaming = veo_stop_streaming,
};

/* ioctls */

static int veo_querycap(struct file *file, void *priv,
			struct v4l2_capability *cap)
{
	struct veo_dev *dev = video_drvdata(file);

	strscpy(cap->driver, VEO_DRIVER_NAME, sizeof(cap->driver));
	strscpy(cap->card, "HP Photosmart Mobile Camera", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "sdio:%s",
		 dev_name(&dev->func->dev));
	return 0;
}

static int veo_enum_input(struct file *file, void *priv,
			  struct v4l2_input *inp)
{
	if (inp->index)
		return -EINVAL;
	strscpy(inp->name, "Camera", sizeof(inp->name));
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	return 0;
}

static int veo_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int veo_s_input(struct file *file, void *priv, unsigned int i)
{
	return i ? -EINVAL : 0;
}

static int veo_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(veo_pixfmts))
		return -EINVAL;
	f->pixelformat = veo_pixfmts[f->index];
	f->flags = V4L2_FMT_FLAG_COMPRESSED;
	return 0;
}

static bool veo_valid_pixfmt(u32 pixfmt)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(veo_pixfmts); i++)
		if (veo_pixfmts[i] == pixfmt)
			return true;
	return false;
}

/* The largest size not above the request, else the smallest. */
static const struct veo_size *veo_snap_size(u32 width, u32 height)
{
	const struct veo_size *best = NULL;
	unsigned int i;

	for (i = 0; i < veo_num_sizes(); i++) {
		const struct veo_size *s = &veo_sizes[i];

		if (s->width <= width && s->height <= height &&
		    (!best || s->width * s->height > best->width * best->height))
			best = s;
	}
	return best ?: &veo_sizes[0];
}

static void veo_fill_pix(struct v4l2_pix_format *pix,
			 const struct veo_size *size, u32 pixfmt)
{
	memset(pix, 0, sizeof(*pix));
	pix->width = size->width;
	pix->height = size->height;
	pix->pixelformat = pixfmt;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = 0;
	pix->sizeimage = size->sizeimage;
	pix->colorspace = V4L2_COLORSPACE_JPEG;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_601;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static int veo_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct veo_dev *dev = video_drvdata(file);

	veo_fill_pix(&f->fmt.pix, dev->size, dev->pixelformat);
	return 0;
}

static int veo_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;
	u32 pixfmt = veo_valid_pixfmt(pix->pixelformat) ? pix->pixelformat :
		     V4L2_PIX_FMT_JPEG;

	veo_fill_pix(pix, veo_snap_size(pix->width, pix->height), pixfmt);
	return 0;
}

static const struct v4l2_fract *veo_snap_interval(const struct veo_size *size,
						  const struct v4l2_fract *want)
{
	const struct v4l2_fract *best = &size->intervals[0];
	u64 target, d, best_d = U64_MAX;
	unsigned int i;

	if (!want->numerator || !want->denominator)
		return best;
	target = div_u64((u64)want->numerator * USEC_PER_SEC,
			 want->denominator);
	for (i = 0; i < size->n_intervals; i++) {
		const struct v4l2_fract *f = &size->intervals[i];
		u64 us = div_u64((u64)f->numerator * USEC_PER_SEC,
				 f->denominator);

		d = us > target ? us - target : target - us;
		if (d < best_d) {
			best_d = d;
			best = f;
		}
	}
	return best;
}

static void veo_set_interval(struct veo_dev *dev, const struct v4l2_fract *f)
{
	dev->timeperframe = *f;
	mutex_lock(&dev->hw_lock);
	dev->interval_ns = div_u64((u64)f->numerator * NSEC_PER_SEC,
				   f->denominator);
	mutex_unlock(&dev->hw_lock);
}

static int veo_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct veo_dev *dev = video_drvdata(file);
	int err;

	err = veo_try_fmt(file, priv, f);
	if (err)
		return err;
	if (vb2_is_busy(&dev->queue))
		return -EBUSY;

	dev->size = veo_snap_size(f->fmt.pix.width, f->fmt.pix.height);
	dev->pixelformat = f->fmt.pix.pixelformat;
	veo_set_interval(dev, veo_snap_interval(dev->size, &dev->timeperframe));
	return 0;
}

static int veo_enum_framesizes(struct file *file, void *priv,
			       struct v4l2_frmsizeenum *fsize)
{
	if (!veo_valid_pixfmt(fsize->pixel_format) ||
	    fsize->index >= veo_num_sizes())
		return -EINVAL;
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = veo_sizes[fsize->index].width;
	fsize->discrete.height = veo_sizes[fsize->index].height;
	return 0;
}

static int veo_enum_frameintervals(struct file *file, void *priv,
				   struct v4l2_frmivalenum *fival)
{
	const struct veo_size *size = NULL;
	unsigned int i;

	if (!veo_valid_pixfmt(fival->pixel_format))
		return -EINVAL;
	for (i = 0; i < veo_num_sizes(); i++)
		if (veo_sizes[i].width == fival->width &&
		    veo_sizes[i].height == fival->height)
			size = &veo_sizes[i];
	if (!size || fival->index >= size->n_intervals)
		return -EINVAL;
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete = size->intervals[fival->index];
	return 0;
}

static int veo_g_parm(struct file *file, void *priv,
		      struct v4l2_streamparm *parm)
{
	struct veo_dev *dev = video_drvdata(file);

	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	memset(&parm->parm.capture, 0, sizeof(parm->parm.capture));
	parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	parm->parm.capture.timeperframe = dev->timeperframe;
	parm->parm.capture.readbuffers = 2;
	return 0;
}

/* The rate is a target; the hardware cycle caps it, timestamps tell. */
static int veo_s_parm(struct file *file, void *priv,
		      struct v4l2_streamparm *parm)
{
	struct veo_dev *dev = video_drvdata(file);
	const struct v4l2_fract *tpf;

	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	tpf = veo_snap_interval(dev->size, &parm->parm.capture.timeperframe);
	veo_set_interval(dev, tpf);
	return veo_g_parm(file, priv, parm);
}

static const struct v4l2_ioctl_ops veo_ioctl_ops = {
	.vidioc_querycap = veo_querycap,
	.vidioc_enum_input = veo_enum_input,
	.vidioc_g_input = veo_g_input,
	.vidioc_s_input = veo_s_input,
	.vidioc_enum_fmt_vid_cap = veo_enum_fmt,
	.vidioc_g_fmt_vid_cap = veo_g_fmt,
	.vidioc_try_fmt_vid_cap = veo_try_fmt,
	.vidioc_s_fmt_vid_cap = veo_s_fmt,
	.vidioc_enum_framesizes = veo_enum_framesizes,
	.vidioc_enum_frameintervals = veo_enum_frameintervals,
	.vidioc_g_parm = veo_g_parm,
	.vidioc_s_parm = veo_s_parm,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,

	.vidioc_log_status = v4l2_ctrl_log_status,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_file_operations veo_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.read = vb2_fop_read,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
	.unlocked_ioctl = video_ioctl2,
};

/* SDIO driver */

static void veo_release(struct v4l2_device *v4l2_dev)
{
	struct veo_dev *dev = container_of(v4l2_dev, struct veo_dev, v4l2_dev);

	v4l2_ctrl_handler_free(&dev->hdl);
	v4l2_device_unregister(&dev->v4l2_dev);
	destroy_workqueue(dev->wq);
	kfree(dev->fw);
	kfree(dev);
}

static int veo_probe(struct sdio_func *func, const struct sdio_device_id *id)
{
	struct mmc_host *host = func->card->host;
	struct video_device *vdev;
	struct veo_dev *dev;
	struct vb2_queue *q;
	u8 strap;
	int err;

	dev = kzalloc_obj(*dev);
	if (!dev)
		return -ENOMEM;
	dev->func = func;
	mutex_init(&dev->lock);
	mutex_init(&dev->hw_lock);
	spin_lock_init(&dev->slock);
	INIT_LIST_HEAD(&dev->bufs);
	INIT_DELAYED_WORK(&dev->work, veo_work);
	dev->stopping = true;
	dev->cfg.exposure = VEO_EXPOSURE_DEF;
	dev->cfg.red = VEO_GAIN_DEF;
	dev->cfg.blue = VEO_GAIN_DEF;
	dev->cur_exposure = VEO_EXPOSURE_DEF;
	dev->cur_red = VEO_GAIN_DEF;
	dev->cur_blue = VEO_GAIN_DEF;

	err = veo_hw_link_check(dev, &strap);
	if (err) {
		dev_err(&func->dev, "card does not answer (%d)\n", err);
		goto err_free;
	}
	dev_dbg(&func->dev, "bus width %u, clock %u Hz, strap %02x\n",
		1 << host->ios.bus_width, host->ios.clock, strap);
	if (host->ios.bus_width != MMC_BUS_WIDTH_1)
		dev_warn_once(&func->dev,
			      "host bus is %u bits wide, this card only drives DAT0\n",
			      1 << host->ios.bus_width);

	dev->wq = alloc_ordered_workqueue(VEO_DRIVER_NAME, 0);
	if (!dev->wq) {
		err = -ENOMEM;
		goto err_disable;
	}

	err = v4l2_device_register(&func->dev, &dev->v4l2_dev);
	if (err)
		goto err_wq;
	dev->v4l2_dev.release = veo_release;

	/* From here on veo_release() frees everything. */
	err = veo_ctrl_init(dev);
	if (err)
		goto err_put;
	dev->v4l2_dev.ctrl_handler = &dev->hdl;

	dev->pixelformat = V4L2_PIX_FMT_JPEG;
	dev->size = &veo_sizes[0];
	dev->timeperframe = veo_sizes[0].intervals[0];

	q = &dev->queue;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF | VB2_READ;
	q->dev = &func->dev;
	q->drv_priv = dev;
	q->buf_struct_size = sizeof(struct veo_buf);
	q->ops = &veo_vb2_ops;
	q->mem_ops = &vb2_vmalloc_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
			     V4L2_BUF_FLAG_TSTAMP_SRC_EOF;
	q->min_queued_buffers = 2;
	q->lock = &dev->lock;
	err = vb2_queue_init(q);
	if (err)
		goto err_put;

	vdev = &dev->vdev;
	strscpy(vdev->name, "HP Photosmart Mobile Camera", sizeof(vdev->name));
	vdev->v4l2_dev = &dev->v4l2_dev;
	vdev->fops = &veo_fops;
	vdev->ioctl_ops = &veo_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->lock = &dev->lock;
	vdev->queue = q;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_READWRITE;
	video_set_drvdata(vdev, dev);
	sdio_set_drvdata(func, dev);

	err = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (err)
		goto err_put;

	dev_info(&func->dev, "registered %s\n", video_device_node_name(vdev));
	return 0;

err_put:
	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);
	v4l2_device_put(&dev->v4l2_dev);
	return err;
err_wq:
	destroy_workqueue(dev->wq);
err_disable:
	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);
err_free:
	kfree(dev);
	return err;
}

static void veo_remove(struct sdio_func *func)
{
	struct veo_dev *dev = sdio_get_drvdata(func);

	/* Stops a running stream; open files keep the state until closed. */
	vb2_video_unregister_device(&dev->vdev);
	cancel_delayed_work_sync(&dev->work);
	v4l2_device_disconnect(&dev->v4l2_dev);

	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);

	v4l2_device_put(&dev->v4l2_dev);
}

static int veo_suspend(struct device *d)
{
	struct veo_dev *dev = sdio_get_drvdata(dev_to_sdio_func(d));

	WRITE_ONCE(dev->stopping, true);
	cancel_delayed_work_sync(&dev->work);

	mutex_lock(&dev->hw_lock);
	dev->was_streaming = dev->streaming;
	if (dev->streaming)
		veo_hw_stop(dev);
	if (dev->initialised)
		veo_hw_standby(dev);
	mutex_unlock(&dev->hw_lock);
	return 0;
}

/* The card state is gone after a suspend; re-initialise from the work. */
static int veo_resume(struct device *d)
{
	struct veo_dev *dev = sdio_get_drvdata(dev_to_sdio_func(d));

	mutex_lock(&dev->hw_lock);
	dev->initialised = false;
	if (dev->was_streaming) {
		dev->state = VEO_ST_RESTART;
		WRITE_ONCE(dev->stopping, false);
		queue_delayed_work(dev->wq, &dev->work, 0);
	}
	mutex_unlock(&dev->hw_lock);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(veo_pm_ops, veo_suspend, veo_resume);

static const struct sdio_device_id veo_ids[] = {
	{ SDIO_DEVICE(SDIO_VENDOR_ID_VEO, SDIO_DEVICE_ID_VEO_CAMERA) },
	{ }
};
MODULE_DEVICE_TABLE(sdio, veo_ids);

static struct sdio_driver veo_driver = {
	.name = VEO_DRIVER_NAME,
	.id_table = veo_ids,
	.probe = veo_probe,
	.remove = veo_remove,
	.drv = {
		.pm = pm_sleep_ptr(&veo_pm_ops),
	},
};

module_sdio_driver(veo_driver);

MODULE_FIRMWARE(VEO_FW_NAME);
MODULE_DESCRIPTION("HP Photosmart Mobile / Veo SDIO camera driver");
MODULE_AUTHOR("Koen Kooi <koen@dominion.thruhere.net>");
MODULE_LICENSE("GPL");
