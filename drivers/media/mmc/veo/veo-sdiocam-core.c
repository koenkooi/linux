// SPDX-License-Identifier: GPL-2.0-only
/*
 * HP Photosmart Mobile / Veo SDIO camera: bridge protocol.
 *
 * Every register of the camera ASIC is a CMD52 on function 1 at
 * 0x10000 | reg; function 1 address 5 is a bridge control byte and address 7
 * the data FIFO. The image sensor sits behind a serial engine in the ASIC.
 * Nothing here knows about V4L2.
 *
 * The SDIO host is claimed for each register burst and released across
 * every sleep, so a slow card never holds the bus while it is polled.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/slab.h>

#include "veo-sdiocam.h"

#define VEO_REG(r)		(0x10000 | (r))
#define VEO_CTRL_ADDR		5
#define VEO_FIFO_ADDR		7
#define VEO_SENSOR_W		0x42
#define VEO_SENSOR_R		0x43

#define VEO_ENGINE_TIMEOUT_MS	121

static void veo_wr(struct veo_dev *dev, u8 reg, u8 val, int *err)
{
	if (!*err)
		sdio_writeb(dev->func, val, VEO_REG(reg), err);
}

static u8 veo_rd(struct veo_dev *dev, u8 reg, int *err)
{
	if (*err)
		return 0;
	return sdio_readb(dev->func, VEO_REG(reg), err);
}

static void veo_rmw(struct veo_dev *dev, u8 reg, u8 clear, u8 set, int *err)
{
	u8 val = veo_rd(dev, reg, err);

	veo_wr(dev, reg, (val & ~clear) | set, err);
}

static void veo_wr_pairs(struct veo_dev *dev, const struct veo_pair *p,
			 unsigned int n, int *err)
{
	while (n-- && !*err) {
		veo_wr(dev, p->reg, p->val, err);
		p++;
	}
}

/* Sleep with the host released; the caller holds it before and after. */
static void veo_sleep_us(struct veo_dev *dev, unsigned int min,
			 unsigned int max)
{
	sdio_release_host(dev->func);
	usleep_range(min, max);
	sdio_claim_host(dev->func);
}

static void veo_sleep_ms(struct veo_dev *dev, unsigned int ms)
{
	sdio_release_host(dev->func);
	msleep(ms);
	sdio_claim_host(dev->func);
}

u32 veo_jpeg_base(unsigned int set)
{
	return set ? VEO_JPEG_B : VEO_JPEG_A;
}

u32 veo_raw_base(unsigned int set)
{
	return set ? VEO_RAW_B : VEO_RAW_A;
}

/* Sensor serial engine: register 0x0d bit 7 is busy, 0x24 the command. */

static int veo_ser_idle(struct veo_dev *dev)
{
	unsigned int i;
	int err = 0;
	u8 busy;

	for (i = 0; i <= 100; i++) {
		busy = veo_rd(dev, 0x0d, &err);
		if (err)
			return err;
		if (!(busy & 0x80))
			return 0;
		veo_sleep_us(dev, 1000, 2000);
	}
	return -ETIMEDOUT;
}

static int veo_ser_cmd(struct veo_dev *dev, u8 cmd)
{
	int err = veo_ser_idle(dev);

	veo_wr(dev, 0x24, cmd, &err);
	return err;
}

/* The final 0x0c clocks the acknowledge; the slave's ACK is never checked. */
static int veo_ser_tx(struct veo_dev *dev, u8 byte)
{
	int err = 0;

	veo_wr(dev, 0x27, byte, &err);
	veo_wr(dev, 0x24, 0x08, &err);
	if (!err)
		err = veo_ser_idle(dev);
	veo_wr(dev, 0x27, 0xff, &err);
	veo_wr(dev, 0x24, 0x0c, &err);
	if (!err)
		err = veo_ser_idle(dev);
	return err;
}

static int veo_ser_rx(struct veo_dev *dev, u8 *byte)
{
	int err = veo_ser_cmd(dev, 0x0e);

	if (!err)
		err = veo_ser_idle(dev);
	veo_wr(dev, 0x27, 0xff, &err);
	veo_wr(dev, 0x24, 0x0c, &err);
	if (!err)
		err = veo_ser_idle(dev);
	*byte = veo_rd(dev, 0x26, &err);
	return err;
}

static int veo_sensor_write(struct veo_dev *dev, u8 reg, u8 val)
{
	int err;

	err = veo_ser_cmd(dev, 0x09);
	if (!err)
		err = veo_ser_tx(dev, VEO_SENSOR_W);
	if (!err)
		err = veo_ser_tx(dev, reg);
	if (!err)
		err = veo_ser_tx(dev, val);
	if (!err)
		err = veo_ser_cmd(dev, 0x0a);
	return err;
}

static int veo_sensor_read(struct veo_dev *dev, u8 reg, u8 *val)
{
	int err;

	err = veo_ser_cmd(dev, 0x09);
	if (!err)
		err = veo_ser_tx(dev, VEO_SENSOR_W);
	if (!err)
		err = veo_ser_tx(dev, reg);
	if (!err)
		err = veo_ser_cmd(dev, 0x09);
	if (!err)
		err = veo_ser_tx(dev, VEO_SENSOR_R);
	if (!err)
		err = veo_ser_rx(dev, val);
	if (!err)
		err = veo_ser_cmd(dev, 0x0a);
	return err;
}

static int veo_sensor_set_bits(struct veo_dev *dev, u8 reg, u8 set)
{
	u8 val;
	int err = veo_sensor_read(dev, reg, &val);

	if (err)
		return err;
	return veo_sensor_write(dev, reg, val | set);
}

static u16 veo_exposure_reg(u8 class, u16 exposure)
{
	u32 reg;

	switch (class) {
	case VEO_CLASS_FULL:
		reg = exposure >> 1;
		break;
	case VEO_CLASS_HALF:
		reg = exposure;
		break;
	default:
		reg = exposure << 1;
		break;
	}
	return min_t(u32, reg, VEO_EXPOSURE_REG_MAX);
}

/* @exposure is in half-class lines; the register counts the class's lines. */
static int __veo_set_exposure(struct veo_dev *dev, const struct veo_mode *mode,
			      u16 exposure)
{
	u16 reg = veo_exposure_reg(mode->class, exposure);
	int err;

	err = veo_sensor_write(dev, 0x1c, reg & 0xff);
	if (!err)
		err = veo_sensor_write(dev, 0x1d, reg >> 8);
	if (!err)
		err = veo_sensor_set_bits(dev, 0x00, 0x84);
	return err;
}

int veo_hw_set_exposure(struct veo_dev *dev, const struct veo_mode *mode,
			u16 exposure)
{
	int err;

	sdio_claim_host(dev->func);
	err = __veo_set_exposure(dev, mode, exposure);
	sdio_release_host(dev->func);
	return err;
}

/*
 * The four gains are {blue, green, green, red} in registers 0x20-0x27 as
 * LE16. The orientation strap rotates them by one pair, following the
 * sensor readout flip it selects.
 */
static int __veo_set_gains(struct veo_dev *dev, u16 red, u16 blue)
{
	static const u8 plain[4] = { 0x20, 0x22, 0x24, 0x26 };
	static const u8 flipped[4] = { 0x24, 0x20, 0x26, 0x22 };
	const u8 *regs = dev->bayer ? flipped : plain;
	u16 words[4] = { blue, 0x100, 0x100, red };
	unsigned int i;
	int err = 0;

	for (i = 0; i < 4 && !err; i++) {
		err = veo_sensor_write(dev, regs[i], words[i] & 0xff);
		if (!err)
			err = veo_sensor_write(dev, regs[i] + 1, words[i] >> 8);
	}
	if (!err)
		err = veo_sensor_set_bits(dev, 0x00, 0x80);
	return err;
}

int veo_hw_set_gains(struct veo_dev *dev, u16 red, u16 blue)
{
	int err;

	sdio_claim_host(dev->func);
	err = __veo_set_gains(dev, red, blue);
	sdio_release_host(dev->func);
	return err;
}

static u8 veo_band_value(unsigned int d)
{
	unsigned int i;

	for (i = 0; i < VEO_BAND_LIMITS; i++)
		if (d < veo_band_limits[i])
			return 0x30 + i;
	return d > 0x3f1 ? 0x3f : 0x3e;
}

/* Band filter step from two free-running sensor counters. */
static int veo_band_filter(struct veo_dev *dev)
{
	unsigned int a, b;
	u8 hi, lo;
	int err;

	err = veo_sensor_write(dev, 0x91, 0x38);
	if (!err)
		err = veo_sensor_write(dev, 0x00, 0x02);
	if (!err)
		err = veo_sensor_read(dev, 0x47, &hi);
	if (!err)
		err = veo_sensor_read(dev, 0x46, &lo);
	if (err)
		return err;
	a = hi << 8 | lo;
	err = veo_sensor_read(dev, 0x49, &hi);
	if (!err)
		err = veo_sensor_read(dev, 0x48, &lo);
	if (err)
		return err;
	b = hi << 8 | lo;

	if (a && b && a < b) {
		err = veo_sensor_write(dev, 0x91, veo_band_value(b - a));
		if (!err)
			err = veo_sensor_write(dev, 0x00, 0x88);
		return err;
	}
	return veo_sensor_write(dev, 0x91, 0x38);
}

static void veo_clock_settle(struct veo_dev *dev, int *err)
{
	veo_wr(dev, 0x65, 0x38, err);
	veo_sleep_ms(dev, 20);
	veo_wr(dev, 0x65, 0x3c, err);
	veo_sleep_ms(dev, 20);
}

static void veo_bridge_defaults(struct veo_dev *dev, int *err)
{
	unsigned int i;

	veo_wr(dev, 0x9f, 0x01, err);
	veo_wr(dev, 0x1a, 0xf1, err);
	veo_wr(dev, 0x1b, 0x02, err);
	veo_wr(dev, 0x1c, 0x00, err);
	veo_wr(dev, 0x12, 0x01, err);
	veo_sleep_us(dev, 5000, 6000);
	veo_wr(dev, 0x5c, 0x00, err);
	veo_wr(dev, 0x2c, 0x01, err);
	veo_wr(dev, 0x64, 0x0f, err);
	veo_wr(dev, 0x48, 0x4f, err);
	veo_wr(dev, 0x47, 0x00, err);
	veo_wr(dev, 0x34, 0x01, err);
	veo_wr(dev, 0x65, 0x0c, err);
	veo_wr(dev, 0x5d, 0x00, err);
	veo_wr(dev, 0xa6, 0x00, err);
	veo_rd(dev, 0x5e, err);
	veo_rd(dev, 0xa7, err);

	veo_wr_pairs(dev, veo_ind_script1, ARRAY_SIZE(veo_ind_script1), err);
	veo_wr(dev, 0x22, 0x90, err);
	veo_wr(dev, 0x21, 0x54, err);
	veo_wr(dev, 0x20, 0x03, err);
	veo_wr(dev, 0x22, 0xa0, err);
	for (i = 0; i < VEO_IND_TABLE_LEN; i++)
		veo_wr(dev, 0x21, veo_ind_table[i], err);
	veo_wr_pairs(dev, veo_ind_script3, ARRAY_SIZE(veo_ind_script3), err);
}

static int veo_load_firmware(struct veo_dev *dev)
{
	const struct firmware *fw;
	int err;

	if (dev->fw)
		return 0;

	err = request_firmware(&fw, VEO_FW_NAME, &dev->func->dev);
	if (err) {
		dev_err(&dev->func->dev, "cannot load %s (%d)\n", VEO_FW_NAME,
			err);
		return err;
	}
	if (fw->size != VEO_FW_LEN) {
		dev_err(&dev->func->dev, "%s is %zu bytes, expected %u\n",
			VEO_FW_NAME, fw->size, VEO_FW_LEN);
		err = -EINVAL;
	} else {
		/* Kept so that a re-init after resume needs no file system. */
		dev->fw = kmemdup(fw->data, fw->size, GFP_KERNEL);
		if (!dev->fw)
			err = -ENOMEM;
	}
	release_firmware(fw);
	return err;
}

int veo_hw_link_check(struct veo_dev *dev, u8 *strap)
{
	int err;

	sdio_claim_host(dev->func);
	err = sdio_enable_func(dev->func);
	if (!err)
		err = sdio_set_block_size(dev->func, VEO_BLKSZ);
	if (!err)
		*strap = veo_rd(dev, 0x13, &err);
	sdio_release_host(dev->func);
	return err;
}

/*
 * The factory initialisation, in the factory order: about 3 s, most of it
 * the 99 sensor writes through the serial engine.
 */
int veo_hw_init(struct veo_dev *dev)
{
	struct sdio_func *func = dev->func;
	unsigned int i;
	int err;
	u8 val;

	dev->initialised = false;

	err = veo_load_firmware(dev);
	if (err)
		return err;

	sdio_claim_host(func);

	err = sdio_enable_func(func);
	if (!err)
		err = sdio_set_block_size(func, VEO_BLKSZ);
	if (err)
		goto out;

	/* Bridge control byte: function 1 address 5, outside the 0x10000 window. */
	val = sdio_readb(func, VEO_CTRL_ADDR, &err);
	if (!err)
		sdio_writeb(func, val | 0x19, VEO_CTRL_ADDR, &err);
	if (!err)
		val = sdio_readb(func, VEO_CTRL_ADDR, &err);
	if (!err)
		sdio_writeb(func, val & ~0x04, VEO_CTRL_ADDR, &err);
	if (!err)
		sdio_writeb(func, val | 0x04, VEO_CTRL_ADDR, &err);
	if (err)
		goto out;
	veo_sleep_us(dev, 5000, 6000);

	veo_bridge_defaults(dev, &err);
	veo_clock_settle(dev, &err);
	val = veo_rd(dev, 0x1a, &err);
	if (err)
		goto out;
	if (val != 0xf1) {
		dev_err(&func->dev, "bridge register 0x1a reads %02x, not f1\n",
			val);
		err = -ENODEV;
		goto out;
	}

	veo_wr(dev, 0x84, 0x0f, &err);
	veo_wr(dev, 0x88, 0x00, &err);
	for (i = 0; i < VEO_FW_LEN && !err; i++)
		veo_wr(dev, 0x89, dev->fw[i], &err);
	veo_wr(dev, 0x84, 0x00, &err);
	veo_wr(dev, 0x88, 0x00, &err);
	veo_clock_settle(dev, &err);

	val = veo_rd(dev, 0x13, &err);
	if (err)
		goto out;
	dev->bayer = val & 1;

	for (i = 0; i < VEO_SENSOR_INIT_LEN && !err; i++)
		err = veo_sensor_write(dev, veo_sensor_init[i].reg,
				       veo_sensor_init[i].val);
	if (!err)
		err = veo_sensor_write(dev, 0x11, dev->bayer ? 0x10 : 0x08);
	if (!err)
		err = veo_sensor_write(dev, 0x0e, 0x98);
	if (!err)
		err = veo_sensor_write(dev, 0x0f, 0x08);
	if (!err)
		err = veo_sensor_set_bits(dev, 0x00, 0x80);
	if (!err)
		err = __veo_set_gains(dev, dev->cur_red, dev->cur_blue);
	if (!err)
		err = veo_band_filter(dev);
	if (err)
		goto out;
	veo_sleep_us(dev, 5000, 6000);
	dev->initialised = true;
out:
	sdio_release_host(func);
	if (err)
		dev_err(&func->dev, "initialisation failed (%d)\n", err);
	return err;
}

/* Read 0x5e and 0xa7 together and latch them: they may clear on read. */
static void veo_latch_irq(struct veo_dev *dev, int *err)
{
	dev->irq5e |= veo_rd(dev, 0x5e, err);
	dev->irqa7 |= veo_rd(dev, 0xa7, err);
}

static void veo_irq_disable(struct veo_dev *dev, u8 mask, int *err)
{
	veo_rmw(dev, 0x5d, mask, 0, err);
	veo_rd(dev, 0x5e, err);
}

static void veo_irq_enable(struct veo_dev *dev, u8 mask, int *err)
{
	veo_rmw(dev, 0x5d, 0, mask, err);
	veo_rd(dev, 0x5e, err);
}

static void veo_pulse_1a(struct veo_dev *dev, int *err)
{
	u8 val = veo_rd(dev, 0x1a, err);

	veo_wr(dev, 0x1a, val & 0x8f, err);
	veo_wr(dev, 0x1a, val | 0x70, err);
}

static void veo_set_buffers(struct veo_dev *dev, unsigned int set, int *err)
{
	u32 raw = veo_raw_base(set), jpeg = veo_jpeg_base(set);

	veo_wr(dev, 0x4e, raw & 0xff, err);
	veo_wr(dev, 0x4f, (raw >> 8) & 0xff, err);
	veo_wr(dev, 0x50, raw >> 16, err);
	veo_wr(dev, 0x3d, raw & 0xff, err);
	veo_wr(dev, 0x3e, (raw >> 8) & 0xff, err);
	veo_wr(dev, 0x3f, raw >> 16, err);
	veo_wr(dev, 0x51, jpeg & 0xff, err);
	veo_wr(dev, 0x52, (jpeg >> 8) & 0xff, err);
	veo_wr(dev, 0x53, jpeg >> 16, err);
}

static void veo_load_quant(struct veo_dev *dev, unsigned int idx, int *err)
{
	const u8 *tab = veo_chip_quant[idx];
	unsigned int i, spin;

	veo_rmw(dev, 0xc0, 0x20, 0, err);
	for (i = 0; i < VEO_CHIP_QUANT_LEN && !*err; i++) {
		veo_wr(dev, 0xc2, tab[i], err);
		for (spin = 0; spin < 200 && !*err; spin++) {
			if (!(veo_rd(dev, 0xc0, err) & 0x10))
				break;
			if (spin >= 8)
				veo_sleep_us(dev, 100, 200);
		}
		if (spin == 200)
			*err = -ETIMEDOUT;
	}
}

static void veo_write_window(struct veo_dev *dev, const struct veo_mode *mode,
			     int *err)
{
	u16 x = mode->x, y = mode->y;

	/* A flipped readout keeps its Bayer phase with a one-pixel shift. */
	if (dev->bayer) {
		if (mode->id != 0x05 && mode->id != 0x06 && mode->id != 0x0b)
			x--;
		y++;
	}
	veo_wr(dev, 0x02, x & 0xff, err);
	veo_wr(dev, 0x03, x >> 8, err);
	veo_wr(dev, 0x04, mode->w & 0xff, err);
	veo_wr(dev, 0x05, mode->w >> 8, err);
	veo_wr(dev, 0x06, y & 0xff, err);
	veo_wr(dev, 0x07, y >> 8, err);
	veo_wr(dev, 0x08, mode->h & 0xff, err);
	veo_wr(dev, 0x09, mode->h >> 8, err);
}

static void veo_write_jpeg_setup(struct veo_dev *dev,
				 const struct veo_mode *mode, int *err)
{
	unsigned int i;

	veo_wr(dev, 0x5a, mode->scaler_w & 0xff, err);
	veo_wr(dev, 0x5b, mode->scaler_w >> 8, err);
	for (i = 0; i < 4; i++)
		veo_wr(dev, 0x70 + i, mode->isp[i], err);
	veo_wr(dev, 0x75, mode->jin_w16, err);
	veo_wr(dev, 0x76, mode->jin_h16, err);
	veo_wr(dev, 0xc8, mode->out_w & 0xff, err);
	veo_wr(dev, 0xc9, mode->out_w >> 8, err);
	veo_wr(dev, 0xca, mode->out_h & 0xff, err);
	veo_wr(dev, 0xcb, mode->out_h >> 8, err);
	veo_wr(dev, 0x4d, mode->fmt4d, err);
	veo_load_quant(dev, dev->cfg.quant_idx, err);
	veo_wr(dev, 0xc0, 0x02, err);
}

static int veo_set_class(struct veo_dev *dev, u8 class)
{
	static const struct veo_pair regs[] = {
		[VEO_CLASS_FULL] = { 0x00, 0x80 },
		[VEO_CLASS_HALF] = { 0x0d, 0x88 },
		[VEO_CLASS_QUARTER] = { 0x0c, 0x98 },
	};
	int err = veo_sensor_write(dev, 0x52, regs[class].reg);

	if (!err)
		err = veo_sensor_write(dev, 0x00, regs[class].val);
	return err;
}

static int veo_sensor_mode(struct veo_dev *dev, const struct veo_mode *mode)
{
	int err;

	err = veo_sensor_write(dev, 0x0e, 0x98);
	if (!err)
		err = veo_sensor_write(dev, 0x0f, 0x08);
	if (!err)
		err = __veo_set_exposure(dev, mode, dev->cur_exposure);
	if (!err)
		err = veo_set_class(dev, mode->class);
	return err;
}

/* The factory preview start: ends with buffer set A armed. */
int veo_hw_mode_start(struct veo_dev *dev, const struct veo_mode *mode)
{
	int err = 0;

	sdio_claim_host(dev->func);
	veo_irq_disable(dev, 0xff, &err);
	veo_wr(dev, 0x90, 0x00, &err);
	veo_wr(dev, 0x5c, 0x00, &err);
	veo_pulse_1a(dev, &err);
	if (err)
		goto out;
	veo_sleep_ms(dev, 50);

	err = veo_sensor_mode(dev, mode);
	if (err)
		goto out;
	veo_sleep_ms(dev, 50);

	veo_write_window(dev, mode, &err);
	veo_wr(dev, 0xc1, 0x00, &err);
	veo_set_buffers(dev, 0, &err);
	veo_write_jpeg_setup(dev, mode, &err);
	veo_irq_enable(dev, 0x02, &err);
	dev->irq5e = 0;
	dev->irqa7 = 0;
	veo_wr(dev, 0x5c, 0x03, &err);
	dev->armed_set = 0;
	dev->t_arm = ktime_get_ns();
out:
	sdio_release_host(dev->func);
	return err;
}

/* Start capture and compression into @set. */
int veo_hw_arm(struct veo_dev *dev, unsigned int set)
{
	int err = 0;

	sdio_claim_host(dev->func);
	veo_wr(dev, 0x5c, 0x00, &err);
	veo_set_buffers(dev, set, &err);
	dev->irq5e = 0;
	dev->irqa7 = 0;
	veo_rmw(dev, 0x5c, 0, 0x01, &err);
	veo_rmw(dev, 0x5c, 0, 0x02, &err);
	sdio_release_host(dev->func);
	dev->armed_set = set;
	dev->t_arm = ktime_get_ns();
	return err;
}

/*
 * The card interrupt is never claimed: the host on the h2200 has none, and
 * the factory driver polled the function 1 pending bit in CCCR 0x05 without
 * enabling it. Whether that bit latches with IENx clear is not known, so
 * every fifth miss reads 0x5e directly and, if the JPEG is done there while
 * CCCR shows nothing, polls 0x5e from then on.
 */
int veo_hw_poll_done(struct veo_dev *dev, bool *done)
{
	int err = 0;
	u8 pending;

	*done = false;
	sdio_claim_host(dev->func);
	if (dev->cccr_fallback) {
		veo_latch_irq(dev, &err);
	} else {
		pending = sdio_f0_readb(dev->func, SDIO_CCCR_INTx, &err);
		if (!err && (pending & BIT(1))) {
			veo_latch_irq(dev, &err);
		} else if (!err && !(++dev->cccr_misses % 5)) {
			veo_latch_irq(dev, &err);
			if (!err && (dev->irq5e & 0x02)) {
				dev->cccr_fallback = true;
				dev_info(&dev->func->dev,
					 "CCCR pending bit does not latch, polling the bridge\n");
			}
		}
	}
	sdio_release_host(dev->func);
	*done = !err && (dev->irq5e & 0x02);
	return err;
}

int veo_hw_frame_len(struct veo_dev *dev, u32 *len)
{
	int err = 0;
	u8 lo, mid, hi;

	sdio_claim_host(dev->func);
	lo = veo_rd(dev, 0x54, &err);
	mid = veo_rd(dev, 0x55, &err);
	hi = veo_rd(dev, 0x56, &err);
	sdio_release_host(dev->func);
	*len = hi << 16 | mid << 8 | lo;
	return err;
}

static void veo_engine_addr(struct veo_dev *dev, u32 src, u32 len, int *err)
{
	u32 half = (len + 1) >> 1;

	veo_wr(dev, 0xda, src & 0xff, err);
	veo_wr(dev, 0xdb, (src >> 8) & 0xff, err);
	veo_wr(dev, 0xdc, src >> 16, err);
	veo_wr(dev, 0xdd, half & 0xff, err);
	veo_wr(dev, 0xde, (half >> 8) & 0xff, err);
	veo_wr(dev, 0xdf, half >> 16, err);
}

static void veo_engine_go(struct veo_dev *dev, u8 last, int *err)
{
	veo_wr(dev, 0x84, 0x00, err);
	veo_wr(dev, 0x88, 0x00, err);
	veo_wr(dev, 0x86, 0x00, err);
	veo_wr(dev, 0x85, 0x08, err);
	veo_wr(dev, 0x85, last, err);
	veo_wr(dev, 0x84, 0x10, err);
}

/* Returns the done byte, which is also the first statistics byte. */
static u8 veo_engine_wait(struct veo_dev *dev, unsigned int timeout_ms,
			  int *err)
{
	unsigned long end = jiffies + msecs_to_jiffies(timeout_ms);
	u8 val = 0;

	while (!*err) {
		veo_wr(dev, 0x87, 0x13, err);
		val = veo_rd(dev, 0x85, err);
		if (*err || (val & 1))
			break;
		if (time_after(jiffies, end)) {
			*err = -ETIMEDOUT;
			break;
		}
		veo_sleep_us(dev, 500, 1000);
	}
	return val;
}

/*
 * Engine command 0x21 streams @len bytes from ASIC memory at @src into the
 * data FIFO; one fixed-address CMD53 of @xfer bytes (a multiple of the
 * block size) then drains it, as the factory driver does.
 */
int veo_hw_read_frame(struct veo_dev *dev, u32 src, u32 len, size_t xfer)
{
	int err = 0, rerr;
	size_t done, n;

	sdio_claim_host(dev->func);
	veo_wr(dev, 0x69, 0x04, &err);
	veo_wr(dev, 0x8a, 0x01, &err);
	veo_engine_addr(dev, src, len, &err);
	veo_wr(dev, 0x90, 0x20, &err);
	veo_wr(dev, 0x61, 0xff, &err);
	veo_wr(dev, 0x86, 0x10, &err);
	veo_wr(dev, 0x85, 0x21, &err);
	veo_engine_go(dev, 0x00, &err);
	if (err)
		goto out;

	/*
	 * The SDIO core maps one request with a single scatterlist entry,
	 * so each piece goes through a small physically contiguous buffer;
	 * the whole frame would need a 512 KiB contiguous allocation that
	 * fails on a fragmented 53 MB board. The core splits a long read
	 * into CMD53s of this size anyway, so the bus sees the same
	 * transfers.
	 */
	rerr = 0;
	for (done = 0; !rerr && done < xfer; done += n) {
		n = min_t(size_t, xfer - done, VEO_CHUNK);
		rerr = sdio_readsb(dev->func, dev->chunk, VEO_FIFO_ADDR, n);
		memcpy(dev->bounce + done, dev->chunk, n);
	}

	veo_engine_wait(dev, VEO_ENGINE_TIMEOUT_MS, &err);
	/*
	 * Clean up even after a timeout so the next command starts idle,
	 * and let the frame check judge the data.
	 */
	if (err == -ETIMEDOUT)
		err = 0;
	veo_wr(dev, 0x90, 0x00, &err);
	veo_wr(dev, 0xa6, 0x00, &err);
	veo_wr(dev, 0x61, 0x00, &err);
	veo_wr(dev, 0x8a, 0x00, &err);
	veo_wr(dev, 0x84, 0x00, &err);
	if (rerr)
		err = rerr;
out:
	sdio_release_host(dev->func);
	return err;
}

/*
 * Engine command with parameters: A (@src) in 0xda-0xdc, B (@dst) in
 * 0xe0-0xe2, parameters through the index/data pair 0x86/0x85, the
 * command at index 0x10 followed by its flag byte. The factory driver's
 * last trailer byte is 1 for the statistics commands and 0 otherwise.
 */
static void veo_engine_cmd(struct veo_dev *dev, u8 cmd, u32 src, u32 dst,
			   u32 len, const u8 *params, u8 flag, int *err)
{
	unsigned int i;

	veo_engine_addr(dev, src, len, err);
	veo_wr(dev, 0xe0, dst & 0xff, err);
	veo_wr(dev, 0xe1, (dst >> 8) & 0xff, err);
	veo_wr(dev, 0xe2, dst >> 16, err);
	veo_wr(dev, 0x90, 0x20, err);
	veo_wr(dev, 0x84, 0x03, err);
	veo_wr(dev, 0x88, 0x00, err);
	if (params) {
		veo_wr(dev, 0x86, 0x01, err);
		for (i = 0; i < 8; i++)
			veo_wr(dev, 0x85, params[i], err);
		veo_wr(dev, 0x84, 0x03, err);
		veo_wr(dev, 0x88, 0x00, err);
	}
	veo_wr(dev, 0x86, 0x10, err);
	veo_wr(dev, 0x85, cmd, err);
	veo_wr(dev, 0x85, flag, err);
	veo_engine_go(dev, params ? 0x01 : 0x00, err);
}

/* The factory's fix-up of the packed statistics bytes; its meaning is unknown. */
static void veo_engine_results(struct veo_dev *dev, u8 first, u8 st[3],
			       int *err)
{
	u8 g, r;

	veo_wr(dev, 0x87, 0x14, err);
	g = veo_rd(dev, 0x85, err);
	veo_wr(dev, 0x87, 0x15, err);
	r = veo_rd(dev, 0x85, err);
	st[0] = (first & 0xe0) + (first & 0x0f) * 2;
	st[1] = (g & 0xf0) + (g & 0x07) * 2;
	st[2] = (r & 0xe0) + (r & 0x0f) * 2;
}

static void veo_engine_idle(struct veo_dev *dev, int *err)
{
	int e = 0;

	veo_wr(dev, 0x90, 0x00, &e);
	veo_wr(dev, 0xa6, 0x00, &e);
	veo_wr(dev, 0x84, 0x00, &e);
	if (!*err)
		*err = e;
}

/*
 * Statistics over the raw frame at @raw into a 40 x 30 cell grid at
 * VEO_STATS_BUF; returns three bytes, which behave as blue, green and red.
 */
int veo_hw_stats(struct veo_dev *dev, u8 cmd, u32 raw, u32 len, u8 flag,
		 u8 st[3])
{
	static const u8 params_640[8] = { 0x80, 2, 0x27, 0x1d, 0x10, 0x10, 0, 0 };
	u8 params[8] = { 0x40, 1, 0x27, 0x1d, 8, 8, 0, 0 };
	int err = 0;
	u8 first;

	if (flag)
		params[0] = 0x48;
	sdio_claim_host(dev->func);
	veo_engine_cmd(dev, cmd, raw, VEO_STATS_BUF, len,
		       cmd == 0x26 || cmd == 0x29 ? params_640 : params, flag,
		       &err);
	first = veo_engine_wait(dev, VEO_ENGINE_TIMEOUT_MS, &err);
	veo_engine_results(dev, first, st, &err);
	veo_engine_idle(dev, &err);
	sdio_release_host(dev->func);
	return err;
}

/* Second pass over the statistics grid, for white balance. */
int veo_hw_awb_stats(struct veo_dev *dev, u8 st[3])
{
	int err = 0;
	u8 first;

	sdio_claim_host(dev->func);
	veo_engine_cmd(dev, 0x25, VEO_STATS_BUF, 0, 0x1c20, NULL, 0, &err);
	first = veo_engine_wait(dev, 8, &err);
	veo_engine_results(dev, first, st, &err);
	veo_engine_idle(dev, &err);
	sdio_release_host(dev->func);
	return err;
}

int veo_hw_read_strap(struct veo_dev *dev, u8 *strap)
{
	int err = 0;

	sdio_claim_host(dev->func);
	*strap = veo_rd(dev, 0x13, &err);
	sdio_release_host(dev->func);
	return err;
}

/* Follow a change of the orientation strap; a mode start must follow. */
int veo_hw_reorient(struct veo_dev *dev, bool flipped, u16 red, u16 blue)
{
	int err;

	dev->bayer = flipped;
	sdio_claim_host(dev->func);
	err = veo_sensor_write(dev, 0x11, flipped ? 0x10 : 0x08);
	if (!err)
		err = __veo_set_gains(dev, red, blue);
	sdio_release_host(dev->func);
	return err;
}

/*
 * Still chain of the factory driver, for the large modes. The host is held
 * across these steps except while a wait sleeps.
 */
#define VEO_STILL_TIMEOUT_MS	2000

/* Wait for any of the given 0x5e / 0xa7 bits, polling as in veo_hw_poll_done. */
static int veo_wait_irq(struct veo_dev *dev, u8 m5e, u8 ma7)
{
	unsigned long end = jiffies + msecs_to_jiffies(VEO_STILL_TIMEOUT_MS);
	int err = 0;
	u8 pending;

	for (;;) {
		if (dev->cccr_fallback) {
			veo_latch_irq(dev, &err);
		} else {
			pending = sdio_f0_readb(dev->func, SDIO_CCCR_INTx, &err);
			if (!err && ((pending & BIT(1)) ||
				     !(++dev->cccr_misses % 5)))
				veo_latch_irq(dev, &err);
		}
		if (err)
			return err;
		if ((dev->irq5e & m5e) || (dev->irqa7 & ma7))
			return 0;
		if (time_after(jiffies, end) || READ_ONCE(dev->stopping))
			return -ETIMEDOUT;
		veo_sleep_us(dev, 2000, 3000);
	}
}

/* Capture one raw frame into set A: 0x5e bit 2 is "frame captured". */
static int veo_still_capture(struct veo_dev *dev, bool bw_off)
{
	int err = 0;

	dev->irq5e = 0;
	dev->irqa7 = 0;
	veo_irq_enable(dev, 0x04, &err);
	veo_rmw(dev, 0x5c, 0, 0x01, &err);
	if (bw_off) {
		veo_wr(dev, 0x22, 0x47, &err);
		veo_wr(dev, 0x21, 0x00, &err);
		veo_wr(dev, 0x22, 0x48, &err);
		veo_wr(dev, 0x21, 0x00, &err);
	}
	if (!err)
		err = veo_wait_irq(dev, 0x04, 0);
	veo_rmw(dev, 0x5c, 0x01, 0, &err);
	veo_irq_disable(dev, 0x04, &err);
	return err;
}

/*
 * Levels calibration of the full readout class: the bridge measures a
 * histogram, the black level L and white level U picked from it stretch
 * [L, U] to the full range through bank 3 indices 0x45 and 0x46.
 */
static int veo_still_levels(struct veo_dev *dev)
{
	unsigned int lo = 0, hi = 0, l, u, votes, i;
	u8 h[8];
	int err = 0;

	veo_wr(dev, 0x22, 0x92, &err);
	veo_wr(dev, 0x21, 0x01, &err);
	veo_rmw(dev, 0xa6, 0, 0x40, &err);
	veo_rd(dev, 0xa7, &err);
	dev->irq5e = 0;
	dev->irqa7 = 0;
	veo_irq_enable(dev, 0x04, &err);
	veo_rmw(dev, 0x5c, 0, 0x20, &err);
	veo_rmw(dev, 0x5c, 0, 0x01, &err);
	if (!err)
		err = veo_wait_irq(dev, 0x04, 0x40);
	if (err)
		return err;

	veo_wr(dev, 0x20, 0x03, &err);
	veo_wr(dev, 0x23, 0x88, &err);
	for (i = 0; i < 8; i++)
		h[i] = veo_rd(dev, 0x21, &err);
	veo_irq_disable(dev, 0x04, &err);
	veo_rmw(dev, 0x5c, 0x01, 0, &err);
	veo_rmw(dev, 0x5c, 0x20, 0, &err);
	veo_rmw(dev, 0xa6, 0x40, 0, &err);
	veo_rd(dev, 0xa7, &err);
	veo_wr(dev, 0x22, 0x92, &err);
	veo_wr(dev, 0x21, 0x00, &err);
	if (err)
		return err;

	for (i = 0; i < 4; i++) {
		lo += h[i];
		hi += h[4 + i];
	}
	votes = (h[0] < 25) + (h[1] < 19) + (h[2] < 13) + (h[3] < 13) +
		(lo < 41 ? 2 : lo < 61 ? 1 : 0);
	l = votes < 2 ? 0x08 : votes < 4 ? 0x10 : votes == 4 ? 0x18 : 0x1e;

	votes = (h[7] < 13) + (h[6] < 19) + (h[5] < 19) + (h[4] < 19) +
		(hi < 31 ? 2 : hi < 40 ? 1 : 0) + (l == 0x1e ? 2 : 0);
	u = votes == 0 ? 0xf9 : votes == 1 ? 0xf4 : votes < 4 ? 0xef : 0xe9;

	veo_wr(dev, 0x20, 0x03, &err);
	veo_wr(dev, 0x22, 0x45, &err);
	veo_wr(dev, 0x21, l, &err);
	veo_wr(dev, 0x20, 0x03, &err);
	veo_wr(dev, 0x22, 0x46, &err);
	veo_wr(dev, 0x21, 0x3fc0 / (u - l), &err);
	return err;
}

static unsigned int pass_timeout_ms = VEO_ENGINE_TIMEOUT_MS;
module_param(pass_timeout_ms, uint, 0644);
MODULE_PARM_DESC(pass_timeout_ms,
		 "how long a still frame waits for each full-resolution engine pass to report done before going on (1-2000, default 121 as the factory driver)");

/*
 * The factory driver polls a full-resolution pass for 121 ms and then goes
 * on to the JPEG run whether or not the pass reported done, so a pass that
 * outlasts the poll is not an error. Only a failed register access is.
 */
static int veo_still_pass(struct veo_dev *dev, u8 cmd, u32 src, u32 dst,
			  u32 len)
{
	unsigned int timeout = clamp(READ_ONCE(pass_timeout_ms), 1U,
				     VEO_STILL_TIMEOUT_MS);
	int err = 0;

	veo_engine_cmd(dev, cmd, src, dst, len, NULL, 0, &err);
	veo_engine_wait(dev, timeout, &err);
	if (err == -ETIMEDOUT)
		err = 0;
	veo_engine_idle(dev, &err);
	return err;
}

int veo_hw_still_setup(struct veo_dev *dev, const struct veo_mode *mode)
{
	static const u8 curve[8] = { 10, 20, 30, 40, 0xd7, 0xe1, 0xeb, 0xf5 };
	unsigned int i;
	int err = 0;

	sdio_claim_host(dev->func);
	veo_irq_disable(dev, 0xff, &err);
	if (mode->class == VEO_CLASS_FULL) {
		veo_wr(dev, 0x20, 0x03, &err);
		veo_wr(dev, 0x22, 0x80, &err);
		for (i = 0; i < ARRAY_SIZE(curve); i++)
			veo_wr(dev, 0x21, curve[i], &err);
	}
	veo_wr(dev, 0x90, 0x00, &err);
	veo_wr(dev, 0x5c, 0x00, &err);
	veo_wr(dev, 0x22, 0x92, &err);
	veo_wr(dev, 0x21, 0x00, &err);
	veo_pulse_1a(dev, &err);
	if (err)
		goto out;
	veo_sleep_ms(dev, 50);
	err = veo_set_class(dev, mode->class);
	if (!err)
		err = __veo_set_exposure(dev, mode, dev->cur_exposure);
	if (err)
		goto out;
	veo_sleep_ms(dev, 70);
	dev->armed_set = 0;
out:
	sdio_release_host(dev->func);
	return err;
}

/*
 * One still: three raw captures (the last one measuring levels in the
 * full class) so exposure settles, the full-resolution engine passes,
 * then a JPEG run of buffer set A. Returns with the JPEG engine started;
 * the caller waits for JPEG done and reads VEO_JPEG_A.
 */
int veo_hw_still_frame(struct veo_dev *dev, const struct veo_mode *mode)
{
	u32 raw = veo_raw_base(0), rawlen = mode->w * mode->h;
	enum veo_still_step step = VEO_STILL_SETUP;
	int err = 0;

	sdio_claim_host(dev->func);
	veo_pulse_1a(dev, &err);
	veo_wr(dev, 0x5c, 0x00, &err);
	veo_wr(dev, 0x90, 0x00, &err);
	veo_write_window(dev, mode, &err);
	veo_wr(dev, 0x4e, raw & 0xff, &err);
	veo_wr(dev, 0x4f, (raw >> 8) & 0xff, &err);
	veo_wr(dev, 0x50, raw >> 16, &err);
	if (err)
		goto out;
	step = VEO_STILL_CAPTURE1;
	err = veo_still_capture(dev, false);
	if (err)
		goto out;
	step = VEO_STILL_CAPTURE2;
	err = veo_still_capture(dev, true);
	if (err)
		goto out;
	if (mode->flags & VEO_MODE_LEVELS) {
		step = VEO_STILL_LEVELS;
		err = veo_still_levels(dev);
	} else {
		step = VEO_STILL_CAPTURE3;
		err = veo_still_capture(dev, false);
	}
	if (err)
		goto out;
	if (mode->flags & VEO_MODE_CMD28) {
		step = VEO_STILL_CMD28;
		err = veo_still_pass(dev, 0x28, raw, raw, rawlen);
		if (err)
			goto out;
	}
	/* Rows 6-11 replace rows 0-5 of the 1288-wide frame. */
	if (mode->flags & VEO_MODE_CMD2C) {
		step = VEO_STILL_CMD2C;
		err = veo_still_pass(dev, 0x2c, raw + 0x1e30, raw, 0x1e30);
		if (err)
			goto out;
	}

	step = VEO_STILL_JPEG;
	veo_set_buffers(dev, 0, &err);
	veo_wr(dev, 0xc1, 0x00, &err);
	veo_write_jpeg_setup(dev, mode, &err);
	dev->irq5e = 0;
	dev->irqa7 = 0;
	veo_irq_enable(dev, 0x02, &err);
	veo_wr(dev, 0x5c, 0x00, &err);
	veo_wr(dev, 0x5c, 0x02, &err);
	dev->armed_set = 0;
	dev->t_arm = ktime_get_ns();
out:
	sdio_release_host(dev->func);
	if (err)
		dev_dbg(&dev->func->dev, "still setup failed at step %d (%d)\n",
			step, err);
	return err;
}

/*
 * The factory still capture (veosdiocam.dll, FUN_00103aec) stops the
 * pipeline as soon as the JPEG run reports done, before the frame is read:
 * a 0x1a pulse, then 0x5c and 0x90 cleared.
 */
int veo_hw_still_done(struct veo_dev *dev)
{
	int err = 0;

	sdio_claim_host(dev->func);
	veo_pulse_1a(dev, &err);
	veo_wr(dev, 0x5c, 0x00, &err);
	veo_wr(dev, 0x90, 0x00, &err);
	sdio_release_host(dev->func);
	return err;
}

int veo_hw_stop(struct veo_dev *dev)
{
	int err = 0;

	sdio_claim_host(dev->func);
	veo_pulse_1a(dev, &err);
	veo_wr(dev, 0x5c, 0x00, &err);
	veo_irq_disable(dev, 0xff, &err);
	veo_wr(dev, 0x90, 0x00, &err);
	sdio_release_host(dev->func);
	return err;
}

int veo_hw_standby(struct veo_dev *dev)
{
	int err = 0;

	sdio_claim_host(dev->func);
	veo_wr(dev, 0x12, 0x01, &err);
	if (!err)
		sdio_writeb(dev->func, 0x00, VEO_CTRL_ADDR, &err);
	sdio_release_host(dev->func);
	dev->initialised = false;
	return err;
}
