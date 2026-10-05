// SPDX-License-Identifier: GPL-2.0-only
/*
 * 1-wire busmaster driver for DS1WM and ASICs with embedded DS1WMs such as
 * HP iPAQs (h5xxx, h2200's HAMCOP, h39xx's ASIC2) and ASIC3-based devices
 * (hx4700).
 *
 * Based on the driver removed by commit 652719b1003a ("w1: remove ds1wm
 * driver") once its only callers, drivers/mfd/asic3.c and
 * drivers/mfd/htc-pasic3.c, were gone. The companion ASIC MFD drivers of
 * the iPAQ boards are new callers; ds2760_battery.c is the w1 slave driver
 * this one feeds. Differences from the removed driver:
 *
 *  - Register access is always byte-wide (ioread8/iowrite8), and bus_shift
 *    only selects the address stride. The removed driver picked 8/16/32-bit
 *    accessors from bus_shift, which is wrong for a register file whose
 *    single-byte registers sit at 4-byte strides on a 16-bit bus: the
 *    iPAQ ASICs use bus_shift = 2, and a 32-bit access there is not
 *    tolerated.
 *  - The clock comes from the common clock framework: probe() gets and
 *    enables it and feeds clk_get_rate() to the divisor search, rather
 *    than using a board-supplied rate.
 *  - There are no mfd_cell .enable/.disable hooks (gone from struct
 *    mfd_cell); clk_prepare_enable()/clk_disable_unprepare() is the whole
 *    enable/disable story.
 *  - struct w1_bus_master is embedded in struct ds1wm_data instead of
 *    being a file-scope static, so that a second instance works.
 *  - platform_driver .remove returns void, and suspend/resume use
 *    dev_pm_ops via DEFINE_SIMPLE_DEV_PM_OPS.
 *  - The irq_set_irq_type() handling driven by IORESOURCE_IRQ_* flags is
 *    dropped: the interrupt is a fixed-trigger internal line of the parent
 *    ASIC, and mfd_cell users never populate those flags anyway.
 *  - The resource size check requires only the five registers the driver
 *    touches (CMD/DATA/INT/INT_EN/CLKDIV) instead of a hardcoded eight,
 *    which would reject the 20-byte OWM window of the iPAQ ASICs at
 *    bus_shift = 2. The unused DS1WM_CNTRL register is dropped with it.
 *  - is_hw_big_endian is dropped: byte-wide access has no endianness.
 *  - The integration tunables (bus_shift, active_high, reset_recover_delay)
 *    are read from the device's fwnode when it has one, with platform_data
 *    as the per-property fallback, so a cell whose parent MFD hands it an
 *    of_node is fully described in devicetree. Either source alone is
 *    enough to probe.
 *
 * Copyright (c) 2004-2005, Szabolcs Gyurko <szabolcs.gyurko@tlt.hu>
 * Copyright (c) 2004-2007, Matt Reimer <mreimer@vpop.net>
 * Copyright (C) 2026 Koen Kooi
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/ds1wm.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/w1.h>

#define DS1WM_CMD	0x00	/* R/W 4 bits command */
#define DS1WM_DATA	0x01	/* R/W 8 bits, transmit/receive buffer */
#define DS1WM_INT	0x02	/* R/W interrupt status */
#define DS1WM_INT_EN	0x03	/* R/W interrupt enable */
#define DS1WM_CLKDIV	0x04	/* R/W 5 bits of divisor and pre-scale */

#define DS1WM_NUM_REGISTERS	5

#define DS1WM_CMD_1W_RESET  BIT(0)	/* force reset on 1-wire bus */
#define DS1WM_CMD_SRA	    BIT(1)	/* enable Search ROM accelerator mode */
#define DS1WM_CMD_DQ_OUTPUT BIT(2)	/* write only - forces bus low */
#define DS1WM_CMD_DQ_INPUT  BIT(3)	/* read only - reflects state of bus */
#define DS1WM_CMD_RST	    BIT(5)	/* software reset */
#define DS1WM_CMD_OD	    BIT(7)	/* overdrive */

#define DS1WM_INT_PD	    BIT(0)	/* presence detect */
#define DS1WM_INT_PDR	    BIT(1)	/* presence detect result */
#define DS1WM_INT_TBE	    BIT(2)	/* tx buffer empty */
#define DS1WM_INT_TSRE	    BIT(3)	/* tx shift register empty */
#define DS1WM_INT_RBF	    BIT(4)	/* rx buffer full */
#define DS1WM_INT_RSRF	    BIT(5)	/* rx shift register full */

#define DS1WM_INTEN_EPD	    BIT(0)	/* enable presence detect int */
#define DS1WM_INTEN_IAS	    BIT(1)	/* INTR active state */
#define DS1WM_INTEN_ETBE    BIT(2)	/* enable tx buffer empty int */
#define DS1WM_INTEN_ETMT    BIT(3)	/* enable tx shift register empty int */
#define DS1WM_INTEN_ERBF    BIT(4)	/* enable rx buffer full int */
#define DS1WM_INTEN_ERSRF   BIT(5)	/* enable rx shift register full int */
#define DS1WM_INTEN_DQO	    BIT(6)	/* enable direct bus driving ops */

#define DS1WM_INTEN_NOT_IAS (~DS1WM_INTEN_IAS)	/* all but INTR active state */

#define DS1WM_TIMEOUT (HZ * 5)

static const struct {
	unsigned long freq;
	unsigned long divisor;
} ds1wm_freq_divisor[] = {
	{   1000000, 0x80 },
	{   2000000, 0x84 },
	{   3000000, 0x81 },
	{   4000000, 0x88 },
	{   5000000, 0x82 },
	{   6000000, 0x85 },
	{   7000000, 0x83 },
	{   8000000, 0x8c },
	{  10000000, 0x86 },
	{  12000000, 0x89 },
	{  14000000, 0x87 },
	{  16000000, 0x90 },
	{  20000000, 0x8a },
	{  24000000, 0x8d },
	{  28000000, 0x8b },
	{  32000000, 0x94 },
	{  40000000, 0x8e },
	{  48000000, 0x91 },
	{  56000000, 0x8f },
	{  64000000, 0x98 },
	{  80000000, 0x92 },
	{  96000000, 0x95 },
	{ 112000000, 0x93 },
	{ 128000000, 0x9c },
	/* extend as needed -- see the OPERATION - CLOCK DIVISOR section of
	 * the DS1WM spec sheet.
	 */
};

struct ds1wm_data {
	struct platform_device *pdev;
	void __iomem	*map;
	unsigned int	bus_shift; /* # of shifts to calc register offsets */
	struct clk	*clk;
	int		irq;
	struct w1_bus_master master;
	int		slave_present;
	struct completion *reset_complete;
	struct completion *read_complete;
	struct completion *write_complete;
	int		read_error;
	u8		read_byte; /* last byte received */
	/* byte to write that leaves all interrupts disabled, accounting
	 * for active_state (IAS) -- optimization to avoid recomputing it
	 * on every register write.
	 */
	u8		int_en_reg_none;
	unsigned int	reset_recover_delay; /* see ds1wm_driver_data */
	/*
	 * Set once a completion interrupt has provably gone missing: later
	 * operations skip the wait for the interrupt, which can only expire,
	 * and poll the status register instead.
	 */
	bool		irq_lost;
};

static inline void ds1wm_write_register(struct ds1wm_data *ds1wm_data, u32 reg, u8 val)
{
	iowrite8(val, ds1wm_data->map + (reg << ds1wm_data->bus_shift));
}

static inline u8 ds1wm_read_register(struct ds1wm_data *ds1wm_data, u32 reg)
{
	u8 val = ioread8(ds1wm_data->map + (reg << ds1wm_data->bus_shift));

	dev_dbg(&ds1wm_data->pdev->dev, "%s reg: %d, val: %#x\n", __func__, reg, val);
	return val;
}

static irqreturn_t ds1wm_isr(int isr, void *data)
{
	struct ds1wm_data *ds1wm_data = data;
	u8 intr;
	u8 inten = ds1wm_read_register(ds1wm_data, DS1WM_INT_EN);

	if (!(inten & DS1WM_INTEN_NOT_IAS)) {
		/*
		 * No enable bit (IAS is polarity, not an enable) is set, so no
		 * operation is in flight and no waiter can be completed.
		 *
		 * DS1WM_INT is clear-on-read and the read is the only thing
		 * that drops a latched condition, so it must not be skipped:
		 * the line is a dedicated demuxed input on every user of this
		 * driver and returning without the read would leave it
		 * asserted forever. Claim the interrupt only if something was
		 * actually latched.
		 */
		intr = ds1wm_read_register(ds1wm_data, DS1WM_INT);

		return intr ? IRQ_HANDLED : IRQ_NONE;
	}

	ds1wm_write_register(ds1wm_data, DS1WM_INT_EN, ds1wm_data->int_en_reg_none);

	/* this read action clears the INTR and certain flags in ds1wm */
	intr = ds1wm_read_register(ds1wm_data, DS1WM_INT);

	ds1wm_data->slave_present = (intr & DS1WM_INT_PDR) ? 0 : 1;

	if ((intr & DS1WM_INT_TSRE) && ds1wm_data->write_complete) {
		inten &= ~DS1WM_INTEN_ETMT;
		complete(ds1wm_data->write_complete);
	}
	if (intr & DS1WM_INT_RBF) {
		/* this read clears the RBF flag */
		ds1wm_data->read_byte = ds1wm_read_register(ds1wm_data, DS1WM_DATA);
		inten &= ~DS1WM_INTEN_ERBF;
		if (ds1wm_data->read_complete)
			complete(ds1wm_data->read_complete);
	}
	if ((intr & DS1WM_INT_PD) && ds1wm_data->reset_complete) {
		inten &= ~DS1WM_INTEN_EPD;
		complete(ds1wm_data->reset_complete);
	}

	ds1wm_write_register(ds1wm_data, DS1WM_INT_EN, inten);
	return IRQ_HANDLED;
}

/*
 * Some parent ASICs never deliver the completion interrupt although the
 * 1-Wire operation finished. DS1WM_INT is clear-on-read, so poll the same
 * register the interrupt handler would have read, for the bit that marks
 * completion of the operation in flight.
 *
 * The handler consumes the same bits, so it is kept off while polling:
 * otherwise an interrupt arriving during the poll would clear the
 * condition and the poll would wait out the whole timeout. Returns 0 when
 * the poll saw the condition, 1 when the handler had completed the
 * operation after all, or -ETIMEDOUT.
 */
static int ds1wm_poll_int(struct ds1wm_data *ds1wm_data,
			  struct completion *done, u8 mask, u8 *intr)
{
	int ret;

	disable_irq(ds1wm_data->irq);

	if (completion_done(done)) {
		ret = 1;
		goto out;
	}

	ret = read_poll_timeout(ds1wm_read_register, *intr, *intr & mask, 100,
				jiffies_to_usecs(DS1WM_TIMEOUT), false,
				ds1wm_data, DS1WM_INT);
out:
	enable_irq(ds1wm_data->irq);

	return ret;
}

static void ds1wm_note_irq_lost(struct ds1wm_data *ds1wm_data, u8 intr)
{
	if (ds1wm_data->irq_lost)
		return;

	ds1wm_data->irq_lost = true;
	dev_warn(&ds1wm_data->pdev->dev,
		 "completion IRQ never arrived but DS1WM_INT=%#x shows the operation completed; polling from now on\n",
		 intr);
}

static int ds1wm_reset(struct ds1wm_data *ds1wm_data)
{
	unsigned long timeleft;
	DECLARE_COMPLETION_ONSTACK(reset_done);

	ds1wm_data->reset_complete = &reset_done;

	/* enable Presence detect only */
	ds1wm_write_register(ds1wm_data, DS1WM_INT_EN,
			     DS1WM_INTEN_EPD | ds1wm_data->int_en_reg_none);

	ds1wm_write_register(ds1wm_data, DS1WM_CMD, DS1WM_CMD_1W_RESET);

	timeleft = ds1wm_data->irq_lost ? 0 :
		wait_for_completion_timeout(&reset_done, DS1WM_TIMEOUT);
	ds1wm_data->reset_complete = NULL;
	if (!timeleft) {
		u8 intr = 0;
		int ret = ds1wm_poll_int(ds1wm_data, &reset_done, DS1WM_INT_PD, &intr);

		if (ret < 0) {
			dev_err(&ds1wm_data->pdev->dev,
				"reset failed, timed out (DS1WM_INT=%#x)\n", intr);
			return 1;
		}

		if (!ret) {
			ds1wm_note_irq_lost(ds1wm_data, intr);
			ds1wm_data->slave_present = (intr & DS1WM_INT_PDR) ? 0 : 1;
		}
	}

	if (!ds1wm_data->slave_present) {
		dev_dbg(&ds1wm_data->pdev->dev, "reset: no devices found\n");
		return 1;
	}

	if (ds1wm_data->reset_recover_delay)
		msleep(ds1wm_data->reset_recover_delay);

	return 0;
}

static int ds1wm_write(struct ds1wm_data *ds1wm_data, u8 data)
{
	unsigned long timeleft;
	DECLARE_COMPLETION_ONSTACK(write_done);

	ds1wm_data->write_complete = &write_done;

	ds1wm_write_register(ds1wm_data, DS1WM_INT_EN,
			     ds1wm_data->int_en_reg_none | DS1WM_INTEN_ETMT);

	ds1wm_write_register(ds1wm_data, DS1WM_DATA, data);

	timeleft = ds1wm_data->irq_lost ? 0 :
		wait_for_completion_timeout(&write_done, DS1WM_TIMEOUT);

	ds1wm_data->write_complete = NULL;
	if (!timeleft) {
		u8 intr = 0;
		int ret = ds1wm_poll_int(ds1wm_data, &write_done, DS1WM_INT_TSRE, &intr);

		if (ret < 0) {
			dev_err(&ds1wm_data->pdev->dev,
				"write failed, timed out (DS1WM_INT=%#x)\n", intr);
			return -ETIMEDOUT;
		}

		if (!ret)
			ds1wm_note_irq_lost(ds1wm_data, intr);
	}

	return 0;
}

static u8 ds1wm_read(struct ds1wm_data *ds1wm_data, unsigned char write_data)
{
	unsigned long timeleft;
	u8 int_en = DS1WM_INTEN_ERBF | ds1wm_data->int_en_reg_none;
	DECLARE_COMPLETION_ONSTACK(read_done);

	ds1wm_read_register(ds1wm_data, DS1WM_DATA);

	ds1wm_data->read_complete = &read_done;
	ds1wm_write_register(ds1wm_data, DS1WM_INT_EN, int_en);

	ds1wm_write_register(ds1wm_data, DS1WM_DATA, write_data);
	timeleft = ds1wm_data->irq_lost ? 0 :
		wait_for_completion_timeout(&read_done, DS1WM_TIMEOUT);

	ds1wm_data->read_complete = NULL;
	if (!timeleft) {
		u8 intr = 0;
		int ret = ds1wm_poll_int(ds1wm_data, &read_done, DS1WM_INT_RBF, &intr);

		if (ret < 0) {
			dev_err(&ds1wm_data->pdev->dev,
				"read failed, timed out (DS1WM_INT=%#x)\n", intr);
			ds1wm_data->read_error = -ETIMEDOUT;
			return 0xFF;
		}

		if (!ret) {
			ds1wm_note_irq_lost(ds1wm_data, intr);
			/* this read clears the RBF flag, same as ds1wm_isr() */
			ds1wm_data->read_byte = ds1wm_read_register(ds1wm_data, DS1WM_DATA);
		}
	}
	ds1wm_data->read_error = 0;
	return ds1wm_data->read_byte;
}

static int ds1wm_find_divisor(unsigned long gclk)
{
	int i;

	for (i = ARRAY_SIZE(ds1wm_freq_divisor) - 1; i >= 0; --i)
		if (gclk >= ds1wm_freq_divisor[i].freq)
			return ds1wm_freq_divisor[i].divisor;

	return 0;
}

static int ds1wm_up(struct ds1wm_data *ds1wm_data)
{
	struct device *dev = &ds1wm_data->pdev->dev;
	unsigned long gclk;
	int divisor, ret;

	ret = clk_prepare_enable(ds1wm_data->clk);
	if (ret)
		return ret;

	gclk = clk_get_rate(ds1wm_data->clk);
	divisor = ds1wm_find_divisor(gclk);
	dev_dbg(dev, "found divisor %#x for clock %lu\n", divisor, gclk);
	if (!divisor) {
		dev_err(dev, "no suitable divisor for %luHz clock\n", gclk);
		clk_disable_unprepare(ds1wm_data->clk);
		return -EINVAL;
	}
	/*
	 * ds1wm_freq_divisor[] entries carry a stray bit 7 next to the real
	 * 5-bit divisor/prescaler encoding; CLKDIV bits [7:5] are reserved
	 * and must be written as 0.
	 */
	ds1wm_write_register(ds1wm_data, DS1WM_CLKDIV, divisor & 0x1f);

	/* Let the w1 clock stabilize. */
	usleep_range(1000, 2000);

	ds1wm_reset(ds1wm_data);

	return 0;
}

static void ds1wm_down(struct ds1wm_data *ds1wm_data)
{
	ds1wm_reset(ds1wm_data);

	/* Disable interrupts. */
	ds1wm_write_register(ds1wm_data, DS1WM_INT_EN, ds1wm_data->int_en_reg_none);

	clk_disable_unprepare(ds1wm_data->clk);
}

/* --------------------------------------------------------------------- */
/* w1 methods */

static u8 ds1wm_read_byte(void *data)
{
	struct ds1wm_data *ds1wm_data = data;

	return ds1wm_read(ds1wm_data, 0xff);
}

static void ds1wm_write_byte(void *data, u8 byte)
{
	struct ds1wm_data *ds1wm_data = data;

	ds1wm_write(ds1wm_data, byte);
}

static u8 ds1wm_reset_bus(void *data)
{
	struct ds1wm_data *ds1wm_data = data;

	ds1wm_reset(ds1wm_data);

	return 0;
}

static void ds1wm_search(void *data, struct w1_master *master_dev,
			 u8 search_type, w1_slave_found_callback slave_found)
{
	struct ds1wm_data *ds1wm_data = data;
	struct device *dev = &ds1wm_data->pdev->dev;
	int i;
	int ms_discrep_bit = -1;
	u64 r = 0; /* holds the progress of the search */
	u64 r_prime, d;
	unsigned int slaves_found = 0;
	unsigned int pass = 0;

	dev_dbg(dev, "search begin\n");
	while (true) {
		++pass;
		if (pass > 100) {
			dev_dbg(dev, "too many attempts (100), search aborted\n");
			return;
		}

		mutex_lock(&master_dev->bus_mutex);
		if (ds1wm_reset(ds1wm_data)) {
			mutex_unlock(&master_dev->bus_mutex);
			dev_dbg(dev, "pass: %d reset error (or no slaves)\n", pass);
			break;
		}

		dev_dbg(dev, "pass: %d r: %0#18llx writing SEARCH_ROM\n", pass, r);
		ds1wm_write(ds1wm_data, search_type);
		dev_dbg(dev, "pass: %d entering ASM\n", pass);
		ds1wm_write_register(ds1wm_data, DS1WM_CMD, DS1WM_CMD_SRA);
		dev_dbg(dev, "pass: %d beginning nibble loop\n", pass);

		r_prime = 0;
		d = 0;
		/* we work one nibble at a time, each nibble interleaved to
		 * form a byte.
		 */
		for (i = 0; i < 16; i++) {
			unsigned char resp, _r, _r_prime, _d;

			_r = (r >> (4 * i)) & 0xf;
			_r = ((_r & 0x1) << 1) |
			     ((_r & 0x2) << 2) |
			     ((_r & 0x4) << 3) |
			     ((_r & 0x8) << 4);

			/* writes _r, then reads back: */
			resp = ds1wm_read(ds1wm_data, _r);

			if (ds1wm_data->read_error) {
				dev_err(dev, "pass: %d nibble: %d read error\n", pass, i);
				break;
			}

			_r_prime = ((resp & 0x02) >> 1) |
				   ((resp & 0x08) >> 2) |
				   ((resp & 0x20) >> 3) |
				   ((resp & 0x80) >> 4);

			_d = ((resp & 0x01) >> 0) |
			     ((resp & 0x04) >> 1) |
			     ((resp & 0x10) >> 2) |
			     ((resp & 0x40) >> 3);

			r_prime |= (u64)_r_prime << (i * 4);
			d |= (u64)_d << (i * 4);
		}
		if (ds1wm_data->read_error) {
			mutex_unlock(&master_dev->bus_mutex);
			dev_err(dev, "pass: %d read error, retrying\n", pass);
			break;
		}
		dev_dbg(dev, "pass: %d r': %0#18llx d:%0#18llx\n", pass, r_prime, d);
		dev_dbg(dev, "pass: %d nibble loop complete, exiting ASM\n", pass);
		/* BIT() is an unsigned long: cast the inversion to the u8 the register takes */
		ds1wm_write_register(ds1wm_data, DS1WM_CMD, (u8)~DS1WM_CMD_SRA);
		dev_dbg(dev, "pass: %d resetting bus\n", pass);
		ds1wm_reset(ds1wm_data);
		mutex_unlock(&master_dev->bus_mutex);
		if ((r_prime & ((u64)1 << 63)) && (d & ((u64)1 << 63))) {
			dev_err(dev, "pass: %d bus error, retrying\n", pass);
			continue; /* start over */
		}

		dev_dbg(dev, "pass: %d found %0#18llx\n", pass, r_prime);
		slave_found(master_dev, r_prime);
		++slaves_found;
		dev_dbg(dev, "pass: %d complete, preparing next pass\n", pass);

		/* any discrepancy found which we already chose the '1'
		 * branch for is now irrelevant; reveal the next branch:
		 */
		d &= ~r;
		/* find last bit set, i.e. the most significant bit set */
		ms_discrep_bit = fls64(d) - 1;
		dev_dbg(dev, "pass: %d new d:%0#18llx MS discrep bit:%d\n",
			pass, d, ms_discrep_bit);

		if (ms_discrep_bit == -1)
			break;

		r = (r & ~(~0ull << ms_discrep_bit)) | (1ull << ms_discrep_bit);
	}
	dev_dbg(dev, "pass: %d total: %d search done ms d bit pos: %d\n",
		pass, slaves_found, ms_discrep_bit);
}

/* --------------------------------------------------------------------- */

static int ds1wm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ds1wm_data *ds1wm_data;
	const struct ds1wm_driver_data *plat = dev_get_platdata(dev);
	u32 bus_shift, reset_recover_delay;
	struct resource *res;
	bool active_high;
	int irq, ret;

	/*
	 * Two possible sources for this core's three integration tunables,
	 * merged per property with the fwnode winning: a parent MFD's
	 * compiled-in ds1wm_driver_data, and the "onewire" child node a
	 * board's devicetree may carry (Documentation/devicetree/bindings/
	 * w1/dallas,ds1wm.yaml). Every MFD core in this tree still sets
	 * .platform_data on its cell, so a board with no such node, or one
	 * whose node omits a property, keeps exactly the values its ASIC
	 * always had; a node that does carry one overrides it.
	 *
	 * "dallas,active-high" being a plain boolean means a devicetree can
	 * only ever turn the IAS bit on, never clear a parent's compiled-in
	 * 1. That asymmetry is deliberate and harmless: the interrupt
	 * polarity between this core and its parent ASIC is fixed inside
	 * the same die, so it is the parent MFD driver -- not a board --
	 * that is authoritative about it in the first place.
	 */
	if (!plat && !dev_fwnode(dev))
		return -ENODEV;

	bus_shift = plat ? plat->bus_shift : 0;
	reset_recover_delay = plat ? plat->reset_recover_delay : 0;
	active_high = plat ? !!plat->active_high : false;

	device_property_read_u32(dev, "dallas,bus-shift", &bus_shift);
	device_property_read_u32(dev, "dallas,reset-recover-delay-ms",
				 &reset_recover_delay);
	if (device_property_read_bool(dev, "dallas,active-high"))
		active_high = true;

	if (bus_shift > 2) {
		dev_err(dev, "illegal bus-shift %u\n", bus_shift);
		return -EINVAL;
	}

	ds1wm_data = devm_kzalloc(dev, sizeof(*ds1wm_data), GFP_KERNEL);
	if (!ds1wm_data)
		return -ENOMEM;

	platform_set_drvdata(pdev, ds1wm_data);
	ds1wm_data->pdev = pdev;
	ds1wm_data->bus_shift = bus_shift;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	if ((DS1WM_NUM_REGISTERS << ds1wm_data->bus_shift) > resource_size(res)) {
		dev_err(dev, "memory resource too small: %d, need %d\n",
			(int)resource_size(res),
			DS1WM_NUM_REGISTERS << ds1wm_data->bus_shift);
		return -EINVAL;
	}

	/*
	 * devm_ioremap() rather than devm_ioremap_resource(), which would
	 * reserve the region exclusively: this cell's resource is always a
	 * sub-range of the parent MFD's window, which the parent has already
	 * claimed, so an exclusive request here would fail with -EBUSY.
	 */
	ds1wm_data->map = devm_ioremap(dev, res->start, resource_size(res));
	if (!ds1wm_data->map)
		return -ENOMEM;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ds1wm_data->irq = irq;

	ds1wm_data->int_en_reg_none = active_high ? DS1WM_INTEN_IAS : 0;
	ds1wm_data->reset_recover_delay = reset_recover_delay;

	ds1wm_data->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(ds1wm_data->clk))
		return dev_err_probe(dev, PTR_ERR(ds1wm_data->clk), "no clock\n");

	/* Mask interrupts, set IAS, before claiming the interrupt line. */
	ds1wm_write_register(ds1wm_data, DS1WM_INT_EN, ds1wm_data->int_en_reg_none);

	ret = devm_request_irq(dev, ds1wm_data->irq, ds1wm_isr, 0, "ds1wm", ds1wm_data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request irq %d\n", ds1wm_data->irq);

	ret = ds1wm_up(ds1wm_data);
	if (ret)
		return ret;

	ds1wm_data->master.data = ds1wm_data;
	ds1wm_data->master.read_byte = ds1wm_read_byte;
	ds1wm_data->master.write_byte = ds1wm_write_byte;
	ds1wm_data->master.reset_bus = ds1wm_reset_bus;
	ds1wm_data->master.search = ds1wm_search;

	ret = w1_add_master_device(&ds1wm_data->master);
	if (ret) {
		ds1wm_down(ds1wm_data);
		return ret;
	}

	dev_dbg(dev, "probe successful, IAS: %d, rec.delay: %u, bus-shift: %u\n",
		active_high, ds1wm_data->reset_recover_delay,
		ds1wm_data->bus_shift);
	return 0;
}

static void ds1wm_remove(struct platform_device *pdev)
{
	struct ds1wm_data *ds1wm_data = platform_get_drvdata(pdev);

	w1_remove_master_device(&ds1wm_data->master);
	ds1wm_down(ds1wm_data);
}

static int ds1wm_suspend(struct device *dev)
{
	ds1wm_down(dev_get_drvdata(dev));

	return 0;
}

static int ds1wm_resume(struct device *dev)
{
	return ds1wm_up(dev_get_drvdata(dev));
}

static DEFINE_SIMPLE_DEV_PM_OPS(ds1wm_pm_ops, ds1wm_suspend, ds1wm_resume);

/*
 * Three of the four companion-ASIC MFD cores that instantiate this driver
 * (drivers/mfd/hamcop.c, drivers/mfd/samcop.c and drivers/mfd/asic2.c --
 * drivers/mfd/asic3.c is the one that does not yet) set .of_compatible on
 * their cell, so the platform device gets an of_node and reports an OF
 * modalias rather than the platform: one below. Without a matching OF
 * table modprobe cannot autoload this module on those boards.
 *
 * One entry, not one per ASIC: every such node carries "dallas,ds1wm" as
 * its fallback compatible (Documentation/devicetree/bindings/w1/
 * dallas,ds1wm.yaml), and of_device_uevent_modalias() emits every one of a
 * node's compatible strings into MODALIAS, so this alias matches them all.
 */
static const struct of_device_id ds1wm_of_match[] = {
	{ .compatible = "dallas,ds1wm" },
	{ }
};
MODULE_DEVICE_TABLE(of, ds1wm_of_match);

static struct platform_driver ds1wm_driver = {
	.driver = {
		.name	= "ds1wm",
		.of_match_table = ds1wm_of_match,
		.pm	= pm_sleep_ptr(&ds1wm_pm_ops),
	},
	.probe		= ds1wm_probe,
	.remove		= ds1wm_remove,
};
module_platform_driver(ds1wm_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Szabolcs Gyurko <szabolcs.gyurko@tlt.hu>, Matt Reimer <mreimer@vpop.net>, Jean-Francois Dagenais <dagenaisj@sonatest.com>");
MODULE_DESCRIPTION("DS1WM w1 busmaster driver");
MODULE_ALIAS("platform:ds1wm");
