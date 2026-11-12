// SPDX-License-Identifier: (GPL-2.0 OR MPL-1.1)
/* hermes.c
 *
 * Driver core for the "Hermes" wireless MAC controller, as used in
 * the Lucent Orinoco and Cabletron RoamAbout cards. It should also
 * work on the hfa3841 and hfa3842 MAC controller chips used in the
 * Prism II chipsets.
 *
 * This is not a complete driver, just low-level access routines for
 * the MAC controller itself.
 *
 * Based on the prism2 driver from Absolute Value Systems' linux-wlan
 * project, the Linux wvlan_cs driver, Lucent's HCF-Light
 * (wvlan_hcf.c) library, and the NetBSD wireless driver (in no
 * particular order).
 *
 * Copyright (C) 2000, David Gibson, Linuxcare Australia.
 * (C) Copyright David Gibson, IBM Corp. 2001-2003.
 *
 * The contents of this file are subject to the Mozilla Public License
 * Version 1.1 (the "License"); you may not use this file except in
 * compliance with the License. You may obtain a copy of the License
 * at http://www.mozilla.org/MPL/
 *
 * Software distributed under the License is distributed on an "AS IS"
 * basis, WITHOUT WARRANTY OF ANY KIND, either express or implied. See
 * the License for the specific language governing rights and
 * limitations under the License.
 *
 * Alternatively, the contents of this file may be used under the
 * terms of the GNU General Public License version 2 (the "GPL"), in
 * which case the provisions of the GPL are applicable instead of the
 * above.  If you wish to allow the use of your version of this file
 * only under the terms of the GPL and not to allow others to use your
 * version of this file under the MPL, indicate your decision by
 * deleting the provisions above and replace them with the notice and
 * other provisions required by the GPL.  If you do not delete the
 * provisions above, a recipient may use your version of this file
 * under either the MPL or the GPL.
 */

#include <linux/net.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include "hermes.h"

/*
 * These are maximum timeouts. Most often, the card will react much
 * faster.
 *
 * Every one of them is spun out in a udelay() loop with interrupts
 * disabled, so a card that stops answering holds a uniprocessor CPU for
 * the whole budget, up to half a second for cmd_init_timeout. They are
 * module parameters so they can be tuned for slow buses without a
 * rebuild.
 */
static unsigned int cmd_busy_timeout = 100;
module_param(cmd_busy_timeout, uint, 0644);
MODULE_PARM_DESC(cmd_busy_timeout,
		 "Iterations of ~1us to wait for the command register to unbusy (default 100, ~100us)");

static unsigned int cmd_init_timeout = 50000;
module_param(cmd_init_timeout, uint, 0644);
MODULE_PARM_DESC(cmd_init_timeout,
		 "Iterations of ~10us to wait for an init command to complete (default 50000, ~500ms; spun with IRQs off)");

static unsigned int cmd_compl_timeout = 20000;
module_param(cmd_compl_timeout, uint, 0644);
MODULE_PARM_DESC(cmd_compl_timeout,
		 "Iterations of ~10us to wait for a command to complete (default 20000, ~200ms; spun with IRQs off)");

static unsigned int alloc_compl_timeout = 1000;
module_param(alloc_compl_timeout, uint, 0644);
MODULE_PARM_DESC(alloc_compl_timeout,
		 "Iterations of ~10us to wait for a frame allocation (default 1000, ~10ms)");

static unsigned int bap_busy_timeout = HERMES_BAP_BUSY_TIMEOUT;
module_param(bap_busy_timeout, uint, 0644);
MODULE_PARM_DESC(bap_busy_timeout,
		 "Iterations of ~1us to wait for a BAP seek (default 10000, ~10ms; paid twice per seek, on every PIO transfer)");

/*
 * AUX port access.  To unlock the AUX port write the access keys to the
 * PARAM0-2 registers, then write HERMES_AUX_ENABLE to the HERMES_CONTROL
 * register.  Then read it and make sure it's HERMES_AUX_ENABLED.
 */
#define HERMES_AUX_ENABLE	0x8000	/* Enable auxiliary port access */
#define HERMES_AUX_DISABLE	0x4000	/* Disable to auxiliary port access */
#define HERMES_AUX_ENABLED	0xC000	/* Auxiliary port is open */
#define HERMES_AUX_DISABLED	0x0000	/* Auxiliary port is closed */

#define HERMES_AUX_PW0	0xFE01
#define HERMES_AUX_PW1	0xDC23
#define HERMES_AUX_PW2	0xBA45

/* HERMES_CMD_DOWNLD */
#define HERMES_PROGRAM_DISABLE             (0x0000 | HERMES_CMD_DOWNLD)
#define HERMES_PROGRAM_ENABLE_VOLATILE     (0x0100 | HERMES_CMD_DOWNLD)
#define HERMES_PROGRAM_ENABLE_NON_VOLATILE (0x0200 | HERMES_CMD_DOWNLD)
#define HERMES_PROGRAM_NON_VOLATILE        (0x0300 | HERMES_CMD_DOWNLD)

static const struct hermes_ops hermes_ops_local;

/*
 * Internal functions
 */

/* Issue a command to the chip. Waiting for it to complete is the caller's
 * problem.
 *
 * Returns -EBUSY if the command register is busy, 0 on success.
 *
 * Callable from any context.
 */
static int hermes_issue_cmd(struct hermes *hw, u16 cmd, u16 param0,
			    u16 param1, u16 param2)
{
	unsigned int k = cmd_busy_timeout;
	u16 reg;

	/* First wait for the command register to unbusy */
	reg = hermes_read_regn(hw, CMD);
	while ((reg & HERMES_CMD_BUSY) && k) {
		k--;
		udelay(1);
		reg = hermes_read_regn(hw, CMD);
	}
	if (reg & HERMES_CMD_BUSY)
		return -EBUSY;

	hermes_write_regn(hw, PARAM2, param2);
	hermes_write_regn(hw, PARAM1, param1);
	hermes_write_regn(hw, PARAM0, param0);
	hermes_write_regn(hw, CMD, cmd);

	return 0;
}

/*
 * Function definitions
 */

/* For doing cmds that wipe the magic constant in SWSUPPORT0 */
static int hermes_doicmd_wait(struct hermes *hw, u16 cmd,
			      u16 parm0, u16 parm1, u16 parm2,
			      struct hermes_response *resp)
{
	int err = 0;
	unsigned int k;
	u16 status, reg;

	err = hermes_issue_cmd(hw, cmd, parm0, parm1, parm2);
	if (err)
		return err;

	reg = hermes_read_regn(hw, EVSTAT);
	k = cmd_init_timeout;
	while ((!(reg & HERMES_EV_CMD)) && k) {
		k--;
		udelay(10);
		reg = hermes_read_regn(hw, EVSTAT);
	}

	hermes_write_regn(hw, SWSUPPORT0, HERMES_MAGIC);

	if (!hermes_present(hw)) {
		pr_err("hermes @ %p: Card not present after command 0x%04x (EVSTAT %#06x, EV_CMD %s, SWSUPPORT0 %#06x)\n",
		       hw->iobase, cmd, reg,
		       (reg & HERMES_EV_CMD) ? "set" : "never set",
		       hermes_read_regn(hw, SWSUPPORT0));
		err = -ENODEV;
		goto out;
	}

	if (!(reg & HERMES_EV_CMD)) {
		pr_err("hermes @ %p: Timeout waiting for card to reset (reg=0x%04x)!\n",
		       hw->iobase, reg);
		err = -ETIMEDOUT;
		goto out;
	}

	status = hermes_read_regn(hw, STATUS);
	if (resp) {
		resp->status = status;
		resp->resp0 = hermes_read_regn(hw, RESP0);
		resp->resp1 = hermes_read_regn(hw, RESP1);
		resp->resp2 = hermes_read_regn(hw, RESP2);
	}

	hermes_write_regn(hw, EVACK, HERMES_EV_CMD);

	if (status & HERMES_STATUS_RESULT) {
		pr_err("hermes @ %p: Result error issuing command 0x%04x (status 0x%04x, resp0 0x%04x, resp1 0x%04x, resp2 0x%04x)\n",
		       hw->iobase, cmd, status,
		       hermes_read_regn(hw, RESP0),
		       hermes_read_regn(hw, RESP1),
		       hermes_read_regn(hw, RESP2));
		err = -EIO;
	}
out:
	return err;
}

void hermes_struct_init(struct hermes *hw, void __iomem *address,
			int reg_spacing)
{
	hw->iobase = address;
	hw->reg_spacing = reg_spacing;
	hw->inten = 0x0;
	hw->eeprom_pda = false;
	hw->ops = &hermes_ops_local;
}
EXPORT_SYMBOL(hermes_struct_init);

static int hermes_init(struct hermes *hw)
{
	u16 reg;
	int err = 0;
	unsigned int k;

	/* We don't want to be interrupted while resetting the chipset */
	hw->inten = 0x0;
	hermes_write_regn(hw, INTEN, 0);
	hermes_write_regn(hw, EVACK, 0xffff);

	/* Normally it's a "can't happen" for the command register to
	 * be busy when we go to issue a command because we are
	 * serializing all commands.  However we want to have some
	 * chance of resetting the card even if it gets into a stupid
	 * state, so we actually wait to see if the command register
	 * will unbusy itself here.
	 */
	k = cmd_busy_timeout;
	reg = hermes_read_regn(hw, CMD);
	while (k && (reg & HERMES_CMD_BUSY)) {
		/* The card has probably been removed: don't wait for the
		 * timeout
		 */
		if (reg == 0xffff) {
			pr_err("hermes @ %p: CMD register read 0xffff at %s() entry -- card not responding\n",
			       hw->iobase, __func__);
			return -ENODEV;
		}

		k--;
		udelay(1);
		reg = hermes_read_regn(hw, CMD);
	}

	/* No need to explicitly handle the timeout - if we've timed
	 * out hermes_issue_cmd() will probably return -EBUSY below
	 */

	/* According to the documentation, EVSTAT may contain
	 * obsolete event occurrence information.  We have to acknowledge
	 * it by writing EVACK.
	 */
	reg = hermes_read_regn(hw, EVSTAT);
	hermes_write_regn(hw, EVACK, reg);

	/* We don't use hermes_docmd_wait here, because the reset wipes
	 * the magic constant in SWSUPPORT0 away, and it gets confused
	 */
	err = hermes_doicmd_wait(hw, HERMES_CMD_INIT, 0, 0, 0, NULL);

	return err;
}

/* Issue a command to the chip, and (busy!) wait for it to
 * complete.
 *
 * Returns:
 *     < 0 on internal error
 *       0 on success
 *     > 0 on error returned by the firmware
 *
 * Callable from any context, but locking is your problem.
 */
static int hermes_docmd_wait(struct hermes *hw, u16 cmd, u16 parm0,
			     struct hermes_response *resp)
{
	int err;
	unsigned int k;
	u16 reg;
	u16 status;

	err = hermes_issue_cmd(hw, cmd, parm0, 0, 0);
	if (err) {
		if (!hermes_present(hw)) {
			if (net_ratelimit())
				pr_warn("hermes @ %p: Card removed while issuing command 0x%04x.\n",
					hw->iobase, cmd);
			err = -ENODEV;
		} else if (net_ratelimit()) {
			pr_err("hermes @ %p: Error %d issuing command 0x%04x.\n",
			       hw->iobase, err, cmd);
		}
		goto out;
	}

	reg = hermes_read_regn(hw, EVSTAT);
	k = cmd_compl_timeout;
	while ((!(reg & HERMES_EV_CMD)) && k) {
		k--;
		udelay(10);
		reg = hermes_read_regn(hw, EVSTAT);
	}

	if (!hermes_present(hw)) {
		pr_warn("hermes @ %p: Card removed while waiting for command 0x%04x completion.\n",
			hw->iobase, cmd);
		err = -ENODEV;
		goto out;
	}

	if (!(reg & HERMES_EV_CMD)) {
		pr_err("hermes @ %p: Timeout waiting for command 0x%04x completion.\n",
		       hw->iobase, cmd);
		err = -ETIMEDOUT;
		goto out;
	}

	status = hermes_read_regn(hw, STATUS);
	if (resp) {
		resp->status = status;
		resp->resp0 = hermes_read_regn(hw, RESP0);
		resp->resp1 = hermes_read_regn(hw, RESP1);
		resp->resp2 = hermes_read_regn(hw, RESP2);
	}

	hermes_write_regn(hw, EVACK, HERMES_EV_CMD);

	if (status & HERMES_STATUS_RESULT)
		err = -EIO;

 out:
	return err;
}

static int hermes_allocate(struct hermes *hw, u16 size, u16 *fid)
{
	int err = 0;
	unsigned int k;
	u16 reg;

	if (size < HERMES_ALLOC_LEN_MIN || size > HERMES_ALLOC_LEN_MAX)
		return -EINVAL;

	err = hermes_docmd_wait(hw, HERMES_CMD_ALLOC, size, NULL);
	if (err)
		return err;

	reg = hermes_read_regn(hw, EVSTAT);
	k = alloc_compl_timeout;
	while ((!(reg & HERMES_EV_ALLOC)) && k) {
		k--;
		udelay(10);
		reg = hermes_read_regn(hw, EVSTAT);
	}

	if (!hermes_present(hw)) {
		pr_warn("hermes @ %p: Card removed waiting for frame allocation.\n",
			hw->iobase);
		return -ENODEV;
	}

	if (!(reg & HERMES_EV_ALLOC)) {
		pr_err("hermes @ %p: Timeout waiting for frame allocation\n",
		       hw->iobase);
		return -ETIMEDOUT;
	}

	*fid = hermes_read_regn(hw, ALLOCFID);
	hermes_write_regn(hw, EVACK, HERMES_EV_ALLOC);

	return 0;
}

/* Set up a BAP to read a particular chunk of data from card's internal buffer.
 *
 * Returns:
 *     < 0 on internal failure (errno)
 *       0 on success
 *     > 0 on error
 * from firmware
 *
 * Callable from any context
 */
static int hermes_bap_seek(struct hermes *hw, int bap, u16 id, u16 offset)
{
	int sreg = bap ? HERMES_SELECT1 : HERMES_SELECT0;
	int oreg = bap ? HERMES_OFFSET1 : HERMES_OFFSET0;
	unsigned int k;
	u16 reg;

	/* Paranoia.. */
	if (offset > HERMES_BAP_OFFSET_MAX || (offset % 2))
		return -EINVAL;

	k = bap_busy_timeout;
	reg = hermes_read_reg(hw, oreg);
	while ((reg & HERMES_OFFSET_BUSY) && k) {
		k--;
		udelay(1);
		reg = hermes_read_reg(hw, oreg);
	}

	if (reg & HERMES_OFFSET_BUSY)
		return -ETIMEDOUT;

	/* Now we actually set up the transfer */
	hermes_write_reg(hw, sreg, id);
	hermes_write_reg(hw, oreg, offset);

	/* Wait for the BAP to be ready */
	k = bap_busy_timeout;
	reg = hermes_read_reg(hw, oreg);
	while ((reg & (HERMES_OFFSET_BUSY | HERMES_OFFSET_ERR)) && k) {
		k--;
		udelay(1);
		reg = hermes_read_reg(hw, oreg);
	}

	if (reg != offset) {
		pr_err("hermes @ %p: BAP%d offset %s: reg=0x%x id=0x%x offset=0x%x\n",
		       hw->iobase, bap,
		       (reg & HERMES_OFFSET_BUSY) ? "timeout" : "error",
		       reg, id, offset);

		if (reg & HERMES_OFFSET_BUSY)
			return -ETIMEDOUT;

		return -EIO;		/* error or wrong offset */
	}

	return 0;
}

/* Read a block of data from the chip's buffer, via the
 * BAP. Synchronization/serialization is the caller's problem.  len
 * must be even.
 *
 * Returns:
 *     < 0 on internal failure (errno)
 *       0 on success
 *     > 0 on error from firmware
 */
static int hermes_bap_pread(struct hermes *hw, int bap, void *buf, int len,
			    u16 id, u16 offset)
{
	int dreg = bap ? HERMES_DATA1 : HERMES_DATA0;
	int err = 0;

	if (len < 0 || (len % 2))
		return -EINVAL;

	err = hermes_bap_seek(hw, bap, id, offset);
	if (err)
		goto out;

	/* Actually do the transfer */
	hermes_read_words(hw, dreg, buf, len / 2);

 out:
	return err;
}

/* Write a block of data to the chip's buffer, via the
 * BAP. Synchronization/serialization is the caller's problem.
 *
 * Returns:
 *     < 0 on internal failure (errno)
 *       0 on success
 *     > 0 on error from firmware
 */
static int hermes_bap_pwrite(struct hermes *hw, int bap, const void *buf,
			     int len, u16 id, u16 offset)
{
	int dreg = bap ? HERMES_DATA1 : HERMES_DATA0;
	int err = 0;

	if (len < 0)
		return -EINVAL;

	err = hermes_bap_seek(hw, bap, id, offset);
	if (err)
		goto out;

	/* Actually do the transfer */
	hermes_write_bytes(hw, dreg, buf, len);

 out:
	return err;
}

/* Read a Length-Type-Value record from the card.
 *
 * If length is NULL, we ignore the length read from the card, and
 * read the entire buffer regardless. This is useful because some of
 * the configuration records appear to have incorrect lengths in
 * practice.
 *
 * Callable from user or bh context.
 */
static int hermes_read_ltv(struct hermes *hw, int bap, u16 rid,
			   unsigned int bufsize, u16 *length, void *buf)
{
	int err = 0;
	int dreg = bap ? HERMES_DATA1 : HERMES_DATA0;
	u16 rlength, rtype;
	unsigned int nwords;

	if (bufsize % 2)
		return -EINVAL;

	err = hermes_docmd_wait(hw, HERMES_CMD_ACCESS, rid, NULL);
	if (err)
		return err;

	err = hermes_bap_seek(hw, bap, rid, 0);
	if (err)
		return err;

	rlength = hermes_read_reg(hw, dreg);

	if (!rlength)
		return -ENODATA;

	rtype = hermes_read_reg(hw, dreg);

	if (length)
		*length = rlength;

	if (rtype != rid)
		pr_warn("hermes @ %p: %s(): rid (0x%04x) does not match type (0x%04x)\n",
			hw->iobase, __func__, rid, rtype);
	if (HERMES_RECLEN_TO_BYTES(rlength) > bufsize)
		pr_warn("hermes @ %p: Truncating LTV record from %d to %d bytes. (rid=0x%04x, len=0x%04x)\n",
			hw->iobase, HERMES_RECLEN_TO_BYTES(rlength), bufsize,
			rid, rlength);

	nwords = min((unsigned int)rlength - 1, bufsize / 2);
	hermes_read_words(hw, dreg, buf, nwords);

	return 0;
}

static int hermes_write_ltv(struct hermes *hw, int bap, u16 rid,
			    u16 length, const void *value)
{
	int dreg = bap ? HERMES_DATA1 : HERMES_DATA0;
	int err = 0;
	unsigned int count;

	if (length == 0)
		return -EINVAL;

	err = hermes_bap_seek(hw, bap, rid, 0);
	if (err)
		return err;

	hermes_write_reg(hw, dreg, length);
	hermes_write_reg(hw, dreg, rid);

	count = length - 1;

	hermes_write_bytes(hw, dreg, value, count << 1);

	err = hermes_docmd_wait(hw, HERMES_CMD_ACCESS | HERMES_CMD_WRITE,
				rid, NULL);

	return err;
}

/*** Hermes AUX control ***/

static inline void
hermes_aux_setaddr(struct hermes *hw, u32 addr)
{
	hermes_write_reg(hw, HERMES_AUXPAGE, (u16)(addr >> 7));
	hermes_write_reg(hw, HERMES_AUXOFFSET, (u16)(addr & 0x7F));
}

static inline int
hermes_aux_control(struct hermes *hw, int enabled)
{
	int desired_state = enabled ? HERMES_AUX_ENABLED : HERMES_AUX_DISABLED;
	int action = enabled ? HERMES_AUX_ENABLE : HERMES_AUX_DISABLE;
	int i;

	/* Already open? */
	if (hermes_read_reg(hw, HERMES_CONTROL) == desired_state)
		return 0;

	hermes_write_reg(hw, HERMES_PARAM0, HERMES_AUX_PW0);
	hermes_write_reg(hw, HERMES_PARAM1, HERMES_AUX_PW1);
	hermes_write_reg(hw, HERMES_PARAM2, HERMES_AUX_PW2);
	hermes_write_reg(hw, HERMES_CONTROL, action);

	for (i = 0; i < 20; i++) {
		udelay(10);
		if (hermes_read_reg(hw, HERMES_CONTROL) ==
		    desired_state)
			return 0;
	}

	return -EBUSY;
}

/*** Hermes programming ***/

/* About to start programming data (Hermes I)
 * offset is the entry point
 *
 * Spectrum_cs' Symbol fw does not require this
 * wl_lkm Agere fw does
 * Don't know about intersil
 */
static int hermesi_program_init(struct hermes *hw, u32 offset)
{
	int err;

	/* Disable interrupts?*/
	/*hw->inten = 0x0;*/
	/*hermes_write_regn(hw, INTEN, 0);*/
	/*hermes_set_irqmask(hw, 0);*/

	/* Acknowledge any outstanding command */
	hermes_write_regn(hw, EVACK, 0xFFFF);

	/* Not 0x0100 | HERMES_CMD_INIT: Intersil Prism 2.5 firmware fails
	 * that command with status 0x7f00.
	 */
	err = hw->ops->init_cmd_wait(hw,
				     0x0000 | HERMES_CMD_INIT,
				     0, 0, 0, NULL);
	if (err)
		return err;

	err = hermes_aux_control(hw, 1);
	pr_debug("AUX enable returned %d\n", err);

	if (err)
		return err;

	pr_debug("Enabling volatile, EP 0x%08x\n", offset);
	err = hw->ops->init_cmd_wait(hw,
				     HERMES_PROGRAM_ENABLE_VOLATILE,
				     offset & 0xFFFFu,
				     offset >> 16,
				     0,
				     NULL);
	pr_debug("PROGRAM_ENABLE returned %d\n", err);

	return err;
}

/* Done programming data (Hermes I)
 *
 * Spectrum_cs' Symbol fw does not require this
 * wl_lkm Agere fw does
 * Don't know about intersil
 */
static int hermesi_program_end(struct hermes *hw)
{
	struct hermes_response resp;
	int rc = 0;
	int err;

	rc = hw->ops->cmd_wait(hw, HERMES_PROGRAM_DISABLE, 0, &resp);

	pr_debug("PROGRAM_DISABLE returned %d, r0 0x%04x, r1 0x%04x, r2 0x%04x\n",
		 rc, resp.resp0, resp.resp1, resp.resp2);

	if (rc == 0 &&
	    ((resp.status & HERMES_STATUS_CMDCODE) != HERMES_CMD_DOWNLD))
		rc = -EIO;

	/*
	 * PROGRAM_DISABLE makes the hardware restart itself from the entry
	 * point just programmed, and it does not answer registers until the
	 * restart is done.
	 */
	usleep_range(5000, 10000);

	err = hermes_aux_control(hw, 0);
	pr_debug("AUX disable returned %d\n", err);

	/* Acknowledge any outstanding command */
	hermes_write_regn(hw, EVACK, 0xFFFF);

	/* More settle time before the restarted firmware's first command */
	usleep_range(5000, 10000);

	/* Reinitialise, ignoring return */
	(void)hw->ops->init_cmd_wait(hw, 0x0000 | HERMES_CMD_INIT,
				      0, 0, 0, NULL);

	return rc ? rc : err;
}

static int hermes_program_bytes(struct hermes *hw, const char *data,
				u32 addr, u32 len)
{
	/* wl lkm splits the programming into chunks of 2000 bytes.
	 * This restriction appears to come from USB. The PCMCIA
	 * adapters can program the whole lot in one go
	 */
	hermes_aux_setaddr(hw, addr);
	hermes_write_bytes(hw, HERMES_AUXDATA, data, len);
	return 0;
}

static int hermes_aux_enable_op(struct hermes *hw, int enable)
{
	return hermes_aux_control(hw, enable);
}

static int hermes_aux_read_bytes(struct hermes *hw, u32 addr, void *buf,
				 u32 len)
{
	hermes_aux_setaddr(hw, addr);
	hermes_read_words(hw, HERMES_AUXDATA, buf, len / 2);
	return 0;
}

/* Read PDA from the adapter */
static int hermes_read_pda(struct hermes *hw, __le16 *pda, u32 pda_addr,
			   u16 pda_len)
{
	int ret;
	u16 pda_size;
	u16 data_len = pda_len;
	__le16 *data = pda;

	if (hw->eeprom_pda) {
		/* PDA of spectrum symbol is in eeprom */

		/* Issue command to read EEPROM */
		ret = hw->ops->cmd_wait(hw, HERMES_CMD_READMIF, 0, NULL);
		if (ret)
			return ret;
	} else {
		/* wl_lkm does not include PDA size in the PDA area.
		 * We will pad the information into pda, so other routines
		 * don't have to be modified
		 */
		pda[0] = cpu_to_le16(pda_len - 2);
			/* Includes CFG_PROD_DATA but not itself */
		pda[1] = cpu_to_le16(0x0800); /* CFG_PROD_DATA */
		data_len = pda_len - 4;
		data = pda + 2;
	}

	/* Open auxiliary port */
	ret = hermes_aux_control(hw, 1);
	pr_debug("AUX enable returned %d\n", ret);
	if (ret)
		return ret;

	/* Read PDA */
	hermes_aux_setaddr(hw, pda_addr);
	hermes_read_words(hw, HERMES_AUXDATA, data, data_len / 2);

	/* Close aux port */
	ret = hermes_aux_control(hw, 0);
	pr_debug("AUX disable returned %d\n", ret);
	if (ret)
		return ret;

	/* Check PDA length */
	pda_size = le16_to_cpu(pda[0]);
	pr_debug("Actual PDA length %d, Max allowed %d\n",
		 pda_size, pda_len);
	if (pda_size > pda_len)
		return -EINVAL;

	return 0;
}

static void hermes_lock_irqsave(spinlock_t *lock,
				unsigned long *flags) __acquires(lock)
{
	spin_lock_irqsave(lock, *flags);
}

static void hermes_unlock_irqrestore(spinlock_t *lock,
				     unsigned long *flags) __releases(lock)
{
	spin_unlock_irqrestore(lock, *flags);
}

static void hermes_lock_irq(spinlock_t *lock) __acquires(lock)
{
	spin_lock_irq(lock);
}

static void hermes_unlock_irq(spinlock_t *lock) __releases(lock)
{
	spin_unlock_irq(lock);
}

/* Hermes operations for local buses */
static const struct hermes_ops hermes_ops_local = {
	.init = hermes_init,
	.cmd_wait = hermes_docmd_wait,
	.init_cmd_wait = hermes_doicmd_wait,
	.allocate = hermes_allocate,
	.read_ltv = hermes_read_ltv,
	.read_ltv_pr = hermes_read_ltv,
	.write_ltv = hermes_write_ltv,
	.bap_pread = hermes_bap_pread,
	.bap_pwrite = hermes_bap_pwrite,
	.read_pda = hermes_read_pda,
	.program_init = hermesi_program_init,
	.program_end = hermesi_program_end,
	.program = hermes_program_bytes,
	.aux_enable = hermes_aux_enable_op,
	.aux_read = hermes_aux_read_bytes,
	.lock_irqsave = hermes_lock_irqsave,
	.unlock_irqrestore = hermes_unlock_irqrestore,
	.lock_irq = hermes_lock_irq,
	.unlock_irq = hermes_unlock_irq,
};

MODULE_DESCRIPTION("Driver core for the Hermes wireless MAC controller");
MODULE_LICENSE("Dual MPL/GPL");
