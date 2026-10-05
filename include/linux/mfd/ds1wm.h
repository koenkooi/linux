/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Platform data for the DS1WM 1-Wire bus master core embedded in several
 * HP iPAQ companion ASICs (HAMCOP, ASIC3, PASIC3, SAMCOP) and standalone
 * DS1WM/FPGA instantiations.
 */
#ifndef __LINUX_MFD_DS1WM_H
#define __LINUX_MFD_DS1WM_H

struct ds1wm_driver_data {
	/*
	 * Register file stride: the byte address of logical register N
	 * (0=CMD, 1=DATA, 2=INT, 3=INT_EN, 4=CLKDIV) is N << bus_shift.
	 * It is board wiring: 2 on h5xxx and h2200's HAMCOP (registers 4-byte
	 * aligned), 1 on hx4700's ASIC3 (registers on 16-bit bus words).
	 * Register access itself is always byte-wide regardless of the
	 * stride.
	 */
	unsigned int bus_shift;

	/*
	 * DS1WM_INTEN's IAS bit: 1 if the DS1WM-to-parent-ASIC interrupt
	 * line is active-high, 0 if active-low. This is chip-internal
	 * wiring, not board policy.
	 */
	int active_high;

	/*
	 * Milliseconds to sleep after a bus reset pulse before the next
	 * transaction. 0 is fine for any 1-wire-spec-compliant slave --
	 * DS1WM's own reset/presence-detect timing is already exact.
	 */
	unsigned int reset_recover_delay;
};

#endif
