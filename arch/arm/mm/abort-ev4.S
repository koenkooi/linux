/* SPDX-License-Identifier: GPL-2.0 */
#include <linux/linkage.h>
#include <asm/assembler.h>
/*
 * Function: v4_early_abort
 *
 * Params  : r2 = pt_regs
 *	   : r4 = aborted context pc
 *	   : r5 = aborted context psr
 *
 * Returns : r4 - r11, r13 preserved
 *
 * Purpose : obtain information about current aborted instruction.
 * Note: we read user space.  This means we might cause a data
 * abort here if the I-TLB and D-TLB aren't seeing the same
 * picture.  Unfortunately, this does happen.  We live with it.
 */
	.align	5
ENTRY(v4_early_abort)
	mrc	p15, 0, r1, c5, c0, 0		@ get FSR
	mrc	p15, 0, r0, c6, c0, 0		@ get FAR
	ldr	r3, [r4]			@ read aborted ARM instruction
	uaccess_disable ip			@ disable userspace access
	/*
	 * FSR[31:10] are undefined on read on SA-1100/SA-1110. Keep only the
	 * domain and status fields: is_write_fault() reads bit 13 as FSR_CM,
	 * which would misreport write faults as reads when it reads back set.
	 * Bit 11 is re-derived below; bit 10 stays clear, as before.
	 */
	and	r1, r1, #0xff			@ keep only domain + status[3:0]
	tst	r3, #1 << 20			@ L = 1 -> write?
	orreq	r1, r1, #1 << 11		@ yes.
	b	do_DataAbort
