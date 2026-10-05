/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Clock indices of the Intel StrongARM SA-1100/SA-1110 clock controller
 */

#ifndef _DT_BINDINGS_CLOCK_INTEL_SA1100_CLOCKS_H
#define _DT_BINDINGS_CLOCK_INTEL_SA1100_CLOCKS_H

/* Core clock (CCLK), the PLL output selected by PPCR.CCF */
#define SA1100_CLK_CPU			0

/* Clock driven on the GPIO27 alternate output, selected by TUCR.TSEL */
#define SA1100_CLK_GPIO27		1

/* Inputs of the TUCR.TSEL selector, in TSEL order 0-7 */
#define SA1100_CLK_TSEL_OSC32K		2
#define SA1100_CLK_TSEL_OSC3M6		3
#define SA1100_CLK_TSEL_VDD_RING_DIV16	4
#define SA1100_CLK_TSEL_PLL96_DIV4	5
#define SA1100_CLK_TSEL_OSC32K_RCLK	6
#define SA1100_CLK_TSEL_OSC3M6_ALT	7
#define SA1100_CLK_TSEL_PLL_DIV16	8
#define SA1100_CLK_TSEL_VDDL_RING_DIV4	9

#endif /* _DT_BINDINGS_CLOCK_INTEL_SA1100_CLOCKS_H */
