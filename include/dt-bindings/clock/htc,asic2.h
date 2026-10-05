/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Device tree binding constants for the clock outputs of the HTC ASIC2
 * companion chip (drivers/mfd/asic2.c), found on the HP/Compaq iPAQ
 * h38xx/h39xx family.
 *
 * ASIC2_CLK_AUDIO is index 0, the audio clock tap selector. Every other
 * index is the ASIC2_GATE_* enum value of the corresponding clock gate in
 * drivers/mfd/asic2.c plus one.
 */

#ifndef _DT_BINDINGS_CLOCK_HTC_ASIC2_H
#define _DT_BINDINGS_CLOCK_HTC_ASIC2_H

#define ASIC2_CLK_AUDIO		0
#define ASIC2_CLK_ADC		1
#define ASIC2_CLK_SPI		2
#define ASIC2_CLK_EX1		3
#define ASIC2_CLK_OWM		4
#define ASIC2_CLK_PWM		5
#define ASIC2_CLK_UART_0	6
#define ASIC2_CLK_UART_1	7
#define ASIC2_CLK_SLOW_UART_0	8
#define ASIC2_CLK_SLOW_UART_1	9
#define ASIC2_CLK_EX2		10
#define ASIC2_CLK_SD		11

#endif
