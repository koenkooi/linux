/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GPIO numbers used directly by h2200.c: buttons/wakeup sources and the
 * fixed-policy output pins that get gpio_request_one() hogs. The
 * CompactFlash socket's lines have no entry here. Every other GPIO is
 * muxed in h2200_pin_config[] without ever needing a symbolic name here --
 * see the old out-of-tree h2200-gpio.h for the full historical GPIO map
 * this was translated from.
 */
#ifndef __ASM_ARCH_H2200_H
#define __ASM_ARCH_H2200_H

#define H2200_GPIO_POWER_BUTTON		0	/* function unverified */
#define H2200_GPIO_RESET_BUTTON_N	1	/* function unverified */
#define H2200_GPIO_AC_IN_N		12	/* AC adapter presence */
#define H2200_GPIO_POWER_ON_N		13	/* power key */
#define H2200_GPIO_RS232_CIR_N		24	/* 1 -> RS232, 0 -> CIR */
#define H2200_GPIO_IR_ON_N		64
#define H2200_GPIO_CIR_RESET		72
#define H2200_GPIO_CIR_POWER_ON	73
#define H2200_GPIO_RS232_ON		80

#endif /* __ASM_ARCH_H2200_H */
