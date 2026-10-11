/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __GPIO_PXA_H
#define __GPIO_PXA_H

#define GPIO_bit(x)	(1 << ((x) & 0x1f))

#define gpio_to_bank(gpio)	((gpio) >> 5)

/* NOTE: some PXAs have fewer on-chip GPIOs (like PXA255, with 85).
 * Those cases currently cause holes in the GPIO number space, the
 * actual number of the last GPIO is recorded by 'pxa_last_gpio'.
 */
extern int pxa_last_gpio;

extern int pxa_irq_to_gpio(int irq);

struct pxa_gpio_platform_data {
	int irq_base;
	int (*gpio_set_wake)(unsigned int gpio, unsigned int on);
};

/*
 * A DT boot has no platform data, so the set_wake callback of the gpio-pxa
 * chip, the backend of its irq_set_wake, would stay NULL and PWER/PRER/PFER
 * would never be programmed for a GPIO wake source, while irq_set_wake()
 * still reports success. The PXA25x/PXA27x gpio_set_wake() lives in
 * arch/arm/mach-pxa/mfp-pxa2xx.c, because it needs the private MFP table,
 * so pxa25x_dt_init_irq() and pxa27x_dt_init_irq() publish it here, early
 * enough for pxa_gpio_probe_dt() to wire it into the chip it creates. It
 * stays NULL on SoCs that have no such callback, which gpio-pxa treats as
 * nothing to do. On a DT boot the callback also takes the trigger type of
 * the interrupt, so that a pad no MFP table marked as a wake source can
 * still be armed for the edges its consumer requested.
 */
extern int (*pxa_gpio_dt_set_wake)(unsigned int gpio, unsigned int type,
				   unsigned int on);

#endif /* __GPIO_PXA_H */
