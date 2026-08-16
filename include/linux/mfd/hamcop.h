/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Register map and shared constants for the Samsung HAMCOP companion ASIC
 * (HP iPAQ h2200/h2210/h2212/h2215).
 *
 * HAMCOP is a S3C24xx-class "SAMCOP" companion chip: NAND controller, SDI
 * (SD/MMC) host, ADC + resistive touchscreen front-end, a DS1WM-compatible
 * 1-Wire master, a 5-channel blink-capable LED controller, a 32-line GPIO
 * block usable as edge/level interrupt sources, and 16 KiB of on-chip SRAM
 * that doubles as the h2200's WinCE-bootloader resume vector (owned
 * entirely by arch/arm/mach-pxa/h2200_pm.c, *not* by this driver -- see
 * below).
 *
 * This header is a from-scratch rewrite against current MFD/irqdomain/CCF
 * conventions, not a straight port of the old out-of-tree ipaq-hamcop.h.
 * Register names and offsets are
 * translated from that file (itself sourced from HP's h22xx ASIC Companion
 * specification), but every offset below is expressed relative to *this
 * driver's own* memory mapping, which starts HAMCOP_SRAM_SIZE bytes into
 * the chip -- not relative to the chip's physical base the way the 2007
 * header expressed them. See the "disjoint mem resource" comment on
 * HAMCOP_SRAM_SIZE for why.
 *
 * Copyright (C) 2026 Koen Kooi
 */
#ifndef __LINUX_MFD_HAMCOP_H
#define __LINUX_MFD_HAMCOP_H

#include <linux/bits.h>
#include <linux/sizes.h>
#include <linux/types.h>

struct device;

/*
 * HAMCOP's on-chip SRAM sits at chip offset 0, and is described to the MFD
 * as a second, separate memory resource covering exactly [chip base, chip
 * base + HAMCOP_SRAM_SIZE). The MFD never maps or touches that range; it
 * passes it to a "hamcop-sram" sub-device, which on h2200 is
 * arch/arm/mach-pxa/h2200_pm.c, owner of the bootloader-preservation dance
 * around suspend and reboot. The MFD's own register resource therefore
 * starts right after the SRAM, at chip offset HAMCOP_SRAM_SIZE, and every
 * register offset in this header is relative to *that* mapping, not to the
 * chip's physical base address.
 *
 * Real hardware fact worth keeping in mind even though software ownership
 * is split: the SRAM block is physically powered by the NAND clock gate
 * (HAMCOP_CLK_NAND below). This driver forces that gate on at probe and
 * never lets NAND/SDI turn it off from under each other -- see the probe
 * function's comment.
 */
#define HAMCOP_SRAM_SIZE		SZ_16K

/* Total chip register window, SRAM included -- matches the 2007 driver's
 * HAMCOP_MAP_SIZE. This driver's own resource is HAMCOP_MAP_SIZE -
 * HAMCOP_SRAM_SIZE bytes, i.e. [HAMCOP_SRAM_SIZE, HAMCOP_MAP_SIZE).
 */
#define HAMCOP_MAP_SIZE			SZ_1M

/* ---------------------------------------------------------------------
 * CPU Interface Control (CIF)
 * ---------------------------------------------------------------------
 */
#define HAMCOP_CIF_IFMODE		0x0000
#define HAMCOP_CIF_BOOTMODE		0x0002

#define HAMCOP_CIF_IFMODE_SRAM		0x0000	/* boot-code read mode */
#define HAMCOP_CIF_IFMODE_VLIO		0x0001	/* normal runtime mode */

/* ---------------------------------------------------------------------
 * DMA controller -- one channel, private to the hamcop-sdi driver, which
 * decodes the registers itself. No register-level decode needed by the
 * core; only the cell resource window is defined.
 * ---------------------------------------------------------------------
 */
#define HAMCOP_DMA_BASE			0x0400
#define HAMCOP_DMA_SIZE			0x002f

/* ---------------------------------------------------------------------
 * Interrupt controller (IC) -- flat 26-line pending/mask, see
 * "IRQ demux" in hamcop.c.
 * ---------------------------------------------------------------------
 */
#define HAMCOP_IC_SRCPND0		0x0800	/* raw pending, w1c */
#define HAMCOP_IC_SRCPND1		0x0804
#define HAMCOP_IC_INTMSK0		0x0808	/* 1 = masked */
#define HAMCOP_IC_INTMSK1		0x080c
#define HAMCOP_IC_PRIORITY0		0x0810
#define HAMCOP_IC_PRIORITY1		0x0814
#define HAMCOP_IC_INTPND0		0x0818	/* pending, post-mask, w1c */
#define HAMCOP_IC_INTPND1		0x081c
#define HAMCOP_IC_DREQ			0x0820

/*
 * hwirq numbers in this linear irq_domain. 0-15 are GPIO bank A/B pins
 * (identity-mapped with the gpio_chip's offsets 0-15, see hamcop.c);
 * 16-25 are fixed-function block interrupts with no GPIO backing and no
 * hardware trigger-type control.
 */
#define HAMCOP_IRQ_RSTBUTTON		0
#define HAMCOP_IRQ_APPBUTTON1		1
#define HAMCOP_IRQ_APPBUTTON2		2
#define HAMCOP_IRQ_APPBUTTON3		3
#define HAMCOP_IRQ_APPBUTTON4		4
#define HAMCOP_IRQ_JOYPAD_LEFT		5
#define HAMCOP_IRQ_JOYPAD_RIGHT		6
#define HAMCOP_IRQ_BATT_FLT		7
#define HAMCOP_IRQ_JOYPAD_DOWN		8
#define HAMCOP_IRQ_JOYPAD_UP		9
#define HAMCOP_IRQ_JOYPAD_ACTION	10
#define HAMCOP_IRQ_SD_DETECT		11
#define HAMCOP_IRQ_GPB4			12
#define HAMCOP_IRQ_GPB5			13
#define HAMCOP_IRQ_GPB6			14
#define HAMCOP_IRQ_EAR_IN		15
#define HAMCOP_IRQ_GPIO_MAX		16	/* first non-GPIO hwirq */
#define HAMCOP_IRQ_DMA			16
#define HAMCOP_IRQ_TICK			17
#define HAMCOP_IRQ_ALARM		18
#define HAMCOP_IRQ_ONEWIRE		19
#define HAMCOP_IRQ_SDHC			20
#define HAMCOP_IRQ_ADCTS		21
#define HAMCOP_IRQ_ADC			22
#define HAMCOP_IRQ_WDT			23
#define HAMCOP_IRQ_KEYPAD		24
#define HAMCOP_IRQ_NFIF			25
#define HAMCOP_NR_IRQS			26

/* ---------------------------------------------------------------------
 * Clock and Power Management (CPM)
 * ---------------------------------------------------------------------
 */
#define HAMCOP_CPM_LOCKTIME		0x0c00	/* reset default 0x0fff */
#define HAMCOP_CPM_PLLCONTROL		0x0c04	/* reset default 0x2820 */
#define HAMCOP_CPM_CLOCKCONTROL		0x0c08	/* reset default 0x03fe */
#define HAMCOP_CPM_CLOCKSLEEP		0x0c0c

/* ClockControl gate bits -- one CCF clk_hw gate per bit below (see
 * hamcop.c's hamcop_gates[]), except PLLON which is not exposed as a
 * consumer-facing gate: it is the chip's own PLL enable and is forced on
 * unconditionally at probe, matching the 2007 driver.
 */
#define HAMCOP_CLK_PLLON		BIT(0)	/* not a CCF gate, see above */
#define HAMCOP_CLK_KEYPAD		BIT(1)
#define HAMCOP_CLK_DMAC			BIT(2)
#define HAMCOP_CLK_NAND			BIT(3)	/* also powers HAMCOP SRAM */
#define HAMCOP_CLK_LED			BIT(4)
#define HAMCOP_CLK_RTC			BIT(5)	/* LED blink timebase too */
#define HAMCOP_CLK_SD			BIT(6)
#define HAMCOP_CLK_1WIRE		BIT(7)
#define HAMCOP_CLK_ADC			BIT(8)
#define HAMCOP_CLK_GPIO			BIT(9)	/* needed by our gpio_chip */

#define HAMCOP_CLKSLEEP_XTCON		BIT(7)
#define HAMCOP_CLKSLEEP_UCLK_ON	BIT(3)
#define HAMCOP_CLKSLEEP_CLKSEL		BIT(1)	/* 0 = mclk/2, 1 = mclk */
#define HAMCOP_CLKSLEEP_SLEEP		BIT(0)

/* ---------------------------------------------------------------------
 * NAND flash controller -- cell resource window only; register-level
 * decode belongs to the hamcop-nand driver.
 * ---------------------------------------------------------------------
 */
#define HAMCOP_NF_BASE			0x1000
#define HAMCOP_NF_SIZE			0x0047

/* ---------------------------------------------------------------------
 * Watchdog timer -- no cell registered (h2200 has no board policy for it
 * yet). WTCON is disabled unconditionally at probe: see the comment
 * there. Offsets otherwise kept for documentation/future use.
 * ---------------------------------------------------------------------
 */
#define HAMCOP_WT_BASE			0x2000
#define HAMCOP_WT_WTCON			(HAMCOP_WT_BASE + 0x0000)

/* ---------------------------------------------------------------------
 * RTC -- no cell registered. HAMCOP's internal RTC block is only used
 * on h2200 as the LED blink clock source (HAMCOP_CLK_RTC above); the
 * system RTC is PXA25x's own. Offsets kept for documentation.
 * ---------------------------------------------------------------------
 */
#define HAMCOP_RTC_BASE			0x2400
#define HAMCOP_RTC_SIZE			0x008c

/* ---------------------------------------------------------------------
 * DS1WM-compatible 1-Wire master -- cell resource window only.
 * ---------------------------------------------------------------------
 */
#define HAMCOP_OWM_BASE			0x2800
#define HAMCOP_OWM_SIZE			0x0014

/* ---------------------------------------------------------------------
 * SDI (SD/MMC host) -- cell resource window only; register-level decode
 * belongs to the hamcop-sdi driver.
 * ---------------------------------------------------------------------
 */
#define HAMCOP_SDI_BASE			0x2c00
#define HAMCOP_SDI_SIZE			0x006f

/* ---------------------------------------------------------------------
 * GPIO -- four 8-pin banks (A/B/C/D). Only A and B have per-pin interrupt
 * capability (GPAINT0/1, GPBINT0/1) and are the ones exposed through the
 * shared hwirq/gpio-offset numbering above; C and D are plain GPIO.
 * ---------------------------------------------------------------------
 */
#define HAMCOP_GPIO_BANK_STRIDE		0x0010
#define HAMCOP_GPIO_NR_BANKS		4
#define HAMCOP_GPIO_PINS_PER_BANK	8
#define HAMCOP_NR_GPIO			(HAMCOP_GPIO_NR_BANKS * \
					 HAMCOP_GPIO_PINS_PER_BANK)	/* 32 */

#define HAMCOP_GPIO_GPA_CON		0x3000
#define HAMCOP_GPIO_GPB_CON		0x3010
#define HAMCOP_GPIO_GPC_CON		0x3020
#define HAMCOP_GPIO_GPD_CON		0x3030

/* CON/DAT/PUP are +0x0/+0x4/+0x8 within each bank's 0x10 stride. */
#define HAMCOP_GPIO_BANK_CON(bank)	(HAMCOP_GPIO_GPA_CON + \
					 (bank) * HAMCOP_GPIO_BANK_STRIDE)
#define HAMCOP_GPIO_BANK_DAT(bank)	(HAMCOP_GPIO_BANK_CON(bank) + 0x0004)
#define HAMCOP_GPIO_BANK_PUP(bank)	(HAMCOP_GPIO_BANK_CON(bank) + 0x0008)

#define HAMCOP_GPIO_CON_MODE_INPUT	0x0
#define HAMCOP_GPIO_CON_MODE_OUTPUT	0x1
#define HAMCOP_GPIO_CON_MASK(pin)	(0x3 << (2 * (pin)))
#define HAMCOP_GPIO_CON_OUTPUT(pin)	(HAMCOP_GPIO_CON_MODE_OUTPUT << (2 * (pin)))

#define HAMCOP_GPIO_CLKOUTCON		0x3040

/* GPA/GPB per-pin interrupt trigger + enable, 4 bits/pin, 4 pins/register:
 * pins 0-3 -> GPAINT0, 4-7 -> GPAINT1, 8-11(GPB0-3) -> GPBINT0,
 * 12-15(GPB4-7) -> GPBINT1.
 */
#define HAMCOP_GPIO_GPAINT0		0x3050
#define HAMCOP_GPIO_GPAINT1		0x3054
#define HAMCOP_GPIO_GPBINT0		0x3058
#define HAMCOP_GPIO_GPBINT1		0x305c

#define HAMCOP_GPIO_INT_MASK(nibble)	(0xf << (4 * (nibble)))
#define HAMCOP_GPIO_INT_INTLV_LOW	0x0
#define HAMCOP_GPIO_INT_INTLV_HIGH	0x1
#define HAMCOP_GPIO_INT_INTLV_RISING	0x2
#define HAMCOP_GPIO_INT_INTLV_FALLING	0x4
#define HAMCOP_GPIO_INT_INTLV_BOTH	0x6
#define HAMCOP_GPIO_INT_INTEN		0x8

#define HAMCOP_GPIO_MON			0x30e0
/*
 * SPUCR ("sleep pull-up control") gates HAMCOP's own internal pull-ups on
 * the pin groups its sub-blocks drive. Every bit is a pull-up *disable*:
 * 0 = pull-up present, 1 = pull-up removed. The chip's reset/bootloader
 * value is not documented anywhere this project has recovered, so a block
 * that needs its pull-ups must set them explicitly rather than assume.
 *
 * SD_CMD and SD_DAT are the pins the SD/MMC bus specification requires the
 * *host* to pull up; on h2200 these two bits are the only thing providing
 * that. The old out-of-tree port does exactly this when it powers the
 * slot -- clearing both bits when the slot is powered and setting them
 * again when it is powered down, so an empty slot doesn't burn current
 * through the pull-ups.
 * drivers/mmc/host/hamcop-sdi.c is the mainline equivalent of that call.
 *
 * PU_fIO is the same mechanism for the NAND flash IO group
 * (the old out-of-tree NAND driver toggles it around chip select).
 * Defined here for completeness; drivers/mtd/nand/raw/
 * hamcop_nand.c does not use it yet -- NAND works without it, since that
 * is a leakage optimisation rather than a bus-integrity requirement.
 */
#define HAMCOP_GPIO_SPUCR		0x30e4
#define HAMCOP_GPIO_SPUCR_PU_SDDA	BIT(0)
#define HAMCOP_GPIO_SPUCR_PU_SDCM	BIT(1)
#define HAMCOP_GPIO_SPUCR_PU_fIO	BIT(2)

/* ---------------------------------------------------------------------
 * Keypad matrix -- unused on h2200 (buttons/joypad are individual GPIO
 * lines handled by a stock gpio-keys node against this driver's
 * gpio_chip). No cell registered, offsets not defined.
 * ---------------------------------------------------------------------
 */

/* ---------------------------------------------------------------------
 * LED controller -- 5 channels, cell resource window only.
 * ---------------------------------------------------------------------
 */
#define HAMCOP_LED_BASE			0x3200
#define HAMCOP_LED_SIZE			0x002f
#define HAMCOP_LED_NR_CHANNELS		5

/* ---------------------------------------------------------------------
 * ADC + resistive touchscreen front-end -- register decode lives in
 * drivers/iio/adc/hamcop-adc.c itself (that driver ioremaps only this
 * BASE/SIZE window, so its own register offsets are 0-based, unlike the
 * GPIO/IC/CPM decodes above which share this file's single MFD-wide
 * mapping). The "hamcop-adc" cell owns both this MEM window and both of
 * the block's hwirqs (HAMCOP_IRQ_ADC conversion-complete and
 * HAMCOP_IRQ_ADCTS pen-contact-detect) -- unlike the 2007 driver's
 * "samcop adc"/"ts-adc" split, mainline's generic resistive-adc-touch
 * consumer (drivers/input/touchscreen/resistive-adc-touch.c) has no
 * notion of interrupts or registers at all, only IIO channels, so it is
 * wired up as a second, resource-less mfd_cell instead (see hamcop.c).
 * ---------------------------------------------------------------------
 */
#define HAMCOP_ADC_BASE			0x3800
#define HAMCOP_ADC_SIZE			0x0014

/*
 * SPUCR lives in the MFD core's own register window, not in any sub-cell's
 * resource, and is shared between blocks (SD pull-ups and the NAND fIO
 * pull-ups are bits in the same register), so it is reached through the
 * core's locked read-modify-write rather than mapped a second time. Pass
 * the *parent* device -- a cell driver's own dev->parent.
 */
void hamcop_set_spucr(struct device *hamcop_dev, u16 mask, u16 bits);

#endif /* __LINUX_MFD_HAMCOP_H */
