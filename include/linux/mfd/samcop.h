/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Register map and shared constants for the Samsung SAMCOP companion ASIC
 * (HP iPAQ h51xx/h54xx/h55xx).
 *
 * SAMCOP is the same silicon family as HAMCOP (include/linux/mfd/hamcop.h)
 * but a materially different part: a USB host controller, an interrupt
 * controller with 26 flat sources (two of which are themselves demux points),
 * an ADC and resistive touchscreen front-end, a DS1WM-compatible 1-Wire
 * master, a 5-channel LED blink controller, PCMCIA/expansion-pack glue, a
 * five-bank GPIO block, and 32 KiB of on-chip SRAM used as a DMA bounce pool.
 *
 * The two chips share the same IP blocks at different base offsets, and
 * IC_INTPND itself diverges (SAMCOP 0x10, HAMCOP 0x18), so no offset or bit
 * here is taken from hamcop.h by analogy.
 *
 * Offsets are relative to the chip's own base (chip offset 0 is the SRAM), not
 * to a mapping that starts after the SRAM window as for HAMCOP, whose SRAM
 * holds the WinCE bootloader's resume vector and has a second owner. SAMCOP's
 * SRAM is only a DMA bounce pool for the OHCI cell, so the driver maps the
 * whole 1 MiB window in one platform_device and hands the SRAM sub-range to
 * that cell as an ordinary resource.
 *
 * Copyright (C) 2026 Koen Kooi
 */
#ifndef __LINUX_MFD_SAMCOP_H
#define __LINUX_MFD_SAMCOP_H

#include <linux/bitops.h>
#include <linux/sizes.h>
#include <linux/types.h>

/* ---------------------------------------------------------------------
 * Chip-wide layout. 64 KiB-granular block bases, unlike HAMCOP's 1 KiB map.
 * ---------------------------------------------------------------------
 */
#define SAMCOP_SRAM_BASE		0x00000
#define SAMCOP_SRAM_SIZE		SZ_32K	/* DMA bounce pool */

#define SAMCOP_USBHOST_BASE		0x20000
/*
 * The USB host block is a stock OHCI controller, whose register file is a small
 * fixed layout. The window is sized to the next block base.
 */
#define SAMCOP_USBHOST_SIZE		0x10000

#define SAMCOP_IC_BASE			0x30000
#define SAMCOP_DMAC_BASE		0x40000	/* not used */
#define SAMCOP_CPM_BASE			0x50000
#define SAMCOP_ADC_BASE			0x80000
#define SAMCOP_UART_BASE		0x90000	/* no known consumer */
#define SAMCOP_MISC_BASE		0xa0000
#define SAMCOP_OWM_BASE			0xa0100
#define SAMCOP_FSI_BASE			0xb0000	/* h54xx/h55xx only */
/*
 * Cell resource window only; the register decode lives in the fingerprint
 * driver. 32 bytes, of which the six defined registers use the first 24.
 */
#define SAMCOP_FSI_SIZE			0x0020
#define SAMCOP_SDI_BASE			0xb0030
#define SAMCOP_GPIO_BASE		0xc0000
#define SAMCOP_LED_BASE			0xc1000
#define SAMCOP_PCMCIA_BASE		0xd0000	/* EPS: expansion-pack status/IRQ glue */

/* Total span of the chip's register window. */
#define SAMCOP_MAP_SIZE			SZ_1M

/* ---------------------------------------------------------------------
 * Interrupt controller (IC) -- flat 26-line pending/mask, 32-bit
 * registers (unlike HAMCOP's paired 16-bit SRCPND0/1 etc.). Two of the
 * 26 sources are themselves demux points into further hwirq ranges
 * appended after the IC's own 0-25 -- see "hwirq layout" below and
 * drivers/mfd/samcop.c's IRQ demux section.
 * ---------------------------------------------------------------------
 */
#define SAMCOP_IC_SRCPND		(SAMCOP_IC_BASE + 0x00000) /* raw pending, w1c */
#define SAMCOP_IC_INTMSK		(SAMCOP_IC_BASE + 0x00008) /* 1 = masked */
#define SAMCOP_IC_PRIORITY		(SAMCOP_IC_BASE + 0x0000c)
#define SAMCOP_IC_INTPND		(SAMCOP_IC_BASE + 0x00010) /* pending, post-mask, w1c */
#define SAMCOP_IC_DREQ			(SAMCOP_IC_BASE + 0x00018)

/*
 * hwirq layout of the driver's single linear irq_domain: IC sources 0-25, then
 * the GPIO block's own 14-source fan-out (hwirq 26-39), then the EPS block's
 * 6-source fan-out (hwirq 40-45); SAMCOP_NR_IRQS = IC(26) + GPIO(14) + EPS(6) =
 * 46. Every IC source is a flat ack/mask/unmask line, except
 * SAMCOP_IRQ_SD_WAKEUP (13), which also has an .irq_set_type
 * (samcop_ic_irq_set_type()).
 *
 * Nine of the 26 IC sources -- APPBUTTON1-4, RECORD, WAKEUP1-3 and SD_WAKEUP
 * -- are backed by GPA pins wired in EXTIN mode. Each needs its GPA pin muxed
 * to SAMCOP_GPIO_CON_MODE_EXTIN and a GPIO_INT/GPIO_ENINT1 trigger and enable
 * pair programmed before it fires; unmasking it in IC_INTMSK is not enough.
 * The probe writes the board's static values for eight of them (everything
 * but SD_WAKEUP), since which GPA pin backs which button is board wiring.
 * SD_WAKEUP is armed by the MFD core itself because its consumer, the SD
 * host, requests it through the ordinary IRQ API.
 */
#define SAMCOP_IRQ_APPBUTTON1		0
#define SAMCOP_IRQ_APPBUTTON2		1
#define SAMCOP_IRQ_APPBUTTON3		2
#define SAMCOP_IRQ_APPBUTTON4		3
#define SAMCOP_IRQ_RECORD		4
#define SAMCOP_IRQ_PCMCIA		5	/* demux -> hwirq 40-45 (EPS) */
#define SAMCOP_IRQ_FCD			6	/* fingerprint */
#define SAMCOP_IRQ_ONEWIRE		7
#define SAMCOP_IRQ_UERR0		8
#define SAMCOP_IRQ_UERR1		9
#define SAMCOP_IRQ_DMA0			10
#define SAMCOP_IRQ_DMA1			11
#define SAMCOP_IRQ_SD			12
#define SAMCOP_IRQ_SD_WAKEUP		13
#define SAMCOP_IRQ_WAKEUP1		14
#define SAMCOP_IRQ_URXD0		15
#define SAMCOP_IRQ_URXD1		16
#define SAMCOP_IRQ_USBH			17
#define SAMCOP_IRQ_ACCEL		18
#define SAMCOP_IRQ_WAKEUP2		19
#define SAMCOP_IRQ_UTXD0		20
#define SAMCOP_IRQ_UTXD1		21
#define SAMCOP_IRQ_ADCTS		22
#define SAMCOP_IRQ_ADC			23
#define SAMCOP_IRQ_WAKEUP3		24
#define SAMCOP_IRQ_GPIO			25	/* demux -> hwirq 26-39 */
#define SAMCOP_IC_NR_IRQS		26

/* GPIO-block fan-out: a sparse subset of GPA/GPB pins, not a linear
 * gpio-offset identity.
 */
#define SAMCOP_GPIO_IRQ_BASE		SAMCOP_IC_NR_IRQS		/* 26 */
#define SAMCOP_IRQ_GPIO_GPA4		(SAMCOP_GPIO_IRQ_BASE + 0)	/* 26 */
#define SAMCOP_IRQ_GPIO_GPA5		(SAMCOP_GPIO_IRQ_BASE + 1)
#define SAMCOP_IRQ_GPIO_GPA6		(SAMCOP_GPIO_IRQ_BASE + 2)
#define SAMCOP_IRQ_GPIO_GPA7		(SAMCOP_GPIO_IRQ_BASE + 3)
#define SAMCOP_IRQ_GPIO_GPA8		(SAMCOP_GPIO_IRQ_BASE + 4)
#define SAMCOP_IRQ_GPIO_GPA9		(SAMCOP_GPIO_IRQ_BASE + 5)
#define SAMCOP_IRQ_GPIO_GPA10		(SAMCOP_GPIO_IRQ_BASE + 6)
#define SAMCOP_IRQ_GPIO_GPA11		(SAMCOP_GPIO_IRQ_BASE + 7)
#define SAMCOP_IRQ_GPIO_GPA18		(SAMCOP_GPIO_IRQ_BASE + 8)
#define SAMCOP_IRQ_GPIO_GPB12		(SAMCOP_GPIO_IRQ_BASE + 9)
#define SAMCOP_IRQ_GPIO_GPB13		(SAMCOP_GPIO_IRQ_BASE + 10)
#define SAMCOP_IRQ_GPIO_GPB14		(SAMCOP_GPIO_IRQ_BASE + 11)
#define SAMCOP_IRQ_GPIO_GPB15		(SAMCOP_GPIO_IRQ_BASE + 12)
#define SAMCOP_IRQ_GPIO_GPB16		(SAMCOP_GPIO_IRQ_BASE + 13)	/* 39 */
#define SAMCOP_GPIO_NR_IRQS		14

/* EPS (PCMCIA/expansion-pack status) fan-out. */
#define SAMCOP_EPS_IRQ_BASE		(SAMCOP_GPIO_IRQ_BASE + SAMCOP_GPIO_NR_IRQS) /* 40 */
#define SAMCOP_IRQ_EPS_CD0		(SAMCOP_EPS_IRQ_BASE + 0)	/* 40 */
#define SAMCOP_IRQ_EPS_CD1		(SAMCOP_EPS_IRQ_BASE + 1)
#define SAMCOP_IRQ_EPS_IRQ0		(SAMCOP_EPS_IRQ_BASE + 2)
#define SAMCOP_IRQ_EPS_IRQ1		(SAMCOP_EPS_IRQ_BASE + 3)
/* CD0_N|CD1_N combined below */
#define SAMCOP_IRQ_EPS_ODET		(SAMCOP_EPS_IRQ_BASE + 4)
#define SAMCOP_IRQ_EPS_BATT_FAULT	(SAMCOP_EPS_IRQ_BASE + 5)	/* 45 */
#define SAMCOP_EPS_NR_IRQS		6

#define SAMCOP_NR_IRQS			(SAMCOP_IC_NR_IRQS + SAMCOP_GPIO_NR_IRQS + \
					 SAMCOP_EPS_NR_IRQS)			  /* 46 */

/* ---------------------------------------------------------------------
 * Clock and Power Management (CPM)
 * ---------------------------------------------------------------------
 */
#define SAMCOP_CPM_LOCKTIME		(SAMCOP_CPM_BASE + 0x0000) /* reset default 0x0fff */
#define SAMCOP_CPM_PLLCONTROL		(SAMCOP_CPM_BASE + 0x0004) /* reset default 0x00028080 */
#define SAMCOP_CPM_CLOCKCONTROL		(SAMCOP_CPM_BASE + 0x0008) /* reset default 0x0ffe */
#define SAMCOP_CPM_CLOCKSLEEP		(SAMCOP_CPM_BASE + 0x000c)

/* ClockControl gate bits. The bit assignment differs from HAMCOP_CLK_*
 * (HAMCOP_CLK_GPIO is bit 9, SAMCOP's is bit 3).
 */
#define SAMCOP_CLK_UCLK_EN		BIT(0)
#define SAMCOP_CLK_USBHOST		BIT(1)
#define SAMCOP_CLK_DMAC			BIT(2)
#define SAMCOP_CLK_GPIO			BIT(3)	/* needed by our gpio_chip */
#define SAMCOP_CLK_FCD			BIT(4)	/* fingerprint, h54xx/h55xx only */
#define SAMCOP_CLK_SD			BIT(5)
#define SAMCOP_CLK_ADC			BIT(6)
#define SAMCOP_CLK_UART2		BIT(7)
#define SAMCOP_CLK_UART1		BIT(8)
#define SAMCOP_CLK_LED			BIT(9)
#define SAMCOP_CLK_MISC			BIT(10)
#define SAMCOP_CLK_1WIRE		BIT(11)

/*
 * ClockSleep bits. There is no XTCON bit; HAMCOP_CLKSLEEP_XTCON (bit 7) has no
 * SAMCOP counterpart.
 */
#define SAMCOP_CLKSLEEP_UCLK_ON	BIT(2)
/*
 * HALF_CLK's name is the opposite of its effect: 1 = the full PXA memory
 * clock, 0 = memory clock / 2. The 2.4/2.6 driver computes the rate that way,
 * and the identically shaped HAMCOP_CLKSLEEP_CLKSEL has the same polarity and
 * is confirmed on real h2200 hardware. samcop_gclk_recalc_rate() follows the
 * behaviour, not the name. The h5000 programs UCLK_ON alone, so HALF_CLK is
 * clear and gclk runs at memory / 2.
 */
#define SAMCOP_CLKSLEEP_HALF_CLK	BIT(1)
#define SAMCOP_CLKSLEEP_SLEEP		BIT(0)

/* ---------------------------------------------------------------------
 * ADC + resistive touchscreen front-end -- cell resource window only. Five
 * registers (Control, TouchScreenControl, Delay, Data0, Data1), the last at
 * +0x10.
 * ---------------------------------------------------------------------
 */
#define SAMCOP_ADC_SIZE			0x0014

/* ---------------------------------------------------------------------
 * OWM (DS1WM-compatible 1-Wire master) -- cell resource window only.
 * samcop-owm.h's own byte registers run Command(0x00)..ClockDivisor(0x10);
 * with ds1wm.c's bus_shift=2 (32-bit-strided MMIO, matching how every
 * other SAMCOP register in this MFD is accessed) the real span is
 * 5 registers * 4 bytes = 0x14.
 * ---------------------------------------------------------------------
 */
#define SAMCOP_OWM_SIZE			0x0014

/* ---------------------------------------------------------------------
 * SDI (SD/MMC host) -- cell resource window only. The highest register is
 * SDIIMSK at 0x40.
 * ---------------------------------------------------------------------
 */
#define SAMCOP_SDI_SIZE			0x0044

/* ---------------------------------------------------------------------
 * LED controller -- 5 channels (LEDCON0-4) and one shared prescaler (LEDPS),
 * cell resource window only.
 * ---------------------------------------------------------------------
 */
#define SAMCOP_LED_SIZE			0x0018
#define SAMCOP_LED_NR_CHANNELS		5

/* ---------------------------------------------------------------------
 * GPIO -- five banks (A-E). GPA is 20 lines wide and needs two 2-bit-per-pin
 * CON registers (CON1 for pins 0-15, CON2 for pins 16-19); GPB-GPE are
 * single-CON-register banks, capped at the 16 pins one such register can
 * address. The board functions never exceed that: GPB0-12, GPC0-1, GPD's
 * eight FCD_DATA lines (bits 1-15) and GPE0-2. The vendor header's WAN1..WAN5
 * names number a modem variant's pins 12-16, one past GPB's capacity; no
 * supported board uses it.
 * ---------------------------------------------------------------------
 */
#define SAMCOP_GPIO_GPA_CON1		(SAMCOP_GPIO_BASE + 0x0000) /* reset 0xaaaaaaaa (EXTIN) */
#define SAMCOP_GPIO_GPA_CON2		(SAMCOP_GPIO_BASE + 0x0004) /* reset 0x0000002a */
#define SAMCOP_GPIO_GPA_DAT		(SAMCOP_GPIO_BASE + 0x0008)
#define SAMCOP_GPIO_GPA_PUP		(SAMCOP_GPIO_BASE + 0x000c) /* reset 0x00080000 */
#define SAMCOP_GPIO_GPB_CON		(SAMCOP_GPIO_BASE + 0x0010)
#define SAMCOP_GPIO_GPB_DAT		(SAMCOP_GPIO_BASE + 0x0014)
#define SAMCOP_GPIO_GPB_PUP		(SAMCOP_GPIO_BASE + 0x0018)
#define SAMCOP_GPIO_GPC_CON		(SAMCOP_GPIO_BASE + 0x001c)
#define SAMCOP_GPIO_GPC_DAT		(SAMCOP_GPIO_BASE + 0x0020)
#define SAMCOP_GPIO_GPC_PUP		(SAMCOP_GPIO_BASE + 0x0024)
#define SAMCOP_GPIO_GPD_CON		(SAMCOP_GPIO_BASE + 0x0028) /* reset 0xaaaa */
#define SAMCOP_GPIO_GPD_DAT		(SAMCOP_GPIO_BASE + 0x002c)
#define SAMCOP_GPIO_GPD_PUP		(SAMCOP_GPIO_BASE + 0x0030)
#define SAMCOP_GPIO_GPE_CON		(SAMCOP_GPIO_BASE + 0x0034) /* reset 0x00000007, no PUP */
#define SAMCOP_GPIO_GPE_DAT		(SAMCOP_GPIO_BASE + 0x0038)

#define SAMCOP_GPIO_CK8MCON		(SAMCOP_GPIO_BASE + 0x003c)

/*
 * SPCR ("sleep pull-up control") gates SAMCOP's internal pull-ups on the pin
 * groups its sub-blocks drive. The bits disable the pull-up: 0 = pull-up
 * present, 1 = pull-up removed. SDPUCR covers the SD bus; the SD host clears it
 * when the slot is powered and sets it when the slot is powered down, so an
 * empty slot does not burn current through the pull-ups.
 *
 * The WinCE OAL writes SPCR = 0x000000ff through both its uncached and its
 * physical mapping, which leaves SDPUCR (bit 10) clear and shows that bits 0
 * and 1 are writable. That is the OAL's policy for every other group and not
 * something this driver reproduces.
 *
 * PU_SDDA and PU_SDCM are real bits but it is not known which pin group each
 * gates; the names are HAMCOP's SPUCR bit names, and nothing writes them.
 *
 * Bits 8 and 9 control the USB pins' pull-ups: the WinCE OHCI driver clears
 * them at init and resume and sets them on suspend. The register, offset and
 * bits are read from the disassembly; that they are the USB pull-ups is
 * inferred from the call sites. The OHCI glue does not write them.
 */
#define SAMCOP_GPIO_SPCR		(SAMCOP_GPIO_BASE + 0x0040)
#define SAMCOP_GPIO_SPCR_PU_SDDA	BIT(0)	/* unverified, see comment */
#define SAMCOP_GPIO_SPCR_PU_SDCM	BIT(1)	/* unverified, see comment */
#define SAMCOP_GPIO_SPCR_PU_USB0	BIT(8)	/* WinCE ohci.dll init clears, suspend sets */
#define SAMCOP_GPIO_SPCR_PU_USB1	BIT(9)	/* WinCE ohci.dll init clears, suspend sets */
#define SAMCOP_GPIO_SPCR_SDPUCR	BIT(10)

/*
 * GPIO fan-out trigger-type and filter config: 3 registers of 8 nibbles (4
 * bits each) = 24 slots for the 14 GPIO fan-out sources above. The fan-out
 * tier packs sequentially (source index N -> register N/8, nibble N%8); the
 * layout is inferred from the macro shapes and has not been confirmed on
 * hardware.
 *
 * The IC-tier allocation sharing these registers is indexed by GPA pin number
 * instead. The WinCE OAL arms GPA16 (SD detect) muxed to EXTIN, then
 * GPIO_INT3 |= 0xf (nibble 0 of register index 2) and FLTCONFIG7 |= 0xf0000
 * (nibble 4), and register index * 8 + nibble = 2 * 8 + 0 = 16 is the GPA pin
 * number.
 */
#define SAMCOP_GPIO_INT1		(SAMCOP_GPIO_BASE + 0x0044)
#define SAMCOP_GPIO_INT2		(SAMCOP_GPIO_BASE + 0x0048)
#define SAMCOP_GPIO_INT3		(SAMCOP_GPIO_BASE + 0x004c)
#define SAMCOP_GPIO_INT(n)		(SAMCOP_GPIO_INT1 + 4 * (n))

#define SAMCOP_GPIO_FLTCONFIG1		(SAMCOP_GPIO_BASE + 0x0050)
#define SAMCOP_GPIO_FLTCONFIG(n)	(SAMCOP_GPIO_FLTCONFIG1 + 4 * (n)) /* n = 0..6 */
#define SAMCOP_GPIO_MON			(SAMCOP_GPIO_BASE + 0x006c)
#define SAMCOP_GPIO_ENINT1		(SAMCOP_GPIO_BASE + 0x0070) /* IC-tier GPA-EXTIN enables */
#define SAMCOP_GPIO_ENINT2		(SAMCOP_GPIO_BASE + 0x0074) /* GPIO-fanout enables, bit N */
#define SAMCOP_GPIO_INTPND		(SAMCOP_GPIO_BASE + 0x0078) /* GPIO-fanout pending, w1c */

#define SAMCOP_GPIO_INTLV_MASK(n)	(0x7 << (4 * (n)))
#define SAMCOP_GPIO_INTLV_LOW(n)	(0x0 << (4 * (n)))
#define SAMCOP_GPIO_INTLV_HIGH(n)	(0x1 << (4 * (n)))
#define SAMCOP_GPIO_INTLV_RISING(n)	(0x2 << (4 * (n)))
#define SAMCOP_GPIO_INTLV_FALLING(n)	(0x4 << (4 * (n)))
#define SAMCOP_GPIO_INTLV_BOTH(n)	(0x6 << (4 * (n)))
#define SAMCOP_GPIO_INTF_MASK(n)	(0x1 << (4 * (n) + 3))	/* glitch filter enable */

#define SAMCOP_GPIO_CON_MODE_INPUT	0x0
#define SAMCOP_GPIO_CON_MODE_OUTPUT	0x1
#define SAMCOP_GPIO_CON_MODE_EXTIN	0x2	/* external interrupt input, GPA/GPB only */
#define SAMCOP_GPIO_CON_MASK(pin)	(0x3 << (2 * (pin)))
#define SAMCOP_GPIO_CON_OUTPUT(pin)	(SAMCOP_GPIO_CON_MODE_OUTPUT << (2 * (pin)))

/*
 * gpio_chip addressing: 32-line banks with holes, ngpio = 160, rather than a
 * packed 47-line map. SAMCOP_GPIO_PINS_PER_BANK (16) is the real
 * CON-register-addressable width for the single-register banks B-E; GPA's
 * extra CON2 widens it to 20 as a special case in the direction callbacks,
 * not by changing the bank stride.
 */
#define SAMCOP_GPIO_BANK_STRIDE		32
#define SAMCOP_GPIO_NR_BANKS		5
#define SAMCOP_NR_GPIO			(SAMCOP_GPIO_NR_BANKS * SAMCOP_GPIO_BANK_STRIDE) /* 160 */
#define SAMCOP_GPIO_PINS_PER_BANK	16	/* real width, B-E; GPA is 20, see above */
#define SAMCOP_GPIO_GPA_NR_PINS		20

enum {
	SAMCOP_GPIO_BANK_A,
	SAMCOP_GPIO_BANK_B,
	SAMCOP_GPIO_BANK_C,
	SAMCOP_GPIO_BANK_D,
	SAMCOP_GPIO_BANK_E,
};

#define SAMCOP_GPIO(bank, pin)		\
	((SAMCOP_GPIO_BANK_##bank) * SAMCOP_GPIO_BANK_STRIDE + (pin))

/*
 * SD/MMC card detect is GPA16 (the vendor headers call it SD_WAKEUP and
 * SD_DETECT_N), active low: pin low = card present.
 *
 * It is not one of the GPIO fan-out sources: the sparse
 * SAMCOP_IRQ_GPIO_GPA4..GPB16 table above has no GPA16 entry. It is one of
 * the nine EXTIN-muxed GPA pins backing an IC-tier source directly, hwirq 13
 * (SAMCOP_IRQ_SD_WAKEUP). A consumer wanting card detect therefore needs both
 * this offset as a plain readable gpiod for the level, and
 * SAMCOP_IRQ_SD_WAKEUP for the edge (gpiod_to_irq() on this offset returns an
 * error by design). The SD host does exactly that through
 * mmc_gpiod_request_cd() and mmc_gpio_set_cd_irq().
 *
 * The WinCE OAL arms this pin with GPA16 muxed to EXTIN, GPIO_INT3 nibble 0,
 * FLTCONFIG7 nibble 4 and GPIO_ENINT1 bit 8. ENINT1 indexes the nine
 * EXTIN-backed IC sources by list position, so bit 8 is the ninth entry,
 * SD_WAKEUP; the 2.4/2.6 driver's table for this source says the same
 * (GPIO_INT3 nibble 0, ENINT1 bit 8) and its card-swap events were detected on
 * a real h5400. The board's static ENINT1 value (0x3f) does not set bit 8:
 * SD_WAKEUP is armed at runtime by samcop_ic_irq_set_type() when the SD host
 * requests its card-detect interrupt.
 */
#define SAMCOP_GPIO_GPA_SD_DETECT	SAMCOP_GPIO(A, 16)

/*
 * Sleeve and expansion-pack power control lines on GPA, used by the sleeve
 * driver for OPT_ON and OPT_RESET.
 *
 * OPT_ON is not driven. The factory firmware treats GPA bit 17 as active low
 * in its writes (the h5550 pcmcia.dll socket power-up clears it on the
 * power-on branch only, and the nk.exe sleeve control clears it for true), but
 * every GPA_CON2 store in the firmware writes 0x5a, which leaves GPA17 in mode
 * 2, an interrupt input, so those writes never reach the pin. Measured on an
 * h5550 with a CF Plus jacket and a CF card, with the pin made an output by a
 * gpiod request, low or high, every read of the socket-0 attribute and
 * common-memory windows returned 0xff and no card enumerated. With GPA_CON2 =
 * 0x58 or 0x5a (GPA17 left in mode 2) the attribute window read a CIS
 * (01 04 df) and the card bound. The h5000 devicetree therefore has no
 * opt-on-gpios, and the sleeve driver treats the line as optional.
 *
 * OPT_RESET is active high and an output in both the firmware and this
 * driver.
 */
#define SAMCOP_GPIO_GPA_OPT_ON		SAMCOP_GPIO(A, 17)
#define SAMCOP_GPIO_GPA_OPT_RESET	SAMCOP_GPIO(A, 18)

/* ---------------------------------------------------------------------
 * PCMCIA / expansion pack interface (EPS) -- status and interrupt glue only;
 * the PCMCIA socket controller itself is not a cell of this driver. The core
 * needs it for the hwirq 40-45 demux.
 * ---------------------------------------------------------------------
 */
#define SAMCOP_PCMCIA_TDW		(SAMCOP_PCMCIA_BASE + 0x0000) /* data width, default 0x01 */
#define SAMCOP_PCMCIA_EPS		(SAMCOP_PCMCIA_BASE + 0x0004) /* raw status, default 0x00 */
#define SAMCOP_PCMCIA_CC		(SAMCOP_PCMCIA_BASE + 0x0008) /* control, default 0 */
#define SAMCOP_PCMCIA_CC_RESET		BIT(0)
#define SAMCOP_PCMCIA_IC		(SAMCOP_PCMCIA_BASE + 0x000c) /* IRQ enable, default 0 */
/* edge config, default 0x25a5 */
#define SAMCOP_PCMCIA_IM		(SAMCOP_PCMCIA_BASE + 0x0010)
#define SAMCOP_PCMCIA_IP		(SAMCOP_PCMCIA_BASE + 0x0014) /* IRQ pending, w1c */

/*
 * USB host DMA master address masking, reset default 0x1: while set, the chip
 * confines the OHCI DMA master to the on-chip SRAM window, in addition to the
 * software-side allocator bound. Left at the reset default.
 */
#define SAMCOP_PCMCIA_MISC		(SAMCOP_PCMCIA_BASE + 0x0018)
#define SAMCOP_PCMCIA_MISC_USB_MASK	BIT(0)

/*
 * Cell resource window: TDW, EPS, CC, IC, IM, IP and MISC are seven 32-bit
 * registers at 0x00..0x18, so the span is 0x18 + 4.
 */
#define SAMCOP_PCMCIA_SIZE		0x001c

#define SAMCOP_PCMCIA_EPS_CD0_N		BIT(0)
#define SAMCOP_PCMCIA_EPS_CD1_N		BIT(1)
#define SAMCOP_PCMCIA_EPS_IRQ0_N	BIT(2)
#define SAMCOP_PCMCIA_EPS_IRQ1_N	BIT(3)
#define SAMCOP_PCMCIA_EPS_ODET0_N	BIT(4)
#define SAMCOP_PCMCIA_EPS_ODET1_N	BIT(5)
#define SAMCOP_PCMCIA_EPS_BATT_FLT	BIT(6)

/*
 * IM selects the edge for each EPS source with a 2-bit field at twice the
 * source number: 0 = none, 1 = falling, 2 = rising, 3 = both. Field N is
 * source N in SAMCOP_PCMCIA_EPS_* order: 0 = CD0, 1 = CD1, 2 = IRQ0, 3 = IRQ1,
 * 4 = ODET0, 5 = ODET1, 6 = BATT_FLT. The silicon reset value is 0x25a5.
 *
 * SAMCOP_PCMCIA_IM_DEFAULT, programmed at probe, differs from the reset value
 * in four places, all matching what the vendor firmware programs and a live
 * IM read of 0x3f55 on a running WinCE system:
 *
 *  - IRQ0_N and IRQ1_N (fields 2 and 3) are falling edge. They are active-low
 *    card interrupt and ready lines, so a card asking for attention is a
 *    high-to-low transition; the reset value's rising edge only fires when
 *    the card releases the line.
 *  - ODET0 and ODET1 (fields 4 and 5) are both edges. With falling edge only,
 *    a sleeve insert interrupted but an eject never did, which left a removed
 *    jacket enumerated.
 *  - CD0 and CD1 (fields 0 and 1) stay falling. The sleeve driver re-arms
 *    them to the opposite of the last observed level on every poll, so this
 *    only supplies the first edge.
 *
 *  - BATT_FLT (field 6) is both edges, because the sleeve driver requests its
 *    interrupt for the bus core, which acts on a fault being asserted and on
 *    its clearing.
 */
#define SAMCOP_PCMCIA_IM_DEFAULT	0x3f55

/* Public API for drivers of the chip's sub-devices */
struct device;
int samcop_hwirq_to_irq(struct device *samcop_dev, unsigned int hwirq);

/*
 * Mux GPD0-7 and GPE0-2 to the FSI/FCD hardware sequencer's alternate
 * function. Exported for the fingerprint driver; these eleven lines are
 * pin-mux, never gpiod, consumers.
 */
int samcop_reset_fcd(struct device *samcop_dev);

/*
 * Set/clear bits in the shared SPCR pull-up control register. SPCR lives in
 * the MFD core's register window, not in any sub-cell's, so a cell cannot map
 * it itself. Exported for drivers/mmc/host/samcop-sdi.c.
 */
void samcop_set_spcr(struct device *samcop_dev, u32 mask, u32 bits);

/*
 * Set/clear bits in GPA_PUP, the GPIO-A bank's per-pin pull-up register, a
 * different register from SPCR (which gates whole sub-block pull-up groups).
 * Same bit sense as SPCR: 0 = pull-up present, 1 = pull-up removed. Bit 16
 * (SD card detect) is clear at reset, so its pull-up is on by default, and the
 * SD host clears and sets it along with slot power. The GPIO-A gpio_chip has no
 * pinconf bias callback, hence a direct exported register operation.
 */
void samcop_set_gpio_a_pullup(struct device *samcop_dev, u32 mask, u32 bits);

#endif /* __LINUX_MFD_SAMCOP_H */
