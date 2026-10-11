/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Register map and shared constants for the Compaq/HTC ASIC2 companion
 * ASIC (HP iPAQ h38xx/h39xx family), see drivers/mfd/asic2.c.
 *
 * The offsets and bits follow the Compaq register header (ipaq-asic2.h,
 * Andrew Christian, 2001) of the out-of-tree handhelds.org kernels.
 *
 * Register access is 32 bit (readl/writel): every register is spaced a
 * flat 4 bytes apart regardless of its documented field width (an 8-bit
 * SPI Control register at 0x0000 is followed by Data at 0x0004), the
 * classic word-granular VLIO layout.
 *
 * Copyright 2001 Compaq Computer Corporation.
 * Copyright (C) 2026 Koen Kooi
 */
#ifndef __LINUX_MFD_ASIC2_H
#define __LINUX_MFD_ASIC2_H

#include <linux/bits.h>
#include <linux/kconfig.h>
#include <linux/types.h>

/*
 * Two register blocks, KPIO and GPIO2, share an identical internal layout
 * (Direction/InterruptType/InterruptEdgeType/InterruptLevelType/
 * InterruptClear-Flag/Data/BattFaultOut/InterruptEnable/Alternate at the
 * same relative offsets), so drivers/mfd/asic2.c takes a block base and
 * reuses one set of relative offsets for both.
 */
#define ASIC2_GPIO_BASE			0x0000
#define ASIC2_KPIO_BASE			0x0200

#define ASIC2_REG_DIRECTION		0x0000	/* R/W: 1=input, 0=output */
#define ASIC2_REG_INT_TYPE		0x0004	/* R/W: 1=edge, 0=level */
#define ASIC2_REG_INT_EDGE		0x0008	/* R/W: 1=rising, 0=falling */
#define ASIC2_REG_INT_LEVEL		0x000c	/* R/W: 1=high, 0=low */
#define ASIC2_REG_INT_CLEAR		0x0010	/* W: ack */
#define ASIC2_REG_INT_FLAG		0x0010	/* R: pending status */
#define ASIC2_REG_DATA			0x0014	/* R/W */
#define ASIC2_REG_BATT_FAULT_OUT	0x0018	/* R/W */
#define ASIC2_REG_INT_ENABLE		0x001c	/* R/W: 1=enabled. KPIO's must
						 * not be read back, see asic2.c.
						 */
#define ASIC2_REG_ALTERNATE		0x003c	/* R/W, unused */

/* ---------------------------------------------------------------------
 * GPIO2: 16 lines, all exposed as a gpio_chip (drivers/mfd/asic2.c). Only
 * five are demuxed as interrupt sources (ASIC2_IRQ_* below), although
 * more are documented as interrupt-capable.
 * ---------------------------------------------------------------------
 */
#define ASIC2_GPIO2_IN_Y1_N		BIT(0)	/* out: touchscreen Y1 */
#define ASIC2_GPIO2_IN_X0		BIT(1)	/* out: touchscreen X0 */
#define ASIC2_GPIO2_IN_Y0		BIT(2)	/* out: touchscreen Y0 */
#define ASIC2_GPIO2_IN_X1_N		BIT(3)	/* out: touchscreen X1 */
#define ASIC2_GPIO2_BT_RST		BIT(4)	/* out: Bluetooth reset */
#define ASIC2_GPIO2_PEN_IRQ		BIT(5)	/* in: pen down */
#define ASIC2_GPIO2_SD_DETECT		BIT(6)	/* in: SD card detect */
#define ASIC2_GPIO2_EAR_IN_N		BIT(7)	/* in: audio jack inserted */
#define ASIC2_GPIO2_OPT_PCM_RESET	BIT(8)	/* out: sleeve card reset */
#define ASIC2_GPIO2_OPT_RESET		BIT(9)	/* out: option-pack reset */
#define ASIC2_GPIO2_USB_DETECT_N	BIT(10)	/* in */
#define ASIC2_GPIO2_SD_CON_SLT		BIT(11)	/* in */
#define ASIC2_GPIO2_OPT_ON		BIT(12)	/* out: option jacket power */
#define ASIC2_GPIO2_OPT_ON_NVRAM	BIT(13)	/* out: mux'd with PWM1 */
#define ASIC2_GPIO2_UART0_RXD_ENABLE	BIT(14)	/* out */
#define ASIC2_GPIO2_UART0_TXD_ENABLE	BIT(15)	/* out */

/* ---------------------------------------------------------------------
 * KPIO: 12 keypad lines (bits 0-11) plus 8 special peripheral-IRQ bits
 * in the same register (bits 16-23). There is no gpio_chip; bits 0-11 are
 * demuxed as one grouped hwirq (ASIC2_IRQ_KEY).
 * ---------------------------------------------------------------------
 */
#define ASIC2_KPIO_KEY_ALL		GENMASK(11, 0)
#define ASIC2_KPIO_SPI_INT		BIT(16)
#define ASIC2_KPIO_OWM_INT		BIT(17)
#define ASIC2_KPIO_ADC_INT		BIT(18)
#define ASIC2_KPIO_UART_0_INT		BIT(19)
#define ASIC2_KPIO_UART_1_INT		BIT(20)
#define ASIC2_KPIO_TIMER_0_INT		BIT(21)
#define ASIC2_KPIO_TIMER_1_INT		BIT(22)
#define ASIC2_KPIO_TIMER_2_INT		BIT(23)

/*
 * hwirq numbers in the linear irq_domain. 0-8 live in KPIO (KEY is a
 * 12-bit group, the other eight are single peripheral status bits), 9-13
 * live in GPIO2 (single bits, identity-mapped with the gpio_chip offsets
 * by asic2_gpio_to_irq()). 14-15 are unused.
 */
#define ASIC2_IRQ_KEY			0
#define ASIC2_IRQ_SPI			1
#define ASIC2_IRQ_OWM			2
#define ASIC2_IRQ_ADC			3
#define ASIC2_IRQ_UART_0		4
#define ASIC2_IRQ_UART_1		5
#define ASIC2_IRQ_TIMER_0		6
#define ASIC2_IRQ_TIMER_1		7
#define ASIC2_IRQ_TIMER_2		8
#define ASIC2_IRQ_GPIO_START		9
#define ASIC2_IRQ_PEN			9
#define ASIC2_IRQ_SD_DETECT		10
#define ASIC2_IRQ_EAR_IN		11
#define ASIC2_IRQ_USB_DETECT		12
#define ASIC2_IRQ_SD_CON_SLT		13
#define ASIC2_NR_IRQS			16

/* ---------------------------------------------------------------------
 * SPI master, driven by drivers/spi/spi-asic2.c. One byte per transfer:
 * write DATA, set CONTROL.SPE, wait for the block to clear SPE, read DATA;
 * SPIE enables the block's KPIO completion interrupt. Two chip selects
 * (sleeve ID EEPROM on CS0, jacket microcontroller on CS1).
 * ---------------------------------------------------------------------
 */
#define ASIC2_SPI_BASE			0x0400
#define ASIC2_SPI_CONTROL		0x0000	/* R/W, 8 significant bits */
#define ASIC2_SPI_DATA			0x0004	/* R/W, 8 significant bits */
#define ASIC2_SPI_CHIP_SELECT_DISABLED	0x0008	/* W, 8 significant bits */

#define ASIC2_SPI_CONTROL_SPR(clk)	((clk) & 0x0f)
#define ASIC2_SPI_CONTROL_SPE		BIT(4)	/* enable */
#define ASIC2_SPI_CONTROL_SPIE		BIT(5)	/* interrupt enable */
#define ASIC2_SPI_CONTROL_SEL_CS1	BIT(6)	/* 0=CS0, 1=CS1 */
#define ASIC2_SPI_CONTROL_CPOL		BIT(7)

/* ---------------------------------------------------------------------
 * PWM: one clock-enable bit (ASIC2_CLOCK_PWM) is shared by both PWM0 and
 * PWM1, unlike ASIC3. PWM0 drives the frontlight; PWM1's pin is
 * multiplexed with GPIO13 (option-jacket power).
 * ---------------------------------------------------------------------
 */
#define ASIC2_PWM_0_BASE		0x0600
#define ASIC2_PWM_1_BASE		0x0700
#define ASIC2_PWM_TIME_BASE		0x0000	/* R/W 6 bits */
#define ASIC2_PWM_PERIOD_TIME		0x0004	/* R/W 12 bits */
#define ASIC2_PWM_DUTY_TIME		0x0008	/* R/W 12 bits */

/* ---------------------------------------------------------------------
 * LED: three blink-timer channels with a 0x80 stride, driving the three
 * physical status LEDs. The block has no clock-enable bit of its own, it
 * is fed by EX0 (32.768 kHz), which is forced on at probe.
 * ---------------------------------------------------------------------
 */
#define ASIC2_NUM_LEDS			3

#define ASIC2_LED0_BASE			0x0800
#define ASIC2_LED1_BASE			0x0880
#define ASIC2_LED2_BASE			0x0900
#define ASIC2_LED_TIMEBASE		0x0000	/* R/W 7 bits */
#define ASIC2_LED_PERIODTIME		0x0004	/* R/W 12 bits */
#define ASIC2_LED_DUTYTIME		0x0008	/* R/W 12 bits */
#define ASIC2_LED_AUTOSTOPCOUNT		0x000c	/* R/W 16 bits, not changeable */

/* ---------------------------------------------------------------------
 * Clock enable: one flat 18-bit register, one CCF gate per bit exposed by
 * drivers/mfd/asic2.c. EX0 feeds the chip's interrupt controller and is
 * forced on at probe instead of being a consumer-facing gate. Bits without
 * a consumer (AUDIO_1-4, SLOW_UART_0/1) are defined for documentation only.
 *
 * SD_MASK is a 3-bit selector field, not four independent gates, and the
 * SD/MMC host card clock of the h39xx is sourced through it. The h3900
 * selects SD_2 (24.576 MHz), the same rate as the hclk of the ASIC3 SD
 * host; the SA-1100 h3800 selects SD_3 instead, which is board policy and
 * why the selector is a field rather than a fixed rate.
 * ---------------------------------------------------------------------
 */
#define ASIC2_CLOCK_BASE		0x1000
#define ASIC2_CLOCK_ENABLE		0x0000

#define ASIC2_CLOCK_AUDIO_MASK		GENMASK(3, 0)
#define ASIC2_CLOCK_ADC			BIT(4)	/* CX4 -- 1.024 MHz to the ADC */
#define ASIC2_CLOCK_SPI			BIT(5)	/* CX5 -- 4.096 MHz to SPI */
#define ASIC2_CLOCK_OWM			BIT(6)	/* CX6 -- 4.096 MHz to the 1-Wire master */
#define ASIC2_CLOCK_PWM			BIT(7)	/* CX7 -- 2.048 MHz, both PWM blocks */
#define ASIC2_CLOCK_UART_1		BIT(8)	/* CX8 -- 24.576 MHz to UART1 */
#define ASIC2_CLOCK_UART_0		BIT(9)	/* CX9 -- 24.576 MHz to UART0 */
#define ASIC2_CLOCK_SD_MASK		GENMASK(12, 10)
#define ASIC2_CLOCK_SD_1		(1 << 10)	/* 16.934 MHz to SD */
#define ASIC2_CLOCK_SD_2		(2 << 10)	/* 24.576 MHz to SD, h39xx */
#define ASIC2_CLOCK_SD_3		(3 << 10)	/* 33.869 MHz to SD, h38xx */
#define ASIC2_CLOCK_SD_4		(4 << 10)	/* 49.152 MHz to SD */
#define ASIC2_CLOCK_EX0			BIT(13)	/* 32.768 kHz, forced on, not a gate */
#define ASIC2_CLOCK_EX1			BIT(14)	/* 24.576 MHz: ADC/SPI/OWM/PWM/UART/SD */
#define ASIC2_CLOCK_EX2			BIT(15)	/* 33.869 MHz, unused */
#define ASIC2_CLOCK_SLOW_UART_0		BIT(16)
#define ASIC2_CLOCK_SLOW_UART_1		BIT(17)

/* ---------------------------------------------------------------------
 * ADC + resistive touchscreen front-end: an interrupt-driven sample state
 * machine (light sensor, Imin, Vs/Mbat and the TP_X0/TP_Y1 touchpanel
 * channels), with the touchscreen plates driven through the GPIO2 bits
 * ASIC2_GPIO2_IN_{Y1_N,X0,Y0,X1_N}.
 * ---------------------------------------------------------------------
 */
#define ASIC2_ADC_BASE			0x1200
#define ASIC2_ADC_MULTIPLEXER		0x0000	/* R/W 4 bits */
#define ASIC2_ADC_CONTROL_STATUS	0x0004	/* R/W 8 bits */
#define ASIC2_ADC_DATA			0x0008	/* R 10 bits */

#define ASIC2_ADMUX(x)			((x) & 0x07)
#define ASIC2_ADMUX_LIGHTSENSOR	ASIC2_ADMUX(0)
#define ASIC2_ADMUX_IMIN		ASIC2_ADMUX(1)
#define ASIC2_ADMUX_VS_MBAT		ASIC2_ADMUX(2)
#define ASIC2_ADMUX_TP_X0		ASIC2_ADMUX(3)
#define ASIC2_ADMUX_TP_Y1		ASIC2_ADMUX(4)
#define ASIC2_ADMUX_CLKEN		BIT(3)

#define ASIC2_ADCSR_ADPS(x)		((x) & 0x0f)
#define ASIC2_ADCSR_FREE_RUN		BIT(4)
#define ASIC2_ADCSR_INT_ENABLE		BIT(5)
#define ASIC2_ADCSR_START		BIT(6)
#define ASIC2_ADCSR_ENABLE		BIT(7)

/*
 * Interrupt block (INTR). ASIC2_INTMASK_GLOBAL is the chip-wide gate for
 * the external interrupt line: with it clear nothing is dispatched, however
 * the per-source registers are programmed.
 */
#define ASIC2_INTR_BASE			0x1600
#define ASIC2_INTR_MASK_AND_FLAG	0x0000	/* R/(W) 8 bits */
#define ASIC2_INTR_CLOCK_PRESCALE	0x0004	/* R/(W) 5 bits */
#define ASIC2_INTR_TIMER_SET		0x0008	/* R/(W) 8 bits */

/*
 * GLOBAL is set = enabled. The per-source bits are not uniform: UART_0,
 * UART_1 and TIMER are "1: mask, 0: enable", OWM is "0: mask, 1: enable".
 */
#define ASIC2_INTMASK_GLOBAL		BIT(0)	/* must be set or nothing dispatches */
#define ASIC2_INTMASK_UART_0		BIT(4)	/* 1: mask, 0: enable */
#define ASIC2_INTMASK_UART_1		BIT(5)	/* 1: mask, 0: enable */
#define ASIC2_INTMASK_TIMER		BIT(6)	/* 1: mask, 0: enable */
#define ASIC2_INTMASK_OWM		BIT(7)	/* 0: mask, 1: enable */

/*
 * Time base of the INTR block: a 4-bit prescaler of the 32.768 kHz EX0
 * clock, and an enable bit that starts it.
 */
#define ASIC2_INTCPS_CPS(x)		((x) & 0x0f)	/* 4 bits, max 14 */
#define ASIC2_INTCPS_SET		BIT(4)		/* time base enable */

struct gpio_desc;

/*
 * Touch-plate and pen GPIO handles that drivers/mfd/asic2.c passes to
 * drivers/iio/adc/asic2-adc.c as the platform data of the "asic2-adc" cell.
 * They are requested by asic2.c against its own gpio_chip (GPIO2 offsets
 * 0-3 and 5, the ASIC2_GPIO2_IN_{Y1_N,X0,Y0,X1_N} and _PEN_IRQ bits), so
 * that both drivers serialize on the lock that guards every GPIO2 register
 * access instead of asic2-adc.c mapping the same data register a second
 * time.
 */
struct asic2_adc_touch_pdata {
	struct gpio_desc *gpio_y1_n;
	struct gpio_desc *gpio_x0;
	struct gpio_desc *gpio_y0;
	struct gpio_desc *gpio_x1_n;
	struct gpio_desc *gpio_pen;
};

/* ---------------------------------------------------------------------
 * 1-Wire: the register layout (4-byte stride, with Command/Data/Interrupt/
 * InterruptEnable/ClockDivisor at 0x00/0x04/0x08/0x0c/0x10) matches
 * drivers/w1/masters/ds1wm.c with bus_shift = 2, so the "ds1wm" platform
 * driver is used directly. The core registers a "ds1wm" clkdev alias for
 * the OWM clock gate.
 * ---------------------------------------------------------------------
 */
#define ASIC2_OWM_BASE			0x1800

/* ---------------------------------------------------------------------
 * ASIC1 (h38xx only). A physically separate chip with no chip select of
 * its own: the SoC issues plain bus cycles to 0x1c00-0x1eff of this same
 * window and ASIC2 forwards them. 0x1f00 itself is ASIC2's own
 * FlashWP_VPP_ON register, not ASIC1's. Its blocks are 16 bit, unlike
 * this chip's flat 32-bit stride, and their register offsets live with
 * their own drivers; only each block's position inside the window is
 * needed here, for the mfd cells that drivers/mfd/asic2.c adds when the
 * devicetree describes the chip. h39xx has no ASIC1 (ASIC3 replaced it)
 * and decodes nothing here.
 * ---------------------------------------------------------------------
 */
#define ASIC1_GPIO_BASE			0x1e60
#define ASIC1_GPIO_SIZE			0x40

/*
 * ASIC1's SD/MMC host: 4-byte strided registers of 16 bit data, from
 * StartStopClock at 0x00 to BufferPartFull at 0x50, rounded up to a
 * 4-byte aligned size. The register list lives with the driver.
 */
#define ASIC1_MMC_BASE			0x1c00
#define ASIC1_MMC_SIZE			0x54

/*
 * Flash Vpp/write-protect enable. It is ASIC2's own register even though
 * it sits right past the span forwarded to ASIC1, which is why that span
 * ends at 0x1eff. It takes a raw 0 or 1, not a bit mask. h3800 only: h39xx
 * gates the Vpp of its NOR flash from a PXA GPIO instead, so the register
 * is never written there.
 */
#define ASIC2_FLASH_WP_VPP_ON		0x1f00

/* Whole-chip register window */
#define ASIC2_MAP_SIZE			0x2000

struct device;

/*
 * Locked read-modify-write of one register in the ASIC2 window, for cell
 * drivers that share a register with the core (see drivers/mfd/asic2.c).
 */
#if IS_ENABLED(CONFIG_MFD_ASIC2)
void asic2_update_bits(struct device *asic2_dev, u32 reg, u32 mask, u32 bits);
#else
static inline void asic2_update_bits(struct device *asic2_dev, u32 reg,
				     u32 mask, u32 bits)
{
}
#endif

#endif /* __LINUX_MFD_ASIC2_H */
