/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Compaq/HTC ASIC3 companion ASIC register map. The same chip is used in
 * the HP iPAQ hx4700 and h39xx and in several HTC WinCE phones.
 *
 * The chip is described by a devicetree binding
 * (Documentation/devicetree/bindings/mfd/htc,asic3.yaml), so there is no
 * platform_data.
 */

#ifndef __LINUX_MFD_ASIC3_H
#define __LINUX_MFD_ASIC3_H

#include <linux/bits.h>
#include <linux/types.h>

#define ASIC3_NUM_GPIO_BANKS	4
#define ASIC3_GPIOS_PER_BANK	16
#define ASIC3_NUM_GPIOS		64
#define ASIC3_NR_IRQS		(ASIC3_NUM_GPIOS + 6)

/* Fixed-function IRQs, hwirq 64-69 in the domain drivers/mfd/asic3.c builds */
#define ASIC3_IRQ_LED0		64
#define ASIC3_IRQ_LED1		65
#define ASIC3_IRQ_LED2		66
#define ASIC3_IRQ_SPI		67
#define ASIC3_IRQ_SMBUS		68
#define ASIC3_IRQ_OWM		69

#define ASIC3_GPIO_BANK_A	0
#define ASIC3_GPIO_BANK_B	1
#define ASIC3_GPIO_BANK_C	2
#define ASIC3_GPIO_BANK_D	3

#define ASIC3_GPIO(bank, gpio) \
	((ASIC3_GPIOS_PER_BANK * ASIC3_GPIO_BANK_##bank) + (gpio))

/*
 * All offsets below are byte offsets in the chip's own 32-bit-addressed
 * register map; asic3_read_register() and asic3_write_register() shift by
 * the bus_shift computed from the size of the "core" reg resource (1 on a
 * 16-bit VLIO wiring) before the MMIO access.
 */
#define ASIC3_GPIO_A_BASE	0x0000
#define ASIC3_GPIO_B_BASE	0x0100
#define ASIC3_GPIO_C_BASE	0x0200
#define ASIC3_GPIO_D_BASE	0x0300
#define ASIC3_GPIO_BASE_INCR	(ASIC3_GPIO_B_BASE - ASIC3_GPIO_A_BASE)
#define ASIC3_BANK_TO_BASE(bank) \
	(ASIC3_GPIO_A_BASE + ((bank) * ASIC3_GPIO_BASE_INCR))

#define ASIC3_GPIO_MASK          0x00    /* R/W 0:don't mask */
#define ASIC3_GPIO_DIRECTION     0x04    /* R/W 0:input */
#define ASIC3_GPIO_OUT           0x08    /* R/W 0:output low */
#define ASIC3_GPIO_TRIGGER_TYPE  0x0c    /* R/W 0:level */
#define ASIC3_GPIO_EDGE_TRIGGER  0x10    /* R/W 0:falling */
#define ASIC3_GPIO_LEVEL_TRIGGER 0x14    /* R/W 0:low level detect */
#define ASIC3_GPIO_SLEEP_MASK    0x18    /* R/W 0:don't mask in sleep mode */
#define ASIC3_GPIO_SLEEP_OUT     0x1c    /* R/W level 0:low in sleep mode */
#define ASIC3_GPIO_BAT_FAULT_OUT 0x20    /* R/W level 0:low in batt_fault */
#define ASIC3_GPIO_INT_STATUS    0x24    /* R/W 0:none, 1:detect */
#define ASIC3_GPIO_ALT_FUNCTION  0x28	 /* R/W 1:LED register control */
#define ASIC3_GPIO_SLEEP_CONF    0x2c    /* R/W bit 1: autosleep */
#define ASIC3_GPIO_STATUS        0x30    /* R   Pin status */

/*
 * ASIC3 GPIO alternate-function config word.
 * Bits 0..6   gpio number
 * Bits 7..13  Alternate function
 * Bit  14     Direction
 * Bit  15     Initial value
 */
#define ASIC3_CONFIG_GPIO_PIN(config)  ((config) & 0x7f)
#define ASIC3_CONFIG_GPIO_ALT(config)  (((config) & (0x7f << 7)) >> 7)
#define ASIC3_CONFIG_GPIO_DIR(config)  (((config) & BIT(14)) >> 14)
#define ASIC3_CONFIG_GPIO_INIT(config) (((config) & BIT(15)) >> 15)
#define ASIC3_CONFIG_GPIO(gpio, alt, dir, init) (((gpio) & 0x7f) \
	| (((alt) & 0x7f) << 7) | (((dir) & 0x1) << 14) \
	| (((init) & 0x1) << 15))

/*
 * Alternate functions used for the LED and CompactFlash lines.
 * ASIC3_GPIOC0_LED0 to ASIC3_GPIOC2_LED2 are consumed directly by
 * drivers/leds/leds-asic3.c, as that routing is chip-internal rather than
 * board policy. The MFD core does not program any of these.
 */
#define ASIC3_GPIOC0_LED0		ASIC3_CONFIG_GPIO(32, 1, 0, 0)
#define ASIC3_GPIOC1_LED1		ASIC3_CONFIG_GPIO(33, 1, 0, 0)
#define ASIC3_GPIOC2_LED2		ASIC3_CONFIG_GPIO(34, 1, 0, 0)
#define ASIC3_GPIOC4_CF_nCD		ASIC3_CONFIG_GPIO(36, 1, 0, 0)
#define ASIC3_GPIOC5_nCIOW		ASIC3_CONFIG_GPIO(37, 1, 1, 0)
#define ASIC3_GPIOC6_nCIOR		ASIC3_CONFIG_GPIO(38, 1, 1, 0)
#define ASIC3_GPIOC7_nPCE_1		ASIC3_CONFIG_GPIO(39, 1, 0, 0)
#define ASIC3_GPIOC8_nPCE_2		ASIC3_CONFIG_GPIO(40, 1, 0, 0)
#define ASIC3_GPIOC9_nPOE		ASIC3_CONFIG_GPIO(41, 1, 0, 0)
#define ASIC3_GPIOC10_nPWE		ASIC3_CONFIG_GPIO(42, 1, 0, 0)
#define ASIC3_GPIOC11_PSKTSEL		ASIC3_CONFIG_GPIO(43, 1, 0, 0)
#define ASIC3_GPIOC12_nPREG		ASIC3_CONFIG_GPIO(44, 1, 0, 0)
#define ASIC3_GPIOC13_nPWAIT		ASIC3_CONFIG_GPIO(45, 1, 1, 0)
#define ASIC3_GPIOC14_nPIOIS16		ASIC3_CONFIG_GPIO(46, 1, 1, 0)
#define ASIC3_GPIOC15_nPIOR		ASIC3_CONFIG_GPIO(47, 1, 0, 0)
#define ASIC3_GPIOD4_CF_nCD		ASIC3_CONFIG_GPIO(52, 1, 0, 0)
#define ASIC3_GPIOD11_nCIOIS16		ASIC3_CONFIG_GPIO(59, 1, 0, 0)
#define ASIC3_GPIOD12_nCWAIT		ASIC3_CONFIG_GPIO(60, 1, 0, 0)
#define ASIC3_GPIOD15_nPIOW		ASIC3_CONFIG_GPIO(63, 1, 0, 0)

#define ASIC3_NUM_LEDS			3

/*
 * LED blink-timer block, one per channel at a 0x0100 stride. GPIOC0/1/2
 * are hardwired to these three channels' alternate function
 * (ASIC3_GPIOC0_LED0 and siblings above), so drivers/leds/leds-asic3.c
 * switches that routing itself.
 */
#define ASIC3_LED0_BASE			0x0700
#define ASIC3_LED1_BASE			0x0800
#define ASIC3_LED2_BASE			0x0900
#define ASIC3_LED_TIMEBASE		0x0000	/* R/W  7 bits */
#define ASIC3_LED_PERIODTIME		0x0004	/* R/W 12 bits */
#define ASIC3_LED_DUTYTIME		0x0008	/* R/W 12 bits */
#define ASIC3_LED_AUTOSTOPCOUNT		0x000c	/* R/W 16 bits */

#define ASIC3_CLOCK_BASE	0x0A00
#define ASIC3_CLOCK_CDEX	0x00
#define ASIC3_CLOCK_SEL		0x04

#define CLOCK_CDEX_SOURCE0      BIT(0)
#define CLOCK_CDEX_SOURCE1      BIT(1)
#define CLOCK_CDEX_SPI          BIT(2)
#define CLOCK_CDEX_OWM          BIT(3)
#define CLOCK_CDEX_PWM0         BIT(4)
#define CLOCK_CDEX_PWM1         BIT(5)
#define CLOCK_CDEX_LED0         BIT(6)
#define CLOCK_CDEX_LED1         BIT(7)
#define CLOCK_CDEX_LED2         BIT(8)
/* Clocks settings: 1 for 24.576 MHz, 0 for 12.288Mhz */
#define CLOCK_CDEX_SD_HOST      BIT(9)
#define CLOCK_CDEX_SD_BUS       BIT(10)
#define CLOCK_CDEX_SMBUS        BIT(11)
#define CLOCK_CDEX_CONTROL_CX   BIT(12)
#define CLOCK_CDEX_EX0          BIT(13)  /* R/W: 32.768 kHz crystal */
#define CLOCK_CDEX_EX1          BIT(14)  /* R/W: 24.576 MHz crystal */

#define CLOCK_SEL_SD_HCLK_SEL   BIT(0)
#define CLOCK_SEL_SD_BCLK_SEL   BIT(1)
#define CLOCK_SEL_CX            BIT(2)   /* INT clock source (32.768 kHz) */

#define ASIC3_INTR_BASE		0x0B00
#define ASIC3_INTR_INT_MASK       0x00  /* Interrupt mask control */
#define ASIC3_INTR_P_INT_STAT     0x04  /* Peripheral interrupt status */
#define ASIC3_INTR_INT_CPS        0x08  /* Interrupt timer clock pre-scale */
#define ASIC3_INTR_INT_TBS        0x0c  /* Interrupt timer set */

#define ASIC3_INTMASK_GINTMASK    BIT(0)  /* Global INTs mask 1:enable */
#define ASIC3_INTMASK_GINTEL      BIT(1)  /* 1: rising edge, 0: hi level */
#define ASIC3_INTMASK_MASK0       BIT(2)  /* LED0, shift by (irq-64) */

#define ASIC3_INTR_PERIPHERAL_A   BIT(0)
#define ASIC3_INTR_PERIPHERAL_B   BIT(1)
#define ASIC3_INTR_PERIPHERAL_C   BIT(2)
#define ASIC3_INTR_PERIPHERAL_D   BIT(3)
#define ASIC3_INTR_LED0           BIT(4)
#define ASIC3_INTR_LED1           BIT(5)
#define ASIC3_INTR_LED2           BIT(6)
#define ASIC3_INTR_SPI            BIT(7)
#define ASIC3_INTR_SMBUS          BIT(8)
#define ASIC3_INTR_OWM            BIT(9)
#define ASIC3_INTR_PENDING_MASK   0x3ff	/* all ten bits above */

/* Basic control of the SD block. */
#define ASIC3_SDHWCTRL_BASE     0x0E00
#define ASIC3_SDHWCTRL_SDCONF     0x00

#define ASIC3_SDHWCTRL_SUSPEND    BIT(0)  /* 1=suspend all SD operations */
#define ASIC3_SDHWCTRL_CLKSEL     BIT(1)  /* 1=SDICK, 0=HCLK */
#define ASIC3_SDHWCTRL_PCLR       BIT(2)  /* All registers of SDIO cleared */
#define ASIC3_SDHWCTRL_LEVCD      BIT(3)  /* SD card detection: 0:low */
#define ASIC3_SDHWCTRL_LEVWP      BIT(4)  /* SD write protection: 0=high */
#define ASIC3_SDHWCTRL_SDLED      BIT(5)  /* SD card LED signal 0=disable */
#define ASIC3_SDHWCTRL_SDPWR      BIT(6)  /* SD card power ctrl 1=enable */

#define ASIC3_EXTCF_BASE        0x1100
#define ASIC3_EXTCF_SELECT        0x00
#define ASIC3_EXTCF_RESET         0x04

#define ASIC3_EXTCF_SMOD0	         BIT(0)  /* slot number of mode 0 */
#define ASIC3_EXTCF_SMOD1	         BIT(1)  /* slot number of mode 1 */
#define ASIC3_EXTCF_SMOD2	         BIT(2)  /* slot number of mode 2 */
#define ASIC3_EXTCF_OWM_EN	         BIT(4)  /* enable onewire module */
#define ASIC3_EXTCF_OWM_SMB	         BIT(5)  /* OWM bus selection */
#define ASIC3_EXTCF_OWM_RESET            BIT(6)
#define ASIC3_EXTCF_CF0_SLEEP_MODE       BIT(7)
#define ASIC3_EXTCF_CF1_SLEEP_MODE       BIT(8)
#define ASIC3_EXTCF_CF0_PWAIT_EN         BIT(10)
#define ASIC3_EXTCF_CF1_PWAIT_EN         BIT(11)
#define ASIC3_EXTCF_CF0_BUF_EN           BIT(12)
#define ASIC3_EXTCF_CF1_BUF_EN           BIT(13)
#define ASIC3_EXTCF_SD_MEM_ENABLE        BIT(14)
#define ASIC3_EXTCF_CF_SLEEP             BIT(15)

/* The 1-Wire interface (DS1WM) is handled by drivers/w1/masters/ds1wm.c */
#define ASIC3_OWM_BASE		0xC00

/*
 * The SD configuration registers are in a second, physically disjoint
 * window (the "sd" named reg resource of the devicetree node), not at an
 * offset within the "core" window.
 */
#define ASIC3_SD_CONFIG_BASE	0x0400 /* Assumes 32 bit addressing */
#define ASIC3_SD_CONFIG_SIZE	0x0200 /* Assumes 32 bit addressing */
#define ASIC3_SD_CTRL_BASE	0x1000
#define ASIC3_SDIO_CTRL_BASE	0x1200

#define ASIC3_MAP_SIZE_32BIT	0x2000
#define ASIC3_MAP_SIZE_16BIT	0x1000

/*
 * Register access for sub-device drivers (such as drivers/leds/leds-asic3.c)
 * that need direct access to blocks the gpio_chip/clk_hw API does not cover,
 * such as the LED PWM timers. "reg" is a byte offset in the chip's own
 * 32-bit-addressed map, as for the ASIC3_* macros above; the bus_shift
 * adjustment happens inside.
 */
struct asic3;
u32 asic3_read_register(struct asic3 *asic, unsigned int reg);
void asic3_write_register(struct asic3 *asic, unsigned int reg, u32 value);

#endif /* __LINUX_MFD_ASIC3_H */
