// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Board setup for the HP iPAQ h5000 family (h5150, h5450/h5455,
 * h5550/h5555).
 *
 * Copyright 2000-2003  Hewlett-Packard Company.
 * Copyright 2002       Jamey Hicks <jamey.hicks@hp.com>
 * Copyright 2004-2005  Phil Blundell <pb@handhelds.org>
 * Copyright 2007-2008  Anton Vorontsov <cbouatmailru@gmail.com>
 *
 * Author: Jamey Hicks.
 *
 * This is not a MACHINE_START board port: the board boots as a plain
 * "marvell,pxa250" devicetree machine via arch/arm/mach-pxa/pxa-dt.c, and
 * this file only patches in what the devicetree does not describe -- the
 * pin muxing, the static memory timings and the GPIO hogs. All of it is
 * gated behind of_machine_is_compatible(), since the generic PXA25x DT
 * machine has no per-board hook to call into.
 *
 * No device is registered from here: the board's devices belong in the
 * devicetree, where the generic PXA25x DT machine populates them.
 *
 * The pin table was transcribed from three vendor sources that agree
 * pin for pin on the GPIO numbering: the GPIO name list of the HP 2.6
 * kernel's h5400.h, its h5400_gpio_modes[] table for the direction and
 * alternate function of each pin, and its h5400-init.h table for the
 * sleep level of each pin. A few alternate-function outputs deviate from
 * the stock low-power state of the mfp-pxa25x.h macros to follow that
 * sleep table; each is marked [!=std LPM].
 */

#include <linux/clk/pxa.h>
#include <linux/gpio/legacy.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/soc/pxa/cpu.h>

#include "generic.h"
#include "h5000.h"
#include "mfp-pxa25x.h"
#include "smemc.h"

/*
 * h54xx uses the BTUART on GPIO42-45 for the onboard Bluetooth, while
 * h51xx/h55xx use the HWUART. The shared table is muxed for the HWUART;
 * these four pins override it on a PXA250.
 */
static unsigned long h5400_btuart_pin_config[] __initdata = {
	GPIO42_BTUART_RXD,
	MFP_CFG_OUT(GPIO43, AF2, DRIVE_LOW),
	GPIO44_BTUART_CTS,
	MFP_CFG_OUT(GPIO45, AF2, DRIVE_LOW),
};

static unsigned long h5000_pin_config[] __initdata = {
	/*
	 * Buttons and miscellaneous plain GPIOs. Only GPIO0 is marked
	 * wakeup-capable: the power key sets wakeup-source, and
	 * gpio_set_wake() arms PWER/PRER/PFER only for a pin whose config
	 * carries MFP_LPM_CAN_WAKEUP.
	 */
	GPIO0_GPIO | WAKEUP_ON_EDGE_BOTH,	/* H5000_GPIO_POWER_BUTTON */
	GPIO1_GPIO,				/* H5000_GPIO_RESET_BUTTON_N */
	GPIO2_GPIO,				/* H5000_GPIO_OPT_INT */
	GPIO3_GPIO,				/* H5000_GPIO_BACKUP_POWER */
	GPIO4_GPIO,				/* H5000_GPIO_ACTION_BUTTON */
	GPIO5_GPIO,				/* H5000_GPIO_COM_DCD_SOMETHING */
	MFP_CFG_IN(GPIO6, AF0),			/* not connected */
	GPIO7_GPIO,				/* H5000_GPIO_RESET_BUTTON_AGAIN_N */
	MFP_CFG_OUT(GPIO8, AF0, DRIVE_HIGH),	/* not connected */
	GPIO9_GPIO,				/* H5000_GPIO_RSO_N (BATT_FAULT) */
	GPIO10_GPIO,				/* H5000_GPIO_ASIC_INT_N */

	/* Bluetooth (LMX9814) control lines */
	MFP_CFG_OUT(GPIO11, AF0, DRIVE_LOW),	/* H5000_GPIO_BT_ENV_0 */
	MFP_CFG_OUT(GPIO13, AF0, DRIVE_LOW),	/* H5000_GPIO_BT_ENV_1 */
	GPIO14_GPIO,				/* H5000_GPIO_BT_WU */

	/* 32 kHz clock output; the vendor sleep level is high [!=std LPM] */
	MFP_CFG_OUT(GPIO12, AF1, DRIVE_HIGH),

	/*
	 * Static memory chip selects. GPIO33 (nCS_5, the companion ASIC)
	 * is absent, as it is from the vendor table.
	 */
	GPIO15_nCS_1,
	GPIO78_nCS_2,
	GPIO79_nCS_3,
	GPIO80_nCS_4,

	MFP_CFG_OUT(GPIO16, AF0, DRIVE_LOW),	/* not connected */
	MFP_CFG_OUT(GPIO17, AF0, DRIVE_LOW),	/* not connected */

	/*
	 * GPIO18 is the static memory controller's RDY input, which is
	 * shared with the variable-latency nCS_1 cycle programmed below
	 * and with the PC Card socket. The boot loader already muxes it
	 * this way; naming it keeps the setting from being an undocumented
	 * inheritance. GPIO19-21 (DREQ_1, DREQ_0 and the companion ASIC's
	 * bus output enable) stay unmuxed, as in the vendor table.
	 */
	GPIO18_RDY,

	MFP_CFG_OUT(GPIO22, AF0, DRIVE_LOW),	/* not connected */

	/*
	 * SSP1 to the expansion pack connector. GPIO24 is a plain GPIO
	 * chip select, not the SSP1_SFRM alternate function.
	 */
	GPIO23_SSP1_SCLK,
	MFP_CFG_OUT(GPIO24, AF0, DRIVE_HIGH),	/* H5000_GPIO_OPT_SPI_CS_N */
	GPIO25_SSP1_TXD,
	GPIO26_SSP1_RXD,
	MFP_CFG_OUT(GPIO27, AF0, DRIVE_LOW),	/* not connected */

	/* I2S to the audio codec */
	GPIO28_I2S_BITCLK_OUT,
	GPIO29_I2S_SDATA_IN,
	GPIO30_I2S_SDATA_OUT,
	GPIO31_I2S_SYNC,
	GPIO32_I2S_SYSCLK,

	/*
	 * FFUART, the RS232 header, which is enabled by
	 * H5000_GPIO_POWER_RS232_N (hogged below). TXD and RTS follow the
	 * vendor sleep table. [!=std LPM]
	 */
	GPIO34_FFUART_RXD,
	GPIO35_FFUART_CTS,
	GPIO36_FFUART_DCD,
	GPIO37_FFUART_DSR,
	GPIO38_FFUART_RI,
	MFP_CFG_OUT(GPIO39, AF2, DRIVE_LOW),	/* FFUART_TXD [!=std LPM] */
	GPIO40_FFUART_DTR,
	MFP_CFG_OUT(GPIO41, AF2, DRIVE_LOW),	/* FFUART_RTS [!=std LPM] */

	/*
	 * HWUART to the onboard Bluetooth (LMX9814). The WinCE serial
	 * driver programs GPIO42/44 as AF3 inputs and GPIO43/45 as AF3
	 * outputs, and its registry names the HWUART base address, never the
	 * BTUART. A PXA250 (h54xx) has no HWUART; h5000_init() muxes these
	 * pins for the BTUART there. TXD and RTS follow the vendor sleep
	 * table. [!=std LPM]
	 */
	GPIO42_HWUART_RXD,
	MFP_CFG_OUT(GPIO43, AF3, DRIVE_LOW),	/* HWUART_TXD [!=std LPM] */
	GPIO44_HWUART_CTS,
	MFP_CFG_OUT(GPIO45, AF3, DRIVE_LOW),	/* HWUART_RTS [!=std LPM] */

	/*
	 * PC Card / CompactFlash bus strobes. The boot loader's GAFR1
	 * value already selects AF2 for GPIO48-55 and AF1 for GPIO56/57,
	 * and the vendor sleep table gives the levels. GPIO54 is the one
	 * strobe whose vendor sleep level (low) differs from the stock
	 * macro (high); the board table wins. [!=std LPM]
	 * GPIO46/47 (IrDA) are left unmuxed.
	 */
	GPIO48_nPOE,
	GPIO49_nPWE,
	GPIO50_nPIOR,
	GPIO51_nPIOW,
	GPIO52_nPCE_1,
	GPIO53_nPCE_2,
	MFP_CFG_OUT(GPIO54, AF2, DRIVE_LOW),	/* nPSKTSEL [!=std LPM] */
	GPIO55_nPREG,
	GPIO56_nPWAIT,
	GPIO57_nIOIS16,

	/* Power, charging, sensors, motor and audio-adjacent control lines */
	MFP_CFG_OUT(GPIO58, AF0, DRIVE_LOW),	/* H5000_GPIO_IRDA_SD */
	MFP_CFG_OUT(GPIO59, AF0, DRIVE_LOW),	/* USB charge source switch */
	MFP_CFG_OUT(GPIO60, AF0, DRIVE_LOW),	/* H5000_GPIO_POWER_SD_N */
	MFP_CFG_OUT(GPIO61, AF0, DRIVE_HIGH),	/* H5000_GPIO_POWER_RS232_N, hogged low below */
	MFP_CFG_OUT(GPIO62, AF0, DRIVE_HIGH),	/* H5000_GPIO_POWER_ACCEL_N */
	MFP_CFG_OUT(GPIO63, AF0, DRIVE_LOW),	/* not connected */
	MFP_CFG_OUT(GPIO64, AF0, DRIVE_LOW),	/* H5000_GPIO_OPT_NVRAM */
	MFP_CFG_OUT(GPIO65, AF0, DRIVE_LOW),	/* H5000_GPIO_CHG_EN */
	MFP_CFG_OUT(GPIO66, AF0, DRIVE_LOW),	/* H5000_GPIO_USB_PULLUP */
	MFP_CFG_OUT(GPIO67, AF0, DRIVE_LOW),	/* H5000_GPIO_BT_2V8 */
	MFP_CFG_OUT(GPIO68, AF0, DRIVE_LOW),	/* H5000_GPIO_EXT_CHG_RATE */
	MFP_CFG_OUT(GPIO69, AF0, DRIVE_LOW),	/* not connected */
	MFP_CFG_OUT(GPIO70, AF0, DRIVE_LOW),	/* H5000_GPIO_CIR_RESET */
	MFP_CFG_OUT(GPIO71, AF0, DRIVE_HIGH),	/* H5000_GPIO_POWER_LIGHT_SENSOR_N */
	MFP_CFG_OUT(GPIO72, AF0, DRIVE_LOW),	/* H5000_GPIO_BT_M_RESET */
	MFP_CFG_OUT(GPIO73, AF0, DRIVE_LOW),	/* H5000_GPIO_STD_CHG_RATE */
	GPIO74_GPIO,				/* H5000_GPIO_SD_WP_N */
	MFP_CFG_OUT(GPIO75, AF0, DRIVE_HIGH),	/* H5000_GPIO_MOTOR_ON_N */
	GPIO76_GPIO,				/* H5000_GPIO_HEADPHONE_DETECT */
	MFP_CFG_OUT(GPIO77, AF0, DRIVE_LOW),	/* H5000_GPIO_USB_CHG_RATE */
};

/*
 * Set-and-forget pins with no driver of their own. H5000_GPIO_POWER_RS232_N
 * is the inverted FORCEON of the RS232 transceiver, so driving it low
 * forces the transceiver on whatever state the boot loader left it in;
 * the serial port is the only console this board has.
 */
static const struct {
	unsigned int gpio;
	unsigned long flags;
	const char *label;
} h5000_gpio_hogs[] = {
	{ H5000_GPIO_POWER_RS232_N, GPIOF_OUT_INIT_LOW, "rs232-forceon" },
};

/*
 * Static memory: CS0 flash, CS1 MediaQ (16-bit bus, variable latency I/O),
 * CS5 SAMCOP.
 */
static void __init fix_msc(void)
{
	__raw_writel(0x129c24f2, MSC0);
	__raw_writel(0x7ff424fa, MSC1);
	__raw_writel(0x7ff47ff4, MSC2);

	__raw_writel(__raw_readl(MDREFR) | MDREFR_K2FREE | MDREFR_K2DB2, MDREFR);

	/*
	 * The SAMCOP on SDCLK2 does not tolerate MDREFR_K2DB2 changing. The
	 * bit is set once above, and the clock driver would otherwise clear
	 * and set it again on a frequency change, so tell it to leave it
	 * alone. The vendor cpufreq driver for this board dropped the same
	 * bit from its MDREFR sequence.
	 */
	pxa2xx_clk_exclude_mdrefr_db2(MDREFR_K2DB2);
}

static int __init h5000_init(void)
{
	if (!of_machine_is_compatible("hp,ipaq-h5000"))
		return 0;

	fix_msc();
	pxa2xx_mfp_config(ARRAY_AND_SIZE(h5000_pin_config));
	if (!cpu_is_pxa255())
		pxa2xx_mfp_config(ARRAY_AND_SIZE(h5400_btuart_pin_config));

	return 0;
}
arch_initcall(h5000_init);

static int __init h5000_late_init(void)
{
	int i, ret;

	if (!of_machine_is_compatible("hp,ipaq-h5000"))
		return 0;

	for (i = 0; i < ARRAY_SIZE(h5000_gpio_hogs); i++) {
		ret = gpio_request_one(h5000_gpio_hogs[i].gpio,
				       h5000_gpio_hogs[i].flags,
				       h5000_gpio_hogs[i].label);
		if (ret)
			pr_err("h5000: failed to hog GPIO%u (%s): %d\n",
			       h5000_gpio_hogs[i].gpio,
			       h5000_gpio_hogs[i].label, ret);
	}

	return 0;
}
late_initcall(h5000_late_init);
