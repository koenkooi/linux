// SPDX-License-Identifier: GPL-2.0-only
/*
 * Board setup for the HP iPAQ h2200 (h2210/h2212/h2215 are the same
 * board).
 *
 * This is not a MACHINE_START board port: the board boots as a plain
 * "marvell,pxa250" devicetree machine via arch/arm/mach-pxa/pxa-dt.c, and
 * this file only patches in what the devicetree does not describe yet --
 * the pin muxing of every pad except the audio ones, which are left to
 * the pin controller, and the static memory timings. All of it is gated
 * behind of_machine_is_compatible() rather than a real
 * MACHINE_START/.init_machine hook, since the generic PXA25x DT machine
 * doesn't have a per-board hook to call into. This shim shrinks as the
 * remaining pads move to &pinctrl states in pxa250-hp-ipaq-h2200.dts.
 *
 * No device is registered from here: the board's devices belong in the
 * devicetree, where the generic PXA25x DT machine populates them.
 *
 * The pin table below is translated from the old out-of-tree
 * h2200-init.h (raw GAFR/GPDR/GPSR/PGSR arrays) and h2200-gpio.h (the
 * GPIO_NR_H2200_* names), cross-checked against
 * mfp-pxa25x.h. A handful of pins don't match either
 * source verbatim; each is commented at the point of deviation.
 */

#include <linux/gpio/legacy.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kstrtox.h>
#include <linux/of.h>
#include <linux/printk.h>

#include "generic.h"
#include "h2200.h"
#include "mfp-pxa25x.h"
#include "smemc.h"

/*
 * Static memory timing: CS0 = HAMCOP, CS1 = MediaQ 1178. The MSC0/1/2
 * values are the ones the WinCE bootloader already programs -- direct
 * precedent is the deleted mainline h5000.c's own fix_msc(), same shape,
 * sibling PXA25x iPAQ, same MediaQ chip on CS1.
 */
#define H2200_MSC0_DEFAULT	0x246c7ffc

/*
 * nCS_1 (MediaQ) half of MSC0, overridable as h2200.msc0_cs1=0xNNNN so the
 * strobe widths that bound host writes into video memory can be tried
 * without a rebuild. The nCS_0 half stays at the HAMCOP value above. A
 * VLIO cycle ends only when the chip asserts RDY and the static memory
 * controller has no bus-error timeout, so a value that is too fast for
 * the chip corrupts video memory or hangs the machine inside one store.
 */
static u32 h2200_msc0_cs1 = H2200_MSC0_DEFAULT >> 16;

static int __init h2200_msc0_cs1_setup(char *str)
{
	u32 val;

	if (kstrtou32(str, 0, &val) || val > 0xffff)
		return -EINVAL;

	/* RT and RBW are the bus protocol, not timing: refuse to change them. */
	if ((val & 0xf) != ((H2200_MSC0_DEFAULT >> 16) & 0xf)) {
		pr_warn("h2200: msc0_cs1=%#x changes nCS_1 ROM type or bus width, ignored\n",
			val);
		return -EINVAL;
	}

	h2200_msc0_cs1 = val;
	return 0;
}
early_param("h2200.msc0_cs1", h2200_msc0_cs1_setup);

static void __init fix_msc(void)
{
	u32 cs0 = H2200_MSC0_DEFAULT & 0xffff;

	__raw_writel(cs0 | (h2200_msc0_cs1 << 16), MSC0);
	__raw_writel(0x7ff07ff0, MSC1);
	__raw_writel(0x7ff07ff0, MSC2);
}

static unsigned long h2200_pin_config[] __initdata = {
	/*
	 * Wakeup-capable inputs: the board wakes on either edge of GPIO0,
	 * GPIO12 and GPIO13 (PWER = PFER = PRER = GPIO0 | GPIO12 | GPIO13,
	 * plus the RTC). Actually asserting PWER happens at suspend time via
	 * enable_irq_wake() -> gpio_set_wake(), so all this table does is
	 * mark the pins wakeup-capable.
	 */
	GPIO0_GPIO | WAKEUP_ON_EDGE_BOTH,	/* H2200_GPIO_POWER_BUTTON */
	GPIO12_GPIO | WAKEUP_ON_EDGE_BOTH,	/* H2200_GPIO_AC_IN_N */
	GPIO13_GPIO | WAKEUP_ON_EDGE_BOTH,	/* H2200_GPIO_POWER_ON_N */

	/*
	 * ASIC_INT_N is a wakeup source for whatever is routed through the
	 * HAMCOP: hamcop_irq_set_wake() propagates the wake request of a
	 * HAMCOP interrupt to this GPIO. No node asks for it yet, the power
	 * key being on PXA GPIO13. gpio_set_wake() arms PWER/PFER from this
	 * static table, independent of the edge the hwirq is configured for
	 * at runtime, so the falling edge has to match the HAMCOP node's
	 * interrupt specifier by hand.
	 */
	GPIO5_GPIO | WAKEUP_ON_EDGE_FALL,	/* ASIC_INT_N, HAMCOP's upstream IRQ */

	/*
	 * GPIO1 is kept as plain input (AF0) rather than the GPIO1_RST
	 * hardware-reset alternate function; it is not known whether the
	 * reset alternate function is wired on this board.
	 */
	GPIO1_GPIO,	/* H2200_GPIO_RESET_BUTTON_N */

	/* Plain status/sense inputs */
	GPIO2_GPIO,	/* BATT_DOOR_N */
	GPIO3_GPIO,	/* USB_DETECT_N, vbus sense */
	GPIO4_GPIO,	/* RS232_DCD (parallel path, active even w/ RS232 off) */
	MFP_CFG_IN(GPIO6, AF1),	/* NAND flash size strap, despite its
				 * CIR_SIR_N signal name, not CIR/SIR
				 */
	GPIO7_GPIO,	/* CF_INT_N, PCMCIA ready/-IREQ */
	/*
	 * SD_DETECT_N: AF0 (GPIO) rather than AF1, its MMC_CS0 alternate
	 * function -- PXA255 edge-detect GPIO IRQs are assumed to require
	 * AF0 GPIO mode (open question, verify against hardware/the PXA255
	 * manual).
	 */
	GPIO8_GPIO,	/* SD_DETECT_N, SD card detect */
	GPIO9_GPIO,	/* CF_DETECT_N, PCMCIA card detect */
	MFP_CFG_OUT(GPIO10, AF0, DRIVE_LOW),	/* BT_WAKE */
	GPIO11_3_6MHz,
	GPIO14_GPIO,	/* MQ1178_IRQ_N, the MediaQ frame interrupt */

	/* PC Card / CompactFlash socket strobes and control lines */
	MFP_CFG_OUT(GPIO48, AF2, DRIVE_LOW),	/* nPOE, low in sleep */
	GPIO49_nPWE,
	MFP_CFG_OUT(GPIO50, AF2, DRIVE_LOW),	/* nPIOR, low in sleep */
	MFP_CFG_OUT(GPIO51, AF2, DRIVE_LOW),	/* nPIOW, low in sleep */
	MFP_CFG_OUT(GPIO52, AF2, DRIVE_LOW),	/* nPCE_1, low in sleep */
	MFP_CFG_OUT(GPIO53, AF2, DRIVE_LOW),	/* nPCE_2, low in sleep */
	GPIO54_nPSKTSEL,
	MFP_CFG_OUT(GPIO55, AF2, DRIVE_LOW),	/* nPREG, low in sleep */
	GPIO56_nPWAIT,
	GPIO57_nIOIS16,
	MFP_CFG_OUT(GPIO23, AF0, DRIVE_LOW),	/* CF_RESET */
	MFP_CFG_OUT(GPIO25, AF0, DRIVE_HIGH),	/* CF_ADD_EN_N */
	MFP_CFG_OUT(GPIO26, AF0, DRIVE_LOW),	/* CF_POWER_EN */
	MFP_CFG_OUT(GPIO63, AF0, DRIVE_LOW),	/* CF_BUFF_EN */

	/* Static memory chip selects */
	GPIO15_nCS_1,	/* MediaQ 1178 */
	GPIO78_nCS_2,	/* HAMCOP */

	/* DMA/ready, and the CPU-speed straps parked as plain inputs */
	GPIO18_RDY,
	GPIO20_DREQ_0,
	GPIO19_GPIO,	/* CPU_200MHZ strap -- not DREQ_1 on this board */
	GPIO27_GPIO,	/* CPU_400MHZ strap */

	/*
	 * FFUART: the physical RS232 header, gated by the level-shifter
	 * enable in the hog list below. TXD/DTR/RTS are driven low in
	 * sleep, as WinCE leaves them, unlike the idle-high low-power
	 * state of the stock macros.
	 */
	GPIO34_FFUART_RXD,
	GPIO35_FFUART_CTS,
	GPIO36_FFUART_DCD,
	GPIO37_FFUART_DSR,
	GPIO38_FFUART_RI,
	MFP_CFG_OUT(GPIO39, AF2, DRIVE_LOW),	/* FFUART_TXD, low in sleep */
	MFP_CFG_OUT(GPIO40, AF2, DRIVE_LOW),	/* FFUART_DTR, low in sleep */
	MFP_CFG_OUT(GPIO41, AF2, DRIVE_LOW),	/* FFUART_RTS, low in sleep */

	/*
	 * HWUART, not BTUART: onboard Zeevo Bluetooth. GPIO42-45 are the
	 * same physical pins either way (AF1/AF2 = BTUART, AF3 = HWUART),
	 * but the h2200 is a PXA255 and the rule for the iPAQs is PXA250 ->
	 * BTUART, PXA255 -> HWUART. BTUART got no HCI response from the
	 * chip in any protocol, baud rate or flow-control combination that
	 * was tried.
	 *
	 * The ports are driven by 8250 through 8250_pxa.c, which grants
	 * UART_CAP_AFE to every PXA port and applies auto-flow-control from
	 * CRTSCTS, so HWUART is not privileged in that respect. Userspace
	 * has to ask for flow control (hciattach's "flow" option): without
	 * it the Zeevo chip is never throttled, and a synchronous console
	 * write on FFUART can block for long enough to overrun the receive
	 * FIFO of this port.
	 *
	 * Parking these as plain GPIO at boot and switching them to the
	 * alternate function from a serial .configure() hook when the port
	 * is opened would avoid phantom-powering the Zeevo chip while it's
	 * off, but the modern 8250 layer has no such hook, so
	 * -- same as the deleted mainline h5000.c -- these are muxed
	 * statically here instead. The Zeevo chip's power sequencing
	 * (GPIO10/74/76) is separate from the UART pins.
	 */
	GPIO42_HWUART_RXD,
	GPIO43_HWUART_TXD,
	GPIO44_HWUART_CTS,
	GPIO45_HWUART_RTS,

	/*
	 * STUART: wired for SIR/IrDA, but net/irda is gone from mainline
	 * so this is just a plain UART now, with the alternate functions
	 * of the standard macros.
	 */
	GPIO46_STUART_RXD,
	MFP_CFG_OUT(GPIO47, AF1, DRIVE_LOW),	/* STUART_TXD, low in sleep */

	/*
	 * Not here: the I2S pads (GPIO28-32) and the UDA1380 power/reset,
	 * speaker amplifier and microphone GPIOs (GPIO70/71, GPIO65/69, GPIO66)
	 * are left to the pin controller.
	 */

	/* USB_PULLUP_N, the USB D+ pullup */
	MFP_CFG_OUT(GPIO33, AF0, DRIVE_HIGH),

	/*
	 * Backlight: PWM0 brightness, then GPIO17 and the GPIO22 light
	 * switch, whose run-time levels belong to pwm-backlight and to the
	 * DT regulator on GPIO22; only direction/AF/sleep-state are
	 * established here. This table, not
	 * &pinctrl, is what puts GPIO16 in its PWM0 alternate function.
	 */
	GPIO16_PWM0_OUT,
	MFP_CFG_OUT(GPIO17, AF0, DRIVE_LOW),
	MFP_CFG_OUT(GPIO22, AF0, DRIVE_LOW),

	/*
	 * LCD data lines 58/59: the PXA255 LCD controller is unwired on
	 * this board (MediaQ 1178 is the real display chip), so these are
	 * plain GPIO inputs; nothing on them needs an output level.
	 */
	GPIO58_GPIO,
	GPIO59_GPIO,

	/*
	 * MediaQ 1178 power/reset -- runtime level owned by the display
	 * controller's driver, not a boot-time hog; only direction/AF/
	 * sleep-state are established here.
	 */
	MFP_CFG_OUT(GPIO60, AF0, DRIVE_LOW),	/* MQ1178_VDD */
	MFP_CFG_OUT(GPIO67, AF0, DRIVE_LOW),	/* MQ1178_POWER_ON */

	/*
	 * SD slot power -- runtime level owned by the SD host's slot supply,
	 * not a boot-time hog; only direction/AF/sleep-state here, same split
	 * as the MediaQ pins above.
	 */
	MFP_CFG_OUT(GPIO61, AF0, DRIVE_LOW),	/* SD_POWER_EN */

	/* Bootloader-present strap, no consumer yet */
	GPIO62_GPIO,

	/*
	 * Bluetooth module control: reset and power outputs, the module
	 * ident input, and BT_IRQ, a plain-GPIO input (BT_WAKE is GPIO10
	 * above).
	 */
	MFP_CFG_OUT(GPIO74, AF0, DRIVE_LOW),	/* BT_RESET_N */
	MFP_CFG_OUT(GPIO76, AF0, DRIVE_LOW),	/* BT_POWER_ON */
	GPIO75_GPIO,	/* BT_IDENT */
	GPIO77_GPIO,	/* BT_IRQ */

	/* SDRAM size strap, no consumer needed (DT memory node is static) */
	GPIO79_GPIO,

	/*
	 * CHG_EN, the MAX1898's enable input: an output, driven low in
	 * sleep so that charging stops in suspend. Its run-time level
	 * belongs to the charger driver, not a boot-time hog. This entry
	 * sets only the direction and the sleep level, not the output latch,
	 * so until a driver claims the line it keeps the level the boot
	 * loader left.
	 */
	MFP_CFG_OUT(GPIO21, AF0, DRIVE_LOW),

	/*
	 * Fixed board policy pins -- driven to their cold-boot level by
	 * the gpio_request_one() hogs below, not by any driver.
	 */
	MFP_CFG_OUT(GPIO24, AF0, DRIVE_LOW),	/* RS232_CIR_N */
	MFP_CFG_OUT(GPIO64, AF0, DRIVE_LOW),	/* IR_ON_N */
	/*
	 * MQ1178_RESET_N: direction/AF/sleep-state only -- runtime level is
	 * the display controller driver's, not a boot-time hog (see the
	 * GPIO60/67 comment above).
	 */
	MFP_CFG_OUT(GPIO68, AF0, DRIVE_LOW),
	MFP_CFG_OUT(GPIO72, AF0, DRIVE_LOW),	/* CIR_RESET */
	MFP_CFG_OUT(GPIO73, AF0, DRIVE_LOW),	/* CIR_POWER_ON */
	MFP_CFG_OUT(GPIO80, AF0, DRIVE_LOW),	/* RS232_ON */
};

/*
 * Set-and-forget output pins nobody drives dynamically yet -- board
 * policy applied unconditionally at boot (the serial cable is treated as
 * always connected, IrDA/CIR have no driver at all).
 * The audio pins and MediaQ 1178 power/reset are NOT here:
 * their drivers own them.
 */
static const struct {
	unsigned int gpio;
	unsigned long flags;
	const char *label;
} h2200_gpio_hogs[] = {
	{ H2200_GPIO_RS232_ON,	   GPIOF_OUT_INIT_HIGH, "rs232-on" },
	{ H2200_GPIO_RS232_CIR_N, GPIOF_OUT_INIT_HIGH, "rs232-not-cir" },
	{ H2200_GPIO_IR_ON_N,	   GPIOF_OUT_INIT_HIGH, "ir-off" },
	{ H2200_GPIO_CIR_RESET,   GPIOF_OUT_INIT_LOW,  "cir-reset" },
	{ H2200_GPIO_CIR_POWER_ON, GPIOF_OUT_INIT_LOW, "cir-power-off" },
};

/*
 * The board boots as the generic "marvell,pxa250" DT machine (pxa-dt.c),
 * which has no per-board .init_machine hook, so pin muxing and the static
 * memory timings are applied from a gated initcall instead -- same shape
 * as e.g. arch/arm/mach-shmobile/regulator-quirk-rcar-gen2.c. Runs as
 * arch_initcall so it's guaranteed to land after pxa2xx_mfp_init()
 * (postcore_initcall), which is what populates the gpio_desc[] table
 * pxa2xx_mfp_config() validates pins against.
 *
 * Nothing here goes through gpiolib: it only writes MFP and static memory
 * controller registers directly (fix_msc(), pxa2xx_mfp_config()).
 */
static int __init h2200_init(void)
{
	if (!of_machine_is_compatible("hp,ipaq-h2200"))
		return 0;

	fix_msc();
	pxa2xx_mfp_config(ARRAY_AND_SIZE(h2200_pin_config));

	return 0;
}
arch_initcall(h2200_init);

/*
 * gpio_request_one() goes straight through gpiolib rather than the driver
 * core, so it gets no -EPROBE_DEFER retry: called before pxa-gpio and
 * pinctrl-pxa25x have registered (both at device_initcall level, after
 * arch_initcall) it fails permanently and the hog is lost.
 *
 * late_initcall rather than device_initcall: it runs after every
 * device_initcall whatever the link order of arch/ and drivers/ objects.
 */
static int __init h2200_late_init(void)
{
	int i, ret;

	if (!of_machine_is_compatible("hp,ipaq-h2200"))
		return 0;

	for (i = 0; i < ARRAY_SIZE(h2200_gpio_hogs); i++) {
		ret = gpio_request_one(h2200_gpio_hogs[i].gpio,
				       h2200_gpio_hogs[i].flags,
				       h2200_gpio_hogs[i].label);
		if (ret)
			pr_err("h2200: failed to hog GPIO%u (%s): %d\n",
			       h2200_gpio_hogs[i].gpio,
			       h2200_gpio_hogs[i].label, ret);
	}

	return 0;
}
late_initcall(h2200_late_init);
