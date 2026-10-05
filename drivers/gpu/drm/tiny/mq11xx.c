// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM driver for the MediaQ 11xx family of external display controllers,
 * as used on the HP iPAQ h2200 (MediaQ 1178, though real units also report
 * 1188) and the HP iPAQ h5xxx family (MediaQ 1132; the 1100 named by the
 * vendor tables is what the PCI header of a real h5550 disproves, see
 * mq11xx_variant_h5xxx).
 *
 * Neither board's screen is driven by the PXA25x's own LCD controller; that
 * unit is unwired on both. Instead a MediaQ chip sits on a PXA static memory
 * bank and does its own scanout from a 256 KiB VRAM window with its 8 KiB
 * register window immediately after it. The register bank base offsets
 * below (CC/MIU/IC/GC/GE/DC/FP) are chip-family-wide, not specific to any
 * one board.
 *
 * This is a drm/tiny driver: there is no SoC CRTC to hang a drm_panel or
 * drm_bridge off, and new fbdev drivers are not accepted. It follows the
 * structure of cirrus-qemu.c (a shmem-backed shadow plane that is
 * damage-clipped into VRAM on each atomic_update, with no GEM objects
 * backed directly by the slow VLIO bus VRAM) and uses the fixed-mode
 * connector/crtc helpers of sharp-memory.c, since each chip only ever
 * drives one fixed panel per board.
 *
 * What differs between boards is data, not behaviour: panel geometry
 * (struct mq11xx_variant.mode), the power-on register table
 * (.init_regs/.n_init_regs), and which MediaQ part is expected
 * (.expect_device_id, checked against the chip's own PCI header at probe
 * time by mq11xx_identify()). The bring-up sequence, the verify-and-retry
 * register load and the shadow-plane pipeline are shared.
 *
 * Vblank is not supported: there is no drm_vblank_init() and no IRQ
 * handler, and atomic_update copies the damage into VRAM synchronously with
 * no regard for the scanout position, so a large update can tear. The
 * 256 KiB VRAM holds one 240x320 RGB565 frame (153600 bytes) but not two,
 * so a hardware page flip is not available at this format; tear-free
 * updates need the copy timed against the scanout instead. The interrupt
 * output of the chip reaches GPIO14 on the h2200 and is not connected on
 * the h5xxx.
 *
 * The init sequence (supply power-up, endianness quirk, oscillator enable,
 * CC-module power-up, register table load with verify-and-retry, then
 * enabling scanout). Two caveats apply:
 *
 *  1. The one known h2200 power-up timing is a 10 ms settle delay after
 *     POWER_ON; nothing is known about RESET_N and VDD.
 *     mq11xx_power_up() therefore orders VDD, then POWER_ON, then releases
 *     RESET_N. VDD and POWER_ON are regulator supplies whose settle times
 *     are the startup delays of the supplies themselves in devicetree.
 *     Only the 10 ms of POWER_ON is taken from a reference; the other
 *     delays and VDD's assumed polarity are not.
 *
 *  2. The register tables are vendor-tuned values dumped from WinCE, most
 *     notably the FP block's frame-rate-control dither pattern and the
 *     DC.config_8 = 0x86098609 value of the h2200. No datasheet exists to
 *     say which of them are required, so they are used as-is.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_shmem.h>
#include <drm/drm_file.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#define DRIVER_NAME "mq11xx"
#define DRIVER_DESC "MediaQ 11xx family display driver"
#define DRIVER_MAJOR 1
#define DRIVER_MINOR 0

/*
 * Register bank base offsets within the 8 KiB register window. PCI is the
 * chip's PCI-compatible configuration header, not a real PCI bus: the chips
 * sit on a plain memory-mapped bus and the header exists for software
 * identification only (see mq11xx_identify()). It occupies the 0x400-0x4ff
 * gap between the DC and UHC banks.
 */
#define MQ11XX_REG_CC		0x000
#define MQ11XX_REG_MIU		0x080
#define MQ11XX_REG_IC		0x100
#define MQ11XX_REG_GC		0x180
#define MQ11XX_REG_GE		0x200
#define MQ11XX_REG_DC		0x380
#define MQ11XX_REG_PCI		0x400
#define MQ11XX_REG_FP		0x600

/* Individual registers this driver names and touches directly. */
#define MQ11XX_DC_CONFIG_0	(MQ11XX_REG_DC + 0x00)	/* endianness */
#define MQ11XX_DC_CONFIG_1	(MQ11XX_REG_DC + 0x04)	/* oscillator */
#define MQ11XX_DC_CONFIG_2	(MQ11XX_REG_DC + 0x08)	/* CC module enable */
#define MQ11XX_DC_CONFIG_5	(MQ11XX_REG_DC + 0x14)	/* GE/UHC/UDC clocks */
#define MQ11XX_DC_CONFIG_8	(MQ11XX_REG_DC + 0x20)	/* opaque, per variant */
#define MQ11XX_MIU_0		(MQ11XX_REG_MIU + 0x00)
#define MQ11XX_IC_CONTROL	(MQ11XX_REG_IC + 0x00)
#define MQ11XX_GC_CONTROL	(MQ11XX_REG_GC + 0x00)
#define MQ11XX_GC_WINDOW_STRIDE	(MQ11XX_REG_GC + 0x38)	/* gc0e, bytes/scanline */
#define MQ11XX_FP_OUTPUT_CONTROL (MQ11XX_REG_FP + 0x08)
/*
 * FP.pulse_width_mod_control (fp0f): two independent 8-bit duty fields for the
 * internal two-channel PWM generator of the chip, channel 0 in bits 15:8 and
 * channel 1 in bits 31:24. Only channel 0 has a known consumer, the backlight
 * of the h5xxx, so only one pwm_chip line is exported, see
 * mq11xx_variant.has_internal_pwm.
 */
#define MQ11XX_FP_PULSE_WIDTH_MOD_CONTROL (MQ11XX_REG_FP + 0x3c)
#define MQ11XX_PWM_CHAN0_SHIFT	8
#define MQ11XX_PWM_MAX_LEVEL	0xff
/*
 * The real period of the generator is set by the frc_pattern and
 * pwm_clock_selector entries of mq11xx_h5xxx_init_regs[], and no datasheet
 * states what that is in time units. The driver therefore declares a fixed
 * nominal period of 1 kHz, a conventional backlight PWM frequency, and
 * expresses the duty against it. The ratio that is programmed is exact, the
 * absolute frequency reported with it is a placeholder.
 */
#define MQ11XX_PWM_PERIOD_NS	1000000
/* PCI configuration header: little-endian 16-bit vendor_id, then device_id */
#define MQ11XX_PCI_VENDOR_ID	(MQ11XX_REG_PCI + 0x00)
#define MQ11XX_PCI_DEVICE_ID	(MQ11XX_REG_PCI + 0x02)

#define MQ11XX_CONFIG_CC_MODULE_ENABLE	BIT(0)	/* DC.config_2 */
#define MQ11XX_CONFIG_GE_ENABLE	BIT(0)	/* DC.config_5 */
#define MQ11XX_MIU_ENABLE		BIT(0)	/* MIU.miu_0 */
#define MQ11XX_GC_CONTROL_ENABLE	BIT(0)	/* GC.control */

/*
 * MQ11XX_PCI_VENDOR_MEDIAQ is the value every chip in this family reports;
 * the device IDs are per part and are compared against
 * struct mq11xx_variant.expect_device_id by mq11xx_identify(). The variant
 * is already known from the devicetree compatible, so the read only
 * confirms it.
 */
#define MQ11XX_PCI_VENDOR_MEDIAQ	0x4d51
#define MQ11XX_PCI_DEVICE_1100		0x0100
#define MQ11XX_PCI_DEVICE_1132		0x0120
#define MQ11XX_PCI_DEVICE_1178		0x0178
#define MQ11XX_PCI_DEVICE_1188		0x0188

struct mq11xx_reg {
	u16 offset;
	u32 value;
};

/*
 * Per-board variant descriptor. .expect_device_id is the MediaQ PCI device
 * ID this variant is built for; mq11xx_identify() reads the chip's own PCI
 * header at probe time and warns if it disagrees with both .expect_device_id
 * and .tolerate_device_id.
 */
struct mq11xx_variant {
	const char *name;
	u16 expect_device_id;
	/*
	 * Optional second device ID to accept without warning, using the
	 * *same* .init_regs/.mode as .expect_device_id -- i.e. "treat this
	 * other real chip ID as if it were the expected one". 0 (unused) for
	 * variants with only one known real ID. How well-founded that reuse
	 * is differs per board and is documented at each variant: h2200's is
	 * an unverified measure, h5xxx's rests on the vendor driver
	 * writing the same registers on the 1100 and the 1132.
	 */
	u16 tolerate_device_id;
	const struct drm_display_mode *mode;
	const struct mq11xx_reg *init_regs;
	size_t n_init_regs;
	/*
	 * DC.config_1 (dc01), written by mq11xx_chip_init() to start the
	 * oscillator before the register table can be loaded. It is the
	 * value of the variant's own table and must be identical to its dc01
	 * entry, otherwise the oscillator settle delay runs with the wrong
	 * clock source selected.
	 *
	 * Mandatory: 0 is not an unset marker. A variant that omits it
	 * writes 0, which leaves the oscillator disabled, and
	 * mq11xx_chip_init() then runs its fixed settle delay with no
	 * oscillator running.
	 */
	u32 dc_config_1;
	/*
	 * FP.output_control (fp02), re-applied by mq11xx_output_enable() at
	 * every DPMS-on. It differs between the boards, and must be identical
	 * to the fp02 entry of .init_regs, or DPMS-on reconfigures the pin
	 * outputs of the panel behind the back of mq11xx_load_regs().
	 *
	 * Mandatory, like .dc_config_1: 0 is a real value here, with all
	 * flat-panel output pins disabled, so a variant that omits it gets a
	 * blank panel on the first DPMS-on.
	 */
	u32 fp_output_control;
	/*
	 * DC.config_8 (0x20), written once by mq11xx_chip_init() right after
	 * the CC module is enabled, or not at all when it is 0. Only the h2200
	 * needs it: the vendor display driver of the h5xxx never writes this
	 * register. Unlike .dc_config_1 and .fp_output_control, 0 is safe as
	 * "do not write", as no board wants the register left at zero.
	 */
	u32 dc_config_8;
	/*
	 * Optional second-panel register delta, applied on top of .init_regs
	 * when the panel decision of the board selects it (see
	 * .envee_default_philips and mq11xx_apply_envee_delta()). NULL for
	 * variants with a single panel.
	 */
	const struct mq11xx_reg *envee_regs;
	size_t n_envee_regs;
	/*
	 * Fixed per-board panel decision, consulted when envee_regs is set:
	 * true applies the Philips delta, false keeps the Sharp base table.
	 * The vendor display driver does not read the ENVEE pin either, it
	 * uses a decision that the platform code latched earlier. This is the
	 * fixed stand-in for it, because the pin read at probe has returned
	 * the wrong panel on a unit that runs the Philips table.
	 */
	bool envee_default_philips;
	/*
	 * True if the channel 0 duty field of the PWM register is the
	 * backlight brightness control of the board, so that the variant
	 * exports a pwm_chip, see mq11xx_pwm_chip_register(). On the h2200
	 * the brightness is modulated by the PWM peripheral of the PXA SoC
	 * instead, which is independent of this chip. The h5xxx has no such
	 * wiring and no other brightness path.
	 */
	bool has_internal_pwm;
};

struct mq11xx_device {
	struct drm_device drm;

	const struct mq11xx_variant *variant;

	struct drm_plane primary_plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;

	void __iomem *vram;
	void __iomem *regs;

	struct regulator *vdd;		/* h2200: MQ1178_VDD */
	struct regulator *power;	/* h2200: MQ1178_POWER_ON;
					 * h5xxx: MQ_POWER_ON
					 */
	struct gpio_desc *reset_gpio;	/* h2200 only: MQ1178_RESET_N */
	struct regulator *panel;	/* h5xxx: LCD_EN, the panel's supply
					 * rather than chip power, see
					 * mq11xx_output_enable()
					 */
};

#define to_mq11xx(_dev) container_of(_dev, struct mq11xx_device, drm)

/* ------------------------------------------------------------------ */
/* HP iPAQ h2200 (MediaQ 1178) variant data */

/*
 * 3.5" QVGA TFT, portrait: 240 wide by 320 tall, as the gc08/gc09 window
 * registers in mq11xx_h2200_init_regs[] say (0xef/0x13f). The window
 * stride written at bring-up is hdisplay * 2, so this orientation is load
 * bearing, not cosmetic. Physical size is an approximation, not measured.
 *
 * The timings describe what the register table programs; nothing writes
 * them to the chip. Each GC counter field holds the last index of its
 * span: gc02's horizontal total 0x13b and gc03's vertical total 0x154 give
 * htotal 316 and vtotal 341, gc03's display end 0x13f gives vdisplay 320,
 * and gc04/gc05 put hsync at 306-311 and vsync at 340-341. gc02's
 * horizontal display end (0xf8, 248) runs 8 pixels past the 240-pixel
 * window; that gap is treated as front porch.
 *
 * The chip's source clock frequency is undocumented, so the pixel clock
 * is derived from a measured frame period instead: 316 * 341 pixels in a
 * 17.747 ms frame is 6.072 MHz, about 56.35 Hz. The frame period is the
 * median of 759 intervals between wraps of the gc1d scanline counter
 * (GC+0x74), polled for 20 s on a running h2200, and gc1d's bits[21:12]
 * read back the 320 used as vdisplay. htotal has not been measured, so
 * the split of that clock between line length and line rate rests on the
 * register table alone.
 */
static const struct drm_display_mode mq11xx_mode_h2200 = {
	.type = DRM_MODE_TYPE_DRIVER,
	.clock = 6072,
	.hdisplay = 240,
	.hsync_start = 306,
	.hsync_end = 311,
	.htotal = 316,
	.vdisplay = 320,
	.vsync_start = 340,
	.vsync_end = 341,
	.vtotal = 341,
	.width_mm = 53,
	.height_mm = 71,
};

/*
 * Power-on register table for the MediaQ 1178 as dumped from WinCE, limited
 * to the registers the chip actually has: a few GC/FP/GE registers, the SPI
 * and UHC/I2S blocks and the FP frc_pattern tail at fp70-77 (MediaQ
 * 1100/1132 only) are left out, and DC.config_0/config_3 are handled in
 * mq11xx_chip_init() instead.
 *
 * No datasheet exists for this chip, so the values are opaque; most notably
 * the FP block's frame-rate-control dither pattern (fp10-fp3f) and
 * DC.config_8 = 0x86098609, see .dc_config_8 of the variant.
 */
static const struct mq11xx_reg mq11xx_h2200_init_regs[] = {
	/* DC: config_0 (endian) and config_3 (RO) are not part of this table */
	{ MQ11XX_REG_DC + 0x04, 0x00000042 },	/* dc01 config_1 */
	{ MQ11XX_REG_DC + 0x08, 0x00000001 },	/* dc02 config_2 */
	{ MQ11XX_REG_DC + 0x10, 0x00000004 },	/* dc04 config_4 */
	{ MQ11XX_REG_DC + 0x14, 0x00000009 },	/* dc05 config_5, bit0 already set */

	/* CC: all 5 registers valid for every chip */
	{ MQ11XX_REG_CC + 0x00, 0x00000000 },	/* cc00 cpu_control */
	{ MQ11XX_REG_CC + 0x04, 0x00201010 },	/* cc01 fifo_status */
	{ MQ11XX_REG_CC + 0x08, 0x00000000 },	/* cc02 gpio_control_0 */
	{ MQ11XX_REG_CC + 0x0c, 0x00000000 },	/* cc03 gpio_control_1 */
	{ MQ11XX_REG_CC + 0x10, 0x00000004 },	/* cc04 cpu_test_mode */

	/* MIU: all 7 registers valid for the 1168/1178/1188 chip family */
	{ MQ11XX_REG_MIU + 0x00, 0x00000001 },	/* mm00 miu_0, bit0 already set */
	{ MQ11XX_REG_MIU + 0x04, 0x1b676ca0 },	/* mm01 miu_1 */
	{ MQ11XX_REG_MIU + 0x08, 0x00000000 },	/* mm02 */
	{ MQ11XX_REG_MIU + 0x0c, 0x0000bb4d },	/* mm03 */
	{ MQ11XX_REG_MIU + 0x10, 0x3eee3e45 },	/* mm04 */
	{ MQ11XX_REG_MIU + 0x14, 0x00170211 },	/* mm05 */
	{ MQ11XX_REG_MIU + 0x18, 0x00000000 },	/* mm06 */

	/* GC: gc06/gc07 (counter inits), gc0f (reserved), gc12/gc13 (cursor
	 * fg/bg -- no hardware cursor is used) are skipped per mqInitValid.
	 */
	{ MQ11XX_REG_GC + 0x00, 0x051100c8 },	/* gc00 control, output disabled */
	{ MQ11XX_REG_GC + 0x04, 0x00000200 },	/* gc01 power_sequencing */
	{ MQ11XX_REG_GC + 0x08, 0x00f8013b },	/* gc02 horizontal_display */
	{ MQ11XX_REG_GC + 0x0c, 0x013f0154 },	/* gc03 vertical_display */
	{ MQ11XX_REG_GC + 0x10, 0x01370132 },	/* gc04 horizontal_sync */
	{ MQ11XX_REG_GC + 0x14, 0x01550154 },	/* gc05 vertical_sync */
	{ MQ11XX_REG_GC + 0x20, 0x00ef0000 },	/* gc08 horizontal_window */
	{ MQ11XX_REG_GC + 0x24, 0x013f0000 },	/* gc09 vertical_window */
	{ MQ11XX_REG_GC + 0x28, 0x00f88000 },	/* gc0a reserved_28 */
	{ MQ11XX_REG_GC + 0x2c, 0x006e8037 },	/* gc0b line_clock */
	{ MQ11XX_REG_GC + 0x30, 0x00000000 },	/* gc0c window_start_address */
	{ MQ11XX_REG_GC + 0x34, 0x00457ff2 },	/* gc0d alt_window_start_address */
	{ MQ11XX_REG_GC + 0x38, 0x000001e0 },	/* gc0e window_stride */
	{ MQ11XX_REG_GC + 0x40, 0x010107b4 },	/* gc10 cursor_position */
	{ MQ11XX_REG_GC + 0x44, 0x0a1900d3 },	/* gc11 cursor_start_address */
	{ MQ11XX_REG_GC + 0x50, 0x02fec234 },	/* gc14 */
	{ MQ11XX_REG_GC + 0x54, 0x00000000 },	/* gc15 */
	{ MQ11XX_REG_GC + 0x58, 0x00000000 },	/* gc16 */
	{ MQ11XX_REG_GC + 0x5c, 0x00000000 },	/* gc17 */
	{ MQ11XX_REG_GC + 0x60, 0x00000000 },	/* gc18 */
	{ MQ11XX_REG_GC + 0x64, 0x00000000 },	/* gc19 */
	{ MQ11XX_REG_GC + 0x68, 0x00000000 },	/* gc1a frame_clock_control */

	/* FP: fp09/fp0d/fp0e (dummy/test regs) and fp40-fp77 (unused on this
	 * chip family -- fp70-77's frc_pattern tail is MediaQ 1100/1132
	 * only) are skipped per mqInitValid.
	 */
	{ MQ11XX_REG_FP + 0x00, 0x00006020 },	/* fp00 control */
	{ MQ11XX_REG_FP + 0x04, 0x000c5008 },	/* fp01 pin_control_1 */
	{ MQ11XX_REG_FP + 0x08, 0x9ffdfcfd },	/* fp02 output_control */
	{ MQ11XX_REG_FP + 0x0c, 0x00000000 },	/* fp03 input_control */
	{ MQ11XX_REG_FP + 0x10, 0x00bd0001 },	/* fp04 stn_panel_control */
	{ MQ11XX_REG_FP + 0x14, 0x01010000 },	/* fp05 polarity_control */
	{ MQ11XX_REG_FP + 0x18, 0x80000000 },	/* fp06 pin_output_select_0 */
	{ MQ11XX_REG_FP + 0x1c, 0x00000000 },	/* fp07 pin_output_select_1 */
	{ MQ11XX_REG_FP + 0x20, 0xbc3e5b3f },	/* fp08 pin_output_data */
	{ MQ11XX_REG_FP + 0x28, 0x00000000 },	/* fp0a pin_weak_pull_down */
	{ MQ11XX_REG_FP + 0x2c, 0x00000000 },	/* fp0b additional_pin_out_sel */
	{ MQ11XX_REG_FP + 0x30, 0x00000003 },	/* fp0c dummy1 */
	{ MQ11XX_REG_FP + 0x3c, 0x25180000 },	/* fp0f pulse_width_mod_control */
	/* fp10-fp3f: frc_pattern[] dither table, opaque WinCE-dumped data */
	{ MQ11XX_REG_FP + 0x40, 0x553fde9e }, { MQ11XX_REG_FP + 0x44, 0xcfb7f3fb },
	{ MQ11XX_REG_FP + 0x48, 0x5ba7df29 }, { MQ11XX_REG_FP + 0x4c, 0xf7787cd6 },
	{ MQ11XX_REG_FP + 0x50, 0xb6f7c3b9 }, { MQ11XX_REG_FP + 0x54, 0x757fffdd },
	{ MQ11XX_REG_FP + 0x58, 0x3de535d0 }, { MQ11XX_REG_FP + 0x5c, 0xbdf8231f },
	{ MQ11XX_REG_FP + 0x60, 0xfeaee16d }, { MQ11XX_REG_FP + 0x64, 0x9c78ceff },
	{ MQ11XX_REG_FP + 0x68, 0xc8ffefac }, { MQ11XX_REG_FP + 0x6c, 0xfdfb3bff },
	{ MQ11XX_REG_FP + 0x70, 0x7f2dbdf9 }, { MQ11XX_REG_FP + 0x74, 0xbf7536ea },
	{ MQ11XX_REG_FP + 0x78, 0xfbdb7d65 }, { MQ11XX_REG_FP + 0x7c, 0xb1dabdee },
	{ MQ11XX_REG_FP + 0x80, 0xfe7a5eef }, { MQ11XX_REG_FP + 0x84, 0x77ffefe7 },
	{ MQ11XX_REG_FP + 0x88, 0xdffea7ff }, { MQ11XX_REG_FP + 0x8c, 0xe7faed77 },
	{ MQ11XX_REG_FP + 0x90, 0xdffef735 }, { MQ11XX_REG_FP + 0x94, 0x53feefff },
	{ MQ11XX_REG_FP + 0x98, 0x5f7bfdff }, { MQ11XX_REG_FP + 0x9c, 0x7dbefefc },
	{ MQ11XX_REG_FP + 0xa0, 0x8dfae4f6 }, { MQ11XX_REG_FP + 0xa4, 0xfffbf7db },
	{ MQ11XX_REG_FP + 0xa8, 0xfaffdff7 }, { MQ11XX_REG_FP + 0xac, 0xdffeb6f7 },
	{ MQ11XX_REG_FP + 0xb0, 0xe736eeff }, { MQ11XX_REG_FP + 0xb4, 0x7ef7ed7b },
	{ MQ11XX_REG_FP + 0xb8, 0xdf1ceef7 }, { MQ11XX_REG_FP + 0xbc, 0x7bdeefb3 },
	{ MQ11XX_REG_FP + 0xc0, 0x5ffdeff7 }, { MQ11XX_REG_FP + 0xc4, 0xdfb9e7ff },
	{ MQ11XX_REG_FP + 0xc8, 0xdbf8c7f7 }, { MQ11XX_REG_FP + 0xcc, 0x5b5bf5ff },
	{ MQ11XX_REG_FP + 0xd0, 0xfe696cdf }, { MQ11XX_REG_FP + 0xd4, 0x6efafff7 },
	{ MQ11XX_REG_FP + 0xd8, 0x2cfb7fff }, { MQ11XX_REG_FP + 0xdc, 0x7ffa5cf5 },
	{ MQ11XX_REG_FP + 0xe0, 0xcbf30000 }, { MQ11XX_REG_FP + 0xe4, 0x0000e6d3 },
	{ MQ11XX_REG_FP + 0xe8, 0x0000f2b3 }, { MQ11XX_REG_FP + 0xec, 0x0000effe },
	{ MQ11XX_REG_FP + 0xf0, 0xff5efffd }, { MQ11XX_REG_FP + 0xf4, 0x7ffd77b7 },
	{ MQ11XX_REG_FP + 0xf8, 0x6ff9fef7 }, { MQ11XX_REG_FP + 0xfc, 0x7ffafff7 },

	/* GE: only ge0a/ge0b/ge11 are valid for the 1178; the rest are
	 * either unconditionally "not set" or gated to other chip variants.
	 */
	{ MQ11XX_REG_GE + 0x28, 0x00000000 },	/* ge0a */
	{ MQ11XX_REG_GE + 0x2c, 0x6b6ffeae },	/* ge0b */
	{ MQ11XX_REG_GE + 0x44, 0x00000055 },	/* ge11 */
};

static const struct mq11xx_variant mq11xx_variant_h2200 = {
	.name = "h2200",
	/*
	 * An h2210 has been seen to report device ID 0x0188 (a MediaQ 1188)
	 * instead of 0x0178, so h2200 units evidently shipped with either
	 * chip. The init table below was dumped from 1178 silicon and its
	 * suitability for the 1188 has not been verified, so a detected 1188
	 * is accepted through .tolerate_device_id with a notice rather than
	 * by widening the expected ID: the table is reused as is, and the
	 * notice is the only sign that it may not suit the chip.
	 */
	.expect_device_id = MQ11XX_PCI_DEVICE_1178,
	.tolerate_device_id = MQ11XX_PCI_DEVICE_1188,
	.mode = &mq11xx_mode_h2200,
	.init_regs = mq11xx_h2200_init_regs,
	.n_init_regs = ARRAY_SIZE(mq11xx_h2200_init_regs),
	/* The dc01 entry of mq11xx_h2200_init_regs[]. */
	.dc_config_1 = 0x00000042,
	/* The fp02 entry of mq11xx_h2200_init_regs[]. */
	.fp_output_control = 0x9ffdfcfd,
	/* Needed for MediaQ 1178/88 on h2200; opaque, see the file comment. */
	.dc_config_8 = 0x86098609,
};

/* ------------------------------------------------------------------ */
/* HP iPAQ h5xxx (MediaQ 1132; see the variant comment below) variant data */

static const struct drm_display_mode mq11xx_mode_h5xxx = {
	/*
	 * Portrait, matching the GC08/GC09 window registers below (240 wide,
	 * 0xef = 239 and 0x13f = 319). The 57 x 77 mm size of the 3.8" QVGA
	 * TFT is the figure given for the whole h3xxx and h5xxx family, not
	 * measured on these units.
	 */
	DRM_SIMPLE_MODE(240, 320, 57, 77),
};

/*
 * Power-on register table for the Sharp panel, as dumped from WinCE,
 * restricted to the registers that the chip family actually takes: a
 * register that the 1100 and 1132 do not take is never written, whatever
 * the dumped table holds for it. They take the fp70-77 frc_pattern tail,
 * which the h2200 table lacks, and not fp20-2f; gc06, gc07, gc0f, gc12,
 * gc13 and fp40-6f are not taken by any chip of the family.
 *
 * Filtering on the registers that the dump leaves at zero is not the same
 * thing, and wrote 65 registers that are never written on this chip.
 */
static const struct mq11xx_reg mq11xx_h5xxx_init_regs[] = {
	/* DC: config_0 (endianness) and config_3 (read-only) are skipped, as above */
	{ MQ11XX_REG_DC + 0x004, 0x00000403 },	/* dc01 */
	{ MQ11XX_REG_DC + 0x008, 0x00000001 },	/* dc02 */
	{ MQ11XX_REG_DC + 0x010, 0x00000004 },	/* dc04 */
	{ MQ11XX_REG_DC + 0x014, 0x00000003 },	/* dc05 */
	/* CC: all 5 registers set for this chip. */
	{ MQ11XX_REG_CC + 0x000, 0x00000000 },	/* cc00 */
	{ MQ11XX_REG_CC + 0x004, 0x00001010 },	/* cc01 */
	{ MQ11XX_REG_CC + 0x008, 0x000002a2 },	/* cc02 */
	{ MQ11XX_REG_CC + 0x00c, 0x00000000 },	/* cc03 */
	{ MQ11XX_REG_CC + 0x010, 0x00000004 },	/* cc04 */
	/* MIU: only 5 registers are valid for the 1100/1132 family */
	{ MQ11XX_REG_MIU + 0x000, 0x00000001 },	/* mm00 */
	{ MQ11XX_REG_MIU + 0x004, 0x1b676ca8 },	/* mm01 */
	{ MQ11XX_REG_MIU + 0x008, 0x00000000 },	/* mm02 */
	{ MQ11XX_REG_MIU + 0x00c, 0x00001479 },	/* mm03 */
	{ MQ11XX_REG_MIU + 0x010, 0x6bfc2d76 },	/* mm04 */
	/* GC */
	{ MQ11XX_REG_GC + 0x000, 0x080100c8 },	/* gc00 */
	{ MQ11XX_REG_GC + 0x004, 0x00000000 },	/* gc01 */
	{ MQ11XX_REG_GC + 0x008, 0x00f0011a },	/* gc02 */
	{ MQ11XX_REG_GC + 0x00c, 0x013f015f },	/* gc03 */
	{ MQ11XX_REG_GC + 0x010, 0x011100fa },	/* gc04 */
	{ MQ11XX_REG_GC + 0x014, 0x015a0158 },	/* gc05 */
	/* gc06/gc07 (counter inits), gc0f, gc12/gc13: masked out for every
	 * chip, so they are absent here as in the h2200 table above.
	 */
	{ MQ11XX_REG_GC + 0x020, 0x00ef0000 },	/* gc08 */
	{ MQ11XX_REG_GC + 0x024, 0x013f0000 },	/* gc09 */
	{ MQ11XX_REG_GC + 0x028, 0x00000000 },	/* gc0a */
	{ MQ11XX_REG_GC + 0x02c, 0x011700f2 },	/* gc0b */
	{ MQ11XX_REG_GC + 0x030, 0x00000000 },	/* gc0c */
	{ MQ11XX_REG_GC + 0x034, 0x00066373 },	/* gc0d */
	{ MQ11XX_REG_GC + 0x038, 0x000001e0 },	/* gc0e */
	{ MQ11XX_REG_GC + 0x040, 0x02ff07ff },	/* gc10 */
	{ MQ11XX_REG_GC + 0x044, 0x000000ff },	/* gc11 */
	{ MQ11XX_REG_GC + 0x050, 0x00000000 },	/* gc14 */
	{ MQ11XX_REG_GC + 0x054, 0x00000000 },	/* gc15 */
	{ MQ11XX_REG_GC + 0x058, 0x00000000 },	/* gc16 */
	{ MQ11XX_REG_GC + 0x05c, 0x00000000 },	/* gc17 */
	{ MQ11XX_REG_GC + 0x060, 0x00000000 },	/* gc18 */
	{ MQ11XX_REG_GC + 0x064, 0x00000000 },	/* gc19 */
	{ MQ11XX_REG_GC + 0x068, 0x00000000 },	/* gc1a */
	/* FP */
	{ MQ11XX_REG_FP + 0x000, 0x00006120 },	/* fp00 */
	{ MQ11XX_REG_FP + 0x004, 0x003d5008 },	/* fp01 */
	{ MQ11XX_REG_FP + 0x008, 0xfffcfcfd },	/* fp02 */
	{ MQ11XX_REG_FP + 0x00c, 0x00000002 },	/* fp03 */
	{ MQ11XX_REG_FP + 0x010, 0x80bd0001 },	/* fp04 */
	{ MQ11XX_REG_FP + 0x014, 0xc9000000 },	/* fp05 */
	{ MQ11XX_REG_FP + 0x018, 0x80000000 },	/* fp06 */
	{ MQ11XX_REG_FP + 0x01c, 0x00040005 },	/* fp07 */
	{ MQ11XX_REG_FP + 0x020, 0x00040004 },	/* fp08 */
	{ MQ11XX_REG_FP + 0x028, 0x00000000 },	/* fp0a */
	{ MQ11XX_REG_FP + 0x02c, 0x00000000 },	/* fp0b */
	{ MQ11XX_REG_FP + 0x03c, 0x00000120 },	/* fp0f */
	{ MQ11XX_REG_FP + 0x040, 0x2645a216 },	/* fp10 */
	{ MQ11XX_REG_FP + 0x044, 0x1941d898 },	/* fp11 */
	{ MQ11XX_REG_FP + 0x048, 0xc2f5b84c },	/* fp12 */
	{ MQ11XX_REG_FP + 0x04c, 0x411585dd },	/* fp13 */
	{ MQ11XX_REG_FP + 0x050, 0xad1d0a85 },	/* fp14 */
	{ MQ11XX_REG_FP + 0x054, 0xd7928b26 },	/* fp15 */
	{ MQ11XX_REG_FP + 0x058, 0x4848e722 },	/* fp16 */
	{ MQ11XX_REG_FP + 0x05c, 0x696d0932 },	/* fp17 */
	{ MQ11XX_REG_FP + 0x060, 0xf2d20075 },	/* fp18 */
	{ MQ11XX_REG_FP + 0x064, 0xd547235e },	/* fp19 */
	{ MQ11XX_REG_FP + 0x068, 0x310304f6 },	/* fp1a */
	{ MQ11XX_REG_FP + 0x06c, 0x3cef4474 },	/* fp1b */
	{ MQ11XX_REG_FP + 0x070, 0x22a3d351 },	/* fp1c */
	{ MQ11XX_REG_FP + 0x074, 0x01825700 },	/* fp1d */
	{ MQ11XX_REG_FP + 0x078, 0xb7d793d1 },	/* fp1e */
	{ MQ11XX_REG_FP + 0x07c, 0x10906408 },	/* fp1f */
	/* fp20-fp2f: 16 registers that only the 1168, 1178 and 1188 take,
	 * deliberately absent here.
	 */
	{ MQ11XX_REG_FP + 0x0c0, 0x7ba9b45b },	/* fp30 */
	{ MQ11XX_REG_FP + 0x0c4, 0xb56738c3 },	/* fp31 */
	{ MQ11XX_REG_FP + 0x0c8, 0xde47a2ba },	/* fp32 */
	{ MQ11XX_REG_FP + 0x0cc, 0x88241cf0 },	/* fp33 */
	{ MQ11XX_REG_FP + 0x0d0, 0x35c661b2 },	/* fp34 */
	{ MQ11XX_REG_FP + 0x0d4, 0x7ae04b8d },	/* fp35 */
	{ MQ11XX_REG_FP + 0x0d8, 0x7acb010d },	/* fp36 */
	{ MQ11XX_REG_FP + 0x0dc, 0xa216c122 },	/* fp37 */
	{ MQ11XX_REG_FP + 0x0e0, 0xd64fb56a },	/* fp38 */
	{ MQ11XX_REG_FP + 0x0e4, 0xe2b4dea9 },	/* fp39 */
	{ MQ11XX_REG_FP + 0x0e8, 0x4267c3f7 },	/* fp3a */
	{ MQ11XX_REG_FP + 0x0ec, 0x9b75c799 },	/* fp3b */
	{ MQ11XX_REG_FP + 0x0f0, 0x01aee179 },	/* fp3c */
	{ MQ11XX_REG_FP + 0x0f4, 0x586fd0a8 },	/* fp3d */
	{ MQ11XX_REG_FP + 0x0f8, 0x4eeb130e },	/* fp3e */
	{ MQ11XX_REG_FP + 0x0fc, 0x42604a40 },	/* fp3f */
	/* fp40-fp6f: 48 registers that are masked out for every chip of the
	 * family, deliberately absent here.
	 */
	{ MQ11XX_REG_FP + 0x1c0, 0x00000405 },	/* fp70 */
	{ MQ11XX_REG_FP + 0x1c4, 0xced6a94e },	/* fp71 */
	{ MQ11XX_REG_FP + 0x1c8, 0x00000002 },	/* fp72 */
	{ MQ11XX_REG_FP + 0x1cc, 0x00000000 },	/* fp73 */
	{ MQ11XX_REG_FP + 0x1d0, 0x00000000 },	/* fp74 */
	{ MQ11XX_REG_FP + 0x1d4, 0x00000000 },	/* fp75 */
	{ MQ11XX_REG_FP + 0x1d8, 0x95f9c859 },	/* fp76 */
	{ MQ11XX_REG_FP + 0x1dc, 0x30388dc9 },	/* fp77 */
	/* GE: only ge0a/ge0b are set for this chip. */
	{ MQ11XX_REG_GE + 0x028, 0x40000280 },	/* ge0a */
	{ MQ11XX_REG_GE + 0x02c, 0x00000000 },	/* ge0b */
};

/*
 * Register values of the Philips panel that differ from the Sharp table
 * above (cc02, gc00, gc02-gc05, gc0b, fp01, fp05 and fp06), applied on top
 * of it when the board selects the Philips panel
 * (mq11xx_apply_envee_delta()). fp08 is 0x00040004 for both panels and is
 * therefore not part of the delta.
 */
static const struct mq11xx_reg mq11xx_h5xxx_philips_regs[] = {
	{ MQ11XX_REG_CC + 0x008, 0x00000a22 },	/* cc02, Sharp had 0x000002a2 */
	{ MQ11XX_REG_GC + 0x000, 0x051100c8 },	/* gc00, Sharp had 0x080100c8 */
	{ MQ11XX_REG_GC + 0x008, 0x00f8013b },	/* gc02, Sharp had 0x00f0011a */
	{ MQ11XX_REG_GC + 0x00c, 0x013f0150 },	/* gc03, Sharp had 0x013f015f */
	{ MQ11XX_REG_GC + 0x010, 0x01370132 },	/* gc04, Sharp had 0x011100fa */
	{ MQ11XX_REG_GC + 0x014, 0x01510150 },	/* gc05, Sharp had 0x015a0158 */
	{ MQ11XX_REG_GC + 0x02c, 0x006e0037 },	/* gc0b, Sharp had 0x011700f2 */
	{ MQ11XX_REG_FP + 0x004, 0x00bcc02c },	/* fp01, Sharp had 0x003d5008 */
	{ MQ11XX_REG_FP + 0x014, 0xc3000000 },	/* fp05, Sharp had 0xc9000000 */
	{ MQ11XX_REG_FP + 0x018, 0xa0000000 },	/* fp06, Sharp had 0x80000000 */
};

static const struct mq11xx_variant mq11xx_variant_h5xxx = {
	.name = "h5xxx",
	/*
	 * The PCI header of a real h5550 reads vendor 0x4d51, device 0x0120,
	 * which is a MediaQ 1132. The boot loader of this family also detects
	 * the machine by the PCI ID of a 1132, while the vendor tables and
	 * Kconfig symbol call it a 1100.
	 *
	 * The register data does not depend on which of the two it is. The
	 * vendor driver gates its table writes on per-chip register masks,
	 * and in the six blocks that it loads (DC, CC, MIU, GC, FP and GE) no
	 * mask separates the 1100 from the 1132. The only masks that do are
	 * in its SPI block, which it never loads. It therefore writes the
	 * same registers on both and one table is correct for either, which
	 * is also why a detected 1100 is tolerated silently.
	 *
	 * The same unit is a Philips panel: as the previous firmware had
	 * left them, FP.pin_control_1 read 0x00bcc02c and FP.polarity_control
	 * read 0xc3000000, which are the fp01 and fp05 entries of
	 * mq11xx_h5xxx_philips_regs[] and not the Sharp values, and fp00,
	 * fp02, fp03 and fp04 read as the shared base table. fp0f differed
	 * only in the backlight duty bits, which are live state. A read of
	 * the ENVEE pin at probe reported a Sharp panel on that unit, though,
	 * and that would program the Sharp CRT and timing registers onto the
	 * Philips panel. The variant therefore uses a fixed decision,
	 * .envee_default_philips below, as the vendor display driver uses a
	 * cached one and not the pin. The Sharp table has not been compared
	 * with hardware.
	 */
	.expect_device_id = MQ11XX_PCI_DEVICE_1132,
	.tolerate_device_id = MQ11XX_PCI_DEVICE_1100,
	.mode = &mq11xx_mode_h5xxx,
	.init_regs = mq11xx_h5xxx_init_regs,
	.n_init_regs = ARRAY_SIZE(mq11xx_h5xxx_init_regs),
	/* The dc01 entry of mq11xx_h5xxx_init_regs[]. The low two bits select
	 * the internal oscillator; the vendor configuration table also sets
	 * bit 10.
	 */
	.dc_config_1 = 0x00000403,
	/* The fp02 entry of mq11xx_h5xxx_init_regs[], which the panel delta
	 * does not change.
	 */
	.fp_output_control = 0xfffcfcfd,
	.envee_regs = mq11xx_h5xxx_philips_regs,
	.n_envee_regs = ARRAY_SIZE(mq11xx_h5xxx_philips_regs),
	/*
	 * The only real unit is a Philips one. A future h5xxx model with a
	 * Sharp panel would need its own variant.
	 */
	.envee_default_philips = true,
	/* The only backlight brightness path of the h5xxx */
	.has_internal_pwm = true,
};

static const struct of_device_id mq11xx_of_match[] = {
	{ .compatible = "mediaq,mq1132", .data = &mq11xx_variant_h5xxx },
	{ .compatible = "mediaq,mq1178", .data = &mq11xx_variant_h2200 },
	{ }
};
MODULE_DEVICE_TABLE(of, mq11xx_of_match);

/* ------------------------------------------------------------------ */

/*
 * Verify-and-retry register write: up to 10 tries, 1 ms apart, reading back
 * after each write to confirm it latched. The chip is not necessarily ready
 * to accept every write right after the reset/power sequence, and an
 * unverified write would silently produce a corrupt display.
 */
static int mq11xx_write_verify(struct mq11xx_device *mq, u16 offset, u32 value)
{
	unsigned int tries;

	for (tries = 0; tries < 10; tries++) {
		writel(value, mq->regs + offset);
		if (readl(mq->regs + offset) == value)
			return 0;
		mdelay(1);
	}

	return -ENODEV;
}

static int mq11xx_load_regs(struct mq11xx_device *mq)
{
	const struct mq11xx_reg *regs = mq->variant->init_regs;
	unsigned int i;
	int ret;

	for (i = 0; i < mq->variant->n_init_regs; i++) {
		ret = mq11xx_write_verify(mq, regs[i].offset, regs[i].value);
		if (ret) {
			drm_err(&mq->drm,
				"register 0x%04x = 0x%08x did not stick\n",
				regs[i].offset, regs[i].value);
			return ret;
		}
	}

	return 0;
}

/*
 * Second-panel selection, for variants that carry an ENVEE-gated register
 * delta (mq->variant->envee_regs, only h5xxx so far). A no-op for the
 * others.
 *
 * mq->variant->init_regs always holds the Sharp values, so when the board's
 * decision, mq->variant->envee_default_philips, is Sharp nothing is done,
 * and when it is Philips the differing registers are overwritten with the
 * Philips values on top of the table that mq11xx_load_regs() just loaded.
 * The ENVEE pin is deliberately not read: it reported a Sharp panel on a
 * unit that runs the Philips table, which would program Sharp CRT and
 * timing registers onto Philips silicon, and the vendor display driver does
 * not read it either.
 *
 * The vendor driver chose the whole table before loading anything. Loading
 * the Sharp table first and correcting it afterwards means that a Philips
 * panel is briefly configured with Sharp timings. That is believed harmless
 * as scanout is off and LCD_EN is low until mq11xx_output_enable(), but that
 * is reasoning and not a measurement.
 */
static int mq11xx_apply_envee_delta(struct mq11xx_device *mq)
{
	const struct mq11xx_reg *regs = mq->variant->envee_regs;
	unsigned int i;
	int ret;

	if (!regs)
		return 0;

	if (!mq->variant->envee_default_philips) {
		drm_info(&mq->drm, "%s: Sharp panel timing\n", mq->variant->name);
		return 0;
	}

	drm_info(&mq->drm, "%s: Philips panel timing\n", mq->variant->name);

	for (i = 0; i < mq->variant->n_envee_regs; i++) {
		ret = mq11xx_write_verify(mq, regs[i].offset, regs[i].value);
		if (ret) {
			drm_err(&mq->drm,
				"Philips override 0x%04x = 0x%08x did not stick\n",
				regs[i].offset, regs[i].value);
			return ret;
		}
	}

	return 0;
}

/*
 * Read the chip's PCI-compatible configuration header and log it, warning
 * on a mismatch with this variant's expectation. This is what tells a
 * MediaQ 1100 from a 1132 on h5xxx.
 * On the h2200 it found a real MQ1188 where an MQ1178 was expected, and on
 * the h5xxx it showed that the chip is a 1132 and not the 1100 of the vendor
 * tables.
 *
 * A mismatch is deliberately not fatal: a wrong or absent header should
 * leave a loud kernel message rather than a probe failure that silently
 * leaves the screen dark.
 */
static void mq11xx_identify(struct mq11xx_device *mq)
{
	u16 vendor = readw(mq->regs + MQ11XX_PCI_VENDOR_ID);
	u16 device = readw(mq->regs + MQ11XX_PCI_DEVICE_ID);

	drm_info(&mq->drm, "%s: PCI header reports vendor 0x%04x device 0x%04x\n",
		 mq->variant->name, vendor, device);

	if (vendor != MQ11XX_PCI_VENDOR_MEDIAQ)
		drm_warn(&mq->drm,
			 "unexpected PCI vendor 0x%04x (expected MediaQ 0x%04x) -- is this really a MediaQ chip?\n",
			 vendor, MQ11XX_PCI_VENDOR_MEDIAQ);

	if (device != mq->variant->expect_device_id &&
	    device != mq->variant->tolerate_device_id)
		drm_warn(&mq->drm,
			 "PCI device 0x%04x does not match the %s variant's expected 0x%04x\n",
			 device, mq->variant->name, mq->variant->expect_device_id);
	else if (device == mq->variant->tolerate_device_id)
		drm_info(&mq->drm,
			 "PCI device 0x%04x is a tolerated alternate for the %s variant, using the init_regs of 0x%04x\n",
			 device, mq->variant->name, mq->variant->expect_device_id);
}

/*
 * Supply power-up sequence; see the file comment for which parts of the
 * ordering and timing are inferred. The settle time of each supply is its
 * own startup delay, which regulator_enable() waits out before returning,
 * so reset is only released once the power supply has had its 10 ms.
 */
static int mq11xx_power_up(struct mq11xx_device *mq)
{
	int ret;

	ret = regulator_enable(mq->vdd);
	if (ret)
		return ret;

	ret = regulator_enable(mq->power);
	if (ret) {
		regulator_disable(mq->vdd);
		return ret;
	}

	gpiod_set_value(mq->reset_gpio, 0);	/* active low: release */
	usleep_range(1000, 2000);

	return 0;
}

static void mq11xx_power_down(struct mq11xx_device *mq)
{
	gpiod_set_value(mq->reset_gpio, 1);	/* active low: assert */
	regulator_disable(mq->power);
	regulator_disable(mq->vdd);
}

static void mq11xx_power_down_action(void *data)
{
	mq11xx_power_down(data);
}

/*
 * One-time chip bring-up: endianness quirk, oscillator enable (100 ms; 30 ms
 * is not enough), CC-module power (10 ms), the DC.config_8 value, MIU
 * enable, interrupt controller mask, the PCI-header identity read, then the
 * full register table load. Scanout stays off until the first atomic_enable
 * (see mq11xx_output_enable()).
 *
 * The caller powers the chip up and ties mq11xx_power_down_action() to devm
 * before calling in here, because the register load can fail and the chip
 * must not be left powered with nothing to unwind it.
 */
static int mq11xx_chip_init(struct mq11xx_device *mq)
{
	u32 val;
	int ret;

	/*
	 * Endianness quirk: the low 2 bits of DC.config_0 (little endian, no
	 * byte swapping, the only combination used) are replicated across
	 * all 16 bits of the half-word the chip latches at this point.
	 */
	writew(0x5555, mq->regs + MQ11XX_DC_CONFIG_0);

	/*
	 * Enable the oscillator; wait for it to run. The value is
	 * per-variant: only the low two bits select the source, and the
	 * 0x42 of the h2200 selects none of the three defined sources on the
	 * h5xxx. The register table reloads dc01 shortly after, so the value
	 * only matters for the settle delay.
	 */
	writel(mq->variant->dc_config_1, mq->regs + MQ11XX_DC_CONFIG_1);
	mdelay(100);

	/* Power the CPU-interface configuration module. */
	writel(MQ11XX_CONFIG_CC_MODULE_ENABLE, mq->regs + MQ11XX_DC_CONFIG_2);
	mdelay(10);

	/* Needed for MediaQ 1178/88 on h2200, so per variant; see .dc_config_8. */
	if (mq->variant->dc_config_8)
		writel(mq->variant->dc_config_8, mq->regs + MQ11XX_DC_CONFIG_8);

	val = readl(mq->regs + MQ11XX_MIU_0);
	writel(val | MQ11XX_MIU_ENABLE, mq->regs + MQ11XX_MIU_0);

	/* Interrupt controller stays fully masked: no vblank support. */
	writel(0, mq->regs + MQ11XX_IC_CONTROL);

	mq11xx_identify(mq);

	ret = mq11xx_load_regs(mq);
	if (ret)
		return ret;

	/*
	 * The init table carries a window stride left over from the WinCE
	 * configuration it was dumped from, which never matches the
	 * stride the CPU side uses (fb->pitches[0] of the RGB565 mode).
	 * Overwrite it after the table has loaded so it always wins.
	 */
	ret = mq11xx_write_verify(mq, MQ11XX_GC_WINDOW_STRIDE,
				  mq->variant->mode->hdisplay * 2);
	if (ret) {
		drm_err(&mq->drm, "window_stride fixup did not stick\n");
		return ret;
	}

	return mq11xx_apply_envee_delta(mq);
}

static void mq11xx_output_enable(struct mq11xx_device *mq)
{
	u32 val;
	int ret;

	val = readl(mq->regs + MQ11XX_GC_CONTROL);
	writel(val | MQ11XX_GC_CONTROL_ENABLE, mq->regs + MQ11XX_GC_CONTROL);

	val = readl(mq->regs + MQ11XX_DC_CONFIG_5);
	writel(val | MQ11XX_CONFIG_GE_ENABLE, mq->regs + MQ11XX_DC_CONFIG_5);

	/*
	 * Power the panel's own supply rail on before driving the flat-panel
	 * interface pins below. Driving the data and sync lines of a TFT panel
	 * before its supply is stable can leave its row and column drivers
	 * latched onto garbage, which shows as corruption in the first scanned
	 * rows. mq11xx_output_disable() drops it first on the way down as
	 * well.
	 */
	ret = regulator_enable(mq->panel);
	if (ret) {
		drm_err(&mq->drm, "failed to enable panel supply (%d), display stays off\n",
			ret);
		return;
	}

	/*
	 * Re-enable the flat-panel pin outputs with the value of this
	 * variant, which its register table already loaded. A constant would
	 * overwrite fp02 on the other board at every DPMS-on. The counterpart
	 * in mq11xx_output_disable() is a plain 0 on every chip, so that one
	 * stays a literal.
	 */
	writel(mq->variant->fp_output_control,
	       mq->regs + MQ11XX_FP_OUTPUT_CONTROL);
}

static void mq11xx_output_disable(struct mq11xx_device *mq)
{
	u32 val;

	/* Blank the panel before tearing down scanout -- see the mirrored
	 * comment in mq11xx_output_enable() above.
	 */
	regulator_disable(mq->panel);

	val = readl(mq->regs + MQ11XX_GC_CONTROL);
	writel(val & ~MQ11XX_GC_CONTROL_ENABLE, mq->regs + MQ11XX_GC_CONTROL);

	val = readl(mq->regs + MQ11XX_DC_CONFIG_5);
	writel(val & ~MQ11XX_CONFIG_GE_ENABLE, mq->regs + MQ11XX_DC_CONFIG_5);

	writel(0, mq->regs + MQ11XX_FP_OUTPUT_CONTROL);
}

/*
 * Backlight brightness of the h5xxx. The duty levels are 8 bits and inverted:
 * the duty count of channel 0 of the chip runs opposite to the brightness on
 * the screen, so the darkest level is the highest count.
 *
 * The period is fixed in hardware, MQ11XX_PWM_PERIOD_NS, so a request for a
 * shorter one is refused and a longer one rounds down to it, the usual
 * contract for a PWM whose only programmable field is the duty.
 */
static int mq11xx_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    const struct pwm_state *state)
{
	struct mq11xx_device *mq = dev_get_drvdata(pwmchip_parent(chip));
	unsigned int level;
	u64 duty;
	u32 val;
	int idx;

	if (state->polarity != PWM_POLARITY_NORMAL)
		return -EINVAL;
	if (state->period < MQ11XX_PWM_PERIOD_NS)
		return -EINVAL;

	duty = min_t(u64, state->duty_cycle, MQ11XX_PWM_PERIOD_NS);
	if (!state->enabled)
		duty = 0;

	/* Darkest is the highest count, see the comment above. */
	level = MQ11XX_PWM_MAX_LEVEL -
		div_u64(duty * MQ11XX_PWM_MAX_LEVEL, MQ11XX_PWM_PERIOD_NS);

	if (!drm_dev_enter(&mq->drm, &idx))
		return -ENODEV;

	val = readl(mq->regs + MQ11XX_FP_PULSE_WIDTH_MOD_CONTROL);
	val &= ~(MQ11XX_PWM_MAX_LEVEL << MQ11XX_PWM_CHAN0_SHIFT);
	val |= level << MQ11XX_PWM_CHAN0_SHIFT;
	writel(val, mq->regs + MQ11XX_FP_PULSE_WIDTH_MOD_CONTROL);

	drm_dev_exit(idx);

	return 0;
}

/*
 * fp0f holds no enable bit of its own, the generator always runs, so the
 * darkest count is reported back as disabled, the same compromise that
 * pwm-iqs620a makes for its duty only device. Rounding the duty up makes
 * .apply() idempotent over the read-back state: .apply() rounds the same
 * conversion down, so an unrounded read would come back one count darker
 * every time the core re-applied it.
 */
static int mq11xx_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				struct pwm_state *state)
{
	struct mq11xx_device *mq = dev_get_drvdata(pwmchip_parent(chip));
	unsigned int level, duty_level;
	int idx;

	if (!drm_dev_enter(&mq->drm, &idx))
		return -ENODEV;

	level = (readl(mq->regs + MQ11XX_FP_PULSE_WIDTH_MOD_CONTROL) >>
		 MQ11XX_PWM_CHAN0_SHIFT) & MQ11XX_PWM_MAX_LEVEL;

	drm_dev_exit(idx);

	duty_level = MQ11XX_PWM_MAX_LEVEL - level;

	state->period = MQ11XX_PWM_PERIOD_NS;
	state->polarity = PWM_POLARITY_NORMAL;
	state->enabled = duty_level > 0;
	state->duty_cycle = DIV_ROUND_UP_ULL((u64)duty_level * MQ11XX_PWM_PERIOD_NS,
					     MQ11XX_PWM_MAX_LEVEL);

	return 0;
}

static const struct pwm_ops mq11xx_pwm_ops = {
	.apply = mq11xx_pwm_apply,
	.get_state = mq11xx_pwm_get_state,
};

/*
 * Best effort, not fatal to probe(): the display works with the brightness
 * that the fp0f entry of mq11xx_h5xxx_init_regs[] left in the register, so a
 * failure to register the pwm_chip, which realistically means memory
 * exhaustion, must not turn a working display into a probe failure. The
 * platform device has no devicetree node yet, so no consumer can be wired
 * to it from the devicetree either.
 */
static void mq11xx_pwm_chip_register(struct mq11xx_device *mq, struct device *dev)
{
	struct pwm_chip *chip;
	int ret;

	chip = devm_pwmchip_alloc(dev, 1, 0);
	if (IS_ERR(chip)) {
		drm_warn(&mq->drm, "devm_pwmchip_alloc failed (%ld), backlight brightness stays fixed\n",
			 PTR_ERR(chip));
		return;
	}

	chip->ops = &mq11xx_pwm_ops;

	ret = devm_pwmchip_add(dev, chip);
	if (ret)
		drm_warn(&mq->drm, "devm_pwmchip_add failed (%d), backlight brightness stays fixed\n",
			 ret);
}

/* ------------------------------------------------------------------ */
/* Shadow-plane atomic pipeline, structurally identical to cirrus-qemu.c's */

static const u32 mq11xx_primary_plane_formats[] = {
	DRM_FORMAT_RGB565,
};

static const u64 mq11xx_primary_plane_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

static int mq11xx_primary_plane_helper_atomic_check(struct drm_plane *plane,
						    struct drm_atomic_commit *state)
{
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc *new_crtc = new_plane_state->crtc;
	struct drm_crtc_state *new_crtc_state = NULL;

	if (new_crtc)
		new_crtc_state = drm_atomic_get_new_crtc_state(state, new_crtc);

	return drm_atomic_helper_check_plane_state(new_plane_state, new_crtc_state,
						    DRM_PLANE_NO_SCALING,
						    DRM_PLANE_NO_SCALING,
						    false, false);
}

static void mq11xx_primary_plane_helper_atomic_update(struct drm_plane *plane,
						      struct drm_atomic_commit *state)
{
	struct mq11xx_device *mq = to_mq11xx(plane->dev);
	struct drm_plane_state *plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_shadow_plane_state *shadow_plane_state = to_drm_shadow_plane_state(plane_state);
	struct drm_framebuffer *fb = plane_state->fb;
	struct drm_plane_state *old_plane_state = drm_atomic_get_old_plane_state(state, plane);
	struct iosys_map vram_map = IOSYS_MAP_INIT_VADDR_IOMEM(mq->vram);
	struct drm_atomic_helper_damage_iter iter;
	struct drm_rect damage;
	int idx;

	if (!fb)
		return;

	if (!drm_dev_enter(&mq->drm, &idx))
		return;

	drm_atomic_helper_damage_iter_init(&iter, old_plane_state, plane_state);
	drm_atomic_for_each_plane_damage(&iter, &damage) {
		unsigned int offset = drm_fb_clip_offset(fb->pitches[0], fb->format, &damage);
		struct iosys_map dst = IOSYS_MAP_INIT_OFFSET(&vram_map, offset);

		drm_fb_memcpy(&dst, fb->pitches, shadow_plane_state->data, fb, &damage);
	}

	drm_dev_exit(idx);
}

static const struct drm_plane_helper_funcs mq11xx_primary_plane_helper_funcs = {
	DRM_GEM_SHADOW_PLANE_HELPER_FUNCS,
	.atomic_check = mq11xx_primary_plane_helper_atomic_check,
	.atomic_update = mq11xx_primary_plane_helper_atomic_update,
};

static const struct drm_plane_funcs mq11xx_primary_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	DRM_GEM_SHADOW_PLANE_FUNCS,
};

static enum drm_mode_status mq11xx_crtc_helper_mode_valid(struct drm_crtc *crtc,
							  const struct drm_display_mode *mode)
{
	struct mq11xx_device *mq = to_mq11xx(crtc->dev);

	return drm_crtc_helper_mode_valid_fixed(crtc, mode, mq->variant->mode);
}

static int mq11xx_crtc_helper_atomic_check(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	if (!crtc_state->enable)
		return 0;

	return drm_atomic_helper_check_crtc_primary_plane(crtc_state);
}

static void mq11xx_crtc_helper_atomic_enable(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct mq11xx_device *mq = to_mq11xx(crtc->dev);
	int idx;

	if (!drm_dev_enter(&mq->drm, &idx))
		return;

	mq11xx_output_enable(mq);

	drm_dev_exit(idx);
}

static void mq11xx_crtc_helper_atomic_disable(struct drm_crtc *crtc,
					      struct drm_atomic_commit *state)
{
	struct mq11xx_device *mq = to_mq11xx(crtc->dev);

	/*
	 * Not guarded by drm_dev_enter/exit, same as ili9225.c's own
	 * disable hook and for the same reason: we want the graceful
	 * register-level scanout disable to actually run on regular driver
	 * unload. mq11xx_remove() calls drm_dev_unplug() before
	 * drm_atomic_helper_shutdown() (so hotplug-time disable correctly
	 * skips touching now-absent hardware), but that ordering means an
	 * enter/exit-guarded version of this hook would find the device
	 * already marked unplugged during a normal rmmod/unbind and never
	 * run at all -- dead code on the only path meant to invoke it. The
	 * MediaQ chip stays mapped and clocked through remove() (the VRAM/
	 * register ioremaps are devm-managed and torn down after .remove()
	 * returns), so touching mq->regs here is safe either way.
	 */
	mq11xx_output_disable(mq);
}

static const struct drm_crtc_helper_funcs mq11xx_crtc_helper_funcs = {
	.mode_valid = mq11xx_crtc_helper_mode_valid,
	.atomic_check = mq11xx_crtc_helper_atomic_check,
	.atomic_enable = mq11xx_crtc_helper_atomic_enable,
	.atomic_disable = mq11xx_crtc_helper_atomic_disable,
};

static const struct drm_crtc_funcs mq11xx_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
};

static const struct drm_encoder_funcs mq11xx_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int mq11xx_connector_helper_get_modes(struct drm_connector *connector)
{
	struct mq11xx_device *mq = to_mq11xx(connector->dev);

	return drm_connector_helper_get_modes_fixed(connector, mq->variant->mode);
}

static const struct drm_connector_helper_funcs mq11xx_connector_helper_funcs = {
	.get_modes = mq11xx_connector_helper_get_modes,
};

static const struct drm_connector_funcs mq11xx_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int mq11xx_pipe_init(struct mq11xx_device *mq)
{
	struct drm_device *dev = &mq->drm;
	struct drm_plane *primary_plane = &mq->primary_plane;
	struct drm_crtc *crtc = &mq->crtc;
	struct drm_encoder *encoder = &mq->encoder;
	struct drm_connector *connector = &mq->connector;
	int ret;

	ret = drm_universal_plane_init(dev, primary_plane, 0,
				       &mq11xx_primary_plane_funcs,
					mq11xx_primary_plane_formats,
					ARRAY_SIZE(mq11xx_primary_plane_formats),
					mq11xx_primary_plane_format_modifiers,
					DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(primary_plane, &mq11xx_primary_plane_helper_funcs);
	drm_plane_enable_fb_damage_clips(primary_plane);

	ret = drm_crtc_init_with_planes(dev, crtc, primary_plane, NULL,
					&mq11xx_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(crtc, &mq11xx_crtc_helper_funcs);

	ret = drm_encoder_init(dev, encoder, &mq11xx_encoder_funcs,
			       DRM_MODE_ENCODER_DPI, NULL);
	if (ret)
		return ret;
	encoder->possible_crtcs = drm_crtc_mask(crtc);

	ret = drm_connector_init(dev, connector, &mq11xx_connector_funcs,
				 DRM_MODE_CONNECTOR_DPI);
	if (ret)
		return ret;
	drm_connector_helper_add(connector, &mq11xx_connector_helper_funcs);

	return drm_connector_attach_encoder(connector, encoder);
}

/* ------------------------------------------------------------------ */

static const struct drm_mode_config_funcs mq11xx_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int mq11xx_mode_config_init(struct mq11xx_device *mq)
{
	struct drm_device *dev = &mq->drm;
	int ret;

	ret = drmm_mode_config_init(dev);
	if (ret)
		return ret;

	dev->mode_config.min_width = mq->variant->mode->hdisplay;
	dev->mode_config.max_width = mq->variant->mode->hdisplay;
	dev->mode_config.min_height = mq->variant->mode->vdisplay;
	dev->mode_config.max_height = mq->variant->mode->vdisplay;
	dev->mode_config.preferred_depth = 16;
	dev->mode_config.prefer_shadow = 0;
	dev->mode_config.funcs = &mq11xx_mode_config_funcs;

	return 0;
}

DEFINE_DRM_GEM_FOPS(mq11xx_fops);

static const struct drm_driver mq11xx_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,

	.name	= DRIVER_NAME,
	.desc	= DRIVER_DESC,
	.major	= DRIVER_MAJOR,
	.minor	= DRIVER_MINOR,

	.fops	= &mq11xx_fops,
	DRM_GEM_SHMEM_DRIVER_OPS,
	DRM_FBDEV_SHMEM_DRIVER_OPS,
};

static int mq11xx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mq11xx_device *mq;
	struct drm_device *drm;
	int ret;

	mq = devm_drm_dev_alloc(dev, &mq11xx_driver, struct mq11xx_device, drm);
	if (IS_ERR(mq))
		return PTR_ERR(mq);
	drm = &mq->drm;

	mq->variant = device_get_match_data(dev);
	if (!mq->variant)
		return -ENODEV;

	platform_set_drvdata(pdev, mq);

	mq->vram = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mq->vram))
		return dev_err_probe(dev, PTR_ERR(mq->vram), "failed to map VRAM\n");

	/*
	 * Clear the VRAM up front: only damaged regions are ever copied in,
	 * and this chip scans out a single non-double-buffered VRAM
	 * independently of how much of the plane has been committed, so
	 * leftover contents would otherwise show up as noise.
	 */
	memset_io(mq->vram, 0, resource_size(platform_get_resource(pdev, IORESOURCE_MEM, 0)));

	mq->regs = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(mq->regs))
		return dev_err_probe(dev, PTR_ERR(mq->regs), "failed to map registers\n");

	/*
	 * Not every board switches every supply: h5xxx has no separate VDD
	 * switch and h2200 no separate panel switch. The rails still exist
	 * there, so a supply the devicetree does not describe resolves to the
	 * regulator core's dummy supply rather than being optional.
	 */
	mq->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(mq->vdd))
		return dev_err_probe(dev, PTR_ERR(mq->vdd), "failed to get vdd supply\n");

	mq->power = devm_regulator_get(dev, "power");
	if (IS_ERR(mq->power))
		return dev_err_probe(dev, PTR_ERR(mq->power), "failed to get power supply\n");

	/* h2200 only; h5xxx's devicetree describes no reset line (NULL). */
	mq->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(mq->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(mq->reset_gpio), "failed to get reset gpio\n");

	/*
	 * Driven from mq11xx_output_enable()/disable() rather than with the
	 * chip power: it gates the panel together with the MediaQ FP power
	 * gate at DPMS time, separately from the external supply gate of the
	 * chip, which is "power" above.
	 */
	mq->panel = devm_regulator_get(dev, "panel");
	if (IS_ERR(mq->panel))
		return dev_err_probe(dev, PTR_ERR(mq->panel), "failed to get panel supply\n");

	/*
	 * Power up and tie the power-down to devm before chip_init() writes a
	 * single register: the register load can fail, and a failed probe
	 * must not leave the chip powered and out of reset with no driver
	 * bound; regulator_put() on unwind also requires every enable to have
	 * been balanced first.
	 */
	ret = mq11xx_power_up(mq);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, mq11xx_power_down_action, mq);
	if (ret)
		return ret;

	ret = mq11xx_chip_init(mq);
	if (ret)
		return dev_err_probe(dev, ret, "chip bring-up failed\n");

	if (mq->variant->has_internal_pwm)
		mq11xx_pwm_chip_register(mq, dev);

	ret = mq11xx_mode_config_init(mq);
	if (ret)
		return ret;

	ret = mq11xx_pipe_init(mq);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	drm_client_setup(drm, NULL);

	return 0;
}

static void mq11xx_remove(struct platform_device *pdev)
{
	struct mq11xx_device *mq = platform_get_drvdata(pdev);

	drm_dev_unplug(&mq->drm);
	drm_atomic_helper_shutdown(&mq->drm);

	/* mq11xx_power_down_action() runs via devm after .remove() returns. */
}

/*
 * System sleep is a light suspend and resume: the MediaQ stays powered
 * across sleep and keeps its registers, so only the panel facing gating has
 * to be redone. Suspend gates the FP clock and GC scanout and drops the
 * panel enable, and resume does the opposite. The forced atomic disable
 * and enable of the DRM helpers run the DPMS off and on paths of this
 * driver, mq11xx_output_disable() and mq11xx_output_enable(), so no
 * additional register sequence is needed. The settle times after
 * suspending and resuming are those of the vendor display driver.
 */
static int mq11xx_pm_suspend(struct device *dev)
{
	struct mq11xx_device *mq = dev_get_drvdata(dev);
	int ret;

	ret = drm_mode_config_helper_suspend(&mq->drm);
	if (ret)
		return ret;

	msleep(58);

	return 0;
}

static int mq11xx_pm_resume(struct device *dev)
{
	struct mq11xx_device *mq = dev_get_drvdata(dev);

	drm_mode_config_helper_resume(&mq->drm);

	msleep(90);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mq11xx_pm_ops, mq11xx_pm_suspend, mq11xx_pm_resume);

static struct platform_driver mq11xx_platform_driver = {
	.probe	= mq11xx_probe,
	.remove	= mq11xx_remove,
	.driver	= {
		.name = DRIVER_NAME,
		.of_match_table = mq11xx_of_match,
		.pm = pm_sleep_ptr(&mq11xx_pm_ops),
	},
};
module_platform_driver(mq11xx_platform_driver);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
