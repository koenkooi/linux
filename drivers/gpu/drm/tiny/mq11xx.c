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
 * below (CC/MIU/IC/GC/DC/FP) are chip-family-wide, not specific to any
 * one board. The graphics engine bank is the exception: it moved between
 * parts, so it comes from the per-chip description (struct mq11xx_chip).
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
 * register load and the shadow-plane pipeline are shared. What differs
 * between chips, where the graphics engine sits and which optional display
 * features are known to work, is struct mq11xx_chip, chosen from the PCI
 * device ID by mq11xx_identify() independently of the board.
 *
 * Vblank is not supported: there is no drm_vblank_init() and no IRQ
 * handler, and atomic_update copies the damage into VRAM synchronously with
 * no regard for the scanout position (by the CPU, or through a DMA channel
 * while present_dma is set), so a large update can tear. The
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

#include <linux/backlight.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/sizes.h>
#include <linux/stdarg.h>

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

/*
 * The video memory sits behind a 16-bit VLIO chip select and is mapped as
 * device memory, which on XScale is bufferable but marked so that the write
 * buffer does not coalesce stores. A write-combining mapping (bufferable
 * only) lets it merge them. Whether that speeds up the frame copy on this
 * bus, where each 16-bit beat is its own strobe, depends on the board and
 * has to be measured, so the mapping is selectable at load time. The
 * registers keep the device mapping either way.
 */
static bool mq11xx_wc;
module_param_named(wc, mq11xx_wc, bool, 0444);
MODULE_PARM_DESC(wc, "Map video memory write-combining instead of device memory (default: off)");

/*
 * Whether the damage copy into video memory is made by a dmaengine memcpy
 * channel instead of the CPU. The channel writes video memory no faster than
 * the CPU does; what it gives is a CPU that sleeps through the transfer,
 * which only pays off where something else wants the CPU, and a DMA channel
 * is a scarce, shared resource on these SoCs. The right choice depends on
 * the board's workload, so it is switchable at run time, which also lets
 * the two paths be compared on a running system.
 */
static bool mq11xx_present_dma;
module_param_named(present_dma, mq11xx_present_dma, bool, 0644);
MODULE_PARM_DESC(present_dma, "Copy damage into video memory with a DMA channel instead of the CPU; may be changed at run time (default: off)");

/*
 * Whether the driver uses the chip's 2D engine to clear video memory and to
 * draw solid damage rectangles, on the chips where the engine is known to
 * work. The engine's results on the MQ1188 were verified from userspace on
 * one board only and the MQ1132 path has not run on hardware, so a user who
 * sees corruption on a board has to be able to fall back to CPU drawing
 * without a rebuild, and to compare the two on a running system.
 */
static bool mq11xx_engine = true;
module_param_named(engine, mq11xx_engine, bool, 0644);
MODULE_PARM_DESC(engine, "Clear video memory and draw solid damage with the 2D engine on chips where it is known to work; may be changed at run time (default: on)");

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
#define MQ11XX_REG_DC		0x380
#define MQ11XX_REG_PCI		0x400
#define MQ11XX_REG_FP		0x600

/*
 * Graphics engine (GE) bank. The MQ1132 datasheet puts it at 0x200. The
 * MQ1188 has the same register layout at 0x1400: its WinCE display driver
 * reaches the engine only there, and GE0A/GE0B written at 0x1428/0x142c
 * read back while 0x228/0x22c keep their own values. On the MQ1188,
 * 0x200-0x24c is an unrelated block (see mq11xx_h2200_init_regs[]).
 */
#define MQ1132_GE_BASE		0x200
#define MQ1188_GE_BASE		0x1400

/* Graphics engine registers, relative to the chip's engine bank. */
#define MQ11XX_GE_COMMAND	0x00	/* ge00, starts the command */
#define MQ11XX_GE_SIZE		0x04	/* ge01 */
#define MQ11XX_GE_DEST		0x08	/* ge02 */
#define MQ11XX_GE_DEST_STRIDE	0x28	/* ge0a */
#define MQ11XX_GE_BASE_ADDR	0x2c	/* ge0b */
#define MQ11XX_GE_PATTERN_FG	0x48	/* ge12 */

#define MQ11XX_GE_ROP_PATCOPY	0xf0
#define MQ11XX_GE_TYPE_BITBLT	(2 << 8)
#define MQ11XX_GE_MONO_PATTERN	BIT(15)
#define MQ11XX_GE_SOLID_PATTERN	BIT(30)

#define MQ11XX_GE_X		GENMASK(11, 0)	/* ge01, ge02 */
#define MQ11XX_GE_Y		GENMASK(27, 16)
#define MQ11XX_GE_STRIDE	GENMASK(9, 0)	/* ge0a, bytes */
#define MQ11XX_GE_DEPTH		GENMASK(31, 30)	/* ge0a: 0 8 bpp, 1 16 bpp */
#define MQ11XX_GE_BASE_MASK	GENMASK(19, 3)	/* ge0b, 8-byte aligned */

/*
 * Video memory layout. The scanout frame starts at offset 0 (GC0C = 0 in
 * every register table). The last KiB holds the hardware cursor image:
 * GC11 addresses it in 1 KiB units, and both boards' WinCE drivers place
 * it there.
 */
#define MQ11XX_SRAM_FRAME	0x00000
#define MQ11XX_SRAM_CURSOR	0x3fc00
#define MQ11XX_SRAM_CURSOR_SIZE	SZ_1K

/* Individual registers this driver names and touches directly. */
#define MQ11XX_CC_CONTROL	(MQ11XX_REG_CC + 0x00)	/* cc00 */
#define MQ11XX_CC_STATUS	(MQ11XX_REG_CC + 0x04)	/* cc01 */
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
 * FP.input_control and FP.pin_input_data. ENVEE is bit 1 of both: set in
 * input_control to enable readback of the pin, then read from
 * pin_input_data. Sharp panels read it clear and Philips panels set.
 */
#define MQ11XX_FP_INPUT_CONTROL (MQ11XX_REG_FP + 0x0c)
#define MQ11XX_FP_PIN_INPUT_DATA (MQ11XX_REG_FP + 0x24)
#define MQ11XX_FP_ENVEE	BIT(1)
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

/*
 * The CPU interface caches frame buffer reads, and the cache does not see
 * engine writes: after an engine fill a read of a word the CPU had read
 * before returned the old value on an MQ1188, as the MQ1132 datasheet
 * warns. CC00[1] turns the cache off.
 */
#define MQ11XX_CC_READ_CACHE_OFF	BIT(1)	/* cc00 */
#define MQ11XX_CC_GE_BUSY		BIT(16)	/* cc01 */
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
 * Optional features of a chip. For the MQ1132 they come from its
 * datasheet; for the MQ1188, which has none, a feature is listed only
 * once it has been exercised on real silicon.
 */
#define MQ11XX_CAP_ENGINE	BIT(0)	/* 2D engine at .ge_base */
#define MQ11XX_CAP_ROTATION	BIT(1)	/* GC00[13:12] + signed GC0E transforms */
#define MQ11XX_CAP_PIXEL_DOUBLE	BIT(2)	/* GC00[15:14] */
#define MQ11XX_CAP_ALT_WINDOW	BIT(3)	/* GC0D, GC00[11:10] */
#define MQ11XX_CAP_PALETTE_8BPP	BIT(4)	/* 8 bpp through the palette at 0x800 */
#define MQ11XX_CAP_CURSOR	BIT(5)	/* 64x64 2 bpp cursor, GC10-GC13 */
#define MQ11XX_CAP_FRAME_LATCH	BIT(6)	/* IN02 VENABLE-falling frame latch */

static const char * const mq11xx_cap_names[] = {
	"engine", "rotation", "pixel-doubling", "alt-window",
	"palette-8bpp", "cursor", "frame-latch",
};

/*
 * Per-chip description, selected from the PCI device ID at bring-up. The
 * board variant (struct mq11xx_variant) says how a board wires and
 * programs its chip; this says what the silicon itself offers.
 */
struct mq11xx_chip {
	u16 device_id;
	const char *name;
	u16 ge_base;
	unsigned int caps;
};

#define MQ1132_CAPS	(MQ11XX_CAP_ENGINE | MQ11XX_CAP_ROTATION |		\
			 MQ11XX_CAP_PIXEL_DOUBLE | MQ11XX_CAP_ALT_WINDOW |	\
			 MQ11XX_CAP_PALETTE_8BPP | MQ11XX_CAP_CURSOR |		\
			 MQ11XX_CAP_FRAME_LATCH)

static const struct mq11xx_chip mq11xx_chips[] = {
	{
		.device_id = MQ11XX_PCI_DEVICE_1132,
		.name = "MQ1132",
		.ge_base = MQ1132_GE_BASE,
		.caps = MQ1132_CAPS,
	}, {
		/* Covered by the same datasheet as the MQ1132. */
		.device_id = MQ11XX_PCI_DEVICE_1100,
		.name = "MQ1100",
		.ge_base = MQ1132_GE_BASE,
		.caps = MQ1132_CAPS,
	}, {
		/*
		 * Measured on an h2200: engine fill and copy (including
		 * overlapping copies), all eight scanout transforms, pixel
		 * doubling, 8 bpp through the palette, the cursor fetch at
		 * 0x3fc00 and the IN02 frame latch. The alternate window
		 * takes its registers but was never shown to scan out, so it
		 * stays off.
		 */
		.device_id = MQ11XX_PCI_DEVICE_1188,
		.name = "MQ1188",
		.ge_base = MQ1188_GE_BASE,
		.caps = MQ1132_CAPS & ~MQ11XX_CAP_ALT_WINDOW,
	}, {
		/*
		 * Presumed to share the MQ1188's layout, but no MQ1178 has
		 * ever been read, so nothing optional is enabled.
		 */
		.device_id = MQ11XX_PCI_DEVICE_1178,
		.name = "MQ1178",
		.ge_base = MQ1188_GE_BASE,
	},
};

/* Anything unrecognised gets the baseline: scanout only. */
static const struct mq11xx_chip mq11xx_chip_unknown = {
	.name = "unknown",
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
	 * when the ENVEE pin reports a Philips panel (see
	 * mq11xx_apply_envee_delta()). NULL for variants with a single panel.
	 */
	const struct mq11xx_reg *envee_regs;
	size_t n_envee_regs;
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

/*
 * Resources of the DMA present path. They are set up from the plane's
 * atomic_check, the first time it finds present_dma set, so that the commit
 * tail neither allocates nor acquires anything. Checks are serialised by the
 * plane lock and publish @chan last, with release semantics; the commit
 * tail, whose commits on the one CRTC run in order, reads @chan with acquire
 * semantics and owns @failed. mq11xx_remove() tears everything down once no
 * commit can be running any more. A failed setup is not retried.
 */
struct mq11xx_present_dma {
	struct dma_chan *chan;		/* NULL: unavailable or not set up */
	void *bounce;			/* one frame in coherent memory */
	dma_addr_t bounce_dma;
	dma_addr_t vram_dma;		/* the frame at the start of video memory */
	size_t size;
	unsigned int pitch;
	bool tried;
	bool failed;			/* a transfer failed: CPU copies only */
};

struct mq11xx_device {
	struct drm_device drm;

	const struct mq11xx_variant *variant;
	const struct mq11xx_chip *chip;

	struct drm_plane primary_plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;

	void __iomem *vram;
	resource_size_t vram_size;
	phys_addr_t vram_phys;
	bool vram_wc;
	void __iomem *regs;

	/*
	 * Chip power follows the CRTC: off while the display is blanked.
	 * hw_lock serialises that against the pwm_chip callbacks, which
	 * sysfs can call at any time -- a register access while the chip is
	 * unpowered is never valid on this static-memory bus. pwm_level is
	 * the channel-0 duty count to restore after each re-power, since the
	 * register table load resets it; -1 until the pwm_chip reads it.
	 */
	struct mutex hw_lock;
	bool powered;
	/* First hw_init done; later re-inits (every unblank) log at debug level. */
	bool init_logged;
	int pwm_level;

	/*
	 * Serialises engine commands, which the commit tail issues without
	 * hw_lock; taken after hw_lock where both are held. ge_failed latches
	 * the engine off after a command timed out. The engine surface
	 * registers are rewritten only when the wanted layout differs from
	 * ge_stride_reg/ge_base_reg, which bring-up invalidates.
	 */
	struct mutex ge_lock;
	bool ge_failed;
	bool ge_surface_valid;
	u32 ge_stride_reg;
	u32 ge_base_reg;

	struct regulator *vdd;		/* h2200: MQ1178_VDD */
	struct regulator *power;	/* h2200: MQ1178_POWER_ON;
					 * h5xxx: MQ_POWER_ON
					 */
	struct gpio_desc *reset_gpio;	/* h2200 only: MQ1178_RESET_N */
	struct regulator *panel;	/* h5xxx: LCD_EN, the panel's supply
					 * rather than chip power, see
					 * mq11xx_output_enable()
					 */
	struct backlight_device *backlight;	/* NULL if DT names none */

	struct mq11xx_present_dma present_dma;
};

#define to_mq11xx(_dev) container_of(_dev, struct mq11xx_device, drm)

/* Bring-up lines: info on the first init only, since every unblank repeats them. */
static __printf(2, 3) void mq11xx_init_info(struct mq11xx_device *mq,
					    const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;

	if (mq->init_logged)
		drm_dbg(&mq->drm, "%pV", &vaf);
	else
		drm_info(&mq->drm, "%pV", &vaf);

	va_end(args);
}

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
 * to the registers the chip actually has: a few GC/FP registers, the SPI
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

	/*
	 * MQ1188 0x200 block. These would be GE0A, GE0B and GE11 of an engine
	 * at 0x200, but on the MQ1188 the engine is at 0x1400 and this block
	 * is an undocumented one of its own: all three read back as latched
	 * registers that engine writes do not disturb, and WinCE writes
	 * 0x55 to 0x244 in its front-light sequence. The values are the ones
	 * read off a running unit.
	 */
	{ 0x228, 0x00000000 },
	{ 0x22c, 0x6b6ffeae },
	{ 0x244, 0x00000055 },
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
	 * left out here.
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
	 * family, left out here.
	 */
	{ MQ11XX_REG_FP + 0x1c0, 0x00000405 },	/* fp70 */
	{ MQ11XX_REG_FP + 0x1c4, 0xced6a94e },	/* fp71 */
	{ MQ11XX_REG_FP + 0x1c8, 0x00000002 },	/* fp72 */
	{ MQ11XX_REG_FP + 0x1cc, 0x00000000 },	/* fp73 */
	{ MQ11XX_REG_FP + 0x1d0, 0x00000000 },	/* fp74 */
	{ MQ11XX_REG_FP + 0x1d4, 0x00000000 },	/* fp75 */
	{ MQ11XX_REG_FP + 0x1d8, 0x95f9c859 },	/* fp76 */
	{ MQ11XX_REG_FP + 0x1dc, 0x30388dc9 },	/* fp77 */
	/*
	 * GE0A/GE0B (a 640-byte destination stride, base 0) are left out:
	 * they match no mode this driver sets, and the engine pitch and base
	 * belong to whatever issues engine commands.
	 */
};

/*
 * Register values of the Philips panel that differ from the Sharp table
 * above (cc02, gc00, gc02-gc05, gc0b, fp01, fp05, fp06 and fp08), applied
 * on top of it when the ENVEE pin reports a Philips panel
 * (mq11xx_apply_envee_delta()). fp08 is 0x00040004 in the vendor tables of
 * both panels, but the vendor code sets its bit 0 afterwards for a Philips
 * panel. With fp07 bit 0 set, that bit drives the panel's ENVDD pin, so
 * the delta carries the value the vendor code leaves in the register.
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
	{ MQ11XX_REG_FP + 0x020, 0x00040005 },	/* fp08, Sharp had 0x00040004 */
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
	 * The panel is chosen at every bring-up from the ENVEE pin (see
	 * mq11xx_apply_envee_delta()). h5550 units with either panel exist:
	 * the panel registers that WinCE leaves behind match the Sharp table
	 * on a unit that reads the pin clear and the Philips delta on one
	 * that reads it set.
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
 * mq->variant->init_regs always holds the Sharp values, so when the ENVEE
 * pin reads clear nothing is done, and when it reads set the differing
 * registers are overwritten with the Philips values on top of the table that
 * mq11xx_load_regs() just loaded. The pin read is the one bootldr does, and
 * hackndev's h5400_init_lcd_info() before it.
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
	u32 ctl, pin;
	int ret;

	if (!regs)
		return 0;

	ctl = readl(mq->regs + MQ11XX_FP_INPUT_CONTROL);
	writel(ctl | MQ11XX_FP_ENVEE, mq->regs + MQ11XX_FP_INPUT_CONTROL);
	pin = readl(mq->regs + MQ11XX_FP_PIN_INPUT_DATA);
	mq11xx_init_info(mq, "%s: ENVEE pin_input_data = %#010x, %s panel timing\n",
			 mq->variant->name, pin,
			 pin & MQ11XX_FP_ENVEE ? "Philips" : "Sharp");
	if (!(pin & MQ11XX_FP_ENVEE))
		return 0;

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

static void mq11xx_select_chip(struct mq11xx_device *mq, u16 device)
{
	const struct mq11xx_chip *chip = &mq11xx_chip_unknown;
	char caps[96] = "";
	unsigned int i;
	int len = 0;

	for (i = 0; i < ARRAY_SIZE(mq11xx_chips); i++) {
		if (mq11xx_chips[i].device_id == device) {
			chip = &mq11xx_chips[i];
			break;
		}
	}
	mq->chip = chip;

	for (i = 0; i < ARRAY_SIZE(mq11xx_cap_names); i++) {
		if (!(chip->caps & BIT(i)))
			continue;
		len += scnprintf(caps + len, sizeof(caps) - len, " %s",
				 mq11xx_cap_names[i]);
		if (BIT(i) == MQ11XX_CAP_ENGINE)
			len += scnprintf(caps + len, sizeof(caps) - len,
					 "@0x%x", chip->ge_base);
	}

	mq11xx_init_info(mq, "chip %s, capabilities:%s\n", chip->name,
			 len ? caps : " none");
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

	mq11xx_init_info(mq, "%s: PCI header reports vendor 0x%04x device 0x%04x\n",
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
		mq11xx_init_info(mq,
				 "PCI device 0x%04x is a tolerated alternate for the %s variant, using the init_regs of 0x%04x\n",
				 device, mq->variant->name, mq->variant->expect_device_id);

	mq11xx_select_chip(mq, vendor == MQ11XX_PCI_VENDOR_MEDIAQ ? device : 0);
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

/*
 * devm_add_action_or_reset() callback. Only a powered chip holds supply
 * references to drop: a chip that failed mq11xx_hw_init() was already
 * powered down there, and a blanked one by mq11xx_output_disable().
 */
static void mq11xx_power_down_action(void *data)
{
	struct mq11xx_device *mq = data;

	if (mq->powered) {
		mq11xx_power_down(mq);
		mq->powered = false;
	}
}

/*
 * One-time chip bring-up: endianness quirk, oscillator enable (100 ms; 30 ms
 * is not enough), CC-module power (10 ms), the DC.config_8 value, MIU
 * enable, interrupt controller mask, the PCI-header identity read, then the
 * full register table load. Scanout stays off until the first atomic_enable
 * (see mq11xx_output_enable()).
 *
 * Only called from mq11xx_hw_init(), which powers the chip up first and
 * powers it back down if this fails.
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

static unsigned int mq11xx_pwm_read_level(struct mq11xx_device *mq)
{
	return (readl(mq->regs + MQ11XX_FP_PULSE_WIDTH_MOD_CONTROL) >>
		MQ11XX_PWM_CHAN0_SHIFT) & MQ11XX_PWM_MAX_LEVEL;
}

static void mq11xx_pwm_write_level(struct mq11xx_device *mq, unsigned int level)
{
	u32 val;

	val = readl(mq->regs + MQ11XX_FP_PULSE_WIDTH_MOD_CONTROL);
	val &= ~(MQ11XX_PWM_MAX_LEVEL << MQ11XX_PWM_CHAN0_SHIFT);
	val |= level << MQ11XX_PWM_CHAN0_SHIFT;
	writel(val, mq->regs + MQ11XX_FP_PULSE_WIDTH_MOD_CONTROL);
}

/* ------------------------------------------------------------------ */
/* 2D graphics engine */

/*
 * The engine fills inside video memory on its own, so a clear or a solid
 * rectangle costs a few register writes instead of every pixel crossing the
 * 16-bit host bus at about 8 MB/s; an MQ1188 engine fills at about 185 MB/s.
 * DRM has no 2D interface, so only the driver itself issues commands.
 *
 * Every command is synchronous: its operands, then GE00, then a bounded
 * poll of the busy bit. So at most one command is ever queued, the FIFO
 * level fields are never needed, and nothing is still drawing when the CPU
 * next touches video memory. Neither auto-execute nor the host source FIFO
 * is used: the chip holds the CPU's bus cycle while a FIFO is full, the
 * PXA bus has no timeout, and a miscounted source push would hang the
 * machine.
 */

/*
 * A command's timeout: a fixed allowance plus the bytes it writes at 16 MB/s,
 * a tenth of the rate measured on an MQ1188.
 */
#define MQ11XX_GE_TIMEOUT_BASE_US	2000
#define MQ11XX_GE_TIMEOUT_BYTES_PER_US	16

/* Below this, the command's register writes cost about what an upload does. */
#define MQ11XX_GE_FILL_MIN_PIXELS	16

/* The bring-up clear draws video memory as one surface of this pitch. */
#define MQ11XX_GE_CLEAR_PITCH		512
#define MQ11XX_GE_CLEAR_CHECKS		16

static bool mq11xx_ge_capable(struct mq11xx_device *mq)
{
	return (mq->chip->caps & MQ11XX_CAP_ENGINE) && !READ_ONCE(mq->ge_failed);
}

static bool mq11xx_ge_wanted(struct mq11xx_device *mq)
{
	return READ_ONCE(mq11xx_engine) && mq11xx_ge_capable(mq);
}

static void mq11xx_ge_write(struct mq11xx_device *mq, unsigned int reg, u32 val)
{
	writel(val, mq->regs + mq->chip->ge_base + reg);
}

static u32 mq11xx_ge_xy(unsigned int x, unsigned int y)
{
	return FIELD_PREP(MQ11XX_GE_X, x) | FIELD_PREP(MQ11XX_GE_Y, y);
}

static unsigned int mq11xx_ge_cpp(struct mq11xx_device *mq)
{
	return FIELD_GET(MQ11XX_GE_DEPTH, mq->ge_stride_reg) + 1;
}

/* Coordinates and sizes are 12-bit fields. */
static bool mq11xx_ge_rect_valid(const struct drm_rect *r)
{
	return r->x1 >= 0 && r->y1 >= 0 && r->x1 < r->x2 && r->y1 < r->y2 &&
	       r->x2 <= FIELD_MAX(MQ11XX_GE_X) && r->y2 <= FIELD_MAX(MQ11XX_GE_Y);
}

/*
 * Waits for the command just issued; the chip holds off CPU accesses to
 * video memory while the engine draws, so nothing touches it before this
 * returns 0. On a timeout, clearing the engine enable bit resets the
 * engine's state machines and FIFOs, and the engine is not used again.
 */
static int mq11xx_ge_wait(struct mq11xx_device *mq, size_t bytes)
{
	unsigned long timeout_us = MQ11XX_GE_TIMEOUT_BASE_US +
				   bytes / MQ11XX_GE_TIMEOUT_BYTES_PER_US;
	u32 status, val;
	int ret;

	lockdep_assert_held(&mq->ge_lock);

	ret = readl_poll_timeout_atomic(mq->regs + MQ11XX_CC_STATUS, status,
					!(status & MQ11XX_CC_GE_BUSY), 1,
					timeout_us);
	if (!ret)
		return 0;

	drm_err(&mq->drm,
		"graphics engine busy for %lu us (CC01 0x%08x), resetting it and drawing with the CPU from now on\n",
		timeout_us, status);

	val = readl(mq->regs + MQ11XX_DC_CONFIG_5);
	writel(val & ~MQ11XX_CONFIG_GE_ENABLE, mq->regs + MQ11XX_DC_CONFIG_5);
	writel(val, mq->regs + MQ11XX_DC_CONFIG_5);

	WRITE_ONCE(mq->ge_failed, true);
	mq->ge_surface_valid = false;

	status = readl(mq->regs + MQ11XX_CC_STATUS);
	if (status & MQ11XX_CC_GE_BUSY)
		drm_err(&mq->drm, "graphics engine still busy after its reset (CC01 0x%08x)\n",
			status);

	return ret;
}

/*
 * Points the engine at a pitch-linear surface of @cpp bytes per pixel
 * starting @base bytes into video memory. The engine takes the stride from
 * GE0A for both source and destination, so every command draws within this
 * one surface. The first command after a bring-up also turns the frame
 * buffer read cache off, since the CPU reads what the engine wrote.
 */
static int mq11xx_ge_set_surface(struct mq11xx_device *mq, size_t base,
				 unsigned int pitch, unsigned int cpp)
{
	u32 stride;

	lockdep_assert_held(&mq->ge_lock);

	if (cpp < 1 || cpp > 2 || pitch > FIELD_MAX(MQ11XX_GE_STRIDE) ||
	    (base & ~MQ11XX_GE_BASE_MASK))
		return -EINVAL;

	stride = FIELD_PREP(MQ11XX_GE_STRIDE, pitch) |
		 FIELD_PREP(MQ11XX_GE_DEPTH, cpp - 1);

	if (mq->ge_surface_valid && mq->ge_stride_reg == stride &&
	    mq->ge_base_reg == base)
		return 0;

	if (!mq->ge_surface_valid)
		writel(MQ11XX_CC_READ_CACHE_OFF, mq->regs + MQ11XX_CC_CONTROL);

	mq11xx_ge_write(mq, MQ11XX_GE_DEST_STRIDE, stride);
	mq11xx_ge_write(mq, MQ11XX_GE_BASE_ADDR, base);
	mq->ge_stride_reg = stride;
	mq->ge_base_reg = base;
	mq->ge_surface_valid = true;

	return 0;
}

/* Fills @r of the current surface with @colour, a pixel value. */
static int mq11xx_ge_fill(struct mq11xx_device *mq, const struct drm_rect *r,
			  u32 colour)
{
	unsigned int w = drm_rect_width(r), h = drm_rect_height(r);

	lockdep_assert_held(&mq->ge_lock);

	if (!mq11xx_ge_capable(mq) || !mq->ge_surface_valid)
		return -ENODEV;
	if (!mq11xx_ge_rect_valid(r))
		return -EINVAL;

	mq11xx_ge_write(mq, MQ11XX_GE_SIZE, mq11xx_ge_xy(w, h));
	mq11xx_ge_write(mq, MQ11XX_GE_DEST, mq11xx_ge_xy(r->x1, r->y1));
	mq11xx_ge_write(mq, MQ11XX_GE_PATTERN_FG, colour);
	mq11xx_ge_write(mq, MQ11XX_GE_COMMAND,
			MQ11XX_GE_ROP_PATCOPY | MQ11XX_GE_TYPE_BITBLT |
			MQ11XX_GE_MONO_PATTERN | MQ11XX_GE_SOLID_PATTERN);

	return mq11xx_ge_wait(mq, (size_t)w * h * mq11xx_ge_cpp(mq));
}

/*
 * Clears video memory with one fill of a 512-byte-pitch surface covering
 * all of it, then spot-checks the result. Whatever the engine did not
 * clear, the CPU does; a clear that reads back wrong also latches the
 * engine off.
 */
static void mq11xx_ge_clear_vram(struct mq11xx_device *mq)
{
	unsigned int lines = mq->vram_size / MQ11XX_GE_CLEAR_PITCH;
	size_t done = (size_t)lines * MQ11XX_GE_CLEAR_PITCH;
	size_t step = done / MQ11XX_GE_CLEAR_CHECKS, off = 0;
	struct drm_rect r = DRM_RECT_INIT(0, 0, MQ11XX_GE_CLEAR_PITCH / 2, lines);
	unsigned int i;
	u32 val = 0;

	if (!step || mq11xx_ge_set_surface(mq, MQ11XX_SRAM_FRAME,
					   MQ11XX_GE_CLEAR_PITCH, 2) ||
	    mq11xx_ge_fill(mq, &r, 0))
		goto fallback;

	/* The first and the last word of each sixteenth. */
	for (i = 0; i < 2 * MQ11XX_GE_CLEAR_CHECKS; i++) {
		off = (i / 2) * step + (i & 1 ? step - sizeof(u32) : 0);
		val = readl(mq->vram + off);
		if (val) {
			drm_err(&mq->drm,
				"graphics engine clear left 0x%08x at video memory offset 0x%zx, drawing with the CPU from now on\n",
				val, off);
			WRITE_ONCE(mq->ge_failed, true);
			goto fallback;
		}
	}

	if (done < mq->vram_size)
		memset_io(mq->vram + done, 0, mq->vram_size - done);
	return;

fallback:
	memset_io(mq->vram, 0, mq->vram_size);
}

/*
 * Engine half of every bring-up, after the register table load, which has
 * left GE0A/GE0B undefined and the read cache on. @clear: video memory is
 * still to be cleared, by the engine.
 */
static void mq11xx_ge_bring_up(struct mq11xx_device *mq, bool clear)
{
	mutex_lock(&mq->ge_lock);
	mq->ge_surface_valid = false;
	if (clear)
		mq11xx_ge_clear_vram(mq);
	mutex_unlock(&mq->ge_lock);
}

/* Whether the @r pixels at @vaddr, @pitch bytes per row, are all one RGB565 value. */
static bool mq11xx_rect_is_solid16(const void *vaddr, unsigned int pitch,
				   const struct drm_rect *r, u32 *colour)
{
	unsigned int w = drm_rect_width(r), h = drm_rect_height(r), x, y;
	__le16 c = *(const __le16 *)vaddr;

	for (y = 0; y < h; y++) {
		const __le16 *px = vaddr + y * pitch;

		for (x = 0; x < w; x++)
			if (px[x] != c)
				return false;
	}

	*colour = le16_to_cpu(c);
	return true;
}

/*
 * Draws a damage rectangle whose shadow pixels are all one colour with an
 * engine fill instead of uploading it: clears, letterbox bars and blank
 * console lines then cost no host bus traffic. The shadow buffer is
 * cached system memory, and the scan stops at the first differing pixel.
 * The engine surface is the frame as the CPU path writes it, at the frame
 * buffer's own pitch. Returns 0 when the engine drew the rectangle.
 */
static int mq11xx_ge_fill_damage(struct mq11xx_device *mq,
				 struct drm_framebuffer *fb,
				 const struct iosys_map *data,
				 const struct drm_rect *damage)
{
	unsigned int pitch = fb->pitches[0];
	u32 colour;
	int ret;

	if (!mq11xx_ge_wanted(mq) || fb->format->cpp[0] != 2 || data->is_iomem ||
	    drm_rect_width(damage) * drm_rect_height(damage) < MQ11XX_GE_FILL_MIN_PIXELS)
		return -ENODEV;

	if (!mq11xx_rect_is_solid16(data->vaddr +
				    drm_fb_clip_offset(pitch, fb->format, damage),
				    pitch, damage, &colour))
		return -EINVAL;

	mutex_lock(&mq->ge_lock);
	ret = mq11xx_ge_set_surface(mq, MQ11XX_SRAM_FRAME, pitch, 2);
	if (!ret)
		ret = mq11xx_ge_fill(mq, damage, colour);
	mutex_unlock(&mq->ge_lock);

	return ret;
}

/*
 * Full bring-up from power-off: rails and reset, the video memory clear,
 * then mq11xx_chip_init()'s register load. Runs at probe and again on
 * every unblank, because mq11xx_output_disable() cuts chip power and
 * neither the registers nor the video memory can be assumed to survive
 * that. Leaves the chip powered down again if the register load fails.
 */
static int mq11xx_hw_init(struct mq11xx_device *mq)
{
	bool ge_clear;
	int ret;

	lockdep_assert_held(&mq->hw_lock);

	/*
	 * The chip is identified during the register load, so only a re-init
	 * already knows whether its engine can clear video memory.
	 */
	ge_clear = mq11xx_ge_wanted(mq);

	ret = mq11xx_power_up(mq);
	if (ret)
		return ret;

	/*
	 * Clear the whole video memory before any scanout: the chip scans
	 * out of a single, non-double-buffered VRAM with no vsync gating,
	 * and mq11xx_output_enable() turns scanout on before the shadow
	 * plane's first full-damage copy lands, so anything not cleared here
	 * is visible as noise until then. It must run after
	 * mq11xx_power_up(): a CPU write to the chip select while the chip
	 * is unpowered or held in reset does not reach the memory.
	 * The engine clear instead runs after the register load, which
	 * programs the engine, and still before scanout starts; the CPU
	 * clear takes about 30 ms of bus time, the engine's under 2 ms.
	 */
	if (!ge_clear)
		memset_io(mq->vram, 0, mq->vram_size);

	ret = mq11xx_chip_init(mq);
	if (ret) {
		mq11xx_power_down(mq);
		return ret;
	}

	mq11xx_ge_bring_up(mq, ge_clear);

	mq->powered = true;
	mq->init_logged = true;

	if (mq->variant->has_internal_pwm && mq->pwm_level >= 0)
		mq11xx_pwm_write_level(mq, mq->pwm_level);

	return 0;
}

/*
 * Unblank. Mirrors probe's bring-up order: the chip is powered, out of
 * reset and fully programmed by mq11xx_hw_init() before any of the
 * scanout and flat-panel writes below touch it.
 */
static void mq11xx_output_enable(struct mq11xx_device *mq)
{
	u32 val;
	int ret;

	mutex_lock(&mq->hw_lock);

	if (!mq->powered) {
		ret = mq11xx_hw_init(mq);
		if (ret) {
			drm_err(&mq->drm, "chip bring-up failed (%d), display stays off\n",
				ret);
			goto out_unlock;
		}
	}

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
		mq11xx_power_down(mq);
		mq->powered = false;
		goto out_unlock;
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

	mutex_unlock(&mq->hw_lock);

	/*
	 * Backlight last, once the panel is powered and driven -- the same
	 * order drm_panel_enable() gives panel drivers. Called outside
	 * hw_lock because a pwm-backlight on this chip's own pwm_chip would
	 * take that lock from the PWM callbacks.
	 */
	backlight_enable(mq->backlight);
	return;

out_unlock:
	mutex_unlock(&mq->hw_lock);
}

/*
 * Blank: the reverse of mq11xx_output_enable(), ending with the chip's
 * own power cut, so a blanked screen draws nothing from the MediaQ.
 */
static void mq11xx_output_disable(struct mq11xx_device *mq)
{
	u32 val;

	/*
	 * Backlight first, while the panel is still powered and driven: these
	 * normally-white TFTs pass the light straight through once undriven,
	 * so a lit backlight turns a blanked screen bright white instead of
	 * dark. This is the order drm_panel_disable() gives panel drivers.
	 */
	backlight_disable(mq->backlight);

	mutex_lock(&mq->hw_lock);

	if (!mq->powered)
		goto out_unlock;

	/* Blank the panel before tearing down scanout -- see the mirrored
	 * comment in mq11xx_output_enable() above.
	 */
	regulator_disable(mq->panel);

	val = readl(mq->regs + MQ11XX_GC_CONTROL);
	writel(val & ~MQ11XX_GC_CONTROL_ENABLE, mq->regs + MQ11XX_GC_CONTROL);

	val = readl(mq->regs + MQ11XX_DC_CONFIG_5);
	writel(val & ~MQ11XX_CONFIG_GE_ENABLE, mq->regs + MQ11XX_DC_CONFIG_5);

	writel(0, mq->regs + MQ11XX_FP_OUTPUT_CONTROL);

	mq11xx_power_down(mq);
	mq->powered = false;

out_unlock:
	mutex_unlock(&mq->hw_lock);
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

	/* While blanked, only remember the level for mq11xx_hw_init(). */
	mutex_lock(&mq->hw_lock);
	mq->pwm_level = level;
	if (mq->powered)
		mq11xx_pwm_write_level(mq, level);
	mutex_unlock(&mq->hw_lock);

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

	mutex_lock(&mq->hw_lock);
	level = mq->powered ? mq11xx_pwm_read_level(mq) : mq->pwm_level;
	mutex_unlock(&mq->hw_lock);

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

	/* Probe just powered the chip, so fp0f holds the table's level. */
	mutex_lock(&mq->hw_lock);
	mq->pwm_level = mq11xx_pwm_read_level(mq);
	mutex_unlock(&mq->hw_lock);

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

/*
 * The DMA present path. The CPU first copies each rectangle from the
 * shadow buffer into a coherent bounce buffer at the same frame offsets,
 * and the channel moves it from there, so the channel never reads cached
 * memory and no cache maintenance is needed. The shadow pages are read
 * exactly as the CPU path reads them, so whatever their aliasing in the
 * virtually indexed data cache means for that path it means here too.
 */

/*
 * A transfer never legitimately runs long: a whole frame takes about
 * 20 ms. The fixed part covers descriptor setup and interrupt latency; the
 * proportional part assumes 1 MB/s, an eighth of the measured rate, which
 * leaves room for a lowest-priority channel to be held off by other DMA
 * traffic. A timeout costs a terminate and a CPU copy, so a generous limit
 * is cheaper than a spurious one.
 */
#define MQ11XX_PRESENT_DMA_BASE_MS	100
#define MQ11XX_PRESENT_DMA_BYTES_PER_MS	1000

/*
 * A descriptor costs the CPU a few microseconds to set up, about what it
 * spends stalled on the video memory bus while storing this many bytes
 * (the bus moves roughly 8 MB/s, so 64 bytes take about 8 us), so anything
 * moved in pieces smaller than this is left to the CPU. Without it a
 * narrow, tall rectangle such as a text caret would need a descriptor per
 * row and cost more than the copy it replaces.
 */
#define MQ11XX_PRESENT_DMA_MIN_CHUNK	64

static void mq11xx_dma_done(void *arg)
{
	complete(arg);
}

/*
 * Copies @n chunks of @chunk bytes as @n separate transfers, the i-th from
 * @src + i * @stride to @dst + i * @stride. All of them are prepared and
 * submitted before the channel is started, and only the last interrupts.
 * Waits for the last one up to @timeout_ms. No transfer is left in flight
 * on return.
 */
static int mq11xx_dma_round(struct dma_chan *chan, dma_addr_t dst,
			    dma_addr_t src, size_t chunk, size_t stride,
			    unsigned int n, unsigned int timeout_ms)
{
	DECLARE_COMPLETION_ONSTACK(done);
	struct dma_async_tx_descriptor *tx;
	dma_cookie_t cookie;
	unsigned long flags;
	unsigned int i;
	int ret;

	for (i = 0; i < n; i++) {
		flags = DMA_CTRL_ACK;
		if (i == n - 1)
			flags |= DMA_PREP_INTERRUPT;

		tx = dmaengine_prep_dma_memcpy(chan, dst + i * stride,
					       src + i * stride, chunk, flags);
		if (!tx) {
			ret = -ENOMEM;
			goto err_terminate;
		}
		if (i == n - 1) {
			tx->callback = mq11xx_dma_done;
			tx->callback_param = &done;
		}

		cookie = dmaengine_submit(tx);
		ret = dma_submit_error(cookie);
		if (ret)
			goto err_terminate;
	}
	dma_async_issue_pending(chan);

	if (!wait_for_completion_timeout(&done, msecs_to_jiffies(timeout_ms))) {
		ret = -ETIMEDOUT;
		goto err_terminate;
	}

	return 0;

err_terminate:
	/* Also discards transfers submitted before the failure. */
	dmaengine_terminate_sync(chan);
	return ret;
}

/*
 * Request the channel and map the bounce buffer and video memory for it.
 * Any failure leaves the path inert with the CPU copy in charge; probe
 * never depends on it. It is done on first use rather than at probe so
 * that a board that never sets present_dma does not hold a channel and
 * 150 KB of contiguous memory, which is a lot on 64 MB, for its whole
 * life. Called from atomic_check because it sleeps and allocates, which
 * the commit tail must not do.
 */
static void mq11xx_present_dma_setup(struct mq11xx_device *mq)
{
	const struct drm_display_mode *mode = mq->variant->mode;
	struct mq11xx_present_dma *d = &mq->present_dma;
	unsigned int pitch = mode->hdisplay * 2;
	size_t size = (size_t)pitch * mode->vdisplay;
	const char *why;
	struct dma_chan *chan;
	dma_addr_t bounce_dma, vram_dma;
	dma_cap_mask_t mask;
	struct device *dev;
	void *bounce;

	d->tried = true;

	if (size > mq->vram_size) {
		why = "video memory is smaller than a frame";
		goto err_log;
	}

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
	chan = dma_request_chan_by_mask(&mask);
	if (IS_ERR(chan)) {
		why = "no DMA_MEMCPY channel";
		goto err_log;
	}
	dev = chan->device->dev;

	bounce = dma_alloc_coherent(dev, size, &bounce_dma,
				    GFP_KERNEL | __GFP_NOWARN);
	if (!bounce) {
		why = "out of memory";
		goto err_release;
	}

	vram_dma = dma_map_resource(dev, mq->vram_phys, size, DMA_FROM_DEVICE, 0);
	if (dma_mapping_error(dev, vram_dma)) {
		why = "cannot map video memory for DMA";
		goto err_free;
	}

	if (!is_dma_copy_aligned(chan->device, vram_dma, bounce_dma, pitch)) {
		why = "video memory or line not aligned for the channel";
		goto err_unmap;
	}

	d->bounce = bounce;
	d->bounce_dma = bounce_dma;
	d->vram_dma = vram_dma;
	d->size = size;
	d->pitch = pitch;
	/* Pairs with the acquire in mq11xx_present_dma_rect(). */
	smp_store_release(&d->chan, chan);
	drm_info(&mq->drm, "present_dma: using channel %s\n", dma_chan_name(chan));

	return;

err_unmap:
	dma_unmap_resource(dev, vram_dma, size, DMA_FROM_DEVICE, 0);
err_free:
	dma_free_coherent(dev, size, bounce, bounce_dma);
err_release:
	dma_release_channel(chan);
err_log:
	drm_info(&mq->drm, "present_dma unavailable (%s), copying with the CPU\n", why);
}

static void mq11xx_present_dma_fini(struct mq11xx_device *mq)
{
	struct mq11xx_present_dma *d = &mq->present_dma;
	struct device *dev;

	if (!d->chan)
		return;

	dev = d->chan->device->dev;
	dma_unmap_resource(dev, d->vram_dma, d->size, DMA_FROM_DEVICE, 0);
	dma_free_coherent(dev, d->size, d->bounce, d->bounce_dma);
	dma_release_channel(d->chan);
	d->chan = NULL;
}

/*
 * Present the rows of @damage from the shadow buffer @data through the
 * channel, and wait for them. Returns 0 with the rectangle in video
 * memory, or an error with nothing in flight and the caller to copy the
 * rectangle itself; it may have written part of it.
 *
 * The channel moves multiples of its copy alignment between aligned
 * addresses, so the rectangle is widened to it. The extra pixels come
 * from the same shadow buffer and are what the frame holds there.
 *
 * Returns -EMSGSIZE, with nothing written, for a rectangle whose pieces
 * are below MQ11XX_PRESENT_DMA_MIN_CHUNK. Any failure of the transfer
 * itself turns the path off for good, so a channel that has stopped making
 * progress costs one timeout rather than one per rectangle.
 *
 * The chip must stay powered until this returns. Power is only cut by
 * mq11xx_output_disable() from the CRTC disable hook, which the commit tail
 * runs before it updates planes, and by a failed re-init in the enable hook,
 * which it runs before them. A later commit on the CRTC waits for this one
 * to signal hw_done, but only for a limited time, after which the helpers
 * let it go on regardless. That is why the time this function can take is
 * bounded: with at most one timeout per commit, of at most a few hundred
 * milliseconds, it stays far below that limit. mq11xx_remove() unplugs the
 * device, waiting for every drm_dev_enter() section, before it lets power
 * go. hw_lock is not held across the wait: it would serialise
 * the present path with the PWM callbacks.
 */
static int mq11xx_present_dma_rect(struct mq11xx_device *mq,
				   struct drm_framebuffer *fb,
				   const struct iosys_map *data,
				   const struct drm_rect *damage)
{
	struct mq11xx_present_dma *d = &mq->present_dma;
	/* Orders the reads of the other fields after setup's writes to them. */
	struct dma_chan *chan = smp_load_acquire(&d->chan);
	struct iosys_map bounce_map;
	struct dma_device *ddev;
	unsigned int align, cpp, x1, x2, rows, n, timeout_ms;
	size_t chunk, stride, offset, total;
	struct drm_rect wide;
	struct iosys_map dst;
	int ret;

	if (!chan || d->failed || fb->pitches[0] != d->pitch)
		return -ENODEV;

	iosys_map_set_vaddr(&bounce_map, d->bounce);
	ddev = chan->device;
	align = 1 << ddev->copy_align;
	cpp = fb->format->cpp[0];
	rows = drm_rect_height(damage);

	x1 = ALIGN_DOWN(damage->x1 * cpp, align);
	x2 = min(ALIGN(damage->x2 * cpp, align), d->pitch);
	wide = DRM_RECT_INIT(x1 / cpp, damage->y1, (x2 - x1) / cpp, rows);
	offset = drm_fb_clip_offset(d->pitch, fb->format, &wide);

	if (x2 - x1 == d->pitch) {
		/* Rows are adjacent, so the whole rectangle is one transfer. */
		chunk = (size_t)rows * d->pitch;
		stride = chunk;
		n = 1;
	} else {
		chunk = x2 - x1;
		stride = d->pitch;
		n = rows;
	}

	if (chunk < MQ11XX_PRESENT_DMA_MIN_CHUNK)
		return -EMSGSIZE;

	if (offset + (n - 1) * stride + chunk > d->size ||
	    !is_dma_copy_aligned(ddev, d->vram_dma + offset,
				 d->bounce_dma + offset, chunk))
		return -EINVAL;

	dst = IOSYS_MAP_INIT_OFFSET(&bounce_map, offset);
	drm_fb_memcpy(&dst, fb->pitches, data, fb, &wide);

	total = chunk * n;
	timeout_ms = MQ11XX_PRESENT_DMA_BASE_MS +
		     DIV_ROUND_UP(total, MQ11XX_PRESENT_DMA_BYTES_PER_MS);

	ret = mq11xx_dma_round(chan, d->vram_dma + offset,
			       d->bounce_dma + offset, chunk, stride, n,
			       timeout_ms);
	if (ret) {
		d->failed = true;
		drm_warn(&mq->drm, "present_dma: transfer failed (%d), copying with the CPU from now on\n",
			 ret);
	}

	return ret;
}

static int mq11xx_primary_plane_helper_atomic_check(struct drm_plane *plane,
						    struct drm_atomic_commit *state)
{
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc *new_crtc = new_plane_state->crtc;
	struct drm_crtc_state *new_crtc_state = NULL;
	struct mq11xx_device *mq = to_mq11xx(plane->dev);

	if (READ_ONCE(mq11xx_present_dma) && !mq->present_dma.tried)
		mq11xx_present_dma_setup(mq);

	if (new_crtc)
		new_crtc_state = drm_atomic_get_new_crtc_state(state, new_crtc);

	return drm_atomic_helper_check_plane_state(new_plane_state, new_crtc_state,
						    DRM_PLANE_NO_SCALING,
						    DRM_PLANE_NO_SCALING,
						    false, false);
}

/*
 * A write-combining mapping lets the CPU's write buffer hold stores back.
 * Reading back a location just written forces them out to the bus, so the
 * scanout sees the frame promptly; on the PXA255 the CP15 drain-write-buffer
 * operation is not a reliable substitute (specification update erratum 3).
 */
static void mq11xx_vram_flush(struct mq11xx_device *mq, size_t offset)
{
	/*
	 * Any halfword of the region does the job. A C8 damage rectangle can
	 * start on an odd byte, and an unaligned readw() there is an
	 * alignment fault.
	 */
	if (mq->vram_wc)
		readw(mq->vram + ALIGN_DOWN(offset, sizeof(u16)));
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
	bool dma;
	int idx;

	if (!fb)
		return;

	if (!drm_dev_enter(&mq->drm, &idx))
		return;

	/*
	 * The commit tail only updates planes on an active CRTC, but an
	 * unblank whose chip bring-up failed leaves the CRTC active with the
	 * chip unpowered. Written only from this same commit path, so no
	 * lock is needed to read it here.
	 */
	if (!mq->powered)
		goto out_exit;

	/*
	 * Read once: every rectangle of this commit is finished before it
	 * returns, so a change through sysfs simply takes effect on the next.
	 */
	dma = READ_ONCE(mq11xx_present_dma);

	drm_atomic_helper_damage_iter_init(&iter, old_plane_state, plane_state);
	drm_atomic_for_each_plane_damage(&iter, &damage) {
		unsigned int offset = drm_fb_clip_offset(fb->pitches[0], fb->format, &damage);
		struct iosys_map dst = IOSYS_MAP_INIT_OFFSET(&vram_map, offset);

		if (!mq11xx_ge_fill_damage(mq, fb, shadow_plane_state->data,
					   &damage))
			continue;

		if (dma && !mq11xx_present_dma_rect(mq, fb, shadow_plane_state->data,
						    &damage))
			continue;

		drm_fb_memcpy(&dst, fb->pitches, shadow_plane_state->data, fb, &damage);
		/* RGB565 is the only format, so the last pixel is 2 bytes. */
		mq11xx_vram_flush(mq, offset +
				  (drm_rect_height(&damage) - 1) * fb->pitches[0] +
				  drm_rect_width(&damage) * 2 - 2);
	}

out_exit:
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

/*
 * The chip is unpowered while the CRTC is off, so plane updates must be
 * skipped then and, on unblank, must follow the CRTC enable that powers
 * and reprograms the chip. The _rpm commit tail gives exactly that order
 * (disables, enables, then planes on active CRTCs only); the unblank's
 * modeset marks the whole plane damaged, so the copy restores the full
 * picture into the freshly cleared video memory.
 */
static const struct drm_mode_config_helper_funcs mq11xx_mode_config_helper_funcs = {
	.atomic_commit_tail = drm_atomic_helper_commit_tail_rpm,
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
	dev->mode_config.helper_private = &mq11xx_mode_config_helper_funcs;

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
	struct resource *res;
	int ret;

	mq = devm_drm_dev_alloc(dev, &mq11xx_driver, struct mq11xx_device, drm);
	if (IS_ERR(mq))
		return PTR_ERR(mq);
	drm = &mq->drm;

	mq->variant = device_get_match_data(dev);
	mq->chip = &mq11xx_chip_unknown;
	if (!mq->variant)
		return -ENODEV;

	platform_set_drvdata(pdev, mq);

	if (mq11xx_wc) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res)
			return dev_err_probe(dev, -ENODEV, "no VRAM resource\n");
		mq->vram = devm_ioremap_resource_wc(dev, res);
		mq->vram_wc = true;
	} else {
		mq->vram = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	}
	if (IS_ERR(mq->vram))
		return dev_err_probe(dev, PTR_ERR(mq->vram), "failed to map VRAM\n");
	mq->vram_size = resource_size(res);
	mq->vram_phys = res->start;

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

	mq->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(mq->backlight))
		return dev_err_probe(dev, PTR_ERR(mq->backlight), "failed to get backlight\n");

	ret = drmm_mutex_init(drm, &mq->hw_lock);
	if (ret)
		return ret;
	ret = drmm_mutex_init(drm, &mq->ge_lock);
	if (ret)
		return ret;
	mq->pwm_level = -1;

	/*
	 * Tie the power-down to devm before the first register write: a
	 * probe failure after mq11xx_hw_init() must not leave the chip powered
	 * and out of reset with no driver bound, and regulator_put() on unwind
	 * requires every enable to have been balanced first.
	 */
	ret = devm_add_action_or_reset(dev, mq11xx_power_down_action, mq);
	if (ret)
		return ret;

	/*
	 * Bring the chip up once here, not just at the first unblank, so
	 * a chip that does not respond fails probe instead of binding a
	 * display that can never light.
	 */
	mutex_lock(&mq->hw_lock);
	ret = mq11xx_hw_init(mq);
	mutex_unlock(&mq->hw_lock);
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
	/* Unplugged, so no commit can be using the channel any more. */
	mq11xx_present_dma_fini(mq);

	/* mq11xx_power_down_action() runs via devm after .remove() returns. */
}

/*
 * System sleep reuses the blank and unblank path: the forced atomic
 * disable and enable of the DRM helpers run mq11xx_output_disable() and
 * mq11xx_output_enable(), so the chip is powered off across sleep and fully
 * reprogrammed by mq11xx_hw_init() on resume, exactly as at probe. The
 * settle times after suspending and resuming are those of the vendor
 * display driver.
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
