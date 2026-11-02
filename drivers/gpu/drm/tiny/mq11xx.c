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
 * atomic_update copies the damage into VRAM synchronously with no regard
 * for the scanout position (by the CPU, or through a DMA channel while
 * present_dma is set), so a large update can tear. The 256 KiB VRAM holds
 * one 240x320 RGB565 frame (153600 bytes) but not two, so a hardware page
 * flip is not available at this format; tear-free updates need the copy
 * timed against the scanout instead. Frames that fit twice (C8, or
 * pixel-doubled) are double-buffered: the damage goes into the frame that
 * is not on screen, and the scan then flips to it.
 *
 * Vblank events come from the end-of-frame interrupt of the chip when the
 * devicetree describes its interrupt output (the h2200 wires it to GPIO14),
 * and otherwise from the DRM vblank timer running at the refresh rate of
 * the mode (the h5xxx leaves the output unconnected).
 *
 * The rotation property of the primary plane is backed by the scanout
 * transforms of the chip: the framebuffer is stored in video memory as it
 * is, and only the order in which the scan walks through it changes, so a
 * rotated picture costs no more bus traffic than an upright one.
 *
 * Chips with an 8 bpp scanout also take C8 framebuffers, shown through the
 * chip's 256-entry palette, which is the CRTC's gamma LUT; at 16 bpp the
 * palette is bypassed. A C8 frame halves the bytes each update puts on the
 * host bus. RGB565 remains the preferred format and fbdev's.
 *
 * Chips that double pixels at scanout also take a half-size framebuffer
 * (120x160 on a 240x320 panel) scaled up exactly 2x in both axes, through
 * the standard plane scaling UAPI. The chip repeats every pixel and line
 * of the whole window, so the doubled plane always covers the full screen,
 * and each update puts a quarter of the bytes on the host bus.
 *
 * Chips with the 64x64 two-colour hardware cursor expose it as a cursor
 * plane. It takes ARGB8888 images that are opaque in at most two colours
 * and fully transparent elsewhere; any other image is refused, so
 * userspace draws that cursor itself.
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

#include <linux/auxiliary_bus.h>
#include <linux/backlight.h>
#include <linux/bitfield.h>
#include <linux/bitrev.h>
#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/lcm.h>
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
#include <linux/slab.h>
#include <linux/stdarg.h>
#include <linux/wordpart.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_color_mgmt.h>
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
#include <drm/drm_vblank.h>
#include <drm/drm_vblank_helper.h>

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

static bool mq11xx_page_flip = true;
module_param_named(page_flip, mq11xx_page_flip, bool, 0644);
MODULE_PARM_DESC(page_flip, "Double-buffer frames that fit twice in video memory (C8 or pixel-doubled) and flip between them; may be changed at run time (default: on)");

/*
 * Panel power order of the h5550's WinCE display driver (ddi.dll), for the
 * Sharp panel: panel supply, then scanout, then the panel enable on MediaQ
 * GPIO1 (cc02 bit 7) with 40 ms on each side; on the way down the data pins
 * are driven to fixed levels for about 63 ms, GPIO1 drops, and the supply
 * goes about 63 ms later. The register table otherwise raises GPIO1 at load,
 * before the panel has a supply. Off by default until tested on a panel.
 */
static bool mq11xx_panel_seq;
module_param_named(panel_seq, mq11xx_panel_seq, bool, 0644);
MODULE_PARM_DESC(panel_seq, "h5xxx Sharp panel: WinCE power-on and power-off order (default N)");

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
#define MQ11XX_GE_SOURCE	0x0c	/* ge03 */
#define MQ11XX_GE_DEST_STRIDE	0x28	/* ge0a */
#define MQ11XX_GE_BASE_ADDR	0x2c	/* ge0b */
#define MQ11XX_GE_PATTERN_FG	0x48	/* ge12 */

#define MQ11XX_GE_ROP_PATCOPY	0xf0
#define MQ11XX_GE_ROP_SRCCOPY	0xcc
#define MQ11XX_GE_TYPE_BITBLT	(2 << 8)
#define MQ11XX_GE_MONO_PATTERN	BIT(15)
#define MQ11XX_GE_X_NEGATIVE	BIT(11)	/* walk the copy right to left */
#define MQ11XX_GE_Y_NEGATIVE	BIT(12)	/* walk the copy bottom to top */
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
 * it there. Everything from the end of the frame up to the cursor slot is
 * off-screen. A frame small enough to fit twice gets a second buffer at the
 * top of it, clear of any full-size frame at offset 0.
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
#define MQ11XX_IC_CONTROL	(MQ11XX_REG_IC + 0x00)	/* in00 */
#define MQ11XX_IC_MASK		(MQ11XX_REG_IC + 0x04)	/* in01, 1 = unmasked */
#define MQ11XX_IC_STATUS	(MQ11XX_REG_IC + 0x08)	/* in02, write 1 to clear */
#define MQ11XX_GC_CONTROL	(MQ11XX_REG_GC + 0x00)
#define MQ11XX_GC_WINDOW_START	(MQ11XX_REG_GC + 0x30)	/* gc0c */
#define MQ11XX_GC_WINDOW_STRIDE	(MQ11XX_REG_GC + 0x38)	/* gc0e, bytes/scanline */
#define MQ11XX_GC_CURSOR_POSITION (MQ11XX_REG_GC + 0x40)	/* gc10 */
#define MQ11XX_GC_CURSOR_ADDRESS (MQ11XX_REG_GC + 0x44)	/* gc11 */
#define MQ11XX_GC_CURSOR_FG	(MQ11XX_REG_GC + 0x48)	/* gc12 */
#define MQ11XX_GC_CURSOR_BG	(MQ11XX_REG_GC + 0x4c)	/* gc13 */
#define MQ11XX_GC_VERTICAL_PARAMETER (MQ11XX_REG_GC + 0x74)	/* gc1d */
#define MQ11XX_FP_OUTPUT_CONTROL (MQ11XX_REG_FP + 0x08)
#define MQ11XX_CC_GPIO_CONTROL	(MQ11XX_REG_CC + 0x08)	/* cc02 */
#define MQ11XX_CC_GPIO1_HIGH	BIT(7)	/* h5xxx Sharp panel enable */
#define MQ11XX_FP_PIN_CONTROL_1	(MQ11XX_REG_FP + 0x04)	/* fp01 */
#define MQ11XX_FP_GPIO_CONTROL_0 (MQ11XX_REG_FP + 0x1c)	/* fp07 */
#define MQ11XX_FP_GPIO_CONTROL_1 (MQ11XX_REG_FP + 0x20)	/* fp08 */
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
#define MQ11XX_GC_CONTROL_DEPTH		GENMASK(7, 4)	/* GC.control */
#define MQ11XX_GC_DEPTH_8BPP		0x3	/* through the palette */
#define MQ11XX_GC_DEPTH_16BPP		0xc	/* RGB565, palette bypassed */
#define MQ11XX_GC_CONTROL_CURSOR	BIT(8)	/* GC.control */
#define MQ11XX_GC_CONTROL_X_DECREMENT	BIT(12)	/* GC.control */
#define MQ11XX_GC_CONTROL_COLUMN_SCAN	BIT(13)	/* GC.control */
#define MQ11XX_GC_CONTROL_DOUBLE_H	BIT(14)	/* GC.control */
#define MQ11XX_GC_CONTROL_DOUBLE_V	BIT(15)	/* GC.control */
#define MQ11XX_GC_STRIDE		GENMASK(15, 0)	/* gc0e, two's complement */
/*
 * Colour palette, one 32-bit word per index. Only the top six bits of
 * each 8-bit field are stored: an MQ1188 reads 0xffffffff back as
 * 0x00fcfcfc.
 */
#define MQ11XX_PALETTE			0x800
#define MQ11XX_PALETTE_SIZE		256
#define MQ11XX_PALETTE_RED		GENMASK(7, 0)
#define MQ11XX_PALETTE_GREEN		GENMASK(15, 8)
#define MQ11XX_PALETTE_BLUE		GENMASK(23, 16)

/*
 * Hardware cursor: a 64x64 image of two 1 bpp planes, AND and XOR, 16
 * bytes per row. Each row is four 32-bit words of 16 pixels, the AND bits
 * in the low half and the XOR bits in the high half, leftmost pixel in the
 * least significant bit. AND 1 is transparent, AND 0 XOR 0 shows the
 * background colour (gc13) and AND 0 XOR 1 the foreground colour (gc12).
 * The position is in display pixels relative to the image window; HCO and
 * VCO skip that many columns and rows of the image, which is how it is
 * clipped at the left and top. gc10 and gc11 take effect at the next
 * falling edge of VSYNC after their most significant byte is written.
 */
#define MQ11XX_CURSOR_SIZE		64
#define MQ11XX_CURSOR_WORDS		(MQ11XX_CURSOR_SIZE * MQ11XX_CURSOR_SIZE / 16)
#define MQ11XX_GC_CURSOR_X		GENMASK(10, 0)	/* gc10, HCS */
#define MQ11XX_GC_CURSOR_Y		GENMASK(25, 16)	/* gc10, VCS */
#define MQ11XX_GC_CURSOR_ADDR		GENMASK(7, 0)	/* gc11, 1 KiB units */
#define MQ11XX_GC_CURSOR_HCO		GENMASK(21, 16)	/* gc11 */
#define MQ11XX_GC_CURSOR_VCO		GENMASK(29, 24)	/* gc11 */
#define MQ11XX_GC_CURSOR_RED		GENMASK(7, 2)	/* gc12, gc13 */
#define MQ11XX_GC_CURSOR_GREEN		GENMASK(15, 10)
#define MQ11XX_GC_CURSOR_BLUE		GENMASK(23, 18)

/*
 * gc1d: the scan line being fetched in bits 9:0. The first line of vertical
 * blanking (320, equal to vdisplay, on a running h2200) is the line the
 * WinCE display driver's wait-for-blank loop spins against.
 */
#define MQ11XX_GC_VCOUNT		GENMASK(9, 0)	/* gc1d */

/*
 * in00[0] drives the chip's interrupt output; with in00[1] clear it is
 * active low. in01/in02 bit 3 is the falling edge of vertical display
 * enable, i.e. the end of the last displayed line: on an h2200 it
 * latches at scan line 321 of 341, once per frame. The MQ1188 latches
 * in02 bits whether or not they are unmasked, but only unmasked bits
 * drive the output.
 */
#define MQ11XX_IC_OUTPUT_ENABLE		BIT(0)	/* in00 */
#define MQ11XX_IC_VENABLE_FALLING	BIT(3)	/* in01, in02 */

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
#define MQ11XX_CAP_START_LATCH	BIT(7)	/* GC0C takes effect at the next frame */

static const char * const mq11xx_cap_names[] = {
	"engine", "rotation", "pixel-doubling", "alt-window",
	"palette-8bpp", "cursor", "frame-latch", "start-latch",
};

/*
 * Scanout transforms. The scan walks the stored image along both of its
 * axes: GC00[12] runs each stored row from its last pixel, a negative GC0E
 * stride runs the rows from the last one up, and GC00[13] makes each
 * displayed line follow a stored column instead of a row. The eight
 * combinations are the four rotations and their mirror images; a chip
 * lists the ones it offers as a bitmask indexed by the combination.
 */
#define MQ11XX_SCAN_X_DECREMENT	BIT(0)
#define MQ11XX_SCAN_Y_DECREMENT	BIT(1)
#define MQ11XX_SCAN_COLUMNS	BIT(2)
#define MQ11XX_SCAN_ALL		GENMASK(7, 0)

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
	u8 transforms;		/* with MQ11XX_CAP_ROTATION */
	u8 transforms_8bpp;	/* of those, the ones offered at 8 bpp */
	bool ge_8bpp;		/* the engine draws 8 bpp surfaces */
	bool double_8bpp;	/* with MQ11XX_CAP_PIXEL_DOUBLE, also at 8 bpp */
	/* With MQ11XX_CAP_CURSOR, the transforms the cursor is shown under. */
	u8 cursor_transforms;
	bool cursor_double;	/* ... and whether also while pixels are doubled */
};

/*
 * The datasheet gives the window start rule for 8 bpp row scans in either
 * X direction, but its 8n + 2 stride rule for column scans only for
 * 16 bpp, so no chip offers a column scan at 8 bpp.
 */
#define MQ11XX_SCAN_ROWS	GENMASK(MQ11XX_SCAN_COLUMNS - 1, 0)

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
		.transforms = MQ11XX_SCAN_ALL,
		.transforms_8bpp = MQ11XX_SCAN_ROWS,
		.ge_8bpp = true,
		.double_8bpp = true,
		.cursor_transforms = MQ11XX_SCAN_ALL,
		.cursor_double = true,
	}, {
		/* Covered by the same datasheet as the MQ1132. */
		.device_id = MQ11XX_PCI_DEVICE_1100,
		.name = "MQ1100",
		.ge_base = MQ1132_GE_BASE,
		.caps = MQ1132_CAPS,
		.transforms = MQ11XX_SCAN_ALL,
		.transforms_8bpp = MQ11XX_SCAN_ROWS,
		.ge_8bpp = true,
		.double_8bpp = true,
		.cursor_transforms = MQ11XX_SCAN_ALL,
		.cursor_double = true,
	}, {
		/*
		 * Measured on an h2200: engine fill and copy (including
		 * overlapping copies), seven of the eight scanout transforms,
		 * pixel doubling, 8 bpp through the palette, the cursor fetch
		 * at 0x3fc00 and the IN02 frame latch. Five GC0C writes made
		 * mid-frame each moved the fetch at line 1 of the next frame,
		 * never within the frame. The column scan with both axes
		 * reversed was never set, and the alternate window takes its
		 * registers but was never shown to scan out, so both stay
		 * off.
		 */
		.device_id = MQ11XX_PCI_DEVICE_1188,
		.name = "MQ1188",
		.ge_base = MQ1188_GE_BASE,
		.caps = (MQ1132_CAPS & ~MQ11XX_CAP_ALT_WINDOW) |
			MQ11XX_CAP_START_LATCH,
		.transforms = MQ11XX_SCAN_ALL &
			      ~BIT(MQ11XX_SCAN_COLUMNS | MQ11XX_SCAN_Y_DECREMENT |
				   MQ11XX_SCAN_X_DECREMENT),
		/*
		 * 8 bpp was fetched only from an upright frame, and the
		 * engine and pixel doubling were only run at 16 bpp.
		 */
		.transforms_8bpp = BIT(0),
		/*
		 * The cursor was only fetched over an upright, undoubled
		 * scan, so it is not offered together with a transform or
		 * pixel doubling.
		 */
		.cursor_transforms = BIT(0),
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
	 * The display timing that .envee_regs programs, used instead of
	 * .mode when the delta is applied at probe.
	 */
	const struct drm_display_mode *envee_mode;
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

/*
 * The damage of one commit, as up to MQ11XX_FLIP_RECTS rectangles or, once
 * there are more, their bounding box.
 */
#define MQ11XX_FLIP_RECTS	8

struct mq11xx_flip_damage {
	unsigned int n;
	bool merged;		/* r[0] bounds more rectangles than fit */
	struct drm_rect r[MQ11XX_FLIP_RECTS];
};

struct mq11xx_device {
	struct drm_device drm;

	const struct mq11xx_variant *variant;
	const struct mq11xx_chip *chip;
	/* The variant's mode for the panel chosen at probe. */
	const struct drm_display_mode *mode;
	/* The last register load applied the variant's .envee_regs. */
	bool envee_applied;

	/*
	 * Frame interrupt, or 0 to run vblank from the timer. irq_lock
	 * covers irq_live (the chip is powered and scanning out, so its
	 * registers may be touched from atomic context) and irq_wanted
	 * (DRM has vblank enabled), and orders the interrupt controller
	 * writes against the handler.
	 */
	int irq;
	spinlock_t irq_lock;	/* irq_live, irq_wanted, in00-in02 */
	bool irq_live;
	bool irq_wanted;

	struct drm_plane primary_plane;
	struct drm_plane cursor_plane;	/* initialised if mq11xx_has_cursor() */
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;

	void __iomem *vram;
	resource_size_t vram_size;
	phys_addr_t vram_phys;
	bool vram_wc;
	void __iomem *regs;
	/*
	 * Whether GC00[15:14] double the frame; bring-up leaves them clear.
	 */
	bool scan_double;
	/*
	 * Bytes per pixel of the depth the registers hold, and whether the
	 * palette has lost its contents since it was last loaded; bring-up
	 * leaves RGB565 and an undefined palette. Only the commit tail
	 * changes them after that.
	 */
	unsigned int scan_cpp;
	bool palette_stale;
	/*
	 * Double buffering. scan_base is where the frame on screen is stored:
	 * 0, or the back buffer slot at the top of the off-screen area.
	 * flip_valid: the other buffer holds the frame shown before the
	 * current one, in the current layout, and flip_prev is the damage that
	 * turned it into the current one. Bring-up leaves 0 and no valid
	 * buffer; only the commit tail changes them after that.
	 */
	size_t scan_base;
	bool flip_valid;
	struct mq11xx_flip_damage flip_prev;

	/*
	 * The cursor image in video memory, word by word, valid while
	 * cursor_loaded is set. Every bring-up clears video memory and so
	 * clears cursor_loaded; only the commit tail changes them after that.
	 */
	bool cursor_loaded;
	u32 cursor_image[MQ11XX_CURSOR_WORDS];

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

	/*
	 * The scanout transform the registers hold (MQ11XX_SCAN_*). Every
	 * bring-up leaves the plain row scan; only the commit tail changes it.
	 */
	unsigned int scan_transform;

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
	 * fg/bg, written by the cursor plane) are skipped per mqInitValid.
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

/*
 * 3.8" QVGA TFT, portrait: h5xxx's UI runs portrait natively, matching
 * the GC08/GC09 window registers below (240 wide, 0xef=239 / 0x13f=319)
 * with no rotation. The 57 x 77 mm glass size is the figure given for the
 * whole h3xxx and h5xxx family, not measured on these units.
 *
 * The timings are the ones each panel's register table programs, decoded
 * with the MQ1132 datasheet's encodings: horizontal total = gc02[10:0] + 2,
 * vertical total = gc03[9:0] + 1, sync positions from gc04/gc05. The pixel
 * clock is GRCLK / (FD x SD) from gc00[22:20] and gc00[27:24], with GRCLK
 * taken as its documented 48 MHz maximum; no h5xxx frame period has been
 * measured, so the refresh rates (about 60 Hz for both panels) are
 * predictions. The same assumption applied to the h2200 table predicts 59
 * Hz where 56.35 Hz is measured.
 */
static const struct drm_display_mode mq11xx_mode_h5xxx_sharp = {
	.type = DRM_MODE_TYPE_DRIVER,
	.clock = 6000,		/* 48 MHz / (1 x 8) */
	.hdisplay = 240,
	.hsync_start = 250,
	.hsync_end = 273,
	.htotal = 284,
	.vdisplay = 320,
	.vsync_start = 344,
	.vsync_end = 346,
	.vtotal = 352,
	.width_mm = 57,
	.height_mm = 77,
};

static const struct drm_display_mode mq11xx_mode_h5xxx_philips = {
	.type = DRM_MODE_TYPE_DRIVER,
	.clock = 6400,		/* 48 MHz / (1.5 x 5) */
	.hdisplay = 240,
	.hsync_start = 306,
	.hsync_end = 311,
	.htotal = 317,
	.vdisplay = 320,
	.vsync_start = 336,
	.vsync_end = 337,
	.vtotal = 337,
	.width_mm = 57,
	.height_mm = 77,
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
	/*
	 * GE0A/GE0B (a 640-byte destination stride, base 0) are left out:
	 * they match no mode this driver sets, and the engine pitch and base
	 * belong to whatever issues engine commands.
	 */
};

/*
 * Register values of the Philips panel that differ from the Sharp table
 * above (cc02, gc00, gc02-gc05, gc0b, fp01, fp05 and fp06), applied on top
 * of it when the ENVEE pin reports a Philips panel
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
	 * The panel is chosen at every bring-up from the ENVEE pin (see
	 * mq11xx_apply_envee_delta()). On an h5550 the pin reads clear, i.e.
	 * Sharp, and Sharp timing is the one that gives a readable picture
	 * there. A Philips FP register read left by the previous firmware does
	 * not decide this unit, since it may come from a different h5550.
	 */
	.expect_device_id = MQ11XX_PCI_DEVICE_1132,
	.tolerate_device_id = MQ11XX_PCI_DEVICE_1100,
	.mode = &mq11xx_mode_h5xxx_sharp,
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
	.envee_mode = &mq11xx_mode_h5xxx_philips,
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

	mq->envee_applied = false;
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
	mq->envee_applied = true;

	return 0;
}

static void mq11xx_select_chip(struct mq11xx_device *mq, u16 device)
{
	const struct mq11xx_chip *chip = &mq11xx_chip_unknown;
	char caps[128] = "";
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
 * Program the interrupt controller for the current irq_live/irq_wanted
 * state. The stale end-of-frame latch is cleared before the output is
 * enabled, so the first event marks a real frame end; either way the
 * output then starts high and every frame end is a falling edge, which
 * is all a PXA GPIO can detect.
 */
static void mq11xx_irq_update(struct mq11xx_device *mq)
{
	lockdep_assert_held(&mq->irq_lock);

	if (!mq->irq_live)
		return;

	if (mq->irq_wanted) {
		writel(MQ11XX_IC_VENABLE_FALLING, mq->regs + MQ11XX_IC_MASK);
		writel(MQ11XX_IC_VENABLE_FALLING, mq->regs + MQ11XX_IC_STATUS);
		writel(MQ11XX_IC_OUTPUT_ENABLE, mq->regs + MQ11XX_IC_CONTROL);
	} else {
		writel(0, mq->regs + MQ11XX_IC_CONTROL);
		writel(0, mq->regs + MQ11XX_IC_MASK);
	}
}

/*
 * Tell the interrupt code whether the chip is up. Nothing may touch its
 * registers from atomic context while it is unpowered: a bus access to
 * an unpowered chip is never valid on this static-memory bus.
 */
static void mq11xx_irq_set_live(struct mq11xx_device *mq, bool live)
{
	unsigned long flags;

	if (!mq->irq)
		return;

	spin_lock_irqsave(&mq->irq_lock, flags);
	if (live) {
		mq->irq_live = true;
		mq11xx_irq_update(mq);
	} else if (mq->irq_live) {
		writel(0, mq->regs + MQ11XX_IC_CONTROL);
		writel(0, mq->regs + MQ11XX_IC_MASK);
		mq->irq_live = false;
	}
	spin_unlock_irqrestore(&mq->irq_lock, flags);
}

static irqreturn_t mq11xx_irq_handler(int irq, void *arg)
{
	struct mq11xx_device *mq = arg;
	bool frame_end = false;

	spin_lock(&mq->irq_lock);
	if (mq->irq_live && mq->irq_wanted &&
	    readl(mq->regs + MQ11XX_IC_STATUS) & MQ11XX_IC_VENABLE_FALLING) {
		writel(MQ11XX_IC_VENABLE_FALLING, mq->regs + MQ11XX_IC_STATUS);
		frame_end = true;
	}
	spin_unlock(&mq->irq_lock);

	if (!frame_end)
		return IRQ_NONE;

	drm_crtc_handle_vblank(&mq->crtc);

	return IRQ_HANDLED;
}

static int mq11xx_crtc_enable_vblank_irq(struct drm_crtc *crtc)
{
	struct mq11xx_device *mq = to_mq11xx(crtc->dev);
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&mq->irq_lock, flags);
	if (mq->irq_live) {
		mq->irq_wanted = true;
		mq11xx_irq_update(mq);
	} else {
		/* Bring-up failed: no frame will ever end. */
		ret = -EIO;
	}
	spin_unlock_irqrestore(&mq->irq_lock, flags);

	return ret;
}

static void mq11xx_crtc_disable_vblank_irq(struct drm_crtc *crtc)
{
	struct mq11xx_device *mq = to_mq11xx(crtc->dev);
	unsigned long flags;

	spin_lock_irqsave(&mq->irq_lock, flags);
	mq->irq_wanted = false;
	mq11xx_irq_update(mq);
	spin_unlock_irqrestore(&mq->irq_lock, flags);
}

/*
 * Use the chip's frame interrupt if the devicetree describes one and the
 * chip is known to latch its end-of-frame source; otherwise vblank runs
 * from the timer. Needs the chip identified, so runs after the first
 * mq11xx_hw_init().
 */
static int mq11xx_irq_init(struct mq11xx_device *mq, struct platform_device *pdev)
{
	int irq;

	irq = platform_get_irq_optional(pdev, 0);
	if (irq == -ENXIO)
		return 0;
	if (irq < 0)
		return dev_err_probe(&pdev->dev, irq, "failed to get interrupt\n");

	if (!(mq->chip->caps & MQ11XX_CAP_FRAME_LATCH)) {
		drm_info(&mq->drm, "%s: frame interrupt not known to work, using the vblank timer\n",
			 mq->chip->name);
		return 0;
	}

	mq->irq = irq;

	return 0;
}

/*
 * One-time chip bring-up: endianness quirk, oscillator enable (100 ms; 30 ms
 * is not enough), CC-module power (10 ms), the DC.config_8 value, MIU
 * enable, interrupt output off (until mq11xx_output_enable() hands the chip
 * to the vblank code), the PCI-header identity read, then the full register
 * table load. Scanout stays off until the first atomic_enable
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

	/* Output off until mq11xx_irq_update() asks for frame interrupts. */
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
				  mq->mode->hdisplay * 2);
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
 * Copies the rectangle of @dst's size at (@sx, @sy) to @dst, within the
 * current surface. Overlap is handled as the WinCE drivers do it: when the
 * destination lies right of or below the source, the engine walks that
 * axis backwards, starting from the far edge of both rectangles.
 */
static int mq11xx_ge_copy(struct mq11xx_device *mq, const struct drm_rect *dst,
			  int sx, int sy)
{
	unsigned int w = drm_rect_width(dst), h = drm_rect_height(dst);
	struct drm_rect src = DRM_RECT_INIT(sx, sy, w, h);
	u32 cmd = MQ11XX_GE_ROP_SRCCOPY | MQ11XX_GE_TYPE_BITBLT;
	int dx = dst->x1, dy = dst->y1;

	lockdep_assert_held(&mq->ge_lock);

	if (!mq11xx_ge_capable(mq) || !mq->ge_surface_valid)
		return -ENODEV;
	if (!mq11xx_ge_rect_valid(dst) || !mq11xx_ge_rect_valid(&src))
		return -EINVAL;

	if (dx > sx) {
		cmd |= MQ11XX_GE_X_NEGATIVE;
		dx += w - 1;
		sx += w - 1;
	}
	if (dy > sy) {
		cmd |= MQ11XX_GE_Y_NEGATIVE;
		dy += h - 1;
		sy += h - 1;
	}

	mq11xx_ge_write(mq, MQ11XX_GE_SIZE, mq11xx_ge_xy(w, h));
	mq11xx_ge_write(mq, MQ11XX_GE_DEST, mq11xx_ge_xy(dx, dy));
	mq11xx_ge_write(mq, MQ11XX_GE_SOURCE, mq11xx_ge_xy(sx, sy));
	mq11xx_ge_write(mq, MQ11XX_GE_COMMAND, cmd);

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

/* Whether the @r pixels at @vaddr, @pitch bytes per row, are all one index. */
static bool mq11xx_rect_is_solid8(const void *vaddr, unsigned int pitch,
				  const struct drm_rect *r, u32 *colour)
{
	unsigned int w = drm_rect_width(r), h = drm_rect_height(r), y;
	u8 c = *(const u8 *)vaddr;

	for (y = 0; y < h; y++)
		if (memchr_inv(vaddr + y * pitch, c, w))
			return false;

	*colour = c;
	return true;
}

/*
 * Draws a damage rectangle whose shadow pixels are all one colour with an
 * engine fill instead of uploading it: clears, letterbox bars and blank
 * console lines then cost no host bus traffic. The shadow buffer is
 * cached system memory, and the scan stops at the first differing pixel.
 * The engine surface is the frame as the CPU path writes it, @base bytes
 * into video memory and unrotated at @vram_pitch whatever the scanout
 * transform. Returns 0 when the engine drew the rectangle.
 */
static int mq11xx_ge_fill_damage(struct mq11xx_device *mq,
				 struct drm_framebuffer *fb,
				 const struct iosys_map *data,
				 const struct drm_rect *damage,
				 unsigned int vram_pitch, size_t base)
{
	unsigned int pitch = fb->pitches[0], cpp = fb->format->cpp[0];
	const void *vaddr;
	bool solid;
	u32 colour;
	int ret;

	if (!mq11xx_ge_wanted(mq) || (cpp == 1 && !mq->chip->ge_8bpp) || data->is_iomem ||
	    drm_rect_width(damage) * drm_rect_height(damage) < MQ11XX_GE_FILL_MIN_PIXELS)
		return -ENODEV;

	vaddr = data->vaddr + drm_fb_clip_offset(pitch, fb->format, damage);
	if (cpp == 1)
		solid = mq11xx_rect_is_solid8(vaddr, pitch, damage, &colour);
	else
		solid = mq11xx_rect_is_solid16(vaddr, pitch, damage, &colour);
	if (!solid)
		return -EINVAL;

	mutex_lock(&mq->ge_lock);
	ret = mq11xx_ge_set_surface(mq, base, vram_pitch, cpp);
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

	/*
	 * Both register tables clear GC00[15:12] and start the window at the
	 * frame at 16 bpp, and mq11xx_chip_init() set the row pitch as the
	 * stride.
	 */
	mq->scan_transform = 0;
	mq->scan_double = false;
	mq->scan_cpp = 2;
	mq->scan_base = MQ11XX_SRAM_FRAME;
	mq->flip_valid = false;
	mq->palette_stale = true;
	mq->cursor_loaded = false;
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
static bool mq11xx_use_panel_seq(struct mq11xx_device *mq)
{
	return READ_ONCE(mq11xx_panel_seq) && mq->variant->envee_regs &&
	       !mq->envee_applied;
}

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

	if (mq11xx_use_panel_seq(mq)) {
		val = readl(mq->regs + MQ11XX_CC_GPIO_CONTROL);
		writel(val & ~MQ11XX_CC_GPIO1_HIGH,
		       mq->regs + MQ11XX_CC_GPIO_CONTROL);
		ret = regulator_enable(mq->panel);
		if (ret) {
			drm_err(&mq->drm, "failed to enable panel supply (%d), display stays off\n",
				ret);
			mq11xx_power_down(mq);
			mq->powered = false;
			goto out_unlock;
		}
		writel(mq->variant->fp_output_control,
		       mq->regs + MQ11XX_FP_OUTPUT_CONTROL);
		val = readl(mq->regs + MQ11XX_GC_CONTROL);
		writel(val | MQ11XX_GC_CONTROL_ENABLE, mq->regs + MQ11XX_GC_CONTROL);
		val = readl(mq->regs + MQ11XX_DC_CONFIG_5);
		writel(val | MQ11XX_CONFIG_GE_ENABLE, mq->regs + MQ11XX_DC_CONFIG_5);
		msleep(40);
		val = readl(mq->regs + MQ11XX_CC_GPIO_CONTROL);
		writel(val | MQ11XX_CC_GPIO1_HIGH, mq->regs + MQ11XX_CC_GPIO_CONTROL);
		msleep(40);
		goto live;
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

live:
	mq11xx_irq_set_live(mq, true);

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

	mq11xx_irq_set_live(mq, false);

	if (mq11xx_use_panel_seq(mq)) {
		val = readl(mq->regs + MQ11XX_FP_OUTPUT_CONTROL);
		writel(val & ~BIT(28), mq->regs + MQ11XX_FP_OUTPUT_CONTROL);
		val = readl(mq->regs + MQ11XX_FP_PIN_CONTROL_1);
		writel(val & ~BIT(3), mq->regs + MQ11XX_FP_PIN_CONTROL_1);
		writel(0x00fcfcfd, mq->regs + MQ11XX_FP_GPIO_CONTROL_0);
		val = readl(mq->regs + MQ11XX_FP_GPIO_CONTROL_1);
		writel((val & 1) | 0x00fcfcfc, mq->regs + MQ11XX_FP_GPIO_CONTROL_1);
		msleep(63);
		val = readl(mq->regs + MQ11XX_CC_GPIO_CONTROL);
		writel(val & ~MQ11XX_CC_GPIO1_HIGH, mq->regs + MQ11XX_CC_GPIO_CONTROL);
		msleep(63);
		regulator_disable(mq->panel);
		writel(0xfffcfcfd, mq->regs + MQ11XX_FP_GPIO_CONTROL_0);
		val = readl(mq->regs + MQ11XX_FP_GPIO_CONTROL_1);
		writel(val & 0x00030302, mq->regs + MQ11XX_FP_GPIO_CONTROL_1);
		val = readl(mq->regs + MQ11XX_FP_PIN_CONTROL_1);
		writel(val | BIT(0), mq->regs + MQ11XX_FP_PIN_CONTROL_1);
	} else {
		/* Blank the panel before tearing down scanout -- see the
		 * mirrored comment in mq11xx_output_enable() above.
		 */
		regulator_disable(mq->panel);
	}

	/*
	 * The cursor goes with scanout; the cursor plane turns it back on
	 * when the unblank's modeset updates it.
	 */
	val = readl(mq->regs + MQ11XX_GC_CONTROL);
	writel(val & ~(MQ11XX_GC_CONTROL_ENABLE | MQ11XX_GC_CONTROL_CURSOR),
	       mq->regs + MQ11XX_GC_CONTROL);

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
 * Fatal to probe(): a backlight using this PWM can never bind without it, and
 * the display waits for its backlight in mq11xx_display_probe(), so carrying
 * on would leave the display deferred for ever with no error to show for it.
 */
static int mq11xx_pwm_chip_register(struct mq11xx_device *mq, struct device *dev)
{
	struct pwm_chip *chip;
	int ret;

	/* Probe just powered the chip, so fp0f holds the table's level. */
	mutex_lock(&mq->hw_lock);
	mq->pwm_level = mq11xx_pwm_read_level(mq);
	mutex_unlock(&mq->hw_lock);

	chip = devm_pwmchip_alloc(dev, 1, 0);
	if (IS_ERR(chip))
		return dev_err_probe(dev, PTR_ERR(chip), "failed to allocate PWM chip\n");

	chip->ops = &mq11xx_pwm_ops;

	ret = devm_pwmchip_add(dev, chip);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add PWM chip\n");

	return 0;
}

/* ------------------------------------------------------------------ */
/* Scanout rotation and reflection */

/* Longest wait for vertical blanking, two frames at the slowest mode. */
#define MQ11XX_BLANK_POLL_US		100
#define MQ11XX_BLANK_TIMEOUT_US		50000

/*
 * The scan showing a DRM rotation, which reflects the framebuffer and then
 * turns it counter-clockwise. In a column scan with GC00[12] clear the
 * displayed lines start at successive stored columns from the left, and a
 * negative stride runs each line from the last stored row up: that is how
 * an MQ1188's fetch address was measured to step.
 */
static unsigned int mq11xx_scan_transform(unsigned int rotation)
{
	bool rx = rotation & DRM_MODE_REFLECT_X;
	bool ry = rotation & DRM_MODE_REFLECT_Y;
	unsigned int t = 0;

	switch (rotation & DRM_MODE_ROTATE_MASK) {
	case DRM_MODE_ROTATE_90:
		t = MQ11XX_SCAN_COLUMNS;
		rx = !rx;
		break;
	case DRM_MODE_ROTATE_180:
		rx = !rx;
		ry = !ry;
		break;
	case DRM_MODE_ROTATE_270:
		t = MQ11XX_SCAN_COLUMNS;
		ry = !ry;
		break;
	}

	if (rx)
		t |= MQ11XX_SCAN_X_DECREMENT;
	if (ry)
		t |= MQ11XX_SCAN_Y_DECREMENT;

	return t;
}

static bool mq11xx_scan_supported(struct mq11xx_device *mq, unsigned int t)
{
	return !t || ((mq->chip->caps & MQ11XX_CAP_ROTATION) &&
		      (mq->chip->transforms & BIT(t)));
}

static unsigned int mq11xx_supported_rotations(struct mq11xx_device *mq)
{
	static const unsigned int each[] = {
		DRM_MODE_ROTATE_90,
		DRM_MODE_ROTATE_180,
		DRM_MODE_ROTATE_270,
		DRM_MODE_ROTATE_0 | DRM_MODE_REFLECT_X,
		DRM_MODE_ROTATE_0 | DRM_MODE_REFLECT_Y,
	};
	unsigned int supported = DRM_MODE_ROTATE_0, i;

	for (i = 0; i < ARRAY_SIZE(each); i++)
		if (mq11xx_scan_supported(mq, mq11xx_scan_transform(each[i])))
			supported |= each[i];

	return supported;
}

/*
 * Pitch of the stored frame at @cpp bytes per pixel. The datasheet
 * requires a column scan's stride to be 8n + 2 bytes, so there each row is
 * padded past its 8-byte-aligned length: 642 bytes for a 320-pixel row,
 * 154,080 bytes for the frame, 480 more than an unpadded one. Column
 * scans are offered at 16 bpp only. A doubled frame (@dbl) is half the
 * display's width and height, and only ever scanned upright.
 */
static unsigned int mq11xx_scan_pitch(struct mq11xx_device *mq, unsigned int t,
				      unsigned int cpp, bool dbl)
{
	const struct drm_display_mode *mode = mq->mode;

	if (t & MQ11XX_SCAN_COLUMNS)
		return ALIGN(mode->vdisplay * 2, 8) + 2;

	return (mode->hdisplay >> dbl) * cpp;
}

/*
 * Waits until the beam has left the window, so that registers written next
 * change the picture between two frames rather than within one.
 */
static void mq11xx_wait_blank(struct mq11xx_device *mq)
{
	u32 val;

	if (readl_poll_timeout(mq->regs + MQ11XX_GC_VERTICAL_PARAMETER, val,
			       FIELD_GET(MQ11XX_GC_VCOUNT, val) > mq->mode->vdisplay,
			       MQ11XX_BLANK_POLL_US, MQ11XX_BLANK_TIMEOUT_US))
		drm_dbg_kms(&mq->drm, "no vertical blanking seen, changing the scan mid-frame\n");
}

/*
 * The window start (GC0C) and stride for the frame stored @base bytes into
 * video memory, for transform @t at @cpp bytes per pixel, doubled when @dbl
 * is set. The window starts at the stored pixel shown top left: the last
 * pixel of a row for an x decrement, the first of the last row for a y
 * decrement. For the column scans that is where an MQ1188's fetch was
 * measured to begin, and where the h5550's WinCE driver starts the one it
 * calls 90 degrees (it starts the other one a byte further on); the start
 * corners that Table 4-7 of the datasheet gives for those two would run
 * the fetch outside the frame.
 */
static u32 mq11xx_scan_start(struct mq11xx_device *mq, unsigned int t,
			     unsigned int cpp, bool dbl, size_t base,
			     u32 *stride)
{
	const struct drm_display_mode *mode = mq->mode;
	bool columns = t & MQ11XX_SCAN_COLUMNS;
	unsigned int width = (columns ? mode->vdisplay : mode->hdisplay) >> dbl;
	unsigned int rows = (columns ? mode->hdisplay : mode->vdisplay) >> dbl;
	unsigned int pitch = mq11xx_scan_pitch(mq, t, cpp, dbl);
	u32 start = base;

	*stride = pitch;
	if (t & MQ11XX_SCAN_X_DECREMENT)
		start += (width - 1) * cpp;
	if (t & MQ11XX_SCAN_Y_DECREMENT) {
		start += (rows - 1) * pitch;
		*stride = -*stride;
	}

	return start;
}

/*
 * Points the scan at the frame stored @base bytes into video memory, for
 * transform @t at @cpp bytes per pixel, doubled when @dbl is set.
 *
 * Doubling leaves the window registers GC08/GC09 at the display's size:
 * with both GC00[15:14] set, an MQ1188 was measured to fetch 120 pixels
 * per line and each stored line for two display lines, reading exactly a
 * 120x160 frame for its 240x320 window. Only the stride follows the
 * smaller frame.
 *
 * Only GC0C is known to take effect at a frame boundary, so all three
 * registers are written once the beam has left the window, as the WinCE
 * driver does.
 */
static void mq11xx_scan_program(struct mq11xx_device *mq, unsigned int t,
				unsigned int cpp, bool dbl, size_t base)
{
	bool columns = t & MQ11XX_SCAN_COLUMNS;
	u32 start, stride, control;

	start = mq11xx_scan_start(mq, t, cpp, dbl, base, &stride);

	control = readl(mq->regs + MQ11XX_GC_CONTROL) &
		  ~(MQ11XX_GC_CONTROL_COLUMN_SCAN | MQ11XX_GC_CONTROL_X_DECREMENT |
		    MQ11XX_GC_CONTROL_DOUBLE_H | MQ11XX_GC_CONTROL_DOUBLE_V |
		    MQ11XX_GC_CONTROL_DEPTH);
	control |= FIELD_PREP(MQ11XX_GC_CONTROL_DEPTH,
			      cpp == 1 ? MQ11XX_GC_DEPTH_8BPP : MQ11XX_GC_DEPTH_16BPP);
	if (columns)
		control |= MQ11XX_GC_CONTROL_COLUMN_SCAN;
	if (t & MQ11XX_SCAN_X_DECREMENT)
		control |= MQ11XX_GC_CONTROL_X_DECREMENT;
	if (dbl)
		control |= MQ11XX_GC_CONTROL_DOUBLE_H | MQ11XX_GC_CONTROL_DOUBLE_V;

	mq11xx_wait_blank(mq);

	writel(control, mq->regs + MQ11XX_GC_CONTROL);
	writel(stride & MQ11XX_GC_STRIDE, mq->regs + MQ11XX_GC_WINDOW_STRIDE);
	writel(start, mq->regs + MQ11XX_GC_WINDOW_START);

	mq->scan_transform = t;
	mq->scan_cpp = cpp;
	mq->scan_double = dbl;
	mq->scan_base = base;
}

/*
 * Flips the scan to the frame stored @base bytes into video memory, in the
 * layout it already has. The commit's vblank event goes out at the next
 * frame end, after which the old frame must not be read again. An MQ1188
 * was measured to take a GC0C write into use at the start of the next
 * frame wherever the beam was, so when the frame interrupt sends the
 * events the write is made at once: a write before the frame end is in use
 * by the frame after it, and a later one delays the event by a frame. The
 * vblank timer is not tied to the scan, and other chips were never
 * measured, so there the write waits until the beam has left the window.
 */
static void mq11xx_scan_flip(struct mq11xx_device *mq, size_t base)
{
	u32 start, stride;

	start = mq11xx_scan_start(mq, mq->scan_transform, mq->scan_cpp,
				  mq->scan_double, base, &stride);

	if (!mq->irq || !(mq->chip->caps & MQ11XX_CAP_START_LATCH))
		mq11xx_wait_blank(mq);

	writel(start, mq->regs + MQ11XX_GC_WINDOW_START);
	mq->scan_base = base;
}

/*
 * Brings the scan to the frame stored @base bytes into video memory, for
 * transform @t at @cpp bytes per pixel, doubled when @dbl is set.
 */
static void mq11xx_scan_update(struct mq11xx_device *mq, unsigned int t,
			       unsigned int cpp, bool dbl, size_t base)
{
	if (t != mq->scan_transform || cpp != mq->scan_cpp ||
	    dbl != mq->scan_double)
		mq11xx_scan_program(mq, t, cpp, dbl, base);
	else if (base != mq->scan_base)
		mq11xx_scan_flip(mq, base);
}

/* The off-screen area ends at the cursor slot, or earlier in a smaller window. */
static size_t mq11xx_pool_end(struct mq11xx_device *mq)
{
	return min_t(resource_size_t, mq->vram_size, MQ11XX_SRAM_CURSOR);
}

/*
 * Where the back buffer goes for a frame of @rows rows of @pitch bytes
 * stored for transform @t, or 0 when the frame is not double-buffered. It
 * sits at the top of the off-screen area, clear of the cursor slot and of
 * any full-size frame at offset 0, so the scan can go on showing either
 * while the back buffer is filled. It starts on a whole row, so that one
 * engine surface at offset 0 covers both buffers, and on a whole 64-bit
 * word, as the engine's base address register wants.
 */
static size_t mq11xx_flip_slot(struct mq11xx_device *mq, unsigned int t,
			       unsigned int pitch, unsigned int rows)
{
	size_t frame = (size_t)pitch * rows, end = mq11xx_pool_end(mq), slot;

	if (!READ_ONCE(mq11xx_page_flip) || (t & MQ11XX_SCAN_COLUMNS) ||
	    end < 2 * frame)
		return 0;

	slot = rounddown(end - frame, lcm(pitch, 8));

	return slot >= frame ? slot : 0;
}

/* ------------------------------------------------------------------ */
/* Palette */

static void mq11xx_palette_set(struct drm_crtc *crtc, unsigned int index,
			       u16 red, u16 green, u16 blue)
{
	struct mq11xx_device *mq = to_mq11xx(crtc->dev);

	writel(FIELD_PREP(MQ11XX_PALETTE_RED, red >> 8) |
	       FIELD_PREP(MQ11XX_PALETTE_GREEN, green >> 8) |
	       FIELD_PREP(MQ11XX_PALETTE_BLUE, blue >> 8),
	       mq->regs + MQ11XX_PALETTE + index * sizeof(u32));
}

/*
 * Loads the palette from @crtc_state's gamma LUT, or a grey ramp without
 * one. Changing entries the scan is using shows as a band of wrong colours,
 * so they are written once the beam has left the window, as the WinCE
 * display drivers' SetPalette does. The 256 register writes take a small
 * fraction of the vertical blanking interval.
 */
static void mq11xx_palette_load(struct mq11xx_device *mq,
				const struct drm_crtc_state *crtc_state)
{
	mq11xx_wait_blank(mq);

	if (crtc_state->gamma_lut)
		drm_crtc_load_palette_8(&mq->crtc, crtc_state->gamma_lut->data,
					mq11xx_palette_set);
	else
		drm_crtc_fill_palette_8(&mq->crtc, mq11xx_palette_set);

	mq->palette_stale = false;
}

/* ------------------------------------------------------------------ */
/* Shadow-plane atomic pipeline, structurally identical to cirrus-qemu.c's */

/*
 * RGB565 stays first: the palette is bypassed at 16 bpp, and fbdev
 * emulation takes the preferred depth. C8 is offered only by chips with
 * MQ11XX_CAP_PALETTE_8BPP and halves the bytes a frame puts on the host
 * bus, 76,800 instead of 153,600 at 240x320.
 */
static const u32 mq11xx_primary_plane_formats[] = {
	DRM_FORMAT_RGB565,
	DRM_FORMAT_C8,
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
	const struct drm_display_mode *mode = mq->mode;
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
 * go. hw_lock is deliberately not held across the wait: it would serialise
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

/* src:dst of a doubled plane, in 16.16 fixed point */
#define MQ11XX_DOUBLE_SCALE	(DRM_PLANE_NO_SCALING / 2)

/*
 * atomic_check only lets a plane's source be smaller than its full-screen
 * destination when it is exactly half of it in both axes. The source stays
 * in framebuffer coordinates, so a quarter turn pairs its width with the
 * destination's height.
 */
static bool mq11xx_plane_doubled(const struct drm_plane_state *state)
{
	unsigned int w = drm_rotation_90_or_270(state->rotation) ? state->crtc_h :
								    state->crtc_w;

	return state->src_w >> 16 != w;
}

static int mq11xx_primary_plane_helper_atomic_check(struct drm_plane *plane,
						    struct drm_atomic_commit *state)
{
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_plane_state *old_plane_state = drm_atomic_get_old_plane_state(state, plane);
	struct drm_crtc *new_crtc = new_plane_state->crtc;
	struct drm_crtc_state *new_crtc_state = NULL;
	struct mq11xx_device *mq = to_mq11xx(plane->dev);
	const struct drm_display_mode *mode = mq->mode;
	unsigned int rotation = new_plane_state->rotation;
	struct drm_framebuffer *fb = new_plane_state->fb;
	bool can_double = mq->chip->caps & MQ11XX_CAP_PIXEL_DOUBLE;
	unsigned int t, width, height;
	bool dbl;
	int ret;

	if (READ_ONCE(mq11xx_present_dma) && !mq->present_dma.tried)
		mq11xx_present_dma_setup(mq);

	if (new_crtc)
		new_crtc_state = drm_atomic_get_new_crtc_state(state, new_crtc);

	/* GC00[15:14] double the whole window, so the plane stays full screen. */
	ret = drm_atomic_helper_check_plane_state(new_plane_state, new_crtc_state,
						  can_double ? MQ11XX_DOUBLE_SCALE :
							       DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING,
						  false, false);
	if (ret || !new_plane_state->visible)
		return ret;

	/*
	 * The helper admits any ratio in between; only 1:1 or 2x in both axes
	 * are real. A column scan shows the framebuffer's rows as display
	 * columns, so its source is the display turned a quarter.
	 */
	t = mq11xx_scan_transform(rotation);
	dbl = mq11xx_plane_doubled(new_plane_state);
	width = t & MQ11XX_SCAN_COLUMNS ? mode->vdisplay : mode->hdisplay;
	height = t & MQ11XX_SCAN_COLUMNS ? mode->hdisplay : mode->vdisplay;
	if (new_plane_state->src_w != (width >> dbl) << 16 ||
	    new_plane_state->src_h != (height >> dbl) << 16 ||
	    (dbl && (width % 2 || height % 2))) {
		drm_dbg_atomic(plane->dev, "source 0x%x x 0x%x, scaling is 1:1 or exactly 2x\n",
			       new_plane_state->src_w, new_plane_state->src_h);
		return -EINVAL;
	}

	/* Doubling was never measured together with a rotated or reflected scan. */
	if (dbl && t) {
		drm_dbg_atomic(plane->dev, "pixel doubling with rotation 0x%x not supported\n",
			       rotation);
		return -EINVAL;
	}
	if (dbl && fb->format->cpp[0] == 1 && !mq->chip->double_8bpp) {
		drm_dbg_atomic(plane->dev, "pixel doubling at 8 bpp not supported on the %s\n",
			       mq->chip->name);
		return -EINVAL;
	}
	if (!mq11xx_scan_supported(mq, t)) {
		drm_dbg_atomic(plane->dev, "rotation 0x%x not supported on the %s\n",
			       rotation, mq->chip->name);
		return -EINVAL;
	}
	if (fb->format->cpp[0] == 1 && !(mq->chip->transforms_8bpp & BIT(t))) {
		drm_dbg_atomic(plane->dev, "rotation 0x%x not supported at 8 bpp on the %s\n",
			       rotation, mq->chip->name);
		return -EINVAL;
	}

	/* The frame is stored whole, at offset 0 or in the back buffer. */
	width >>= dbl;
	height >>= dbl;
	if (fb->width != width || fb->height != height) {
		drm_dbg_atomic(plane->dev, "framebuffer %ux%u, rotation 0x%x needs %ux%u\n",
			       fb->width, fb->height, rotation, width, height);
		return -EINVAL;
	}

	/*
	 * The whole frame moves in video memory, or changes its layout. The
	 * flag is copied from the old state on every duplicate, so it has to
	 * be cleared again, or one rotation or format change would turn every
	 * later commit into a full-frame upload.
	 */
	new_plane_state->ignore_damage_clips =
		old_plane_state->rotation != rotation ||
		(old_plane_state->fb && old_plane_state->fb->format != fb->format);

	return 0;
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

/*
 * Draws @damage of the shadow buffer into the frame stored @base bytes into
 * video memory at @pitch: an engine fill when it is all one colour,
 * otherwise a copy by the CPU or, into the frame at offset 0 while
 * present_dma is set (@dma), by the DMA channel.
 */
static void mq11xx_present_rect(struct mq11xx_device *mq,
				struct drm_framebuffer *fb,
				const struct iosys_map *data,
				const struct drm_rect *damage, unsigned int pitch,
				size_t base, bool dma)
{
	struct iosys_map vram_map = IOSYS_MAP_INIT_VADDR_IOMEM(mq->vram);
	unsigned int vram_pitch[DRM_FORMAT_MAX_PLANES] = { pitch };
	size_t offset = base + drm_fb_clip_offset(pitch, fb->format, damage);
	struct iosys_map dst = IOSYS_MAP_INIT_OFFSET(&vram_map, offset);

	if (!mq11xx_ge_fill_damage(mq, fb, data, damage, pitch, base))
		return;

	/* It copies at the framebuffer's pitch, the unrotated frame's. */
	if (dma && base == MQ11XX_SRAM_FRAME && fb->pitches[0] == pitch &&
	    !mq11xx_present_dma_rect(mq, fb, data, damage))
		return;

	drm_fb_memcpy(&dst, vram_pitch, data, fb, damage);
	mq11xx_vram_flush(mq, offset + (drm_rect_height(damage) - 1) * pitch);
}

static void mq11xx_flip_damage_add(struct mq11xx_flip_damage *d,
				   const struct drm_rect *r)
{
	struct drm_rect *b = &d->r[0];
	unsigned int i;

	if (!d->merged && d->n < MQ11XX_FLIP_RECTS) {
		d->r[d->n++] = *r;
		return;
	}

	if (!d->merged) {
		for (i = 1; i < d->n; i++) {
			b->x1 = min(b->x1, d->r[i].x1);
			b->y1 = min(b->y1, d->r[i].y1);
			b->x2 = max(b->x2, d->r[i].x2);
			b->y2 = max(b->y2, d->r[i].y2);
		}
		d->n = 1;
		d->merged = true;
	}

	b->x1 = min(b->x1, r->x1);
	b->y1 = min(b->y1, r->y1);
	b->x2 = max(b->x2, r->x2);
	b->y2 = max(b->y2, r->y2);
}

/* Whether one rectangle of @d, which bounds nothing more, contains @r. */
static bool mq11xx_flip_damage_covers(const struct mq11xx_flip_damage *d,
				      const struct drm_rect *r)
{
	unsigned int i;

	if (d->merged)
		return false;

	for (i = 0; i < d->n; i++)
		if (d->r[i].x1 <= r->x1 && d->r[i].y1 <= r->y1 &&
		    d->r[i].x2 >= r->x2 && d->r[i].y2 >= r->y2)
			return true;

	return false;
}

/*
 * Brings @r of the back buffer at @back up to date with the front buffer
 * at @front before the new damage is drawn into it. The front buffer holds
 * the newer frame everywhere, so the engine copies @r from it, which puts
 * no bytes on the host bus. The copy moves bytes, so it views both buffers
 * as one 16 bpp surface at the frame's pitch, the depth the MQ1188's
 * engine was run at, with @r widened to whole 16-bit pixels; copying a
 * little more of the front buffer is harmless. Without the engine, @r is
 * uploaded from the shadow buffer instead.
 */
static void mq11xx_flip_seed(struct mq11xx_device *mq,
			     struct drm_framebuffer *fb,
			     const struct iosys_map *data,
			     const struct drm_rect *r, unsigned int pitch,
			     size_t front, size_t back)
{
	unsigned int cpp = fb->format->cpp[0];
	unsigned int x1 = r->x1 * cpp / 2, x2 = DIV_ROUND_UP(r->x2 * cpp, 2);
	struct drm_rect dst = DRM_RECT_INIT(x1, back / pitch + r->y1, x2 - x1,
					    drm_rect_height(r));
	int ret = -ENODEV;

	if (mq11xx_ge_wanted(mq)) {
		mutex_lock(&mq->ge_lock);
		ret = mq11xx_ge_set_surface(mq, MQ11XX_SRAM_FRAME, pitch, 2);
		if (!ret)
			ret = mq11xx_ge_copy(mq, &dst, x1, front / pitch + r->y1);
		mutex_unlock(&mq->ge_lock);
	}

	if (ret)
		mq11xx_present_rect(mq, fb, data, r, pitch, back, false);
}

/*
 * A frame that fits twice in video memory is double-buffered: each commit
 * draws into the buffer that is not on screen and then flips the scan to
 * it, so the scan never shows a half-drawn frame. That buffer still holds
 * the frame before the one on screen, so the previous commit's damage is
 * first brought over from the front buffer. The first frame of a new
 * layout, or back at offset 0 after double buffering, is drawn whole into
 * memory the scan is not reading and only then shown.
 */
static void mq11xx_primary_plane_helper_atomic_update(struct drm_plane *plane,
						      struct drm_atomic_commit *state)
{
	struct mq11xx_device *mq = to_mq11xx(plane->dev);
	struct drm_plane_state *plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_shadow_plane_state *shadow_plane_state = to_drm_shadow_plane_state(plane_state);
	const struct iosys_map *data = &shadow_plane_state->data[0];
	struct drm_framebuffer *fb = plane_state->fb;
	struct drm_plane_state *old_plane_state = drm_atomic_get_old_plane_state(state, plane);
	unsigned int t = mq11xx_scan_transform(plane_state->rotation);
	struct mq11xx_flip_damage cur = { };
	struct drm_atomic_helper_damage_iter iter;
	struct drm_rect damage, whole_frame;
	unsigned int pitch, cpp, i;
	size_t slot, target, front;
	bool dma, dbl, flip, whole, same;
	int idx;

	if (!fb)
		return;

	cpp = fb->format->cpp[0];
	dbl = mq11xx_plane_doubled(plane_state);
	pitch = mq11xx_scan_pitch(mq, t, cpp, dbl);
	whole_frame = DRM_RECT_INIT(0, 0, fb->width, fb->height);

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

	slot = mq11xx_flip_slot(mq, t, pitch, fb->height);
	flip = !!slot;
	front = mq->scan_base;
	target = flip && front == MQ11XX_SRAM_FRAME ? slot : MQ11XX_SRAM_FRAME;
	same = t == mq->scan_transform && cpp == mq->scan_cpp &&
	       dbl == mq->scan_double;
	whole = target != front && !(flip && same && mq->flip_valid);

	/*
	 * Read once: every rectangle of this commit is finished before it
	 * returns, so a change through sysfs simply takes effect on the next.
	 */
	dma = READ_ONCE(mq11xx_present_dma);

	if (whole) {
		cur.n = 1;
		cur.r[0] = whole_frame;
	} else {
		drm_atomic_helper_damage_iter_init(&iter, old_plane_state, plane_state);
		drm_atomic_for_each_plane_damage(&iter, &damage)
			mq11xx_flip_damage_add(&cur, &damage);
	}

	if (target == front) {
		/*
		 * A new transform is scanned from the old picture until the
		 * full upload that atomic_check asked for below replaces it.
		 */
		mq11xx_scan_update(mq, t, cpp, dbl, target);
	} else if (!whole) {
		/* Nothing changed: the frame on screen stays, and so does the back. */
		if (!cur.n)
			goto out_exit;

		for (i = 0; i < mq->flip_prev.n; i++)
			if (!mq11xx_flip_damage_covers(&cur, &mq->flip_prev.r[i]))
				mq11xx_flip_seed(mq, fb, data, &mq->flip_prev.r[i],
						 pitch, front, target);
	}

	if (whole) {
		mq11xx_present_rect(mq, fb, data, &whole_frame, pitch, target, dma);
	} else {
		drm_atomic_helper_damage_iter_init(&iter, old_plane_state, plane_state);
		drm_atomic_for_each_plane_damage(&iter, &damage)
			mq11xx_present_rect(mq, fb, data, &damage, pitch, target, dma);
	}

	if (target != front)
		mq11xx_scan_update(mq, t, cpp, dbl, target);

	mq->flip_valid = flip;
	if (flip)
		mq->flip_prev = cur;

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

/* ------------------------------------------------------------------ */
/* Cursor plane on the hardware cursor */

/*
 * The cursor image is converted from the framebuffer in atomic_check, so
 * that an image the hardware cannot show fails the commit and userspace
 * falls back to a software cursor. The masks hold the image as it appears
 * on the display, bit n of a row being pixel n from the left.
 */
struct mq11xx_cursor_state {
	struct drm_plane_state base;
	bool converted;		/* the fields below hold base.fb's image */
	bool mirror;		/* the scan runs each line from its last pixel */
	bool dbl;		/* the scan doubles pixels and lines */
	u32 fg, bg;		/* gc12, gc13 */
	u64 and_mask[MQ11XX_CURSOR_SIZE];
	u64 xor_mask[MQ11XX_CURSOR_SIZE];
};

static struct mq11xx_cursor_state *
to_mq11xx_cursor_state(struct drm_plane_state *state)
{
	return container_of(state, struct mq11xx_cursor_state, base);
}

static bool mq11xx_has_cursor(struct mq11xx_device *mq)
{
	return (mq->chip->caps & MQ11XX_CAP_CURSOR) &&
	       mq->vram_size >= MQ11XX_SRAM_CURSOR + MQ11XX_SRAM_CURSOR_SIZE;
}

static const u32 mq11xx_cursor_plane_formats[] = {
	DRM_FORMAT_ARGB8888,
};

static u32 mq11xx_cursor_colour(u32 argb)
{
	return FIELD_PREP(MQ11XX_GC_CURSOR_RED, (argb >> 18) & 0x3f) |
	       FIELD_PREP(MQ11XX_GC_CURSOR_GREEN, (argb >> 10) & 0x3f) |
	       FIELD_PREP(MQ11XX_GC_CURSOR_BLUE, (argb >> 2) & 0x3f);
}

/*
 * Fully transparent pixels stay transparent and opaque ones take the
 * foreground or the background colour, compared at the six bits per
 * component the colour registers keep. A translucent pixel or a third
 * colour has no representation.
 */
static int mq11xx_cursor_convert(struct drm_plane *plane,
				 struct mq11xx_cursor_state *cstate)
{
	struct drm_plane_state *state = &cstate->base;
	struct drm_framebuffer *fb = state->fb;
	struct iosys_map map[DRM_FORMAT_MAX_PLANES];
	struct iosys_map data[DRM_FORMAT_MAX_PLANES];
	unsigned int sx = state->src_x >> 16, sy = state->src_y >> 16;
	unsigned int w = state->src_w >> 16, h = state->src_h >> 16;
	unsigned int x, y, colours = 0;
	__le32 row[MQ11XX_CURSOR_SIZE];
	int ret;

	ret = drm_gem_fb_begin_cpu_access(fb, DMA_FROM_DEVICE);
	if (ret)
		return ret;
	ret = drm_gem_fb_vmap(fb, map, data);
	if (ret)
		goto out_end_access;

	cstate->fg = 0;
	cstate->bg = 0;
	for (y = 0; y < MQ11XX_CURSOR_SIZE; y++) {
		cstate->and_mask[y] = ~0ULL;
		cstate->xor_mask[y] = 0;
	}

	for (y = 0; y < h; y++) {
		iosys_map_memcpy_from(row, &data[0],
				      (sy + y) * fb->pitches[0] + sx * sizeof(row[0]),
				      w * sizeof(row[0]));

		for (x = 0; x < w; x++) {
			u32 px = le32_to_cpu(row[x]);
			u32 c = mq11xx_cursor_colour(px);

			if (!(px >> 24))
				continue;
			if (px >> 24 != 0xff) {
				drm_dbg_atomic(plane->dev, "cursor pixel %u,%u is translucent\n",
					       x, y);
				ret = -EINVAL;
				goto out_vunmap;
			}

			if (!colours) {
				cstate->fg = c;
				colours = 1;
			} else if (c != cstate->fg && colours == 1) {
				cstate->bg = c;
				colours = 2;
			} else if (c != cstate->fg && c != cstate->bg) {
				drm_dbg_atomic(plane->dev, "cursor has a third colour\n");
				ret = -EINVAL;
				goto out_vunmap;
			}

			cstate->and_mask[y] &= ~BIT_ULL(x);
			if (c == cstate->fg)
				cstate->xor_mask[y] |= BIT_ULL(x);
		}
	}

	cstate->converted = true;

out_vunmap:
	drm_gem_fb_vunmap(fb, map);
out_end_access:
	drm_gem_fb_end_cpu_access(fb, DMA_FROM_DEVICE);
	return ret;
}

/*
 * The primary plane state this commit leaves behind. A commit that changes
 * the primary plane takes the CRTC's lock, which this one holds as well,
 * so the current state cannot change underneath.
 */
static const struct drm_plane_state *
mq11xx_primary_state(struct mq11xx_device *mq, struct drm_atomic_commit *state)
{
	struct drm_plane_state *primary;

	primary = drm_atomic_get_new_plane_state(state, &mq->primary_plane);

	return primary ?: mq->primary_plane.state;
}

static int mq11xx_cursor_plane_helper_atomic_check(struct drm_plane *plane,
						   struct drm_atomic_commit *state)
{
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_plane_state *old_state = drm_atomic_get_old_plane_state(state, plane);
	struct mq11xx_cursor_state *cstate = to_mq11xx_cursor_state(new_state);
	struct mq11xx_device *mq = to_mq11xx(plane->dev);
	struct drm_crtc_state *crtc_state = NULL;
	const struct drm_plane_state *primary;
	unsigned int t = 0, w, h;
	bool dbl = false;
	int scale, ret;

	if (new_state->fb != old_state->fb ||
	    drm_plane_get_damage_clips_count(new_state) ||
	    new_state->src_x != old_state->src_x ||
	    new_state->src_y != old_state->src_y ||
	    new_state->src_w != old_state->src_w ||
	    new_state->src_h != old_state->src_h)
		cstate->converted = false;

	if (new_state->crtc && new_state->fb) {
		crtc_state = drm_atomic_get_new_crtc_state(state, new_state->crtc);

		primary = mq11xx_primary_state(mq, state);
		if (primary->fb) {
			t = mq11xx_scan_transform(primary->rotation);
			dbl = mq11xx_plane_doubled(primary);
		}

		if (!(mq->chip->cursor_transforms & BIT(t)) ||
		    (dbl && !mq->chip->cursor_double)) {
			drm_dbg_atomic(plane->dev, "no cursor on the %s with rotation 0x%x%s\n",
				       mq->chip->name, primary->rotation,
				       dbl ? " and pixel doubling" : "");
			return -EINVAL;
		}
	}

	/* The chip doubles the cursor with the rest of the window. */
	scale = dbl ? MQ11XX_DOUBLE_SCALE : DRM_PLANE_NO_SCALING;
	ret = drm_atomic_helper_check_plane_state(new_state, crtc_state,
						  scale, scale, true, true);
	if (ret || !new_state->visible)
		return ret;

	w = new_state->src_w >> 16;
	h = new_state->src_h >> 16;
	if ((new_state->src_x | new_state->src_y |
	     new_state->src_w | new_state->src_h) & 0xffff ||
	    w > MQ11XX_CURSOR_SIZE || h > MQ11XX_CURSOR_SIZE ||
	    new_state->crtc_w != w << dbl || new_state->crtc_h != h << dbl) {
		drm_dbg_atomic(plane->dev, "cursor source 0x%x x 0x%x shown %ux%u\n",
			       new_state->src_w, new_state->src_h,
			       new_state->crtc_w, new_state->crtc_h);
		return -EINVAL;
	}

	cstate->mirror = t & MQ11XX_SCAN_X_DECREMENT;
	cstate->dbl = dbl;

	return cstate->converted ? 0 : mq11xx_cursor_convert(plane, cstate);
}

static u64 mq11xx_shr64(u64 v, unsigned int n)
{
	return n < 64 ? v >> n : 0;
}

static u64 mq11xx_rev64(u64 v)
{
	return (u64)bitrev32(lower_32_bits(v)) << 32 | bitrev32(upper_32_bits(v));
}

/*
 * Loads the image for the cursor's placement: shifted left by @sx columns
 * and up by @sy rows where the image itself is clipped, transparent from
 * column @cols and row @rows on, which lie beyond the right and bottom
 * edges, and reflected for a scan that runs each line from its last pixel:
 * the datasheet says GC00[12] runs the cursor's lines that way too. Only
 * the words that change are written.
 */
static void mq11xx_cursor_load(struct mq11xx_device *mq,
			       const struct mq11xx_cursor_state *cstate,
			       unsigned int sx, unsigned int sy,
			       unsigned int cols, unsigned int rows)
{
	u64 keep = cols < 64 ? BIT_ULL(cols) - 1 : ~0ULL;
	bool written = false;
	unsigned int r, k, i;
	u32 word;

	for (r = 0; r < MQ11XX_CURSOR_SIZE; r++) {
		u64 am = ~0ULL, xm = 0;

		if (r < rows && r + sy < MQ11XX_CURSOR_SIZE) {
			am = mq11xx_shr64(cstate->and_mask[r + sy], sx) |
			     ~mq11xx_shr64(~0ULL, sx) | ~keep;
			xm = mq11xx_shr64(cstate->xor_mask[r + sy], sx) & keep;
		}
		if (cstate->mirror) {
			am = mq11xx_rev64(am);
			xm = mq11xx_rev64(xm);
		}

		for (k = 0; k < 4; k++) {
			word = (u16)(am >> (16 * k)) |
			       (u32)(u16)(xm >> (16 * k)) << 16;
			i = r * 4 + k;
			if (mq->cursor_loaded && mq->cursor_image[i] == word)
				continue;
			writel(word, mq->vram + MQ11XX_SRAM_CURSOR + i * 4);
			mq->cursor_image[i] = word;
			written = true;
		}
	}

	if (written)
		mq11xx_vram_flush(mq, MQ11XX_SRAM_CURSOR);
	mq->cursor_loaded = true;
}

static void mq11xx_cursor_hide(struct mq11xx_device *mq)
{
	u32 control = readl(mq->regs + MQ11XX_GC_CONTROL);

	if (control & MQ11XX_GC_CONTROL_CURSOR)
		writel(control & ~MQ11XX_GC_CONTROL_CURSOR,
		       mq->regs + MQ11XX_GC_CONTROL);
}

/*
 * HCO and VCO clip an upright image at the left and top, as both boards'
 * WinCE drivers use them. What they count in a reflected or doubled scan
 * is not documented, so there the image itself is shifted instead, by
 * whole image pixels; a doubled pixel half off the left or top edge is
 * then left out.
 */
static void mq11xx_cursor_plane_helper_atomic_update(struct drm_plane *plane,
						     struct drm_atomic_commit *state)
{
	struct drm_plane_state *plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct mq11xx_cursor_state *cstate = to_mq11xx_cursor_state(plane_state);
	struct mq11xx_device *mq = to_mq11xx(plane->dev);
	const struct drm_display_mode *mode = mq->mode;
	unsigned int hco = 0, vco = 0, sx = 0, sy = 0, cols, rows;
	int x = plane_state->crtc_x, y = plane_state->crtc_y;
	unsigned int dbl = cstate->dbl;
	u32 control;
	int idx;

	if (!drm_dev_enter(&mq->drm, &idx))
		return;

	/* As in the primary plane update: a failed unblank leaves the chip unpowered. */
	if (!mq->powered)
		goto out_exit;

	if (!plane_state->visible) {
		mq11xx_cursor_hide(mq);
		goto out_exit;
	}

	if (x < 0 && (cstate->mirror || dbl)) {
		sx = DIV_ROUND_UP(-x, 1 << dbl);
		x += sx << dbl;
	} else if (x < 0) {
		hco = -x;
		x = 0;
	}
	if (y < 0 && dbl) {
		sy = DIV_ROUND_UP(-y, 1 << dbl);
		y += sy << dbl;
	} else if (y < 0) {
		vco = -y;
		y = 0;
	}

	cols = hco + DIV_ROUND_UP(mode->hdisplay - x, 1 << dbl);
	rows = vco + DIV_ROUND_UP(mode->vdisplay - y, 1 << dbl);
	mq11xx_cursor_load(mq, cstate, sx, sy, cols, rows);

	writel(cstate->fg, mq->regs + MQ11XX_GC_CURSOR_FG);
	writel(cstate->bg, mq->regs + MQ11XX_GC_CURSOR_BG);
	/* Single 32-bit stores, so the most significant byte lands last. */
	writel(FIELD_PREP(MQ11XX_GC_CURSOR_X, x) | FIELD_PREP(MQ11XX_GC_CURSOR_Y, y),
	       mq->regs + MQ11XX_GC_CURSOR_POSITION);
	writel(FIELD_PREP(MQ11XX_GC_CURSOR_ADDR, MQ11XX_SRAM_CURSOR / SZ_1K) |
	       FIELD_PREP(MQ11XX_GC_CURSOR_HCO, hco) |
	       FIELD_PREP(MQ11XX_GC_CURSOR_VCO, vco),
	       mq->regs + MQ11XX_GC_CURSOR_ADDRESS);

	control = readl(mq->regs + MQ11XX_GC_CONTROL);
	if (!(control & MQ11XX_GC_CONTROL_CURSOR))
		writel(control | MQ11XX_GC_CONTROL_CURSOR,
		       mq->regs + MQ11XX_GC_CONTROL);

out_exit:
	drm_dev_exit(idx);
}

static void mq11xx_cursor_plane_helper_atomic_disable(struct drm_plane *plane,
						      struct drm_atomic_commit *state)
{
	struct mq11xx_device *mq = to_mq11xx(plane->dev);
	int idx;

	if (!drm_dev_enter(&mq->drm, &idx))
		return;
	if (mq->powered)
		mq11xx_cursor_hide(mq);
	drm_dev_exit(idx);
}

static const struct drm_plane_helper_funcs mq11xx_cursor_plane_helper_funcs = {
	.atomic_check = mq11xx_cursor_plane_helper_atomic_check,
	.atomic_update = mq11xx_cursor_plane_helper_atomic_update,
	.atomic_disable = mq11xx_cursor_plane_helper_atomic_disable,
};

static struct drm_plane_state *
mq11xx_cursor_plane_duplicate_state(struct drm_plane *plane)
{
	struct mq11xx_cursor_state *cstate;

	if (WARN_ON(!plane->state))
		return NULL;

	cstate = kmemdup(to_mq11xx_cursor_state(plane->state), sizeof(*cstate),
			 GFP_KERNEL);
	if (!cstate)
		return NULL;
	__drm_atomic_helper_plane_duplicate_state(plane, &cstate->base);

	return &cstate->base;
}

static void mq11xx_cursor_plane_destroy_state(struct drm_plane *plane,
					      struct drm_plane_state *state)
{
	__drm_atomic_helper_plane_destroy_state(state);
	kfree(to_mq11xx_cursor_state(state));
}

static void mq11xx_cursor_plane_reset(struct drm_plane *plane)
{
	struct mq11xx_cursor_state *cstate;

	if (plane->state) {
		mq11xx_cursor_plane_destroy_state(plane, plane->state);
		plane->state = NULL;
	}

	cstate = kzalloc(sizeof(*cstate), GFP_KERNEL);
	if (cstate)
		__drm_atomic_helper_plane_reset(plane, &cstate->base);
}

static const struct drm_plane_funcs mq11xx_cursor_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = mq11xx_cursor_plane_reset,
	.atomic_duplicate_state = mq11xx_cursor_plane_duplicate_state,
	.atomic_destroy_state = mq11xx_cursor_plane_destroy_state,
};

static enum drm_mode_status mq11xx_crtc_helper_mode_valid(struct drm_crtc *crtc,
							  const struct drm_display_mode *mode)
{
	struct mq11xx_device *mq = to_mq11xx(crtc->dev);

	return drm_crtc_helper_mode_valid_fixed(crtc, mode, mq->mode);
}

static int mq11xx_crtc_helper_atomic_check(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	if (!crtc_state->enable)
		return 0;

	if (crtc_state->color_mgmt_changed && crtc_state->gamma_lut &&
	    drm_color_lut_size(crtc_state->gamma_lut) != MQ11XX_PALETTE_SIZE) {
		drm_dbg_atomic(crtc->dev, "gamma LUT of %u entries, the palette has %u\n",
			       drm_color_lut_size(crtc_state->gamma_lut),
			       MQ11XX_PALETTE_SIZE);
		return -EINVAL;
	}

	return drm_atomic_helper_check_crtc_primary_plane(crtc_state);
}

/*
 * The gamma LUT is the C8 palette; RGB565 bypasses it, so it only needs
 * loading while C8 is shown. That is done here, before the plane update
 * switches the depth, so the first C8 frame already has its colours.
 */
static void mq11xx_crtc_helper_atomic_begin(struct drm_crtc *crtc,
					    struct drm_atomic_commit *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	struct drm_framebuffer *fb = crtc->primary->state->fb;
	struct mq11xx_device *mq = to_mq11xx(crtc->dev);
	int idx;

	if (!fb || fb->format->format != DRM_FORMAT_C8)
		return;
	if (!crtc_state->color_mgmt_changed && !mq->palette_stale &&
	    mq->scan_cpp == 1)
		return;

	if (!drm_dev_enter(&mq->drm, &idx))
		return;
	/* As in the plane update: a failed unblank leaves the chip unpowered. */
	if (mq->powered)
		mq11xx_palette_load(mq, crtc_state);
	drm_dev_exit(idx);
}

static void mq11xx_crtc_helper_atomic_enable(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct mq11xx_device *mq = to_mq11xx(crtc->dev);
	int idx;

	if (drm_dev_enter(&mq->drm, &idx)) {
		mq11xx_output_enable(mq);
		drm_dev_exit(idx);
	}

	drm_crtc_vblank_on(crtc);
}

static void mq11xx_crtc_helper_atomic_disable(struct drm_crtc *crtc,
					      struct drm_atomic_commit *state)
{
	struct mq11xx_device *mq = to_mq11xx(crtc->dev);
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	drm_crtc_vblank_off(crtc);

	/*
	 * The commit tail skips atomic_flush on an inactive CRTC, so an
	 * event on the commit that turns it off is completed here.
	 */
	if (crtc_state->event && !crtc_state->active) {
		spin_lock_irq(&crtc->dev->event_lock);
		drm_crtc_send_vblank_event(crtc, crtc_state->event);
		spin_unlock_irq(&crtc->dev->event_lock);
		crtc_state->event = NULL;
	}

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
	.atomic_begin = mq11xx_crtc_helper_atomic_begin,
	.atomic_flush = drm_crtc_vblank_atomic_flush,
	.atomic_enable = mq11xx_crtc_helper_atomic_enable,
	.atomic_disable = mq11xx_crtc_helper_atomic_disable,
};

#define MQ11XX_CRTC_FUNCS						\
	.reset = drm_atomic_helper_crtc_reset,				\
	.destroy = drm_crtc_cleanup,					\
	.set_config = drm_atomic_helper_set_config,			\
	.page_flip = drm_atomic_helper_page_flip,			\
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state, \
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state

/* No frame interrupt: vblank events are paced by a timer at the mode's rate. */
static const struct drm_crtc_funcs mq11xx_crtc_timer_funcs = {
	MQ11XX_CRTC_FUNCS,
	DRM_CRTC_VBLANK_TIMER_FUNCS,
};

static const struct drm_crtc_funcs mq11xx_crtc_irq_funcs = {
	MQ11XX_CRTC_FUNCS,
	.enable_vblank = mq11xx_crtc_enable_vblank_irq,
	.disable_vblank = mq11xx_crtc_disable_vblank_irq,
};

static const struct drm_encoder_funcs mq11xx_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int mq11xx_connector_helper_get_modes(struct drm_connector *connector)
{
	struct mq11xx_device *mq = to_mq11xx(connector->dev);

	return drm_connector_helper_get_modes_fixed(connector, mq->mode);
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
	bool palette = mq->chip->caps & MQ11XX_CAP_PALETTE_8BPP;
	struct drm_plane *cursor_plane = NULL;
	int ret;

	ret = drm_universal_plane_init(dev, primary_plane, 0,
				       &mq11xx_primary_plane_funcs,
					mq11xx_primary_plane_formats,
					palette ? ARRAY_SIZE(mq11xx_primary_plane_formats) : 1,
					mq11xx_primary_plane_format_modifiers,
					DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(primary_plane, &mq11xx_primary_plane_helper_funcs);
	drm_plane_enable_fb_damage_clips(primary_plane);

	if (mq->chip->caps & MQ11XX_CAP_ROTATION) {
		ret = drm_plane_create_rotation_property(primary_plane,
							 DRM_MODE_ROTATE_0,
							 mq11xx_supported_rotations(mq));
		if (ret)
			return ret;
	}

	if (mq11xx_has_cursor(mq)) {
		cursor_plane = &mq->cursor_plane;
		ret = drm_universal_plane_init(dev, cursor_plane, 0,
					       &mq11xx_cursor_plane_funcs,
					       mq11xx_cursor_plane_formats,
					       ARRAY_SIZE(mq11xx_cursor_plane_formats),
					       mq11xx_primary_plane_format_modifiers,
					       DRM_PLANE_TYPE_CURSOR, NULL);
		if (ret)
			return ret;
		drm_plane_helper_add(cursor_plane, &mq11xx_cursor_plane_helper_funcs);
	}

	ret = drm_crtc_init_with_planes(dev, crtc, primary_plane, cursor_plane,
					mq->irq ? &mq11xx_crtc_irq_funcs :
						  &mq11xx_crtc_timer_funcs,
					NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(crtc, &mq11xx_crtc_helper_funcs);

	if (palette) {
		ret = drm_mode_crtc_set_gamma_size(crtc, MQ11XX_PALETTE_SIZE);
		if (ret)
			return ret;
		drm_crtc_enable_color_mgmt(crtc, 0, false, MQ11XX_PALETTE_SIZE);
	}

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

/*
 * The cursor image is reflected and doubled with the scan, so a commit
 * that changes the primary plane's transform or doubling updates the
 * cursor plane too, whose atomic_check then also decides whether the
 * cursor can still be shown.
 */
static int mq11xx_mode_config_atomic_check(struct drm_device *dev,
					   struct drm_atomic_commit *state)
{
	struct mq11xx_device *mq = to_mq11xx(dev);
	struct drm_plane_state *old_state, *new_state, *cursor_state;

	new_state = drm_atomic_get_new_plane_state(state, &mq->primary_plane);
	if (mq11xx_has_cursor(mq) && new_state) {
		old_state = drm_atomic_get_old_plane_state(state, &mq->primary_plane);
		if (mq11xx_scan_transform(new_state->rotation) !=
		    mq11xx_scan_transform(old_state->rotation) ||
		    mq11xx_plane_doubled(new_state) != mq11xx_plane_doubled(old_state)) {
			cursor_state = drm_atomic_get_plane_state(state, &mq->cursor_plane);
			if (IS_ERR(cursor_state))
				return PTR_ERR(cursor_state);
		}
	}

	return drm_atomic_helper_check(dev, state);
}

static const struct drm_mode_config_funcs mq11xx_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = mq11xx_mode_config_atomic_check,
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

	/*
	 * No minimum: a pixel-doubled plane takes a framebuffer of half the
	 * screen's size, and the primary plane's atomic_check requires the
	 * exact size for each layout anyway.
	 */
	dev->mode_config.max_width = mq->mode->hdisplay;
	dev->mode_config.max_height = mq->mode->vdisplay;

	/* A quarter turn scans a framebuffer of the transposed size. */
	if (mq->chip->caps & MQ11XX_CAP_ROTATION) {
		dev->mode_config.max_width = max(mq->mode->hdisplay, mq->mode->vdisplay);
		dev->mode_config.max_height = dev->mode_config.max_width;
	}
	if (mq11xx_has_cursor(mq)) {
		dev->mode_config.cursor_width = MQ11XX_CURSOR_SIZE;
		dev->mode_config.cursor_height = MQ11XX_CURSOR_SIZE;
	}
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

	ret = drmm_mutex_init(drm, &mq->hw_lock);
	if (ret)
		return ret;
	ret = drmm_mutex_init(drm, &mq->ge_lock);
	if (ret)
		return ret;
	mq->pwm_level = -1;
	mq->mode = mq->variant->mode;
	spin_lock_init(&mq->irq_lock);

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

	/*
	 * The panel is chosen during the register load; use the mode that
	 * goes with it.
	 */
	if (mq->envee_applied && mq->variant->envee_mode)
		mq->mode = mq->variant->envee_mode;

	ret = mq11xx_irq_init(mq, pdev);
	if (ret)
		return ret;

	if (mq->variant->has_internal_pwm) {
		ret = mq11xx_pwm_chip_register(mq, dev);
		if (ret)
			return ret;
	}

	/*
	 * The rest of the bring-up needs the backlight, which on h5xxx needs
	 * the pwm_chip just registered. Waiting for it here would defer this
	 * probe, whose unwind removes that pwm_chip again, so neither side
	 * could ever finish. A device of its own lets the waiting part defer
	 * while this one stays bound, as in ti-sn65dsi86.
	 */
	if (!devm_auxiliary_device_create(dev, "display", NULL))
		return dev_err_probe(dev, -ENODEV, "failed to add display device\n");

	return 0;
}

static int mq11xx_display_probe(struct auxiliary_device *adev,
				const struct auxiliary_device_id *id)
{
	struct device *dev = &adev->dev;
	struct mq11xx_device *mq = dev_get_drvdata(dev->parent);
	struct drm_device *drm = &mq->drm;
	struct backlight_device *backlight;
	int ret;

	/*
	 * First, and the only step here that can defer: the steps below set
	 * up DRM state tied to the parent's drm_device, which a second run
	 * could not set up again.
	 */
	backlight = devm_of_find_backlight(dev);
	if (IS_ERR(backlight))
		return dev_err_probe(dev, PTR_ERR(backlight), "failed to get backlight\n");
	mq->backlight = backlight;

	ret = mq11xx_mode_config_init(mq);
	if (ret)
		return ret;

	ret = mq11xx_pipe_init(mq);
	if (ret)
		return ret;

	ret = drm_vblank_init(drm, 1);
	if (ret)
		return ret;

	/* On this device, so it outlives mq11xx_display_remove() as it must. */
	if (mq->irq) {
		ret = devm_request_irq(dev, mq->irq, mq11xx_irq_handler, 0,
				       DRIVER_NAME, mq);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request interrupt\n");
	}
	drm_info(drm, "vblank from %s\n",
		 mq->irq ? "the frame interrupt" : "the vblank timer");

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	drm_client_setup(drm, NULL);

	return 0;
}

static void mq11xx_display_remove(struct auxiliary_device *adev)
{
	struct mq11xx_device *mq = dev_get_drvdata(adev->dev.parent);

	drm_dev_unplug(&mq->drm);
	drm_atomic_helper_shutdown(&mq->drm);
	/* Unplugged, so no commit can be using the channel any more. */
	mq11xx_present_dma_fini(mq);
}

/*
 * System sleep reuses the blank and unblank path: the forced atomic
 * disable and enable of the DRM helpers run mq11xx_output_disable() and
 * mq11xx_output_enable(), so the chip is powered off across sleep and fully
 * reprogrammed by mq11xx_hw_init() on resume, exactly as at probe. The
 * settle times after suspending and resuming are those of the vendor
 * display driver.
 *
 * These belong to the display device: there is no mode config to suspend
 * until it has bound. Its last probe attempt also moved it behind the
 * backlight in the suspend order, so it blanks first and unblanks last,
 * and the backlight's own resume finds the light off, not an undriven
 * panel to shine through.
 */
static int mq11xx_pm_suspend(struct device *dev)
{
	struct mq11xx_device *mq = dev_get_drvdata(dev->parent);
	int ret;

	ret = drm_mode_config_helper_suspend(&mq->drm);
	if (ret)
		return ret;

	msleep(58);

	return 0;
}

static int mq11xx_pm_resume(struct device *dev)
{
	struct mq11xx_device *mq = dev_get_drvdata(dev->parent);

	drm_mode_config_helper_resume(&mq->drm);

	msleep(90);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mq11xx_pm_ops, mq11xx_pm_suspend, mq11xx_pm_resume);

static const struct auxiliary_device_id mq11xx_display_id_table[] = {
	{ .name = DRIVER_NAME ".display" },
	{ }
};
MODULE_DEVICE_TABLE(auxiliary, mq11xx_display_id_table);

static struct auxiliary_driver mq11xx_display_driver = {
	.name = "display",
	.probe = mq11xx_display_probe,
	.remove = mq11xx_display_remove,
	.id_table = mq11xx_display_id_table,
	.driver = {
		.pm = pm_sleep_ptr(&mq11xx_pm_ops),
		/* mq11xx_display_probe() can only run to completion once. */
		.suppress_bind_attrs = true,
	},
};

static struct platform_driver mq11xx_platform_driver = {
	.probe	= mq11xx_probe,
	.driver	= {
		.name = DRIVER_NAME,
		.of_match_table = mq11xx_of_match,
	},
};

static int __init mq11xx_init(void)
{
	int ret;

	ret = auxiliary_driver_register(&mq11xx_display_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&mq11xx_platform_driver);
	if (ret)
		auxiliary_driver_unregister(&mq11xx_display_driver);

	return ret;
}
module_init(mq11xx_init);

static void __exit mq11xx_exit(void)
{
	platform_driver_unregister(&mq11xx_platform_driver);
	auxiliary_driver_unregister(&mq11xx_display_driver);
}
module_exit(mq11xx_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
