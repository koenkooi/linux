// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM driver for the LCD controller integrated in the Intel/StrongARM
 * SA-1100 and SA-1110 application processors (the two share the identical
 * block; the legacy fbdev driver for the same silicon is accordingly named
 * drivers/video/fbdev/sa1100fb.c, not sa1110fb.c).
 *
 * Copyright (C) 2026 Koen Kooi <koen@dominion.thruhere.net>
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/math.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_encoder.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_module.h>
#include <drm/drm_plane.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_rect.h>

/**
 * DOC: overview
 *
 * This is a KMS driver for the LCD controller integrated in the SA-1100 and
 * SA-1110 application processors. The PXA2xx LCD controller shares the
 * LCCR0-3 control-register split, but this part has no frame-descriptor DMA
 * engine. A single pair of registers, DBAR1/DBAR2, gives the DMA engine a
 * base address, and the controller re-reads that address at the start of
 * every frame on its own; there is no descriptor to chain and no branch
 * register.
 *
 * That keeps the driver simple, except for one hardware quirk: this
 * controller has no register field for pixel format at all. The 2-bit
 * "pixel bit size" (PBS) selector lives in bits [13:12] of the *first
 * in-memory palette entry* that DBAR1 points at, read once by the DMA
 * engine at the start of scanout -- see ``drivers/video/fbdev/sa1100fb.c``'s own
 * ``palette_pbs()`` and ``sa1100fb_enable_controller()``. Even in this
 * driver's only supported mode (16bpp, no palette lookup), that one word
 * still has to exist in memory immediately before the pixel data, and
 * DBAR1 has to point at it, not at the pixel data itself.
 *
 * A GEM DMA framebuffer handed to this driver by userspace has no room for
 * that leading word and no guarantee of being physically followed by
 * anything at all, so this driver cannot scan out of it directly the way
 * a descriptor-based controller can. Instead it owns one persistent
 * DMA-coherent buffer -- a fixed 2-byte format-select header followed by a fixed-size pixel
 * region -- allocated once at probe for @SA1100_LCDC_MAX_WIDTH x
 * @SA1100_LCDC_MAX_HEIGHT, and every plane update copies the damaged part
 * of the client's pixel data into it. This is a "shadow buffer,
 * damage-clipped memcpy on update" scheme; here the reason for it is the one
 * mandatory header word. The cost is a CPU copy per flip and no true
 * page-flipping (the scanout address never changes, so a torn frame is
 * possible mid-copy); for a board with no other DRM consumer
 * yet, that is an acceptable trade against the
 * complexity of teaching userspace's dumb-buffer allocator to leave room
 * for a hardware-mandated two-byte header.
 *
 * Scanning out of a private buffer rather than the client's own is exactly
 * what &drm_framebuffer_funcs.dirty exists for, so &drm_mode_config_funcs
 * .fb_create here is drm_gem_fb_create_with_dirty() and the primary plane
 * carries the FB_DAMAGE_CLIPS property, as for any driver that scans out of a private copy.
 * Neither is optional: drm_fbdev_dma_driver_fbdev_probe() decides between
 * mapping the console straight onto the scanout GEM object and running a
 * deferred-I/O shadow purely on whether ``fb->funcs->dirty`` is set, and
 * drm_fbdev_dma_helper_fb_dirty() returns without doing anything when it is
 * not -- so a driver of this shape that keeps the plain drm_gem_fb_create()
 * gets its &drm_plane_helper_funcs.atomic_update called once, at the
 * initial modeset, and never again.
 *
 * Panel supplies and timings are not this driver's business, for the same
 * reason as for other DPI controllers: a &drm_panel reached over the OF graph owns the
 * connector, and per-board rail sequencing has no place in shared silicon
 * code.
 */

/**
 * DOC: clock
 *
 * The pixel clock divides the core clock (CCLK), the PLL output that also
 * clocks the CPU core -- there is no separate LCD PLL on this part. The
 * rate is read when the CRTC is enabled, so a CCLK changed by cpufreq
 * takes effect only at the next enable.
 */

#define DRIVER_NAME	"sa1100-lcdc"
#define DRIVER_DESC	"Intel/StrongARM SA-1100 LCD controller"

/*
 * Register offsets and bit/field definitions, transcribed from
 * arch/arm/mach-sa1100/include/mach/SA-1100.h, which is private to
 * mach-sa1100 and not something a drivers/gpu/drm/tiny/ driver may include.
 * Only the fields a single-panel active (TFT) 16bpp configuration uses
 * appear below; the passive/dual-panel and 4/8bpp palette fields are
 * absent because this driver never touches them.
 */
#define LCCR0			0x000
#define LCSR			0x004
#define DBAR1			0x010
#define DBAR2			0x018
#define LCCR1			0x020
#define LCCR2			0x024
#define LCCR3			0x028

#define LCCR0_LEN		BIT(0)	/* controller enable */
#define LCCR0_LDM		BIT(3)	/* disable-done interrupt mask */
#define LCCR0_BAM		BIT(4)	/* base-address-update interrupt mask */
#define LCCR0_ERM		BIT(5)	/* error interrupt mask (every LCSR error bit) */
#define LCCR0_PAS		BIT(7)	/* active (TFT) rather than passive */
/*
 * Frame-buffer/palette DMA byte order (LCCR0 bit 8): 0 = little endian
 * (palette and pixel data are packed into each 32-bit DMA word starting
 * with the least-significant half-word, which is what this driver's only
 * format, DRM_FORMAT_RGB565, needs on this little-endian core), 1 = big
 * endian. The reset default is 0; the CRTC enable path clears the bit
 * explicitly rather than relying on the reset default.
 */
#define LCCR0_BLE		BIT(8)

/*
 * Every interrupt source masked at once, for the paths that want the
 * controller silent. This part has no separate quick/graceful disable
 * request bit: clearing LCCR0_LEN always
 * triggers the one and only disable sequence, and LCSR_LDD signals when it
 * has actually finished.
 */
#define LCCR0_ALL_MASKS		(LCCR0_LDM | LCCR0_BAM | LCCR0_ERM)

/*
 * The output FIFO runs dry while the first frames after every enable start
 * the DMA, so errors stay masked for this long (about six frames at 64 Hz)
 * and are unmasked, with the startup errors cleared, by
 * sa1100_lcdc_unmask_errors().
 */
#define SA1100_LCDC_STARTUP_ERRORS_MS	100

#define LCSR_LDD		BIT(0)	/* disable done */
#define LCSR_BAU		BIT(1)	/* base address update (unused) */
#define LCSR_BER		BIT(2)	/* bus error */
#define LCSR_ABC		BIT(3)	/* AC bias clock count (passive only) */
#define LCSR_IOL		BIT(4)	/* input FIFO overrun, lower panel */
#define LCSR_IUL		BIT(5)	/* input FIFO underrun, lower panel */
#define LCSR_IOU		BIT(6)	/* input FIFO overrun, upper panel */
#define LCSR_IUU		BIT(7)	/* input FIFO underrun, upper panel */
#define LCSR_OOL		BIT(8)	/* output FIFO overrun, lower panel */
#define LCSR_OUL		BIT(9)	/* output FIFO underrun, lower panel */
#define LCSR_OOU		BIT(10)	/* output FIFO overrun, upper panel */
#define LCSR_OUU		BIT(11)	/* output FIFO underrun, upper panel */

#define LCSR_ERRORS		(LCSR_BER | LCSR_IOL | LCSR_IUL | LCSR_IOU | \
				 LCSR_IUU | LCSR_OOL | LCSR_OUL | LCSR_OOU | LCSR_OUU)

/*
 * LCCR1_PPL is pixels-per-line in units of 16, biased by one -- a coarser
 * granularity than the per-pixel PPL field of the PXA2xx controller. A 320
 * pixel wide panel is (320 - 16) / 16 = 19.
 */
#define LCCR1_PPL(pixels)	((((pixels) - 16) / 16) << 4)
/*
 * LCCR1_HSW is biased by one here, following mach-sa1100's SA-1100.h. Older
 * copies of the same Intel-derived header describe the field as
 * "pulse width - 2" instead, so the two encodings disagree by one Tpix about
 * what a given field value means. The programmed value settles it: the
 * factory firmware of the h3870 leaves HSW = 2, which a 3-pixel hsync
 * produces through this macro. LCCR2_VSW is the same story and the same
 * field value.
 */
#define LCCR1_HSW(tpix)		(((tpix) - 1) << 10)	/* hsync width */
#define LCCR1_ELW(tpix)		(((tpix) - 1) << 16)	/* front porch */
#define LCCR1_BLW(tpix)		(((tpix) - 1) << 24)	/* back porch */

#define LCCR2_LPP(lines)	(((lines) - 1) << 0)	/* lines per panel, biased */
#define LCCR2_VSW(tln)		(((tln) - 1) << 10)	/* vsync width */
#define LCCR2_EFW(tln)		((tln) << 16)		/* front porch, not biased */
#define LCCR2_BFW(tln)		((tln) << 24)		/* back porch, not biased */

/*
 * fpix = fcpu / (2 * (field + 2)).
 */
#define LCCR3_PCD(field)	((field) << 0)
#define LCCR3_PCD_MAX		255
#define LCCR3_VSP		BIT(20)	/* vsync active low */
#define LCCR3_HSP		BIT(21)	/* hsync active low */
#define LCCR3_PCP		BIT(22)	/* data driven on the falling edge */
#define LCCR3_OEP		BIT(23)	/* output-enable active low */

/*
 * The mandatory PBS (pixel bit size) selector this file's own overview DOC
 * comment describes: 2 selects 12/16bpp mode, the only mode this driver
 * ever programs. It lives in the first in-memory palette entry, not in a
 * register.
 */
#define SA1100_LCDC_PBS_16BPP		2
#define SA1100_LCDC_PBS_SHIFT		12

/*
 * The controller reads a fixed 16-entry "dummy" palette table before the
 * real pixel data in 12/16bpp mode (SA-1100.h's own "12/16-Bit pixel
 * dummy-Palette Space" comment) -- only entry 0's PBS field is meaningful,
 * but the whole 16-entry table has to physically exist for the DMA engine
 * to read through before it reaches real pixel data.
 */
#define SA1100_LCDC_PALETTE_ENTRIES	16
#define SA1100_LCDC_HEADER_SIZE		(SA1100_LCDC_PALETTE_ENTRIES * sizeof(u16))

/*
 * The panel of the only board this driver serves, the h3800's 320x240.
 * The cap sizes the one shadow buffer this driver allocates at probe, out
 * of coherent memory on a machine with 32 or 64 MiB of RAM, and rejects a
 * mode or framebuffer this driver cannot scan out of before the copy in
 * sa1100_lcdc_plane_atomic_update() runs off the end of it. A board with a
 * larger panel raises it.
 */
#define SA1100_LCDC_MAX_WIDTH		320
#define SA1100_LCDC_MAX_HEIGHT		240
#define SA1100_LCDC_SHADOW_SIZE		(SA1100_LCDC_HEADER_SIZE + \
					 SA1100_LCDC_MAX_WIDTH * SA1100_LCDC_MAX_HEIGHT * 2)

/*
 * A graceful disable finishes the frame in progress before it completes, so
 * allow for a few frame times.
 */
#define SA1100_LCDC_DISABLE_TIMEOUT_MS	100

/*
 * Warn, but still program the divider, when the achievable pixel clock
 * misses the mode by more than this.
 */
#define SA1100_LCDC_PCLK_TOLERANCE_PCT	5

struct sa1100_lcdc {
	struct drm_device drm;
	struct drm_plane plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector *connector;

	void __iomem *base;
	struct clk *clk;
	bool clk_enabled;
	struct completion disable_done;
	struct delayed_work unmask_errors_work;

	/* The one persistent scanout buffer this file's overview DOC
	 * comment describes: a fixed PBS header followed by fixed-size
	 * pixel storage, allocated once and never resized. DBAR1 is
	 * programmed to @shadow_dma once, at enable, and never rewritten
	 * on a plane update -- only the memory it points at changes.
	 */
	void *shadow;
	dma_addr_t shadow_dma;
};

static inline struct sa1100_lcdc *to_sa1100_lcdc(struct drm_device *drm)
{
	return container_of(drm, struct sa1100_lcdc, drm);
}

static u32 sa1100_lcdc_read(struct sa1100_lcdc *lcdc, u32 reg)
{
	return readl(lcdc->base + reg);
}

static void sa1100_lcdc_write(struct sa1100_lcdc *lcdc, u32 reg, u32 val)
{
	writel(val, lcdc->base + reg);
}

/*
 * field = round(fcpu / (2 * fpix)) - 2, the inverse of LCCR3_PCD()'s own
 * formula. For example a 206.4 MHz CPU clock and a 5722 kHz pixel clock
 * give round(206400 / (2 * 5722)) - 2 = 18 - 2 = 16.
 */
static u32 sa1100_lcdc_pcd(struct sa1100_lcdc *lcdc, const struct drm_display_mode *mode)
{
	unsigned long rate = clk_get_rate(lcdc->clk);
	unsigned long target = mode->clock * 1000UL;
	unsigned long achieved, delta;
	u32 field;

	if (!target)
		return 0;

	field = DIV_ROUND_CLOSEST(rate, 2 * target);
	field = clamp_t(u32, field, 3, LCCR3_PCD_MAX + 2) - 2;

	achieved = rate / (2 * (field + 2));
	delta = abs_diff(achieved, target);
	if (delta > target / 100 * SA1100_LCDC_PCLK_TOLERANCE_PCT)
		drm_warn(&lcdc->drm,
			 "pixel clock %lu Hz requested, %lu Hz programmed\n",
			 target, achieved);

	return field;
}

/*
 * Stop a running controller and leave every interrupt source masked: the
 * bootloader may hand the controller over still scanning out of memory this
 * kernel does not own.
 *
 * There is no separate quick-disable request -- clearing LEN is the only
 * disable this part has, and it always finishes the current frame before
 * stopping (LCSR_LDD signals this, see sa1100_lcdc_crtc_atomic_disable()).
 * This path does not wait for it: at probe there is nothing waiting to
 * observe the last frame anyway, and a wait here would need the clock
 * enabled first.
 */
static void sa1100_lcdc_quiesce(struct sa1100_lcdc *lcdc)
{
	u32 lccr0 = sa1100_lcdc_read(lcdc, LCCR0);

	sa1100_lcdc_write(lcdc, LCCR0,
			  (lccr0 | LCCR0_ALL_MASKS) & ~LCCR0_LEN);
	sa1100_lcdc_write(lcdc, LCSR, sa1100_lcdc_read(lcdc, LCSR));
}

static void sa1100_lcdc_unmask_errors(struct work_struct *work)
{
	struct sa1100_lcdc *lcdc = container_of(to_delayed_work(work),
						struct sa1100_lcdc,
						unmask_errors_work);
	int idx;

	if (!drm_dev_enter(&lcdc->drm, &idx))
		return;

	/* The startup underruns are latched in LCSR even while masked. */
	sa1100_lcdc_write(lcdc, LCSR, LCSR_ERRORS);
	sa1100_lcdc_write(lcdc, LCCR0,
			  sa1100_lcdc_read(lcdc, LCCR0) & ~LCCR0_ERM);

	drm_dev_exit(idx);
}

static irqreturn_t sa1100_lcdc_irq(int irq, void *data)
{
	struct sa1100_lcdc *lcdc = data;
	u32 lcsr;

	lcsr = sa1100_lcdc_read(lcdc, LCSR);
	if (!lcsr)
		return IRQ_NONE;

	if (lcsr & LCSR_LDD) {
		u32 lccr0 = sa1100_lcdc_read(lcdc, LCCR0);

		sa1100_lcdc_write(lcdc, LCCR0, lccr0 | LCCR0_LDM);
		complete(&lcdc->disable_done);
	}

	/*
	 * A bus error shares this part's single ERM mask bit with every FIFO
	 * under/overrun source -- see LCCR0_ERM's own comment.
	 * Reporting is therefore all-or-nothing: a silently swallowed bus
	 * error is worse than one FIFO-underrun line on hardware this new, so
	 * sa1100_lcdc_unmask_errors() unmasks it once the startup frames of
	 * every enable are over.
	 *
	 * The first error re-masks it. Every source behind ERM can repeat per
	 * scanline, and at this part's line rate a chronic underrun would be
	 * tens of thousands of interrupts a second on a 206 MHz core whose
	 * serial console is the only way to see anything at all -- so this is
	 * a bounded one-shot report, not a ratelimited running commentary.
	 */
	if (lcsr & LCSR_ERRORS) {
		u32 lccr0 = sa1100_lcdc_read(lcdc, LCCR0);

		sa1100_lcdc_write(lcdc, LCCR0, lccr0 | LCCR0_ERM);
		drm_err(&lcdc->drm,
			"controller error, LCSR 0x%08x; further error reports masked\n",
			lcsr);
	}

	/* Write-one-to-clear: acknowledge exactly the bits that were read. */
	sa1100_lcdc_write(lcdc, LCSR, lcsr);

	return IRQ_HANDLED;
}

static const u32 sa1100_lcdc_formats[] = {
	DRM_FORMAT_RGB565,
};

static const u64 sa1100_lcdc_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

static int sa1100_lcdc_plane_atomic_check(struct drm_plane *plane,
					  struct drm_atomic_commit *state)
{
	struct drm_plane_state *new_plane_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *new_crtc_state = NULL;
	struct drm_framebuffer *fb = new_plane_state->fb;
	int ret;

	if (new_plane_state->crtc)
		new_crtc_state =
			drm_atomic_get_new_crtc_state(state, new_plane_state->crtc);

	ret = drm_atomic_helper_check_plane_state(new_plane_state, new_crtc_state,
						  DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING,
						  false, false);
	if (ret || !new_plane_state->visible || !fb)
		return ret;

	/*
	 * The shadow buffer this driver copies into has no per-line padding
	 * either -- see this file's overview DOC comment.
	 */
	if (fb->pitches[0] != fb->width * fb->format->cpp[0]) {
		drm_dbg_kms(plane->dev, "pitch %u must be %u\n",
			    fb->pitches[0], fb->width * fb->format->cpp[0]);
		return -EINVAL;
	}

	/*
	 * The shadow is laid out at the framebuffer's pitch while the
	 * controller scans hdisplay pixels per line, so a wider framebuffer,
	 * or one scanned out from an offset, would come out skewed.
	 */
	if (fb->width != new_crtc_state->mode.hdisplay ||
	    new_plane_state->src.x1 || new_plane_state->src.y1) {
		drm_dbg_kms(plane->dev,
			    "framebuffer must be %u pixels wide and scanned out from 0,0\n",
			    new_crtc_state->mode.hdisplay);
		return -EINVAL;
	}

	return 0;
}

/*
 * Copy the damaged part of the client's pixel data into the shadow buffer,
 * behind the fixed PBS header. DBAR1 itself is never rewritten here -- see
 * struct sa1100_lcdc::shadow's own comment -- so nothing protects this copy
 * from a concurrent DMA read. A torn frame is possible; this file's overview DOC comment
 * records that as an accepted trade, and keeping the copy
 * to the damage rectangles keeps the window it is possible in as small as
 * the client's own update.
 *
 * The shadow's pixel region is laid out at exactly the framebuffer's own
 * pitch -- sa1100_lcdc_plane_atomic_check() rejects any other -- so the
 * destination offset of a damage rectangle is the source offset, and
 * drm_fb_memcpy() can be handed fb->pitches for both.
 */
static void sa1100_lcdc_plane_atomic_update(struct drm_plane *plane,
					    struct drm_atomic_commit *state)
{
	struct sa1100_lcdc *lcdc = to_sa1100_lcdc(plane->dev);
	struct drm_plane_state *plane_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct drm_plane_state *old_plane_state =
		drm_atomic_get_old_plane_state(state, plane);
	struct drm_framebuffer *fb = plane_state->fb;
	struct drm_atomic_helper_damage_iter iter;
	struct iosys_map shadow_map, src;
	struct drm_gem_dma_object *dma_obj;
	struct drm_rect damage;
	int idx;

	if (!fb)
		return;

	if (!drm_dev_enter(&lcdc->drm, &idx))
		return;

	dma_obj = drm_fb_dma_get_gem_obj(fb, 0);
	iosys_map_set_vaddr(&src, dma_obj->vaddr + fb->offsets[0]);
	iosys_map_set_vaddr(&shadow_map, lcdc->shadow + SA1100_LCDC_HEADER_SIZE);

	drm_atomic_helper_damage_iter_init(&iter, old_plane_state, plane_state);
	drm_atomic_for_each_plane_damage(&iter, &damage) {
		unsigned int offset = drm_fb_clip_offset(fb->pitches[0],
							 fb->format, &damage);
		struct iosys_map dst = IOSYS_MAP_INIT_OFFSET(&shadow_map, offset);

		drm_fb_memcpy(&dst, fb->pitches, &src, fb, &damage);
	}

	drm_dev_exit(idx);
}

static const struct drm_plane_helper_funcs sa1100_lcdc_plane_helper_funcs = {
	.prepare_fb = drm_gem_plane_helper_prepare_fb,
	.atomic_check = sa1100_lcdc_plane_atomic_check,
	.atomic_update = sa1100_lcdc_plane_atomic_update,
};

static const struct drm_plane_funcs sa1100_lcdc_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static int sa1100_lcdc_crtc_atomic_check(struct drm_crtc *crtc,
					 struct drm_atomic_commit *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	if (!crtc_state->enable)
		return 0;

	if (crtc_state->mode.hdisplay > SA1100_LCDC_MAX_WIDTH ||
	    crtc_state->mode.vdisplay > SA1100_LCDC_MAX_HEIGHT) {
		drm_dbg_kms(crtc->dev, "mode %dx%d exceeds the %dx%d shadow buffer\n",
			    crtc_state->mode.hdisplay, crtc_state->mode.vdisplay,
			    SA1100_LCDC_MAX_WIDTH, SA1100_LCDC_MAX_HEIGHT);
		return -EINVAL;
	}

	return drm_atomic_helper_check_crtc_primary_plane(crtc_state);
}

/*
 * Program the header word once per enable (cheap, and the controller is
 * stopped anyway) and start the controller. The register write order is
 * the one drivers/video/fbdev/sa1100fb.c's own sa1100fb_enable_controller()
 * uses, cited there to the SA-1100 manual's section 11.7.10: timing
 * registers, then LCCR0 with LEN still clear, then DBAR1/DBAR2, then LCCR0
 * with LEN set -- setting LEN is what starts the DMA fetch, so the base
 * address must already be in place.
 *
 * DBAR2 is left zero: it feeds DMA channel 2, which only a dual-panel (STN)
 * configuration reads, and LCCR0_PAS (active/TFT, set unconditionally
 * below) is never paired with LCCR0_SDS (dual panel) by this driver.
 */
static void sa1100_lcdc_crtc_atomic_enable(struct drm_crtc *crtc,
					   struct drm_atomic_commit *state)
{
	struct sa1100_lcdc *lcdc = to_sa1100_lcdc(crtc->dev);
	const struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	const struct drm_display_info *info = &lcdc->connector->display_info;
	u32 lccr0, lccr1, lccr2, lccr3;
	int idx;

	if (!drm_dev_enter(&lcdc->drm, &idx))
		return;

	*(u16 *)lcdc->shadow = SA1100_LCDC_PBS_16BPP << SA1100_LCDC_PBS_SHIFT;

	/*
	 * ERM is set for the startup frames and cleared by
	 * sa1100_lcdc_unmask_errors() -- see sa1100_lcdc_irq()'s own comment.
	 */
	lccr0 = LCCR0_PAS | LCCR0_LDM | LCCR0_BAM | LCCR0_ERM;
	/* Explicit little-endian assertion -- see LCCR0_BLE's own comment. */
	lccr0 &= ~LCCR0_BLE;

	lccr1 = LCCR1_PPL(mode->hdisplay) |
		LCCR1_HSW(mode->hsync_end - mode->hsync_start) |
		LCCR1_ELW(mode->hsync_start - mode->hdisplay) |
		LCCR1_BLW(mode->htotal - mode->hsync_end);

	lccr2 = LCCR2_LPP(mode->vdisplay) |
		LCCR2_VSW(mode->vsync_end - mode->vsync_start) |
		LCCR2_EFW(mode->vsync_start - mode->vdisplay) |
		LCCR2_BFW(mode->vtotal - mode->vsync_end);

	lccr3 = LCCR3_PCD(sa1100_lcdc_pcd(lcdc, mode));
	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		lccr3 |= LCCR3_HSP;
	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		lccr3 |= LCCR3_VSP;
	if (info->bus_flags & DRM_BUS_FLAG_DE_LOW)
		lccr3 |= LCCR3_OEP;
	if (info->bus_flags & DRM_BUS_FLAG_PIXDATA_DRIVE_NEGEDGE)
		lccr3 |= LCCR3_PCP;

	if (clk_prepare_enable(lcdc->clk)) {
		drm_err(&lcdc->drm, "failed to enable the LCD clock\n");
		drm_dev_exit(idx);
		return;
	}

	lcdc->clk_enabled = true;

	sa1100_lcdc_write(lcdc, LCCR3, lccr3);
	sa1100_lcdc_write(lcdc, LCCR2, lccr2);
	sa1100_lcdc_write(lcdc, LCCR1, lccr1);
	sa1100_lcdc_write(lcdc, LCCR0, lccr0);
	sa1100_lcdc_write(lcdc, DBAR1, lcdc->shadow_dma);
	sa1100_lcdc_write(lcdc, DBAR2, 0);
	sa1100_lcdc_write(lcdc, LCCR0, lccr0 | LCCR0_LEN);

	schedule_delayed_work(&lcdc->unmask_errors_work,
			      msecs_to_jiffies(SA1100_LCDC_STARTUP_ERRORS_MS));

	drm_dev_exit(idx);
}

/*
 * Not guarded by drm_dev_enter()/drm_dev_exit(), so the controller is stopped
 * even after the device was unplugged.
 */
static void sa1100_lcdc_crtc_atomic_disable(struct drm_crtc *crtc,
					    struct drm_atomic_commit *state)
{
	struct sa1100_lcdc *lcdc = to_sa1100_lcdc(crtc->dev);
	u32 lccr0;

	/* The enable above bailed out before it touched the controller. */
	if (!lcdc->clk_enabled)
		return;

	cancel_delayed_work_sync(&lcdc->unmask_errors_work);

	reinit_completion(&lcdc->disable_done);

	sa1100_lcdc_write(lcdc, LCSR, ~0u);

	/* Unmask the disable-done source and mask the errors -- the output
	 * FIFO runs dry while the last frame is finished -- then clear LEN,
	 * this part's only disable request, always graceful (see
	 * sa1100_lcdc_quiesce()).
	 */
	lccr0 = (sa1100_lcdc_read(lcdc, LCCR0) & ~LCCR0_LDM) | LCCR0_ERM;
	sa1100_lcdc_write(lcdc, LCCR0, lccr0 & ~LCCR0_LEN);

	if (!wait_for_completion_timeout(&lcdc->disable_done,
					 msecs_to_jiffies(SA1100_LCDC_DISABLE_TIMEOUT_MS)))
		drm_err(&lcdc->drm, "timed out waiting for the controller to stop\n");

	sa1100_lcdc_write(lcdc, LCCR0, lccr0 | LCCR0_LDM);

	clk_disable_unprepare(lcdc->clk);
	lcdc->clk_enabled = false;
}

static const struct drm_crtc_helper_funcs sa1100_lcdc_crtc_helper_funcs = {
	.atomic_check = sa1100_lcdc_crtc_atomic_check,
	.atomic_enable = sa1100_lcdc_crtc_atomic_enable,
	.atomic_disable = sa1100_lcdc_crtc_atomic_disable,
};

static const struct drm_crtc_funcs sa1100_lcdc_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
};

static const struct drm_encoder_funcs sa1100_lcdc_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int sa1100_lcdc_pipe_init(struct sa1100_lcdc *lcdc, struct drm_bridge *bridge)
{
	struct drm_device *drm = &lcdc->drm;
	int ret;

	ret = drm_universal_plane_init(drm, &lcdc->plane, 0,
				       &sa1100_lcdc_plane_funcs,
				       sa1100_lcdc_formats,
				       ARRAY_SIZE(sa1100_lcdc_formats),
				       sa1100_lcdc_format_modifiers,
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&lcdc->plane, &sa1100_lcdc_plane_helper_funcs);
	drm_plane_enable_fb_damage_clips(&lcdc->plane);

	ret = drm_crtc_init_with_planes(drm, &lcdc->crtc, &lcdc->plane, NULL,
					&sa1100_lcdc_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(&lcdc->crtc, &sa1100_lcdc_crtc_helper_funcs);

	ret = drm_encoder_init(drm, &lcdc->encoder, &sa1100_lcdc_encoder_funcs,
			       DRM_MODE_ENCODER_DPI, NULL);
	if (ret)
		return ret;
	lcdc->encoder.possible_crtcs = drm_crtc_mask(&lcdc->crtc);

	ret = drm_bridge_attach(&lcdc->encoder, bridge, NULL, 0);
	if (ret)
		return ret;

	lcdc->connector = drm_panel_bridge_connector(bridge);
	if (IS_ERR(lcdc->connector))
		return PTR_ERR(lcdc->connector);

	return 0;
}

static const struct drm_mode_config_funcs sa1100_lcdc_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int sa1100_lcdc_mode_config_init(struct sa1100_lcdc *lcdc)
{
	struct drm_device *drm = &lcdc->drm;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	drm->mode_config.min_width = 16;
	drm->mode_config.max_width = SA1100_LCDC_MAX_WIDTH;
	drm->mode_config.min_height = 1;
	drm->mode_config.max_height = SA1100_LCDC_MAX_HEIGHT;
	drm->mode_config.preferred_depth = 16;
	drm->mode_config.funcs = &sa1100_lcdc_mode_config_funcs;

	return 0;
}

DEFINE_DRM_GEM_DMA_FOPS(sa1100_lcdc_fops);

static const struct drm_driver sa1100_lcdc_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,

	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.major = 1,
	.minor = 0,

	.fops = &sa1100_lcdc_fops,
	/*
	 * The plane update copies from the CPU address of the GEM object,
	 * which an imported buffer only has with the _VMAP variant.
	 */
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
	DRM_FBDEV_DMA_DRIVER_OPS,
};

static int sa1100_lcdc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct drm_bridge *bridge;
	struct sa1100_lcdc *lcdc;
	struct drm_device *drm;
	int ret, irq;

	lcdc = devm_drm_dev_alloc(dev, &sa1100_lcdc_driver, struct sa1100_lcdc, drm);
	if (IS_ERR(lcdc))
		return PTR_ERR(lcdc);
	drm = &lcdc->drm;

	platform_set_drvdata(pdev, drm);
	init_completion(&lcdc->disable_done);
	INIT_DELAYED_WORK(&lcdc->unmask_errors_work, sa1100_lcdc_unmask_errors);

	lcdc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(lcdc->base))
		return PTR_ERR(lcdc->base);

	/*
	 * See this file's own "clock" DOC comment: the device tree node
	 * names a single unnamed clock, expected to be the PLL-derived CPU
	 * clock this part's pixel-clock divider actually runs from.
	 */
	lcdc->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(lcdc->clk))
		return dev_err_probe(dev, PTR_ERR(lcdc->clk),
				     "failed to get the LCD clock\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	lcdc->shadow = dmam_alloc_coherent(dev, SA1100_LCDC_SHADOW_SIZE,
					   &lcdc->shadow_dma, GFP_KERNEL);
	if (!lcdc->shadow)
		return -ENOMEM;

	ret = clk_prepare_enable(lcdc->clk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable the LCD clock\n");

	/*
	 * The bootloader may hand the controller over still scanning out of
	 * memory this kernel does not own, see sa1100_lcdc_quiesce().
	 */
	sa1100_lcdc_quiesce(lcdc);

	clk_disable_unprepare(lcdc->clk);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, sa1100_lcdc_irq, 0, DRIVER_NAME, lcdc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request the LCD interrupt\n");

	bridge = devm_drm_of_get_bridge(dev, dev->of_node, 0, 0);
	if (IS_ERR(bridge))
		return dev_err_probe(dev, PTR_ERR(bridge),
				     "failed to find the panel or bridge\n");

	ret = sa1100_lcdc_mode_config_init(lcdc);
	if (ret)
		return ret;

	ret = sa1100_lcdc_pipe_init(lcdc, bridge);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	drm_client_setup_with_fourcc(drm, DRM_FORMAT_RGB565);

	return 0;
}

static void sa1100_lcdc_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static void sa1100_lcdc_shutdown(struct platform_device *pdev)
{
	drm_atomic_helper_shutdown(platform_get_drvdata(pdev));
}

static int sa1100_lcdc_suspend(struct device *dev)
{
	return drm_mode_config_helper_suspend(dev_get_drvdata(dev));
}

static int sa1100_lcdc_resume(struct device *dev)
{
	return drm_mode_config_helper_resume(dev_get_drvdata(dev));
}

static DEFINE_SIMPLE_DEV_PM_OPS(sa1100_lcdc_pm_ops, sa1100_lcdc_suspend, sa1100_lcdc_resume);

static const struct of_device_id sa1100_lcdc_of_match[] = {
	{ .compatible = "intel,sa1110-lcdc" },
	{ .compatible = "intel,sa1100-lcdc" },
	{ }
};
MODULE_DEVICE_TABLE(of, sa1100_lcdc_of_match);

static struct platform_driver sa1100_lcdc_platform_driver = {
	.probe = sa1100_lcdc_probe,
	.remove = sa1100_lcdc_remove,
	.shutdown = sa1100_lcdc_shutdown,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = sa1100_lcdc_of_match,
		.pm = pm_sleep_ptr(&sa1100_lcdc_pm_ops),
	},
};

drm_module_platform_driver(sa1100_lcdc_platform_driver);

MODULE_AUTHOR("Koen Kooi <koen@dominion.thruhere.net>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
