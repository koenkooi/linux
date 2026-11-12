// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM driver for the Marvell PXA25x integrated LCD controller.
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
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_drv.h>
#include <drm/drm_encoder.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
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
 * This is a KMS driver for the LCD controller integrated in the Marvell
 * PXA25x application processor. It binds the same ``marvell,pxa2xx-lcdc``
 * device tree node that the legacy fbdev driver
 * ``drivers/video/fbdev/pxafb.c`` binds, and lands beside it rather than
 * replacing it, the way ``drm/pl111`` landed beside ``amba-clcd``.
 * ``drivers/Makefile`` links ``video/`` before ``gpu/`` and a device binds
 * the first matching registered driver, so a kernel that wants this driver
 * must not build ``CONFIG_FB_PXA``.
 *
 * The controller scans system memory directly through its own frame
 * descriptor DMA engine, so the framebuffer is a physically contiguous GEM
 * DMA object that the hardware reads in place. The shadow-buffer shape used
 * by the other small DRM drivers for this class of handheld
 * (``drm/tiny/mq11xx.c``, ``drm/tiny/w100.c``) does not apply here: those
 * chips hold their pixels in on-chip VRAM behind a slow 16-bit static memory
 * bank, which is why they render into cached system memory and copy out. This
 * controller has no VRAM at all and there is nothing to copy into.
 *
 * Panel supplies and timings are not this driver's business. They belong to a
 * &drm_panel reached over the OF graph, because this is generic PXA25x
 * silicon and per-board rail sequencing inside it would be board detection in
 * a shared driver. The panel bridge owns the connector.
 *
 * The driver matches ``marvell,pxa2xx-lcdc``, which the shared PXA2xx
 * devicetree include gives to the controller on PXA25x, PXA27x and PXA3xx
 * alike. The PXA27x and PXA3xx variants add palette-format and per-overlay
 * mask registers at 0x010 and 0x014 and a second DMA channel set, none of
 * which this driver programs, so probe declines any machine that is not
 * compatible with ``marvell,pxa250`` before it touches the hardware.
 *
 * DOC: vblank
 *
 * The driver takes the controller interrupt but does not implement vblank.
 * The hardware has the sources for it -- ``LCSR_SOF``, ``LCSR_EOF`` and
 * ``LCSR_BS``, gated by ``LCCR0_SFM``, ``LCCR0_EFM`` and ``LCCR0_BM`` -- but
 * whether end-of-frame actually asserts once per frame from a self-linked
 * descriptor, or additionally requires a per-descriptor interrupt bit in
 * LDCMD, is not answerable from the register definitions available here. A
 * driver that calls drm_vblank_init() and then never receives an interrupt
 * makes every atomic commit wait for an event that never arrives, which on a
 * machine whose only console is the panel is unrecoverable. Losing the
 * feature is recoverable. The experiment that unblocks it is one boot with
 * ``LCCR0_EFM`` cleared, counting ``LCSR_EOF`` over a second.
 *
 * Without vblank support drm_atomic_helper_check_modeset() marks every CRTC
 * state ``no_vblank`` and the default commit tail completes flip events
 * through drm_atomic_helper_fake_vblank(), so event delivery still works.
 *
 * The interrupt is claimed for two other reasons. It carries ``LCSR_LDD``,
 * the only signal that a graceful disable has finished and that scanout DMA
 * has therefore stopped, which must be known before the clock is gated. And
 * ``LCSR_BER`` (bus error) and ``LCSR_ABC`` have no mask bit in LCCR0 at all,
 * so an unclaimed line could be asserted forever with no handler.
 */

#define DRIVER_NAME	"pxa-lcdc"
#define DRIVER_DESC	"Marvell PXA25x LCD controller"

/*
 * Register offsets and bit definitions, transcribed from
 * drivers/video/fbdev/pxa3xx-regs.h. That header is private to the fbdev
 * driver and is not exported, so the subset this driver needs is carried
 * here rather than included across subsystems. Only the registers a PXA25x
 * single-panel active TFT configuration uses appear below; the palette,
 * overlay, smart-panel and second-channel registers are absent
 * because this driver never touches them.
 */
#define LCCR0			0x000
#define LCCR1			0x004
#define LCCR2			0x008
#define LCCR3			0x00c
#define FBR0			0x020
#define LCSR			0x038
#define FDADR0			0x200

#define LCCR0_ENB		BIT(0)	/* controller enable */
#define LCCR0_LDM		BIT(3)	/* disable-done interrupt mask */
#define LCCR0_SFM		BIT(4)	/* start-of-frame interrupt mask */
#define LCCR0_IUM		BIT(5)	/* input FIFO underrun mask */
#define LCCR0_EFM		BIT(6)	/* end-of-frame interrupt mask */
#define LCCR0_PAS		BIT(7)	/* active (TFT) rather than passive */
#define LCCR0_DIS		BIT(10)	/* graceful disable */
#define LCCR0_QDM		BIT(11)	/* quick-disable interrupt mask */
#define LCCR0_BM		BIT(20)	/* branch interrupt mask */
#define LCCR0_OUM		BIT(21)	/* output FIFO underrun mask */

/* Every maskable interrupt source the controller has on this part. */
#define LCCR0_ALL_MASKS		(LCCR0_LDM | LCCR0_SFM | LCCR0_IUM | \
				 LCCR0_EFM | LCCR0_QDM | LCCR0_BM | LCCR0_OUM)

/*
 * Every source is masked while the controller runs, the FIFO underrun ones
 * included, as the driver has no per-frame use for them. Only the
 * disable-done source is unmasked, for the duration of a graceful disable.
 */

#define LCCR1_PPL(x)		(((x) - 1) << 0)	/* pixels per line */
#define LCCR1_HSW(x)		(((x) - 1) << 10)	/* hsync width */
#define LCCR1_ELW(x)		(((x) - 1) << 16)	/* front porch */
#define LCCR1_BLW(x)		(((x) - 1) << 24)	/* back porch */

#define LCCR2_LPP(x)		(((x) - 1) << 0)	/* lines per panel */
#define LCCR2_VSW(x)		(((x) - 1) << 10)	/* vsync width */
#define LCCR2_EFW(x)		((x) << 16)	/* front porch, not biased */
#define LCCR2_BFW(x)		((x) << 24)	/* back porch, not biased */

#define LCCR3_PCD(x)		((x) << 0)	/* pixel clock divisor */
#define LCCR3_PCD_MAX		255
#define LCCR3_VSP		BIT(20)	/* vsync active low */
#define LCCR3_HSP		BIT(21)	/* hsync active low */
#define LCCR3_PCP		BIT(22)	/* data driven on the falling edge */
#define LCCR3_OEP		BIT(23)	/* output enable active low */
/*
 * The bits-per-pixel field is three bits at 24 with a fourth bit at 29 on
 * later parts. This driver scans out RGB565 and nothing else, so only that
 * one encoding is named.
 */
#define LCCR3_BPP_RGB565	(4 << 24)

#define LCSR_LDD		BIT(0)	/* disable done */
#define LCSR_BER		BIT(2)	/* bus error */
#define LCSR_IUL		BIT(4)	/* input FIFO underrun, lower panel */
#define LCSR_IUU		BIT(5)	/* input FIFO underrun, upper panel */
#define LCSR_OU			BIT(6)	/* output FIFO underrun */

#define LCSR_ERRORS		(LCSR_BER | LCSR_IUL | LCSR_IUU | LCSR_OU)

/* Bit 0 of a frame branch register address is the branch request itself. */
#define FBR_BRANCH		BIT(0)

/*
 * One frame descriptor finishes a frame and then reloads the descriptor at
 * @fdadr. Pointing @fdadr back at the descriptor itself makes the controller
 * re-read the same four words every frame, which is what lets a scanout
 * address change take effect without stopping the controller.
 */
struct pxa_lcdc_dma_desc {
	u32 fdadr;
	u32 fsadr;
	u32 fidr;
	u32 ldcmd;
};

/*
 * A graceful disable finishes the frame in progress before it completes, so
 * the wait has to outlast one frame. One frame is 20.5 ms at the 48.7 Hz this
 * controller runs a 320x240 panel at, and the slowest reachable pixel clock
 * roughly doubles that, so 100 ms is several frames of slack.
 */
#define PXA_LCDC_DISABLE_TIMEOUT_MS	100

/*
 * Warn, but still program the divider, when the achievable pixel clock misses
 * the mode by more than this. The divider is an integer, so its granularity
 * near a typical operating point is several percent; a miss larger than this
 * means the divider did not land on the panel rather than that it rounded.
 */
#define PXA_LCDC_PCLK_TOLERANCE_PCT	5

struct pxa_lcdc {
	struct drm_device drm;
	struct drm_plane plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector *connector;

	void __iomem *base;
	struct clk *clk;
	struct completion disable_done;

	struct pxa_lcdc_dma_desc *desc;
	dma_addr_t desc_dma;
};

static inline struct pxa_lcdc *to_pxa_lcdc(struct drm_device *drm)
{
	return container_of(drm, struct pxa_lcdc, drm);
}

static u32 pxa_lcdc_read(struct pxa_lcdc *lcdc, u32 reg)
{
	return readl(lcdc->base + reg);
}

static void pxa_lcdc_write(struct pxa_lcdc *lcdc, u32 reg, u32 val)
{
	writel(val, lcdc->base + reg);
}

/*
 * The register wants LCLK / (2 * pixel clock) - 1. Note that pxafb's
 * get_pcd() computes LCLK / (2 * pixel clock) and truncates, dropping the
 * bias, so it programs one divider step too slow.
 */
static u32 pxa_lcdc_pcd(struct pxa_lcdc *lcdc, const struct drm_display_mode *mode)
{
	unsigned long rate = clk_get_rate(lcdc->clk);
	unsigned long target = mode->clock * 1000UL;
	unsigned long achieved;
	unsigned long delta;
	u32 pcd;

	if (!target)
		return 0;

	pcd = DIV_ROUND_CLOSEST(rate, 2 * target);
	pcd = clamp_t(u32, pcd, 1, LCCR3_PCD_MAX + 1) - 1;

	achieved = rate / (2 * (pcd + 1));
	delta = abs_diff(achieved, target);
	if (delta > target / 100 * PXA_LCDC_PCLK_TOLERANCE_PCT)
		drm_warn(&lcdc->drm,
			 "pixel clock %lu Hz requested, %lu Hz programmed\n",
			 target, achieved);

	drm_dbg_kms(&lcdc->drm, "LCD clock %lu Hz, PCD %u, pixel clock %lu Hz\n",
		    rate, pcd, achieved);

	return pcd;
}

/*
 * Stop a running controller and leave every interrupt source masked.
 *
 * Clearing LCCR0_ENB is the quick disable: it stops the fetch immediately
 * rather than at the end of the frame. That is what is wanted for a
 * controller inherited from the bootloader, whose frame descriptor points at
 * an address this kernel does not own; letting it finish a frame would leave
 * a DMA master reading a foreign address for up to another frame. The panel's
 * rails are still down at this point, so there is no picture to preserve.
 */
static void pxa_lcdc_quiesce(struct pxa_lcdc *lcdc)
{
	u32 lccr0 = pxa_lcdc_read(lcdc, LCCR0);

	pxa_lcdc_write(lcdc, LCCR0, (lccr0 | LCCR0_ALL_MASKS) & ~LCCR0_ENB);
	pxa_lcdc_write(lcdc, LCSR, pxa_lcdc_read(lcdc, LCSR));
}

static irqreturn_t pxa_lcdc_irq(int irq, void *data)
{
	struct pxa_lcdc *lcdc = data;
	u32 lcsr;

	lcsr = pxa_lcdc_read(lcdc, LCSR);
	if (!lcsr)
		return IRQ_NONE;

	if (lcsr & LCSR_LDD) {
		u32 lccr0 = pxa_lcdc_read(lcdc, LCCR0);

		pxa_lcdc_write(lcdc, LCCR0, lccr0 | LCCR0_LDM);
		complete(&lcdc->disable_done);
	}

	if (lcsr & LCSR_ERRORS)
		drm_err_ratelimited(&lcdc->drm, "controller error, LCSR 0x%08x\n",
				    lcsr);

	/* Write-one-to-clear: acknowledge exactly the bits that were read. */
	pxa_lcdc_write(lcdc, LCSR, lcsr);

	return IRQ_HANDLED;
}

static const u32 pxa_lcdc_formats[] = {
	DRM_FORMAT_RGB565,
};

static const u64 pxa_lcdc_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

static int pxa_lcdc_plane_atomic_check(struct drm_plane *plane,
				       struct drm_atomic_commit *state)
{
	struct drm_plane_state *new_plane_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *new_crtc_state = NULL;
	struct drm_framebuffer *fb = new_plane_state->fb;
	unsigned int pitch;
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
	 * The controller has no stride register: it walks the frame as one
	 * contiguous run of hdisplay * cpp bytes per line, so a pitch wider
	 * than the scanned width shears the picture.
	 */
	pitch = (drm_rect_width(&new_plane_state->src) >> 16) *
		fb->format->cpp[0];
	if (fb->pitches[0] != pitch) {
		drm_dbg_kms(plane->dev, "pitch %u must be %u\n",
			    fb->pitches[0], pitch);
		return -EINVAL;
	}

	return 0;
}

/*
 * Update the scanout address in place. The controller reloads the frame
 * descriptor at the end of every frame because the descriptor links to
 * itself, so a new address is picked up at the next frame boundary. The
 * write is a single naturally aligned 32-bit store into DMA-coherent memory,
 * so a concurrent descriptor fetch reads either the old address or the new
 * one, never a mixture.
 *
 * The controller also offers an immediate branch through FBR0, which takes
 * effect on the current frame. That is not used here because the only signal
 * that a branch has been consumed is LCSR_BS, and this driver does not
 * implement vblank, so a second address change inside one frame would race
 * the first with nothing to serialise against.
 */
static void pxa_lcdc_plane_atomic_update(struct drm_plane *plane,
					 struct drm_atomic_commit *state)
{
	struct pxa_lcdc *lcdc = to_pxa_lcdc(plane->dev);
	struct drm_plane_state *plane_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct drm_framebuffer *fb = plane_state->fb;
	int idx;

	if (!fb)
		return;

	if (!drm_dev_enter(&lcdc->drm, &idx))
		return;

	/*
	 * The frame length is the scanned rectangle, not the framebuffer:
	 * a source offset moves the start address, and the controller has
	 * no concept of a framebuffer taller than the frame it scans.
	 */
	lcdc->desc->fsadr = drm_fb_dma_get_gem_addr(fb, plane_state, 0);
	lcdc->desc->ldcmd = fb->pitches[0] *
			    (drm_rect_height(&plane_state->src) >> 16);

	drm_dev_exit(idx);
}

static const struct drm_plane_helper_funcs pxa_lcdc_plane_helper_funcs = {
	.prepare_fb = drm_gem_plane_helper_prepare_fb,
	.atomic_check = pxa_lcdc_plane_atomic_check,
	.atomic_update = pxa_lcdc_plane_atomic_update,
};

static const struct drm_plane_funcs pxa_lcdc_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static int pxa_lcdc_crtc_atomic_check(struct drm_crtc *crtc,
				      struct drm_atomic_commit *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	if (!crtc_state->enable)
		return 0;

	return drm_atomic_helper_check_crtc_primary_plane(crtc_state);
}

/*
 * Turn the controller on. The register order is the one the programmer's
 * manual gives for this part, and the parts of it that matter are that the
 * frame descriptor address is loaded while the enable bit is clear and that
 * the enable bit is set last, because setting it starts the descriptor fetch.
 *
 * LCCR4 and LCCR5 are never written. On the PXA27x and PXA3xx they hold
 * palette formats and per-overlay masks; this part has neither, and this
 * driver programs neither on any part. FDADR1 is not written either: the
 * second DMA channel exists only for a dual-scan panel, which LCCR0_SDS
 * selects and which this driver never sets.
 */
static void pxa_lcdc_crtc_atomic_enable(struct drm_crtc *crtc,
					struct drm_atomic_commit *state)
{
	struct pxa_lcdc *lcdc = to_pxa_lcdc(crtc->dev);
	const struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	const struct drm_display_info *info = &lcdc->connector->display_info;
	u32 lccr0, lccr1, lccr2, lccr3;
	int idx;

	if (!drm_dev_enter(&lcdc->drm, &idx))
		return;

	lccr0 = LCCR0_PAS | LCCR0_ALL_MASKS;

	lccr1 = LCCR1_PPL(mode->hdisplay) |
		LCCR1_HSW(mode->hsync_end - mode->hsync_start) |
		LCCR1_ELW(mode->hsync_start - mode->hdisplay) |
		LCCR1_BLW(mode->htotal - mode->hsync_end);

	lccr2 = LCCR2_LPP(mode->vdisplay) |
		LCCR2_VSW(mode->vsync_end - mode->vsync_start) |
		LCCR2_EFW(mode->vsync_start - mode->vdisplay) |
		LCCR2_BFW(mode->vtotal - mode->vsync_end);

	lccr3 = LCCR3_PCD(pxa_lcdc_pcd(lcdc, mode)) | LCCR3_BPP_RGB565;
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

	pxa_lcdc_write(lcdc, LCCR3, lccr3);
	pxa_lcdc_write(lcdc, LCCR2, lccr2);
	pxa_lcdc_write(lcdc, LCCR1, lccr1);
	pxa_lcdc_write(lcdc, LCCR0, lccr0);
	pxa_lcdc_write(lcdc, FDADR0, lcdc->desc_dma);
	pxa_lcdc_write(lcdc, LCCR0, lccr0 | LCCR0_ENB);

	drm_dev_exit(idx);
}

/*
 * Not guarded by drm_dev_enter()/drm_dev_exit(), unlike the enable above:
 * remove() unplugs the device before shutting the atomic state down, so a
 * guarded disable would be dead code on a normal unbind, and this path must
 * run because it is what stops scanout DMA before the buffer goes away.
 */
static void pxa_lcdc_crtc_atomic_disable(struct drm_crtc *crtc,
					 struct drm_atomic_commit *state)
{
	struct pxa_lcdc *lcdc = to_pxa_lcdc(crtc->dev);
	u32 lccr0;

	/*
	 * The completion is rearmed before every wait. A completion that is
	 * only initialised once stays signalled after a late completion, and
	 * the next disable then returns immediately and gates the clock on a
	 * controller that is still scanning.
	 */
	reinit_completion(&lcdc->disable_done);

	pxa_lcdc_write(lcdc, LCSR, ~0u);

	/*
	 * Two writes, in this order: the disable-done source has to be
	 * unmasked before the disable is requested.
	 */
	lccr0 = pxa_lcdc_read(lcdc, LCCR0) & ~LCCR0_LDM;
	pxa_lcdc_write(lcdc, LCCR0, lccr0);
	pxa_lcdc_write(lcdc, LCCR0, lccr0 | LCCR0_DIS);

	if (!wait_for_completion_timeout(&lcdc->disable_done,
					 msecs_to_jiffies(PXA_LCDC_DISABLE_TIMEOUT_MS)))
		drm_err(&lcdc->drm, "timed out waiting for the controller to stop\n");

	pxa_lcdc_write(lcdc, LCCR0, lccr0 | LCCR0_LDM);

	/*
	 * Only now: gating the clock while the controller still scans strands
	 * the panel's sync signals part way through a frame.
	 */
	clk_disable_unprepare(lcdc->clk);
}

static const struct drm_crtc_helper_funcs pxa_lcdc_crtc_helper_funcs = {
	.atomic_check = pxa_lcdc_crtc_atomic_check,
	.atomic_enable = pxa_lcdc_crtc_atomic_enable,
	.atomic_disable = pxa_lcdc_crtc_atomic_disable,
};

static const struct drm_crtc_funcs pxa_lcdc_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
};

static const struct drm_encoder_funcs pxa_lcdc_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int pxa_lcdc_pipe_init(struct pxa_lcdc *lcdc, struct drm_bridge *bridge)
{
	struct drm_device *drm = &lcdc->drm;
	int ret;

	ret = drm_universal_plane_init(drm, &lcdc->plane, 0,
				       &pxa_lcdc_plane_funcs,
				       pxa_lcdc_formats,
				       ARRAY_SIZE(pxa_lcdc_formats),
				       pxa_lcdc_format_modifiers,
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&lcdc->plane, &pxa_lcdc_plane_helper_funcs);

	ret = drm_crtc_init_with_planes(drm, &lcdc->crtc, &lcdc->plane, NULL,
					&pxa_lcdc_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(&lcdc->crtc, &pxa_lcdc_crtc_helper_funcs);

	ret = drm_encoder_init(drm, &lcdc->encoder, &pxa_lcdc_encoder_funcs,
			       DRM_MODE_ENCODER_DPI, NULL);
	if (ret)
		return ret;
	lcdc->encoder.possible_crtcs = drm_crtc_mask(&lcdc->crtc);

	ret = drm_bridge_attach(&lcdc->encoder, bridge, NULL, 0);
	if (ret)
		return ret;

	/*
	 * The panel bridge creates the connector. Keeping a pointer to it is
	 * what lets the CRTC read the panel's bus flags at enable time
	 * instead of hardcoding a board's signal polarities.
	 */
	lcdc->connector = drm_panel_bridge_connector(bridge);
	if (IS_ERR(lcdc->connector))
		return PTR_ERR(lcdc->connector);

	return 0;
}

static const struct drm_mode_config_funcs pxa_lcdc_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int pxa_lcdc_mode_config_init(struct pxa_lcdc *lcdc)
{
	struct drm_device *drm = &lcdc->drm;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	/*
	 * Both active-size fields in LCCR1 and LCCR2 are ten bits wide and
	 * encoded minus one, so the controller spans one to 1024 in each
	 * direction. The mode itself comes from the panel.
	 */
	drm->mode_config.min_width = 1;
	drm->mode_config.max_width = 1024;
	drm->mode_config.min_height = 1;
	drm->mode_config.max_height = 1024;
	drm->mode_config.preferred_depth = 16;
	drm->mode_config.funcs = &pxa_lcdc_mode_config_funcs;

	return 0;
}

DEFINE_DRM_GEM_DMA_FOPS(pxa_lcdc_fops);

static const struct drm_driver pxa_lcdc_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,

	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.major = 1,
	.minor = 0,

	.fops = &pxa_lcdc_fops,
	DRM_GEM_DMA_DRIVER_OPS,
	DRM_FBDEV_DMA_DRIVER_OPS,
};

static void pxa_lcdc_release_rmem(void *data)
{
	of_reserved_mem_device_release(data);
}

static int pxa_lcdc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct drm_bridge *bridge;
	struct pxa_lcdc *lcdc;
	struct drm_device *drm;
	int ret, irq;

	if (!of_machine_is_compatible("marvell,pxa250"))
		return -ENODEV;

	/*
	 * Scanout memory must be physically contiguous. Without a dedicated
	 * region every frame buffer is a high-order buddy allocation, which
	 * fails once a small, swapping system has fragmented its memory.
	 * The region is attached ahead of the DRM device so that it is
	 * released only after the device's own managed teardown has run.
	 */
	ret = of_reserved_mem_device_init(dev);
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret,
				     "failed to attach the reserved memory\n");
	if (!ret) {
		ret = devm_add_action_or_reset(dev, pxa_lcdc_release_rmem, dev);
		if (ret)
			return ret;
	}

	lcdc = devm_drm_dev_alloc(dev, &pxa_lcdc_driver, struct pxa_lcdc, drm);
	if (IS_ERR(lcdc))
		return PTR_ERR(lcdc);
	drm = &lcdc->drm;

	platform_set_drvdata(pdev, drm);
	init_completion(&lcdc->disable_done);

	lcdc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(lcdc->base))
		return PTR_ERR(lcdc->base);

	/*
	 * The device tree node names a single unnamed clock, the controller's
	 * gate. Its rate is the input to the pixel clock divider.
	 */
	lcdc->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(lcdc->clk))
		return dev_err_probe(dev, PTR_ERR(lcdc->clk),
				     "failed to get the LCD clock\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	lcdc->desc = dmam_alloc_coherent(dev, sizeof(*lcdc->desc),
					 &lcdc->desc_dma, GFP_KERNEL);
	if (!lcdc->desc)
		return -ENOMEM;

	/*
	 * The branch request overloads bit 0 of a descriptor address, so a
	 * descriptor address must not have it set.
	 */
	if (WARN_ON(lcdc->desc_dma & FBR_BRANCH))
		return -EINVAL;

	lcdc->desc->fdadr = lcdc->desc_dma;
	lcdc->desc->fidr = 0;

	ret = clk_prepare_enable(lcdc->clk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable the LCD clock\n");

	/*
	 * The bootloader may hand the controller over still scanning, out of
	 * a frame descriptor that belongs to it rather than to this kernel.
	 * Stopping it once here rather than on every enable means no later
	 * path has to reason about an inherited configuration, and a device
	 * that never receives a modeset is left with the controller stopped
	 * and its clock gated.
	 */
	pxa_lcdc_quiesce(lcdc);

	/*
	 * The LCD clock rate decides the pixel clock the divider can reach.
	 * Reporting it makes a misparented gate visible in the boot log
	 * rather than only on the glass.
	 */
	drm_info(drm, "LCD clock %lu Hz\n", clk_get_rate(lcdc->clk));

	clk_disable_unprepare(lcdc->clk);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, pxa_lcdc_irq, 0, DRIVER_NAME, lcdc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request the LCD interrupt\n");

	bridge = devm_drm_of_get_bridge(dev, dev->of_node, 0, 0);
	if (IS_ERR(bridge))
		return dev_err_probe(dev, PTR_ERR(bridge),
				     "failed to find the panel or bridge\n");

	ret = pxa_lcdc_mode_config_init(lcdc);
	if (ret)
		return ret;

	ret = pxa_lcdc_pipe_init(lcdc, bridge);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	drm_client_setup_with_fourcc(drm, DRM_FORMAT_RGB565);

	return 0;
}

static void pxa_lcdc_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static void pxa_lcdc_shutdown(struct platform_device *pdev)
{
	drm_atomic_helper_shutdown(platform_get_drvdata(pdev));
}

static int pxa_lcdc_suspend(struct device *dev)
{
	return drm_mode_config_helper_suspend(dev_get_drvdata(dev));
}

static int pxa_lcdc_resume(struct device *dev)
{
	return drm_mode_config_helper_resume(dev_get_drvdata(dev));
}

static DEFINE_SIMPLE_DEV_PM_OPS(pxa_lcdc_pm_ops, pxa_lcdc_suspend, pxa_lcdc_resume);

static const struct of_device_id pxa_lcdc_of_match[] = {
	{ .compatible = "marvell,pxa2xx-lcdc" },
	{ }
};
MODULE_DEVICE_TABLE(of, pxa_lcdc_of_match);

static struct platform_driver pxa_lcdc_platform_driver = {
	.probe = pxa_lcdc_probe,
	.remove = pxa_lcdc_remove,
	.shutdown = pxa_lcdc_shutdown,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = pxa_lcdc_of_match,
		.pm = pm_sleep_ptr(&pxa_lcdc_pm_ops),
	},
};

drm_module_platform_driver(pxa_lcdc_platform_driver);

MODULE_AUTHOR("Koen Kooi <koen@dominion.thruhere.net>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
