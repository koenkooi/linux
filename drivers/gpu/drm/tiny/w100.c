// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM driver for the ATI Imageon W3220 ("w32xx"), the external display
 * controller on the HP iPAQ hx4700. It sits on the PXA27x nCS2 static
 * memory bank as a plain 32-bit memory-mapped device with its own 16-byte
 * indirect config window, an 8 KiB register window and up to 2 MiB of
 * on-chip SDRAM. There is no PXA LCD controller involved: the chip drives
 * the Sony panel's parallel RGB interface itself.
 *
 * The structure follows mq11xx.c: a shmem-backed shadow plane that is
 * damage-clipped and copied into video memory on each atomic update. The
 * chip has two real panel timings, 480x640 (native) and 240x320 (a
 * hardware quarter-size mode, not a software scale), which are exposed as
 * two DRM modes.
 *
 * The register map and the bring-up sequence come from the former
 * drivers/video/fbdev/w100fb.c and w100fb.h, and the board data (register
 * defaults, mode timings, memory configuration, GPIO setup and the
 * crystal frequency) from the former arch/arm/mach-pxa/hx4700.c. Register
 * and field comments below cite the w100fb source they were taken from.
 *
 * The chip's GFX block, a ROP3 blitter that addresses chip memory only,
 * is brought up too. It is driven by polled register writes, needs no
 * microcode, and is used for the power-on video-memory clear and for
 * solid-colour damage fills.
 *
 * The same blitter can also be driven through the chip's command
 * processor: the host writes CCE command packets into a ring buffer in
 * video memory and the processor executes them without a register write
 * per operation. That path needs the processor's microcode, which is not
 * part of the kernel; it is loaded with request_firmware() from
 * imageon/W3220_cp.bin, following the r128/radeon precedent for ATI CCE
 * microcode. It is optional: without the firmware file, or without the
 * wrap aperture in the device tree, the driver says so once and keeps
 * programming the engine by polled register writes. The only source for
 * this block is the kdrive ati_dma.c and w100_regs.h; w100fb.c defines no
 * mmCP_* register.
 *
 * The chip's own fixed-function video front-end is driven as a second
 * KMS plane (DRM_PLANE_TYPE_OVERLAY): planar YUV scanned out of video
 * memory, converted to RGB and integer-upscaled in hardware, and
 * composited above the graphics plane by the CRTC. Neither w100fb nor
 * kdrive touched that block, so the overlay register protocol is ported
 * from libw100 0.0.2's lib/overlay.c (GPLv2, Manuel Teira, 2006), with
 * the mplayer libvo backend vo_imageon.c as the reference consumer. The
 * overlay section below cites the per-register sources.
 *
 * Not carried over: the chip-ID branches for the W100 and W3200, the
 * rotate/flip modes (the two modes here are independent panel timings),
 * suspend/resume, and the chip interrupt, which is not described in the
 * device tree. Vblank events come from a timer instead.
 *
 * Panel power is board GPIO sequencing, ported from the former
 * sony_lcd_init() and sony_lcd_off(), and is driven from the CRTC's
 * atomic_enable and atomic_disable. The chip's own reset line is
 * requested once at probe and never toggled again.
 *
 * w100fb ran its hardware init once at probe and on blank/unblank
 * toggled only the panel power GPIOs. DRM cannot tell a blank from a
 * modeset inside the CRTC helper, so atomic_enable replays the whole mode
 * programming sequence (memory controller, PLL, CRTC and LCD registers)
 * every time. Every write is a full field assignment, so this is
 * idempotent, at the cost of more register traffic on unblank.
 */

#include <linux/backlight.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

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
#include <drm/drm_modes.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_plane.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_property.h>
#include <drm/drm_rect.h>
#include <drm/drm_vblank.h>
#include <drm/drm_vblank_helper.h>

#define DRIVER_NAME "w100"
#define DRIVER_DESC "ATI Imageon W3220 display driver"
#define DRIVER_MAJOR 1
#define DRIVER_MINOR 0

/*
 * ------------------------------------------------------------------
 * Register map, transcribed from w100fb.h's "Block CIF"/"Block CG"/
 * "Block DISPLAY"/"Block MC"/"Block BM" sections. Offsets are real ATI
 * register mnemonics (the "mm"/"cfg" prefixes in the recovered header
 * are the vendor's own naming for "memory-mapped" vs. the CIF's own
 * indirect 8-bit config space), kept close to the original names so the
 * two sources stay diffable.
 */

/* Indirect CIF config space -- 16 bytes at chip offset 0, 8-bit registers.
 * w100fb.h's cfg* offsets.
 */
#define W100_CFG_LEN			0x10
#define W100_CFG_REG_BASE		0x04	/* cfgREG_BASE */
#define W100_CFG_INTF_CNTL		0x05	/* cfgINTF_CNTL */
#define W100_CFG_STATUS		0x06	/* cfgSTATUS */
#define W100_CFG_CPU_DEFAULTS		0x07	/* cfgCPU_DEFAULTS */

/* Memory-mapped register window -- 8 KiB at chip offset 0x10000.
 * w100fb.h's mm* offsets.
 */
#define W100_REG_BASE			0x10000
#define W100_REG_LEN			0x2000

#define W100_REG_CHIP_ID		0x0000	/* mmCHIP_ID */
#define W100_REG_WRAP_TOP_DIR		0x0010	/* mmWRAP_TOP_DIR */
#define W100_REG_WRAP_START_DIR	0x0014	/* mmWRAP_START_DIR */
#define W100_REG_CIF_CNTL		0x0018	/* mmCIF_CNTL */
#define W100_REG_CFGREG_BASE		0x001c	/* mmCFGREG_BASE */
#define W100_REG_CIF_IO		0x0020	/* mmCIF_IO */
#define W100_REG_CIF_READ_DBG		0x0024	/* mmCIF_READ_DBG */
#define W100_REG_CIF_WRITE_DBG		0x0028	/* mmCIF_WRITE_DBG */

#define W100_REG_SCRATCH_UMSK		0x0280	/* mmSCRATCH_UMSK */
#define W100_REG_GEN_INT_CNTL		0x0200	/* mmGEN_INT_CNTL */
#define W100_REG_GEN_INT_STATUS	0x0204	/* mmGEN_INT_STATUS */

#define W100_REG_MEM_EXT_CNTL		0x018c	/* mmMEM_EXT_CNTL */
#define W100_REG_MC_FB_LOCATION	0x0188	/* mmMC_FB_LOCATION */
#define W100_REG_MC_EXT_MEM_LOCATION	0x0190	/* mmMC_EXT_MEM_LOCATION */
#define W100_REG_MEM_EXT_TIMING_CNTL	0x0194	/* mmMEM_EXT_TIMING_CNTL */
#define W100_REG_MEM_SDRAM_MODE_REG	0x0198	/* mmMEM_SDRAM_MODE_REG */
#define W100_REG_MEM_IO_CNTL		0x019c	/* mmMEM_IO_CNTL */
#define W100_REG_MC_BIST_CTRL		0x01a4	/* mmMC_BIST_CTRL */
#define W100_REG_RBBM_CNTL		0x0144	/* mmRBBM_CNTL */

#define W100_REG_BM_EXT_MEM_BANDWIDTH	0x0a00	/* mmBM_EXT_MEM_BANDWIDTH */
#define W100_REG_BM_OFFSET		0x0a04	/* mmBM_OFFSET */
#define W100_REG_BM_MEM_EXT_TIMING_CNTL 0x0a08	/* mmBM_MEM_EXT_TIMING_CNTL */
#define W100_REG_BM_MEM_EXT_CNTL	0x0a0c	/* mmBM_MEM_EXT_CNTL */
#define W100_REG_BM_MEM_MODE_REG	0x0a10	/* mmBM_MEM_MODE_REG */
#define W100_REG_BM_MEM_IO_CNTL	0x0a18	/* mmBM_MEM_IO_CNTL */
#define W100_REG_BM_CONFIG		0x0a1c	/* mmBM_CONFIG */

#define W100_REG_CLK_PIN_CNTL		0x0080	/* mmCLK_PIN_CNTL */
#define W100_REG_PLL_REF_FB_DIV	0x0084	/* mmPLL_REF_FB_DIV */
#define W100_REG_PLL_CNTL		0x0088	/* mmPLL_CNTL */
#define W100_REG_SCLK_CNTL		0x008c	/* mmSCLK_CNTL */
#define W100_REG_PCLK_CNTL		0x0090	/* mmPCLK_CNTL */
#define W100_REG_CLK_TEST_CNTL		0x0094	/* mmCLK_TEST_CNTL */
#define W100_REG_PWRMGT_CNTL		0x0098	/* mmPWRMGT_CNTL */

#define W100_REG_GRAPHIC_CTRL		0x0414	/* mmGRAPHIC_CTRL */
#define W100_REG_GRAPHIC_OFFSET	0x0418	/* mmGRAPHIC_OFFSET */
#define W100_REG_GRAPHIC_PITCH		0x041c	/* mmGRAPHIC_PITCH */
#define W100_REG_ACTIVE_H_DISP		0x0424	/* mmACTIVE_H_DISP */
#define W100_REG_ACTIVE_V_DISP		0x0428	/* mmACTIVE_V_DISP */
#define W100_REG_GRAPHIC_H_DISP	0x042c	/* mmGRAPHIC_H_DISP */
#define W100_REG_GRAPHIC_V_DISP	0x0430	/* mmGRAPHIC_V_DISP */
#define W100_REG_CRTC_TOTAL		0x0420	/* mmCRTC_TOTAL */
#define W100_REG_DISP_INT_CNTL		0x0488	/* mmDISP_INT_CNTL */
#define W100_REG_CRTC_SS		0x048c	/* mmCRTC_SS */
#define W100_REG_CRTC_LS		0x0490	/* mmCRTC_LS */
#define W100_REG_CRTC_REV		0x0494	/* mmCRTC_REV */
#define W100_REG_CRTC_DCLK		0x049c	/* mmCRTC_DCLK */
#define W100_REG_CRTC_GS		0x04a0	/* mmCRTC_GS */
#define W100_REG_CRTC_VPOS_GS		0x04a4	/* mmCRTC_VPOS_GS */
#define W100_REG_CRTC_GCLK		0x04a8	/* mmCRTC_GCLK */
#define W100_REG_CRTC_GOE		0x04ac	/* mmCRTC_GOE */
#define W100_REG_CRTC_FRAME		0x04b0	/* mmCRTC_FRAME */
#define W100_REG_CRTC_FRAME_VPOS	0x04b4	/* mmCRTC_FRAME_VPOS */
#define W100_REG_LCDD_CNTL1		0x04c4	/* mmLCDD_CNTL1 */
#define W100_REG_LCDD_CNTL2		0x04c8	/* mmLCDD_CNTL2 */
#define W100_REG_GENLCD_CNTL1		0x04cc	/* mmGENLCD_CNTL1 */
#define W100_REG_GENLCD_CNTL2		0x04d0	/* mmGENLCD_CNTL2 */
#define W100_REG_DISP_DB_BUF_CNTL	0x04d8	/* mmDISP_DB_BUF_CNTL */
#define W100_REG_CRTC_DEFAULT_COUNT	0x04e0	/* mmCRTC_DEFAULT_COUNT */
#define W100_REG_LCD_BACKGROUND_COLOR	0x04e4	/* mmLCD_BACKGROUND_COLOR */
#define W100_REG_CRTC_PS1_ACTIVE	0x04f0	/* mmCRTC_PS1_ACTIVE */
#define W100_REG_GENLCD_CNTL3		0x0524	/* mmGENLCD_CNTL3 */
#define W100_REG_GPIO_DATA		0x04b8	/* mmGPIO_DATA */
#define W100_REG_GPIO_CNTL1		0x04bc	/* mmGPIO_CNTL1 */
#define W100_REG_GPIO_CNTL2		0x04c0	/* mmGPIO_CNTL2 */
#define W100_REG_GPIO_DATA2		0x0528	/* mmGPIO_DATA2 */
#define W100_REG_GPIO_CNTL3		0x052c	/* mmGPIO_CNTL3 */
#define W100_REG_GPIO_CNTL4		0x0530	/* mmGPIO_CNTL4 */
#define W100_REG_LCD_FORMAT		0x0410	/* mmLCD_FORMAT */
#define W100_REG_DISP_DEBUG2		0x0538	/* mmDISP_DEBUG2 */

/* Block DISPLAY, video overlay front-end. Real, distinct silicon that both
 * historical drivers left dormant, so these offsets come from libw100
 * 0.0.2's lib/w100regs.h:97-106 (libw100-0.0.2.tar.gz, md5
 * 72983b019c71045237adae72084efc18) -- the only source that ever drove the
 * block. Each is cross-confirmed against the recovered w100fb.h:66-67
 * (mmVIDEO_CTRL/mmGRAPHIC_KEY) and against the same mm* names in kdrive's
 * w100_regs.h, reconstructed from w100.patch.
 */
#define W100_REG_VIDEO_CTRL		0x0434	/* mmVIDEO_CTRL */
#define W100_REG_GRAPHIC_KEY		0x0438	/* mmGRAPHIC_KEY */
#define W100_REG_VIDEO_Y_OFFSET	0x043c	/* mmVIDEO_Y_OFFSET */
#define W100_REG_VIDEO_Y_PITCH		0x0440	/* mmVIDEO_Y_PITCH */
#define W100_REG_VIDEO_U_OFFSET	0x0444	/* mmVIDEO_U_OFFSET */
#define W100_REG_VIDEO_U_PITCH		0x0448	/* mmVIDEO_U_PITCH */
#define W100_REG_VIDEO_V_OFFSET	0x044c	/* mmVIDEO_V_OFFSET */
#define W100_REG_VIDEO_V_PITCH		0x0450	/* mmVIDEO_V_PITCH */
#define W100_REG_VIDEO_H_POS		0x0454	/* mmVIDEO_H_POS */
#define W100_REG_VIDEO_V_POS		0x0458	/* mmVIDEO_V_POS */
/* Block GFX, the chip's 2D drawing engine ("E2"): a ROP3 blitter that
 * reads and writes the chip's own memory only. Offsets from the
 * recovered w100fb.h:121-146,195, plus the one register that file never
 * declares at all -- mmRBBM_SOFT_RESET, needed only by the engine-reset
 * recovery path -- taken from kdrive's own w100_regs.h:286
 * (reconstructed from w100.patch, md5 0477bc6766ed74206d3a1ece1f1d7a16).
 * Unlike the video front-end above, this block was driven in production
 * by *both* historical drivers, so every offset here is doubly
 * confirmed.
 */
#define W100_REG_RBBM_STATUS		0x0140	/* mmRBBM_STATUS */
#define W100_REG_RBBM_SOFT_RESET	0x0148	/* mmRBBM_SOFT_RESET */
#define W100_REG_DST_OFFSET		0x1004	/* mmDST_OFFSET */
#define W100_REG_DST_PITCH		0x1008	/* mmDST_PITCH */
#define W100_REG_SRC_Y_X		0x1034	/* mmSRC_Y_X */
#define W100_REG_DST_Y_X		0x1038	/* mmDST_Y_X */
#define W100_REG_DP_GUI_MASTER_CNTL	0x106c	/* mmDP_GUI_MASTER_CNTL */
#define W100_REG_DP_BRUSH_FRGD_CLR	0x107c	/* mmDP_BRUSH_FRGD_CLR */
#define W100_REG_DEFAULT_SC_BOTTOM_RIGHT 0x10a8	/* mmDEFAULT_SC_BOTTOM_RIGHT */
#define W100_REG_DST_WIDTH_HEIGHT	0x1198	/* mmDST_WIDTH_HEIGHT */
#define W100_REG_SRC_OFFSET		0x11ac	/* mmSRC_OFFSET */
#define W100_REG_SRC_PITCH		0x11b0	/* mmSRC_PITCH */
#define W100_REG_SC_TOP_LEFT		0x11bc	/* mmSC_TOP_LEFT */
#define W100_REG_SC_BOTTOM_RIGHT	0x11c0	/* mmSC_BOTTOM_RIGHT */
#define W100_REG_SRC_SC_BOTTOM_RIGHT	0x11c4	/* mmSRC_SC_BOTTOM_RIGHT */
#define W100_REG_DP_CNTL		0x11c8	/* mmDP_CNTL */
#define W100_REG_DP_DATATYPE		0x12c4	/* mmDP_DATATYPE */
#define W100_REG_DP_MIX		0x12c8	/* mmDP_MIX */
#define W100_REG_DP_WRITE_MSK		0x12cc	/* mmDP_WRITE_MSK */

/*
 * ------------------------------------------------------------------
 * The command processor ("CP", whose microcode ATI calls the CCE), which
 * fetches CCE packets from a ring buffer in the chip's own memory and
 * executes them against the GFX block above.
 *
 * Unlike every other register block in this file, there is exactly *one*
 * source for these offsets: kdrive's w100_regs.h:34-65 ("Block CP"),
 * reconstructed from w100.patch (md5 0477bc6766ed74206d3a1ece1f1d7a16).
 * The recovered w100fb.c/.h never used the command processor and its
 * header declares no mmCP_* register at all, so nothing here is doubly
 * confirmed the way the GFX block above is. mmWRAP_BUF_A is the
 * exception -- it is a CIF register (w100_regs.h:7), in the same block
 * as the mmWRAP_START_DIR/mmWRAP_TOP_DIR pair w100_chip_bringup()
 * already programs, and it is listed here because the ring is its only
 * user.
 */
#define W100_REG_WRAP_BUF_A		0x0008	/* mmWRAP_BUF_A */
#define W100_REG_CP_RB_CNTL		0x0210	/* mmCP_RB_CNTL */
#define W100_REG_CP_RB_BASE		0x0214	/* mmCP_RB_BASE */
#define W100_REG_CP_RB_RPTR_ADDR	0x0218	/* mmCP_RB_RPTR_ADDR */
#define W100_REG_CP_RB_RPTR		0x021c	/* mmCP_RB_RPTR */
#define W100_REG_CP_RB_WPTR		0x0220	/* mmCP_RB_WPTR */
#define W100_REG_CP_CSQ_CNTL		0x0230	/* mmCP_CSQ_CNTL */
#define W100_REG_CP_ME_RAM_ADDR	0x0244	/* mmCP_ME_RAM_ADDR */
#define W100_REG_CP_ME_RAM_DATAH	0x024c	/* mmCP_ME_RAM_DATAH */
#define W100_REG_CP_ME_RAM_DATAL	0x0250	/* mmCP_ME_RAM_DATAL */
#define W100_REG_CP_CSQ_STAT		0x02ec	/* mmCP_CSQ_STAT */

#define W100_CHIP_ID_W3220		0x57441002	/* CHIP_ID_W3220 */

/* defCIF_IO -- fixed drive-strength/timing constant, every known board
 * uses this value unmodified.
 */
#define W100_DEF_CIF_IO		0x000c0902
/* defINTF_CNTL, already carries ad_inc_a/ad_inc_b set and
 * rd_data_rdy_a/b clear -- the recovered driver's own field writes on
 * top of this default are a no-op, see w100_cif_bringup()'s comment.
 */
#define W100_DEF_INTF_CNTL		0x11
/* defCPU_DEFAULTS (0x06, access_ind_addr_a|access_ind_addr_b) with
 * access_scratch_reg (bit 3) also set, matching the recovered driver's
 * field writes on top of the default -- see w100_cif_bringup().
 */
#define W100_DEF_CPU_DEFAULTS		0x0e

/*
 * WRAP_BUF_BASE_VALUE/WRAP_BUF_TOP_VALUE: the chip-offset bounds of the
 * CIF wrap aperture, i.e. the host-side window whose writes the chip's
 * wrap unit redirects elsewhere in its own memory. Both are programmed
 * >> 1 to match wrap_start_dir_t/wrap_top_dir_t's own 23-bit
 * half-word-address field.
 *
 * W100_WRAP_BASE is also the "wrap" device-tree window's own chip
 * offset, in the same way W100_FB_BASE is the "fb" window's: kdrive
 * computes the host address of the command ring as
 * mem_base + (mmWRAP_START_DIR.start_addr << 1) (ATIDMAInit(),
 * ati_dma.c:198-200), i.e. the base of exactly this region.
 */
#define W100_WRAP_BASE			0x00080000
#define W100_WRAP_TOP			0x000bffff
#define W100_WRAP_START_DIR_VAL	(W100_WRAP_BASE >> 1)
#define W100_WRAP_TOP_DIR_VAL		(W100_WRAP_TOP >> 1)

/* MEM_INT_BASE_VALUE/MEM_INT_SIZE -- the chip's small always-present
 * internal SRAM, mapped to its own fixed default location whenever
 * "external" (see W100_FB_BASE's own comment) memory is active. Nothing
 * on the host side ever reads/writes this region directly.
 */
#define W100_MEM_INT_BASE_VALUE	0x100000
#define W100_MEM_INT_SIZE		0x05ffff

/* MEM_EXT_BASE_VALUE -- chip-internal address where this driver's
 * framebuffer lives once the mode's own w100_mem_info is mapped in by
 * w100_setup_memory(). Despite the "ext"/"external memory" naming
 * throughout this register block (inherited unchanged from the vendor's
 * own w100fb.h), hx4700's own w100_mem_info.size (2 MiB, see
 * hx4700_w100_mem below) is the chip's own on-package SDRAM, not a
 * separate off-chip DIMM the PXA static-memory bus needs to know
 * about -- "external" here means "external to the chip's tiny always-on
 * SRAM", not "external to the W3220 package". This is also the fixed
 * chip-internal offset this driver's own "fb" ioremap window (see
 * w100_probe()) is anchored to: PXA_CS2_PHYS + W100_FB_BASE is the real
 * host physical address of pixel data.
 */
#define W100_FB_BASE			0x00800000

/*
 * VRAM layout inside the "fb" window, which the device tree maps over the
 * whole 2 MiB the chip's memory controller places at W100_FB_BASE. The
 * scanout buffer sits at offset 0, sized for the largest mode, 480x640
 * RGB565. The video overlay's source planes are copied into a fixed
 * reservation immediately above it, because mmVIDEO_Y_OFFSET is a
 * chip-internal address and the chip has no bus-master path to system
 * memory.
 *
 * The reservation is sized for a full-screen source in either supported
 * format: 480x640 YUV420 is 480*640*3/2 bytes and 480x640 YUV422 is
 * 480*640*2, so the latter is the budget. The command processor's ring
 * buffer sits immediately above the overlay reservation.
 */
#define W100_FB_SIZE			(480 * 640 * 2)
#define W100_OVERLAY_VRAM_OFFSET	W100_FB_SIZE
#define W100_OVERLAY_VRAM_SIZE		(480 * 640 * 2)
#define W100_CP_RING_VRAM_OFFSET	(W100_OVERLAY_VRAM_OFFSET + W100_OVERLAY_VRAM_SIZE)

/*
 * The overlay scaler is upscale-only and integer-only: video_hor_exp and
 * video_ver_exp are 2-bit fields giving a factor of exp + 1, i.e. exactly
 * 1x/2x/3x/4x, confirmed by libw100's own arithmetic
 * (tw = ovrect->w * (props->video_hor_exp + 1), lib/overlay.c:81-82).
 * drm_rect_calc_hscale() computes src/dst in 16.16 fixed point, so a
 * *larger* value means downscaling and the 4x upscale limit is therefore
 * the *minimum* scale drm_atomic_helper_check_plane_state() may accept.
 */
#define W100_OVERLAY_MAX_EXP		4
#define W100_OVERLAY_MIN_SCALE		(DRM_PLANE_NO_SCALING / W100_OVERLAY_MAX_EXP)

#define W100_TESTCLK_SRC_PLL		0x1	/* TESTCLK_SRC_PLL */

/* clk_pin_cntl_t */
#define W100_CLK_PIN_CNTL_OSC_EN	BIT(0)
#define W100_CLK_PIN_CNTL_OSC_GAIN	GENMASK(5, 1)

/* pll_ref_fb_div_t */
#define W100_PLL_REF_FB_DIV_REF_DIV	GENMASK(3, 0)
#define W100_PLL_REF_FB_DIV_FB_INT	GENMASK(13, 8)
#define W100_PLL_REF_FB_DIV_FB_FRAC	GENMASK(18, 16)
#define W100_PLL_REF_FB_DIV_RESET_TIME	GENMASK(23, 20)
#define W100_PLL_REF_FB_DIV_LOCK_TIME	GENMASK(31, 24)

/* pll_cntl_t */
#define W100_PLL_CNTL_PWDN		BIT(0)
#define W100_PLL_CNTL_RESET		BIT(1)
#define W100_PLL_CNTL_TCPOFF		BIT(6)
#define W100_PLL_CNTL_PCP		GENMASK(9, 7)
#define W100_PLL_CNTL_PVG		GENMASK(12, 10)
#define W100_PLL_CNTL_VCOFR		BIT(13)
#define W100_PLL_CNTL_IOFFSET		GENMASK(15, 14)
#define W100_PLL_CNTL_DACTAL		GENMASK(23, 20)
#define W100_PLL_CNTL_CP_CLIP		GENMASK(25, 24)
#define W100_PLL_CNTL_CONF		GENMASK(28, 26)
#define W100_PLL_CNTL_MBCTRL		GENMASK(30, 29)

/* sclk_cntl_t -- only the fields this driver ever sets. */
#define W100_SCLK_CNTL_SRC_SEL		GENMASK(1, 0)
#define W100_SCLK_CNTL_POST_DIV_FAST	GENMASK(7, 4)
#define W100_SCLK_CNTL_CLKON_HYS	GENMASK(10, 8)
#define W100_SCLK_CNTL_POST_DIV_SLOW	GENMASK(14, 11)

/* pclk_cntl_t -- pclk_force_disp (bit 16) is never set by this driver,
 * so the whole register reduces to these two fields at every call site.
 */
#define W100_PCLK_CNTL_SRC_SEL		GENMASK(1, 0)
#define W100_PCLK_CNTL_POST_DIV	GENMASK(7, 4)

/* clk_test_cntl_t */
#define W100_CLK_TEST_CNTL_TESTCLK_SEL		GENMASK(3, 0)
#define W100_CLK_TEST_CNTL_START_CHECK_FREQ	BIT(7)
#define W100_CLK_TEST_CNTL_TSTCOUNT_RST	BIT(8)
#define W100_CLK_TEST_CNTL_TEST_COUNT		GENMASK(31, 24)

/* pwrmgt_cntl_t -- fields this driver sets once in w100_pwm_setup() plus
 * the one field w100_pll_set_clk() mutates afterwards (MODE_REQ).
 */
#define W100_PWRMGT_CNTL_MODE_REQ		GENMASK(3, 2)
#define W100_PWRMGT_CNTL_FAST_NOML_COND	GENMASK(11, 8)
#define W100_PWRMGT_CNTL_NOML_FAST_COND	GENMASK(15, 12)
#define W100_PWRMGT_CNTL_IDLE_TIMER		GENMASK(23, 16)
#define W100_PWRMGT_CNTL_BUSY_TIMER		GENMASK(31, 24)

/* cif_cntl_t -- only the fields w100_cif_bringup() touches. */
#define W100_CIF_CNTL_INTB_OE				BIT(10)
#define W100_CIF_CNTL_EN_WAIT_TO_COMPENSATE_DQ_PROP_DLY BIT(11)
#define W100_CIF_CNTL_INTERRUPT_ACTIVE_HIGH		BIT(20)
#define W100_CIF_CNTL_DIS_SYSTEM_BITS			BIT(26)
#define W100_CIF_CNTL_DIS_MR				BIT(27)

/* cif_read_dbg_t */
#define W100_CIF_READ_DBG_DIS_RD_SAME_BYTE_TO_TRIG_FETCH BIT(17)

/* cif_write_dbg_t */
#define W100_CIF_WRITE_DBG_DIS_TIMEOUT_DURING_RBBM		BIT(7)
#define W100_CIF_WRITE_DBG_DIS_PACKER_FUL_DURING_RBBM_TIMEOUT	BIT(8)
#define W100_CIF_WRITE_DBG_EN_DWORD_SPLIT_TO_RBBM		BIT(9)

/* graphic_ctrl_t_w32xx -- w3200/w3220-only layout (10-bit
 * total_req_graphic, vs. w100's own 9-bit field this driver never uses).
 */
#define W100_GRAPHIC_CTRL_COLOR_DEPTH		GENMASK(2, 0)
#define W100_GRAPHIC_CTRL_PORTRAIT_MODE	GENMASK(4, 3)
#define W100_GRAPHIC_CTRL_REQ_FREQ		GENMASK(9, 6)
#define W100_GRAPHIC_CTRL_EN_CRTC		BIT(10)
#define W100_GRAPHIC_CTRL_EN_GRAPHIC_REQ	BIT(11)
#define W100_GRAPHIC_CTRL_EN_GRAPHIC_CRTC	BIT(12)
#define W100_GRAPHIC_CTRL_TOTAL_REQ_GRAPHIC	GENMASK(22, 13)
#define W100_GRAPHIC_CTRL_LCD_PCLK_ON		BIT(23)
#define W100_GRAPHIC_CTRL_LCD_SCLK_ON		BIT(24)
#define W100_GRAPHIC_CTRL_PCLK_RUNNING		BIT(25)	/* read-back: PCLK has stopped when clear */
#define W100_GRAPHIC_CTRL_SCLK_RUNNING		BIT(26)	/* read-back: SCLK has stopped when clear */

/* video_ctrl_t, transcribed from libw100's lib/w100regs.h:1465-1485 (LSB
 * first; field-identical to the recovered w100fb.h:533-553). Only the
 * fields this driver ever sets are named. Every other bit is written as 0,
 * matching the zero-initialised shadow word libw100's own
 * ac_overlay_setup() builds: uv_combine[9] (planar U/V, not interleaved),
 * video_ch_sel[19] (the RGB source path, not implemented -- see the
 * overlay section), video_portrait[21:20] (no rotation here),
 * video_inv_hor[24] (see w100_overlay_plane_init()), gamma_sel[27:26],
 * dis_limit[28], en_uv_hblend[29] and rgb_gamma_sel[31:30].
 */
#define W100_VIDEO_CTRL_VIDEO_MODE		BIT(0)
#define W100_VIDEO_CTRL_KEYER_EN		BIT(1)
#define W100_VIDEO_CTRL_EN_VIDEO_REQ		BIT(2)
#define W100_VIDEO_CTRL_EN_GRAPHIC_REQ_VIDEO	BIT(3)
#define W100_VIDEO_CTRL_EN_VIDEO_CRTC		BIT(4)
#define W100_VIDEO_CTRL_HOR_EXP		GENMASK(6, 5)
#define W100_VIDEO_CTRL_VER_EXP		GENMASK(8, 7)
#define W100_VIDEO_CTRL_TOTAL_REQ_VIDEO	GENMASK(18, 10)
#define W100_VIDEO_CTRL_YUV2RGB_EN		BIT(22)
#define W100_VIDEO_CTRL_YUV2RGB_OPTION		BIT(23)
#define W100_VIDEO_CTRL_VIDEO_INV_VER		BIT(25)

/* graphic_key_t (lib/w100regs.h:1492-1495) -- one 16-bit key value plus one
 * 16-bit compare mask, matching libw100's own ac_ovlkey_t
 * { uint16_t color; uint16_t mask; } (lib/actypes.h:104-107) and its
 * assignment in ac_overlay_setup() (lib/overlay.c:256-263).
 */
#define W100_GRAPHIC_KEY_COLOR			GENMASK(15, 0)
#define W100_GRAPHIC_KEY_MASK			GENMASK(31, 16)

/* video_{y,u,v}_offset_t (24-bit chip-internal byte address) and
 * video_{y,u,v}_pitch_t (11-bit byte pitch), lib/w100regs.h:1502-1560. All
 * six registers share the same two shapes.
 */
#define W100_VIDEO_OFFSET			GENMASK(23, 0)
#define W100_VIDEO_PITCH			GENMASK(10, 0)

/* active_h_disp_t/active_v_disp_t/graphic_h_disp_t/graphic_v_disp_t/
 * crtc_total_t all share this 10-bit-start/6-reserved/10-bit-end/
 * 6-reserved shape, and so do video_h_pos_t/video_v_pos_t
 * (lib/w100regs.h:1562-1584).
 */
#define W100_DISP_RANGE(start, end) ((u32)(start) | ((u32)(end) << 16))

/* mc_fb_location_t/mc_ext_mem_location_t -- plain 16+16 start/top split,
 * both already pre-shifted right by 8 by their callers (matching the
 * recovered driver's own ">> 8" at each call site).
 */
#define W100_MC_LOCATION(start, top) ((u32)(start) | ((u32)(top) << 16))

/* disp_db_buf_cntl_wr_t db_buf_cntl field, both real values the
 * recovered driver ever writes -- see w100_update_disable/enable().
 */
#define W100_DISP_DB_BUF_CNTL_DISABLE	0x78	/* db_buf_cntl=0x1e, rest 0 */
#define W100_DISP_DB_BUF_CNTL_ENABLE	0x7b	/* + update_db_buf|en_db_buf */

/* rbbm_status_t, w100fb.h:818-845. Only these two fields are polled;
 * the rest are per-block busy bits nothing here consumes (and several
 * of them, rb3d/se/re/tam/tdm/pb, have no register block on this chip
 * at all).
 */
#define W100_RBBM_STATUS_CMDFIFO_AVAIL	GENMASK(6, 0)
#define W100_RBBM_STATUS_GUI_ACTIVE	BIT(31)

/* rbbm_soft_reset_t, kdrive w100_regs.h:3582-3593. Only the 2D engine's
 * own reset bit is ever pulsed here.
 */
#define W100_RBBM_SOFT_RESET_E2	BIT(5)

/* sclk_cntl_t clock-force bits, w100fb.h:702-726. kdrive's engine-reset
 * path forces exactly these three on across the reset pulse.
 */
#define W100_SCLK_CNTL_FORCE_E2	BIT(21)
#define W100_SCLK_CNTL_FORCE_E3	BIT(22)
#define W100_SCLK_CNTL_FORCE_IDCT	BIT(23)

/* dp_gui_master_cntl_t, w100fb.h:795-816 -- the 2D operation
 * descriptor. Writing it is what selects fill vs. copy vs. ROP.
 */
#define W100_GMC_SRC_PITCH_OFFSET_CNTL	BIT(0)
#define W100_GMC_DST_PITCH_OFFSET_CNTL	BIT(1)
#define W100_GMC_SRC_CLIPPING		BIT(2)
#define W100_GMC_DST_CLIPPING		BIT(3)
#define W100_GMC_BRUSH_DATATYPE	GENMASK(7, 4)
#define W100_GMC_DST_DATATYPE		GENMASK(11, 8)
#define W100_GMC_SRC_DATATYPE		GENMASK(14, 12)
#define W100_GMC_BYTE_PIX_ORDER	BIT(15)
#define W100_GMC_DEFAULT_SEL		BIT(16)
#define W100_GMC_ROP3			GENMASK(24, 17)
#define W100_GMC_DP_SRC_SOURCE		GENMASK(27, 25)
#define W100_GMC_CLR_CMP_FCN_DIS	BIT(28)
#define W100_GMC_WR_MSK_DIS		BIT(30)
#define W100_GMC_DP_OP			BIT(31)

/* dp_cntl_t, w100fb.h:896-909. The two dst_*_dir bits carry
 * overlapping-copy correctness; the src_*_dir/major_x bits are only
 * written as part of the engine's quiescent state.
 */
#define W100_DP_CNTL_DST_X_DIR		BIT(0)
#define W100_DP_CNTL_DST_Y_DIR		BIT(1)
#define W100_DP_CNTL_SRC_X_DIR		BIT(2)
#define W100_DP_CNTL_SRC_Y_DIR		BIT(3)
#define W100_DP_CNTL_DST_MAJOR_X	BIT(4)
#define W100_DP_CNTL_SRC_MAJOR_X	BIT(5)

/* dp_datatype_t, w100fb.h:847-862 */
#define W100_DP_DATATYPE_DST		GENMASK(3, 0)
#define W100_DP_DATATYPE_BRUSH		GENMASK(11, 8)
#define W100_DP_DATATYPE_SRC2_TYPE	BIT(12)
#define W100_DP_DATATYPE_SRC2		GENMASK(15, 13)
#define W100_DP_DATATYPE_SRC		GENMASK(18, 16)
#define W100_DP_DATATYPE_BYTE_PIX_ORDER BIT(30)

/* dp_mix_t, w100fb.h:864-877 */
#define W100_DP_MIX_SRC_SOURCE		GENMASK(10, 8)
#define W100_DP_MIX_SRC2_SOURCE	GENMASK(13, 11)
#define W100_DP_MIX_ROP3		GENMASK(23, 16)
#define W100_DP_MIX_OP			BIT(24)

/* dst_pitch_t/src_pitch_t (kdrive w100_regs.h:2088-2092,2422-2426): a
 * 14-bit pitch plus a 2-bit multiplier selector both historical drivers
 * leave at zero. dst_offset_t/src_offset_t (:2078,:2412) are plain
 * 24-bit chip-internal byte addresses.
 */
#define W100_2D_PITCH			GENMASK(13, 0)
#define W100_2D_OFFSET			GENMASK(23, 0)

/* dst_y_x_t/src_y_x_t/sc_top_left_t/sc_bottom_right_t all share one
 * 14-bit-X/2-reserved/14-bit-Y/2-reserved shape (kdrive w100_regs.h:
 * 2142-2147, 2476-2481, 2918-2923 and default_sc_bottom_right_t at
 * :2799-2804), matching w100fb.c:376's own (y << 16) | x.
 */
#define W100_2D_X			GENMASK(13, 0)
#define W100_2D_Y			GENMASK(29, 16)
#define W100_2D_YX(x, y) \
	(FIELD_PREP(W100_2D_X, (x)) | FIELD_PREP(W100_2D_Y, (y)))

/* dst_width_height_t, kdrive w100_regs.h:2175-2181: a 14-bit height at
 * [13:0] and a 14-bit width at [29:16] (declared there as an 8-bit b0
 * and a 6-bit b1 half, contiguous). This is mmDST_WIDTH_HEIGHT (0x1198)
 * and it is *not* mmDST_HEIGHT_WIDTH (0x103C), whose own struct
 * (:2188-2194) transposes the two fields. w100fb.c:377-378 drives the
 * former and kdrive's non-packet3 path drives the latter; both are
 * correct against their own register. This driver uses w100fb.c's,
 * because every other offset in the GFX block here is transcribed from
 * that same file.
 */
#define W100_2D_HEIGHT			GENMASK(13, 0)
#define W100_2D_WIDTH			GENMASK(29, 16)
#define W100_2D_WH(w, h) \
	(FIELD_PREP(W100_2D_HEIGHT, (h)) | FIELD_PREP(W100_2D_WIDTH, (w)))

/* Largest coordinate any of the 14-bit geometry fields can carry;
 * W100_MAXINT in kdrive's own w100_const.h.
 */
#define W100_2D_MAXINT			0x1fff

/* w100_const.h's own BRUSH_SOLIDCOLOR/BRUSH_NONE (kdrive) == w100fb.h:
 * 788-789's GMC_BRUSH_SOLID_COLOR/GMC_BRUSH_NONE, DP_SRC_MEM_RECTANGULAR
 * == SOURCE_MEM_RECTANGULAR, DP_OP_ROP == OP_ROP. Both sources agree on
 * every value.
 */
#define W100_BRUSH_SOLIDCOLOR		13
#define W100_BRUSH_NONE		15
#define W100_DP_SRC_MEM_RECTANGULAR	2
#define W100_DP_OP_ROP			0

/*
 * The 16bpp datatype. Both historical drivers use the value 3 and call
 * it different things: kdrive's w100_const.h names it
 * DATATYPE_ARGB1555, while w100fb.c:328 assigns a bare 3 to
 * gmc_dst_datatype with its own trailing comment naming it
 * DstType_16Bpp_444. Neither name matches this driver's RGB565 scanout
 * (W100_GRAPHIC_CTRL_COLOR_DEPTH == 6 plus the mmDISP_DEBUG2 565 bit,
 * see w100_set_dispregs()),
 * and that is fine: a ROP3 is a bitwise operation on raw pixel bits and
 * a solid fill's colour goes into mmDP_BRUSH_FRGD_CLR raw, so the
 * datatype selects *pixel size*, not channel layout. w100fb.c drove a
 * 565 panel with datatype 3 in production for years. Fill colours
 * therefore have to be supplied already in RGB565 layout.
 *
 * w100fb.c's own engine init writes SRC_DATATYPE_EQU_DST (w100fb.h:783)
 * into gmc_src_datatype rather than the datatype itself -- that
 * constant is also 3, so the two sources emit an identical register
 * value here and there is nothing to choose between them.
 */
#define W100_DATATYPE_16BPP		3

/*
 * The 16 raster operations the engine's ROP3 unit can perform, in X11's
 * own GXclear..GXset order -- that order is not incidental, it is the
 * index both of kdrive's tables below are written against.
 */
enum w100_rop {
	W100_ROP_CLEAR,		/* 0 */
	W100_ROP_AND,		/* src AND dst */
	W100_ROP_AND_REVERSE,	/* src AND NOT dst */
	W100_ROP_COPY,		/* src */
	W100_ROP_AND_INVERTED,	/* NOT src AND dst */
	W100_ROP_NOOP,		/* dst */
	W100_ROP_XOR,		/* src XOR dst */
	W100_ROP_OR,		/* src OR dst */
	W100_ROP_NOR,		/* NOT src AND NOT dst */
	W100_ROP_EQUIV,		/* NOT src XOR dst */
	W100_ROP_INVERT,	/* NOT dst */
	W100_ROP_OR_REVERSE,	/* src OR NOT dst */
	W100_ROP_COPY_INVERTED,	/* NOT src */
	W100_ROP_OR_INVERTED,	/* NOT src OR dst */
	W100_ROP_NAND,		/* NOT src OR NOT dst */
	W100_ROP_SET,		/* 1 */
	W100_ROP_COUNT
};

/*
 * A 2D operand: a rectangle of RGB565 pixels somewhere in the chip's own
 * memory. "offset" is a chip-internal byte address (the same address
 * space W100_FB_BASE names), not a host address and not an offset into
 * the ioremapped window -- the GFX engine can only address chip memory,
 * and has no bus-master path to system RAM at all
 * (ATIGetPixmapOffsetPitch()'s own "XXX this only works for surfaces
 * allocated in external memory", ati_draw.c:201).
 */
struct w100_2d_surface {
	u32 offset;
	u32 pitch;	/* bytes per line */
	u16 width;	/* pixels */
	u16 height;	/* lines */
};

/*
 * ------------------------------------------------------------------
 * Command-processor constants. Bitfields from kdrive's w100_regs.h
 * structs, packet encodings from its ati_dma.h, magic values from its
 * w100_const.h. As with the mmCP_* offsets above, kdrive is the only
 * source for any of this.
 */

/* wrap_buf_a_t: the chip-internal address the wrap unit redirects this
 * aperture's writes to, plus a size code for the redirected block.
 */
#define W100_WRAP_BUF_A_OFFSET_ADDR	GENMASK(23, 0)
#define W100_WRAP_BUF_A_BLOCK_SIZE	GENMASK(26, 24)

/* cp_rb_cntl_t. rb_no_update and rb_bufsz are the only two fields
 * ATIDMAInit() sets; rb_rptr_wr_ena stays clear, which is what makes
 * mmCP_RB_RPTR read-only-to-the-host and owned by the processor.
 */
#define W100_CP_RB_CNTL_BUFSZ		GENMASK(5, 0)
#define W100_CP_RB_CNTL_BLKSZ		GENMASK(13, 8)
#define W100_CP_RB_CNTL_NO_UPDATE	BIT(27)
#define W100_CP_RB_CNTL_RPTR_WR_ENA	BIT(31)

/* cp_csq_cntl_t. CSQ_CNTL_MODE_FREERUN (w100_const.h:1) is the mode
 * ATIDMAInit() runs in: the processor fetches from the ring on its own
 * rather than from a host-written command queue aperture.
 */
#define W100_CP_CSQ_CNTL_MODE		GENMASK(31, 28)
#define W100_CP_CSQ_MODE_FREERUN	8

/* cp_rb_rptr_t/cp_rb_wptr_t are both a bare 23-bit dword index, which is
 * how ATIRequestEntries()/ATISubmitEntries() use them. The driver masks
 * reads with W100_CP_RING_MASK rather than this, because its ring is far
 * smaller than the field is wide and the ring's own size is the bound
 * that matters; the field width is recorded for the register map's sake.
 */
#define W100_CP_RB_PTR			GENMASK(22, 0)

/* cp_me_ram_addr_t's address field is 8 bits, so the microcode engine's
 * RAM is 256 entries deep -- the bound the firmware size is checked
 * against.
 */
#define W100_CP_ME_RAM_DEPTH		256

/*
 * Ring geometry. rbsize 10 is ATIDMAInit()'s own choice (ati_dma.c:182,
 * "with rbsize = 10, DMA buffer will be of 0x2000 (8192) bytes"), and it
 * is the size mmCP_RB_CNTL.rb_bufsz encodes directly: the ring holds
 * 1 << (rbsize + 1) dwords. w100_const.h's WB_BLOCK_SIZE_A_0..4 map
 * rbsize 9..13 onto wrap_buf_a_t.block_size_a 0..4, i.e. the code is
 * rbsize - 9.
 */
#define W100_CP_RB_BUFSZ		10
#define W100_CP_RING_DWORDS		(1u << (W100_CP_RB_BUFSZ + 1))
#define W100_CP_RING_MASK		(W100_CP_RING_DWORDS - 1)
#define W100_CP_RING_BYTES		(W100_CP_RING_DWORDS * 4)
#define W100_CP_WRAP_BLOCK_SIZE	(W100_CP_RB_BUFSZ - 9)

/*
 * CCE packet encodings, ati_dma.h:31-58.
 *
 * A type-0 packet is a register write burst: a header naming the first
 * register (as a dword index, hence >> 2) and a dword count, then that
 * many values for consecutive registers. A type-3 packet is an opcode
 * the microcode implements; the two used here are the "multi" fill and
 * blit opcodes from w100_const.h:23-24, whose opcode bytes are already
 * folded into the constants.
 *
 * A type-2 packet is a no-op with no payload. It is named here for the
 * taxonomy's sake and never emitted: the usual reason to
 * emit one is to pad out the tail of a ring so a packet does not
 * straddle the wrap point, and w100_cp_ring_emit() makes packets wrap
 * cleanly instead.
 *
 * Both macros take the *payload* dword count, matching kdrive's own
 * DMA_PACKET0()/DMA_PACKET3(); the encoded field is count - 1.
 */
#define W100_CCE_PACKET0		0x00000000
#define W100_CCE_PACKET2		0x80000000
#define W100_CCE_PACKET3_PAINT_MULTI	0xc0001a00
#define W100_CCE_PACKET3_BITBLT_MULTI	0xc0001b00

#define W100_CCE_PACKET0_HDR(reg, count) \
	(W100_CCE_PACKET0 | (((count) - 1) << 16) | ((reg) >> 2))
#define W100_CCE_PACKET3_HDR(op, count) \
	((op) | (((count) - 1) << 16))

/*
 * The firmware file. Named for the part rather than dropped into the
 * existing radeon/ directory, because it is a different chip from a
 * different product line.
 */
#define W100_CP_FW_NAME		"imageon/W3220_cp.bin"

/*
 * ------------------------------------------------------------------
 * Board data, transcribed from the recovered arch/arm/mach-pxa/
 * hx4700.c:416-529 (w3220_regs, w3220_modes[], w3220_mem_info,
 * w3220_bm_mem_info, w3220_gpio_info, w3220_info.xtal_freq).
 */

struct w100_gen_regs {
	u32 lcd_format;
	u32 lcdd_cntl1;
	u32 lcdd_cntl2;
	u32 genlcd_cntl1;
	u32 genlcd_cntl2;
	u32 genlcd_cntl3;
};

/*
 * Fields dropped relative to the recovered w100_mode struct
 * (include/video/w100fb.h), because both hx4700.c:431-477 hw modes carry
 * an identical value and no other board's data will ever be added here:
 * fast_pll_freq (0, "fast pll mode" is never engaged by this driver),
 * pixclk_divider_rotated (identical to pixclk_divider for both modes --
 * rotation is dropped entirely, see the file comment), sysclk_src
 * (always CLK_SRC_PLL) and sysclk_divider (always 0, see
 * w100_init_clocks()'s own comment).
 */
struct w100_hw_mode {
	unsigned int xres;
	unsigned int yres;
	u16 left_margin;
	u16 right_margin;
	u16 upper_margin;
	u16 lower_margin;
	u32 crtc_ss;
	u32 crtc_ls;
	u32 crtc_gs;
	u32 crtc_vpos_gs;
	u32 crtc_rev;
	u32 crtc_dclk;
	u32 crtc_gclk;
	u32 crtc_goe;
	u32 crtc_ps1_active;
	u8 pll_freq;		/* PLL Fout, MHz -- w100_pll_entry.freq units */
	u8 pixclk_divider;
	const struct drm_display_mode *drm_mode;
};

struct w100_mem_info {
	u32 ext_cntl;
	u32 sdram_mode_reg;
	u32 ext_timing_cntl;
	u32 io_cntl;
	u32 size;
};

struct w100_bm_mem_info {
	u32 ext_mem_bw;
	u32 offset;
	u32 ext_timing_ctl;
	u32 ext_cntl;
	u32 mode_reg;
	u32 io_cntl;
	u32 config;
};

struct w100_gpio_regs {
	u32 init_data1;
	u32 init_data2;
	u32 gpio_dir1;
	u32 gpio_oe1;
	u32 gpio_dir2;
	u32 gpio_oe2;
};

struct w100_pll_entry {
	u16 freq;
	u8 m;
	u8 n_int;
	u8 n_fac;
	u8 tfgoal;
	u8 lock_time;
};

/* hx4700.c:422-429's w3220_regs -- the *active* register values (the
 * recovered file also carries commented-out prior values alongside
 * several of these; only the live ones are transcribed here).
 */
static const struct w100_gen_regs hx4700_w100_regs = {
	.lcd_format	= 0x00000003,
	.lcdd_cntl1	= 0x00000000,
	.lcdd_cntl2	= 0x0003ffff,
	.genlcd_cntl1	= 0x00abf003,
	.genlcd_cntl2	= 0x00000003,
	.genlcd_cntl3	= 0x000102aa,
};

/* hx4700.c:479-485's w3220_mem_info. size = 0x1fffff, 2 MiB. */
static const struct w100_mem_info hx4700_w100_mem = {
	.ext_cntl	= 0x09640011,
	.sdram_mode_reg	= 0x00600021,
	.ext_timing_cntl = 0x1a001545,
	.io_cntl	= 0x7ddd7333,
	.size		= 0x1fffff,
};

/* hx4700.c:487-495's w3220_bm_mem_info. */
static const struct w100_bm_mem_info hx4700_w100_bm_mem = {
	.ext_mem_bw	= 0x50413e01,
	.offset		= 0,
	.ext_timing_ctl	= 0x00043f7f,
	.ext_cntl	= 0x00000010,
	.mode_reg	= 0x00250000,
	.io_cntl	= 0x0fff0000,
	.config		= 0x08301480,
};

/* hx4700.c:497-504's w3220_gpio_info -- the W3220's *own* GPIO block
 * (mmGPIO_DATA/CNTL1-4), unrelated to any PXA GPIO. Nothing on hx4700
 * hangs off this block (no consumer of w100fb_gpio_read/write() ever
 * existed for this board); it is programmed once during bring-up purely
 * because the board data table says so.
 */
static const struct w100_gpio_regs hx4700_w100_gpio = {
	.init_data1	= 0xdfe00100,
	.gpio_dir1	= 0xffff0000,
	.gpio_oe1	= 0x00000000,
	.init_data2	= 0x00000000,
	.gpio_dir2	= 0x00000000,
	.gpio_oe2	= 0x00000000,
};

/* xtal_16000000[] from w100fb.c:1009-1016 -- hx4700's own w3220_info.
 * xtal_freq is 16000000 (hx4700.c:514), so the recovered driver's other
 * two crystal tables (12.5 MHz, 14.318 MHz -- other boards this driver
 * doesn't support) are dropped entirely.
 */
static const struct w100_pll_entry hx4700_w100_pll_table[] = {
	/* freq   m  n_int  n_fac  tfgoal  lock_time */
	{  72,    1,   8,     0,    0xe0,      48 },
	{  80,    1,   9,     0,    0xe0,      13 },
	{  95,    1,  10,     7,    0xe0,      38 },
	{  96,    1,  11,     0,    0xe0,      36 },
};

/*
 * DRM-visible mode geometry, derived from the same hx4700.c:431-477
 * w3220_modes[] margins used for the real CRTC_TOTAL/ACTIVE_*_DISP
 * register programming below. hsync/vsync pulse width is *not* part of
 * the recovered board data (the real sync pulse geometry is baked into
 * the raw crtc_ss/crtc_ls/crtc_gs register values written verbatim by
 * w100_init_lcd(), not derived from any DRM timing field), so
 * hsync_end/vsync_end here are a minimal one-unit placeholder purely so
 * userspace mode listings have a non-degenerate value -- these two
 * fields drive no register write anywhere in this file.
 *
 * htotal/vtotal and .clock are not cosmetic: they set the frame period
 * of the vblank timer that paces page-flip events. The totals are the
 * values w100_init_lcd() writes to mmCRTC_TOTAL. The clock is pll_freq
 * divided by pixclk_divider + 1, because PCLK_CNTL.pclk_post_div
 * divides by N + 1 (w100fb.c:1242 programs 1 for "P = 2", and its
 * calc_hsync() divides by post_div + 1). That gives about 56.8 Hz for
 * the VGA timing. Whether the CRTC counts h/v_total or total + 1
 * clocks, and whether the QVGA timing's mmCRTC_DCLK value divides the
 * dot clock further, are both undocumented, so the QVGA rate derived
 * here (about 225 Hz) is an upper bound rather than a measurement.
 */
#define W100_MODE_CLOCK_KHZ	(95 * 1000 / (4 + 1))

/*
 * Both modes drive the same 60 x 80 mm portrait glass. The size is a
 * provisional figure (8 x 6 cm as given for the hx4700, to be measured),
 * so a client that divides a mode's resolution by it gets that mode's
 * effective density: the QVGA mode comes out at half the VGA one.
 */
static const struct drm_display_mode hx4700_w100_mode_vga = {
	DRM_MODE("480x640", DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
		 W100_MODE_CLOCK_KHZ, 480, 496, 497, 511, 0,
		 640, 647, 648, 655, 0, 0),
	.width_mm = 60,
	.height_mm = 80,
};

static const struct drm_display_mode hx4700_w100_mode_qvga = {
	DRM_MODE("240x320", DRM_MODE_TYPE_DRIVER,
		 W100_MODE_CLOCK_KHZ, 240, 248, 249, 257, 0,
		 320, 324, 325, 329, 0, 0),
	.width_mm = 60,
	.height_mm = 80,
};

static const struct w100_hw_mode hx4700_w100_hw_modes[] = {
	{
		.xres = 480, .yres = 640,
		.left_margin = 15, .right_margin = 16,
		.upper_margin = 8, .lower_margin = 7,
		.crtc_ss	= 0x00000000,
		.crtc_ls	= 0xa1ff01f9,
		.crtc_gs	= 0xc0000000,
		.crtc_vpos_gs	= 0x0000028f,
		.crtc_rev	= 0,
		.crtc_dclk	= 0x80000000,
		.crtc_gclk	= 0x040a0104,
		.crtc_goe	= 0,
		.crtc_ps1_active = 0x00000000,
		.pll_freq	= 95,
		.pixclk_divider	= 4,
		.drm_mode	= &hx4700_w100_mode_vga,
	},
	{
		.xres = 240, .yres = 320,
		.left_margin = 9, .right_margin = 8,
		.upper_margin = 5, .lower_margin = 4,
		.crtc_ss	= 0x80150014,
		.crtc_ls	= 0xa0fb00f7,
		.crtc_gs	= 0xc0080007,
		.crtc_vpos_gs	= 0x00080007,
		.crtc_rev	= 0x0000000a,
		.crtc_dclk	= 0x81700030,
		.crtc_gclk	= 0x8015010f,
		.crtc_goe	= 0x00000000,
		.crtc_ps1_active = 0x00000000,
		.pll_freq	= 95,
		.pixclk_divider	= 4,
		.drm_mode	= &hx4700_w100_mode_qvga,
	},
};

struct w100_device {
	struct drm_device drm;

	void __iomem *cfg;	/* 16 B indirect CIF config window */
	void __iomem *regs;	/* 8 KiB memory-mapped register window */
	void __iomem *vram;	/* video memory window at chip offset W100_FB_BASE */
	size_t vram_size;	/* ...and how much of it the device tree maps */

	/*
	 * The CIF wrap aperture at chip offset W100_WRAP_BASE, through
	 * which the host writes command-ring dwords. NULL when the device
	 * tree has no "wrap" window, which is a supported configuration:
	 * the driver then never brings the command processor up at all.
	 */
	void __iomem *wrap;

	struct drm_plane primary_plane;
	struct drm_plane overlay_plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;

	/* Driver-private overlay colour-key properties. There is no core
	 * drm_plane_create_colorkey_property(); the shape here follows
	 * drivers/gpu/drm/armada/armada_overlay.c's precedent, cut down to
	 * what mmGRAPHIC_KEY actually is -- see w100_overlay_plane_init().
	 */
	struct drm_property *colorkey_prop;
	struct drm_property *colorkey_mask_prop;
	struct drm_property *colorkey_mode_prop;

	/* W3220_nRESET, PXA GPIO61 -- deasserted once here and never
	 * toggled again, matching the recovered global_gpios[]'s own
	 * GPIOF_OUT_INIT_HIGH with no other gpio_set_value() call on this
	 * line anywhere in hx4700.c.
	 */
	struct gpio_desc *reset_gpio;

	/* Panel power sequence, ported from sony_lcd_init()/sony_lcd_off()
	 * (hx4700.c:361-399), driven from atomic_enable()/atomic_disable().
	 */
	struct gpio_desc *sqn_gpio;		/* GPIO84 LCD_SQN */
	struct gpio_desc *lvdd_gpio;		/* GPIO110 LCD_LVDD_3V3_ON */
	struct gpio_desc *avdd_gpio;		/* GPIO111 LCD_AVDD_3V3_ON */
	struct gpio_desc *slin1_gpio;		/* GPIO70 LCD_SLIN1 */
	struct gpio_desc *panel_reset_gpio;	/* GPIO62 LCD_nRESET */
	struct gpio_desc *pc1_gpio;		/* GPIO59 LCD_PC1 */
	struct gpio_desc *n2v7_gpio;		/* GPIO112 LCD_N2V7_7V3_ON */

	struct backlight_device *backlight;	/* NULL if DT names none */

	/*
	 * Persistent clock-programming state, mirrors the recovered
	 * driver's own module-static struct power_state (w100fb.c:971-982):
	 * w100_pll_adjust()/w100_pll_set_clk() mutate individual fields of
	 * pll_cntl/pll_ref_fb_div/pwrmgt_cntl across several register
	 * writes while relying on the *other* fields staying whatever
	 * w100_pwm_setup() last put there -- a real cross-call dependency
	 * in the original driver, not an artefact of its use of a module
	 * static, so it is preserved here as instance state rather than
	 * flattened away.
	 */
	u32 sclk_cntl;
	u32 pll_cntl;
	u32 pll_ref_fb_div;
	u32 pwrmgt_cntl;

	/*
	 * GFX 2D engine. @accel_lock serialises whole operations: each one
	 * is a burst of register writes that only means anything as a unit,
	 * and the engine has exactly one set of them. The atomic commit
	 * path is already serialised by DRM, but the lock keeps the engine
	 * state consistent against the teardown paths. @accel_ready tracks
	 * whether the engine's quiescent state is currently established:
	 * false until the
	 * first w100_2d_setup(), i.e. until a modeset has brought up the
	 * memory controller and SCLK the engine needs, set by every
	 * w100_2d_setup_locked() and cleared again by every
	 * w100_2d_engine_reset(), which wipes exactly that state. @hw_mode is
	 * the mode that brought it up -- the reset path needs it to rebuild
	 * the engine's quiescent state. @vram_cleared latches the one-shot
	 * power-on clear, see w100_2d_clear_vram().
	 */
	struct mutex accel_lock;
	const struct w100_hw_mode *hw_mode;
	bool accel_ready;
	bool vram_cleared;

	/*
	 * Command processor. @ring_ready is what selects the transport:
	 * false and every 2D operation is a burst of polled register
	 * writes, true and it is a packet in the ring. It only ever
	 * becomes true if the device tree carries a wrap aperture *and*
	 * the microcode file loads, so on a system with neither the two
	 * fields below are simply never touched. All three are covered by
	 * @accel_lock, which already serialises whole 2D operations.
	 *
	 * @ring_wptr is the driver's own copy of the ring write pointer,
	 * in dwords. @ring_free is a cached count of how many dwords the
	 * processor has since consumed, refreshed from mmCP_RB_RPTR only
	 * when a submission would otherwise not fit -- that read crosses
	 * the static-memory bus, so it is worth not doing per
	 * packet. @fw is the microcode, fetched once and re-uploaded on
	 * each mode apply; @fw_warned keeps a missing firmware file to one
	 * log line rather than one per mode apply.
	 */
	u32 ring_wptr;
	u32 ring_free;
	bool ring_ready;
	const struct firmware *fw;
	bool fw_warned;
};

#define to_w100(_dev) container_of(_dev, struct w100_device, drm)

/* ------------------------------------------------------------------ */
/* Chip bring-up: soft reset, CIF indirect-config aperture setup, the
 * W3220's own GPIO block. Runs once at probe, matching w100_hw_init()'s
 * own single call site in the recovered driver (w100fb_probe()).
 */

static void w100_soft_reset(struct w100_device *w100)
{
	u16 val = readw(w100->cfg + W100_CFG_STATUS);

	writew(val | 0x08, w100->cfg + W100_CFG_STATUS);
	usleep_range(100, 200);
	writew(0x00, w100->cfg + W100_CFG_STATUS);
	usleep_range(100, 200);
}

static void w100_chip_bringup(struct w100_device *w100)
{
	u32 val;
	unsigned int i;

	w100_soft_reset(w100);

	/*
	 * "This is what the fpga_init code does on reset. May be wrong but
	 * there is little info available" -- w100fb.c:892-893's own
	 * comment, ported verbatim including the dummy-read delay loop.
	 */
	writel(0x31, w100->regs + W100_REG_SCRATCH_UMSK);
	for (i = 0; i < 10000; i++)
		readl(w100->regs + W100_REG_SCRATCH_UMSK);
	writel(0x30, w100->regs + W100_REG_SCRATCH_UMSK);

	/* mmCIF_IO <- defCIF_IO, unmodified -- w100fb.c:900-901. */
	writel(W100_DEF_CIF_IO, w100->regs + W100_REG_CIF_IO);

	/* mmCIF_WRITE_DBG: clear dis_packer_ful_during_rbbm_timeout, set
	 * en_dword_split_to_rbbm and dis_timeout_during_rbbm -- w100fb.c:
	 * 903-907.
	 */
	val = readl(w100->regs + W100_REG_CIF_WRITE_DBG);
	val &= ~W100_CIF_WRITE_DBG_DIS_PACKER_FUL_DURING_RBBM_TIMEOUT;
	val |= W100_CIF_WRITE_DBG_EN_DWORD_SPLIT_TO_RBBM |
	       W100_CIF_WRITE_DBG_DIS_TIMEOUT_DURING_RBBM;
	writel(val, w100->regs + W100_REG_CIF_WRITE_DBG);

	/* mmCIF_READ_DBG: set dis_rd_same_byte_to_trig_fetch --
	 * w100fb.c:909-911.
	 */
	val = readl(w100->regs + W100_REG_CIF_READ_DBG);
	val |= W100_CIF_READ_DBG_DIS_RD_SAME_BYTE_TO_TRIG_FETCH;
	writel(val, w100->regs + W100_REG_CIF_READ_DBG);

	/* mmCIF_CNTL: clear en_wait_to_compensate_dq_prop_dly, set
	 * dis_system_bits/dis_mr/intb_oe/interrupt_active_high --
	 * w100fb.c:913-919.
	 */
	val = readl(w100->regs + W100_REG_CIF_CNTL);
	val &= ~W100_CIF_CNTL_EN_WAIT_TO_COMPENSATE_DQ_PROP_DLY;
	val |= W100_CIF_CNTL_INTB_OE | W100_CIF_CNTL_INTERRUPT_ACTIVE_HIGH |
	       W100_CIF_CNTL_DIS_SYSTEM_BITS | W100_CIF_CNTL_DIS_MR;
	writel(val, w100->regs + W100_REG_CIF_CNTL);

	/*
	 * cfgINTF_CNTL/cfgCPU_DEFAULTS: the recovered driver ORs
	 * ad_inc_a/ad_inc_b/access_ind_addr_a/access_ind_addr_b/
	 * access_scratch_reg on top of defINTF_CNTL/defCPU_DEFAULTS, but
	 * every one of those bits is already part of the default constant
	 * or the sole bit the write adds (access_scratch_reg) -- see
	 * W100_DEF_INTF_CNTL/W100_DEF_CPU_DEFAULTS' own comments. Net
	 * effect is a plain write of the pre-computed constant, matching
	 * w100fb.c:921-934.
	 */
	writeb(W100_DEF_INTF_CNTL, w100->cfg + W100_CFG_INTF_CNTL);
	writeb(W100_DEF_CPU_DEFAULTS, w100->cfg + W100_CFG_CPU_DEFAULTS);

	/* cfgREG_BASE <- W100_REG_BASE >> 16 -- w100fb.c:937. */
	writeb(W100_REG_BASE >> 16, w100->cfg + W100_CFG_REG_BASE);

	/* mmCFGREG_BASE <- 0 (defCFGREG_BASE with cfgreg_base=W100_CFG_BASE,
	 * i.e. 0) -- w100fb.c:939-941.
	 */
	writel(0, w100->regs + W100_REG_CFGREG_BASE);

	/* mmWRAP_START_DIR/mmWRAP_TOP_DIR -- w100fb.c:943-949. */
	writel(W100_WRAP_START_DIR_VAL, w100->regs + W100_REG_WRAP_START_DIR);
	writel(W100_WRAP_TOP_DIR_VAL, w100->regs + W100_REG_WRAP_TOP_DIR);

	/* mmRBBM_CNTL -- fixed constant, w100fb.c:951. */
	writel(0x2440, w100->regs + W100_REG_RBBM_CNTL);

	/* mmDISP_DEBUG2: force the "565 colour" bit -- w100fb.c:954-957. */
	val = readl(w100->regs + W100_REG_DISP_DEBUG2);
	val |= BIT(23);
	writel(val, w100->regs + W100_REG_DISP_DEBUG2);

	/* The W3220's own GPIO block -- w100fb.c:959-967. */
	writel(hx4700_w100_gpio.init_data1, w100->regs + W100_REG_GPIO_DATA);
	writel(hx4700_w100_gpio.init_data2, w100->regs + W100_REG_GPIO_DATA2);
	writel(hx4700_w100_gpio.gpio_dir1, w100->regs + W100_REG_GPIO_CNTL1);
	writel(hx4700_w100_gpio.gpio_oe1, w100->regs + W100_REG_GPIO_CNTL2);
	writel(hx4700_w100_gpio.gpio_dir2, w100->regs + W100_REG_GPIO_CNTL3);
	writel(hx4700_w100_gpio.gpio_oe2, w100->regs + W100_REG_GPIO_CNTL4);
}

/* ------------------------------------------------------------------ */
/* PLL calibration, ported from w100fb.c:1042-1207. Fout = (xtal /
 * (M+1)) * ((N_int+1) + (N_fac/8)), programmed via a DAC-comparator
 * search rather than a closed-form register value -- see
 * w100_pll_adjust()'s own port below for the real algorithm.
 */

static unsigned int w100_pll_test_count(struct w100_device *w100)
{
	u32 val;

	udelay(5);

	val = FIELD_PREP(W100_CLK_TEST_CNTL_TESTCLK_SEL, W100_TESTCLK_SRC_PLL) |
	      W100_CLK_TEST_CNTL_TSTCOUNT_RST;
	writel(val, w100->regs + W100_REG_CLK_TEST_CNTL);
	val &= ~W100_CLK_TEST_CNTL_TSTCOUNT_RST;
	writel(val, w100->regs + W100_REG_CLK_TEST_CNTL);

	val |= W100_CLK_TEST_CNTL_START_CHECK_FREQ;
	writel(val, w100->regs + W100_REG_CLK_TEST_CNTL);

	usleep_range(20, 40);

	val = readl(w100->regs + W100_REG_CLK_TEST_CNTL);
	writel(val & ~W100_CLK_TEST_CNTL_START_CHECK_FREQ,
	       w100->regs + W100_REG_CLK_TEST_CNTL);

	return FIELD_GET(W100_CLK_TEST_CNTL_TEST_COUNT, val);
}

/* w100fb.c:1073-1125. Binary-search-like DAC sweep of the VCO input
 * voltage until the test-clock counter brackets the target tfgoal.
 */
static int w100_pll_adjust(struct w100_device *w100, const struct w100_pll_entry *pll)
{
	unsigned int tf80, tf20;
	u32 pcp, pvg, vcofr, ioffset, dactal;

	pcp = 4;
	pvg = 0;
	vcofr = 0;
	ioffset = 0;

	while (1) {
		dactal = 0xd;	/* VCO input = 0.8 * VDD */
		w100->pll_cntl = FIELD_PREP(W100_PLL_CNTL_TCPOFF, 1) |
				 FIELD_PREP(W100_PLL_CNTL_PCP, pcp) |
				 FIELD_PREP(W100_PLL_CNTL_PVG, pvg) |
				 FIELD_PREP(W100_PLL_CNTL_VCOFR, vcofr) |
				 FIELD_PREP(W100_PLL_CNTL_IOFFSET, ioffset) |
				 FIELD_PREP(W100_PLL_CNTL_DACTAL, dactal) |
				 (w100->pll_cntl & (W100_PLL_CNTL_CP_CLIP |
						    W100_PLL_CNTL_CONF |
						    W100_PLL_CNTL_MBCTRL));
		writel(w100->pll_cntl, w100->regs + W100_REG_PLL_CNTL);

		tf80 = w100_pll_test_count(w100);
		if (tf80 >= pll->tfgoal) {
			dactal = 0x7;	/* VCO input = 0.2 * VDD */
			w100->pll_cntl = (w100->pll_cntl & ~W100_PLL_CNTL_DACTAL) |
					  FIELD_PREP(W100_PLL_CNTL_DACTAL, dactal);
			writel(w100->pll_cntl, w100->regs + W100_REG_PLL_CNTL);

			tf20 = w100_pll_test_count(w100);
			if (tf20 <= pll->tfgoal)
				return 0;

			if (vcofr == 0 && (pvg == 7 || ioffset == 0)) {
				vcofr = 1;
				pvg = 0;
				ioffset = 0;
				continue;
			}
		}

		if (ioffset < 3) {
			ioffset++;
		} else if (pvg < 7) {
			ioffset = 0;
			pvg++;
		} else {
			return -EIO;
		}
	}
}

/* w100fb.c:1131-1157. */
static int w100_pll_calibrate(struct w100_device *w100, const struct w100_pll_entry *pll)
{
	int ret;

	ret = w100_pll_adjust(w100, pll);

	/* VCO input = 0.5 * VDD, reset and lock */
	w100->pll_cntl = (w100->pll_cntl & ~W100_PLL_CNTL_DACTAL) |
			  FIELD_PREP(W100_PLL_CNTL_DACTAL, 0xa);
	writel(w100->pll_cntl, w100->regs + W100_REG_PLL_CNTL);
	udelay(1);

	w100->pll_cntl &= ~W100_PLL_CNTL_TCPOFF;	/* enable charge pump */
	writel(w100->pll_cntl, w100->regs + W100_REG_PLL_CNTL);

	w100->pll_cntl &= ~W100_PLL_CNTL_DACTAL;	/* Hi-Z, disable DAC */
	writel(w100->pll_cntl, w100->regs + W100_REG_PLL_CNTL);
	usleep_range(400, 500);	/* lock time */

	return ret;
}

/* w100fb.c:1160-1193. auto_mode is always manual (0) in this driver --
 * w100_pwm_setup() never sets it otherwise and nothing here changes it --
 * so the recovered driver's own `if (auto_mode == 1)` blocks around this
 * function are dead code for this board and are dropped.
 */
static int w100_pll_set_clk(struct w100_device *w100, const struct w100_pll_entry *pll)
{
	/* System clock source = XTAL while the PLL is being adjusted. */
	w100->sclk_cntl = (w100->sclk_cntl & ~W100_SCLK_CNTL_SRC_SEL) |
			   FIELD_PREP(W100_SCLK_CNTL_SRC_SEL, 0 /* CLK_SRC_XTAL */);
	writel(w100->sclk_cntl, w100->regs + W100_REG_SCLK_CNTL);

	/* reset_time is preserved, not reassigned -- w100fb.c's
	 * own w100_pll_set_clk() (w100fb.c:1175-1179) never touches
	 * pll_ref_fb_div.f.pll_reset_time, leaving it at the value
	 * w100_pwm_setup() set (5); the persistent struct that field lives
	 * in makes that implicit there, so it has to be explicit here.
	 */
	w100->pll_ref_fb_div = (w100->pll_ref_fb_div & W100_PLL_REF_FB_DIV_RESET_TIME) |
				FIELD_PREP(W100_PLL_REF_FB_DIV_REF_DIV, pll->m) |
				FIELD_PREP(W100_PLL_REF_FB_DIV_FB_INT, pll->n_int) |
				FIELD_PREP(W100_PLL_REF_FB_DIV_FB_FRAC, pll->n_fac) |
				FIELD_PREP(W100_PLL_REF_FB_DIV_LOCK_TIME, pll->lock_time);
	writel(w100->pll_ref_fb_div, w100->regs + W100_REG_PLL_REF_FB_DIV);

	w100->pwrmgt_cntl &= ~W100_PWRMGT_CNTL_MODE_REQ;
	writel(w100->pwrmgt_cntl, w100->regs + W100_REG_PWRMGT_CNTL);

	return w100_pll_calibrate(w100, pll);
}

static int w100_set_pll_freq(struct w100_device *w100, unsigned int freq)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(hx4700_w100_pll_table); i++) {
		if (hx4700_w100_pll_table[i].freq == freq)
			return w100_pll_set_clk(w100, &hx4700_w100_pll_table[i]);
	}

	return -EINVAL;
}

/* w100fb.c:1211-1285's w100_pwm_setup(): establish a known baseline
 * clock state on XTAL before every mode (re)programming. clk_pin_cntl
 * and pclk_cntl need no persistent state (see their own field comments
 * in struct w100_device); sclk_cntl/pll_cntl/pll_ref_fb_div/pwrmgt_cntl
 * do, and are (re)established here in full every time.
 */
static void w100_pwm_setup(struct w100_device *w100)
{
	u32 val;

	/* mmCLK_PIN_CNTL: osc_en=1, osc_gain=0x1f, xtalin_dbl_en=0 (hx4700's
	 * own w3220_info carries no xtal_dbl) -- w100fb.c:1213-1219.
	 */
	val = W100_CLK_PIN_CNTL_OSC_EN |
	      FIELD_PREP(W100_CLK_PIN_CNTL_OSC_GAIN, 0x1f);
	writel(val, w100->regs + W100_REG_CLK_PIN_CNTL);

	/* mmSCLK_CNTL: src=XTAL, post_div_fast=0 (Pfast=1), clkon_hys=3,
	 * post_div_slow=0 (Pslow=1), every "Dynamic"/force_* field 0 --
	 * w100fb.c:1221-1239.
	 */
	w100->sclk_cntl = FIELD_PREP(W100_SCLK_CNTL_SRC_SEL, 0 /* CLK_SRC_XTAL */) |
			  FIELD_PREP(W100_SCLK_CNTL_CLKON_HYS, 3);
	writel(w100->sclk_cntl, w100->regs + W100_REG_SCLK_CNTL);

	/* mmPCLK_CNTL: src=XTAL, post_div=1 (P=2), force_disp=0 --
	 * w100fb.c:1241-1244.
	 */
	val = FIELD_PREP(W100_PCLK_CNTL_SRC_SEL, 0 /* CLK_SRC_XTAL */) |
	      FIELD_PREP(W100_PCLK_CNTL_POST_DIV, 1);
	writel(val, w100->regs + W100_REG_PCLK_CNTL);

	/* mmPLL_REF_FB_DIV: M=1 (ref_div=0), N=1.0 (fb_div_int=0,
	 * fb_div_frac=0), reset_time=5, lock_time=0xff -- w100fb.c:
	 * 1246-1251.
	 */
	w100->pll_ref_fb_div = FIELD_PREP(W100_PLL_REF_FB_DIV_RESET_TIME, 5) |
				FIELD_PREP(W100_PLL_REF_FB_DIV_LOCK_TIME, 0xff);
	writel(w100->pll_ref_fb_div, w100->regs + W100_REG_PLL_REF_FB_DIV);

	/* mmPLL_CNTL: power down + reset, tcpoff=0, pcp=4, dactal=0 (Hi-Z),
	 * cp_clip=3, conf=2, mbctrl=2 -- w100fb.c:1253-1271. pll_mode/
	 * pll_refclk_sel/pll_fbclk_sel/pll_pm_en/pll_pecc_mode/
	 * pll_pecc_scon/pll_ring_off are all 0 and have no consumer in this
	 * driver, so no field macro exists for them.
	 */
	w100->pll_cntl = W100_PLL_CNTL_PWDN | W100_PLL_CNTL_RESET |
			  FIELD_PREP(W100_PLL_CNTL_PCP, 4) |
			  FIELD_PREP(W100_PLL_CNTL_CP_CLIP, 3) |
			  FIELD_PREP(W100_PLL_CNTL_CONF, 2) |
			  FIELD_PREP(W100_PLL_CNTL_MBCTRL, 2);
	writel(w100->pll_cntl, w100->regs + W100_REG_PLL_CNTL);

	/* mmPWRMGT_CNTL: enable=0, mode_req=1 (normal), the two hw_en bits
	 * 0 (manual mode), fast_noml_cond=noml_fast_cond=1 (PM4,ENG),
	 * idle_timer=busy_timer=0xff -- w100fb.c:1273-1284.
	 */
	w100->pwrmgt_cntl = FIELD_PREP(W100_PWRMGT_CNTL_MODE_REQ, 1) |
			     FIELD_PREP(W100_PWRMGT_CNTL_FAST_NOML_COND, 1) |
			     FIELD_PREP(W100_PWRMGT_CNTL_NOML_FAST_COND, 1) |
			     FIELD_PREP(W100_PWRMGT_CNTL_IDLE_TIMER, 0xff) |
			     FIELD_PREP(W100_PWRMGT_CNTL_BUSY_TIMER, 0xff);
	writel(w100->pwrmgt_cntl, w100->regs + W100_REG_PWRMGT_CNTL);
}

/* w100fb.c:1291-1302. Both hx4700 hw modes use CLK_SRC_PLL for both
 * pixclk_src and sysclk_src with an identical pll_freq/pixclk_divider,
 * but the PLL is still (re)programmed generically from the selected
 * hw mode's own fields, not hardcoded, in case a future board/mode adds
 * a genuinely different value.
 */
static void w100_init_clocks(struct w100_device *w100, const struct w100_hw_mode *hw)
{
	int ret;

	ret = w100_set_pll_freq(w100, hw->pll_freq);
	if (ret)
		drm_warn(&w100->drm, "PLL calibration for %u MHz did not converge\n",
			 hw->pll_freq);

	/*
	 * sysclk_src/sysclk_divider from the recovered w100_mode struct are
	 * dropped from struct w100_hw_mode (see its own comment): both
	 * hx4700.c:431-477 hw modes carry the identical
	 * .sysclk_src=CLK_SRC_PLL, .sysclk_divider=0, so src=PLL and
	 * post_div_fast=post_div_slow=0 are hardcoded here rather than
	 * threaded through as per-mode fields with only one real value.
	 */
	w100->sclk_cntl = FIELD_PREP(W100_SCLK_CNTL_SRC_SEL, 1 /* CLK_SRC_PLL */) |
			   (w100->sclk_cntl & W100_SCLK_CNTL_CLKON_HYS);
	writel(w100->sclk_cntl, w100->regs + W100_REG_SCLK_CNTL);
}

/* ------------------------------------------------------------------ */
/* Memory controller. w100fb.c:1371-1422's w100_setup_memory(), always
 * taking the "extmem_active" branch: hx4700's own w3220_info always
 * supplies both mem/bm_mem, so the recovered driver's own
 * !par->extmem_active early-init branch (which leaves the chip's
 * internal SRAM mapped at the framebuffer's usual place and never gets
 * taken again after the very first activation on this board) is dead
 * code here and is dropped.
 */
static void w100_setup_memory(struct w100_device *w100)
{
	const struct w100_mem_info *mem = &hx4700_w100_mem;
	const struct w100_bm_mem_info *bm = &hx4700_w100_bm_mem;

	/* Internal SRAM stays at its own default location -- nothing on
	 * the host side maps or touches it.
	 */
	writel(W100_MC_LOCATION(W100_MEM_INT_BASE_VALUE >> 8,
				 (W100_MEM_INT_BASE_VALUE + W100_MEM_INT_SIZE) >> 8),
	       w100->regs + W100_REG_MC_FB_LOCATION);

	/* "External" (see W100_FB_BASE's own comment) memory mapped at the
	 * framebuffer base.
	 */
	writel(W100_MC_LOCATION(W100_FB_BASE >> 8, (W100_FB_BASE + mem->size) >> 8),
	       w100->regs + W100_REG_MC_EXT_MEM_LOCATION);

	writel(0x00007800, w100->regs + W100_REG_MC_BIST_CTRL);
	writel(mem->ext_cntl, w100->regs + W100_REG_MEM_EXT_CNTL);

	/* SDRAM mode-register-set training: two fixed pulses, then the
	 * board's own value -- w100fb.c:1404-1409.
	 */
	writel(0x00200021, w100->regs + W100_REG_MEM_SDRAM_MODE_REG);
	usleep_range(100, 200);
	writel(0x80200021, w100->regs + W100_REG_MEM_SDRAM_MODE_REG);
	usleep_range(100, 200);
	writel(mem->sdram_mode_reg, w100->regs + W100_REG_MEM_SDRAM_MODE_REG);
	usleep_range(100, 200);

	writel(mem->ext_timing_cntl, w100->regs + W100_REG_MEM_EXT_TIMING_CNTL);
	writel(mem->io_cntl, w100->regs + W100_REG_MEM_IO_CNTL);

	writel(bm->ext_mem_bw, w100->regs + W100_REG_BM_EXT_MEM_BANDWIDTH);
	writel(bm->offset, w100->regs + W100_REG_BM_OFFSET);
	writel(bm->ext_timing_ctl, w100->regs + W100_REG_BM_MEM_EXT_TIMING_CNTL);
	writel(bm->ext_cntl, w100->regs + W100_REG_BM_MEM_EXT_CNTL);
	writel(bm->mode_reg, w100->regs + W100_REG_BM_MEM_MODE_REG);
	writel(bm->io_cntl, w100->regs + W100_REG_BM_MEM_IO_CNTL);
	writel(bm->config, w100->regs + W100_REG_BM_CONFIG);
}

/* ------------------------------------------------------------------ */
/* CRTC/LCD register programming for one hw mode, w100fb.c:1304-1509. */

static void w100_init_lcd(struct w100_device *w100, const struct w100_hw_mode *hw)
{
	u32 val;

	/* w3200 doesn't like undefined bits being set, so every register
	 * below is a full replacement write, not a read-modify-write --
	 * w100fb.c:1315.
	 */
	writel(W100_DISP_RANGE(hw->left_margin, hw->left_margin + hw->xres),
	       w100->regs + W100_REG_ACTIVE_H_DISP);
	writel(W100_DISP_RANGE(hw->upper_margin, hw->upper_margin + hw->yres),
	       w100->regs + W100_REG_ACTIVE_V_DISP);
	writel(W100_DISP_RANGE(hw->left_margin, hw->left_margin + hw->xres),
	       w100->regs + W100_REG_GRAPHIC_H_DISP);
	writel(W100_DISP_RANGE(hw->upper_margin, hw->upper_margin + hw->yres),
	       w100->regs + W100_REG_GRAPHIC_V_DISP);

	/*
	 * mmCRTC_TOTAL packs crtc_h_total in the low 10 bits and
	 * crtc_v_total in bits 16-25 -- the same shape as the four
	 * DISP_RANGE registers above, but h_total/v_total (a single overall
	 * period) rather than a start/end pair, so it reuses the same macro
	 * against different real values.
	 */
	writel(W100_DISP_RANGE(hw->left_margin + hw->xres + hw->right_margin,
				hw->upper_margin + hw->yres + hw->lower_margin),
	       w100->regs + W100_REG_CRTC_TOTAL);

	writel(hw->crtc_ss, w100->regs + W100_REG_CRTC_SS);
	writel(hw->crtc_ls, w100->regs + W100_REG_CRTC_LS);
	writel(hw->crtc_gs, w100->regs + W100_REG_CRTC_GS);
	writel(hw->crtc_vpos_gs, w100->regs + W100_REG_CRTC_VPOS_GS);
	writel(hw->crtc_rev, w100->regs + W100_REG_CRTC_REV);
	writel(hw->crtc_dclk, w100->regs + W100_REG_CRTC_DCLK);
	writel(hw->crtc_gclk, w100->regs + W100_REG_CRTC_GCLK);
	writel(hw->crtc_goe, w100->regs + W100_REG_CRTC_GOE);
	writel(hw->crtc_ps1_active, w100->regs + W100_REG_CRTC_PS1_ACTIVE);

	writel(hx4700_w100_regs.lcd_format, w100->regs + W100_REG_LCD_FORMAT);
	writel(hx4700_w100_regs.lcdd_cntl1, w100->regs + W100_REG_LCDD_CNTL1);
	writel(hx4700_w100_regs.lcdd_cntl2, w100->regs + W100_REG_LCDD_CNTL2);
	writel(hx4700_w100_regs.genlcd_cntl1, w100->regs + W100_REG_GENLCD_CNTL1);
	writel(hx4700_w100_regs.genlcd_cntl2, w100->regs + W100_REG_GENLCD_CNTL2);
	writel(hx4700_w100_regs.genlcd_cntl3, w100->regs + W100_REG_GENLCD_CNTL3);

	writel(0x00000000, w100->regs + W100_REG_CRTC_FRAME);
	writel(0x00000000, w100->regs + W100_REG_CRTC_FRAME_VPOS);
	writel(0x00000000, w100->regs + W100_REG_CRTC_DEFAULT_COUNT);
	writel(0x0000ff00, w100->regs + W100_REG_LCD_BACKGROUND_COLOR);

	/* "Hack for overlay in ext memory" -- w100fb.c:1364-1367. */
	val = readl(w100->regs + W100_REG_DISP_DEBUG2);
	val |= GENMASK(31, 30);
	writel(val, w100->regs + W100_REG_DISP_DEBUG2);
}

/* w100fb.c:1424-1509, with the rotation/flip branch (rot always 0,
 * offset always 0 -- see the file comment) and the W100/W3200 chip-ID
 * branches (this driver only ever targets W3220) both dropped.
 */
static void w100_set_dispregs(struct w100_device *w100, const struct w100_hw_mode *hw)
{
	u32 val;

	val = FIELD_PREP(W100_PCLK_CNTL_SRC_SEL, 1 /* CLK_SRC_PLL, both hx4700 modes */) |
	      FIELD_PREP(W100_PCLK_CNTL_POST_DIV, hw->pixclk_divider);
	writel(val, w100->regs + W100_REG_PCLK_CNTL);

	val = FIELD_PREP(W100_GRAPHIC_CTRL_COLOR_DEPTH, 6) |
	      W100_GRAPHIC_CTRL_EN_CRTC | W100_GRAPHIC_CTRL_EN_GRAPHIC_REQ |
	      W100_GRAPHIC_CTRL_EN_GRAPHIC_CRTC |
	      FIELD_PREP(W100_GRAPHIC_CTRL_TOTAL_REQ_GRAPHIC, hw->xres >> 1) |
	      W100_GRAPHIC_CTRL_LCD_PCLK_ON | W100_GRAPHIC_CTRL_LCD_SCLK_ON;
	writel(val, w100->regs + W100_REG_GRAPHIC_CTRL);

	writel(W100_FB_BASE, w100->regs + W100_REG_GRAPHIC_OFFSET);
	writel(hw->xres * 2 /* RGB565 */, w100->regs + W100_REG_GRAPHIC_PITCH);
}

/*
 * w100_vsync() in the recovered driver (w100fb.c:1591-1628) -- an
 * internal busy-wait on one vline interrupt event used purely to pace
 * w100fb_activate_var()'s own register reprogramming sequence, unrelated
 * to W3220_IRQ (which no driver has ever used, see the file comment) and
 * unrelated to DRM's vblank machinery, which runs from a timer because
 * the chip's interrupt line is not described in the device tree. Ported
 * as a private synchronisation helper only.
 */
static void w100_wait_vline(struct w100_device *w100)
{
	u32 val;
	int timeout = 30000;	/* 30 ms > 16.8 ms, w100fb.c:1594 */

	val = readl(w100->regs + W100_REG_ACTIVE_V_DISP);
	writel((val >> 16) & 0x3ff, w100->regs + W100_REG_DISP_INT_CNTL);

	val = readl(w100->regs + W100_REG_GEN_INT_CNTL);
	val &= ~BIT(1);
	writel(val, w100->regs + W100_REG_GEN_INT_CNTL);
	writel(BIT(1), w100->regs + W100_REG_GEN_INT_STATUS);
	writel(val | BIT(1), w100->regs + W100_REG_GEN_INT_CNTL);
	writel(BIT(1), w100->regs + W100_REG_GEN_INT_STATUS);

	while (timeout > 0) {
		if (readl(w100->regs + W100_REG_GEN_INT_STATUS) & BIT(1))
			break;
		udelay(1);
		timeout--;
	}

	writel(val, w100->regs + W100_REG_GEN_INT_CNTL);
	writel(BIT(1), w100->regs + W100_REG_GEN_INT_STATUS);
}

static void w100_update_disable(struct w100_device *w100)
{
	writel(W100_DISP_DB_BUF_CNTL_DISABLE, w100->regs + W100_REG_DISP_DB_BUF_CNTL);
}

static void w100_update_enable(struct w100_device *w100)
{
	writel(W100_DISP_DB_BUF_CNTL_ENABLE, w100->regs + W100_REG_DISP_DB_BUF_CNTL);
}

/* ------------------------------------------------------------------ */
/*
 * The GFX 2D engine: solid fill and screen-to-screen copy, all 16
 * raster operations, driven by polled MMIO with no command ring and no
 * microcode.
 *
 * Ported from the two independent working sources for this block:
 * w100fb.c's own w100_fifo_wait()/w100fb_sync()/w100_init_graphic_
 * engine()/w100fb_fillrect()/w100fb_copyarea() (w100fb.c:262-406), and
 * kdrive's ATIWaitAvailMMIO()/ATIWaitIdle()/ATIEngineReset()
 * (ati_dma.c:111-172) plus ATIDrawSetup()/ATIPrepareSolid()/ATISolid()/
 * ATIPrepareCopy()/ATICopy() (ati_draw.c:112-403). Where the two differ
 * the choice is recorded at the point it is made. What is taken from
 * each, in one line: the register set and the engine's quiescent state
 * from w100fb.c, the full 16-entry ROP3 tables and the timeout/reset
 * recovery from kdrive (w100fb.c has neither -- it hardcodes SRCCOPY
 * and PATCOPY, and on a timeout it only prints).
 *
 * Not implemented, because the hardware cannot do it:
 * colour-expand/imageblit (w100fb.c:560 falls back to cfb_imageblit and
 * kdrive wires no such hook), alpha compositing (kdrive NULLs every
 * PrepareBlend/Composite hook in ATIDrawEnable()), and a hardware
 * cursor (no mmCUR_* register exists in any of the three register maps).
 */

/*
 * ATISolidRop[16] and ATIBltRop[16], ati_draw.c:38-105, transcribed
 * verbatim and indexed by enum w100_rop. The two tables are genuinely
 * different rather than a copy of each other -- GXcopy is 0xf0 here and
 * 0xcc below -- because a ROP3 combines pattern, source and destination
 * and the solid path's "source" is the brush, not a source surface.
 */
static const u8 w100_solid_rop3[W100_ROP_COUNT] = {
	0x00, 0xa0, 0x50, 0xf0, 0x0a, 0xaa, 0x5a, 0xfa,
	0x05, 0xa5, 0x55, 0xf5, 0x0f, 0xaf, 0x5f, 0xff,
};

static const u8 w100_blt_rop3[W100_ROP_COUNT] = {
	0x00, 0x88, 0x44, 0xcc, 0x22, 0xaa, 0x66, 0xee,
	0x11, 0x99, 0x55, 0xdd, 0x33, 0xbb, 0x77, 0xff,
};

/*
 * kdrive's own WHILE_NOT_TIMEOUT(.2) bound (ati_dma.c:124,152), in
 * microseconds. w100fb.c polls ten times longer (2,000,000 udelay(1)
 * iterations, w100fb.c:268,283) but only to print and give up, so
 * kdrive's shorter bound is the one that pairs with a real recovery.
 */
#define W100_2D_TIMEOUT_US		200000

/*
 * Depth of the host-side command FIFO, i.e. the cmdfifo_avail value that
 * means "empty". kdrive's ATIWaitIdle() asks for exactly 16 slots under
 * the comment "Empty the fifo" (ati_dma.c:150-151), and since every other
 * ATIWaitAvailMMIO() call there asks for 8 or fewer and the server worked,
 * 16 is both reachable and the author's own figure for a drained FIFO.
 *
 * If the real FIFO were shallower than this, cmdfifo_avail could never
 * reach 16 and every operation would fail its idle wait -- which kdrive
 * would have hit on every ATIWaitIdle() too, so it is well attested. On
 * first hardware the symptom would still be unmissable and the diagnosis
 * is one line: a "timeout waiting for 16 command FIFO slots" on the very
 * first 2D operation means this number is too high, not that the chip is
 * dead.
 */
#define W100_2D_CMDFIFO_DEPTH		16

static int w100_2d_fifo_wait(struct w100_device *w100, unsigned int entries)
{
	u32 status;
	int ret;

	ret = readl_poll_timeout_atomic(w100->regs + W100_REG_RBBM_STATUS, status,
					FIELD_GET(W100_RBBM_STATUS_CMDFIFO_AVAIL,
						  status) >= entries,
					1, W100_2D_TIMEOUT_US);
	if (ret)
		drm_err(&w100->drm, "2D engine: timeout waiting for %u command FIFO slots (RBBM_STATUS 0x%08x)\n",
			entries, status);

	return ret;
}

/*
 * Wait for the engine to go idle -- ATIWaitIdle(), ati_dma.c:139-172.
 *
 * The command-FIFO drain is not optional and is the whole reason this is
 * two waits rather than one. gui_active reports the *engine*, while the
 * register writes that start an operation land in the command FIFO first
 * and are consumed from it asynchronously; rbbm_status_t carries
 * cf_pipe_busy/cp_cmdstrm_busy separately from gui_active precisely
 * because they are different things. So polling gui_active straight after
 * writing the launch register can observe 0 for the window before the
 * engine has even seen that write, and return with the operation not
 * merely unfinished but unstarted. Every caller here then reads the
 * destination back or hands it to a CPU copy, so what that produces is
 * stale pixels rather than a visible stall.
 *
 * kdrive drains first for exactly this reason, under its own "Empty the
 * fifo" comment. w100fb.c has no equivalent because it never waited after
 * issuing at all: w100fb_fillrect()/w100fb_copyarea() return with the blit
 * in flight and w100fb_sync() is the fbdev core's .fb_sync hook, called
 * before CPU access rather than after issue.
 *
 * kdrive follows its gui_active loop with ATIFlushPixelCache(); that
 * function is nothing but a second poll of the same bit (ati_dma.c:66-82 --
 * it writes no cache register at all, despite its name and its comment), so
 * there is nothing further to do here.
 */
static int w100_2d_wait_idle(struct w100_device *w100)
{
	u32 status;
	int ret;

	ret = w100_2d_fifo_wait(w100, W100_2D_CMDFIFO_DEPTH);
	if (ret)
		return ret;

	ret = readl_poll_timeout_atomic(w100->regs + W100_REG_RBBM_STATUS, status,
					!(status & W100_RBBM_STATUS_GUI_ACTIVE),
					1, W100_2D_TIMEOUT_US);
	if (ret)
		drm_err(&w100->drm, "2D engine: idle timeout (RBBM_STATUS 0x%08x)\n",
			status);

	return ret;
}

/*
 * ATIEngineReset(), ati_dma.c:83-109: force the E2/E3/IDCT clocks on
 * across a soft-reset pulse of the 2D block, then put mmSCLK_CNTL back.
 * The register is read back rather than taken from w100->sclk_cntl
 * because that shadow tracks what w100_pll_set_clk() last programmed,
 * which is the same thing here but would silently stop being so if the
 * clock code ever grew another writer.
 *
 * kdrive calls ATIFlushPixelCache() first; that is another poll for
 * gui_active to fall, and every path that reaches this function got
 * here *because* that bit is stuck, so it is skipped -- it would only
 * add another timeout before the reset that clears the condition.
 *
 * The reset clears the source and default scissors that
 * w100_2d_setup_locked() exists to establish, so @accel_ready is dropped
 * here and only the next w100_2d_setup_locked() puts it back. Every
 * caller that means to keep using the engine already re-runs that -- the
 * same pairing kdrive has, where an ATIEngineReset() is always followed
 * by an ATIDrawSetup(). The one that does not is .atomic_disable's ring
 * teardown, which is exactly the case this matters for: without it the
 * engine is left reset with @accel_ready still true, and the next
 * operation to reach it runs against wiped scissors.
 */
static void w100_2d_engine_reset(struct w100_device *w100)
{
	u32 sclk_cntl = readl(w100->regs + W100_REG_SCLK_CNTL);

	w100->accel_ready = false;

	writel(sclk_cntl | W100_SCLK_CNTL_FORCE_E2 | W100_SCLK_CNTL_FORCE_E3 |
	       W100_SCLK_CNTL_FORCE_IDCT, w100->regs + W100_REG_SCLK_CNTL);

	writel(W100_RBBM_SOFT_RESET_E2, w100->regs + W100_REG_RBBM_SOFT_RESET);
	writel(0, w100->regs + W100_REG_RBBM_SOFT_RESET);

	writel(sclk_cntl, w100->regs + W100_REG_SCLK_CNTL);
}

/*
 * Both historical drivers write the pitch registers in units of 16-bit
 * words -- w100fb.c:302 writes par->xres for a 16bpp framebuffer, and
 * ATIGetPixmapOffsetPitch() (ati_draw.c:196-206) writes devKind >> 1,
 * i.e. the byte pitch halved. Those are the same number, and this
 * driver's only pixel format is 16bpp, so the two agree exactly.
 */
static u32 w100_2d_pitch_reg(u32 pitch_bytes)
{
	return FIELD_PREP(W100_2D_PITCH, pitch_bytes >> 1);
}

static void w100_2d_setup_locked(struct w100_device *w100,
				 const struct w100_hw_mode *hw);

/* ------------------------------------------------------------------ */
/*
 * The command processor: the same GFX block as above, reached through a
 * ring buffer of CCE packets instead of through polled register writes.
 * It performs no operation the polled path cannot; what it removes is
 * the per-burst mmRBBM_STATUS.cmdfifo_avail poll, because the processor
 * paces itself against the ring's own read pointer.
 *
 * Ported from kdrive's ATIDMAInit()/ATIDMAFini()/ATIRequestEntries()/
 * ATISubmitEntries()/ATIUploadMicrocode() (ati_dma.c:49-61,175-264,
 * 297-333) and the DRAW_USING_PACKET3 arms of ATISolid()/ATICopy()
 * (ati_draw.c:267-272,382-388). That is the *only* source: the recovered
 * w100fb.c never used this block and w100fb.h declares no mmCP_*
 * register, so unlike the GFX block above nothing here is confirmed by a
 * second independent driver. Everything below is one working 2006 X
 * server's usage, transcribed, with its own defects fixed where they are
 * demonstrable from the source itself -- each one is called out at the
 * point it is fixed.
 *
 * Nothing here has run on real silicon.
 *
 * There is no fence, no scheduler and no dma_fence
 * signalling. This hardware has no completion interrupt to hang one on:
 * mmCP_RB_RPTR_ADDR writeback is left disabled exactly as kdrive leaves
 * it, and the board's W3220_IRQ line (PXA GPIO13) has never been wired
 * by any driver for this chip. Completion is polled -- see
 * w100_cp_ring_drain() -- which is what both this hardware and this
 * driver's callers actually want, since every caller either reads the
 * result back or hands the memory straight to a CPU copy.
 */

/*
 * kdrive's own WHILE_NOT_TIMEOUT(1) bound for ring space
 * (ATIRequestEntries(), ati_dma.c:304), in microseconds. Longer than the
 * GFX engine's own 200 ms because it is waiting on a queue of work
 * rather than on a single operation.
 */
#define W100_CP_RING_TIMEOUT_US	1000000

static bool w100_cp_active(struct w100_device *w100)
{
	return w100->ring_ready;
}

/*
 * Upload the microcode engine's instruction RAM.
 *
 * This is r100_cp_load_microcode()'s loop (drivers/gpu/drm/radeon/
 * r100.c:1128-1142), not kdrive's ATIUploadMicrocode()'s, on purpose.
 * kdrive iterates a hardcoded 256 times over an array with 192 entries
 * (ati_dma.c:57 against ati_microcode.c's own initialiser count), so it
 * reads 512 bytes past the end of that array and writes whatever
 * followed it in .data into ME RAM entries 192-255. Deriving the count
 * from the firmware's own size is both the fix and what makes the file's
 * true length -- which is *not* recoverable from kdrive's source, since
 * its array's tail is already zero-padded -- stop mattering.
 *
 * The file stores each entry as two big-endian words, DATAH first, which
 * is the r128/radeon layout ("Firmware blob looks like this: __be32
 * datah / __be32 datal", 52f97df5da1f). kdrive's C array stores the same
 * two words in the opposite order; the generator that produces the file
 * from that array is where the swap happens, precisely so this loop can
 * stay identical to r100's.
 */
static void w100_cp_upload_microcode(struct w100_device *w100,
				     const struct firmware *fw)
{
	const __be32 *data = (const __be32 *)fw->data;
	size_t i, words = fw->size / 4;

	writel(0, w100->regs + W100_REG_CP_ME_RAM_ADDR);

	for (i = 0; i < words; i += 2) {
		writel(be32_to_cpup(&data[i]), w100->regs + W100_REG_CP_ME_RAM_DATAH);
		writel(be32_to_cpup(&data[i + 1]), w100->regs + W100_REG_CP_ME_RAM_DATAL);
	}
}

/*
 * Fetch and validate the microcode, once, caching it for later mode
 * applies. Retried on each attempt until it succeeds, because a built-in
 * driver's first modeset can easily precede the root filesystem the file
 * lives on; the log line is emitted only the first time so a system that
 * simply has no firmware installed does not repeat itself on every
 * enable.
 *
 * Absent or malformed firmware is not an error at any level above this
 * one: the caller keeps the polled path.
 */
static const struct firmware *w100_cp_get_firmware(struct w100_device *w100)
{
	const struct firmware *fw;
	int ret;

	if (w100->fw)
		return w100->fw;

	/*
	 * request_firmware_direct(), not request_firmware(): this file is
	 * genuinely optional, and the plain call would drag in the
	 * userspace-helper fallback on kernels that enable it -- which can
	 * block for its own timeout, from inside an atomic commit, every
	 * time a system without the firmware applies a mode. The direct
	 * form also suppresses the loader's own failure warning, leaving
	 * the one drm_info() below as the only thing said about it.
	 */
	ret = request_firmware_direct(&fw, W100_CP_FW_NAME, w100->drm.dev);
	if (ret) {
		if (!w100->fw_warned)
			drm_info(&w100->drm,
				 "no %s (%d); 2D operations will use polled register writes\n",
				 W100_CP_FW_NAME, ret);
		w100->fw_warned = true;
		return NULL;
	}

	/*
	 * Two words per entry, and no more entries than
	 * cp_me_ram_addr_t's own 8-bit address field can reach. Same
	 * shape of check as r100_cp_init_microcode()'s "Bogus length"
	 * (r100.c:1090-1096), plus the upper bound r100 does not need
	 * because its files are all exactly ME-RAM sized.
	 */
	if (!fw->size || fw->size % 8 || fw->size > W100_CP_ME_RAM_DEPTH * 8) {
		drm_warn(&w100->drm,
			 "%s has bogus length %zu; 2D operations will use polled register writes\n",
			 W100_CP_FW_NAME, fw->size);
		release_firmware(fw);
		w100->fw_warned = true;
		return NULL;
	}

	w100->fw = fw;

	return fw;
}

static void w100_cp_release_firmware(struct drm_device *drm, void *data)
{
	struct w100_device *w100 = data;

	release_firmware(w100->fw);
	w100->fw = NULL;
}

/*
 * Stop the processor fetching, then reset the GFX block underneath it --
 * ATIDMAFini(), ati_dma.c:249-264.
 */
static void w100_cp_ring_fini_locked(struct w100_device *w100)
{
	if (!w100->ring_ready)
		return;

	w100->ring_ready = false;

	writel(0, w100->regs + W100_REG_CP_CSQ_CNTL);
	w100_2d_engine_reset(w100);
}

/*
 * ATIDMAInit(), ati_dma.c:175-247, in its own order: microcode first,
 * then an engine reset, then the ring registers, then let the processor
 * run.
 *
 * The two addresses this hands the hardware are the whole trick, and
 * getting either wrong produces the same symptom -- a read pointer that
 * never advances. mmCP_RB_BASE is where the *processor* finds the ring,
 * a chip-internal address in video memory. mmWRAP_BUF_A.offset_addr_a is
 * where the *wrap unit* sends the host's writes, and it has to name the
 * same place, because the host cannot reach video memory at that address
 * itself: it writes into the wrap aperture (w100->wrap, chip offset
 * W100_WRAP_BASE) and the wrap unit redirects. Host and processor
 * therefore see one ring through two different addresses.
 *
 * mmCP_RB_RPTR_ADDR is left at 0, i.e. read-pointer writeback to memory
 * disabled, exactly as kdrive leaves it (ati_dma.c:214). Nothing here
 * wants it: the driver reads mmCP_RB_RPTR directly when it needs the
 * pointer, and enabling writeback would mean finding somewhere coherent
 * for the processor to write it to.
 *
 * Runs *before* the 2D engine's own quiescent setup and does not consult
 * @accel_ready, because at this point nothing has established it yet --
 * see w100_2d_setup() for the ordering and why it is that way round.
 * Leaving @ring_ready set on a chip whose 2D block never answers is
 * harmless: w100_2d_fill()/w100_2d_copy() check @accel_ready before they
 * look at the transport at all.
 */
static void w100_cp_ring_init_locked(struct w100_device *w100)
{
	u32 ring_addr = W100_FB_BASE + W100_CP_RING_VRAM_OFFSET;
	const struct firmware *fw;

	w100_cp_ring_fini_locked(w100);

	if (!w100->wrap)
		return;

	fw = w100_cp_get_firmware(w100);
	if (!fw)
		return;

	w100_cp_upload_microcode(w100, fw);
	w100_2d_engine_reset(w100);

	w100->ring_wptr = 0;
	w100->ring_free = W100_CP_RING_DWORDS;

	writel(ring_addr, w100->regs + W100_REG_CP_RB_BASE);
	writel(0, w100->regs + W100_REG_CP_RB_WPTR);
	writel(0, w100->regs + W100_REG_CP_RB_RPTR);
	writel(0, w100->regs + W100_REG_CP_RB_RPTR_ADDR);

	writel(FIELD_PREP(W100_WRAP_BUF_A_OFFSET_ADDR, ring_addr) |
	       FIELD_PREP(W100_WRAP_BUF_A_BLOCK_SIZE, W100_CP_WRAP_BLOCK_SIZE),
	       w100->regs + W100_REG_WRAP_BUF_A);

	writel(W100_CP_RB_CNTL_NO_UPDATE |
	       FIELD_PREP(W100_CP_RB_CNTL_BUFSZ, W100_CP_RB_BUFSZ),
	       w100->regs + W100_REG_CP_RB_CNTL);

	writel(FIELD_PREP(W100_CP_CSQ_CNTL_MODE, W100_CP_CSQ_MODE_FREERUN),
	       w100->regs + W100_REG_CP_CSQ_CNTL);

	w100->ring_ready = true;
}

/*
 * A read pointer that stops moving is the one failure this transport
 * has, and it arrives at both of the places that wait on it. There is
 * nothing to diagnose it with -- no completion interrupt, no error
 * status register anywhere in kdrive's map, and no documentation of what
 * the microcode does when it faults -- so the response is the same in
 * both places: stop using the processor, loudly, and let the caller
 * reissue on the transport that two independent drivers attest.
 *
 * Returns -EAGAIN, which the driver already uses for "the engine was
 * reset underneath you, reissue this" on the polled path.
 */
static int w100_cp_ring_wedged(struct w100_device *w100, const char *why, u32 rptr)
{
	drm_err(&w100->drm,
		"command processor: %s (rptr %u, wptr %u); falling back to polled register writes\n",
		why, rptr, w100->ring_wptr);

	/*
	 * Cheap diagnostic for a dead ring, available now regardless of any
	 * PM work: mmCP_RB_RPTR stuck at 0 while the driver's cached write
	 * pointer is nonzero means the processor never advanced past its
	 * post-reset state at all, rather than merely stalling mid-stream --
	 * the shape a live suspend/resume would leave behind, since nothing
	 * in this driver currently saves or reloads the ring base/microcode
	 * across a real suspend (there are no PM ops here; see the comment
	 * at the top of this file). This does not fix or work around
	 * anything, it only tells the two shapes apart in the log.
	 */
	if (rptr == 0 && w100->ring_wptr != 0)
		drm_err(&w100->drm,
			"command processor: rptr stuck at 0 with wptr %u -- ring/microcode appear dead, not merely stalled\n",
			w100->ring_wptr);

	/*
	 * The teardown resets the 2D block, which clears the source and
	 * default scissors the engine's quiescent state establishes and
	 * that no individual operation reprogrammes. Re-establish them, or
	 * the polled path this just fell back to inherits an engine that
	 * clips everything away -- the same reason kdrive follows every
	 * ATIEngineReset() with an ATIDrawSetup(), and the same thing
	 * w100_2d_fifo_wait_recover() does on its own reset path.
	 */
	w100_cp_ring_fini_locked(w100);
	w100_2d_setup_locked(w100, w100->hw_mode);

	return -EAGAIN;
}

/*
 * ATIRequestEntries(), ati_dma.c:297-323: wait until @dwords will fit,
 * refreshing the cached free count from the processor's read pointer.
 *
 * The strict inequality is kdrive's and is required rather than an
 * off-by-one: the ring is empty when the two pointers are equal, so
 * letting the write pointer catch the read pointer from behind would
 * make a completely full ring indistinguishable from an empty one. One
 * dword is always left unused.
 *
 * kdrive re-reads mmCP_RB_WPTR here too. That register only ever holds
 * what this driver last wrote into it, and every read crosses the
 * static-memory bus, so the shadow copy is used instead.
 */
static int w100_cp_ring_space(struct w100_device *w100, unsigned int dwords)
{
	ktime_t timeout;
	u32 rptr;

	if (WARN_ON(dwords >= W100_CP_RING_DWORDS))
		return -EINVAL;

	timeout = ktime_add_us(ktime_get(), W100_CP_RING_TIMEOUT_US);

	for (;;) {
		if (w100->ring_free > dwords)
			return 0;

		rptr = readl(w100->regs + W100_REG_CP_RB_RPTR) & W100_CP_RING_MASK;

		if (rptr == w100->ring_wptr)
			w100->ring_free = W100_CP_RING_DWORDS;
		else
			w100->ring_free = (W100_CP_RING_DWORDS + rptr -
					   w100->ring_wptr) & W100_CP_RING_MASK;

		if (w100->ring_free > dwords)
			return 0;

		if (ktime_after(ktime_get(), timeout))
			return w100_cp_ring_wedged(w100, "ring never freed space", rptr);

		cpu_relax();
	}
}

/*
 * Write one dword into the ring, through the wrap aperture.
 *
 * The index is masked on every dword, which is radeon's own idiom
 * (radeon_ring_write(), drivers/gpu/drm/radeon/radeon.h:2678-2687) and
 * lets a packet straddle the end of the ring, as it must in a genuinely
 * circular buffer.
 *
 * kdrive does not do this: ATIRequestEntries() hands out an *unmasked*
 * base pointer and its OUT_RING() macro then writes n consecutive dwords
 * from it (ati_dma.h:120-122), so a packet issued near the top of the
 * ring writes past the end of the 8 KiB buffer. It is possible the wrap
 * unit folds those writes back to the ring's start on its own -- that is
 * what the block is named for, and it would explain how kdrive got away
 * with it -- but no source anywhere documents the behaviour. Masking in
 * software is correct under either reading: if the hardware wraps, this
 * produces the same address it would have; if it does not, this is the
 * only thing preventing the overrun.
 */
static void w100_cp_ring_emit(struct w100_device *w100, u32 value)
{
	writel(value, w100->wrap + w100->ring_wptr * 4);
	w100->ring_wptr = (w100->ring_wptr + 1) & W100_CP_RING_MASK;
	w100->ring_free--;
}

/* A type-0 packet with a payload of one, i.e. a single register write --
 * kdrive's compiled OUT_REG() (ati_dma.h:100-104) exactly. Type-0 can
 * carry a burst of consecutive registers and doing so would halve the
 * dwords spent on setup, but the only count > 1 type-0 packet anywhere
 * in kdrive is in an #ifdef arm that its own build does not select, so
 * multi-register bursts are unattested on this processor and are not
 * used here.
 */
static void w100_cp_ring_emit_reg(struct w100_device *w100, u32 reg, u32 value)
{
	w100_cp_ring_emit(w100, W100_CCE_PACKET0_HDR(reg, 1));
	w100_cp_ring_emit(w100, value);
}

/*
 * Ring the doorbell: publish the write pointer. ATISubmitEntries(),
 * ati_dma.c:325-333.
 *
 * The read-back is r100_gfx_set_wptr()'s (r100.c:1120-1124), for the
 * same reason: it is the one point where the host has to know its
 * doorbell has actually reached the chip rather than sitting in a write
 * buffer somewhere between the two.
 */
static void w100_cp_ring_commit(struct w100_device *w100)
{
	writel(w100->ring_wptr, w100->regs + W100_REG_CP_RB_WPTR);
	readl(w100->regs + W100_REG_CP_RB_WPTR);
}

/*
 * Completion, in the only two forms this hardware offers.
 *
 * First the processor has to have consumed everything submitted, which
 * is the read pointer catching the write pointer -- that is a real
 * signal about the *queue*, and it is the one thing the polled register
 * path could never tell anybody. Then the GFX block has to have finished
 * the last packet it was handed, which is the same mmRBBM_STATUS
 * gui_active poll the polled path already ends every operation with
 * (ATIWaitIdle(), ati_dma.c:139-172, polls exactly this after its own
 * mmCP_RB_RPTR read).
 */
static int w100_cp_ring_drain(struct w100_device *w100)
{
	u32 rptr;
	int ret;

	/*
	 * Polled at 10 us rather than the 1 us the engine's own waits use.
	 * This one is waiting on however much work was submitted, not on a
	 * single register burst, so it is bounded by kdrive's longer ring
	 * timeout -- and at that bound a 1 us interval would be a million
	 * iterations of a read across the static-memory bus for no
	 * added resolution.
	 */
	ret = readl_poll_timeout_atomic(w100->regs + W100_REG_CP_RB_RPTR, rptr,
					(rptr & W100_CP_RING_MASK) == w100->ring_wptr,
					10, W100_CP_RING_TIMEOUT_US);
	if (ret)
		return w100_cp_ring_wedged(w100, "ring never drained",
					   rptr & W100_CP_RING_MASK);

	w100->ring_free = W100_CP_RING_DWORDS;

	return w100_2d_wait_idle(w100);
}

/*
 * kdrive's recovery, ati_dma.c:130-134: on a FIFO timeout, reset the
 * engine and re-establish its static state, then retry the wait once.
 *
 * Two things differ from kdrive, both because kdrive could get away with
 * what it did and this cannot. It resumes the interrupted operation as
 * if nothing had happened; the reset has just wiped every operand
 * register the caller queued before the timeout, so what would resume is
 * a half-programmed blit against the freshly re-established defaults.
 * This fails the operation instead, with -EAGAIN once the engine is
 * answering again so a caller can tell "reset, reissue me" from "this
 * chip is not responding". And it reports the second failure at all,
 * where kdrive only prints: a wedged 2D engine on a device with no
 * interrupt is otherwise completely invisible.
 */
static int w100_2d_fifo_wait_recover(struct w100_device *w100, unsigned int entries)
{
	int ret;

	if (!w100_2d_fifo_wait(w100, entries))
		return 0;

	drm_err(&w100->drm, "2D engine: resetting\n");
	w100_2d_engine_reset(w100);
	w100_2d_setup_locked(w100, w100->hw_mode);

	ret = w100_2d_fifo_wait(w100, entries);

	return ret ? : -EAGAIN;
}

/*
 * w100_init_graphic_engine() (w100fb.c:294-357) -- the engine's
 * quiescent state, re-established on every mode apply exactly as the
 * recovered driver re-ran it from every w100fb_activate_var().
 * mmDEFAULT_SC_BOTTOM_RIGHT is the one addition, from kdrive's
 * ATIDrawSetup() (ati_draw.c:120-129); w100fb.c leaves that register at
 * whatever reset left it, and it is what the engine clips against
 * whenever gmc_default_sel selects the default scissor instead of the
 * mmSC_* pair.
 *
 * Every per-operation register written here is overwritten by
 * w100_2d_fill()/w100_2d_copy() before use. What actually has to
 * survive from here is the source scissor and the default scissor.
 */
static void w100_2d_setup_locked(struct w100_device *w100,
				 const struct w100_hw_mode *hw)
{
	u32 gmc, dp_cntl, dp_datatype, dp_mix;
	int ret;

	ret = w100_2d_fifo_wait(w100, 4);
	writel(W100_FB_BASE, w100->regs + W100_REG_DST_OFFSET);
	writel(w100_2d_pitch_reg(hw->xres * 2), w100->regs + W100_REG_DST_PITCH);
	writel(W100_FB_BASE, w100->regs + W100_REG_SRC_OFFSET);
	writel(w100_2d_pitch_reg(hw->xres * 2), w100->regs + W100_REG_SRC_PITCH);

	/*
	 * Both scissor sets are programmed wide open here because the two
	 * sources disagree about which one gmc_default_sel == 0 actually
	 * selects, and neither says. w100fb.c writes the mmSC_* pair (to
	 * the visible size) and never touches mmDEFAULT_SC_BOTTOM_RIGHT;
	 * kdrive writes mmDEFAULT_SC_BOTTOM_RIGHT (to W100_MAXINT) and
	 * never touches the mmSC_* pair -- and both drivers worked. So the
	 * safe reading is that one of the two is live and the other is
	 * inert, without knowing which. Programming both, as here and
	 * per-operation in w100_2d_fill()/w100_2d_copy(), is correct under
	 * either reading; the software bounds check in
	 * w100_2d_rect_valid() is what the driver actually relies on.
	 */
	ret |= w100_2d_fifo_wait(w100, 4);
	writel(0, w100->regs + W100_REG_SC_TOP_LEFT);
	writel(W100_2D_YX(hw->xres, hw->yres), w100->regs + W100_REG_SC_BOTTOM_RIGHT);
	writel(W100_2D_YX(W100_2D_MAXINT, W100_2D_MAXINT),
	       w100->regs + W100_REG_SRC_SC_BOTTOM_RIGHT);
	writel(W100_2D_YX(W100_2D_MAXINT, W100_2D_MAXINT),
	       w100->regs + W100_REG_DEFAULT_SC_BOTTOM_RIGHT);

	dp_cntl = W100_DP_CNTL_DST_X_DIR | W100_DP_CNTL_DST_Y_DIR |
		  W100_DP_CNTL_SRC_X_DIR | W100_DP_CNTL_SRC_Y_DIR |
		  W100_DP_CNTL_DST_MAJOR_X | W100_DP_CNTL_SRC_MAJOR_X;

	gmc = W100_GMC_SRC_PITCH_OFFSET_CNTL | W100_GMC_DST_PITCH_OFFSET_CNTL |
	      W100_GMC_SRC_CLIPPING | W100_GMC_DST_CLIPPING |
	      FIELD_PREP(W100_GMC_BRUSH_DATATYPE, W100_BRUSH_NONE) |
	      FIELD_PREP(W100_GMC_DST_DATATYPE, W100_DATATYPE_16BPP) |
	      FIELD_PREP(W100_GMC_SRC_DATATYPE, W100_DATATYPE_16BPP) |
	      W100_GMC_BYTE_PIX_ORDER |
	      FIELD_PREP(W100_GMC_ROP3, w100_blt_rop3[W100_ROP_COPY]) |
	      FIELD_PREP(W100_GMC_DP_SRC_SOURCE, W100_DP_SRC_MEM_RECTANGULAR) |
	      W100_GMC_CLR_CMP_FCN_DIS | W100_GMC_WR_MSK_DIS |
	      FIELD_PREP(W100_GMC_DP_OP, W100_DP_OP_ROP);

	dp_datatype = FIELD_PREP(W100_DP_DATATYPE_DST, W100_DATATYPE_16BPP) |
		      FIELD_PREP(W100_DP_DATATYPE_BRUSH, W100_BRUSH_NONE) |
		      FIELD_PREP(W100_DP_DATATYPE_SRC2, W100_DATATYPE_16BPP) |
		      FIELD_PREP(W100_DP_DATATYPE_SRC, W100_DATATYPE_16BPP) |
		      W100_DP_DATATYPE_BYTE_PIX_ORDER;

	dp_mix = FIELD_PREP(W100_DP_MIX_SRC_SOURCE, W100_DP_SRC_MEM_RECTANGULAR) |
		 FIELD_PREP(W100_DP_MIX_SRC2_SOURCE, 1) |
		 FIELD_PREP(W100_DP_MIX_ROP3, w100_blt_rop3[W100_ROP_COPY]) |
		 FIELD_PREP(W100_DP_MIX_OP, W100_DP_OP_ROP);

	ret |= w100_2d_fifo_wait(w100, 4);
	writel(dp_cntl, w100->regs + W100_REG_DP_CNTL);
	writel(gmc, w100->regs + W100_REG_DP_GUI_MASTER_CNTL);
	writel(dp_datatype, w100->regs + W100_REG_DP_DATATYPE);
	writel(dp_mix, w100->regs + W100_REG_DP_MIX);

	w100->hw_mode = hw;

	/*
	 * A chip whose command FIFO never reports a free slot is one whose
	 * engine is not there. Leaving accel_ready clear in that case is
	 * what stops every later operation paying three 200 ms timeouts to
	 * rediscover it; the fill callers fall back to the CPU instead.
	 */
	w100->accel_ready = !ret;
}

/*
 * Bring up the command processor, then the engine's quiescent state --
 * in that order, which is kdrive's own ATIDrawEnable(): ATIDMASetup()
 * followed by ATIDrawSetup() (ati_draw.c:444-450).
 *
 * The order matters and is not a style choice. Bringing the processor up
 * resets the 2D block (ATIDMAInit()'s ATIEngineReset(), ati_dma.c:204),
 * and that reset clears exactly the scissor state the engine's quiescent
 * setup exists to establish -- which is why kdrive's own FIFO-timeout
 * recovery re-runs ATIDrawSetup() after every ATIEngineReset(), and why
 * w100_2d_fifo_wait_recover() does the same here. Setting the engine up
 * first and the ring second would throw the setup away.
 *
 * Both are redone from scratch on every mode apply rather than carried
 * across one. It is cheap -- one microcode upload from an already-cached
 * firmware image plus seven register writes -- and this driver's
 * .atomic_enable replays the whole chip programming sequence on every
 * enable anyway.
 *
 * kdrive's ATIDrawSetup() submits its two registers *through* the ring,
 * because its OUT_REG() macro compiles to a type-0 packet in a DMA
 * build. w100_2d_setup_locked() writes them by MMIO instead. The two
 * reach the same register file and the ring is empty at this point, so
 * nothing is racing; a plain register write is simply the smaller thing
 * to do when there is no operation to batch it with.
 */
static void w100_2d_setup(struct w100_device *w100, const struct w100_hw_mode *hw)
{
	mutex_lock(&w100->accel_lock);
	w100_cp_ring_init_locked(w100);
	w100_2d_setup_locked(w100, hw);
	mutex_unlock(&w100->accel_lock);
}

static bool w100_2d_surface_valid(struct w100_device *w100,
				  const struct w100_2d_surface *s)
{
	u32 last_byte;

	if (!s->width || !s->height ||
	    s->width > W100_2D_MAXINT || s->height > W100_2D_MAXINT)
		return false;

	/* Odd pitches cannot be expressed at all, see w100_2d_pitch_reg(). */
	if ((s->pitch & 1) || s->pitch < (u32)s->width * 2 ||
	    !FIELD_FIT(W100_2D_PITCH, s->pitch >> 1))
		return false;

	if (s->offset < W100_FB_BASE || !FIELD_FIT(W100_2D_OFFSET, s->offset))
		return false;

	last_byte = s->offset - W100_FB_BASE +
		    s->pitch * (u32)(s->height - 1) + (u32)s->width * 2;

	return last_byte <= w100->vram_size;
}

static bool w100_2d_rect_valid(const struct w100_2d_surface *s,
			       int x, int y, int width, int height)
{
	return width > 0 && height > 0 && x >= 0 && y >= 0 &&
	       x + width <= s->width && y + height <= s->height;
}

/*
 * Solid fill, in both transports: ATIPrepareSolid() + ATISolid()
 * (ati_draw.c:210-289), collapsed into one operation per transport
 * because no consumer here issues more than one rectangle per setup.
 *
 * @color is a raw destination-format pixel, i.e. RGB565 on this driver
 * -- see W100_DATATYPE_16BPP's comment for why the datatype's name says
 * otherwise and why that does not matter.
 *
 * Both transports return once the engine is idle again. Every source
 * for this hardware instead left the operation in flight and
 * synchronised lazily (w100fb_sync(), ATIWaitMarker()), which is the
 * right shape for a batching consumer; there is none here, and every
 * caller in this driver either reads the result back or hands the
 * memory straight to a CPU memcpy, so paying the wait up front is both
 * simpler and what the callers would have to do anyway.
 *
 * That is worth being clear about, because it bounds what the ring
 * transport can be worth: submitting asynchronously and then
 * immediately waiting does not overlap the CPU with the engine. What it
 * does remove is the polled path's three mmRBBM_STATUS FIFO waits per
 * operation, replacing them with one contiguous burst of ring writes.
 */
static int w100_2d_fill_mmio_locked(struct w100_device *w100,
				    const struct w100_2d_surface *dst,
				    int x, int y, int width, int height,
				    u16 color, u32 gmc)
{
	int ret;

	ret = w100_2d_fifo_wait_recover(w100, 6);
	if (ret)
		return ret;

	/* Belt and braces: w100_2d_rect_valid() has already rejected a
	 * rectangle that leaves the surface, and gmc_dst_clipping above
	 * makes the engine enforce the same bound in hardware.
	 */
	writel(0, w100->regs + W100_REG_SC_TOP_LEFT);
	writel(W100_2D_YX(dst->width, dst->height), w100->regs + W100_REG_SC_BOTTOM_RIGHT);
	writel(w100_2d_pitch_reg(dst->pitch), w100->regs + W100_REG_DST_PITCH);
	writel(dst->offset, w100->regs + W100_REG_DST_OFFSET);
	writel(gmc, w100->regs + W100_REG_DP_GUI_MASTER_CNTL);
	writel(color, w100->regs + W100_REG_DP_BRUSH_FRGD_CLR);

	ret = w100_2d_fifo_wait_recover(w100, 4);
	if (ret)
		return ret;

	/* Every plane writable. kdrive puts the client's own plane mask
	 * here (ATIPrepareSolid()'s pm argument); nothing in this driver
	 * has a use for a partial one.
	 */
	writel(~0u, w100->regs + W100_REG_DP_WRITE_MSK);
	writel(W100_DP_CNTL_DST_X_DIR | W100_DP_CNTL_DST_Y_DIR,
	       w100->regs + W100_REG_DP_CNTL);
	writel(W100_2D_YX(x, y), w100->regs + W100_REG_DST_Y_X);
	/* Writing the extent is what launches the operation. */
	writel(W100_2D_WH(width, height), w100->regs + W100_REG_DST_WIDTH_HEIGHT);

	return w100_2d_wait_idle(w100);
}

/*
 * The same fill as a ring submission: ATIPrepareSolid()'s register set
 * as type-0 packets, then ATISolid()'s PAINT_MULTI (ati_draw.c:268-272).
 *
 * Two differences from the polled path, both forced by the packet:
 *
 * The rectangle goes in as x-major -- (x << 16) | y, then
 * (width << 16) | height -- where every geometry *register* on this chip
 * is y-major. That is not a transcription slip: the polled path writes
 * mmDST_Y_X and mmDST_WIDTH_HEIGHT, and the packet's payload is a
 * distinct encoding the microcode unpacks, so the two orders are simply
 * different interfaces to the same operation. kdrive has it exactly this
 * way round in the two arms of one #ifdef, which is as direct a
 * confirmation as this source can give.
 *
 * And there is no separate "launch" write. The polled path starts the
 * operation by writing the extent register last; here the packet is the
 * operation, and it starts when the processor reaches it.
 */
static int w100_cp_fill_submit_locked(struct w100_device *w100,
				      const struct w100_2d_surface *dst,
				      int x, int y, int width, int height,
				      u16 color, u32 gmc)
{
	int ret;

	/* Eight single-register type-0 packets plus a three-dword type-3. */
	ret = w100_cp_ring_space(w100, 8 * 2 + 3);
	if (ret)
		return ret;

	w100_cp_ring_emit_reg(w100, W100_REG_SC_TOP_LEFT, 0);
	w100_cp_ring_emit_reg(w100, W100_REG_SC_BOTTOM_RIGHT,
			      W100_2D_YX(dst->width, dst->height));
	w100_cp_ring_emit_reg(w100, W100_REG_DST_PITCH, w100_2d_pitch_reg(dst->pitch));
	w100_cp_ring_emit_reg(w100, W100_REG_DST_OFFSET, dst->offset);
	w100_cp_ring_emit_reg(w100, W100_REG_DP_GUI_MASTER_CNTL, gmc);
	w100_cp_ring_emit_reg(w100, W100_REG_DP_BRUSH_FRGD_CLR, color);
	w100_cp_ring_emit_reg(w100, W100_REG_DP_WRITE_MSK, ~0u);
	w100_cp_ring_emit_reg(w100, W100_REG_DP_CNTL,
			      W100_DP_CNTL_DST_X_DIR | W100_DP_CNTL_DST_Y_DIR);

	w100_cp_ring_emit(w100, W100_CCE_PACKET3_HDR(W100_CCE_PACKET3_PAINT_MULTI, 2));
	w100_cp_ring_emit(w100, (u32)x << 16 | y);
	w100_cp_ring_emit(w100, (u32)width << 16 | height);

	w100_cp_ring_commit(w100);

	return 0;
}

/*
 * Submit and wait. The split above exists because submitting several
 * operations and waiting once is the only thing this transport can do
 * that the polled one cannot. Every caller in this driver wants the
 * result before it continues, so this wrapper always waits immediately.
 */
static int w100_cp_fill_locked(struct w100_device *w100,
			       const struct w100_2d_surface *dst,
			       int x, int y, int width, int height,
			       u16 color, u32 gmc)
{
	int ret;

	ret = w100_cp_fill_submit_locked(w100, dst, x, y, width, height, color, gmc);
	if (ret)
		return ret;

	return w100_cp_ring_drain(w100);
}

/* ATIPrepareSolid()'s operation descriptor, ati_draw.c:229-240. Shared
 * by both transports.
 */
static u32 w100_2d_fill_gmc(enum w100_rop rop)
{
	return W100_GMC_DST_PITCH_OFFSET_CNTL | W100_GMC_DST_CLIPPING |
	       FIELD_PREP(W100_GMC_BRUSH_DATATYPE, W100_BRUSH_SOLIDCOLOR) |
	       FIELD_PREP(W100_GMC_DST_DATATYPE, W100_DATATYPE_16BPP) |
	       FIELD_PREP(W100_GMC_SRC_DATATYPE, W100_DATATYPE_16BPP) |
	       W100_GMC_BYTE_PIX_ORDER |
	       FIELD_PREP(W100_GMC_ROP3, w100_solid_rop3[rop]) |
	       FIELD_PREP(W100_GMC_DP_SRC_SOURCE, W100_DP_SRC_MEM_RECTANGULAR) |
	       W100_GMC_CLR_CMP_FCN_DIS |
	       FIELD_PREP(W100_GMC_DP_OP, W100_DP_OP_ROP);
}

static int w100_2d_fill(struct w100_device *w100, const struct w100_2d_surface *dst,
			int x, int y, int width, int height,
			u16 color, enum w100_rop rop)
{
	u32 gmc;
	int ret;

	if (WARN_ON(rop >= W100_ROP_COUNT))
		return -EINVAL;
	if (!w100_2d_surface_valid(w100, dst) ||
	    !w100_2d_rect_valid(dst, x, y, width, height))
		return -EINVAL;

	gmc = w100_2d_fill_gmc(rop);

	mutex_lock(&w100->accel_lock);

	if (!w100->accel_ready)
		ret = -ENODEV;
	else if (w100_cp_active(w100))
		ret = w100_cp_fill_locked(w100, dst, x, y, width, height, color, gmc);
	else
		ret = w100_2d_fill_mmio_locked(w100, dst, x, y, width, height, color, gmc);

	mutex_unlock(&w100->accel_lock);

	return ret;
}

/*
 * Screen-to-screen copy, in both transports: ATIPrepareCopy() +
 * ATICopy() (ati_draw.c:291-403), same collapse as the fill above.
 *
 * Deciding the copy direction is this driver's own, because kdrive's
 * caller (kaa) passed it in and there is no equivalent caller here. The
 * rule is memmove's: step away from the destination, i.e. run backwards
 * along an axis whenever the destination coordinate is greater than the
 * source's. What each transport then *does* with that decision differs,
 * and kdrive's own #ifdef is the authority for the difference -- see
 * each of the two functions below.
 *
 * This is only meaningful when @src and @dst name the same surface. Two
 * *different* surfaces that happen to overlap in chip memory cannot be
 * resolved from coordinates at all, and no caller here does that.
 *
 * __maybe_unused: nothing calls this yet. A screen-to-screen copy needs
 * both operands in chip memory, and the framebuffer here is a shadow
 * plane in system memory, so there is nothing in the scanout path to
 * copy between.
 */
static int w100_2d_copy_mmio_locked(struct w100_device *w100,
				    const struct w100_2d_surface *dst,
				    int dst_x, int dst_y,
				    const struct w100_2d_surface *src,
				    int src_x, int src_y,
				    int width, int height,
				    u32 gmc, u32 dp_cntl, u32 dp_datatype,
				    bool x_forward, bool y_forward)
{
	int ret;

	/*
	 * Both rectangles start on their far edge whenever the engine has
	 * to run backwards along that axis -- ati_draw.c:371-380, which
	 * applies exactly this fixup and only on its non-PACKET3 path.
	 * The ring path below does not do it; see there.
	 */
	if (!x_forward) {
		src_x += width - 1;
		dst_x += width - 1;
	}
	if (!y_forward) {
		src_y += height - 1;
		dst_y += height - 1;
	}

	ret = w100_2d_fifo_wait_recover(w100, 5);
	if (ret)
		return ret;

	/* The source scissor stays at w100_2d_setup_locked()'s wide-open
	 * value, which is what both sources leave it at; the source
	 * rectangle is already bounded by w100_2d_rect_valid().
	 */
	writel(0, w100->regs + W100_REG_SC_TOP_LEFT);
	writel(W100_2D_YX(dst->width, dst->height), w100->regs + W100_REG_SC_BOTTOM_RIGHT);
	writel(w100_2d_pitch_reg(src->pitch), w100->regs + W100_REG_SRC_PITCH);
	writel(src->offset, w100->regs + W100_REG_SRC_OFFSET);
	writel(w100_2d_pitch_reg(dst->pitch), w100->regs + W100_REG_DST_PITCH);

	ret = w100_2d_fifo_wait_recover(w100, 4);
	if (ret)
		return ret;

	writel(dst->offset, w100->regs + W100_REG_DST_OFFSET);
	writel(gmc, w100->regs + W100_REG_DP_GUI_MASTER_CNTL);
	writel(~0u, w100->regs + W100_REG_DP_WRITE_MSK);
	writel(dp_cntl, w100->regs + W100_REG_DP_CNTL);

	ret = w100_2d_fifo_wait_recover(w100, 4);
	if (ret)
		return ret;

	writel(dp_datatype, w100->regs + W100_REG_DP_DATATYPE);
	writel(W100_2D_YX(src_x, src_y), w100->regs + W100_REG_SRC_Y_X);
	writel(W100_2D_YX(dst_x, dst_y), w100->regs + W100_REG_DST_Y_X);
	/* Writing the extent is what launches the operation. */
	writel(W100_2D_WH(width, height), w100->regs + W100_REG_DST_WIDTH_HEIGHT);

	return w100_2d_wait_idle(w100);
}

/*
 * The same copy as a ring submission: ATIPrepareCopy()'s register set as
 * type-0 packets, then ATICopy()'s BITBLT_MULTI (ati_draw.c:383-388).
 *
 * The coordinates handed to the packet are the rectangles' true top-left
 * corners, *not* the far-edge-adjusted ones the polled path above uses.
 * kdrive draws exactly this distinction -- its far-edge fixup sits under
 * `#ifndef DRAW_USING_PACKET3` (ati_draw.c:371-380) and is compiled out
 * of the packet path -- so the MULTI opcode evidently derives the
 * traversal order itself from mmDP_CNTL's direction bits, which are
 * submitted here in the same packet stream. Getting this wrong in either
 * direction corrupts overlapping copies only.
 */
static int w100_cp_copy_locked(struct w100_device *w100,
			       const struct w100_2d_surface *dst,
			       int dst_x, int dst_y,
			       const struct w100_2d_surface *src,
			       int src_x, int src_y,
			       int width, int height,
			       u32 gmc, u32 dp_cntl, u32 dp_datatype)
{
	int ret;

	/* Ten single-register type-0 packets plus a four-dword type-3. */
	ret = w100_cp_ring_space(w100, 10 * 2 + 4);
	if (ret)
		return ret;

	w100_cp_ring_emit_reg(w100, W100_REG_SC_TOP_LEFT, 0);
	w100_cp_ring_emit_reg(w100, W100_REG_SC_BOTTOM_RIGHT,
			      W100_2D_YX(dst->width, dst->height));
	w100_cp_ring_emit_reg(w100, W100_REG_SRC_PITCH, w100_2d_pitch_reg(src->pitch));
	w100_cp_ring_emit_reg(w100, W100_REG_SRC_OFFSET, src->offset);
	w100_cp_ring_emit_reg(w100, W100_REG_DST_PITCH, w100_2d_pitch_reg(dst->pitch));
	w100_cp_ring_emit_reg(w100, W100_REG_DST_OFFSET, dst->offset);
	w100_cp_ring_emit_reg(w100, W100_REG_DP_GUI_MASTER_CNTL, gmc);
	w100_cp_ring_emit_reg(w100, W100_REG_DP_WRITE_MSK, ~0u);
	w100_cp_ring_emit_reg(w100, W100_REG_DP_CNTL, dp_cntl);
	w100_cp_ring_emit_reg(w100, W100_REG_DP_DATATYPE, dp_datatype);

	w100_cp_ring_emit(w100, W100_CCE_PACKET3_HDR(W100_CCE_PACKET3_BITBLT_MULTI, 3));
	w100_cp_ring_emit(w100, (u32)src_x << 16 | src_y);
	w100_cp_ring_emit(w100, (u32)dst_x << 16 | dst_y);
	w100_cp_ring_emit(w100, (u32)width << 16 | height);

	w100_cp_ring_commit(w100);

	return w100_cp_ring_drain(w100);
}

static int __maybe_unused w100_2d_copy(struct w100_device *w100,
				       const struct w100_2d_surface *dst,
				       int dst_x, int dst_y,
				       const struct w100_2d_surface *src,
				       int src_x, int src_y,
				       int width, int height,
				       enum w100_rop rop)
{
	bool x_forward, y_forward;
	u32 gmc, dp_cntl, dp_datatype;
	int ret;

	if (WARN_ON(rop >= W100_ROP_COUNT))
		return -EINVAL;
	if (!w100_2d_surface_valid(w100, dst) || !w100_2d_surface_valid(w100, src) ||
	    !w100_2d_rect_valid(dst, dst_x, dst_y, width, height) ||
	    !w100_2d_rect_valid(src, src_x, src_y, width, height))
		return -EINVAL;

	x_forward = dst_x <= src_x;
	y_forward = dst_y <= src_y;

	gmc = W100_GMC_SRC_PITCH_OFFSET_CNTL | W100_GMC_DST_PITCH_OFFSET_CNTL |
	      W100_GMC_SRC_CLIPPING | W100_GMC_DST_CLIPPING |
	      FIELD_PREP(W100_GMC_BRUSH_DATATYPE, W100_BRUSH_NONE) |
	      FIELD_PREP(W100_GMC_DST_DATATYPE, W100_DATATYPE_16BPP) |
	      FIELD_PREP(W100_GMC_SRC_DATATYPE, W100_DATATYPE_16BPP) |
	      W100_GMC_BYTE_PIX_ORDER |
	      FIELD_PREP(W100_GMC_ROP3, w100_blt_rop3[rop]) |
	      FIELD_PREP(W100_GMC_DP_SRC_SOURCE, W100_DP_SRC_MEM_RECTANGULAR) |
	      W100_GMC_CLR_CMP_FCN_DIS |
	      FIELD_PREP(W100_GMC_DP_OP, W100_DP_OP_ROP);

	dp_cntl = (x_forward ? W100_DP_CNTL_DST_X_DIR : 0) |
		  (y_forward ? W100_DP_CNTL_DST_Y_DIR : 0);

	dp_datatype = FIELD_PREP(W100_DP_DATATYPE_DST, W100_DATATYPE_16BPP) |
		      FIELD_PREP(W100_DP_DATATYPE_SRC, W100_DATATYPE_16BPP) |
		      W100_DP_DATATYPE_BYTE_PIX_ORDER;

	mutex_lock(&w100->accel_lock);

	if (!w100->accel_ready)
		ret = -ENODEV;
	else if (w100_cp_active(w100))
		ret = w100_cp_copy_locked(w100, dst, dst_x, dst_y, src, src_x, src_y,
					  width, height, gmc, dp_cntl, dp_datatype);
	else
		ret = w100_2d_copy_mmio_locked(w100, dst, dst_x, dst_y, src, src_x, src_y,
					       width, height, gmc, dp_cntl, dp_datatype,
					       x_forward, y_forward);

	mutex_unlock(&w100->accel_lock);

	return ret;
}

/*
 * w100fb_clear_screen() (w100fb.c:193-196), which is a memset_io() over
 * the whole visible framebuffer, as a hardware fill instead: ~600 KiB
 * that never crosses the nCS2 static-memory bus.
 *
 * Unlike the recovered driver this runs *once*, on the first mode apply,
 * not on every one. w100fb.c could clear on every activate_var() because
 * fbdev had no notion of a client having already put a frame there;
 * DRM's commit tail does. drm_atomic_helper_commit_tail() is
 * commit_modeset_disables -> commit_planes -> commit_modeset_enables,
 * so on any enabling commit -- including a plain DPMS-on, where the
 * damage machinery may legitimately produce no plane update at all --
 * the primary plane's atomic_update has already uploaded the client's
 * frame by the time the CRTC's atomic_enable calls this. Clearing every
 * time would erase it, with nothing scheduled to put it back.
 *
 * The one-shot clear is still worth doing: chip SDRAM contents are
 * undefined until w100_setup_memory() has run its mode-register-set
 * training, so without this the panel's first light is whatever the
 * memory came up holding.
 *
 * Falls back to the recovered driver's own memset_io() if the engine
 * does not answer, so a chip whose GFX block never comes up is no worse
 * off than it was under w100fb.c.
 */
static void w100_2d_clear_vram(struct w100_device *w100, const struct w100_hw_mode *hw)
{
	const struct w100_2d_surface fb = {
		.offset	= W100_FB_BASE,
		.pitch	= hw->xres * 2,
		.width	= hw->xres,
		.height	= hw->yres,
	};

	if (w100->vram_cleared)
		return;

	if (w100_2d_fill(w100, &fb, 0, 0, hw->xres, hw->yres, 0, W100_ROP_CLEAR))
		memset_io(w100->vram, 0, (size_t)hw->xres * hw->yres * 2);

	w100->vram_cleared = true;
}

/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */

/* w100fb_activate_var() (w100fb.c:412-432), minus calc_hsync()/
 * tg->change() (both dropped, see the file comment).
 *
 * w100_init_graphic_engine() moves up from the tail of the recovered
 * sequence to just after the clocks: the 2D engine is a memory-side
 * block whose only dependencies are SCLK and the memory controller,
 * both already programmed by that point, and having it up before
 * w100_2d_clear_vram() runs is what lets that clear be a hardware fill
 * rather than a 600 KiB CPU memset across the nCS2 bus.
 */
static void w100_apply_mode(struct w100_device *w100, const struct w100_hw_mode *hw)
{
	w100_pwm_setup(w100);
	w100_setup_memory(w100);
	w100_init_clocks(w100, hw);
	w100_2d_setup(w100, hw);
	w100_2d_clear_vram(w100, hw);
	w100_wait_vline(w100);

	w100_update_disable(w100);
	w100_init_lcd(w100, hw);
	w100_set_dispregs(w100, hw);
	w100_update_enable(w100);
}

static const struct w100_hw_mode *w100_find_hw_mode(int hdisplay, int vdisplay)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(hx4700_w100_hw_modes); i++) {
		if (hx4700_w100_hw_modes[i].xres == hdisplay &&
		    hx4700_w100_hw_modes[i].yres == vdisplay)
			return &hx4700_w100_hw_modes[i];
	}

	return NULL;
}

/* ------------------------------------------------------------------ */
/* Panel power, ported verbatim (real GPIOs, real mdelay()s) from
 * sony_lcd_init()/sony_lcd_off() (hx4700.c:361-399).
 */

static void w100_panel_power_up(struct w100_device *w100)
{
	gpiod_set_value_cansleep(w100->sqn_gpio, 1);
	gpiod_set_value_cansleep(w100->lvdd_gpio, 0);
	gpiod_set_value_cansleep(w100->avdd_gpio, 0);
	gpiod_set_value_cansleep(w100->slin1_gpio, 0);
	gpiod_set_value_cansleep(w100->panel_reset_gpio, 1);	/* active low: assert */
	mdelay(10);
	gpiod_set_value_cansleep(w100->pc1_gpio, 0);
	gpiod_set_value_cansleep(w100->lvdd_gpio, 0);
	mdelay(20);

	gpiod_set_value_cansleep(w100->lvdd_gpio, 1);
	mdelay(5);
	gpiod_set_value_cansleep(w100->avdd_gpio, 1);

	/*
	 * hx4700.c:377 carries its own "FIXME: init w3220 registers here"
	 * at exactly this point -- the panel GPIO sequence and the chip's
	 * own register bring-up were never actually interleaved in the
	 * recovered board file either. This driver's chip bring-up
	 * (w100_chip_bringup()) and mode programming (w100_apply_mode())
	 * both run from w100_crtc_helper_atomic_enable(), around this
	 * call, in the same order the recovered probe/activate_var path
	 * used them in.
	 */

	mdelay(5);
	gpiod_set_value_cansleep(w100->slin1_gpio, 1);
	mdelay(10);
	gpiod_set_value_cansleep(w100->panel_reset_gpio, 0);	/* active low: release */
	mdelay(10);
	gpiod_set_value_cansleep(w100->pc1_gpio, 1);
	mdelay(10);
	gpiod_set_value_cansleep(w100->n2v7_gpio, 1);
}

/*
 * Stop the video and graphics fetch, tri-state the LCD data pads and stop
 * the pixel and system clocks, in the order the stock display driver's
 * display-off path uses (ace_ddi AhiDispState, mode 0). Without it the chip
 * keeps scanning out into a panel whose rails are about to drop, and the
 * panel shows its undriven state with the backlight possibly still lit.
 * Every wait is bounded: a chip that never reports the clocks stopped must
 * not hang the commit tail.
 */
static void w100_display_off(struct w100_device *w100)
{
	u32 val;

	w100_update_disable(w100);

	val = readl(w100->regs + W100_REG_VIDEO_CTRL);
	val &= ~(W100_VIDEO_CTRL_EN_VIDEO_REQ | W100_VIDEO_CTRL_EN_VIDEO_CRTC |
		 W100_VIDEO_CTRL_EN_GRAPHIC_REQ_VIDEO);
	writel(val, w100->regs + W100_REG_VIDEO_CTRL);

	val = readl(w100->regs + W100_REG_GRAPHIC_CTRL);
	val &= ~(W100_GRAPHIC_CTRL_EN_GRAPHIC_REQ | W100_GRAPHIC_CTRL_EN_GRAPHIC_CRTC);
	writel(val, w100->regs + W100_REG_GRAPHIC_CTRL);

	w100_update_enable(w100);

	w100_wait_vline(w100);
	usleep_range(16000, 17000);

	val = readl(w100->regs + W100_REG_GENLCD_CNTL1);
	writel(val & 0xffeeaffe, w100->regs + W100_REG_GENLCD_CNTL1);
	val = readl(w100->regs + W100_REG_GENLCD_CNTL2);
	writel(val & ~BIT(0), w100->regs + W100_REG_GENLCD_CNTL2);
	val = readl(w100->regs + W100_REG_LCDD_CNTL2);
	writel(val & 0xfffc0000, w100->regs + W100_REG_LCDD_CNTL2);

	val = readl(w100->regs + W100_REG_GRAPHIC_CTRL);
	writel(val & ~W100_GRAPHIC_CTRL_LCD_PCLK_ON, w100->regs + W100_REG_GRAPHIC_CTRL);
	if (readl_poll_timeout(w100->regs + W100_REG_GRAPHIC_CTRL, val,
			       !(val & W100_GRAPHIC_CTRL_PCLK_RUNNING), 100, 20000))
		drm_dbg(&w100->drm, "PCLK did not report stopped\n");

	val = readl(w100->regs + W100_REG_GRAPHIC_CTRL);
	writel(val & ~W100_GRAPHIC_CTRL_LCD_SCLK_ON, w100->regs + W100_REG_GRAPHIC_CTRL);
	if (readl_poll_timeout(w100->regs + W100_REG_GRAPHIC_CTRL, val,
			       !(val & W100_GRAPHIC_CTRL_SCLK_RUNNING), 100, 20000))
		drm_dbg(&w100->drm, "SCLK did not report stopped\n");
}

/*
 * Panel power-down in the stock order (ahioem AhiOEMSetProperty, with the
 * chip's own display-off between the two halves): SQN and PC1 first, then
 * the chip stops scanning, then reset and N2V7, SLIN1 and AVDD, LVDD last.
 */
static void w100_panel_power_down(struct w100_device *w100)
{
	gpiod_set_value_cansleep(w100->sqn_gpio, 0);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(w100->pc1_gpio, 0);
	msleep(50);

	w100_display_off(w100);

	gpiod_set_value_cansleep(w100->panel_reset_gpio, 1);	/* active low: assert */
	gpiod_set_value_cansleep(w100->n2v7_gpio, 0);
	msleep(35);
	gpiod_set_value_cansleep(w100->slin1_gpio, 0);
	gpiod_set_value_cansleep(w100->avdd_gpio, 0);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(w100->lvdd_gpio, 0);
}

/* ------------------------------------------------------------------ */
/* Shadow-plane atomic pipeline, structurally identical to mq11xx.c's own
 * (see that file's own comment for the cirrus-qemu.c/sharp-memory.c
 * lineage this shape follows).
 */

static const u32 w100_primary_plane_formats[] = {
	DRM_FORMAT_RGB565,
};

static const u64 w100_primary_plane_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

static int w100_primary_plane_helper_atomic_check(struct drm_plane *plane,
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

/*
 * Smallest damage rectangle worth handing to the engine. A polled fill
 * is about a dozen register writes plus three FIFO/idle status reads,
 * i.e. some thirty bus cycles before the engine even starts,
 * whereas the CPU copy it replaces costs one bus cycle per pixel. 256
 * pixels keeps the engine path well clear of that break-even point and
 * spares the shadow scan for cursor-sized updates; it is a margin, not
 * a measured crossover.
 */
#define W100_FILL_MIN_PIXELS	256

/*
 * Whether every pixel of @clip in the shadow buffer is @*color. The
 * shadow is cached system memory, so this costs a small fraction of the
 * uncached bus writes it can save, and real content almost always
 * differs within the first few pixels, which is where it stops looking.
 */
static bool w100_shadow_rect_uniform(const struct iosys_map *shadow,
				     const struct drm_framebuffer *fb,
				     const struct drm_rect *clip, u16 *color)
{
	unsigned int width = drm_rect_width(clip);
	unsigned int height = drm_rect_height(clip);
	unsigned int pitch = fb->pitches[0];
	const u8 *first;
	const u16 *px;
	unsigned int x, y;

	if (shadow->is_iomem)
		return false;

	first = shadow->vaddr + drm_fb_clip_offset(pitch, fb->format, clip);
	px = (const u16 *)first;

	for (x = 1; x < width; x++)
		if (px[x] != px[0])
			return false;

	for (y = 1; y < height; y++)
		if (memcmp(first + y * pitch, first, width * 2))
			return false;

	*color = px[0];

	return true;
}

/*
 * Write one solid-colour damage rectangle with the 2D engine instead of
 * pushing it across the static-memory bus: a full 480x640 clear is
 * 600 KiB of uncached VLIO writes, and it is
 * what every letterbox bar, console clear and switch between the video
 * overlay and a software-rendered frame costs.
 *
 * Only on a commit that leaves the CRTC running without a modeset. The
 * default commit tail updates planes before .atomic_enable, so on an
 * enabling commit the engine state @accel_ready describes may predate a
 * chip that has since lost it; the CPU copy is always correct there.
 *
 * Ordering against the CPU copies of other rectangles in the same
 * update does not matter: every writer stores the shadow's own pixels,
 * so overlapping rectangles converge on the same content whichever
 * lands last, and the fill has finished before w100_2d_fill() returns.
 */
static bool w100_primary_fill_damage(struct w100_device *w100,
				     const struct drm_crtc_state *crtc_state,
				     const struct iosys_map *shadow,
				     const struct drm_framebuffer *fb,
				     const struct drm_rect *clip)
{
	const struct w100_2d_surface surface = {
		.offset	= W100_FB_BASE,
		.pitch	= fb->pitches[0],
		.width	= fb->width,
		.height	= fb->height,
	};
	u16 color;

	if (!crtc_state || !crtc_state->active ||
	    drm_atomic_crtc_needs_modeset(crtc_state))
		return false;

	/* A hint only: w100_2d_fill() rechecks it under @accel_lock. */
	if (!READ_ONCE(w100->accel_ready))
		return false;

	if (drm_rect_width(clip) * drm_rect_height(clip) < W100_FILL_MIN_PIXELS)
		return false;

	if (!w100_shadow_rect_uniform(shadow, fb, clip, &color))
		return false;

	return !w100_2d_fill(w100, &surface, clip->x1, clip->y1,
			     drm_rect_width(clip), drm_rect_height(clip),
			     color, W100_ROP_COPY);
}

static void w100_primary_plane_helper_atomic_update(struct drm_plane *plane,
						      struct drm_atomic_commit *state)
{
	struct w100_device *w100 = to_w100(plane->dev);
	struct drm_plane_state *plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_shadow_plane_state *shadow_plane_state = to_drm_shadow_plane_state(plane_state);
	struct drm_framebuffer *fb = plane_state->fb;
	struct drm_plane_state *old_plane_state = drm_atomic_get_old_plane_state(state, plane);
	struct iosys_map vram_map = IOSYS_MAP_INIT_VADDR_IOMEM(w100->vram);
	struct drm_crtc_state *crtc_state = NULL;
	struct drm_atomic_helper_damage_iter iter;
	struct drm_rect damage;
	int idx;

	if (!fb)
		return;

	if (!drm_dev_enter(&w100->drm, &idx))
		return;

	if (plane_state->crtc)
		crtc_state = drm_atomic_get_new_crtc_state(state, plane_state->crtc);

	drm_atomic_helper_damage_iter_init(&iter, old_plane_state, plane_state);
	drm_atomic_for_each_plane_damage(&iter, &damage) {
		unsigned int offset = drm_fb_clip_offset(fb->pitches[0], fb->format, &damage);
		struct iosys_map dst = IOSYS_MAP_INIT_OFFSET(&vram_map, offset);

		if (w100_primary_fill_damage(w100, crtc_state, &shadow_plane_state->data[0],
					     fb, &damage))
			continue;

		drm_fb_memcpy(&dst, fb->pitches, shadow_plane_state->data, fb, &damage);
	}

	drm_dev_exit(idx);
}

static const struct drm_plane_helper_funcs w100_primary_plane_helper_funcs = {
	DRM_GEM_SHADOW_PLANE_HELPER_FUNCS,
	.atomic_check = w100_primary_plane_helper_atomic_check,
	.atomic_update = w100_primary_plane_helper_atomic_update,
};

static const struct drm_plane_funcs w100_primary_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	DRM_GEM_SHADOW_PLANE_FUNCS,
};

/* ------------------------------------------------------------------ */
/*
 * Hardware video overlay plane: the chip's own fixed-function video
 * front-end, a second scanout source the CRTC composites above the
 * graphics plane, with hardware YUV->RGB conversion, an integer upscaler
 * and a colour keyer.
 *
 * The whole register protocol below is ported from libw100 0.0.2's
 * lib/overlay.c (GPLv2, Manuel Teira, 2006) -- the only source anywhere
 * that drives this block -- with the shipped mplayer libvo backend
 * vo_imageon.c (mplayer-imageon.patch, same project, same release) as the
 * reference consumer. Both are cited line-by-line below.
 *
 * Three limits, each because the prior art does not support
 * anything more:
 *
 *  - Source pixels must live in the chip's own SDRAM: mmVIDEO_Y_OFFSET is
 *    a 24-bit chip-internal address and the chip cannot bus-master out of
 *    system memory. So this is a shadow plane like the primary one --
 *    userspace hands it an ordinary shmem GEM buffer and the driver
 *    copies the visible source rectangle into the VRAM reservation at
 *    W100_OVERLAY_VRAM_OFFSET on each update. That copy is still a large
 *    net win over software YUV: vo_imageon.c's own full-screen path feeds
 *    a 240x320 YUV420 surface and asks for 2x expansion in both axes
 *    (mplayer-imageon.patch's config(), the VOFLAG_FULLSCREEN branch), so
 *    the same picture costs 115,200 bytes across the nCS2 bus and no CPU
 *    colour conversion at all, against 614,400 bytes of software-converted
 *    RGB565 through the primary plane. A zero-copy VRAM-backed GEM object
 *    would need a driver-private allocation ioctl, i.e. new UAPI, which
 *    this plane does not add.
 *  - Only planar YUV. The RGB source path (video_ch_sel = 1) additionally
 *    needs the mmDISP_DEBUG format bits libw100 pokes
 *    (lib/overlay.c:93-132) whose meaning is undocumented magic in that
 *    source too, and no known consumer ever used it.
 *  - Only integer scaling. Continuous scaling on this chip was never an
 *    overlay capability: vo_imageon.c gets it from the E2 2D engine
 *    (mmE2_ARITHMETIC_CNTL's E2_OPC_SCALE_SRC2) into a second,
 *    overlay-sized YUV surface which it then displays, and that engine is
 *    not brought up by this driver.
 */

/* Both formats are three separate planes with independent offset and pitch
 * registers, matching libw100's FMT_YUV420 (8-bit Y, 2x2-subsampled U and
 * V) and FMT_YUV422 (8-bit Y, 2x1-subsampled U and V) surface specs
 * (lib/surfspec.c:146-203). Note there is no interleaved-chroma mode with
 * uv_combine off, so NV12 is not a thing here.
 */
static const u32 w100_overlay_plane_formats[] = {
	DRM_FORMAT_YUV420,
	DRM_FORMAT_YUV422,
};

static const u64 w100_overlay_plane_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

/* Enum values for the driver-private "colorkey_mode" property. The
 * spelling is Armada's (armada_overlay.c's own
 * armada_drm_colorkey_enum_list) so userspace that already knows how to
 * drive an Armada overlay is not surprised by a novel string.
 */
enum w100_colorkey_mode {
	W100_COLORKEY_DISABLED = 0,
	W100_COLORKEY_RGB = 1,
};

/* A 1 bit in mmGRAPHIC_KEY's mask field means "compare this bit". */
#define W100_COLORKEY_MASK_ALL	0xffff

static const struct drm_prop_enum_list w100_colorkey_modes[] = {
	{ W100_COLORKEY_DISABLED, "disabled" },
	{ W100_COLORKEY_RGB, "RGB" },
};

struct w100_overlay_plane_state {
	struct drm_shadow_plane_state base;

	u16 colorkey;
	u16 colorkey_mask;
	u32 colorkey_mode;
};

#define to_w100_overlay_plane_state(x) \
	container_of_const(x, struct w100_overlay_plane_state, base.base)

/* Park the video front-end fully disabled. Writing the whole word clears
 * en_video_req, en_video_crtc *and* en_graphic_req_video together, which
 * is the state ac_overlay_disable() leaves behind (lib/overlay.c:39-53):
 * en_graphic_req_video is a hardware coupling to keyer_en, not a policy
 * bit, and leaving it set with the keyer off stops the graphics plane
 * fetching.
 */
static void w100_overlay_park(struct w100_device *w100)
{
	w100_update_disable(w100);
	writel(0, w100->regs + W100_REG_VIDEO_CTRL);
	w100_update_enable(w100);
}

/* Every plane of both supported formats carries one 8-bit sample per
 * (subsampled) pixel, so a plane's size in bytes is just its subsampled
 * pixel count and the whole surface is the sum over its planes.
 */
static unsigned int w100_overlay_src_size(const struct drm_format_info *format,
					  unsigned int width, unsigned int height)
{
	unsigned int i, size = 0;

	for (i = 0; i < format->num_planes; i++)
		size += (width / (i ? format->hsub : 1)) *
			(height / (i ? format->vsub : 1));

	return size;
}

/* Copy one already-subsampled source plane into the VRAM reservation.
 * Destination pitch is the copied width -- the reservation holds nothing
 * else, so packing the planes tightly is what makes the budget in
 * W100_OVERLAY_VRAM_SIZE add up.
 */
static void w100_overlay_copy_plane(struct w100_device *w100,
				    const struct drm_shadow_plane_state *shadow,
				    const struct drm_framebuffer *fb,
				    unsigned int index, unsigned int vram_offset,
				    unsigned int src_x, unsigned int src_y,
				    unsigned int width, unsigned int height)
{
	struct iosys_map dst = IOSYS_MAP_INIT_VADDR_IOMEM(w100->vram);
	struct iosys_map src = shadow->data[index];
	unsigned int y;

	iosys_map_incr(&dst, vram_offset);
	iosys_map_incr(&src, src_y * fb->pitches[index] + src_x);

	for (y = 0; y < height; y++) {
		iosys_map_memcpy_to(&dst, 0, src.vaddr, width);
		iosys_map_incr(&src, fb->pitches[index]);
		iosys_map_incr(&dst, width);
	}
}

static int w100_overlay_plane_helper_atomic_check(struct drm_plane *plane,
						  struct drm_atomic_commit *state)
{
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc *new_crtc = new_plane_state->crtc;
	struct drm_crtc_state *new_crtc_state = NULL;
	const struct drm_framebuffer *fb = new_plane_state->fb;
	unsigned int src_x, src_y, src_w, src_h, dst_w, dst_h;
	int ret;

	if (new_crtc)
		new_crtc_state = drm_atomic_get_new_crtc_state(state, new_crtc);

	ret = drm_atomic_helper_check_plane_state(new_plane_state, new_crtc_state,
						  W100_OVERLAY_MIN_SCALE,
						  DRM_PLANE_NO_SCALING,
						  true, false);
	if (ret)
		return ret;

	if (!new_plane_state->visible)
		return 0;

	/*
	 * The front-end cannot clip its own output. mmVIDEO_H_POS is just a
	 * start/end pair inside the graphics window; libw100's own
	 * ac_overlay_setpos() answers a right-edge overhang by shrinking
	 * total_req_video while leaving video_h_end past graphic_h_end
	 * (lib/overlay.c:327-338), and nothing anywhere handles a left or top
	 * overhang at all. Rather than guess at either, require the
	 * destination rectangle to land wholly on screen: the helper above has
	 * already clipped ->dst, so any disagreement with what userspace asked
	 * for is exactly "this needed clipping".
	 */
	if (new_plane_state->dst.x1 != new_plane_state->crtc_x ||
	    new_plane_state->dst.y1 != new_plane_state->crtc_y ||
	    drm_rect_width(&new_plane_state->dst) != (int)new_plane_state->crtc_w ||
	    drm_rect_height(&new_plane_state->dst) != (int)new_plane_state->crtc_h)
		return -EINVAL;

	/* mmVIDEO_Y_OFFSET/PITCH are plain byte quantities -- there is no
	 * fractional source addressing, so the 16.16 source rectangle has to
	 * be pixel-exact.
	 */
	if ((new_plane_state->src.x1 | new_plane_state->src.y1 |
	     new_plane_state->src.x2 | new_plane_state->src.y2) & 0xffff)
		return -EINVAL;

	src_x = new_plane_state->src.x1 >> 16;
	src_y = new_plane_state->src.y1 >> 16;
	src_w = drm_rect_width(&new_plane_state->src) >> 16;
	src_h = drm_rect_height(&new_plane_state->src) >> 16;
	dst_w = drm_rect_width(&new_plane_state->dst);
	dst_h = drm_rect_height(&new_plane_state->dst);

	/* drm_atomic_helper_check_plane_state() can only range-check the
	 * scale, and a range cannot express "1x, 2x, 3x or 4x and nothing in
	 * between" -- it will happily accept 1.5x, which a 2-bit exp field has
	 * no representation for. So reject non-integer ratios here.
	 */
	if (dst_w % src_w || dst_h % src_h)
		return -EINVAL;

	/* A source rectangle that does not start and end on a chroma sample
	 * has no representation in the U/V offset+pitch registers.
	 */
	if (src_x % fb->format->hsub || src_w % fb->format->hsub ||
	    src_y % fb->format->vsub || src_h % fb->format->vsub)
		return -EINVAL;

	if (w100_overlay_src_size(fb->format, src_w, src_h) > W100_OVERLAY_VRAM_SIZE)
		return -EINVAL;

	return 0;
}

/*
 * Copy the overlay's source into the VRAM reservation (when @copy) and
 * program the video front-end. The caller holds drm_dev_enter().
 *
 * @copy is false when the CRTC has just been enabled: the plane update ran
 * earlier in the same commit tail, before w100_apply_mode() brought the
 * clocks and the graphics plane up, and the front-end does not start from
 * registers written at that point. The pixels are already in VRAM, which
 * w100_apply_mode() leaves alone, so only the registers are written again.
 */
static void w100_overlay_program(struct w100_device *w100,
				 struct drm_plane_state *plane_state,
				 const struct w100_hw_mode *hw, bool copy)
{
	const struct w100_overlay_plane_state *w100_state;
	const struct drm_shadow_plane_state *shadow;
	const struct drm_framebuffer *fb = plane_state->fb;
	unsigned int offset[DRM_FORMAT_MAX_PLANES], pitch[DRM_FORMAT_MAX_PLANES];
	unsigned int src_x, src_y, src_w, src_h, dst_w, dst_h;
	unsigned int i, vram_offset;
	u32 ctrl;

	/* A plane still attached to the CRTC but entirely clipped away has no
	 * register programming that makes sense; park the front-end instead of
	 * leaving the previous frame latched.
	 */
	if (!plane_state->visible) {
		w100_overlay_park(w100);
		return;
	}

	w100_state = to_w100_overlay_plane_state(plane_state);
	shadow = to_drm_shadow_plane_state(plane_state);

	/* The position is relative to the graphics window, whose start is the
	 * mode's left/upper margin: ac_overlay_setpos() derives video_h_start
	 * from graphic_h_start the same way (lib/overlay.c:320-325).
	 */
	src_x = plane_state->src.x1 >> 16;
	src_y = plane_state->src.y1 >> 16;
	src_w = drm_rect_width(&plane_state->src) >> 16;
	src_h = drm_rect_height(&plane_state->src) >> 16;
	dst_w = drm_rect_width(&plane_state->dst);
	dst_h = drm_rect_height(&plane_state->dst);

	vram_offset = W100_OVERLAY_VRAM_OFFSET;
	for (i = 0; i < fb->format->num_planes; i++) {
		unsigned int hsub = i ? fb->format->hsub : 1;
		unsigned int vsub = i ? fb->format->vsub : 1;
		unsigned int lines = src_h / vsub;

		pitch[i] = src_w / hsub;
		offset[i] = W100_FB_BASE + vram_offset;

		if (copy)
			w100_overlay_copy_plane(w100, shadow, fb, i, vram_offset,
						src_x / hsub, src_y / vsub,
						pitch[i], lines);

		/* Vertical flip: the front-end walks the source backwards from
		 * the address it is handed, so each plane points at its own
		 * last line -- lib/overlay.c:215-224.
		 */
		if (plane_state->rotation & DRM_MODE_REFLECT_Y)
			offset[i] += (lines - 1) * pitch[i];

		vram_offset += pitch[i] * lines;
	}

	ctrl = W100_VIDEO_CTRL_EN_VIDEO_REQ | W100_VIDEO_CTRL_EN_VIDEO_CRTC |
	       W100_VIDEO_CTRL_YUV2RGB_EN |
	       FIELD_PREP(W100_VIDEO_CTRL_HOR_EXP, dst_w / src_w - 1) |
	       FIELD_PREP(W100_VIDEO_CTRL_VER_EXP, dst_h / src_h - 1) |
	       /* total_req_video, the per-line fetch request count, for the
		* YUV path: (destination width + 3) / 4, from
		* ac_overlay_setup()/ac_overlay_setpos() (lib/overlay.c:181,
		* 337) -- both compute it from the *expanded* width.
		*/
	       FIELD_PREP(W100_VIDEO_CTRL_TOTAL_REQ_VIDEO, (dst_w + 3) / 4);

	/* video_mode: 0 = YUV422, 1 = YUV420 (lib/acconstants.h:144-145). */
	if (fb->format->format == DRM_FORMAT_YUV420)
		ctrl |= W100_VIDEO_CTRL_VIDEO_MODE;

	/*
	 * On the hx4700 panel, BT.601- and BT.709-encoded test patches both
	 * decoded as BT.601 with yuv2rgb_option set, so this is the one
	 * setting known to give the only encoding the plane advertises.
	 */
	ctrl |= W100_VIDEO_CTRL_YUV2RGB_OPTION;

	if (plane_state->rotation & DRM_MODE_REFLECT_Y)
		ctrl |= W100_VIDEO_CTRL_VIDEO_INV_VER;

	/* en_graphic_req_video must track keyer_en. libw100 does this in both
	 * ac_overlay_enable() and ac_overlay_disable() (lib/overlay.c:29-31,
	 * 45-47); it is a hardware coupling, not a policy choice, so it is not
	 * exposed as a property.
	 */
	if (w100_state->colorkey_mode != W100_COLORKEY_DISABLED)
		ctrl |= W100_VIDEO_CTRL_KEYER_EN |
			W100_VIDEO_CTRL_EN_GRAPHIC_REQ_VIDEO;

	/*
	 * One mmDISP_DB_BUF_CNTL-bracketed batch. libw100 brackets every
	 * overlay register update with exactly this pair -- around the
	 * mmVIDEO_CTRL/offset/pitch block in ac_overlay_setup()
	 * (lib/overlay.c:265-277) and around the mmVIDEO_H_POS/V_POS block in
	 * ac_overlay_setpos() (lib/overlay.c:340-344). Collapsing its three
	 * separate brackets (setup, setpos, enable) into one is what makes
	 * this a genuinely atomic, tear-free plane update, and it needs no
	 * vblank interrupt -- which matters, because this driver has none:
	 * its vblank events come from a timer.
	 */
	w100_update_disable(w100);

	if (w100_state->colorkey_mode != W100_COLORKEY_DISABLED)
		writel(FIELD_PREP(W100_GRAPHIC_KEY_COLOR, w100_state->colorkey) |
		       FIELD_PREP(W100_GRAPHIC_KEY_MASK, w100_state->colorkey_mask),
		       w100->regs + W100_REG_GRAPHIC_KEY);

	/* Y, U and V are all unconditionally programmed: both advertised
	 * formats are three-plane, so the loop above filled all three slots.
	 */
	writel(ctrl, w100->regs + W100_REG_VIDEO_CTRL);
	writel(FIELD_PREP(W100_VIDEO_OFFSET, offset[0]), w100->regs + W100_REG_VIDEO_Y_OFFSET);
	writel(FIELD_PREP(W100_VIDEO_PITCH, pitch[0]), w100->regs + W100_REG_VIDEO_Y_PITCH);
	writel(FIELD_PREP(W100_VIDEO_OFFSET, offset[1]), w100->regs + W100_REG_VIDEO_U_OFFSET);
	writel(FIELD_PREP(W100_VIDEO_PITCH, pitch[1]), w100->regs + W100_REG_VIDEO_U_PITCH);
	writel(FIELD_PREP(W100_VIDEO_OFFSET, offset[2]), w100->regs + W100_REG_VIDEO_V_OFFSET);
	writel(FIELD_PREP(W100_VIDEO_PITCH, pitch[2]), w100->regs + W100_REG_VIDEO_V_PITCH);

	writel(W100_DISP_RANGE(hw->left_margin + plane_state->dst.x1,
			       hw->left_margin + plane_state->dst.x1 + dst_w),
	       w100->regs + W100_REG_VIDEO_H_POS);
	writel(W100_DISP_RANGE(hw->upper_margin + plane_state->dst.y1,
			       hw->upper_margin + plane_state->dst.y1 + dst_h),
	       w100->regs + W100_REG_VIDEO_V_POS);

	w100_update_enable(w100);
}

static void w100_overlay_plane_helper_atomic_update(struct drm_plane *plane,
						    struct drm_atomic_commit *state)
{
	struct w100_device *w100 = to_w100(plane->dev);
	struct drm_plane_state *plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *crtc_state;
	const struct w100_hw_mode *hw;
	int idx;

	if (!drm_dev_enter(&w100->drm, &idx))
		return;

	if (!plane_state->visible) {
		w100_overlay_park(w100);
		goto out;
	}

	/* The position is relative to the graphics window, whose start is the
	 * mode's margin. Take the mode from the new CRTC state rather than
	 * reading the register back: on an enabling commit the plane update runs
	 * before w100_crtc_helper_atomic_enable() has programmed the chip.
	 */
	crtc_state = drm_atomic_get_new_crtc_state(state, plane_state->crtc);
	hw = w100_find_hw_mode(crtc_state->mode.hdisplay, crtc_state->mode.vdisplay);
	if (WARN_ON(!hw))
		goto out;

	w100_overlay_program(w100, plane_state, hw, true);

out:
	drm_dev_exit(idx);
}

/*
 * Called from w100_crtc_helper_atomic_enable() once the chip is running the
 * mode: write the overlay registers again for a plane that was enabled in the
 * same commit as the modeset (see w100_overlay_program()).
 */
static void w100_overlay_replay(struct w100_device *w100, const struct w100_hw_mode *hw)
{
	struct drm_plane_state *plane_state = w100->overlay_plane.state;

	if (!plane_state || !plane_state->crtc || !plane_state->fb ||
	    !plane_state->visible)
		return;

	w100_overlay_program(w100, plane_state, hw, false);
}

static void w100_overlay_plane_helper_atomic_disable(struct drm_plane *plane,
						     struct drm_atomic_commit *state)
{
	struct w100_device *w100 = to_w100(plane->dev);
	int idx;

	if (!drm_dev_enter(&w100->drm, &idx))
		return;

	w100_overlay_park(w100);

	drm_dev_exit(idx);
}

static void w100_overlay_plane_destroy_state(struct drm_plane *plane,
					     struct drm_plane_state *plane_state)
{
	__drm_gem_destroy_shadow_plane_state(to_drm_shadow_plane_state(plane_state));
	kfree(to_w100_overlay_plane_state(plane_state));
}

static void w100_overlay_plane_reset(struct drm_plane *plane)
{
	struct w100_overlay_plane_state *new_state;

	if (plane->state) {
		w100_overlay_plane_destroy_state(plane, plane->state);
		plane->state = NULL;	/* must be set to NULL here */
	}

	/* The private fields must match the defaults the properties were
	 * attached with; everything standard (rotation, zpos, COLOR_ENCODING,
	 * COLOR_RANGE) is picked up from the attached properties' own defaults
	 * by __drm_atomic_helper_plane_state_reset().
	 */
	new_state = kzalloc_obj(*new_state);
	if (new_state) {
		new_state->colorkey_mask = W100_COLORKEY_MASK_ALL;
		new_state->colorkey_mode = W100_COLORKEY_DISABLED;
	}
	__drm_gem_reset_shadow_plane(plane, new_state ? &new_state->base : NULL);
}

static struct drm_plane_state *w100_overlay_plane_duplicate_state(struct drm_plane *plane)
{
	const struct w100_overlay_plane_state *old_state;
	struct w100_overlay_plane_state *new_state;

	if (WARN_ON(!plane->state))
		return NULL;

	old_state = to_w100_overlay_plane_state(plane->state);

	new_state = kzalloc_obj(*new_state);
	if (!new_state)
		return NULL;

	__drm_gem_duplicate_shadow_plane_state(plane, &new_state->base);

	new_state->colorkey = old_state->colorkey;
	new_state->colorkey_mask = old_state->colorkey_mask;
	new_state->colorkey_mode = old_state->colorkey_mode;

	return &new_state->base.base;
}

static int w100_overlay_plane_atomic_set_property(struct drm_plane *plane,
						  struct drm_plane_state *plane_state,
						  struct drm_property *property,
						  u64 val)
{
	struct w100_device *w100 = to_w100(plane->dev);
	struct w100_overlay_plane_state *w100_state = to_w100_overlay_plane_state(plane_state);

	if (property == w100->colorkey_prop)
		w100_state->colorkey = val;
	else if (property == w100->colorkey_mask_prop)
		w100_state->colorkey_mask = val;
	else if (property == w100->colorkey_mode_prop)
		w100_state->colorkey_mode = val;
	else
		return -EINVAL;

	return 0;
}

static int w100_overlay_plane_atomic_get_property(struct drm_plane *plane,
						  const struct drm_plane_state *plane_state,
						  struct drm_property *property,
						  u64 *val)
{
	struct w100_device *w100 = to_w100(plane->dev);
	const struct w100_overlay_plane_state *w100_state;

	w100_state = to_w100_overlay_plane_state(plane_state);

	if (property == w100->colorkey_prop)
		*val = w100_state->colorkey;
	else if (property == w100->colorkey_mask_prop)
		*val = w100_state->colorkey_mask;
	else if (property == w100->colorkey_mode_prop)
		*val = w100_state->colorkey_mode;
	else
		return -EINVAL;

	return 0;
}

static const struct drm_plane_helper_funcs w100_overlay_plane_helper_funcs = {
	DRM_GEM_SHADOW_PLANE_HELPER_FUNCS,
	.atomic_check = w100_overlay_plane_helper_atomic_check,
	.atomic_update = w100_overlay_plane_helper_atomic_update,
	.atomic_disable = w100_overlay_plane_helper_atomic_disable,
};

/* DRM_GEM_SHADOW_PLANE_FUNCS is not used here: all three of
 * the hooks it supplies are overridden so the driver-private colour-key
 * fields ride along in a subclassed drm_shadow_plane_state.
 */
static const struct drm_plane_funcs w100_overlay_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = w100_overlay_plane_reset,
	.atomic_duplicate_state = w100_overlay_plane_duplicate_state,
	.atomic_destroy_state = w100_overlay_plane_destroy_state,
	.atomic_set_property = w100_overlay_plane_atomic_set_property,
	.atomic_get_property = w100_overlay_plane_atomic_get_property,
};

static int w100_overlay_plane_init(struct w100_device *w100)
{
	struct drm_device *dev = &w100->drm;
	struct drm_plane *plane = &w100->overlay_plane;
	struct drm_property *prop;
	int ret;

	ret = drm_universal_plane_init(dev, plane, 0, &w100_overlay_plane_funcs,
				       w100_overlay_plane_formats,
				       ARRAY_SIZE(w100_overlay_plane_formats),
				       w100_overlay_plane_format_modifiers,
				       DRM_PLANE_TYPE_OVERLAY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(plane, &w100_overlay_plane_helper_funcs);

	/* Immutable: the video front-end composites above the graphics plane
	 * and there is no ordering control anywhere in the register map.
	 */
	ret = drm_plane_create_zpos_immutable_property(plane, 1);
	if (ret)
		return ret;

	/*
	 * Vertical reflection only.
	 *
	 * video_inv_hor exists too, and DRM_MODE_REFLECT_X is not
	 * advertised for it: libw100's horizontally-flipped source-offset
	 * arithmetic carries unexplained "+1" and "-4" fudge terms that do not
	 * even agree between the luma and chroma planes
	 * (lib/overlay.c:230-238), nothing documents them, and no consumer
	 * ever set inv_hor -- every caller in libw100's own tests and in
	 * vo_imageon.c passes 0. The vertical case has no such fudge (it is a
	 * plain "point at the last line", lib/overlay.c:215-224), so it is the
	 * one that can be implemented without guessing.
	 *
	 * 90/270 rotation is not offered at all: video_portrait exists, but
	 * libw100's handling of it is corner-pointing offset fixups its own
	 * comment says work only for RGB surfaces (lib/overlay.c:142-164).
	 */
	ret = drm_plane_create_rotation_property(plane, DRM_MODE_ROTATE_0,
						 DRM_MODE_ROTATE_0 | DRM_MODE_REFLECT_Y);
	if (ret)
		return ret;

	/*
	 * The hardware has exactly one colour-space selector, yuv2rgb_option,
	 * whose meaning no register map documents. With it set, BT.601- and
	 * BT.709-encoded test patches both decoded as BT.601 on the hx4700
	 * panel, so only BT.601 is advertised: what the other value of the
	 * bit selects has not been observed. No range choice is offered
	 * either, as there is no range control bit anywhere in the register
	 * map.
	 */
	ret = drm_plane_create_color_properties(plane,
						BIT(DRM_COLOR_YCBCR_BT601),
						BIT(DRM_COLOR_YCBCR_LIMITED_RANGE),
						DRM_COLOR_YCBCR_BT601,
						DRM_COLOR_YCBCR_LIMITED_RANGE);
	if (ret)
		return ret;

	/*
	 * Colour key. There is no core drm_plane_create_colorkey_property(),
	 * so these are driver-private, following armada_overlay.c's precedent
	 * (armada_overlay_create_properties()) but cut to what mmGRAPHIC_KEY
	 * really is: a single 16-bit RGB565 key plus a 16-bit compare mask,
	 * not Armada's six-property min/max/val/alpha range.
	 *
	 * Semantics as measured on the hx4700 panel (neither libw100 nor
	 * vo_imageon.c ever set a key, so there is no prior art): the video
	 * shows wherever the stored RGB565 value of the primary-plane pixel
	 * matches "colorkey" in every bit selected by "colorkey_mask", and the
	 * graphics show everywhere else. A 1 mask bit means "compared", a 0
	 * bit "ignored"; the compare is exact per pixel, with no tolerance and
	 * no chroma shift against luma.
	 *
	 * The mask therefore defaults to 0xffff, so enabling the key with only
	 * a key value compares every bit; a zero mask would match every pixel
	 * and hide the graphics plane entirely. The mode still defaults to
	 * "disabled": whether the plane is visible at all with the keyer off
	 * has not been settled on hardware, so enabling it is left to
	 * userspace rather than tied to the plane being enabled.
	 */
	prop = drm_property_create_range(dev, 0, "colorkey", 0, 0xffff);
	if (!prop)
		return -ENOMEM;
	w100->colorkey_prop = prop;
	drm_object_attach_property(&plane->base, prop, 0);

	prop = drm_property_create_range(dev, 0, "colorkey_mask", 0, 0xffff);
	if (!prop)
		return -ENOMEM;
	w100->colorkey_mask_prop = prop;
	drm_object_attach_property(&plane->base, prop, W100_COLORKEY_MASK_ALL);

	prop = drm_property_create_enum(dev, 0, "colorkey_mode",
					w100_colorkey_modes,
					ARRAY_SIZE(w100_colorkey_modes));
	if (!prop)
		return -ENOMEM;
	w100->colorkey_mode_prop = prop;
	drm_object_attach_property(&plane->base, prop, W100_COLORKEY_DISABLED);

	return 0;
}

/* ------------------------------------------------------------------ */

static enum drm_mode_status w100_crtc_helper_mode_valid(struct drm_crtc *crtc,
							  const struct drm_display_mode *mode)
{
	if (!w100_find_hw_mode(mode->hdisplay, mode->vdisplay))
		return MODE_BAD;

	return MODE_OK;
}

static int w100_crtc_helper_atomic_check(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	if (!crtc_state->enable)
		return 0;

	if (!w100_find_hw_mode(crtc_state->mode.hdisplay, crtc_state->mode.vdisplay))
		return -EINVAL;

	return drm_atomic_helper_check_crtc_primary_plane(crtc_state);
}

static void w100_crtc_helper_atomic_enable(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct w100_device *w100 = to_w100(crtc->dev);
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	const struct w100_hw_mode *hw;
	int idx;

	hw = w100_find_hw_mode(crtc_state->mode.hdisplay, crtc_state->mode.vdisplay);
	if (WARN_ON(!hw))
		return;

	if (!drm_dev_enter(&w100->drm, &idx))
		return;

	w100_panel_power_up(w100);
	w100_apply_mode(w100, hw);
	w100_overlay_replay(w100, hw);

	/*
	 * Backlight last, once the panel rails are up and the chip is
	 * scanning out -- the same order drm_panel_enable() gives panel
	 * drivers.
	 */
	backlight_enable(w100->backlight);

	drm_dev_exit(idx);

	drm_crtc_vblank_on(crtc);
}

static void w100_crtc_helper_atomic_disable(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct w100_device *w100 = to_w100(crtc->dev);

	drm_crtc_vblank_off(crtc);

	/*
	 * Backlight first, while the panel is still powered and driven: an
	 * undriven TFT passes the light straight through, so a lit
	 * backlight turns a blanked screen white instead of dark. This is
	 * the order drm_panel_disable() gives panel drivers.
	 */
	backlight_disable(w100->backlight);

	/*
	 * Not guarded by drm_dev_enter/exit, matching mq11xx.c's own
	 * .atomic_disable and for the same reason: w100_remove() calls
	 * drm_dev_unplug() before drm_atomic_helper_shutdown(), so an
	 * enter/exit-guarded version of this hook would find the device
	 * already marked unplugged on a normal rmmod/unbind and the panel
	 * would never actually power down. The GPIO descriptors are
	 * devm-managed and torn down after .remove() returns, so touching
	 * them here is safe either way.
	 */

	/*
	 * ATIDMATeardown() (ati_dma.c:281-294): stop the processor
	 * fetching before the display goes away. The same reasoning about
	 * drm_dev_unplug() applies -- the register mappings are devm
	 * resources and outlive .remove(), so this is reachable on a
	 * normal rmmod, which is exactly when it matters that the chip is
	 * not left executing from a ring nobody owns any more. The ring is
	 * rebuilt from scratch by the next .atomic_enable.
	 */
	mutex_lock(&w100->accel_lock);
	w100_cp_ring_fini_locked(w100);
	mutex_unlock(&w100->accel_lock);

	w100_panel_power_down(w100);
}

static const struct drm_crtc_helper_funcs w100_crtc_helper_funcs = {
	.mode_valid = w100_crtc_helper_mode_valid,
	.atomic_check = w100_crtc_helper_atomic_check,
	.atomic_flush = drm_crtc_vblank_atomic_flush,
	.atomic_enable = w100_crtc_helper_atomic_enable,
	.atomic_disable = w100_crtc_helper_atomic_disable,
};

static const struct drm_crtc_funcs w100_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	DRM_CRTC_VBLANK_TIMER_FUNCS,
};

static const struct drm_encoder_funcs w100_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int w100_connector_helper_get_modes(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct drm_display_mode *mode;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(hx4700_w100_hw_modes); i++) {
		mode = drm_mode_duplicate(dev, hx4700_w100_hw_modes[i].drm_mode);
		if (!mode)
			continue;
		drm_mode_probed_add(connector, mode);
		connector->display_info.width_mm = mode->width_mm;
		connector->display_info.height_mm = mode->height_mm;
	}

	return ARRAY_SIZE(hx4700_w100_hw_modes);
}

static const struct drm_connector_helper_funcs w100_connector_helper_funcs = {
	.get_modes = w100_connector_helper_get_modes,
};

static const struct drm_connector_funcs w100_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int w100_pipe_init(struct w100_device *w100)
{
	struct drm_device *dev = &w100->drm;
	struct drm_plane *primary_plane = &w100->primary_plane;
	struct drm_crtc *crtc = &w100->crtc;
	struct drm_encoder *encoder = &w100->encoder;
	struct drm_connector *connector = &w100->connector;
	int ret;

	ret = drm_universal_plane_init(dev, primary_plane, 0,
					&w100_primary_plane_funcs,
					w100_primary_plane_formats,
					ARRAY_SIZE(w100_primary_plane_formats),
					w100_primary_plane_format_modifiers,
					DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(primary_plane, &w100_primary_plane_helper_funcs);
	drm_plane_enable_fb_damage_clips(primary_plane);

	/* Immutable, to give the overlay plane's own zpos something to be
	 * above -- the graphics plane is the bottom layer in silicon.
	 */
	ret = drm_plane_create_zpos_immutable_property(primary_plane, 0);
	if (ret)
		return ret;

	/* The overlay is not passed as the cursor argument here: it is not a
	 * cursor plane, and this chip has no cursor hardware at all (no
	 * mmCUR_* register in any of the three register maps).
	 */
	ret = drm_crtc_init_with_planes(dev, crtc, primary_plane, NULL,
					 &w100_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(crtc, &w100_crtc_helper_funcs);

	/*
	 * The overlay's source buffers live in VRAM above the primary
	 * framebuffer, so a device tree whose "fb" window is too small to hold
	 * both simply gets no overlay plane. The display itself is fine either
	 * way, and an older device tree describing only the 1 MiB this driver
	 * used to need keeps working.
	 */
	if (w100->vram_size >= W100_OVERLAY_VRAM_OFFSET + W100_OVERLAY_VRAM_SIZE) {
		ret = w100_overlay_plane_init(w100);
		if (ret)
			return ret;
		/* Set after drm_crtc_init_with_planes(), which only fixes up
		 * the primary and cursor planes it was handed.
		 */
		w100->overlay_plane.possible_crtcs = drm_crtc_mask(crtc);
	} else {
		drm_info(dev, "video memory window is %zu bytes, %u needed for the overlay plane -- overlay disabled\n",
			 w100->vram_size,
			 W100_OVERLAY_VRAM_OFFSET + W100_OVERLAY_VRAM_SIZE);
	}

	ret = drm_encoder_init(dev, encoder, &w100_encoder_funcs,
				DRM_MODE_ENCODER_DPI, NULL);
	if (ret)
		return ret;
	encoder->possible_crtcs = drm_crtc_mask(crtc);

	ret = drm_connector_init(dev, connector, &w100_connector_funcs,
				  DRM_MODE_CONNECTOR_DPI);
	if (ret)
		return ret;
	drm_connector_helper_add(connector, &w100_connector_helper_funcs);

	return drm_connector_attach_encoder(connector, encoder);
}

/* ------------------------------------------------------------------ */

static const struct drm_mode_config_funcs w100_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int w100_mode_config_init(struct w100_device *w100)
{
	struct drm_device *dev = &w100->drm;
	int ret;

	ret = drmm_mode_config_init(dev);
	if (ret)
		return ret;

	/*
	 * These bound *framebuffer* dimensions, not modes -- they are what
	 * framebuffer_check() (drivers/gpu/drm/drm_framebuffer.c) validates an
	 * ADDFB2 against -- so they cannot be pinned to the smallest panel
	 * timing. The overlay plane's whole point is scanning out a source
	 * smaller than the screen and letting the hardware upscale it:
	 * vo_imageon.c's own full-screen path feeds a 240x320 YUV420 surface
	 * and asks for 2x in both axes. Modes are constrained separately, by
	 * w100_crtc_helper_mode_valid()/w100_find_hw_mode(), and an
	 * undersized primary framebuffer is still rejected by
	 * w100_primary_plane_helper_atomic_check()'s own can_position=false.
	 */
	dev->mode_config.min_width = 1;
	dev->mode_config.max_width = 480;
	dev->mode_config.min_height = 1;
	dev->mode_config.max_height = 640;
	dev->mode_config.preferred_depth = 16;
	dev->mode_config.prefer_shadow = 0;
	dev->mode_config.funcs = &w100_mode_config_funcs;

	return 0;
}

DEFINE_DRM_GEM_FOPS(w100_fops);

static const struct drm_driver w100_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,

	.name	= DRIVER_NAME,
	.desc	= DRIVER_DESC,
	.major	= DRIVER_MAJOR,
	.minor	= DRIVER_MINOR,

	.fops	= &w100_fops,
	DRM_GEM_SHMEM_DRIVER_OPS,
	DRM_FBDEV_SHMEM_DRIVER_OPS,
};

/*
 * Map the CIF wrap aperture, through which the host writes command-ring
 * dwords, if the device tree describes one.
 *
 * Every failure here is non-fatal and leaves w100->wrap NULL, which
 * means no command processor and a driver that keeps programming the 2D
 * engine by polled register writes. A device tree without a "wrap"
 * window is a supported configuration, not an error, and none of the
 * rest of the driver depends on this.
 *
 * The position check is worth making rather than trusting reg-names,
 * because what it catches is otherwise silent. The aperture's chip
 * offset is not something the driver can ask the hardware for; it is
 * fixed by the mmWRAP_START_DIR value w100_chip_bringup() writes. A
 * window in the wrong place would be mapped happily, and the ring writes
 * would land somewhere in the chip that is not the ring: the processor's
 * read pointer would never move and every operation would end in a drain
 * timeout, with nothing pointing at the device tree as the cause.
 */
static void w100_wrap_init(struct w100_device *w100, struct platform_device *pdev,
			   phys_addr_t chip_base)
{
	phys_addr_t expect = chip_base + W100_WRAP_BASE;
	struct resource *res;
	void __iomem *wrap;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "wrap");
	if (!res)
		return;

	if (res->start != expect) {
		drm_warn(&w100->drm,
			 "wrap window is at %pa, but mmWRAP_START_DIR puts it at %pa; ignoring it\n",
			 &res->start, &expect);
		return;
	}

	if (resource_size(res) < W100_CP_RING_BYTES) {
		drm_warn(&w100->drm,
			 "wrap window is %llu bytes, %u needed for the command ring; ignoring it\n",
			 (unsigned long long)resource_size(res), W100_CP_RING_BYTES);
		return;
	}

	wrap = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(wrap)) {
		drm_warn(&w100->drm, "failed to map wrap window (%pe); ignoring it\n", wrap);
		return;
	}

	w100->wrap = wrap;
}

static int w100_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct w100_device *w100;
	struct drm_device *drm;
	struct resource *res;
	u32 chip_id;
	int ret;

	w100 = devm_drm_dev_alloc(dev, &w100_driver, struct w100_device, drm);
	if (IS_ERR(w100))
		return PTR_ERR(w100);
	drm = &w100->drm;

	platform_set_drvdata(pdev, w100);

	ret = drmm_mutex_init(drm, &w100->accel_lock);
	if (ret)
		return ret;

	/*
	 * Registered unconditionally, before anything can load the
	 * microcode: the fetch itself happens lazily on the first mode
	 * apply, so this has to already be in place by then, and it does
	 * nothing at all if no firmware was ever loaded.
	 */
	ret = drmm_add_action_or_reset(drm, w100_cp_release_firmware, w100);
	if (ret)
		return ret;

	w100->cfg = devm_platform_ioremap_resource_byname(pdev, "cfg");
	if (IS_ERR(w100->cfg))
		return dev_err_probe(dev, PTR_ERR(w100->cfg), "failed to map cfg window\n");

	w100->regs = devm_platform_ioremap_resource_byname(pdev, "reg");
	if (IS_ERR(w100->regs))
		return dev_err_probe(dev, PTR_ERR(w100->regs), "failed to map register window\n");

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "fb");
	if (!res)
		return dev_err_probe(dev, -EINVAL, "missing video memory window\n");

	w100->vram = devm_ioremap_resource(dev, res);
	if (IS_ERR(w100->vram))
		return dev_err_probe(dev, PTR_ERR(w100->vram),
				      "failed to map video memory window\n");
	w100->vram_size = resource_size(res);

	if (w100->vram_size < W100_FB_SIZE)
		return dev_err_probe(dev, -EINVAL,
				      "video memory window is %zu bytes, %u needed for the largest mode\n",
				      w100->vram_size, W100_FB_SIZE);

	/*
	 * res->start - W100_FB_BASE is chip offset 0 in host physical
	 * terms, derived from where the device tree puts the one window
	 * whose chip offset the driver already knows. w100_wrap_init()
	 * uses it to check the "wrap" window is where the hardware has
	 * been told to put it.
	 */
	w100_wrap_init(w100, pdev, res->start - W100_FB_BASE);

	/* W3220_nRESET (PXA GPIO61) -- deasserted once here and never
	 * toggled again, see struct w100_device's own field comment.
	 */
	w100->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(w100->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(w100->reset_gpio),
				      "failed to get chip reset gpio\n");

	/*
	 * The six panel-power lines below and panel_reset_gpio just after
	 * are all requested deasserted/off (rails low, reset asserted),
	 * *not* the recovered global_gpios[]'s own GPIOF_OUT_INIT_HIGH
	 * defaults -- that init-high default described the state a
	 * pre-kernel bootloader had already sequenced the panel into by
	 * the time hx4700.c's own arch_initcall ran, which this driver's
	 * probe() has no equivalent assumption to inherit. Starting
	 * deasserted and letting w100_crtc_helper_atomic_enable() run the
	 * real sony_lcd_init() sequence (w100_panel_power_up()) on first
	 * enable is the conservative choice, not a transcribed one.
	 */
	w100->sqn_gpio = devm_gpiod_get(dev, "sqn", GPIOD_OUT_LOW);
	if (IS_ERR(w100->sqn_gpio))
		return dev_err_probe(dev, PTR_ERR(w100->sqn_gpio), "failed to get sqn gpio\n");

	w100->lvdd_gpio = devm_gpiod_get(dev, "lvdd", GPIOD_OUT_LOW);
	if (IS_ERR(w100->lvdd_gpio))
		return dev_err_probe(dev, PTR_ERR(w100->lvdd_gpio), "failed to get lvdd gpio\n");

	w100->avdd_gpio = devm_gpiod_get(dev, "avdd", GPIOD_OUT_LOW);
	if (IS_ERR(w100->avdd_gpio))
		return dev_err_probe(dev, PTR_ERR(w100->avdd_gpio), "failed to get avdd gpio\n");

	w100->slin1_gpio = devm_gpiod_get(dev, "slin1", GPIOD_OUT_LOW);
	if (IS_ERR(w100->slin1_gpio))
		return dev_err_probe(dev, PTR_ERR(w100->slin1_gpio), "failed to get slin1 gpio\n");

	w100->panel_reset_gpio = devm_gpiod_get(dev, "panel-reset", GPIOD_OUT_HIGH);
	if (IS_ERR(w100->panel_reset_gpio))
		return dev_err_probe(dev, PTR_ERR(w100->panel_reset_gpio),
				      "failed to get panel reset gpio\n");

	w100->pc1_gpio = devm_gpiod_get(dev, "pc1", GPIOD_OUT_LOW);
	if (IS_ERR(w100->pc1_gpio))
		return dev_err_probe(dev, PTR_ERR(w100->pc1_gpio), "failed to get pc1 gpio\n");

	w100->n2v7_gpio = devm_gpiod_get(dev, "n2v7", GPIOD_OUT_LOW);
	if (IS_ERR(w100->n2v7_gpio))
		return dev_err_probe(dev, PTR_ERR(w100->n2v7_gpio), "failed to get n2v7 gpio\n");

	w100->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(w100->backlight))
		return dev_err_probe(dev, PTR_ERR(w100->backlight), "failed to get backlight\n");

	/*
	 * The recovered board file released W3220_nRESET once, early at
	 * arch_initcall time, long before its own w100fb_probe() ever ran
	 * -- so the original driver needed no settle delay here at all.
	 * This driver now owns requesting and deasserting that same line
	 * itself, from inside its own probe(), so a conservative settle
	 * delay before the first register access is this driver's own
	 * addition, not a transcribed value.
	 */
	usleep_range(1000, 2000);

	chip_id = readl(w100->regs + W100_REG_CHIP_ID);
	if (chip_id != W100_CHIP_ID_W3220)
		return dev_err_probe(dev, -ENODEV,
				      "unexpected chip ID 0x%08x (expected W3220 0x%08x)\n",
				      chip_id, W100_CHIP_ID_W3220);
	drm_info(drm, "Found W3220 (chip ID 0x%08x)\n", chip_id);

	w100_chip_bringup(w100);

	/* The video front-end comes out of reset in an undefined state and
	 * nothing in this driver touches it again unless an overlay commit
	 * does, so park it disabled once, here.
	 */
	w100_overlay_park(w100);

	ret = w100_mode_config_init(w100);
	if (ret)
		return ret;

	ret = w100_pipe_init(w100);
	if (ret)
		return ret;

	ret = drm_vblank_init(drm, 1);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	drm_client_setup(drm, NULL);

	return 0;
}

static void w100_remove(struct platform_device *pdev)
{
	struct w100_device *w100 = platform_get_drvdata(pdev);

	drm_dev_unplug(&w100->drm);
	drm_atomic_helper_shutdown(&w100->drm);
}

static const struct of_device_id w100_of_match[] = {
	{ .compatible = "amd,imageon-w3220" },
	{ }
};
MODULE_DEVICE_TABLE(of, w100_of_match);

static struct platform_driver w100_platform_driver = {
	.probe	= w100_probe,
	.remove	= w100_remove,
	.driver	= {
		.name = DRIVER_NAME,
		.of_match_table = w100_of_match,
	},
};
module_platform_driver(w100_platform_driver);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_FIRMWARE(W100_CP_FW_NAME);
MODULE_LICENSE("GPL");
