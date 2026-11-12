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
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>

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
#include <drm/drm_modes.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
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
#define W100_REG_VIDEO_CTRL		0x0434	/* mmVIDEO_CTRL */
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

/* WRAP_BUF_BASE_VALUE/WRAP_BUF_TOP_VALUE, both >> 1 to match
 * wrap_start_dir_t/wrap_top_dir_t's own 23-bit half-word-address field.
 */
#define W100_WRAP_START_DIR_VAL	(0x80000 >> 1)
#define W100_WRAP_TOP_DIR_VAL		(0xbffff >> 1)

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

/* video_ctrl_t: only the fetch and CRTC enables the display-off path clears */
#define W100_VIDEO_CTRL_EN_VIDEO_REQ		BIT(2)
#define W100_VIDEO_CTRL_EN_GRAPHIC_REQ_VIDEO	BIT(3)
#define W100_VIDEO_CTRL_EN_VIDEO_CRTC		BIT(4)

/* active_h_disp_t/active_v_disp_t/graphic_h_disp_t/graphic_v_disp_t/
 * crtc_total_t all share this 10-bit-start/6-reserved/10-bit-end/
 * 6-reserved shape.
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
	void __iomem *vram;	/* framebuffer window at chip offset W100_FB_BASE */

	struct drm_plane primary_plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;

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

/* w100fb_activate_var() (w100fb.c:412-432), minus w100fb_clear_screen()
 * (the shadow-plane pipeline's first damage-clipped update already does
 * this in effect) and w100_init_graphic_engine()/calc_hsync()/tg->change()
 * (all dropped, see the file comment).
 */
static void w100_apply_mode(struct w100_device *w100, const struct w100_hw_mode *hw)
{
	w100_pwm_setup(w100);
	w100_setup_memory(w100);
	w100_init_clocks(w100, hw);
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

static void w100_primary_plane_helper_atomic_update(struct drm_plane *plane,
						      struct drm_atomic_commit *state)
{
	struct w100_device *w100 = to_w100(plane->dev);
	struct drm_plane_state *plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_shadow_plane_state *shadow_plane_state = to_drm_shadow_plane_state(plane_state);
	struct drm_framebuffer *fb = plane_state->fb;
	struct drm_plane_state *old_plane_state = drm_atomic_get_old_plane_state(state, plane);
	struct iosys_map vram_map = IOSYS_MAP_INIT_VADDR_IOMEM(w100->vram);
	struct drm_atomic_helper_damage_iter iter;
	struct drm_rect damage;
	int idx;

	if (!fb)
		return;

	if (!drm_dev_enter(&w100->drm, &idx))
		return;

	drm_atomic_helper_damage_iter_init(&iter, old_plane_state, plane_state);
	drm_atomic_for_each_plane_damage(&iter, &damage) {
		unsigned int offset = drm_fb_clip_offset(fb->pitches[0], fb->format, &damage);
		struct iosys_map dst = IOSYS_MAP_INIT_OFFSET(&vram_map, offset);

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

	ret = drm_crtc_init_with_planes(dev, crtc, primary_plane, NULL,
					 &w100_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(crtc, &w100_crtc_helper_funcs);

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

	dev->mode_config.min_width = 240;
	dev->mode_config.max_width = 480;
	dev->mode_config.min_height = 320;
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

static int w100_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct w100_device *w100;
	struct drm_device *drm;
	u32 chip_id;
	int ret;

	w100 = devm_drm_dev_alloc(dev, &w100_driver, struct w100_device, drm);
	if (IS_ERR(w100))
		return PTR_ERR(w100);
	drm = &w100->drm;

	platform_set_drvdata(pdev, w100);

	w100->cfg = devm_platform_ioremap_resource_byname(pdev, "cfg");
	if (IS_ERR(w100->cfg))
		return dev_err_probe(dev, PTR_ERR(w100->cfg), "failed to map cfg window\n");

	w100->regs = devm_platform_ioremap_resource_byname(pdev, "reg");
	if (IS_ERR(w100->regs))
		return dev_err_probe(dev, PTR_ERR(w100->regs), "failed to map register window\n");

	w100->vram = devm_platform_ioremap_resource_byname(pdev, "fb");
	if (IS_ERR(w100->vram))
		return dev_err_probe(dev, PTR_ERR(w100->vram),
				      "failed to map framebuffer window\n");

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
MODULE_LICENSE("GPL");
