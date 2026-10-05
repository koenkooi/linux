// SPDX-License-Identifier: GPL-2.0-only
/*
 * HP iPAQ h2200 suspend/resume support.
 *
 * The h2200's HTC-supplied bootloader ("bootldr") is entered by the CPU
 * on every resume from PM_SUSPEND_MEM, because reset always starts
 * execution at physical address 0 -- which is HAMCOP's 16 KiB SRAM
 * window, mapped at CS0 (PXA_CS0_PHYS). On a WinCE system the bootloader
 * itself knows how to jump back into the sleeping OS; on Linux it just
 * does its normal cold-boot dance and jumps to a fixed WinCE load
 * address (0xa0040000) that happens to land inside kernel code -- never
 * where we actually want to resume.
 *
 * The fix, inherited verbatim from the old out-of-tree h2200_pm.c (Matt
 * Reimer): capture a copy of the bootloader out of SRAM once at boot,
 * patch a tiny position-independent thunk into its unused tail that
 * jumps through PSPR (the PXA "Power Manager Scratch Pad Register")
 * instead of the hardcoded WinCE address, and copy the patched image
 * back into SRAM immediately before every suspend. pxa25x_cpu_pm_prepare()
 * (reached via pxa_pm_prepare(), see pm.c/pm.h) has already stashed
 * __pa_symbol(cpu_resume) in PSPR by the time that copy-back happens, so
 * the thunk lands the CPU straight back into the kernel's own resume
 * path. This is a modernization of the 2007 out-of-tree h2200_pm.c.
 *
 * What changed vs. the 2007 driver, beyond the API churn:
 *  - The old driver monkey-patched the global pxa_pm_ops.enter function
 *    pointer at runtime. That pointer is now `static const`
 *    (arch/arm/mach-pxa/pm.c) and can't be mutated. Composing a board
 *    suspend_ops around the still-exported pxa_pm_{prepare,enter,finish}
 *    hooks and registering it with suspend_set_ops() -- last caller
 *    wins -- is the replacement, exactly the pattern sharpsl_pm.c (same
 *    directory) already uses today.
 *  - The old driver only ever discovered a bad bootloader from
 *    .enter(), i.e. after every device in the system had already
 *    suspended. Refusing from platform_suspend_ops.begin() aborts
 *    before anything is torn down.
 *  - request_firmware_nowait() replaces the old CONFIG_FW_LOADER_MODULE
 *    delayed-work polling hack (that config option no longer exists).
 *  - The old driver's h2200_pxa_pm_enter() also unconditionally
 *    reprogrammed MSC0-2 with hardcoded values after every resume, to
 *    undo whatever the bootloader clobbered. That's now a genuine
 *    save/restore around pxa_pm_enter() instead of hardcoded magic
 *    numbers (the values happen to always come back the same, but nothing
 *    here needs to assume that anymore).
 *  - Bootloader restoration before a warm reset used to be this driver's
 *    .shutdown() hook; the equivalent today is a
 *    SYS_OFF_MODE_RESTART_PREPARE sys-off handler.
 *  - get_hamcop_bootloader(), the old cross-module export used by
 *    samcop_sdi to save and restore the SRAM around its own DMA-into-SRAM
 *    bounce buffer, is gone. The need it served is not: HAMCOP's DMA
 *    engine cannot reach anything but this SRAM, so a DMA capable SDI
 *    driver has to stage through it. The driver publishes the SRAM as a
 *    genalloc pool (h2200_sram_pool(), linux/platform_data/h2200-sram.h)
 *    that consumers suballocate from, so several DMA users can coexist.
 *
 *    Letting others scribble on the window is safe because nothing here
 *    reads it after probe: the bootloader is captured into a RAM copy
 *    once, and every later access is a write back into SRAM, from
 *    .prepare() and from the restart handler. Both run after devices
 *    are suspended or stopped, so a DMA user is quiesced by then. It
 *    would not be safe to keep a live DMA engine running across suspend.
 *
 * The SRAM window reaches this driver as the "hamcop-sram" sub-device of
 * the HAMCOP MFD (drivers/mfd/hamcop.c), from the second "reg" entry of
 * the chip's devicetree node. The MFD passes it on without mapping it, and
 * adds it before its other sub-devices, so this driver has captured the
 * bootloader and published the pool before hamcop-sdi probes.
 *
 * Out of scope here: the HAMCOP MFD chip's own
 * suspend/resume (INTMSK mask/restore, CIF/clock reinit order) is a
 * dev_pm_ops on the HAMCOP platform_driver itself, not this file --
 * this driver knows nothing about HAMCOP's register layout, only about
 * the SRAM window's byte contents.
 */

#include <linux/completion.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/genalloc.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/minmax.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/platform_data/h2200-sram.h>
#include <linux/reboot.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/suspend.h>

#include "pm.h"
#include "smemc.h"

/*
 * 16 KiB, and it cannot be more: the vendor register map puts the CIF
 * block at chip offset 0x4000, directly above the SRAM, so a 32 KiB SRAM
 * would overlap its IFMODE and BOOTMODE registers. SAMCOP, which does
 * have 32 KiB, spaces its blocks 64 KiB apart and leaves room for it.
 *
 * Do not probe above this window to check the size. There is no memory to
 * find there, and a write lands on IFMODE, the register selecting SRAM
 * boot mode versus VLIO bus mode, which would wedge the chip's bus. The
 * same applies to poking /dev/mem above 16 KiB.
 */
#define H2200_SRAM_SIZE		SZ_16K

/*
 * Byte offsets into the captured bootloader image, translated to u32
 * word indices (the image is only ever accessed word-at-a-time).
 */
#define BOOTLDR_SIG_BRANCH	(0x000 / 4)	/* "b 0x80", HTC's entry jump */
#define BOOTLDR_SIG_MAGIC	(0x040 / 4)	/* "ECEC" ASCII tag */
#define BOOTLDR_THUNK		(0x75c / 4)	/* unused tail we patch */

#define BOOTLDR_SIG_BRANCH_VAL	0xea00001eU
#define BOOTLDR_SIG_MAGIC_VAL	0x43454345U	/* "ECEC", little-endian */

/*
 * Position-independent 3-instruction resume thunk, patched into the
 * bootloader image at BOOTLDR_THUNK before every SRAM copy-back:
 *
 *	ldr	r0, [pc, #0]	; r0 = PSPR's address (the literal below)
 *	ldr	pc, [r0]	; pc = *PSPR, i.e. __pa_symbol(cpu_resume)
 *	.word	0x40f00008	; PSPR
 *
 * Verbatim from the old out-of-tree h2200_patch_fw() -- method (2) of the
 * two the original driver's header comment documents ("store a
 * position-independent code fragment into SRAM that jumps to the resume
 * function"), chosen there over patching the kernel's own load address
 * as the safer of the two.
 */
#define THUNK_LDR_R0_PC		0xe59f0000U	/* ldr r0, [pc, #0] */
#define THUNK_LDR_PC_R0		0xe590f000U	/* ldr pc, [r0] */
#define THUNK_PSPR_ADDR		0x40f00008U	/* PSPR physical address */

struct h2200_pm {
	struct device *dev;
	void __iomem *sram;
	struct gen_pool *sram_pool;
	u32 *bootloader;
	u32 thunk_orig[3];
	bool bootloader_valid;
	struct completion fw_done;
};

/* Single-instance board driver -- same shape as sharpsl_pm.c's sharpsl_pm. */
static struct h2200_pm *h2200_pm;

static void h2200_pm_patch_bootloader(struct h2200_pm *hpm)
{
	hpm->thunk_orig[0] = hpm->bootloader[BOOTLDR_THUNK + 0];
	hpm->thunk_orig[1] = hpm->bootloader[BOOTLDR_THUNK + 1];
	hpm->thunk_orig[2] = hpm->bootloader[BOOTLDR_THUNK + 2];

	hpm->bootloader[BOOTLDR_THUNK + 0] = THUNK_LDR_R0_PC;
	hpm->bootloader[BOOTLDR_THUNK + 1] = THUNK_LDR_PC_R0;
	hpm->bootloader[BOOTLDR_THUNK + 2] = THUNK_PSPR_ADDR;
}

/*
 * Undo the patch for a reboot. The thunk jumps through PSPR, which is armed
 * with a valid target only by pxa_pm_prepare() on the suspend path. A reboot
 * that re-enters the thunk at physical 0 would jump to whatever PSPR holds
 * (its power-on content on a unit that never suspended) and hang before any
 * exception vectors exist. A reboot means "start over", so restore the
 * captured bootloader's own cold-boot entry instead.
 */
static void h2200_pm_unpatch_bootloader(struct h2200_pm *hpm)
{
	hpm->bootloader[BOOTLDR_THUNK + 0] = hpm->thunk_orig[0];
	hpm->bootloader[BOOTLDR_THUNK + 1] = hpm->thunk_orig[1];
	hpm->bootloader[BOOTLDR_THUNK + 2] = hpm->thunk_orig[2];
}

static bool h2200_pm_bootloader_signed(struct h2200_pm *hpm)
{
	return hpm->bootloader[BOOTLDR_SIG_BRANCH] == BOOTLDR_SIG_BRANCH_VAL &&
	       hpm->bootloader[BOOTLDR_SIG_MAGIC] == BOOTLDR_SIG_MAGIC_VAL;
}

static void h2200_pm_fw_cb(const struct firmware *fw, void *context)
{
	struct h2200_pm *hpm = context;

	if (!fw) {
		dev_err(hpm->dev,
			"no bootloader in SRAM and h2200_bootloader.bin firmware request failed -- suspend will stay disabled\n");
		goto out;
	}

	memcpy(hpm->bootloader, fw->data, min_t(size_t, fw->size, H2200_SRAM_SIZE));
	release_firmware(fw);

	h2200_pm_patch_bootloader(hpm);
	hpm->bootloader_valid = true;
	dev_info(hpm->dev, "captured bootloader from h2200_bootloader.bin firmware, suspend enabled\n");

out:
	complete_all(&hpm->fw_done);
}

/*
 * Copy the bootloader image back into SRAM, patched (the resume thunk) for
 * suspend or original for reboot.
 *
 * Everything about resuming or warm-resetting this board goes through the
 * three words at BOOTLDR_THUNK: reset starts the CPU at physical 0, which is
 * this window.
 */
static void h2200_pm_copy_to_sram(struct h2200_pm *hpm)
{
	memcpy_toio(hpm->sram, hpm->bootloader, H2200_SRAM_SIZE);
}

/*
 * platform_suspend_ops.begin(): the earliest hook available, run before
 * any device is quiesced. Replaces the 2007 driver's equivalent check in
 * .enter(), which used to discover a missing bootloader only after every
 * device in the system had already suspended.
 */
static int h2200_pm_begin(suspend_state_t state)
{
	if (!h2200_pm || !h2200_pm->bootloader_valid) {
		pr_err("h2200-pm: no valid bootloader captured; refusing to suspend\n");
		return -EBUSY;
	}

	return 0;
}

/*
 * platform_suspend_ops.prepare(): devices are quiesced by now. Copy the
 * patched bootloader back into SRAM -- cheap insurance against anything
 * that may have clobbered it since boot -- then hand off to the core
 * PXA25x prepare hook, which stashes the real resume address in PSPR for
 * our thunk to pick up.
 */
static int h2200_pm_prepare(void)
{
	/* remove() clears h2200_pm, so a suspend may race with unbinding. */
	if (!h2200_pm)
		return -ENODEV;

	h2200_pm_copy_to_sram(h2200_pm);

	return pxa_pm_prepare();
}

/*
 * platform_suspend_ops.enter(): the bootloader's cold-boot path
 * reprograms MSC0-2 (static memory timings for HAMCOP on CS0 and the
 * MediaQ 1178 on CS1) before our resume thunk ever runs, so whatever
 * h2200.c's fix_msc() established at boot is gone by the time Linux
 * device resume starts touching CS-mapped hardware. Save and restore
 * around the actual sleep so those three registers are back to what
 * this boot's fix_msc() set, as early as possible -- IRQs are still off
 * here, before any driver's own resume runs.
 */
static int h2200_pm_enter(suspend_state_t state)
{
	u32 msc0, msc1, msc2;
	int ret;

	msc0 = __raw_readl(MSC0);
	msc1 = __raw_readl(MSC1);
	msc2 = __raw_readl(MSC2);

	ret = pxa_pm_enter(state);

	__raw_writel(msc0, MSC0);
	__raw_writel(msc1, MSC1);
	__raw_writel(msc2, MSC2);

	return ret;
}

static const struct platform_suspend_ops h2200_suspend_ops = {
	.begin		= h2200_pm_begin,
	.prepare	= h2200_pm_prepare,
	.enter		= h2200_pm_enter,
	.finish		= pxa_pm_finish,
	.valid		= suspend_valid_only_mem,
};

/*
 * Re-copy the bootloader, unpatched, before a warm reset; see
 * h2200_pm_unpatch_bootloader() for why the resume thunk must not be left in
 * place. This sys-off handler is the hook point for the reset path, as the
 * platform driver has no .shutdown of its own.
 *
 * Must run before hamcop_restart_prepare() (drivers/mfd/hamcop.c), its
 * sibling handler on the same chain: that one flips HAMCOP's IFMODE to
 * SRAM mode, which changes the bus timing HAMCOP applies to every CS0
 * cycle, the SRAM window included, and no write timing is specified for
 * that mode. Both handlers sit on independent platform devices, so only
 * priority fixes their relative order; SYS_OFF_PRIO_HIGH makes this one
 * run first regardless of probe or link order.
 *
 * SYS_OFF_MODE_RESTART_PREPARE is the right mode. kernel_restart() calls
 * kernel_restart_prepare(), whose last statement is device_shutdown(), and
 * only then fires this chain, so every driver's .shutdown has already run,
 * including the quiesce of the hamcop-sdi DMA engine that shares this
 * window. The handlers on this chain may also sleep. SYS_OFF_MODE_RESTART
 * is an atomic notifier called with interrupts off after syscore_shutdown(),
 * which is no place for a 16 KiB write across a chip select.
 */
static int h2200_pm_restart(struct sys_off_data *data)
{
	struct h2200_pm *hpm = data->cb_data;

	if (hpm->bootloader_valid) {
		h2200_pm_unpatch_bootloader(hpm);
		h2200_pm_copy_to_sram(hpm);
	}

	return NOTIFY_DONE;
}

/*
 * Publish the SRAM window as a genalloc pool for HAMCOP's DMA-capable
 * function drivers (see include/linux/platform_data/h2200-sram.h).
 *
 * gen_pool_dma_alloc() hands back both a CPU pointer and a "DMA
 * address". The latter is genuinely just the bus address of an MMIO
 * aperture -- there is no IOMMU, no cache to worry about and nothing to
 * map, so the physical address of the window is the right value and no
 * dma_map_*()/dma_sync_*() is involved anywhere. meson-gx-mmc.c's
 * dram_access_quirk path does exactly the same thing with its
 * controller-internal SRAM.
 *
 * The pointer this hands out is an ioremap() cookie, so consumers must
 * use memcpy_toio()/memcpy_fromio() rather than plain memcpy().
 *
 * Granularity is one SD/MMC block, which is the only thing anyone
 * allocates here and keeps the bitmap trivially small (32 chunks).
 */
#define H2200_SRAM_POOL_ORDER	9	/* 512 bytes */

static int h2200_pm_add_sram_pool(struct h2200_pm *hpm, struct resource *res)
{
	struct gen_pool *pool;
	int ret;

	pool = devm_gen_pool_create(hpm->dev, H2200_SRAM_POOL_ORDER,
				    NUMA_NO_NODE, "hamcop-sram");
	if (IS_ERR(pool))
		return PTR_ERR(pool);

	ret = gen_pool_add_virt(pool, (unsigned long)hpm->sram, res->start,
				H2200_SRAM_SIZE, NUMA_NO_NODE);
	if (ret)
		return ret;

	hpm->sram_pool = pool;

	return 0;
}

/*
 * Not gen_pool_get()-based: that looks up pools by owning
 * device, and the owner here is "hamcop-sram", which is a sibling of the
 * HAMCOP MFD cells that need the SRAM, not an ancestor they could find it
 * through. A plain accessor is the smallest thing that bridges that.
 */
struct gen_pool *h2200_sram_pool(void)
{
	return h2200_pm ? h2200_pm->sram_pool : NULL;
}
EXPORT_SYMBOL_GPL(h2200_sram_pool);

static int h2200_pm_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct h2200_pm *hpm;
	int ret;

	/*
	 * The boot image signature and the thunk offset are the h2200's HTC
	 * bootloader's, not anything HAMCOP defines.
	 */
	if (!of_machine_is_compatible("hp,ipaq-h2200"))
		return -ENODEV;

	hpm = devm_kzalloc(&pdev->dev, sizeof(*hpm), GFP_KERNEL);
	if (!hpm)
		return -ENOMEM;

	/*
	 * Zeroed, not plain kmalloc: on the firmware path below,
	 * h2200_pm_fw_cb() copies only min(fw->size, H2200_SRAM_SIZE) bytes
	 * in, while both consumers (h2200_pm_prepare() and h2200_pm_restart())
	 * memcpy_toio() the full 16 KiB back out. A short blob would otherwise
	 * push uninitialised kernel heap into MMIO on every suspend and warm
	 * reset. The SRAM-capture path below overwrites all 16 KiB anyway, so
	 * this costs one clear at probe and nothing else.
	 */
	hpm->bootloader = devm_kzalloc(&pdev->dev, H2200_SRAM_SIZE, GFP_KERNEL);
	if (!hpm->bootloader)
		return -ENOMEM;

	hpm->dev = &pdev->dev;
	init_completion(&hpm->fw_done);

	hpm->sram = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(hpm->sram))
		return PTR_ERR(hpm->sram);

	memcpy_fromio(hpm->bootloader, hpm->sram, H2200_SRAM_SIZE);

	if (h2200_pm_bootloader_signed(hpm)) {
		h2200_pm_patch_bootloader(hpm);
		hpm->bootloader_valid = true;
		complete_all(&hpm->fw_done);
		dev_info(&pdev->dev, "captured valid HTC bootloader from SRAM\n");
	} else {
		dev_warn(&pdev->dev,
			 "no valid HTC bootloader signature in SRAM, requesting h2200_bootloader.bin\n");
		ret = request_firmware_nowait(THIS_MODULE, true,
					      "h2200_bootloader.bin", &pdev->dev,
					      GFP_KERNEL, hpm, h2200_pm_fw_cb);
		if (ret) {
			dev_err(&pdev->dev,
				"request_firmware_nowait() failed: %d\n", ret);
			complete_all(&hpm->fw_done);
		}
	}

	platform_set_drvdata(pdev, hpm);

	/*
	 * Only after the bootloader has been captured out of SRAM -- the
	 * pool's whole point is to let other drivers overwrite the window.
	 * A failure here is not fatal: it just means no HAMCOP DMA, and
	 * suspend/resume, which is this driver's actual job, is unaffected.
	 */
	ret = h2200_pm_add_sram_pool(hpm, res);
	if (ret)
		dev_warn(&pdev->dev,
			 "failed to create SRAM pool, HAMCOP DMA unavailable: %d\n",
			 ret);

	h2200_pm = hpm;

	ret = devm_register_sys_off_handler(&pdev->dev,
					    SYS_OFF_MODE_RESTART_PREPARE,
					    SYS_OFF_PRIO_HIGH,
					    h2200_pm_restart, hpm);
	if (ret)
		dev_warn(&pdev->dev,
			 "failed to register restart handler: %d\n", ret);

	suspend_set_ops(&h2200_suspend_ops);

	return 0;
}

/*
 * The suspend ops stay registered, so h2200_pm_begin() is what refuses
 * once the captured image is gone. Only reachable if the HAMCOP MFD fails
 * its probe after adding this sub-device. A firmware request still in
 * flight holds a pointer into the devm allocations that are about to be
 * freed, so wait for its callback first.
 */
static void h2200_pm_remove(struct platform_device *pdev)
{
	struct h2200_pm *hpm = platform_get_drvdata(pdev);

	h2200_pm = NULL;
	wait_for_completion(&hpm->fw_done);
}

static struct platform_driver h2200_pm_driver = {
	.probe	= h2200_pm_probe,
	.remove	= h2200_pm_remove,
	.driver	= {
		.name = "hamcop-sram",
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(h2200_pm_driver);
