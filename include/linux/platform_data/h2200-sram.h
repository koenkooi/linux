/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Access to HAMCOP's on-chip SRAM on the HP iPAQ h2200.
 *
 * HAMCOP's DMA engine can only reach the chip's own 16 KiB SRAM, never
 * DRAM, so every DMA-capable HAMCOP function driver has to stage its
 * transfers through that SRAM. The window is owned by h2200_pm.c rather
 * than by the HAMCOP MFD, because it is also physical address 0 -- the
 * CPU's reset vector, holding the HTC bootloader that h2200_pm.c
 * rewrites before every suspend and warm reset. Both cannot ioremap the
 * same range, so the MFD's resource starts above it.
 *
 * h2200_pm.c therefore publishes the window as a genalloc pool and this
 * is the only way to reach it from a driver. Callers must use
 * gen_pool_dma_alloc() (or a sibling) rather than assuming the whole
 * pool is theirs: HAMCOP's NAND controller is a potential second DMA
 * user of the same SRAM.
 *
 * Contents are scratch between boot and the next suspend or reset:
 * h2200_pm.c keeps its own RAM copy of the bootloader and only ever writes
 * the SRAM afterwards, so scribbling on it is safe as long as it has
 * stopped by the time devices are suspended.
 */
#ifndef _LINUX_PLATFORM_DATA_H2200_SRAM_H
#define _LINUX_PLATFORM_DATA_H2200_SRAM_H

struct gen_pool;

#if IS_ENABLED(CONFIG_MACH_H2200)
/**
 * h2200_sram_pool - get the genalloc pool backing HAMCOP's SRAM
 *
 * Returns the pool, or NULL if h2200_pm has not probed yet or this is
 * not an h2200. A NULL return is not an error for a caller that can
 * fall back to PIO; it should be treated as "DMA unavailable".
 */
struct gen_pool *h2200_sram_pool(void);
#else
static inline struct gen_pool *h2200_sram_pool(void)
{
	return NULL;
}
#endif

#endif /* _LINUX_PLATFORM_DATA_H2200_SRAM_H */
