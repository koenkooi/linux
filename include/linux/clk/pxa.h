/* SPDX-License-Identifier: GPL-2.0-only */

#include <linux/compiler.h>
#include <linux/init.h>
#include <linux/types.h>

extern int pxa25x_clocks_init(void __iomem *regs);
extern int pxa27x_clocks_init(void __iomem *regs);
extern int pxa3xx_clocks_init(void __iomem *regs, void __iomem *oscc_reg);

/*
 * Board init calls this to stop pxa2xx_cpll_change() from writing the named
 * MDREFR bits on a frequency change, for a companion ASIC wired to the
 * corresponding SDCLK output that misbehaves when that bit toggles.
 */
void __init pxa2xx_clk_exclude_mdrefr_db2(u32 mask);

#ifdef CONFIG_PXA3xx
extern unsigned	pxa3xx_get_clk_frequency_khz(int);
extern void pxa3xx_clk_update_accr(u32 disable, u32 enable, u32 xclkcfg, u32 mask);
#else
#define pxa3xx_get_clk_frequency_khz(x)		(0)
#define pxa3xx_clk_update_accr(disable, enable, xclkcfg, mask) do { } while (0)
#endif
