// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel SA-1110 static memory controller
 *
 * Programs the per-chip-select access timing (MSC0/MSC1/MSC2) its device
 * tree children ask for, then populates those children. The order is the
 * point of the driver: on nCS3 to nCS5 the controller can be placed in
 * variable latency I/O mode, where a cycle ends only when the slave
 * asserts RDY and there is no bus error timeout, so the first access to
 * a device whose chip select is still at its reset timing can stall the
 * CPU inside one load or store for as long as the slave holds RDY low --
 * long enough, on this SoC, to starve DRAM refresh as well.
 *
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/math.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/types.h>

#define SA1110_SMC_NUM_CS		6
#define SA1110_SMC_CS_PER_MSC		2
#define SA1110_SMC_NUM_MSC		(SA1110_SMC_NUM_CS / SA1110_SMC_CS_PER_MSC)

/*
 * One 16-bit half of an MSC register configures one chip select, the low
 * half for the even one of the pair. Five fields fill the half with no
 * gap. This is not the PXA2xx layout, which places the same five fields
 * at different offsets and adds a return-data-buffering bit.
 */
#define SA1110_MSC_RT			GENMASK(1, 0)
#define SA1110_MSC_RBW			BIT(2)
#define SA1110_MSC_RDF			GENMASK(7, 3)
#define SA1110_MSC_RDN			GENMASK(12, 8)
#define SA1110_MSC_RRR			GENMASK(15, 13)

#define SA1110_MSC_CS_MASK		GENMASK(15, 0)
#define SA1110_MSC_CS_BITS		16

#define SA1110_MSC_RT_NONBURST_ROM	0
#define SA1110_MSC_RT_SRAM_VLIO		1
#define SA1110_MSC_RT_BURST4_ROM	2
#define SA1110_MSC_RT_BURST8_ROM	3

/* RDF and RDN hold (cycles - 1) in five bits, RRR holds cycle pairs in three. */
#define SA1110_SMC_MAX_ACCESS_CYCLES	32
#define SA1110_SMC_MAX_RECOVERY_CYCLES	14

/* A child's reg entry is <chip-select> <offset> <size>. */
#define SA1110_SMC_OF_REG_CELLS		3

struct sa1110_smc {
	struct device *dev;
	void __iomem *msc[SA1110_SMC_NUM_MSC];
	unsigned long cs_configured;
};

struct sa1110_smc_devtype {
	const char *name;
	u32 rt;
	u32 cs_first;
	u32 cs_last;
};

/*
 * "sram" and "vlio" share the RT=1 encoding, which the controller reads
 * as byte-writable SRAM on nCS0 to nCS2 and as variable latency I/O on
 * nCS3 to nCS5. Naming them apart lets a device tree state which one it
 * means and lets this driver reject the combination that does not exist,
 * rather than silently running a device on the wrong bus protocol.
 */
static const struct sa1110_smc_devtype sa1110_smc_devtypes[] = {
	{ "nonburst-rom", SA1110_MSC_RT_NONBURST_ROM, 0, 5 },
	{ "sram",	  SA1110_MSC_RT_SRAM_VLIO,    0, 2 },
	{ "vlio",	  SA1110_MSC_RT_SRAM_VLIO,    3, 5 },
	{ "burst4-rom",	  SA1110_MSC_RT_BURST4_ROM,   0, 5 },
	{ "burst8-rom",	  SA1110_MSC_RT_BURST8_ROM,   0, 5 },
};

static const char * const sa1110_smc_timing_props[] = {
	"intel,sa1110-smc-type",
	"intel,sa1110-smc-first-access-cycles",
	"intel,sa1110-smc-next-access-cycles",
	"intel,sa1110-smc-recovery-cycles",
};

static bool sa1110_smc_describes_timing(struct device_node *np)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sa1110_smc_timing_props); i++)
		if (of_property_present(np, sa1110_smc_timing_props[i]))
			return true;

	return false;
}

static int sa1110_smc_read_cycles(struct sa1110_smc *smc,
				  struct device_node *np, const char *prop,
				  u32 min, u32 max, u32 *out)
{
	int ret;

	ret = of_property_read_u32(np, prop, out);
	if (ret)
		return dev_err_probe(smc->dev, ret, "%pOF: missing %s\n",
				     np, prop);

	if (*out < min || *out > max)
		return dev_err_probe(smc->dev, -ERANGE,
				     "%pOF: %s is %u, outside %u..%u\n",
				     np, prop, *out, min, max);

	return 0;
}

static int sa1110_smc_timing_word(struct sa1110_smc *smc,
				  struct device_node *np, u32 cs, u32 *out)
{
	const struct sa1110_smc_devtype *type = NULL;
	u32 first, next, recovery, width;
	const char *name;
	unsigned int i;
	int ret;

	ret = of_property_read_string(np, "intel,sa1110-smc-type", &name);
	if (ret)
		return dev_err_probe(smc->dev, ret,
				     "%pOF: missing intel,sa1110-smc-type\n",
				     np);

	for (i = 0; i < ARRAY_SIZE(sa1110_smc_devtypes); i++) {
		if (!strcmp(name, sa1110_smc_devtypes[i].name)) {
			type = &sa1110_smc_devtypes[i];
			break;
		}
	}

	if (!type)
		return dev_err_probe(smc->dev, -EINVAL,
				     "%pOF: unknown device type \"%s\"\n",
				     np, name);

	if (cs < type->cs_first || cs > type->cs_last)
		return dev_err_probe(smc->dev, -EINVAL,
				     "%pOF: device type \"%s\" does not exist on nCS%u\n",
				     np, name, cs);

	ret = of_property_read_u32(np, "bank-width", &width);
	if (ret)
		return dev_err_probe(smc->dev, ret, "%pOF: missing bank-width\n",
				     np);

	if (width != 2 && width != 4)
		return dev_err_probe(smc->dev, -EINVAL,
				     "%pOF: bank-width %u, this controller drives 2 or 4\n",
				     np, width);

	ret = sa1110_smc_read_cycles(smc, np,
				     "intel,sa1110-smc-first-access-cycles",
				     1, SA1110_SMC_MAX_ACCESS_CYCLES, &first);
	if (ret)
		return ret;

	ret = sa1110_smc_read_cycles(smc, np,
				     "intel,sa1110-smc-next-access-cycles",
				     1, SA1110_SMC_MAX_ACCESS_CYCLES, &next);
	if (ret)
		return ret;

	ret = sa1110_smc_read_cycles(smc, np,
				     "intel,sa1110-smc-recovery-cycles",
				     0, SA1110_SMC_MAX_RECOVERY_CYCLES,
				     &recovery);
	if (ret)
		return ret;

	*out = FIELD_PREP(SA1110_MSC_RT, type->rt) |
	       FIELD_PREP(SA1110_MSC_RBW, width == 2) |
	       FIELD_PREP(SA1110_MSC_RDF, first - 1) |
	       FIELD_PREP(SA1110_MSC_RDN, next - 1) |
	       FIELD_PREP(SA1110_MSC_RRR, DIV_ROUND_UP(recovery, 2));

	return 0;
}

static int sa1110_smc_configure_cs(struct sa1110_smc *smc,
				   struct device_node *np, u32 cs)
{
	void __iomem *reg = smc->msc[cs / SA1110_SMC_CS_PER_MSC];
	unsigned int shift = (cs % SA1110_SMC_CS_PER_MSC) * SA1110_MSC_CS_BITS;
	u32 timing, val;
	int ret;

	ret = sa1110_smc_timing_word(smc, np, cs, &timing);
	if (ret)
		return ret;

	val = readl(reg);
	val &= ~(SA1110_MSC_CS_MASK << shift);
	val |= timing << shift;
	writel(val, reg);

	/*
	 * Read the register back before returning: the next access to this
	 * chip select may be the caller's own probe reaching into a VLIO
	 * window, and that access must not overtake the timing write on the
	 * write buffer.
	 */
	readl(reg);

	dev_dbg(smc->dev, "%pOF: nCS%u timing %#06x\n", np, cs, timing);

	return 0;
}

static int sa1110_smc_setup_child(struct sa1110_smc *smc,
				  struct device_node *np)
{
	unsigned long referenced = 0;
	int num_regs, i, ret;
	u32 cs;

	if (!sa1110_smc_describes_timing(np)) {
		dev_dbg(smc->dev, "%pOF: no timing described, left as found\n",
			np);
		return 0;
	}

	num_regs = of_property_count_elems_of_size(np, "reg",
						   SA1110_SMC_OF_REG_CELLS *
						   sizeof(u32));
	if (num_regs <= 0)
		return dev_err_probe(smc->dev, num_regs ?: -EINVAL,
				     "%pOF: no usable reg property\n", np);

	for (i = 0; i < num_regs; i++) {
		ret = of_property_read_u32_index(np, "reg",
						 i * SA1110_SMC_OF_REG_CELLS,
						 &cs);
		if (ret)
			return dev_err_probe(smc->dev, ret,
					     "%pOF: cannot read reg entry %d\n",
					     np, i);

		if (cs >= SA1110_SMC_NUM_CS)
			return dev_err_probe(smc->dev, -EINVAL,
					     "%pOF: nCS%u does not exist\n",
					     np, cs);

		if (smc->cs_configured & BIT(cs))
			return dev_err_probe(smc->dev, -EEXIST,
					     "%pOF: nCS%u already configured\n",
					     np, cs);

		referenced |= BIT(cs);
	}

	/*
	 * A device spanning several chip selects gets the same timing on
	 * each of them: the properties describe one device on one bus, and
	 * the controller has no way to express anything else.
	 */
	for_each_set_bit(cs, &referenced, SA1110_SMC_NUM_CS) {
		ret = sa1110_smc_configure_cs(smc, np, cs);
		if (ret)
			return ret;

		smc->cs_configured |= BIT(cs);
	}

	return 0;
}

static int sa1110_smc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sa1110_smc *smc;
	unsigned int i;
	int ret;

	smc = devm_kzalloc(dev, sizeof(*smc), GFP_KERNEL);
	if (!smc)
		return -ENOMEM;

	smc->dev = dev;

	for (i = 0; i < SA1110_SMC_NUM_MSC; i++) {
		smc->msc[i] = devm_platform_ioremap_resource(pdev, i);
		if (IS_ERR(smc->msc[i]))
			return PTR_ERR(smc->msc[i]);
	}

	/*
	 * Every chip select is programmed before any child is instantiated,
	 * and one child the tree describes badly aborts the whole probe
	 * rather than being skipped: a device created on a chip select this
	 * driver could not program is precisely the access that hangs.
	 */
	for_each_available_child_of_node_scoped(dev->of_node, child) {
		ret = sa1110_smc_setup_child(smc, child);
		if (ret)
			return ret;
	}

	platform_set_drvdata(pdev, smc);

	return of_platform_default_populate(dev->of_node, NULL, dev);
}

static const struct of_device_id sa1110_smc_of_match[] = {
	{ .compatible = "intel,sa1110-smc" },
	{ }
};

static struct platform_driver sa1110_smc_driver = {
	.probe = sa1110_smc_probe,
	.driver = {
		.name = "sa1110-smc",
		.of_match_table = sa1110_smc_of_match,
		.suppress_bind_attrs = true,
	},
};

/*
 * subsys_initcall, and no remove(): every consumer of this controller is
 * a child it populates itself, and unbinding it would leave those
 * children live on chip selects whose timing could then be reprogrammed
 * out from under them.
 */
static int __init sa1110_smc_init(void)
{
	return platform_driver_register(&sa1110_smc_driver);
}
subsys_initcall(sa1110_smc_init);
