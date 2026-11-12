/*
 * Support for the GPIO/IRQ expander chips present on several HTC phones.
 * These are implemented in CPLD chips present on the board.
 *
 * Copyright (c) 2007 Kevin O'Connor <kevin@koconnor.net>
 * Copyright (c) 2007 Philipp Zabel <philipp.zabel@gmail.com>
 *
 * This file may be distributed under the terms of the GNU GPL license.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/spinlock.h>
#include <linux/platform_data/gpio-htc-egpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio/driver.h>

struct egpio_chip {
	int              reg_start;
	int              cached_values;
	unsigned long    is_out;
	struct device    *dev;
	struct gpio_chip chip;
};

struct egpio_info {
	spinlock_t        lock;

	/* iomem info */
	void __iomem      *base_addr;
	int               bus_shift;	/* byte shift */
	int               reg_shift;	/* bit shift */
	int               reg_mask;

	/* irq info */
	int               ack_register;
	int               ack_write;
	u16               irqs_enabled;
	uint              irq_start;
	int               nirqs;
	uint              chained_irq;

	/* egpio info */
	int               nchips;
	struct egpio_chip chip[] __counted_by(nchips);
};

static inline void egpio_writew(u16 value, struct egpio_info *ei, int reg)
{
	writew(value, ei->base_addr + (reg << ei->bus_shift));
}

static inline u16 egpio_readw(struct egpio_info *ei, int reg)
{
	return readw(ei->base_addr + (reg << ei->bus_shift));
}

/*
 * IRQs
 */

static inline void ack_irqs(struct egpio_info *ei)
{
	egpio_writew(ei->ack_write, ei, ei->ack_register);
	pr_debug("EGPIO ack - write %x to base+%x\n",
			ei->ack_write, ei->ack_register << ei->bus_shift);
}

static void egpio_ack(struct irq_data *data)
{
}

/* There does not appear to be a way to proactively mask interrupts
 * on the egpio chip itself.  So, we simply ignore interrupts that
 * aren't desired. */
static void egpio_mask(struct irq_data *data)
{
	struct egpio_info *ei = irq_data_get_irq_chip_data(data);
	ei->irqs_enabled &= ~(1 << (data->irq - ei->irq_start));
	pr_debug("EGPIO mask %d %04x\n", data->irq, ei->irqs_enabled);
}

static void egpio_unmask(struct irq_data *data)
{
	struct egpio_info *ei = irq_data_get_irq_chip_data(data);
	ei->irqs_enabled |= 1 << (data->irq - ei->irq_start);
	pr_debug("EGPIO unmask %d %04x\n", data->irq, ei->irqs_enabled);
}

static struct irq_chip egpio_muxed_chip = {
	.name		= "htc-egpio",
	.irq_ack	= egpio_ack,
	.irq_mask	= egpio_mask,
	.irq_unmask	= egpio_unmask,
};

static void egpio_handler(struct irq_desc *desc)
{
	struct egpio_info *ei = irq_desc_get_handler_data(desc);
	int irqpin;

	/* Read current pins. */
	unsigned long readval = egpio_readw(ei, ei->ack_register);
	pr_debug("IRQ reg: %x\n", (unsigned int)readval);
	/* Ack/unmask interrupts. */
	ack_irqs(ei);
	/* Process all set pins. */
	readval &= ei->irqs_enabled;
	for_each_set_bit(irqpin, &readval, ei->nirqs) {
		/* Run irq handler */
		pr_debug("got IRQ %d\n", irqpin);
		generic_handle_irq(ei->irq_start + irqpin);
	}
}

static inline int egpio_pos(struct egpio_info *ei, int bit)
{
	return bit >> ei->reg_shift;
}

static inline int egpio_bit(struct egpio_info *ei, int bit)
{
	return 1 << (bit & ((1 << ei->reg_shift)-1));
}

/*
 * Input pins
 */

static int egpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct egpio_chip *egpio;
	struct egpio_info *ei;
	unsigned           bit;
	int                reg;
	int                value;

	pr_debug("egpio_get_value(%d)\n", chip->base + offset);

	egpio = gpiochip_get_data(chip);
	ei    = dev_get_drvdata(egpio->dev);
	bit   = egpio_bit(ei, offset);
	reg   = egpio->reg_start + egpio_pos(ei, offset);

	if (test_bit(offset, &egpio->is_out)) {
		return !!(egpio->cached_values & (1 << offset));
	} else {
		value = egpio_readw(ei, reg);
		pr_debug("readw(%p + %x) = %x\n",
			 ei->base_addr, reg << ei->bus_shift, value);
		return !!(value & bit);
	}
}

static int egpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct egpio_chip *egpio;

	egpio = gpiochip_get_data(chip);
	return test_bit(offset, &egpio->is_out) ? -EINVAL : 0;
}


/*
 * Output pins
 */

static int egpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	unsigned long     flag;
	struct egpio_chip *egpio;
	struct egpio_info *ei;
	int               pos;
	int               reg;
	int               shift;

	pr_debug("egpio_set(%s, %d(%d), %d)\n",
			chip->label, offset, offset+chip->base, value);

	egpio = gpiochip_get_data(chip);
	ei    = dev_get_drvdata(egpio->dev);
	pos   = egpio_pos(ei, offset);
	reg   = egpio->reg_start + pos;
	shift = pos << ei->reg_shift;

	pr_debug("egpio %s: reg %d = 0x%04x\n", value ? "set" : "clear",
			reg, (egpio->cached_values >> shift) & ei->reg_mask);

	spin_lock_irqsave(&ei->lock, flag);
	if (value)
		egpio->cached_values |= (1 << offset);
	else
		egpio->cached_values &= ~(1 << offset);
	egpio_writew((egpio->cached_values >> shift) & ei->reg_mask, ei, reg);
	spin_unlock_irqrestore(&ei->lock, flag);

	return 0;
}

static int egpio_direction_output(struct gpio_chip *chip,
					unsigned offset, int value)
{
	struct egpio_chip *egpio;

	egpio = gpiochip_get_data(chip);
	if (test_bit(offset, &egpio->is_out))
		return egpio_set(chip, offset, value);

	return -EINVAL;
}

static int egpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
	struct egpio_chip *egpio;

	egpio = gpiochip_get_data(chip);

	if (test_bit(offset, &egpio->is_out))
		return GPIO_LINE_DIRECTION_OUT;

	return GPIO_LINE_DIRECTION_IN;
}

static void egpio_write_cache(struct egpio_info *ei)
{
	int               i;
	struct egpio_chip *egpio;
	int               shift;

	for (i = 0; i < ei->nchips; i++) {
		egpio = &(ei->chip[i]);
		if (!egpio->is_out)
			continue;

		for (shift = 0; shift < egpio->chip.ngpio;
				shift += (1<<ei->reg_shift)) {

			int reg = egpio->reg_start + egpio_pos(ei, shift);

			if (!((egpio->is_out >> shift) & ei->reg_mask))
				continue;

			pr_debug("EGPIO: setting %x to %x, was %x\n", reg,
				(egpio->cached_values >> shift) & ei->reg_mask,
				egpio_readw(ei, reg));

			egpio_writew((egpio->cached_values >> shift)
					& ei->reg_mask, ei, reg);
		}
	}
}


/*
 * Setup
 */

/*
 * Build the descriptor a legacy board file passes as platform_data from the
 * devicetree node, so both probe paths share the register geometry code.
 * irq_base/num_irqs have no devicetree equivalent: they are raw Linux IRQ
 * numbers and this driver has no irq_domain, see the chained IRQ handling
 * in egpio_probe().
 */
static struct htc_egpio_platform_data *
egpio_parse_dt(struct device *dev, struct fwnode_handle ***fwnodes)
{
	struct htc_egpio_platform_data *pdata;
	struct fwnode_handle **nodes;
	struct htc_egpio_chip *chips;
	struct device_node *child;
	int nchips, i = 0;
	u32 val;
	int ret;

	nchips = of_get_available_child_count(dev->of_node);
	if (!nchips)
		return ERR_PTR(dev_err_probe(dev, -EINVAL,
					     "no gpio-chip@ child node\n"));

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	chips = devm_kcalloc(dev, nchips, sizeof(*chips), GFP_KERNEL);
	if (!chips)
		return ERR_PTR(-ENOMEM);

	nodes = devm_kcalloc(dev, nchips, sizeof(*nodes), GFP_KERNEL);
	if (!nodes)
		return ERR_PTR(-ENOMEM);

	ret = of_property_read_u32(dev->of_node, "htc,reg-width", &val);
	if (ret)
		return ERR_PTR(dev_err_probe(dev, ret,
					     "missing htc,reg-width\n"));
	pdata->reg_width = val;

	ret = of_property_read_u32(dev->of_node, "htc,bus-width", &val);
	if (ret)
		return ERR_PTR(dev_err_probe(dev, ret,
					     "missing htc,bus-width\n"));
	pdata->bus_width = val;

	if (!of_property_read_u32(dev->of_node, "htc,ack-register", &val))
		pdata->ack_register = val;
	pdata->invert_acks = of_property_read_bool(dev->of_node,
						   "htc,invert-acks");

	for_each_available_child_of_node(dev->of_node, child) {
		ret = of_property_read_u32(child, "reg", &val);
		if (ret)
			goto err_child;
		chips[i].reg_start = val;

		ret = of_property_read_u32(child, "ngpios", &val);
		if (ret)
			goto err_child;
		/* is_out and cached_values are single words indexed by pin */
		if (!val || val > BITS_PER_LONG) {
			ret = -EINVAL;
			goto err_child;
		}
		chips[i].num_gpios = val;

		ret = of_property_read_u32(child, "htc,direction", &val);
		if (ret)
			goto err_child;
		chips[i].direction = val;

		if (!of_property_read_u32(child, "htc,initial-values", &val))
			chips[i].initial_values = val;

		/* Consumers use a phandle, so gpiolib may pick the base. */
		chips[i].gpio_base = -1;
		/* Consumer phandles point at the child, not the parent node. */
		nodes[i] = of_fwnode_handle(child);
		i++;
	}

	pdata->chip = chips;
	pdata->num_chips = nchips;
	*fwnodes = nodes;

	return pdata;

err_child:
	dev_err(dev, "%pOF: bad or missing reg/ngpios/htc,direction\n", child);
	of_node_put(child);
	return ERR_PTR(ret);
}

static int egpio_probe(struct platform_device *pdev)
{
	struct htc_egpio_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct fwnode_handle **fwnodes = NULL;
	struct resource   *res;
	struct egpio_info *ei;
	struct gpio_chip  *chip;
	unsigned int      irq, irq_end;
	int               i;
	int               ret;

	if (!pdata) {
		if (!pdev->dev.of_node)
			return dev_err_probe(&pdev->dev, -EINVAL,
					     "neither platform_data nor a devicetree node\n");

		pdata = egpio_parse_dt(&pdev->dev, &fwnodes);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	/* Initialize ei data structure. */
	ei = devm_kzalloc(&pdev->dev, struct_size(ei, chip, pdata->num_chips), GFP_KERNEL);
	if (!ei)
		return -ENOMEM;

	ei->nchips = pdata->num_chips;

	spin_lock_init(&ei->lock);

	/* Find chained irq */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res)
		ei->chained_irq = res->start;

	/*
	 * A DT-probed chip has no irq_base/num_irqs, so a chained handler
	 * would consume the parent line and ack the chip forever without
	 * dispatching anything.
	 */
	if (ei->chained_irq && !dev_get_platdata(&pdev->dev)) {
		dev_warn(&pdev->dev,
			 "ignoring chained IRQ %u: DT probing has no irq_domain yet, input pins cannot raise interrupts\n",
			 ei->chained_irq);
		ei->chained_irq = 0;
	}

	/* Map egpio chip into virtual address space. */
	ei->base_addr = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ei->base_addr))
		return PTR_ERR(ei->base_addr);

	if ((pdata->bus_width != 16) && (pdata->bus_width != 32))
		return -EINVAL;

	ei->bus_shift = fls(pdata->bus_width - 1) - 3;
	pr_debug("bus_shift = %d\n", ei->bus_shift);

	if ((pdata->reg_width != 8) && (pdata->reg_width != 16))
		return -EINVAL;

	ei->reg_shift = fls(pdata->reg_width - 1);
	pr_debug("reg_shift = %d\n", ei->reg_shift);

	ei->reg_mask = (1 << pdata->reg_width) - 1;

	platform_set_drvdata(pdev, ei);

	for (i = 0; i < ei->nchips; i++) {
		ei->chip[i].reg_start = pdata->chip[i].reg_start;
		ei->chip[i].cached_values = pdata->chip[i].initial_values;
		ei->chip[i].is_out = pdata->chip[i].direction;
		ei->chip[i].dev = &(pdev->dev);
		chip = &(ei->chip[i].chip);
		chip->label = devm_kasprintf(&pdev->dev, GFP_KERNEL,
					     "htc-egpio-%d",
					     i);
		if (!chip->label)
			return -ENOMEM;

		chip->parent          = &pdev->dev;
		if (fwnodes)
			chip->fwnode  = fwnodes[i];
		chip->owner           = THIS_MODULE;
		chip->get             = egpio_get;
		chip->set             = egpio_set;
		chip->direction_input = egpio_direction_input;
		chip->direction_output = egpio_direction_output;
		chip->get_direction   = egpio_get_direction;
		chip->base            = pdata->chip[i].gpio_base;
		chip->ngpio           = pdata->chip[i].num_gpios;

		ret = devm_gpiochip_add_data(&pdev->dev, chip, &ei->chip[i]);
		if (ret)
			return dev_err_probe(&pdev->dev, ret,
					     "failed to register gpiochip %d\n", i);
	}

	/* Set initial pin values */
	egpio_write_cache(ei);

	ei->irq_start = pdata->irq_base;
	ei->nirqs = pdata->num_irqs;
	ei->ack_register = pdata->ack_register;

	if (ei->chained_irq) {
		/* Setup irq handlers */
		ei->ack_write = 0xFFFF;
		if (pdata->invert_acks)
			ei->ack_write = 0;
		irq_end = ei->irq_start + ei->nirqs;
		for (irq = ei->irq_start; irq < irq_end; irq++) {
			irq_set_chip_and_handler(irq, &egpio_muxed_chip,
						 handle_simple_irq);
			irq_set_chip_data(irq, ei);
			irq_clear_status_flags(irq, IRQ_NOREQUEST | IRQ_NOPROBE);
		}
		irq_set_irq_type(ei->chained_irq, IRQ_TYPE_EDGE_RISING);
		irq_set_chained_handler_and_data(ei->chained_irq,
						 egpio_handler, ei);
		ack_irqs(ei);

		device_init_wakeup(&pdev->dev, 1);
	}

	return 0;
}

static int egpio_suspend(struct device *dev)
{
	struct egpio_info *ei = dev_get_drvdata(dev);

	if (ei->chained_irq && device_may_wakeup(dev))
		enable_irq_wake(ei->chained_irq);
	return 0;
}

static int egpio_resume(struct device *dev)
{
	struct egpio_info *ei = dev_get_drvdata(dev);

	if (ei->chained_irq && device_may_wakeup(dev))
		disable_irq_wake(ei->chained_irq);

	/* Update registers from the cache, in case
	   the CPLD was powered off during suspend */
	egpio_write_cache(ei);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(egpio_pm_ops, egpio_suspend, egpio_resume);

static const struct of_device_id egpio_of_match[] = {
	{ .compatible = "htc,egpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, egpio_of_match);

static struct platform_driver egpio_driver = {
	.driver = {
		.name = "htc-egpio",
		.suppress_bind_attrs = true,
		.pm = pm_sleep_ptr(&egpio_pm_ops),
		.of_match_table = egpio_of_match,
	},
	.probe = egpio_probe,
};
module_platform_driver(egpio_driver);

MODULE_DESCRIPTION("HTC EGPIO CPLD GPIO/IRQ expander driver");
MODULE_LICENSE("GPL");
