// SPDX-License-Identifier: GPL-2.0-only
/*
 * GPIO combination keys driver
 *
 * Keys that are identified by the combination of several active GPIO lines,
 * such as a 5-way navigation pad whose contacts sit between the directions.
 *
 * Copyright (c) 2026 Koen Kooi <koen@dominion.thruhere.net>
 */

#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/device/devres.h>
#include <linux/device-id/of.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_wakeup.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#define GCK_MAX_LINES	32
#define GCK_MAX_CODES	4

struct gpio_combo_key {
	u32 mask;
	unsigned int ncodes;
	u32 codes[GCK_MAX_CODES];
};

struct gpio_combo_keys {
	struct device *dev;
	struct input_dev *input;
	struct gpio_descs *lines;
	int *irqs;
	struct gpio_combo_key *keys;
	unsigned int nkeys;
	unsigned long settle;
	struct delayed_work work;
	struct mutex lock;	/* protects candidate and reported */
	u32 candidate;
	unsigned long changed_at;	/* jiffies of the last change of candidate */
	u32 wake_enabled;	/* lines whose enable_irq_wake() succeeded */
	int reported;		/* index into keys[], -1 for none */
};

static int gck_sample(struct gpio_combo_keys *gck, u32 *mask)
{
	DECLARE_BITMAP(vals, GCK_MAX_LINES) = { };
	int err;

	err = gpiod_get_array_value_cansleep(gck->lines->ndescs,
					     gck->lines->desc,
					     gck->lines->info, vals);
	if (err)
		return err;

	*mask = bitmap_read(vals, 0, GCK_MAX_LINES);
	return 0;
}

static bool gck_has_code(const struct gpio_combo_key *key, u32 code)
{
	unsigned int i;

	for (i = 0; key && i < key->ncodes; i++)
		if (key->codes[i] == code)
			return true;

	return false;
}

/* All lines not listed by a key must be inactive, so only equality matches. */
static int gck_match(const struct gpio_combo_keys *gck, u32 mask)
{
	unsigned int i;

	for (i = 0; i < gck->nkeys; i++)
		if (gck->keys[i].mask == mask)
			return i;

	return -1;
}

/* Called with gck->lock held. */
static void gck_report(struct gpio_combo_keys *gck, u32 mask)
{
	const struct gpio_combo_key *old, *new;
	int idx = gck_match(gck, mask);
	unsigned int i;

	if (idx == gck->reported)
		return;

	old = gck->reported >= 0 ? &gck->keys[gck->reported] : NULL;
	new = idx >= 0 ? &gck->keys[idx] : NULL;

	if (new)
		input_event(gck->input, EV_MSC, MSC_SCAN, mask);

	/* Keys shared by consecutive matches stay held. */
	for (i = 0; old && i < old->ncodes; i++)
		if (!gck_has_code(new, old->codes[i]))
			input_report_key(gck->input, old->codes[i], 0);

	for (i = 0; new && i < new->ncodes; i++)
		if (!gck_has_code(old, new->codes[i]))
			input_report_key(gck->input, new->codes[i], 1);

	input_sync(gck->input);
	gck->reported = idx;
}

static void gck_work(struct work_struct *work)
{
	struct gpio_combo_keys *gck =
		container_of(work, struct gpio_combo_keys, work.work);
	u32 now;
	int err;

	mutex_lock(&gck->lock);

	err = gck_sample(gck, &now);
	if (err) {
		/* A transient read error must not release a held key. */
		dev_warn_ratelimited(gck->dev, "failed to read lines: %d\n", err);
		mod_delayed_work(system_wq, &gck->work, gck->settle);
		goto out;
	}

	/*
	 * An edge can be missed while its line is masked by IRQF_ONESHOT, so
	 * a change nobody announced restarts the window instead of being
	 * reported.
	 */
	if (now != gck->candidate) {
		gck->candidate = now;
		gck->changed_at = jiffies;
		mod_delayed_work(system_wq, &gck->work, gck->settle);
		goto out;
	}

	/*
	 * The IRQ thread updates the candidate and re-arms the work in two
	 * steps; a work item that was already running in between must not
	 * report a combination that has not been stable for a whole window.
	 */
	if (time_before(jiffies, gck->changed_at + gck->settle)) {
		mod_delayed_work(system_wq, &gck->work,
				 gck->changed_at + gck->settle - jiffies);
		goto out;
	}

	gck_report(gck, now);
out:
	mutex_unlock(&gck->lock);
}

static irqreturn_t gck_irq_thread(int irq, void *data)
{
	struct gpio_combo_keys *gck = data;
	int err;

	if (device_may_wakeup(gck->dev))
		pm_wakeup_event(gck->dev, jiffies_to_msecs(gck->settle) + 50);

	mutex_lock(&gck->lock);
	err = gck_sample(gck, &gck->candidate);
	gck->changed_at = jiffies;
	mutex_unlock(&gck->lock);
	if (err)
		dev_warn_ratelimited(gck->dev, "failed to read lines: %d\n", err);

	/*
	 * Every edge of any line pushes the deadline out again: the contacts of
	 * one press close up to tens of milliseconds apart, and only the
	 * combination that survives a whole window is a real one.
	 */
	mod_delayed_work(system_wq, &gck->work, gck->settle);

	return IRQ_HANDLED;
}

static void gck_cancel_work(void *data)
{
	struct gpio_combo_keys *gck = data;

	cancel_delayed_work_sync(&gck->work);
}

static int gck_parse_keys(struct gpio_combo_keys *gck)
{
	struct device *dev = gck->dev;
	unsigned int nlines = gck->lines->ndescs;
	unsigned int i, k = 0;
	int count;

	count = device_get_child_node_count(dev);
	if (count < 1)
		return dev_err_probe(dev, -EINVAL, "no key nodes\n");

	gck->keys = devm_kcalloc(dev, count, sizeof(*gck->keys), GFP_KERNEL);
	if (!gck->keys)
		return -ENOMEM;

	device_for_each_child_node_scoped(dev, child) {
		struct gpio_combo_key *key = &gck->keys[k];
		u32 idx[GCK_MAX_LINES];
		int n, err;

		n = fwnode_property_count_u32(child, "active-lines");
		if (n < 1 || n > nlines)
			return dev_err_probe(dev, -EINVAL,
					     "%pfwP: bad active-lines\n", child);

		err = fwnode_property_read_u32_array(child, "active-lines", idx, n);
		if (err)
			return dev_err_probe(dev, err,
					     "%pfwP: cannot read active-lines\n", child);

		for (i = 0; i < n; i++) {
			if (idx[i] >= nlines)
				return dev_err_probe(dev, -EINVAL,
						     "%pfwP: line index out of range\n",
						     child);
			if (key->mask & BIT(idx[i]))
				return dev_err_probe(dev, -EINVAL,
						     "%pfwP: duplicate line\n", child);
			key->mask |= BIT(idx[i]);
		}

		if (fwnode_property_present(child, "linux,code") ==
		    fwnode_property_present(child, "linux,keycodes"))
			return dev_err_probe(dev, -EINVAL,
					     "%pfwP: need exactly one of linux,code and linux,keycodes\n",
					     child);

		if (fwnode_property_present(child, "linux,code")) {
			key->ncodes = 1;
			err = fwnode_property_read_u32(child, "linux,code",
						       &key->codes[0]);
		} else {
			n = fwnode_property_count_u32(child, "linux,keycodes");
			if (n < 1 || n > GCK_MAX_CODES)
				return dev_err_probe(dev, -EINVAL,
						     "%pfwP: bad linux,keycodes\n",
						     child);
			key->ncodes = n;
			err = fwnode_property_read_u32_array(child, "linux,keycodes",
							     key->codes, n);
		}
		if (err)
			return dev_err_probe(dev, err,
					     "%pfwP: cannot read key codes\n", child);

		for (i = 0; i < key->ncodes; i++)
			input_set_capability(gck->input, EV_KEY, key->codes[i]);

		for (i = 0; i < k; i++)
			if (gck->keys[i].mask == key->mask)
				return dev_err_probe(dev, -EINVAL,
						     "%pfwP: combination 0x%x is used twice\n",
						     child, key->mask);
		k++;
	}

	gck->nkeys = k;
	return 0;
}

static int gck_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_combo_keys *gck;
	struct input_dev *input;
	const char *label;
	unsigned int i;
	u32 ms = 50;
	int err;

	gck = devm_kzalloc(dev, sizeof(*gck), GFP_KERNEL);
	if (!gck)
		return -ENOMEM;

	gck->dev = dev;
	gck->reported = -1;
	INIT_DELAYED_WORK(&gck->work, gck_work);
	err = devm_mutex_init(dev, &gck->lock);
	if (err)
		return err;

	gck->lines = devm_gpiod_get_array(dev, "line", GPIOD_IN);
	if (IS_ERR(gck->lines))
		return dev_err_probe(dev, PTR_ERR(gck->lines),
				     "failed to get line-gpios\n");
	if (gck->lines->ndescs < 2 || gck->lines->ndescs > GCK_MAX_LINES)
		return dev_err_probe(dev, -EINVAL, "need 2..%d line-gpios\n",
				     GCK_MAX_LINES);

	device_property_read_u32(dev, "debounce-delay-ms", &ms);
	if (ms > 999)
		return dev_err_probe(dev, -EINVAL, "debounce-delay-ms too large\n");
	/*
	 * A delayed work of N jiffies can run after only N - 1 full ticks, so
	 * one more is needed for the window to be at least ms long.
	 */
	gck->settle = msecs_to_jiffies(ms) + 1;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;
	gck->input = input;

	if (device_property_read_string(dev, "label", &label))
		label = "gpio-combo-keys";
	input->name = label;
	input->phys = "gpio-combo-keys/input0";
	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;
	input_set_capability(input, EV_MSC, MSC_SCAN);
	if (device_property_read_bool(dev, "autorepeat"))
		__set_bit(EV_REP, input->evbit);
	input_set_drvdata(input, gck);

	err = gck_parse_keys(gck);
	if (err)
		return err;

	gck->irqs = devm_kcalloc(dev, gck->lines->ndescs, sizeof(*gck->irqs),
				 GFP_KERNEL);
	if (!gck->irqs)
		return -ENOMEM;

	err = input_register_device(input);
	if (err)
		return dev_err_probe(dev, err, "failed to register input device\n");

	/*
	 * Registered after the input device and before the interrupts, so that
	 * teardown frees the interrupts first, then stops the work, and only
	 * then unregisters the input device the work reports to.
	 */
	err = devm_add_action_or_reset(dev, gck_cancel_work, gck);
	if (err)
		return err;

	for (i = 0; i < gck->lines->ndescs; i++) {
		int irq = gpiod_to_irq(gck->lines->desc[i]);

		if (irq < 0)
			return dev_err_probe(dev, irq,
					     "line %u has no interrupt\n", i);
		gck->irqs[i] = irq;

		err = devm_request_threaded_irq(dev, irq, NULL, gck_irq_thread,
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						dev_name(dev), gck);
		if (err)
			return dev_err_probe(dev, err,
					     "failed to request line %u interrupt\n", i);
	}

	if (device_property_read_bool(dev, "wakeup-source")) {
		err = devm_device_init_wakeup(dev);
		if (err)
			return dev_err_probe(dev, err, "failed to init wakeup\n");
	}

	platform_set_drvdata(pdev, gck);

	/* A combination held at boot is reported after one window. */
	mutex_lock(&gck->lock);
	err = gck_sample(gck, &gck->candidate);
	gck->changed_at = jiffies;
	mutex_unlock(&gck->lock);
	if (err)
		dev_warn(dev, "failed to read lines: %d\n", err);
	mod_delayed_work(system_wq, &gck->work, gck->settle);

	return 0;
}

static int gck_suspend(struct device *dev)
{
	struct gpio_combo_keys *gck = dev_get_drvdata(dev);
	unsigned int i;

	if (device_may_wakeup(dev)) {
		gck->wake_enabled = 0;
		for (i = 0; i < gck->lines->ndescs; i++) {
			if (enable_irq_wake(gck->irqs[i]))
				dev_warn(dev, "cannot enable wake on line %u\n", i);
			else
				gck->wake_enabled |= BIT(i);
		}

		/*
		 * The work queue is not freezable and reads the lines over the
		 * parent's bus, so nothing may run after this point; a press
		 * during suspend wakes the system and resume resamples.
		 */
		cancel_delayed_work_sync(&gck->work);
		mutex_lock(&gck->lock);
		gck_report(gck, 0);
		mutex_unlock(&gck->lock);
		return 0;
	}

	for (i = 0; i < gck->lines->ndescs; i++)
		disable_irq(gck->irqs[i]);
	cancel_delayed_work_sync(&gck->work);

	/* No key may stay held across suspend. */
	mutex_lock(&gck->lock);
	gck_report(gck, 0);
	mutex_unlock(&gck->lock);

	return 0;
}

static int gck_resume(struct device *dev)
{
	struct gpio_combo_keys *gck = dev_get_drvdata(dev);
	unsigned int i;
	int err;

	for (i = 0; i < gck->lines->ndescs; i++) {
		if (device_may_wakeup(dev)) {
			if (gck->wake_enabled & BIT(i))
				disable_irq_wake(gck->irqs[i]);
		} else {
			enable_irq(gck->irqs[i]);
		}
	}

	/* Report what is held now and release what changed while asleep. */
	mutex_lock(&gck->lock);
	err = gck_sample(gck, &gck->candidate);
	gck->changed_at = jiffies;
	mutex_unlock(&gck->lock);
	if (err)
		dev_warn(dev, "failed to read lines: %d\n", err);
	mod_delayed_work(system_wq, &gck->work, gck->settle);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(gck_pm_ops, gck_suspend, gck_resume);

static const struct of_device_id gck_of_match[] = {
	{ .compatible = "gpio-combo-keys" },
	{ }
};
MODULE_DEVICE_TABLE(of, gck_of_match);

static struct platform_driver gck_driver = {
	.probe = gck_probe,
	.driver = {
		.name = "gpio-combo-keys",
		.pm = pm_sleep_ptr(&gck_pm_ops),
		.of_match_table = gck_of_match,
	},
};
module_platform_driver(gck_driver);

MODULE_AUTHOR("Koen Kooi <koen@dominion.thruhere.net>");
MODULE_DESCRIPTION("GPIO combination keys driver");
MODULE_LICENSE("GPL");
