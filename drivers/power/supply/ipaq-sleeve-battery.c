// SPDX-License-Identifier: GPL-2.0-only
/*
 * Battery in an HP iPAQ expansion pack ("sleeve")
 *
 * Some packs (the CompactFlash Plus and PC Card Plus jackets) carry their
 * own battery and a microcontroller that reports its state over the
 * sleeve SPI link. Whether a pack has one is stated by the Extended
 * Battery byte of its ID EEPROM record, so a supply is registered only for
 * a record that parsed in full and describes the pack on the connector
 * (ipaq_sleeve_usable_record()). A pack that could not be read, or whose
 * identity was forced, gets no supply at all.
 *
 * Polling follows the iPAQ's own WinCE battery driver: every 5 s, two
 * attempts per poll, and three failed polls in a row stop polling until
 * the pack is next inserted. The first poll comes 5 s after insertion,
 * giving the pack's microcontroller that long to settle.
 *
 * The reply's chemistry byte and every flag value other than "no battery"
 * (0x80) and "critically low" (4) have no known meaning, so technology
 * and status are reported as unknown rather than guessed.
 */

#include <linux/ipaq-sleeve.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define IPAQ_SLEEVE_BAT_SETTLE_MS	5000
#define IPAQ_SLEEVE_BAT_POLL_MS		5000
#define IPAQ_SLEEVE_BAT_TRIES		2
#define IPAQ_SLEEVE_BAT_MAX_FAILED	3

#define IPAQ_SLEEVE_BAT_FLAG_NONE	0x80
#define IPAQ_SLEEVE_BAT_FLAG_CRITICAL	4

struct ipaq_sleeve_bat {
	struct power_supply_desc desc;
	struct power_supply *psy;
	struct delayed_work work;

	/* Guards everything below against the property reader. */
	struct mutex lock;
	bool present;
	bool have_reading;
	bool critical;
	unsigned int failed;
	u8 percent;
};

/*
 * The connector holds one pack, so there is at most one supply. Touched
 * only from the notifier callback, which the bus core serialises, and
 * from module exit once the notifier is unregistered.
 */
static struct ipaq_sleeve_bat *ipaq_sleeve_bat;

static const enum power_supply_property ipaq_sleeve_bat_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
};

static int ipaq_sleeve_bat_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct ipaq_sleeve_bat *bat = power_supply_get_drvdata(psy);

	guard(mutex)(&bat->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bat->present;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (!bat->present || !bat->have_reading)
			return -ENODATA;
		val->intval = bat->percent;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		if (bat->present && bat->critical)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * One poll. Updates @bat from a successful reply, or counts a failed poll.
 * Returns whether to poll again. Sets @changed when presence or the
 * critical level changed, so userspace is told on a transition rather
 * than on every poll.
 */
static bool ipaq_sleeve_bat_update(struct ipaq_sleeve_bat *bat, bool *changed)
{
	unsigned char chem, percent, flag;
	bool present, critical;
	unsigned int i;
	int ret;

	for (i = 0; i < IPAQ_SLEEVE_BAT_TRIES; i++) {
		ret = ipaq_sleeve_read_battery(&chem, &percent, &flag);
		if (!ret || ret == -EOPNOTSUPP)
			break;
	}

	guard(mutex)(&bat->lock);

	if (ret) {
		/* No backend support will not appear by retrying. */
		if (ret != -EOPNOTSUPP &&
		    ++bat->failed < IPAQ_SLEEVE_BAT_MAX_FAILED)
			return true;

		if (ret == -EOPNOTSUPP)
			dev_info(&bat->psy->dev,
				 "this host cannot reach the pack's battery controller\n");
		else
			dev_warn(&bat->psy->dev,
				 "no reply in %u polls (%d), polling stopped until the pack is reinserted\n",
				 bat->failed, ret);

		*changed = bat->present;
		bat->present = false;
		return false;
	}

	present = flag != IPAQ_SLEEVE_BAT_FLAG_NONE;
	critical = flag == IPAQ_SLEEVE_BAT_FLAG_CRITICAL;

	*changed = present != bat->present || critical != bat->critical;

	bat->failed = 0;
	bat->present = present;
	bat->critical = critical;
	bat->percent = percent;
	bat->have_reading = true;

	return true;
}

static void ipaq_sleeve_bat_poll(struct work_struct *work)
{
	struct ipaq_sleeve_bat *bat =
		container_of(to_delayed_work(work), struct ipaq_sleeve_bat, work);
	bool changed = false;

	if (ipaq_sleeve_bat_update(bat, &changed))
		queue_delayed_work(system_freezable_wq, &bat->work,
				   msecs_to_jiffies(IPAQ_SLEEVE_BAT_POLL_MS));

	if (changed)
		power_supply_changed(bat->psy);
}

static void ipaq_sleeve_bat_detach(void)
{
	struct ipaq_sleeve_bat *bat = ipaq_sleeve_bat;

	if (!bat)
		return;

	ipaq_sleeve_bat = NULL;

	cancel_delayed_work_sync(&bat->work);
	power_supply_unregister(bat->psy);
	kfree(bat);
}

static void ipaq_sleeve_bat_attach(struct ipaq_sleeve_device *sdev)
{
	struct power_supply_config cfg = {};
	struct ipaq_sleeve_bat *bat;

	bat = kzalloc_obj(*bat, GFP_KERNEL);
	if (!bat)
		return;

	mutex_init(&bat->lock);
	INIT_DELAYED_WORK(&bat->work, ipaq_sleeve_bat_poll);

	/* The record declares a battery; the first reply confirms or not. */
	bat->present = true;

	bat->desc.name = "ipaq-sleeve-battery";
	bat->desc.type = POWER_SUPPLY_TYPE_BATTERY;
	bat->desc.properties = ipaq_sleeve_bat_props;
	bat->desc.num_properties = ARRAY_SIZE(ipaq_sleeve_bat_props);
	bat->desc.get_property = ipaq_sleeve_bat_get_property;

	cfg.drv_data = bat;

	bat->psy = power_supply_register(&sdev->dev, &bat->desc, &cfg);
	if (IS_ERR(bat->psy)) {
		dev_err(&sdev->dev, "cannot register battery: %pe\n", bat->psy);
		kfree(bat);
		return;
	}

	ipaq_sleeve_bat = bat;

	queue_delayed_work(system_freezable_wq, &bat->work,
			   msecs_to_jiffies(IPAQ_SLEEVE_BAT_SETTLE_MS));
}

static int ipaq_sleeve_bat_notify(struct notifier_block *nb,
				  unsigned long event, void *data)
{
	struct ipaq_sleeve_device *sdev = data;
	const struct ipaq_sleeve_record *rec;

	switch (event) {
	case IPAQ_SLEEVE_EVENT_ADD:
		ipaq_sleeve_bat_detach();

		rec = ipaq_sleeve_usable_record(sdev);
		if (!rec || !ipaq_sleeve_record_ext_battery(rec))
			return NOTIFY_DONE;

		ipaq_sleeve_bat_attach(sdev);
		return NOTIFY_OK;
	case IPAQ_SLEEVE_EVENT_REMOVE:
		ipaq_sleeve_bat_detach();
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

static struct notifier_block ipaq_sleeve_bat_nb = {
	.notifier_call = ipaq_sleeve_bat_notify,
};

static int __init ipaq_sleeve_bat_init(void)
{
	return ipaq_sleeve_register_notifier(&ipaq_sleeve_bat_nb);
}
module_init(ipaq_sleeve_bat_init);

static void __exit ipaq_sleeve_bat_exit(void)
{
	ipaq_sleeve_unregister_notifier(&ipaq_sleeve_bat_nb);
	ipaq_sleeve_bat_detach();
}
module_exit(ipaq_sleeve_bat_exit);

MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION("HP iPAQ expansion pack (sleeve) battery");
MODULE_LICENSE("GPL");
