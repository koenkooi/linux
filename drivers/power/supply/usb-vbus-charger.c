// SPDX-License-Identifier: GPL-2.0-only
/*
 * USB VBUS charge input sensed through a USB PHY
 *
 * Reports a USB power supply that is online while the PHY reports VBUS,
 * and optionally drives a GPIO that switches VBUS into the charger input
 * while VBUS is present and a priority charger (mains) is offline, and
 * GPIOs that select the charge current taken from VBUS according to the
 * configuration the host accepted.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/usb/phy.h>
#include <linux/workqueue.h>

struct usb_vbus_charger_current {
	u32 limit_ua;
	u32 gpiodata;
};

struct usb_vbus_charger {
	struct device *dev;
	struct usb_phy *phy;
	struct gpio_desc *enable;
	struct power_supply *priority;
	struct power_supply *psy;
	struct power_supply_desc desc;
	enum power_supply_property props[2];

	struct gpio_descs *current_gpios;
	struct usb_vbus_charger_current *current_map;
	unsigned int current_map_size;
	unsigned int current_default;

	struct notifier_block usb_nb;
	struct notifier_block psy_nb;
	struct work_struct work;
	unsigned long vbus_lost;
	unsigned int granted_ma;

	/* serialises the probe-time update, the work and sysfs writes */
	struct mutex lock;
	bool online;
	int enabled;
	unsigned int current_index;
	unsigned int auto_index;
};

/* Called with chg->lock held, or in probe and removal, where nothing races. */
static void usb_vbus_charger_set_current(struct usb_vbus_charger *chg,
					 unsigned int index)
{
	struct gpio_desc **desc = chg->current_gpios->desc;
	unsigned int n = chg->current_gpios->ndescs;
	u32 data = chg->current_map[index].gpiodata;
	unsigned int i;

	/* Bit 0 of the mapping is the last GPIO, as in gpio-charger. */
	for (i = 0; i < n; i++)
		gpiod_set_value_cansleep(desc[n - 1 - i], (data >> i) & 1);

	chg->current_index = index;
	dev_dbg(chg->dev, "charge current %u uA\n",
		chg->current_map[index].limit_ua);
}

/*
 * The highest entry not above the request, or the lowest entry when the
 * request is below all of them: a write never raises the current past
 * what was asked for.
 */
static unsigned int usb_vbus_charger_current_index(struct usb_vbus_charger *chg,
						   int ua)
{
	unsigned int i;

	for (i = 0; i < chg->current_map_size; i++)
		if (ua >= 0 && chg->current_map[i].limit_ua <= ua)
			return i;

	return chg->current_map_size - 1;
}

static bool usb_vbus_charger_priority_online(struct usb_vbus_charger *chg)
{
	union power_supply_propval val;

	if (!chg->priority)
		return false;

	if (power_supply_get_property(chg->priority,
				      POWER_SUPPLY_PROP_ONLINE, &val))
		return false;

	return val.intval;
}

/*
 * Only a configuration the host accepted grants more than the default,
 * and only while VBUS feeds the charger: the current GPIOs may act on the
 * priority input as well, so neither a grant nor a write may outlive the
 * switch to it, and the default is held at every update until VBUS feeds
 * the charger again.
 *
 * While VBUS feeds it, the choice is applied when it changes or VBUS was
 * lost, not on every update, so a CONSTANT_CHARGE_CURRENT_MAX write holds
 * until the next change of host, grant or input. The lost flag catches a
 * replug that is over before the work runs, which last_event alone would
 * not show.
 */
static void usb_vbus_charger_select_current(struct usb_vbus_charger *chg,
					    unsigned long event, bool lost,
					    bool vbus_feeds)
{
	unsigned int granted = READ_ONCE(chg->granted_ma);
	unsigned int index = chg->current_default;

	if (event == USB_EVENT_ENUMERATED && granted && vbus_feeds)
		index = usb_vbus_charger_current_index(chg, granted * 1000);

	if (!lost && vbus_feeds && index == chg->auto_index)
		return;

	chg->auto_index = index;
	if (index != chg->current_index)
		usb_vbus_charger_set_current(chg, index);
}

static void usb_vbus_charger_update(struct usb_vbus_charger *chg)
{
	unsigned long event = READ_ONCE(chg->phy->last_event);
	bool online, changed, lost, vbus_feeds;
	int enable;

	mutex_lock(&chg->lock);

	online = event == USB_EVENT_VBUS || event == USB_EVENT_ENUMERATED;
	changed = online != chg->online;
	chg->online = online;
	lost = test_and_clear_bit(0, &chg->vbus_lost) || !online;

	enable = online && !usb_vbus_charger_priority_online(chg);
	vbus_feeds = enable;

	/* Lower the current before the input switches, raise it after. */
	if (chg->current_gpios && !vbus_feeds)
		usb_vbus_charger_select_current(chg, event, lost, vbus_feeds);

	if (chg->enable && enable != chg->enabled) {
		gpiod_set_value_cansleep(chg->enable, enable);
		chg->enabled = enable;
		dev_dbg(chg->dev, "charge input %s\n",
			enable ? "connected" : "disconnected");
	}

	if (chg->current_gpios && vbus_feeds)
		usb_vbus_charger_select_current(chg, event, lost, vbus_feeds);

	mutex_unlock(&chg->lock);

	if (changed)
		power_supply_changed(chg->psy);
}

static void usb_vbus_charger_work(struct work_struct *work)
{
	struct usb_vbus_charger *chg =
		container_of(work, struct usb_vbus_charger, work);

	usb_vbus_charger_update(chg);
}

/*
 * Atomic chain: last_event is read again from the work. Only
 * USB_EVENT_ENUMERATED carries a pointer to the granted current in mA.
 * The PHY core's charger notification passes the current as the event
 * and the PHY as data, so a 4 mA one would otherwise read as a grant.
 */
static int usb_vbus_charger_usb_event(struct notifier_block *nb,
				      unsigned long event, void *data)
{
	struct usb_vbus_charger *chg =
		container_of(nb, struct usb_vbus_charger, usb_nb);

	if (event == USB_EVENT_ENUMERATED && data && data != chg->phy)
		WRITE_ONCE(chg->granted_ma, *(unsigned int *)data);
	else
		WRITE_ONCE(chg->granted_ma, 0);
	if (event == USB_EVENT_NONE)
		set_bit(0, &chg->vbus_lost);
	schedule_work(&chg->work);

	return NOTIFY_OK;
}

static int usb_vbus_charger_psy_event(struct notifier_block *nb,
				      unsigned long event, void *data)
{
	struct usb_vbus_charger *chg =
		container_of(nb, struct usb_vbus_charger, psy_nb);

	if (event == PSY_EVENT_PROP_CHANGED && data == chg->priority)
		schedule_work(&chg->work);

	return NOTIFY_OK;
}

static int usb_vbus_charger_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct usb_vbus_charger *chg = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = READ_ONCE(chg->online);
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		mutex_lock(&chg->lock);
		val->intval = chg->current_map[chg->current_index].limit_ua;
		mutex_unlock(&chg->lock);
		return 0;
	default:
		return -EINVAL;
	}
}

static int usb_vbus_charger_set_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 const union power_supply_propval *val)
{
	struct usb_vbus_charger *chg = power_supply_get_drvdata(psy);
	unsigned int index;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		index = usb_vbus_charger_current_index(chg, val->intval);
		mutex_lock(&chg->lock);
		/*
		 * Only the host that is present can grant more than the
		 * default, and the default is restored when it goes away.
		 */
		if (chg->online)
			usb_vbus_charger_set_current(chg, index);
		else
			ret = -EAGAIN;
		mutex_unlock(&chg->lock);
		return ret;
	default:
		return -EINVAL;
	}
}

static int usb_vbus_charger_property_is_writeable(struct power_supply *psy,
						  enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX;
}

static int usb_vbus_charger_init_current(struct usb_vbus_charger *chg)
{
	struct device *dev = chg->dev;
	u32 def;
	int len, i;

	/* Low until the mapping says otherwise. */
	chg->current_gpios = devm_gpiod_get_array_optional(dev,
							   "charge-current-limit",
							   GPIOD_OUT_LOW);
	if (IS_ERR(chg->current_gpios))
		return dev_err_probe(dev, PTR_ERR(chg->current_gpios),
				     "cannot get charge current GPIOs\n");
	if (!chg->current_gpios)
		return 0;

	len = device_property_count_u32(dev, "charge-current-limit-mapping");
	if (len <= 0 || len % 2)
		return dev_err_probe(dev, -EINVAL,
				     "invalid charge-current-limit-mapping\n");

	chg->current_map_size = len / 2;
	chg->current_map = devm_kcalloc(dev, chg->current_map_size,
					sizeof(*chg->current_map), GFP_KERNEL);
	if (!chg->current_map)
		return -ENOMEM;

	len = device_property_read_u32_array(dev, "charge-current-limit-mapping",
					     (u32 *)chg->current_map, len);
	if (len)
		return dev_err_probe(dev, len,
				     "cannot read charge-current-limit-mapping\n");

	for (i = 0; i < chg->current_map_size; i++) {
		if (chg->current_gpios->ndescs < 32 &&
		    chg->current_map[i].gpiodata >> chg->current_gpios->ndescs)
			return dev_err_probe(dev, -EINVAL,
					     "charge current %u uA needs more GPIOs\n",
					     chg->current_map[i].limit_ua);
		if (i && chg->current_map[i].limit_ua >=
			 chg->current_map[i - 1].limit_ua)
			return dev_err_probe(dev, -EINVAL,
					     "charge-current-limit-mapping not in descending order\n");
	}

	chg->current_default = chg->current_map_size - 1;
	if (!device_property_read_u32(dev, "charge-current-limit-default-microamp",
				      &def)) {
		for (i = 0; i < chg->current_map_size; i++)
			if (chg->current_map[i].limit_ua == def)
				break;
		if (i == chg->current_map_size)
			return dev_err_probe(dev, -EINVAL,
					     "default charge current %u uA not in the mapping\n",
					     def);
		chg->current_default = i;
	}

	chg->auto_index = chg->current_default;
	usb_vbus_charger_set_current(chg, chg->current_default);

	return 0;
}

static void usb_vbus_charger_reset_current(void *data)
{
	struct usb_vbus_charger *chg = data;

	usb_vbus_charger_set_current(chg, chg->current_default);
}

static void usb_vbus_charger_unregister_usb(void *data)
{
	struct usb_vbus_charger *chg = data;

	usb_unregister_notifier(chg->phy, &chg->usb_nb);
	cancel_work_sync(&chg->work);
}

static void usb_vbus_charger_unregister_psy(void *data)
{
	struct usb_vbus_charger *chg = data;

	power_supply_unreg_notifier(&chg->psy_nb);
	cancel_work_sync(&chg->work);
}

/* Runs after both notifiers are gone and the work is cancelled. */
static void usb_vbus_charger_disconnect(void *data)
{
	struct usb_vbus_charger *chg = data;

	gpiod_set_value_cansleep(chg->enable, 0);
}

static int usb_vbus_charger_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config psy_cfg = {};
	struct usb_vbus_charger *chg;
	struct power_supply *priority;
	int ret;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = dev;
	ret = devm_mutex_init(dev, &chg->lock);
	if (ret)
		return ret;
	INIT_WORK(&chg->work, usb_vbus_charger_work);

	chg->enable = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(chg->enable))
		return dev_err_probe(dev, PTR_ERR(chg->enable),
				     "cannot get enable GPIO\n");
	if (chg->enable) {
		ret = devm_add_action_or_reset(dev,
					       usb_vbus_charger_disconnect, chg);
		if (ret)
			return ret;
	}

	ret = usb_vbus_charger_init_current(chg);
	if (ret)
		return ret;
	if (chg->current_gpios) {
		ret = devm_add_action_or_reset(dev,
					       usb_vbus_charger_reset_current,
					       chg);
		if (ret)
			return ret;
	}

	if (device_property_present(dev, "usb-phy"))
		chg->phy = devm_usb_get_phy_by_phandle(dev, "usb-phy", 0);
	else
		chg->phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB2);
	/*
	 * A PHY without a DT node, such as a board-file gpio-vbus, registers
	 * whenever its parent binds and is reported as -ENODEV until then.
	 */
	if (PTR_ERR_OR_ZERO(chg->phy) == -ENODEV)
		return dev_err_probe(dev, -EPROBE_DEFER, "USB PHY not ready\n");
	if (IS_ERR(chg->phy))
		return dev_err_probe(dev, PTR_ERR(chg->phy),
				     "cannot get USB PHY\n");

	priority = devm_power_supply_get_by_reference(dev, "priority-charger");
	if (!priority)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "priority charger not ready\n");
	if (IS_ERR(priority)) {
		ret = PTR_ERR(priority);
		if (ret != -ENOENT)
			return dev_err_probe(dev, ret,
					     "cannot get priority charger\n");
		priority = NULL;
	}
	chg->priority = priority;

	chg->desc.name = dev->of_node ? dev->of_node->name : dev_name(dev);
	chg->desc.type = POWER_SUPPLY_TYPE_USB;
	chg->props[chg->desc.num_properties++] = POWER_SUPPLY_PROP_ONLINE;
	if (chg->current_gpios) {
		chg->props[chg->desc.num_properties++] =
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX;
		chg->desc.set_property = usb_vbus_charger_set_property;
		chg->desc.property_is_writeable =
			usb_vbus_charger_property_is_writeable;
	}
	chg->desc.properties = chg->props;
	chg->desc.get_property = usb_vbus_charger_get_property;

	psy_cfg.fwnode = dev_fwnode(dev);
	psy_cfg.drv_data = chg;

	chg->psy = devm_power_supply_register(dev, &chg->desc, &psy_cfg);
	if (IS_ERR(chg->psy))
		return dev_err_probe(dev, PTR_ERR(chg->psy),
				     "cannot register power supply\n");

	chg->usb_nb.notifier_call = usb_vbus_charger_usb_event;
	ret = usb_register_notifier(chg->phy, &chg->usb_nb);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register USB notifier\n");
	ret = devm_add_action_or_reset(dev, usb_vbus_charger_unregister_usb,
				       chg);
	if (ret)
		return ret;

	if (chg->priority) {
		chg->psy_nb.notifier_call = usb_vbus_charger_psy_event;
		ret = power_supply_reg_notifier(&chg->psy_nb);
		if (ret)
			return dev_err_probe(dev, ret,
					     "cannot register supply notifier\n");
		ret = devm_add_action_or_reset(dev,
					       usb_vbus_charger_unregister_psy,
					       chg);
		if (ret)
			return ret;
	}

	usb_vbus_charger_update(chg);

	return 0;
}

static const struct of_device_id usb_vbus_charger_of_match[] = {
	{ .compatible = "usb-vbus-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, usb_vbus_charger_of_match);

static struct platform_driver usb_vbus_charger_driver = {
	.probe = usb_vbus_charger_probe,
	.driver = {
		.name = "usb-vbus-charger",
		.of_match_table = usb_vbus_charger_of_match,
	},
};
module_platform_driver(usb_vbus_charger_driver);

MODULE_DESCRIPTION("USB VBUS charge input sensed through a USB PHY");
MODULE_AUTHOR("Koen Kooi <koen@dominion.thruhere.net>");
MODULE_LICENSE("GPL");
