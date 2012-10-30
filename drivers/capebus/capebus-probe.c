/*
 * Capebus bus infrastructure
 *
 * Copyright (C) 2012 Pantelis Antoniou <panto@antoniou-consulting.com>
 * Copyright (C) 2012 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_i2c.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include <linux/capebus.h>

LIST_HEAD(cape_buses);
EXPORT_SYMBOL(cape_buses);

DEFINE_MUTEX(cape_buses_mutex);
EXPORT_SYMBOL(cape_buses_mutex);

/*
 * Cape Bus Class
 */
static void release_capebus_dev(struct device *dev)
{
	struct cape_dev *cape_dev = to_cape_dev(dev);

	kfree(cape_dev);
}

static struct class capebus_class = {
	.name		= "capebus",
	.dev_release	= &release_capebus_dev,
};

static int __init capebus_class_init(void)
{
	return class_register(&capebus_class);
}
postcore_initcall(capebus_class_init);

static struct cape_bus *cape_bus_find(const char *name, int busno)
{
	struct cape_bus *bus;
	int found;

	if (busno < 0)
		return NULL;

	found = 0;
	cape_bus_for_each(bus) {
		if (strcmp(name, bus->name) == 0 && bus->busno == busno) {
			found = 1;
			break;
		}
	}
	return found ? bus : NULL;
}

static int cape_bus_pick_busno(const char *name, int busno)
{
	struct cape_bus *bus;

	BUG_ON(name == NULL);

	/* fixed id */
	if (busno >= 0)
		return busno;

	/* dynamic id */
	busno = -1;
	cape_bus_for_each(bus) {
		/* name must match */
		if (strcmp(name, bus->name) != 0)
			continue;
		busno = max(busno, bus->busno);
	}
	return busno + 1;
}

int cape_bus_register(struct cape_bus *bus, const char *name, int busno,
		struct device *parent, struct cape_bus_ops *ops)
{
	struct cape_bus *b2;
	int r;

	if (name == NULL)
		return -EINVAL;

	INIT_LIST_HEAD(&bus->node);
	INIT_LIST_HEAD(&bus->devices);
	INIT_LIST_HEAD(&bus->slots);

	/* do everything under lock */
	mutex_lock(&cape_buses_mutex);

	b2 = cape_bus_find(name, busno);
	if (b2 != NULL) {
		if (parent != NULL)
			dev_err(parent, "capebus %s:%d in use\n", name, busno);
		else
			pr_err("capebus %s:%d in use\n", name, busno);
		r = -EBUSY;
		goto err_unlock;
	}
	bus->name = name;
	bus->busno = cape_bus_pick_busno(name, busno);
	bus->ops = ops;

	bus->dev.class = &capebus_class;
	bus->dev.parent = parent;
	dev_set_name(&bus->dev, "%s:%d", bus->name, bus->busno);
	r = device_register(&bus->dev);
	if (r != 0) {
		if (parent != NULL)
			dev_err(parent, "capebus #%d failed to register dev\n",
					bus->busno);
		else
			pr_err("capebus #%d failed to register dev\n",
					bus->busno);
		goto err_unlock;
	}

	list_add_tail(&bus->node, &cape_buses);
	mutex_unlock(&cape_buses_mutex);

	dev_info(&bus->dev, "Registered\n");

	return 0;
err_unlock:
	mutex_unlock(&cape_buses_mutex);
	return r;
}

int cape_bus_deregister(struct cape_bus *bus)
{
	return -EINVAL;	/* not yet supported */
}

/* must have cape_buses_mutex */
struct cape_slot *cape_slot_find(struct cape_bus *bus, int slotno)
{
	struct cape_slot *slot;
	int found;

	found = 0;
	cape_slot_for_each(bus, slot) {
		if (slot->slotno == slotno) {
			found = 1;
			break;
		}
	}
	return found ? slot : NULL;
}

/**
 * cape_bus_release_dev - free a cape device structure when all users
 *                        of it are finished.
 * @dev: device that's been disconnected
 *
 * Will be called only by the device core when all users of this cape device are
 * done.
 */
static void cape_bus_release_dev(struct device *dev)
{
	struct cape_dev *cdev;

	cdev = to_cape_dev(dev);
	/* cape_release_capabilities(cdev); TODO */
	/* cape_release_of_node(cdev); TODO */
	kfree(cdev);
}

/* mutex lock must be held */
static struct cape_dev *cape_bus_scan_slot(struct cape_slot *slot)
{
	struct cape_bus *bus = slot->bus;
	struct cape_dev *dev;
	const struct cape_device_id *id;

	/* get the ID (if a device exists) */
	id = bus->ops->get_dev_id(slot);
	if (id == NULL)
		return ERR_PTR(-ENODEV);

	/* slot must not have a device yet */
	dev = slot->dev;
	if (dev == NULL) {
		dev = kzalloc(sizeof(*dev), GFP_KERNEL);
		if (dev == NULL) {
			dev_info(&bus->dev, "Failed to allocate cape device "
					"for slot #%d\n", slot->slotno);
			return ERR_PTR(-ENOMEM);
		}

		INIT_LIST_HEAD(&dev->bus_list);
		dev->bus = bus;
		dev->slot = slot;
	}

	dev->id = id;
	dev->text_id = bus->ops->get_text_dev_id(slot);

	/* capebus_set_of_node(dev); TODO */

	return dev;
}

int cape_bus_scan_one_slot(struct cape_bus *bus, struct cape_slot *slot)
{
	struct cape_dev *dev;
	int r;

	mutex_lock(&cape_buses_mutex);

	dev = slot->dev;
	if (dev == NULL) {

		dev = cape_bus_scan_slot(slot);
		if (IS_ERR(dev)) {
			r = PTR_ERR(dev);
			goto err_out;
		}

		dev_info(&bus->dev, "Slot #%d id='%s'\n", slot->slotno,
				dev->text_id ? dev->text_id : "");

		slot->dev = dev;

		dev->dev.release = cape_bus_release_dev;
		dev->dev.parent = &dev->bus->dev;
		dev->dev.bus = &capebus_bus_type;
		dev_set_name(&dev->dev, "%s-%d:%d",
			     dev->bus->name, dev->bus->busno,
			     dev->slot->slotno);

		list_add_tail(&dev->bus_list, &bus->devices);

	} else {
		dev_info(&bus->dev, "Slot #%d id='%s' - rescan\n", slot->slotno,
				dev->text_id ? dev->text_id : "");

		if (dev->added) {
			r = -EEXIST;
			goto err_out;
		}
	}

	r = device_register(&dev->dev);
	if (r != 0) {
		dev_info(&bus->dev, "Slot #%d id='%s' - "
				"Failed to register\n",
				slot->slotno,
				dev->text_id ? dev->text_id : "");
		r = 0;
	} else {
		if (dev->bus->ops->dev_registered)
			dev->bus->ops->dev_registered(dev);
	}

err_out:
	mutex_unlock(&cape_buses_mutex);

	return r;
}

int cape_bus_register_slot(struct cape_bus *bus, struct cape_slot *slot,
		int slotno)
{
	struct cape_slot *s2;
	int r;

	r = 0;

	/* invalid (slot must always be numbered - no hotplug) */
	if (slotno < 0) {
		dev_err(&bus->dev, "Slot registration #%d failed\n", slotno);
		return -EINVAL;
	}

	mutex_lock(&cape_buses_mutex);
	s2 = cape_slot_find(bus, slotno);
	if (s2 != NULL) {
		dev_err(&bus->dev, "Slot #%d already exists\n", slotno);
		mutex_unlock(&cape_buses_mutex);
		return -EINVAL;
	}

	INIT_LIST_HEAD(&slot->node);
	slot->bus = bus;
	list_add(&slot->node, &bus->slots);
	slot->slotno = slotno;
	slot->dev = NULL;
	mutex_unlock(&cape_buses_mutex);

	dev_info(&bus->dev, "Slot #%d registered\n", slot->slotno);

	return cape_bus_scan_one_slot(bus, slot);
}
