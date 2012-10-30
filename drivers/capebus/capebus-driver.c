/*
 * Capebus driver infrastructure
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
#include <linux/init.h>
#include <linux/device.h>
#include <linux/mempolicy.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/pm_runtime.h>
#include <linux/suspend.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <linux/capebus.h>

/**
 * capebus_match_device - Tell if a cape device structure has a
 *                        matching cape device id structure
 * @drv: the cape driver to match against
 * @dev: the cape device structure to match against
 *
 * Used by a driver to check whether a cape device present in the
 * system is in its list of supported devices.  Returns the matching
 * cape_device_id structure or %NULL if there is no match.
 */
static const struct cape_device_id *capebus_match_device(
		struct cape_driver *drv, struct cape_dev *dev)
{
	struct cape_bus *bus = dev->bus;
	struct cape_slot *slot = dev->slot;

	BUG_ON(bus == NULL);
	BUG_ON(slot == NULL);
	BUG_ON(bus->ops == NULL);
	BUG_ON(bus->ops->get_dev_id == NULL);

	return bus->ops->get_dev_id(slot);
}

/**
 * capebus_device_probe - check if a driver wants to claim a
 *                          specific cape device
 * @dev: cape device being probed
 *
 * returns 0 on success, else error.
 * side-effect: cape_dev->driver is set to drv when drv claims cape_dev.
 */
static int capebus_device_probe(struct device *dev)
{
	const struct cape_device_id *id;
	int error = 0;
	struct cape_driver *drv;
	struct cape_dev *cape_dev;
	struct device *parent;

	drv = to_cape_driver(dev->driver);
	cape_dev = to_cape_dev(dev);
	cape_dev = capebus_dev_get(cape_dev);

	/* sanity checks */
	if (cape_dev == NULL ||
		cape_dev->bus == NULL || cape_dev->bus->ops == NULL ||
		cape_dev->driver != NULL || drv->probe == NULL) {
		error = -EINVAL;
		goto err_no_sanity;
	}

	id = capebus_match_device(drv, cape_dev);
	if (!id) {
		error = -ENODEV;
		goto err_no_match;
	}

	/* The parent device must be in active state when probing */
	parent = cape_dev->dev.parent;
	if (parent)
		pm_runtime_get_sync(parent);

	/* Unbound cape devices are always set to disabled and suspended.
	 * During probe, the device is set to enabled and active and the
	 * usage count is incremented.  If the driver supports runtime PM,
	 * it should call pm_runtime_put_noidle() in its probe routine and
	 * pm_runtime_get_noresume() in its remove routine.
	 */
	pm_runtime_get_noresume(&cape_dev->dev);
	pm_runtime_set_active(&cape_dev->dev);
	pm_runtime_enable(&cape_dev->dev);

	/* call the driver's probe method */
	error = drv->probe(cape_dev, id);

	/* release the parent no matter what */
	if (parent)
		pm_runtime_put(parent);

	if (error != 0)
		goto err_probe_fail;

	/* call the probed bus method */
	if (cape_dev->bus->ops->dev_probed != NULL) {
		error = cape_dev->bus->ops->dev_probed(cape_dev);
		if (error != 0)
			goto err_dev_probed_fail;
	}

	/* all is fine... */
	cape_dev->driver = drv;
	cape_dev->added = 1;

	return 0;

err_dev_probed_fail:
	if (drv->remove) {
		pm_runtime_get_sync(&cape_dev->dev);
		drv->remove(cape_dev);
		pm_runtime_put_noidle(&cape_dev->dev);
	}
err_probe_fail:
	pm_runtime_disable(&cape_dev->dev);
	pm_runtime_set_suspended(&cape_dev->dev);
	pm_runtime_put_noidle(&cape_dev->dev);
err_no_match:
	/* nothing */
err_no_sanity:
	capebus_dev_put(cape_dev);
	return error;
}

static int capebus_device_remove(struct device *dev)
{
	struct cape_dev *cape_dev = to_cape_dev(dev);
	struct cape_driver *drv = cape_dev->driver;

	if (drv) {
		/* call the removed bus method (if added prev.) */
		if (cape_dev->added) {
			BUG_ON(cape_dev->bus == NULL);
			BUG_ON(cape_dev->bus->ops == NULL);
			if (cape_dev->bus->ops->dev_removed)
				cape_dev->bus->ops->dev_removed(cape_dev);
			cape_dev->added = 0;
		}
		if (drv->remove) {
			pm_runtime_get_sync(dev);
			drv->remove(cape_dev);
			pm_runtime_put_noidle(dev);
		}
		cape_dev->driver = NULL;
	}

	/* Undo the runtime PM settings in local_capebus_probe() */
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_put_noidle(dev);

	capebus_dev_put(cape_dev);
	return 0;
}

static void capebus_device_shutdown(struct device *dev)
{
	struct cape_dev *cape_dev = to_cape_dev(dev);
	struct cape_driver *drv = cape_dev->driver;

	if (drv && drv->shutdown)
		drv->shutdown(cape_dev);

	capebus_disable_device(cape_dev);

	if (!device_may_wakeup(dev))
		capebus_enable_wake(cape_dev, false);
}

static int capebus_bus_match(struct device *dev, struct device_driver *drv);
static int capebus_device_probe(struct device *dev);
static int capebus_device_remove(struct device *dev);
static void capebus_device_shutdown(struct device *dev);

struct bus_type capebus_bus_type = {
	.name		= "capebus",
	.match		= capebus_bus_match,
	.probe		= capebus_device_probe,
	.remove		= capebus_device_remove,
	.shutdown	= capebus_device_shutdown,
	.dev_attrs	= capebus_dev_attrs,
	.bus_attrs	= capebus_bus_attrs,
	.pm		= NULL,	/* No PM for now */
};
EXPORT_SYMBOL(capebus_bus_type);

/**
 * __capebus_register_driver - register a new capebus driver
 * @drv: the driver structure to register
 * @owner: owner module of drv
 * @mod_name: module name string
 *
 * Adds the driver structure to the list of registered drivers.
 * Returns a negative value on error, otherwise 0.
 * If no error occurred, the driver remains registered even if
 * no device was claimed during registration.
 */
int __capebus_register_driver(struct cape_driver *drv, struct module *owner,
			  const char *mod_name)
{
	/* initialize common driver fields */
	drv->driver.bus = &capebus_bus_type;
	drv->driver.owner = owner;
	drv->driver.mod_name = mod_name;

	/* register with core */
	return driver_register(&drv->driver);
}
EXPORT_SYMBOL(__capebus_register_driver);

/**
 * capebus_unregister_driver - unregister a capebus driver
 * @drv: the driver structure to unregister
 *
 * Deletes the driver structure from the list of registered cape drivers,
 * gives it a chance to clean up by calling its remove() function for
 * each device it was responsible for, and marks those devices as
 * driverless.
 */

void
capebus_unregister_driver(struct cape_driver *drv)
{
	/* TODO: not really working properly */
	driver_unregister(&drv->driver);
}
EXPORT_SYMBOL(capebus_unregister_driver);

/**
 * capebus_bus_match - Tell if a cape device structure has a matching
 *                     cape device id structure
 * @dev: the cape device structure to match against
 * @drv: the device driver to search for matching cape device id structures
 *
 * Used by a driver to check whether a cape device present in the
 * system is in its list of supported devices. Returns the matching
 * cape_device_id structure or %NULL if there is no match.
 */
static int capebus_bus_match(struct device *dev, struct device_driver *drv)
{
	struct cape_dev *cape_dev = to_cape_dev(dev);
	struct cape_driver *cape_drv = to_cape_driver(drv);
	const struct cape_device_id *found_id;

	found_id = capebus_match_device(cape_drv, cape_dev);
	if (found_id)
		return 1;

	return 0;
}

/**
 * capebus_dev_get - increments the reference count of the capebus
 *                   device structure
 * @dev: the device being referenced
 *
 * Each live reference to a device should be refcounted.
 *
 * Drivers for cape devices should normally record such references in
 * their probe() methods, when they bind to a device, and release
 * them by calling capebus_dev_put(), in their disconnect() methods.
 *
 * A pointer to the device with the incremented reference counter is returned.
 */
struct cape_dev *capebus_dev_get(struct cape_dev *dev)
{
	if (dev)
		get_device(&dev->dev);
	return dev;
}
EXPORT_SYMBOL(capebus_dev_get);

/**
 * capebus_dev_put - release a use of the capebus device structure
 * @dev: device that's been disconnected
 *
 * Must be called when a user of a device is finished with it.  When the last
 * user of the device calls this function, the memory of the device is freed.
 */
void capebus_dev_put(struct cape_dev *dev)
{
	if (dev)
		put_device(&dev->dev);
}
EXPORT_SYMBOL(capebus_dev_put);

static int __init capebus_driver_init(void)
{
	return bus_register(&capebus_bus_type);
}

postcore_initcall(capebus_driver_init);

const struct of_device_id *
capebus_of_match_device(struct cape_dev *cdev,
		const char *property, const char *value)
{
	struct cape_bus *bus = cdev->bus;
	struct device *dev = &cdev->dev;
	struct device_node *pnode = cape_bus_to_parent_of_node(bus);
	struct device_node *node;
	const struct of_device_id *match;
	const char* cp;
	int cplen, l;

	dev_dbg(dev, "Iterating on parent of node "
			"name='%s' type='%s' full_name='%s'\n",
			pnode->name, pnode->type, pnode->full_name);

	match = NULL;
	for_each_child_of_node(pnode, node) {

		dev->of_node = node;
		match = of_match_device(dev->driver->of_match_table, dev);
		if (!match)
			goto next_node;

		cp = of_get_property(node, property, &cplen);
		if (cp == NULL)
			goto next_node;

		while (cplen > 0) {
			if (of_compat_cmp(cp, value, strlen(value)) == 0)
				break;
			l = strlen(cp) + 1;
			cp += l;
			cplen -= l;
		}

		/* matched */
		if (cplen > 0)
			break;
next_node:
		match = NULL;
		dev->of_node = NULL;
	}

	if (match == NULL) {
		dev_dbg(dev, "Failed to find matching child-node\n");
		return NULL;
	}

	dev_dbg(dev, "Found matching child node "
			"name='%s' type='%s' "
			"full_name='%s' (compatible='%s')\n",
		node->name, node->type, node->full_name,
		match->compatible);

	return match;
}
EXPORT_SYMBOL(capebus_of_match_device);

struct device_node *
capebus_of_compatible_device_property_match(struct cape_dev *dev,
		const struct of_device_id *matches,
		const char *prop, const char *prop_value)
{
	const struct of_device_id *match;
	struct device_node *node, *cnode;
	const char* cp;
	int cplen, l;

	if (prop == NULL || prop_value == NULL)
		goto try_non_property;

	/* at first try secondary match */
	for_each_child_of_node(dev->dev.of_node, node) {

		cp = of_get_property(node, prop, &cplen);
		if (cp == NULL)
			continue;

		while (cplen > 0) {
			if (of_compat_cmp(cp, prop_value,
						strlen(prop_value)) == 0)
				break;
			l = strlen(cp) + 1;
			cp += l;
			cplen -= l;
		}

		/* not matched */
		if (cplen <= 0)
			continue;

		/* now iterate in the children nodes */
		for_each_child_of_node(node, cnode) {

			match = of_match_node(matches, cnode);
			if (match) {
				/* release reference to parent, keep this one */
				of_node_put(node);
				return cnode;
			}
		}
	}

try_non_property:
	for_each_child_of_node(dev->dev.of_node, node) {

		match = of_match_node(matches, node);
		if (match)
			return node;
	}

	return NULL;
}
EXPORT_SYMBOL(capebus_of_compatible_device_property_match);

struct platform_device *
capebus_of_platform_compatible_device_create(struct cape_dev *dev,
		const struct of_device_id *matches,
		const char *pdev_name,
		const char *prop, const char *prop_value)
{
	struct device_node *node;
	struct platform_device *pdev;

	node = capebus_of_compatible_device_property_match(dev, matches, prop,
			prop_value);
	if (node == NULL)
		return ERR_PTR(-ENXIO);

	pdev = of_platform_device_create(node, pdev_name, dev->bus->dev.parent);

	/* release the reference to the node */
	of_node_put(node);
	node = NULL;

	if (pdev == NULL) {
		dev_err(&dev->dev, "Failed to create platform device '%s'\n",
				pdev_name);
		return ERR_PTR(-ENODEV);
	}

	return pdev;
}
EXPORT_SYMBOL(capebus_of_platform_compatible_device_create);

struct device_node *
capebus_of_find_property_node(struct cape_dev *dev,
		const char *prop, const char *prop_value,
		const char *name)
{
	struct device_node *node;
	const char* cp;
	int cplen, l;
	struct property *pp;

	node = NULL;
	if (prop == NULL || prop_value == NULL)
		goto find_direct;

	/* at first try secondary match */
	for_each_child_of_node(dev->dev.of_node, node) {

		cp = of_get_property(node, prop, &cplen);
		if (cp == NULL)
			continue;

		while (cplen > 0) {
			if (of_compat_cmp(cp, prop_value,
						strlen(prop_value)) == 0)
				break;
			l = strlen(cp) + 1;
			cp += l;
			cplen -= l;
		}

		/* not matched */
		if (cplen <= 0)
			continue;

		/* found ? */
		pp = of_find_property(node, name, NULL);
		if (pp != NULL)
			return node;
	}
find_direct:
	pp = of_find_property(dev->dev.of_node, name, NULL);
	if (pp == NULL)
		return NULL;

	return of_node_get(dev->dev.of_node);
}
EXPORT_SYMBOL_GPL(capebus_of_find_property_node);

struct property *
capebus_of_find_property(struct cape_dev *dev,
		const char *prop, const char *prop_value,
		const char *name, int *lenp)
{
	struct device_node *node;
	struct property *pp;

	node = capebus_of_find_property_node(dev, prop, prop_value, name);
	if (node == NULL)
		return NULL;

	pp = of_find_property(node, name, lenp);

	of_node_put(node);

	return pp;
}
EXPORT_SYMBOL_GPL(capebus_of_find_property);

const void *capebus_of_get_property(struct cape_dev *dev,
		const char *prop, const char *prop_value,
		const char *name, int *lenp)
{
	struct property *pp;

	pp = capebus_of_find_property(dev, prop, prop_value, name, lenp);
	return pp ? pp->value : NULL;
}
EXPORT_SYMBOL_GPL(capebus_of_get_property);

/* node exists, but it's not available? make it so */
int capebus_of_device_node_enable(struct device_node *node)
{
	struct property *prop;
	int ret;

	prop = kzalloc(sizeof(*prop), GFP_KERNEL);
	if (prop == NULL)
		goto err_no_prop_mem;

	prop->name = kstrdup("status", GFP_KERNEL);
	if (prop->name == NULL)
		goto err_no_name_mem;

	prop->value = kstrdup("okay", GFP_KERNEL);
	if (prop->value == NULL)
		goto err_no_value_mem;

	prop->length = strlen(prop->value) + 1;
	set_bit(OF_DYNAMIC, &prop->_flags);

	ret = prom_update_property(node, prop);
	if (ret != 0)
		goto err_update_failed;

	return 0;

err_update_failed:
	kfree(prop->value);
err_no_value_mem:
	kfree(prop->name);
err_no_name_mem:
	kfree(prop);
err_no_prop_mem:
	return -ENOMEM;
}
EXPORT_SYMBOL_GPL(capebus_of_device_node_enable);

/* Make sure this node is activated (even if it was disabled) */
int capebus_of_platform_device_enable(struct device_node *node)
{
	struct platform_device *pdev, *ppdev;
	int ret;

	if (of_device_is_available(node))
		return 0;

	ret = capebus_of_device_node_enable(node);
	if (ret != 0)
		return ret;

	/* now we need to find the parent of the node */
	ppdev = of_find_device_by_node(node->parent);

	pdev = of_platform_device_create(node, NULL,
			ppdev ? &ppdev->dev : NULL);
	if (IS_ERR_OR_NULL(pdev)) {
		ret = pdev ? PTR_ERR(pdev) : -ENODEV;
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(capebus_of_platform_device_enable);
