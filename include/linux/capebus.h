/*
 * capebus.h
 *
 * Cape bus defines and function prototypes
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
#ifndef LINUX_CAPEBUS_H
#define LINUX_CAPEBUS_H

#include <linux/list.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/of.h>
#include <linux/of_device.h>

struct cape_device_id {
	const char *cntrlboard;	/* controlling board; i.e. "beaglebone" */
	int len;		/* opaque addressing data */
	const void *data;
};

struct cape_dev;
struct cape_bus;
struct cape_slot;

struct cape_slot {
	struct list_head	node;
	struct cape_bus 	*bus;	/* the bus this slot is on */
	int 			slotno;	/* index of this slot */
	struct cape_dev		*dev;	/* the device (if found) */
};

struct cape_driver {
	struct list_head node;
	int (*probe)(struct cape_dev *dev, const struct cape_device_id *id);
	void (*remove)(struct cape_dev *dev);
	int  (*suspend) (struct cape_dev *dev, pm_message_t state);
	int  (*suspend_late) (struct cape_dev *dev, pm_message_t state);
	int  (*resume_early) (struct cape_dev *dev);
	int  (*resume) (struct cape_dev *dev);
	void (*shutdown) (struct cape_dev *dev);
	struct device_driver driver;
};

/*
 * capebus_register_driver must be a macro so that
 * KBUILD_MODNAME can be expanded
 */
#define capebus_register_driver(driver)		\
	__capebus_register_driver(driver, THIS_MODULE, KBUILD_MODNAME)

int __capebus_register_driver(struct cape_driver *drv, struct module *owner,
			  const char *mod_name);

void capebus_unregister_driver(struct cape_driver *dev);

/**
 * module_capebus_driver() - Helper macro for registering a capebus driver
 * @__capebus_driver: capebus_driver struct
 *
 * Helper macro for capebus drivers which do not do anything special in module
 * init/exit. This eliminates a lot of boilerplate. Each module may only
 * use this macro once, and calling it replaces module_init() and module_exit()
 */
#define module_capebus_driver(__capebus_driver) \
	module_driver(__capebus_driver, capebus_register_driver, \
		       capebus_unregister_driver)

#define	to_cape_driver(n) container_of(n, struct cape_driver, driver)

struct cape_bus_ops {
	const struct cape_device_id *(*get_dev_id)(struct cape_slot *slot);
	const char *(*get_text_dev_id)(struct cape_slot *slot);
	int (*dev_probed)(struct cape_dev *dev);	/* probed succesfully */
	void (*dev_removed)(struct cape_dev *dev);	/* removed */
	int (*dev_registered)(struct cape_dev *dev);	/* registered OK */
};

struct cape_bus {
	struct list_head	node;
	const char		*name;
	struct list_head	devices;
	struct cape_dev		*self;
	struct list_head	slots;
	struct cape_bus_ops	*ops;
	int			busno;
	struct device		dev;
	/* TODO: resources.... */
};

#define	to_cape_bus(n) container_of(n, struct cape_bus, dev)

#define cape_bus_to_parent_of_node(n) ((n)->dev.parent->of_node)

struct cape_dev {
	struct list_head	bus_list;	/* node in per-bus list     */
	struct cape_bus		*bus;		/* bus this device is on    */
	struct cape_slot	*slot;		/* cape slot of this device */
	struct cape_driver	*driver;	/* driver of this device    */
	struct device		dev;
	atomic_t		enable_cnt;	/* capebus_enable_device    */
						/* has been called          */
	const struct cape_device_id *id;
	const char 		*text_id;
	unsigned int		added : 1;	/* device has been added    */
	void			*drv_priv;	/* driver private data      */
};

#define	to_cape_dev(n) container_of(n, struct cape_dev, dev)

struct cape_dev *capebus_dev_get(struct cape_dev *dev);
void capebus_dev_put(struct cape_dev *dev);

/* must have cape_buses_mutex */
#define cape_bus_for_each(_bus) \
	list_for_each_entry(_bus, &cape_buses, node)

#define cape_bus_for_each_safe(_bus, _busn) \
	list_for_each_entry_safe(_bus, _busn, &cape_buses, node)

int cape_bus_register(struct cape_bus *bus, const char *name, int busno,
		struct device *parent, struct cape_bus_ops *ops);

/* must have cape_buses_mutex */
#define cape_slot_for_each(_bus, _slot) \
	list_for_each_entry(_slot, &(_bus)->slots, node)

#define cape_slot_for_each_safe(_bus, _slot, _slotn) \
	list_for_each_entry_safe(_slot, _slotn, &(_bus)->slots, node)

int cape_bus_register_slot(struct cape_bus *bus,
		struct cape_slot *slot, int slotno);

int cape_bus_scan_one_slot(struct cape_bus *bus, struct cape_slot *slot);
int cape_bus_scan(struct cape_bus *bus);

extern struct list_head cape_buses;
extern struct mutex cape_buses_mutex;

static inline int capebus_is_enabled(struct cape_dev *cdev)
{
	return atomic_read(&cdev->enable_cnt) > 0;
}

static inline int capebus_enable_device(struct cape_dev *cdev)
{
	if (atomic_add_return(1, &cdev->enable_cnt) > 1)
		return 0;		/* already enabled */

	/* XXX do enable */

	return 0;
}

static inline void capebus_disable_device(struct cape_dev *cdev)
{
	if (atomic_sub_return(1, &cdev->enable_cnt) != 0)
		return;

	/* callback to disable device? */
}

static inline int capebus_enable_wake(struct cape_dev *dev, int what)
{
	return 0;
}

extern struct device_attribute capebus_dev_attrs[];
extern struct bus_attribute capebus_bus_attrs[];

extern struct bus_type capebus_bus_type;

const struct of_device_id *
capebus_of_match_device(struct cape_dev *cdev,
		const char *property, const char *value);

struct device_node *
capebus_of_compatible_device_property_match(struct cape_dev *dev,
		const struct of_device_id *matches,
		const char *prop, const char *prop_value);

struct platform_device *
capebus_of_platform_compatible_device_create(struct cape_dev *dev,
		const struct of_device_id *matches,
		const char *pdev_name,
		const char *prop, const char *prop_value);

/* of tree support */

struct device_node *
capebus_of_find_property_node(struct cape_dev *dev,
		const char *prop, const char *prop_value,
		const char *name);

struct property *
capebus_of_find_property(struct cape_dev *dev,
		const char *prop, const char *prop_value,
		const char *name, int *lenp);

const void *capebus_of_get_property(struct cape_dev *dev,
		const char *prop, const char *prop_value,
		const char *name, int *lenp);

static inline int capebus_of_property_read_u32_array(struct cape_dev *dev,
		const char *prop, const char *prop_value,
		const char *name, u32 *out_values, size_t sz)
{
	struct device_node *node;
	int ret;

	node = capebus_of_find_property_node(dev, prop, prop_value, name);
	ret = of_property_read_u32_array(node, name, out_values, sz);
	of_node_put(node);
	return ret;
}

static inline int capebus_of_property_read_u32(struct cape_dev *dev,
		const char *prop, const char *prop_value,
	       const char *name, u32 *out_value)
{
	return capebus_of_property_read_u32_array(dev, prop,
			prop_value, name, out_value, 1);
}

static inline bool capebus_of_property_read_bool(struct cape_dev *dev,
		const char *prop, const char *prop_value,
		const char *name)
{
	struct device_node *node;
	bool ret;

	node = capebus_of_find_property_node(dev, prop, prop_value, name);
	ret = of_property_read_bool(node, name);
	of_node_put(node);
	return ret;
}

static inline int capebus_of_property_read_string(struct cape_dev *dev,
		const char *prop, const char *prop_value,
		const char *name, const char **out_string)
{
	struct device_node *node;
	int ret;

	node = capebus_of_find_property_node(dev, prop, prop_value, name);
	ret = of_property_read_string(node, name, out_string);
	of_node_put(node);
	return ret;
}

static inline int capebus_of_property_read_string_index(struct cape_dev *dev,
		const char *prop, const char *prop_value,
		const char *name, int index, const char **out_string)
{
	struct device_node *node;
	int ret;

	node = capebus_of_find_property_node(dev, prop, prop_value, name);
	ret = of_property_read_string_index(node, name, index, out_string);
	of_node_put(node);
	return ret;
}

static inline int capebus_of_property_read_u64(struct cape_dev *dev,
		const char *prop, const char *prop_value,
		const char *name, u64 *out_value)
{
	struct device_node *node;
	int ret;

	node = capebus_of_find_property_node(dev, prop, prop_value, name);
	ret = of_property_read_u64(node, name, out_value);
	of_node_put(node);
	return ret;
}

int capebus_of_device_node_enable(struct device_node *node);
int capebus_of_platform_device_enable(struct device_node *node);

#endif
