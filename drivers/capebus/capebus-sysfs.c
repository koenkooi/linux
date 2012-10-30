/*
 * drivers/capebus/capebus-sysfs.c
 *
 * sysfs for capebus devices
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
 *
 * Modeled after PCI's pci-sysfs.c
 *
 */

#include <linux/kernel.h>
#include <linux/stat.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>

#include <linux/capebus.h>

static ssize_t id_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct cape_dev *cdev;

	cdev = to_cape_dev(dev);
	return sprintf(buf, "%s\n", cdev->text_id);
}

struct device_attribute capebus_dev_attrs[] = {
	__ATTR_RO(id),
	__ATTR_NULL,
};

struct bus_attribute capebus_bus_attrs[] = {
	__ATTR_NULL
};
