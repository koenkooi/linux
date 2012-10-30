/*
 * TI Beaglebone capebus controller
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
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_i2c.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/pinctrl/consumer.h>
#include <linux/err.h>
#include <linux/ctype.h>

#include <linux/capebus.h>
#include <linux/capebus/capebus-bone.h>

/* what to fill in to the cntrlboard field of the id */
#define BONE_CAPEBUS_CNTRLBOARD		"beaglebone"

/* various EEPROM definition for the bone */
struct bone_capebus_eeprom_field {
	const char 	*name;
	int 		start;
	int		size;
	unsigned int	ascii : 1;
	unsigned int	strip_trailing_dots : 1;
	const char	*override;
};

static const struct bone_capebus_eeprom_field eeprom_fields[] = {
	[BONE_CAPEBUS_HEADER] = {
		.name		= "header",
		.start		= 0,
		.size		= 4,
		.ascii		= 0,
		.override	= "\xaa\x55\x33\xee",	/* AA 55 33 EE */
	},
	[BONE_CAPEBUS_EEPROM_REV] = {
		.name		= "eeprom-format-revision",
		.start		= 4,
		.size		= 2,
		.ascii		= 1,
		.override	= "A0",
	},
	[BONE_CAPEBUS_BOARD_NAME] = {
		.name		= "board-name",
		.start		= 6,
		.size		= 32,
		.ascii		= 1,
		.strip_trailing_dots = 1,
		.override	= "Override Board Name",
	},
	[BONE_CAPEBUS_VERSION] = {
		.name		= "version",
		.start		= 38,
		.size		= 4,
		.ascii		= 1,
		.override	= "00A0",
	},
	[BONE_CAPEBUS_MANUFACTURER] = {
		.name		= "manufacturer",
		.start		= 42,
		.size		= 16,
		.ascii		= 1,
		.strip_trailing_dots = 1,
		.override	= "Override Manuf",
	},
	[BONE_CAPEBUS_PART_NUMBER] = {
		.name		= "part-number",
		.start		= 58,
		.size		= 16,
		.ascii		= 1,
		.override	= "Override Part#",
	},
	[BONE_CAPEBUS_NUMBER_OF_PINS] = {
		.name		= "number-of-pins",
		.start		= 74,
		.size		= 2,
		.ascii		= 0,
		.override	= NULL,
	},
	[BONE_CAPEBUS_SERIAL_NUMBER] = {
		.name		= "serial-number",
		.start		= 76,
		.size		= 12,
		.ascii		= 1,
		.override	= "0000000000",
	},
	[BONE_CAPEBUS_PIN_USAGE] = {
		.name		= "pin-usage",
		.start		= 88,
		.size		= 140,
		.ascii		= 0,
		.override	= NULL,
	},
	[BONE_CAPEBUS_VDD_3V3EXP] = {
		.name		= "vdd-3v3exp",
		.start		= 228,
		.size		= 2,
		.ascii		= 0,
		.override	= NULL,
	},
	[BONE_CAPEBUS_VDD_5V] = {
		.name		= "vdd-5v",
		.start		= 230,
		.size		= 2,
		.ascii		= 0,
		.override	= NULL,
	},
	[BONE_CAPEBUS_SYS_5V] = {
		.name		= "sys-5v",
		.start		= 232,
		.size		= 2,
		.ascii		= 0,
		.override	= NULL,
	},
	[BONE_CAPEBUS_DC_SUPPLIED] = {
		.name		= "dc-supplied",
		.start		= 234,
		.size		= 2,
		.ascii		= 0,
		.override	= NULL,
	},
};

char *bone_capebus_id_get_field(const struct cape_device_id *id,
		int field, char *buf, int bufsz)
{
	const struct bone_capebus_eeprom_field *ee_field;
	int len;

	/* make sure the ID is valid for the bone */
	if (bone_capebus_match_cntrlboard(id) != 0)
		return NULL;

	if ((unsigned int)field >= ARRAY_SIZE(eeprom_fields))
		return NULL;

	ee_field = &eeprom_fields[field];

	/* enough space? */
	if (bufsz < ee_field->size + ee_field->ascii)
		return NULL;

	memcpy(buf, (char *)id->data + ee_field->start, ee_field->size);

	/* terminate ascii field */
	if (ee_field->ascii)
		buf[ee_field->size] = '\0';;

	if (ee_field->strip_trailing_dots) {
		len = strlen(buf);
		while (len > 1 && buf[len - 1] == '.')
			buf[--len] = '\0';
	}

	return buf;
}
EXPORT_SYMBOL(bone_capebus_id_get_field);

int bone_capebus_match_cntrlboard(const struct cape_device_id *id)
{
	if (strcmp(id->cntrlboard, BONE_CAPEBUS_CNTRLBOARD) != 0)
		return -ENODEV;
	return 0;
}
EXPORT_SYMBOL(bone_capebus_match_cntrlboard);

int bone_capebus_match_board(const struct cape_device_id *id,
		const char **board_names)
{
	char rname[33];
	const char *s;
	int ret;
	int i;

	/* be safe; check for matching cntrlboard */
	ret = bone_capebus_match_cntrlboard(id);
	if (ret != 0)
		return ret;

	s = bone_capebus_id_get_field(id, BONE_CAPEBUS_BOARD_NAME,
			rname, sizeof(rname));
	if (s == NULL)
		return -EINVAL;

	i = 0;
	while (*board_names) {
		if (strcmp(rname, *board_names) == 0)
			return i;
		board_names++;
	}

	return -1;
}
EXPORT_SYMBOL(bone_capebus_match_board);

#ifdef CONFIG_OF
static const struct of_device_id bone_capebus_of_match[] = {
	{
		.compatible = "bone-capebus",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, bone_capebus_of_match);

static const struct of_device_id slot_override_of_match[] = {
	{
		.compatible = "bone-capebus-slot-override",
	},
	{ },
};

#endif

const struct cape_device_id *bone_capebus_get_dev_id(struct cape_slot *slot)
{
	struct cape_bus *bus = slot->bus;
	struct bone_capebus_slot *bone_slot = to_bone_capebus_slot(slot);
	struct i2c_client *client = bone_slot->client;
	struct cape_device_id *id;
	const u8 *p;
	int r;
	char board_name[32+1];
	char version[4+1];
	char manufacturer[16+1];
	char part_number[16+1];

	id = &bone_slot->id;

	/* need to read EEPROM? */
	if (!bone_slot->eeprom_probed) {

		bone_slot->eeprom_probed = 1;

		if (!bone_slot->eeprom_override) {
			r = i2c_memory_read(bone_slot->client,
				bone_slot->eeprom_signature, 0,
				sizeof(bone_slot->eeprom_signature));
			if (r != sizeof(bone_slot->eeprom_signature)) {
				dev_err(&bus->dev,
					"bone: Failed to read EEPROM at "
					"slot %d (addr 0x%02x)\n",
					slot->slotno, client->addr & 0x7f);
				bone_slot->eeprom_failed = 1;
				return NULL;
			}
		} else
			dev_info(&bus->dev,
				"bone: Using override eeprom data at slot %d\n",
				slot->slotno);

		p = bone_slot->eeprom_signature;
		if (BONE_CAPEBUS_MAKE_HEADER(p) != BONE_CAPEBUS_HEADER_VALID) {
			dev_err(&bus->dev, "bone: Invalid EEPROM signature "
				"'%08x' at slot %d (addr 0x%02x)\n",
				BONE_CAPEBUS_MAKE_HEADER(p),
				slot->slotno, client->addr & 0x7f);
			bone_slot->eeprom_failed = 1;
			return NULL;
		}

		bone_slot->id.cntrlboard = BONE_CAPEBUS_CNTRLBOARD;
		bone_slot->id.len = sizeof(bone_slot->eeprom_signature);
		bone_slot->id.data = bone_slot->eeprom_signature;

		bone_capebus_id_get_field(id, BONE_CAPEBUS_BOARD_NAME,
					board_name, sizeof(board_name));
		bone_capebus_id_get_field(id, BONE_CAPEBUS_VERSION,
					version, sizeof(version));
		bone_capebus_id_get_field(id, BONE_CAPEBUS_MANUFACTURER,
					manufacturer, sizeof(manufacturer));
		bone_capebus_id_get_field(id, BONE_CAPEBUS_PART_NUMBER,
					part_number, sizeof(part_number));

		/* board_name,version,manufacturer,part_number */
		snprintf(bone_slot->text_id, sizeof(bone_slot->text_id) - 1,
				"%s,%s,%s,%s", board_name, version,
				manufacturer, part_number);

		/* terminate always */
		bone_slot->text_id[sizeof(bone_slot->text_id) - 1] = '\0';

	}

	/* slot has failed and we don't support hotpluging */
	if (bone_slot->eeprom_failed)
		return NULL;

	return id;
}

const char *bone_capebus_get_text_dev_id(struct cape_slot *slot)
{
	struct bone_capebus_slot *bone_slot = to_bone_capebus_slot(slot);

	if (bone_slot->eeprom_failed || !bone_slot->eeprom_probed)
		return NULL;

	return bone_slot->text_id;
}

struct bonedev_ee_attribute {
	struct device_attribute devattr;
	unsigned int field;
};
#define to_bonedev_ee_attribute(x) \
	container_of((x), struct bonedev_ee_attribute, devattr)

static ssize_t bonedev_ee_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bonedev_ee_attribute *ee_attr = to_bonedev_ee_attribute(attr);
	struct cape_dev *cdev = to_cape_dev(dev);
	const struct cape_device_id *id = cdev->id;
	const struct bone_capebus_eeprom_field *ee_field;
	int i, len;
	char *p, *s;
	u16 val;

	if (id == NULL)
		return -EINVAL;

	/* add newline for ascii fields */
	ee_field = &eeprom_fields[ee_attr->field];

	len = ee_field->size + ee_field->ascii;
	p = kmalloc(len, GFP_KERNEL);
	if (p == NULL)
		return -ENOMEM;

	s = bone_capebus_id_get_field(id, ee_attr->field, p, len);
	if (s == NULL)
		return -EINVAL;

	/* add newline for ascii fields and return */
	if (ee_field->ascii) {
		len = sprintf(buf, "%s\n", s);
		goto out;
	}

	/* case by case handling */
	switch (ee_attr->field) {
		case BONE_CAPEBUS_HEADER:
			len = sprintf(buf, "%02x %02x %02x %02x\n",
					s[0], s[1], s[2], s[3]);
			break;

			/* 2 bytes */
		case BONE_CAPEBUS_NUMBER_OF_PINS:
		case BONE_CAPEBUS_VDD_3V3EXP:
		case BONE_CAPEBUS_VDD_5V:
		case BONE_CAPEBUS_SYS_5V:
		case BONE_CAPEBUS_DC_SUPPLIED:
			/* the bone is LE */
			val = s[0] & (s[1] << 8);
			len = sprintf(buf, "%u\n", (unsigned int)val & 0xffff);
			break;

		case BONE_CAPEBUS_PIN_USAGE:

			len = 0;
			for (i = 0; i < ee_field->size / 2; i++) {
				/* the bone is LE */
				val = s[0] & (s[1] << 8);
				sprintf(buf, "%04x\n", val);
				buf += 5;
				len += 5;
				s += 2;
			}

			break;

		default:
			*buf = '\0';
			len = 0;
			break;
	}

out:
	kfree(p);

	return len;
}

#define BONEDEV_EE_ATTR(_name, _field) \
	{ \
		.devattr = __ATTR(_name, 0440, bonedev_ee_show, NULL), \
		.field = BONE_CAPEBUS_##_field , \
	}

struct bonedev_ee_attribute ee_attrs[] = {
	BONEDEV_EE_ATTR(header, HEADER),
	BONEDEV_EE_ATTR(eeprom-format-revision, EEPROM_REV),
	BONEDEV_EE_ATTR(board-name, BOARD_NAME),
	BONEDEV_EE_ATTR(version, VERSION),
	BONEDEV_EE_ATTR(manufacturer, MANUFACTURER),
	BONEDEV_EE_ATTR(part-number, PART_NUMBER),
	BONEDEV_EE_ATTR(number-of-pins, NUMBER_OF_PINS),
	BONEDEV_EE_ATTR(serial-number, SERIAL_NUMBER),
	BONEDEV_EE_ATTR(pin-usage, PIN_USAGE),
	BONEDEV_EE_ATTR(vdd-3v3exp, VDD_3V3EXP),
	BONEDEV_EE_ATTR(vdd-5v, VDD_5V),
	BONEDEV_EE_ATTR(sys-5v, SYS_5V),
	BONEDEV_EE_ATTR(dc-supplied, DC_SUPPLIED),
};

static struct attribute *ee_attrs_flat[] = {
	&ee_attrs[BONE_CAPEBUS_HEADER		].devattr.attr,
	&ee_attrs[BONE_CAPEBUS_EEPROM_REV	].devattr.attr,
	&ee_attrs[BONE_CAPEBUS_BOARD_NAME	].devattr.attr,
	&ee_attrs[BONE_CAPEBUS_VERSION		].devattr.attr,
	&ee_attrs[BONE_CAPEBUS_MANUFACTURER	].devattr.attr,
	&ee_attrs[BONE_CAPEBUS_PART_NUMBER	].devattr.attr,
	&ee_attrs[BONE_CAPEBUS_NUMBER_OF_PINS	].devattr.attr,
	&ee_attrs[BONE_CAPEBUS_SERIAL_NUMBER	].devattr.attr,
	&ee_attrs[BONE_CAPEBUS_PIN_USAGE	].devattr.attr,
	&ee_attrs[BONE_CAPEBUS_VDD_3V3EXP	].devattr.attr,
	&ee_attrs[BONE_CAPEBUS_VDD_5V		].devattr.attr,
	&ee_attrs[BONE_CAPEBUS_SYS_5V		].devattr.attr,
	&ee_attrs[BONE_CAPEBUS_DC_SUPPLIED	].devattr.attr,
	NULL,
};

static const struct attribute_group bone_ee_attrgroup = {
	.name		= "ee-fields",
	.is_visible 	= NULL,
	.attrs 		= ee_attrs_flat,
};

static int bone_capebus_sysfs_register(struct cape_dev *dev)
{
	return sysfs_create_group(&dev->dev.kobj, &bone_ee_attrgroup);
}

static void bone_capebus_sysfs_unregister(struct cape_dev *dev)
{
	sysfs_remove_group(&dev->dev.kobj, &bone_ee_attrgroup);
}

static int bone_capebus_dev_probed(struct cape_dev *dev)
{
	return 0;
}

static void bone_capebus_dev_removed(struct cape_dev *dev)
{
	bone_capebus_sysfs_unregister(dev);
}

static int bone_capebus_dev_registered(struct cape_dev *dev)
{
	int ret;

	ret = bone_capebus_sysfs_register(dev);
	if (ret != 0) {
		dev_err(&dev->dev, "bone_capebus sysfs registration failed\n");
		return ret;
	}

	return 0;
}

static struct cape_bus_ops bone_capebus_ops = {
	.get_dev_id 		= bone_capebus_get_dev_id,
	.get_text_dev_id	= bone_capebus_get_text_dev_id,
	.dev_probed		= bone_capebus_dev_probed,
	.dev_removed		= bone_capebus_dev_removed,
	.dev_registered		= bone_capebus_dev_registered,
};

static ssize_t slots_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bone_capebus_bus	*bus = platform_get_drvdata(pdev);
	struct bone_capebus_slot *slot;
	ssize_t len, sz;
	int i;

	sz = 0;

	for (i = 0; i < bus->slots_nr; i++) {
		slot = &bus->slots[i];

		len = sprintf(buf, "%02x:%c%c%c%c %s\n",
				(int)slot->eeprom_addr & 0x7f,
				slot->eeprom_probed     ? 'P' : '-',
				slot->eeprom_failed     ? 'F' : '-',
				slot->eeprom_override   ? 'O' : '-',
				(slot->cape_slot.dev && slot->cape_slot.dev->added) ? 'A' : '-',
				slot->text_id);

		buf += len;
		sz += len;
	}
	return sz;
}

static ssize_t slots_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bone_capebus_bus	*bus = platform_get_drvdata(pdev);
	int slotno, err, i, len;
	char *s, *board_name, *version, *p;
	const struct bone_capebus_eeprom_field *ee_field, *eebrd, *eevrs;
	struct bone_capebus_slot *slot;

	eebrd = &eeprom_fields[BONE_CAPEBUS_BOARD_NAME];
	eevrs = &eeprom_fields[BONE_CAPEBUS_VERSION];

	slotno = simple_strtoul(buf, &s, 10);
	if (slotno < 0 || slotno >= bus->slots_nr)
		return -EINVAL;
	slot = &bus->slots[slotno];
	if (slot->eeprom_override || (slot->cape_slot.dev && slot->cape_slot.dev->added))
		return -EINVAL;

	board_name = kzalloc(eebrd->size + 1 + eevrs->size + 1, GFP_KERNEL);
	if (board_name == NULL)
		return -ENOMEM;
	version = board_name + eebrd->size + 1;

	s = strchr(s, ':');
	if (s == NULL) {
		kfree(board_name);
		return -EINVAL;
	}
	s++;
	p = strchr(s, ':');
	if (p == NULL) {
		len = strlen(s);
		strncpy(board_name, s, eebrd->size);
		strcpy(version, "00A0");
	} else {
		len = p - s;
		if (len > eebrd->size)
			len = p - s;
		memcpy(board_name, s, len);
		board_name[len] = '\0';
		strncpy(version, p + 1, eevrs->size);
	}
	board_name[eebrd->size] = '\0';
	version[eevrs->size]  = '\0';

	/* strip trailing spaces, dots & newlines */
	s = board_name + strlen(board_name);
	while (s > board_name &&
			(isspace(s[-1]) || s[-1] == '\n' || s[-1] == '.'))
		*--s = '\0';

	printk(KERN_INFO "Override for slot #%d, board-name '%s', version '%s'\n",
			slotno, board_name, version);

	slot->eeprom_override = 1;
	slot->eeprom_failed = 0;
	slot->eeprom_probed = 0;

	/* zero out signature */
	memset(slot->eeprom_signature, 0,
			sizeof(slot->eeprom_signature));

	/* create an eeprom field */
	for (i = 0; i < ARRAY_SIZE(eeprom_fields); i++) {

		ee_field = &eeprom_fields[i];

		/* point to the entry */
		p = slot->eeprom_signature + ee_field->start;

		/* if no such property, assign default */
		if (i != BONE_CAPEBUS_BOARD_NAME) {

			if (ee_field->override)
				memcpy(p, ee_field->override,
						ee_field->size);
			else
				memset(p, 0, ee_field->size);

			continue;
		}

		/* copy it to the eeprom signature buf */
		len = strlen(board_name);
		if (len > ee_field->size)
			len = ee_field->size;

		/* copy and zero out rest */
		memcpy(p, board_name, len);
		if (len < ee_field->size)
			memset(p + len, 0, ee_field->size - len);
	}

	printk(KERN_INFO "calling cape_bus_scan_one_slot\n");
	err = cape_bus_scan_one_slot(&bus->cape_bus, &slot->cape_slot);

	printk(KERN_INFO "cape_bus_scan_one_slot returned %d\n", err);

	/* failed to scan... */
	if (err != 0)
		slot->eeprom_override = 0;

	kfree(board_name);

	return strlen(buf);
}

static DEVICE_ATTR(slots, 0644, slots_show, slots_store);

static int bone_capebus_bus_sysfs_register(struct bone_capebus_bus *bus)
{
	return device_create_file(bus->dev, &dev_attr_slots);
}

static void bone_capebus_bus_sysfs_unregister(struct bone_capebus_bus *bus)
{
	device_remove_file(bus->dev, &dev_attr_slots);
}

static int __devinit
bone_capebus_probe(struct platform_device *pdev)
{
	struct bone_capebus_bus	*bus;
	struct device_node	*pnode = pdev->dev.of_node;
	const struct of_device_id *cntrlboard_match;
	const struct of_device_id *dev_match;
	struct bone_capebus_slot *slot;
	const struct bone_capebus_eeprom_field *ee_field;
	struct property *prop;
	int length;
	int r;
	struct device_node *node;
	struct i2c_client *client;
	phandle handle;
	u32 *slot_handles = NULL;
	u32 val;
	const char *str;
	u8 *p;
	int i, len;

	/* we don't use platform_data */

	bus = devm_kzalloc(&pdev->dev,
			sizeof(struct bone_capebus_bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&pdev->dev, "Failed to allocate device structure\n");
		return -ENOMEM;
	}

	/* register the cape bus */
	r = cape_bus_register(&bus->cape_bus, "bone", 0, &pdev->dev,
			&bone_capebus_ops);
	if (r != 0) {
		dev_err(&pdev->dev, "Failed to register the cape device\n");
		return r;
	}

	cntrlboard_match = of_match_device(of_match_ptr(bone_capebus_of_match),
			&pdev->dev);
	if (!cntrlboard_match) {
		dev_err(&pdev->dev, "Failed to configure bone capebus\n");
		return -ENODEV;
	}
	bus->dev = &pdev->dev;

	prop = of_find_property(pnode, "slots", &length);
	if (prop == NULL) {
		dev_err(&pdev->dev, "Unable to find required "
				"property 'slots'\n");
		return -EINVAL;
	}
	bus->slots_nr = length / sizeof(u32);
	bus->slots = devm_kzalloc(&pdev->dev,
			sizeof(bus->slots[0]) * bus->slots_nr, GFP_KERNEL);
	if (!bus->slots) {
		dev_err(&pdev->dev, "Failed to allocate %d slot areas\n",
				bus->slots_nr);
		return -ENOMEM;
	}
	slot_handles = devm_kzalloc(&pdev->dev, length, GFP_KERNEL);
	if (!slot_handles) {
		dev_err(&pdev->dev, "Failed to allocate %d slot areas\n",
				bus->slots_nr);
		return -ENOMEM;
	}
	r = of_property_read_u32_array(pnode, "slots",
			slot_handles, bus->slots_nr);
	if (r < 0) {
		dev_err(&pdev->dev, "Failed to read %d slot handles\n",
				bus->slots_nr);
		return r;
	}

	/* now we iterate over any overrides */
	for_each_child_of_node(pnode, node) {

		dev_match = of_match_node(slot_override_of_match, node);
		if (!dev_match)
			continue;

		/* no reg property */
		if (of_property_read_u32(node, "slot", &val) != 0) {
			dev_warn(&pdev->dev, "override: Failed to read "
					"slot property\n");
			continue;
		}

		if (val >= bus->slots_nr) {
			dev_warn(&pdev->dev, "override: invalid slot #%u\n",
					val);
			continue;
		}

		slot = &bus->slots[val];

		if (slot->eeprom_override) {
			dev_warn(&pdev->dev, "override: slot #%u is already "
					"overriden\n", val);
			continue;
		}

		slot->eeprom_override = 1;

		/* zero out signature */
		memset(slot->eeprom_signature, 0,
				sizeof(slot->eeprom_signature));

		/* for any matching field assign them */
		for (i = 0; i < ARRAY_SIZE(eeprom_fields); i++) {

			ee_field = &eeprom_fields[i];

			/* point to the entry */
			p = slot->eeprom_signature + ee_field->start;

			/* if no such property, assign default */
			if (of_property_read_string(node, ee_field->name,
						&str) != 0) {

				if (ee_field->override)
					memcpy(p, ee_field->override,
							ee_field->size);
				else
					memset(p, 0, ee_field->size);

				continue;
			}

			/* copy it to the eeprom signature buf */
			len = strlen(str);
			if (len > ee_field->size)
				len = ee_field->size;

			/* copy and zero out rest */
			memcpy(p, str, len);
			if (len < ee_field->size)
				memset(p + len, 0, ee_field->size - len);
		}
	}

	platform_set_drvdata(pdev, bus);

	/* now find the i2c clients */
	for (i = 0; i < bus->slots_nr; i++) {

		slot = &bus->slots[i];

		handle = slot_handles[i];
		node = of_find_node_by_phandle(handle);
		if (node == NULL) {
			dev_warn(&pdev->dev, "Failed to find node with phandle "
					"0x%x (#%d)\n", handle, i);
			continue;
		}
		dev_dbg(&pdev->dev, "Found device node for phandle "
				"0x%x (#%d)\n", handle, i);

		client = of_find_i2c_device_by_node(node);
		if (client == NULL) {
			dev_warn(&pdev->dev, "Invalid I2C client node with "
					"phandle 0x%x (#%d)\n", handle, i);
			continue;
		}

		slot->client = i2c_use_client(client);
		/* no use for this anymore */
		of_node_put(node);

		/* save handle */
		client = slot->client;	/* get again */
		slot->eeprom_addr = client->addr;
		dev_dbg(&pdev->dev, "Found i2c_client at #%d "
				"(address = 0x%02x)\n",
				i, slot->eeprom_addr);

		r = cape_bus_register_slot(&bus->cape_bus, &slot->cape_slot, i);
		if (r != 0) {
			dev_err(&pdev->dev, "Failed to register slot #%d\n", i);
			continue;
		}

		dev_info(&pdev->dev, "Registered slot #%d OK\n", i);
	}

	/* we don't need the handles anymore */
	devm_kfree(&pdev->dev, slot_handles);
	slot_handles = NULL;

	r = bone_capebus_register_pdev_adapters(bus);
	if (r != 0) {
		dev_err(&pdev->dev, "Failed to register the pdev adapters\n");
		goto err_no_pdevs;
	}

	pm_runtime_enable(bus->dev);
	r = pm_runtime_get_sync(bus->dev);
	if (IS_ERR_VALUE(r)) {
		dev_err(&pdev->dev, "Failed to pm_runtime_get_sync()\n");
		goto err_exit;
	}

	pm_runtime_put(bus->dev);

	bone_capebus_bus_sysfs_register(bus);

	dev_info(&pdev->dev, "initialized OK.\n");

	return 0;

err_exit:
	bone_capebus_unregister_pdev_adapters(bus);
err_no_pdevs:
	platform_set_drvdata(pdev, NULL);

	return r;
}

static int __devexit bone_capebus_remove(struct platform_device *pdev)
{
	struct bone_capebus_bus	*bus = platform_get_drvdata(pdev);
	int ret;

	bone_capebus_bus_sysfs_unregister(bus);
	bone_capebus_unregister_pdev_adapters(bus);

	platform_set_drvdata(pdev, NULL);

	ret = pm_runtime_get_sync(&pdev->dev);
	if (IS_ERR_VALUE(ret))
		return ret;

	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM
#ifdef CONFIG_PM_RUNTIME
static int bone_capebus_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bone_capebus_bus *_dev = platform_get_drvdata(pdev);

	(void)_dev;
	return 0;
}

static int bone_capebus_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bone_capebus_bus *_dev = platform_get_drvdata(pdev);

	(void)_dev;
	return 0;
}
#endif /* CONFIG_PM_RUNTIME */

static struct dev_pm_ops bone_capebus_pm_ops = {
	SET_RUNTIME_PM_OPS(bone_capebus_runtime_suspend,
			   bone_capebus_runtime_resume, NULL)
};
#define BONE_CAPEBUS_PM_OPS (&bone_capebus_pm_ops)
#else
#define BONE_CAPEBUS_PM_OPS NULL
#endif /* CONFIG_PM */

static struct platform_driver bone_capebus_driver = {
	.probe		= bone_capebus_probe,
	.remove		= __devexit_p(bone_capebus_remove),
	.driver		= {
		.name	= "bone-capebus",
		.owner	= THIS_MODULE,
		.pm	= BONE_CAPEBUS_PM_OPS,
		.of_match_table = of_match_ptr(bone_capebus_of_match),
	},
};

module_platform_driver(bone_capebus_driver);

MODULE_AUTHOR("Pantelis Antoniou");
MODULE_DESCRIPTION("Beaglebone cape bus controller");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:capebus_bone");
