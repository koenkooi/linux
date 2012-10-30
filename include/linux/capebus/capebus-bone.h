/*
 * capebus-bone.h
 *
 * Cape bus defines and function prototypes for the beaglebone
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
#ifndef LINUX_CAPEBUS_BONE_H
#define LINUX_CAPEBUS_BONE_H

#include <linux/list.h>
#include <linux/capebus.h>

struct bone_capebus_slot {
	struct cape_slot	cape_slot;
	u32			slot_handle;
	int			eeprom_addr;
	struct i2c_client	*client;
	unsigned int		eeprom_probed : 1;
	unsigned int		eeprom_failed : 1;
	unsigned int		eeprom_override : 1;
	struct cape_device_id	id;
	char			text_id[256];
	char			eeprom_signature[256];
};

#define to_bone_capebus_slot(n)	\
	container_of(n, struct bone_capebus_slot, cape_slot)

struct bone_capebus_bus {
	struct cape_bus			cape_bus;
	struct device			*dev;		/* pdev->dev */
	int 				slots_nr;
	struct bone_capebus_slot	*slots;
};

#define to_bone_capebus_bus(n)	\
	container_of(n, struct bone_capebus_bus, cape_bus)

#define BONE_CAPEBUS_HEADER		0
#define BONE_CAPEBUS_EEPROM_REV		1
#define BONE_CAPEBUS_BOARD_NAME		2
#define BONE_CAPEBUS_VERSION		3
#define BONE_CAPEBUS_MANUFACTURER	4
#define BONE_CAPEBUS_PART_NUMBER	5
#define BONE_CAPEBUS_NUMBER_OF_PINS	6
#define BONE_CAPEBUS_SERIAL_NUMBER	7
#define BONE_CAPEBUS_PIN_USAGE		8
#define BONE_CAPEBUS_VDD_3V3EXP		9
#define BONE_CAPEBUS_VDD_5V		10
#define BONE_CAPEBUS_SYS_5V		11
#define BONE_CAPEBUS_DC_SUPPLIED	12
#define BONE_CAPEBUS_FIELDS_NR		13

#define BONE_CAPEBUS_MAKE_HEADER(p)	\
	({ \
		const u8 *_p = (p); \
		(((u32)_p[0] << 24) | ((u32)_p[1] << 16) | \
		((u32)_p[2] <<  8) |  (u32)_p[3]       ); \
	})

#define BONE_CAPEBUS_HEADER_VALID	0xaa5533ee

char *bone_capebus_id_get_field(const struct cape_device_id *id,
		int field, char *buf, int bufsz);

int bone_capebus_match_cntrlboard(const struct cape_device_id *id);

int bone_capebus_match_board(const struct cape_device_id *id,
		const char **board_names);

/* in pdevs */
int bone_capebus_register_pdev_adapters(struct bone_capebus_bus *bus);
void bone_capebus_unregister_pdev_adapters(struct bone_capebus_bus *bus);

/* generic cape support */

struct bone_capebus_generic_device_data {
	const char *name;
	const struct of_device_id *of_match;
	unsigned int units;
};

struct bone_capebus_generic_device_entry {
	struct list_head node;
	const struct bone_capebus_generic_device_data *data;
	struct platform_device *pdev;
};

struct bone_capebus_generic_info {
	struct cape_dev *dev;
	struct list_head pdev_list;
};

int bone_capebus_probe_prolog(struct cape_dev *dev,
		const struct cape_device_id *id);

struct bone_capebus_generic_info *
bone_capebus_probe_generic(struct cape_dev *dev,
		const struct cape_device_id *id);

void bone_capebus_remove_generic(
		struct bone_capebus_generic_info *info);

#endif
