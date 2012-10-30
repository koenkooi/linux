/*
 * Generic cape support
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
#include <linux/kernel.h>
#include <linux/err.h>

#include <linux/capebus/capebus-bone.h>

/* fwd decl. */
extern struct cape_driver bonegeneric_driver;

static const struct of_device_id bonegeneric_of_match[] = {
	{
		.compatible = "bone-generic-cape",
	},	{ },
};
MODULE_DEVICE_TABLE(of, bonegeneric_of_match);

static int bonegeneric_probe(struct cape_dev *dev,
		const struct cape_device_id *id)
{
	struct bone_capebus_generic_info *ginfo;
	int err;

	err = bone_capebus_probe_prolog(dev, id);
	if (err != 0)
		return err;

	ginfo = bone_capebus_probe_generic(dev, id);
	if (IS_ERR_OR_NULL(ginfo))
		return IS_ERR(ginfo) ? PTR_ERR(ginfo) : -ENODEV;
	dev->drv_priv = ginfo;
	return 0;
}

static void bonegeneric_remove(struct cape_dev *dev)
{
	bone_capebus_remove_generic(dev->drv_priv);
}

struct cape_driver bonegeneric_driver = {
	.driver = {
		.name		= "bonegeneric",
		.owner		= THIS_MODULE,
		.of_match_table	= bonegeneric_of_match,
	},
	.probe		= bonegeneric_probe,
	.remove		= bonegeneric_remove,
};

module_capebus_driver(bonegeneric_driver);

MODULE_AUTHOR("Pantelis Antoniou");
MODULE_DESCRIPTION("Beaglebone generic cape");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bone-generic-cape");
