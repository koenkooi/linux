// SPDX-License-Identifier: GPL-2.0-only
/*
 * rsrc_mgr.c -- Resource management routines and/or wrappers
 *
 * The initial developer of the original code is David A. Hinds
 * <dahinds@users.sourceforge.net>.  Portions created by David A. Hinds
 * are Copyright (C) 1999 David A. Hinds.  All Rights Reserved.
 *
 * (C) 1999		David A. Hinds
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <pcmcia/ss.h>
#include <pcmcia/cistpl.h>
#include "cs_internal.h"

int static_init(struct pcmcia_socket *s)
{
	/* the good thing about SS_CAP_STATIC_MAP sockets is
	 * that they don't need a resource database */

	s->resource_setup_done = 1;

	return 0;
}

struct resource *pcmcia_make_resource(resource_size_t start,
					resource_size_t end,
					unsigned long flags, const char *name)
{
	struct resource *res = kzalloc_obj(*res);

	if (res) {
		res->name = name;
		res->start = start;
		res->end = start + end - 1;
		res->flags = flags;
	}
	return res;
}

/*
 * A statically mapped socket decodes its whole I/O space through one
 * fixed mapping, so every range a card asks for is accounted to one
 * window covering that space. Populating s->io[] is what lets
 * pcmcia_enable_device() and the pcmcia_fixup_*() helpers reach the
 * socket's ->set_io_map(); release_io_space() frees the window once
 * InUse drops back to zero. It is never inserted into a resource tree:
 * the socket driver already owns the range, and io_offset is not an
 * ioport_resource address on every static-map controller.
 */
static int static_find_io(struct pcmcia_socket *s, unsigned int attr,
			  unsigned int *base, unsigned int num,
			  unsigned int align, struct resource **parent)
{
	unsigned int end;
	int i;

	if (!s->io_offset)
		return -EINVAL;
	*base = s->io_offset | (*base & 0x0fff);
	*parent = NULL;
	end = *base + num - 1;

	for (i = 0; i < MAX_IO_WIN; i++) {
		struct resource *res = s->io[i].res;

		if (!res || res->start > *base || res->end < *base)
			continue;

		/* release_io_space() only matches a window covering the range */
		if (end > res->end)
			res->end = end;
		s->io[i].InUse += num;
		return 0;
	}

	for (i = 0; i < MAX_IO_WIN; i++) {
		struct resource *res;

		if (s->io[i].res)
			continue;

		res = pcmcia_make_resource(s->io_offset,
					   max(end - s->io_offset + 1, 0x1000U),
					   IORESOURCE_IO | (attr & IORESOURCE_BITS),
					   dev_name(&s->dev));
		if (!res)
			return -ENOMEM;

		s->io[i].res = res;
		s->io[i].InUse = num;
		return 0;
	}

	return -EINVAL;
}

struct pccard_resource_ops pccard_static_ops = {
	.validate_mem = NULL,
	.find_io = static_find_io,
	.find_mem = NULL,
	.init = static_init,
	.exit = NULL,
};
EXPORT_SYMBOL(pccard_static_ops);


MODULE_AUTHOR("David A. Hinds, Dominik Brodowski");
MODULE_DESCRIPTION("PCMCIA resource management routines");
MODULE_LICENSE("GPL");
MODULE_ALIAS("rsrc_nonstatic");
