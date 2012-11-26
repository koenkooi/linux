/*
 * drivers/capebus/capes/bone-spi-vfd.h -- VFD driver for Nixie Cape
 *
 * Copyright (C) 2012, Matt Ranostay
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

/* Placeholder values until we build up an index
 *
 */

#include <linux/spi/spi.h>
#include <linux/workqueue.h>

/*
 * Not really doing anything with this currently
 */

enum {
	MAX6921_DEVICE,
	MAX6931_DEVICE,
	GENERIC_DEVICE,
};

#define SEGMENT_COUNT	8 

struct bonespivfd_info {
	struct spi_device *spi;
	struct delayed_work vfd_update;
	int refresh_rate;

	struct shift_register_info *shift;
	u32 *buf;

	/* digits cache */
	u32 *digits_cache;
	u32 *digits_mask;
	int max_digits;

	/* segments cache */
	u32 *segments_cache;
	int max_segments;
};



/*
 * We always assume SEG_H is the DP digit
 */

enum vfd_segments {
	SEG_A = 1<<0,
	SEG_B = 1<<1,
	SEG_C = 1<<2,
	SEG_D = 1<<3,
	SEG_E = 1<<4,
	SEG_F = 1<<5,
	SEG_G = 1<<6,
	SEG_H = 1<<7,
};

/*
 * Segments 
 */

static const uint16_t nixie_segment_values[] = {
    	SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,		/* 0 */
	SEG_B | SEG_C,						/* 1 */
	SEG_A | SEG_B | SEG_D | SEG_E | SEG_G,			/* 2 */
	SEG_A | SEG_B | SEG_C | SEG_D | SEG_G,			/* 3 */
	SEG_B | SEG_C | SEG_F | SEG_G,				/* 4 */
	SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,			/* 5 */
	SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,		/* 6 */
	SEG_A | SEG_B | SEG_C,					/* 7 */
	SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,	/* 8 */
	SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G,		/* 9 */
	SEG_G,							/* (hypen) */
	SEG_H,							/* (period) */
	0,							/* (space) */
};

/*
 * Characters
 */
static const char nixie_value_array[] = "0123456789-. ";
