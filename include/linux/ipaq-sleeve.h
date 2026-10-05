/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * iPAQ H3xxx/h5xxx expansion-pack ("sleeve") bus
 *
 * One physical expansion connector, one SPI-readable ID EEPROM protocol and
 * one vendor/device ID namespace, shared across the Compaq/HP iPAQ H3xxx
 * and h5xxx families. Based on the Compaq driver of the 2.4/2.6 handheld
 * kernels (Copyright 2000 Compaq Computer Corporation, Author: Jamey
 * Hicks); see drivers/misc/ipaq-sleeve.c for what changed.
 *
 * ID EEPROM protocol: a 2-byte vendor ID is read at EEPROM offset 6 and a
 * 2-byte device ID at offset 8, through the backend's ops->spi_read().
 *
 * TALON_VENDOR (0x0100) is Navman/Talon Technology's GPS sleeve vendor ID,
 * which matches the Talon Technology name carried by the USB Navman entry
 * in drivers/usb/serial/navman.c, for an unrelated product.
 */
#ifndef _LINUX_IPAQ_SLEEVE_H
#define _LINUX_IPAQ_SLEEVE_H

#include <linux/container_of.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/notifier.h>
#include <linux/pm.h>

enum sleeve_vendor_id {
	SLEEVE_NO_VENDOR	= 0,
	COMPAQ_VENDOR_ID	= 0x1125,
	TALON_VENDOR		= 0x0100,
	BIOCENTRIC_SOLUTIONS	= 0x0101,
	HANVIT_INFO_TECH	= 0x0102,
	SYMBOL_VENDOR		= 0x0103,
	OMRON_CORPORATION	= 0x0104,
	HAGIWARA_SYSCOM_CO	= 0x0105,
	TACTEL_SLEEVE_VENDOR	= 0x0107,
	NEXIAN_VENDOR		= 0x010a,
	PITECH_VENDOR		= 0x0111,
	LIFEVIEW_VENDOR		= 0x0601,
};

/*
 * Not folded into either enum above: the original defined this as a
 * third enumerator of enum sleeve_vendor_id (SLEEVE_ANY_ID = -1), but
 * it is used as a wildcard for *both* vendor and device fields of
 * struct ipaq_sleeve_device_id (see ipaq_sleeve_match_device() below),
 * so it belongs to neither ID namespace on its own.
 */
#define SLEEVE_ANY_ID		((unsigned int)-1)

enum sleeve_device_id {
	SLEEVE_NO_DEVICE		= 0,

	/* COMPAQ_VENDOR_ID sleeve devices */
	SINGLE_COMPACTFLASH_SLEEVE	= 0xCF11,
	SINGLE_PCMCIA_SLEEVE		= 0xD7C3,
	DUAL_PCMCIA_SLEEVE		= 0x0001,
	EXTENDED_BATTERY_SLEEVE	= 0x0002,
	SINGLE_CF_PLUS_SLEEVE		= 0x0003,
	SINGLE_PCMCIA_PLUS_SLEEVE	= 0x0004,
	GPRS_EXPANSION_PACK		= 0x0010,
	BLUETOOTH_EXPANSION_PACK	= 0x0011,
	MERCURY_BACKPAQ		= 0x0100,

	/* TALON_VENDOR sleeve devices -- the Navman GPS/CF sleeve */
	NAVMAN_GPS_SLEEVE		= 0x0001,
	NAVMAN_GPS_SLEEVE_ALT		= 0x0010,

	/* NEXIAN_VENDOR sleeve devices */
	NEXIAN_DUAL_CF_SLEEVE		= 0x0001,
	NEXIAN_CAMERA_CF_SLEEVE	= 0x0003,

	/* SYMBOL_VENDOR sleeve devices */
	SYMBOL_WLAN_SCANNER_SLEEVE	= 0x0011,

	/* LIFEVIEW_VENDOR sleeve devices */
	LIFEVIEW_FLYJACKET_SLEEVE	= 0x8a10,

	/* PITECH_VENDOR sleeve devices */
	MEMPLUG_SLEEVE			= 0x0001,
	MEMPLUG_SLEEVE3			= 0x0003,

	/* TACTEL_SLEEVE_VENDOR sleeve devices */
	TACTEL_BLUETOOTH_CF_SLEEVE	= 0xbcbc,
};

#define IPAQ_SLEEVE_ID(vendor, device)	((vendor) << 16 | (device))

/* SLEEVE_HAS_* feature flags for struct ipaq_sleeve_driver.features */
#define SLEEVE_HAS_CF_MASK		0x00000003	/* 0 to 3 */
#define SLEEVE_HAS_CF(x)		(x)
#define SLEEVE_GET_CF(x)		((x) & SLEEVE_HAS_CF_MASK)
#define SLEEVE_HAS_PCMCIA_MASK		0x0000000c
#define SLEEVE_HAS_PCMCIA(x)		((x) << 2)
#define SLEEVE_GET_PCMCIA(x)		(((x) & SLEEVE_HAS_PCMCIA_MASK) >> 2)

#define SLEEVE_HAS_BLUETOOTH		0x00000010
#define SLEEVE_HAS_GPS			0x00000020
#define SLEEVE_HAS_GPRS		0x00000040
#define SLEEVE_HAS_CAMERA		0x00000080
#define SLEEVE_HAS_ACCELEROMETER	0x00000100
#define SLEEVE_HAS_FLASH		0x00000200
#define SLEEVE_HAS_BARCODE_READER	0x00000400
#define SLEEVE_HAS_802_11B		0x00000800
#define SLEEVE_HAS_EBAT		0x00001000
#define SLEEVE_HAS_VGA_OUT		0x00002000
#define SLEEVE_HAS_VIDEO_OUT		0x00004000
#define SLEEVE_HAS_VIDEO_IN		0x00008000
#define SLEEVE_HAS_IRDA		0x00010000
#define SLEEVE_HAS_AUDIO		0x00020000

#define SLEEVE_HAS_FIXED_BATTERY	0x10000000
#define SLEEVE_HAS_REMOVABLE_BATTERY	0x20000000

/**
 * struct ipaq_sleeve_device - a sleeve enumerated on the bus
 * @dev: driver-core device
 * @vendor: vendor ID copied from the sleeve's ID EEPROM
 * @device: device ID copied from the sleeve's ID EEPROM
 * @id_valid: the EEPROM reads completed and returned non-zero IDs
 *
 * The original also carried a @procent (/proc/bus/sleeve entry) --
 * dropped here, procfs is not part of this port (see
 * drivers/misc/ipaq-sleeve.c).
 */
struct ipaq_sleeve_device {
	struct device dev;
	unsigned short vendor;
	unsigned short device;
	bool id_valid;
};

#define to_ipaq_sleeve_device(_dev) \
	container_of_const(_dev, struct ipaq_sleeve_device, dev)

/**
 * struct ipaq_sleeve_device_id - one id_table entry
 * @vendor: vendor ID, or SLEEVE_ANY_ID to match every vendor
 * @device: device ID, or SLEEVE_ANY_ID to match every device
 *
 * id_table arrays are terminated by a {0,} sentinel (vendor == 0), the
 * same convention used by every other bus_type's id_table in this
 * tree (spi_device_id, i2c_device_id, ...).
 */
struct ipaq_sleeve_device_id {
	unsigned int vendor, device;
};

/**
 * struct ipaq_sleeve_driver - driver for one class of sleeve
 * @driver: driver-core driver
 * @id_table: vendor/device IDs this driver binds to; NULL matches
 *	every sleeve
 * @features: SLEEVE_HAS_* flags describing the sleeve's hardware.
 *	Informational/self-description only -- the bus core does NOT
 *	consult this field. Slot counts for consumers such as the CF
 *	socket come from ipaq_sleeve_features() instead (one authority,
 *	no duplication), so a jacket with no bound ipaq_sleeve_driver at
 *	all still gets its CF/PC Card socket instantiated.
 * @probe: called with the bus already matched to a live sleeve device
 * @remove: called when the sleeve is unplugged or the driver unbinds
 * @suspend: called before suspending the host
 * @resume: called while resuming the host
 */
struct ipaq_sleeve_driver {
	struct device_driver driver;
	const struct ipaq_sleeve_device_id *id_table;
	unsigned int features;

	int (*probe)(struct ipaq_sleeve_device *dev);
	void (*remove)(struct ipaq_sleeve_device *dev);
	int (*suspend)(struct ipaq_sleeve_device *dev, pm_message_t state);
	int (*resume)(struct ipaq_sleeve_device *dev);
};

#define to_ipaq_sleeve_driver(_drv) \
	container_of_const(_drv, struct ipaq_sleeve_driver, driver)

extern const struct bus_type ipaq_sleeve_bus_type;

int ipaq_sleeve_driver_register(struct ipaq_sleeve_driver *drv);
void ipaq_sleeve_driver_unregister(struct ipaq_sleeve_driver *drv);

/*
 * Slot-consumer events. @data passed to the notifier callback is the
 * struct ipaq_sleeve_device * that was just enumerated or is about to be
 * torn down. ADD is notified after the sleeve's device_register() and
 * after its jacket power is up, REMOVE before device_unregister().
 * @id_valid distinguishes an identified sleeve from a physically present
 * jacket whose EEPROM could not be read.
 *
 * The driver core binds at most one driver per device, but the CF or PC
 * Card slots of a sleeve have to exist whether or not any
 * ipaq_sleeve_driver binds to it, so slot consumers subscribe here instead
 * of competing for the device. ipaq_sleeve_register_notifier() replays the
 * current sleeve, if any, as an ADD event on registration, which makes the
 * order of a consumer's initcall against the backend's work irrelevant.
 */
#define IPAQ_SLEEVE_EVENT_ADD		1
#define IPAQ_SLEEVE_EVENT_REMOVE	2

int ipaq_sleeve_register_notifier(struct notifier_block *nb);
int ipaq_sleeve_unregister_notifier(struct notifier_block *nb);
unsigned int ipaq_sleeve_features(const struct ipaq_sleeve_device *sdev);

/*
 * Abstract sleeve power, reset and status lines, addressed by the bus core
 * and by slot consumers through a registered backend's
 * ops->control_egpio()/read_egpio()/egpio_irq().
 *
 * The first four are driven by the bus core itself: the NVRAM power and the
 * main power on insert and eject, and all four when a backend registers and
 * unregisters.
 *
 * The last four are read-only and used by consumers: the two card-detect
 * and two card-interrupt/ready lines of the PCMCIA or CompactFlash
 * sockets of a jacket. A socket driver reads their level with
 * read_egpio() and gets the matching Linux interrupt number from
 * egpio_irq(). On the h5xxx these lines are bits of a SAMCOP register
 * rather than board GPIOs and no gpio_chip reaches them, so a socket
 * driver cannot take them as GPIO descriptors and has to go through this
 * vtable.
 *
 * read_egpio() returns the raw register bit as the backend reads it, not a
 * logical value: a non-zero return for CD0_N means the line is high, that
 * is, no card, and the consumer does the inversion. The IRQ0 and IRQ1
 * identifiers carry no _N suffix although the register bits behind them
 * are active low; consumers use them without inversion for the ready
 * state.
 *
 * IPAQ_EGPIO_OPT_INT is the sleeve-device interrupt of the connector, the
 * line a sleeve asserts to get the attention of the host, distinct from the
 * PC Card interrupt of a card in the jacket's socket. It is an interrupt
 * source only and has no control_egpio() or read_egpio() meaning; only
 * egpio_irq() resolves it. On the h39xx it is a bare SoC pin, separate from
 * the PC Card interrupt pin. The SAMCOP of the h5xxx has no separate option
 * interrupt source, so there it is the same line as the card interrupt and a
 * consumer has to be prepared to share it.
 *
 * The numeric values are opaque line identifiers private to this bus; each
 * backend's ops implementation maps them onto its own registers or GPIOs.
 * A backend may switch() on them, so new identifiers must be appended to
 * avoid rewiring hardware.
 */
enum ipaq_sleeve_egpio {
	IPAQ_EGPIO_OPT_ON,
	IPAQ_EGPIO_OPT_NVRAM_ON,
	IPAQ_EGPIO_OPT_RESET,
	IPAQ_EGPIO_CARD_RESET,
	IPAQ_EGPIO_PCMCIA_CD0_N,
	IPAQ_EGPIO_PCMCIA_CD1_N,
	IPAQ_EGPIO_PCMCIA_IRQ0,
	IPAQ_EGPIO_PCMCIA_IRQ1,
	IPAQ_EGPIO_OPT_INT,
};

void ipaq_sleeve_write_egpio(int nr, int state);
int ipaq_sleeve_read_egpio(int nr);
int ipaq_sleeve_egpio_irq(int nr);

#define ipaq_sleeve_clear_egpio(n) ipaq_sleeve_write_egpio((n), 0)
#define ipaq_sleeve_set_egpio(n) ipaq_sleeve_write_egpio((n), 1)

struct ipaq_sleeve_version {
	char d[3];
};

/*
 * Functions provided to the generic bus core by the per-machine
 * backend that owns the physical SPI link and EGPIO lines (e.g. h5xxx
 * SAMCOP EPS, h3xxx/h39xx ASIC2). @dev is the opaque backend context
 * passed to ipaq_sleeve_register().
 */
struct ipaq_sleeve_ops {
	int (*spi_read)(void *dev, unsigned short address,
			unsigned char *data, unsigned short len);
	int (*spi_write)(void *dev, unsigned short address,
			 unsigned char *data, unsigned short len);
	int (*get_option_detect)(void *dev, int *result);
	int (*get_version)(void *dev, struct ipaq_sleeve_version *ver);
	int (*set_ebat)(void *dev);
	int (*control_egpio)(void *dev, int egpio, int state);
	int (*read_egpio)(void *dev, int egpio);
	int (*egpio_irq)(void *dev, int egpio);
	int (*read_battery)(void *dev, unsigned char *chem,
			    unsigned char *percent, unsigned char *flag);
};

int ipaq_sleeve_register(struct ipaq_sleeve_ops *ops, void *dev);
int ipaq_sleeve_unregister(void);
irqreturn_t ipaq_sleeve_presence_interrupt(int irq, void *dev_id);

#endif /* _LINUX_IPAQ_SLEEVE_H */
